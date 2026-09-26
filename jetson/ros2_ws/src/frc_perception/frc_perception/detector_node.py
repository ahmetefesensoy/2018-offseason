"""Jetson YOLO adapter; refuses to start unless model metadata and hash match."""

from __future__ import annotations

import math
from pathlib import Path

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException

from frc_autonomy_msgs.msg import SemanticObservation, SemanticObservationArray

from .model_contract import load_model_contract
from .projection import BoundingBox, project_detection


class DetectorRuntime:
    def __init__(self, artifact: str, confidence: float) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError("ultralytics is required for detector runtime") from error
        self._model = YOLO(artifact, task="detect")
        self._confidence = confidence

    def infer(self, image):
        result = self._model.predict(image, conf=self._confidence, verbose=False)[0]
        if result.boxes is None:
            return ()
        boxes = result.boxes.xyxy.cpu().tolist()
        classes = result.boxes.cls.cpu().tolist()
        confidences = result.boxes.conf.cpu().tolist()
        return tuple((box, int(class_id), float(confidence)) for box, class_id, confidence in zip(boxes, classes, confidences))


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_node")
        for name, default in (
            ("model_path", "/models/power_up.engine"), ("metadata_path", "/models/power_up.json"),
            ("color_topic", "/camera/color/image_raw"), ("depth_topic", "/camera/depth/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"), ("depth_scale", 0.001),
            ("map_frame", "map"), ("sync_slop_s", 0.04),
        ): self.declare_parameter(name, default)
        artifact = Path(str(self.get_parameter("model_path").value))
        contract = load_model_contract(str(self.get_parameter("metadata_path").value), artifact)
        self._classes = contract.classes
        self._runtime = DetectorRuntime(str(artifact), contract.confidence_threshold)
        self._bridge = CvBridge(); self._tf = Buffer(); self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(SemanticObservationArray, "/perception/observations", 10)
        color = Subscriber(self, Image, str(self.get_parameter("color_topic").value))
        depth = Subscriber(self, Image, str(self.get_parameter("depth_topic").value))
        info = Subscriber(self, CameraInfo, str(self.get_parameter("camera_info_topic").value))
        self._sync = ApproximateTimeSynchronizer((color, depth, info), 8, float(self.get_parameter("sync_slop_s").value))
        self._sync.registerCallback(self._on_frame); self._sequence = 0

    def _on_frame(self, color_message, depth_message, camera_info) -> None:
        try:
            transform = self._tf.lookup_transform(
                str(self.get_parameter("map_frame").value), color_message.header.frame_id,
                Time.from_msg(color_message.header.stamp), timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            self.get_logger().warning(f"Detector frame skipped: {error}"); return
        color = self._bridge.imgmsg_to_cv2(color_message, "bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_message, "passthrough")
        observations = []
        for box, class_id, confidence in self._runtime.infer(color):
            if not 0 <= class_id < len(self._classes): continue
            try:
                point = project_detection(
                    BoundingBox(*(round(value) for value in box)), depth,
                    fx=camera_info.k[0], fy=camera_info.k[4], cx=camera_info.k[2], cy=camera_info.k[5],
                    depth_scale=float(self.get_parameter("depth_scale").value),
                )
            except ValueError:
                continue
            x, y, z = _transform_point(point.x, point.y, point.z, transform.transform)
            self._sequence += 1
            observation = SemanticObservation(); observation.detection_id = f"det-{self._sequence}"
            observation.class_name = self._classes[class_id]; observation.affiliation = SemanticObservation.AFFILIATION_UNKNOWN
            observation.pose.pose.position.x = x; observation.pose.pose.position.y = y; observation.pose.pose.position.z = z
            observation.pose.pose.orientation.w = 1.0; observation.pose.covariance[0] = point.variance
            observation.pose.covariance[7] = point.variance; observation.pose.covariance[35] = 0.25
            size = 0.9 if observation.class_name == "robot" else 0.33
            observation.size.x = size; observation.size.y = size; observation.size.z = size
            observation.confidence = confidence; observations.append(observation)
        message = SemanticObservationArray(); message.header.stamp = color_message.header.stamp
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.observations = observations; self._publisher.publish(message)


def _transform_point(x, y, z, transform):
    q = transform.rotation; tx = transform.translation
    uvx = q.y * z - q.z * y; uvy = q.z * x - q.x * z; uvz = q.x * y - q.y * x
    uuvx = q.y * uvz - q.z * uvy; uuvy = q.z * uvx - q.x * uvz; uuvz = q.x * uvy - q.y * uvx
    return (
        x + 2.0 * (q.w * uvx + uuvx) + tx.x,
        y + 2.0 * (q.w * uvy + uuvy) + tx.y,
        z + 2.0 * (q.w * uvz + uuvz) + tx.z,
    )


def main(args=None) -> None:
    rclpy.init(args=args); node = DetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
