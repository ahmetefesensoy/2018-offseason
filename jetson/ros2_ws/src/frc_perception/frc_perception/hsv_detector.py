"""Train-free yellow power-cube baseline. It does not detect robots or field goals."""
import math


def depth_scale_for_encoding(encoding: str, integer_scale: float = 0.001) -> float:
    if encoding == "32FC1":
        return 1.0  # ROS floating-point depth is expressed in metres.
    if encoding == "16UC1" and math.isfinite(integer_scale) and integer_scale > 0:
        return integer_scale
    raise ValueError(f"Unsupported depth encoding/scale: {encoding}, {integer_scale}")


class HsvCubeDetector:
    def __init__(self, min_area_px: float = 200.0):
        if not math.isfinite(min_area_px) or min_area_px <= 0:
            raise ValueError("Minimum cube area must be finite and positive")
        self.min_area_px = min_area_px

    def infer(self, image):
        import cv2
        import numpy as np

        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("Cube baseline expects a uint8 BGR image")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((18, 90, 70), np.uint8), np.array((40, 255, 255), np.uint8))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area < self.min_area_px or not 0.45 <= width / max(height, 1) <= 2.2:
                continue
            # This is mask fill, not a calibrated probability of a real cube.
            fill = min(1.0, area / (width * height))
            if fill >= 0.5:
                detections.append(((x, y, x + width, y + height), 0, fill))
        return tuple(sorted(detections, key=lambda detection: detection[0]))
