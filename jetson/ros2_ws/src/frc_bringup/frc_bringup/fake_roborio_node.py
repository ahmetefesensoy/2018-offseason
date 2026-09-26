"""NT4 server and planar robot model for a hardware-free bridge/RViz demo."""

from __future__ import annotations

import math
import time
import uuid

import ntcore
import rclpy
from rclpy.node import Node

from .sim_core import PoseState


NT_ROOT = "/frc/autonomy/v1"


class FakeRoboRioNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_roborio")
        self._nt = ntcore.NetworkTableInstance.create()
        self._nt.startServer(persist_filename="", listen_address="127.0.0.1")
        self._session_id = f"sim-{uuid.uuid4()}"
        self._pose = PoseState(0.0, 0.0, 0.0)
        self._velocity = (0.0, 0.0, 0.0)
        self._active = False
        self._last_commit = -1
        self._accepted_sequence = -1
        self._last_accepted_us = 0
        self._valid_until_us = 0
        self._reject_reason = "NO_FRAME"
        self._last_tick_us = time.monotonic_ns() // 1_000

        self._session = self._sub_string("command/session_id", "")
        self._armed = self._sub_boolean("command/armed", False)
        self._sent_at = self._sub_integer("command/sent_at_us", 0)
        self._valid_until = self._sub_integer("command/valid_until_us", 0)
        self._vx = self._sub_double("command/vx_mps", 0.0)
        self._vy = self._sub_double("command/vy_mps", 0.0)
        self._omega = self._sub_double("command/omega_radps", 0.0)
        self._mechanism_enabled = self._sub_boolean("command/mechanism_enabled", False)
        self._intake_action = self._sub_integer("command/intake_action", 0)
        self._elevator_target = self._sub_double("command/elevator_target_m", 0.0)
        self._sequence = self._sub_integer("command/sequence", 0)
        self._commit = self._sub_integer("command/commit_sequence", 0)

        self._status_session = self._pub_string("status/session_id")
        self._status_mode = self._pub_string("status/mode")
        self._status_reject = self._pub_string("status/reject_reason")
        self._status_accepted = self._pub_integer("status/accepted_sequence")
        self._status_time = self._pub_integer("status/roborio_time_us")
        self._status_age = self._pub_integer("status/command_age_us")
        self._status_active = self._pub_boolean("status/command_active")
        self._status_gyro = self._pub_boolean("status/gyro_healthy")
        self._status_drive = self._pub_boolean("status/drivetrain_healthy")
        self._status_pose = self._pub_double_array("status/pose")
        self._status_measured = self._pub_double_array("status/measured_chassis")
        self.create_timer(0.02, self._tick)
        self.get_logger().warning("Fake roboRIO is simulation-only and listens on localhost:5810")

    @staticmethod
    def _path(suffix: str) -> str:
        return f"{NT_ROOT}/{suffix}"

    def _sub_string(self, name, default):
        return self._nt.getStringTopic(self._path(name)).subscribe(default)

    def _sub_boolean(self, name, default):
        return self._nt.getBooleanTopic(self._path(name)).subscribe(default)

    def _sub_integer(self, name, default):
        return self._nt.getIntegerTopic(self._path(name)).subscribe(default)

    def _sub_double(self, name, default):
        return self._nt.getDoubleTopic(self._path(name)).subscribe(default)

    def _pub_string(self, name):
        return self._nt.getStringTopic(self._path(name)).publish()

    def _pub_boolean(self, name):
        return self._nt.getBooleanTopic(self._path(name)).publish()

    def _pub_integer(self, name):
        return self._nt.getIntegerTopic(self._path(name)).publish()

    def _pub_double_array(self, name):
        return self._nt.getDoubleArrayTopic(self._path(name)).publish()

    def _tick(self) -> None:
        now_us = time.monotonic_ns() // 1_000
        dt_seconds = min(max((now_us - self._last_tick_us) / 1e6, 0.0), 0.1)
        self._last_tick_us = now_us
        self._read_new_frame(now_us)

        if self._active and (
            now_us - self._last_accepted_us >= 120_000 or now_us > self._valid_until_us
        ):
            self._active = False
            self._velocity = (0.0, 0.0, 0.0)
            self._reject_reason = "WATCHDOG_EXPIRED"

        self._pose = self._pose.integrate(*self._velocity, dt_seconds)
        self._publish_status(now_us)

    def _read_new_frame(self, now_us: int) -> None:
        commit = self._commit.get()
        if commit <= 0 or commit == self._last_commit:
            return
        self._last_commit = commit
        sequence = self._sequence.get()
        sent_at = self._sent_at.get()
        valid_until = self._valid_until.get()
        velocity = (self._vx.get(), self._vy.get(), self._omega.get())

        if sequence != commit:
            self._reject("NOT_COMMITTED")
        elif self._session.get() != self._session_id:
            self._reject("SESSION_MISMATCH")
        elif sequence <= self._accepted_sequence:
            self._reject("NON_MONOTONIC_SEQUENCE")
        elif not self._armed.get():
            self._reject("DISARMED")
        elif not all(math.isfinite(value) for value in velocity):
            self._reject("NON_FINITE_COMMAND")
        elif self._intake_action.get() not in {0, 1, 2, 3}:
            self._reject("INVALID_MECHANISM_COMMAND")
        elif not math.isfinite(self._elevator_target.get()) or not 0.0 <= self._elevator_target.get() <= 1.52:
            self._reject("INVALID_MECHANISM_COMMAND")
        elif math.hypot(velocity[0], velocity[1]) > 0.75 or abs(velocity[2]) > 1.5:
            self._reject("SPEED_LIMIT")
        elif now_us - sent_at > 100_000 or now_us > valid_until:
            self._reject("STALE_COMMAND")
        else:
            self._velocity = velocity
            self._active = True
            self._accepted_sequence = sequence
            self._last_accepted_us = now_us
            self._valid_until_us = valid_until
            self._reject_reason = "NONE"

    def _reject(self, reason: str) -> None:
        self._active = False
        self._velocity = (0.0, 0.0, 0.0)
        self._reject_reason = reason

    def _publish_status(self, now_us: int) -> None:
        command_age = now_us - self._sent_at.get() if self._sent_at.get() > 0 else -1
        self._status_session.set(self._session_id)
        self._status_mode.set("AUTONOMOUS_SIM")
        self._status_reject.set(self._reject_reason)
        self._status_accepted.set(self._accepted_sequence)
        self._status_time.set(now_us)
        self._status_age.set(command_age)
        self._status_active.set(self._active)
        self._status_gyro.set(True)
        self._status_drive.set(True)
        self._status_pose.set([self._pose.x, self._pose.y, self._pose.yaw])
        self._status_measured.set(list(self._velocity))
        self._nt.flush()

    def destroy_node(self) -> bool:
        self._nt.stopServer()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeRoboRioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
