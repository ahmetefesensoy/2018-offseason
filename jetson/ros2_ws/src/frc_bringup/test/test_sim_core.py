import math
import unittest

from frc_bringup.sim_core import PoseState


class PoseStateTest(unittest.TestCase):
    def test_integrates_robot_relative_velocity_in_world_frame(self):
        pose = PoseState(x=1.0, y=2.0, yaw=math.pi / 2.0)

        updated = pose.integrate(vx_mps=1.0, vy_mps=0.0, omega_radps=0.0, dt_seconds=0.5)

        self.assertAlmostEqual(1.0, updated.x)
        self.assertAlmostEqual(2.5, updated.y)
        self.assertAlmostEqual(math.pi / 2.0, updated.yaw)

    def test_wraps_heading_to_signed_pi(self):
        pose = PoseState(0.0, 0.0, math.pi - 0.1)

        updated = pose.integrate(0.0, 0.0, 1.0, 0.2)

        self.assertAlmostEqual(-math.pi + 0.1, updated.yaw)

    def test_rejects_invalid_or_negative_time_step(self):
        pose = PoseState(0.0, 0.0, 0.0)
        for invalid_dt in (-0.1, math.nan, math.inf):
            with self.subTest(dt=invalid_dt):
                with self.assertRaises(ValueError):
                    pose.integrate(0.0, 0.0, 0.0, invalid_dt)


if __name__ == "__main__":
    unittest.main()
