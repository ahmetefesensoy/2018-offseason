import math
import unittest

from frc_world_model.synthetic_scene import sample_scene


class SyntheticSceneTest(unittest.TestCase):
    def test_same_time_produces_identical_scene(self):
        first = sample_scene(1.25, 2_000_000)
        second = sample_scene(1.25, 2_000_000)

        self.assertEqual(first, second)
        self.assertEqual(5, len(first))
        self.assertTrue(all(item.timestamp_us == 2_000_000 for item in first))

    def test_cubes_stay_fixed_while_robots_move(self):
        start = {item.detection_id: item for item in sample_scene(0.0, 1_000_000)}
        later = {item.detection_id: item for item in sample_scene(1.0, 2_000_000)}

        self.assertEqual(start["cube-left"].x, later["cube-left"].x)
        self.assertAlmostEqual(0.45, later["ally-254"].x - start["ally-254"].x)
        self.assertNotEqual(start["opponent-999"].y, later["opponent-999"].y)

    def test_scene_values_are_finite_and_bounded(self):
        scene = sample_scene(3600.0, 3_600_000_000)

        for item in scene:
            self.assertTrue(all(math.isfinite(value) for value in (item.x, item.y, item.yaw)))
            self.assertGreaterEqual(item.confidence, 0.0)
            self.assertLessEqual(item.confidence, 1.0)


if __name__ == "__main__":
    unittest.main()
