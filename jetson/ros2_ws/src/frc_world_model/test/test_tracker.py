import math
import unittest
from dataclasses import replace

from frc_world_model.model import (
    MultiObjectTracker,
    Observation,
    ObservationError,
    TrackerConfig,
)


def observation(
    detection_id: str,
    class_name: str,
    x: float,
    y: float,
    timestamp_us: int,
    *,
    affiliation: int = 0,
) -> Observation:
    return Observation(
        detection_id=detection_id,
        class_name=class_name,
        affiliation=affiliation,
        x=x,
        y=y,
        yaw=0.0,
        size_x=0.33,
        size_y=0.33,
        confidence=0.9,
        variance_x=0.04,
        variance_y=0.04,
        timestamp_us=timestamp_us,
    )


class MultiObjectTrackerTest(unittest.TestCase):
    def test_rejects_unsafe_tracker_configuration(self):
        unsafe_configs = (
            TrackerConfig(association_gate_m=0.0),
            TrackerConfig(position_gain=1.1),
            TrackerConfig(velocity_gain=-0.1),
            TrackerConfig(track_timeout_us=0),
            TrackerConfig(prediction_step_s=0.0),
            TrackerConfig(prediction_horizon_s=-1.0),
            TrackerConfig(process_noise_variance=-0.1),
        )
        for config in unsafe_configs:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    MultiObjectTracker(config)

    def test_rejects_invalid_batch_without_mutating_tracks(self):
        tracker = MultiObjectTracker(TrackerConfig())
        tracker.update(
            (observation("cube-a", "power_cube", 1.0, 2.0, 1_000_000),),
            1_000_000,
        )
        before = tracker.snapshot(1_000_000)

        with self.assertRaises(ObservationError):
            tracker.update(
                (observation("bad", "robot", math.nan, 0.0, 1_020_000),),
                1_020_000,
            )

        self.assertEqual(before, tracker.snapshot(1_000_000))

    def test_rejects_zero_covariance_as_false_certainty(self):
        tracker = MultiObjectTracker(TrackerConfig())
        invalid = replace(
            observation("cube", "power_cube", 1.0, 2.0, 1_000_000),
            variance_x=0.0,
        )

        with self.assertRaises(ObservationError):
            tracker.update((invalid,), 1_000_000)

        self.assertEqual((), tracker.snapshot(1_000_000))

    def test_creation_order_and_ids_do_not_depend_on_input_order(self):
        left = observation("z", "robot", 2.0, 0.0, 1_000_000)
        right = observation("a", "robot", 4.0, 0.0, 1_000_000)
        tracker = MultiObjectTracker(TrackerConfig())

        tracks = tracker.update((left, right), 1_000_000)

        self.assertEqual(
            ("trk-000001", "trk-000002"),
            tuple(track.track_id for track in tracks),
        )
        self.assertEqual((4.0, 2.0), tuple(track.x for track in tracks))

    def test_associates_nearest_compatible_track_and_estimates_velocity(self):
        tracker = MultiObjectTracker(
            TrackerConfig(position_gain=1.0, velocity_gain=1.0)
        )
        first = tracker.update(
            (observation("r1", "robot", 1.0, 1.0, 1_000_000),),
            1_000_000,
        )[0]

        second = tracker.update(
            (observation("r2", "robot", 1.2, 1.0, 1_200_000),),
            1_200_000,
        )[0]

        self.assertEqual(first.track_id, second.track_id)
        self.assertAlmostEqual(1.0, second.vx)
        self.assertAlmostEqual(0.0, second.vy)

    def test_expires_stale_tracks(self):
        tracker = MultiObjectTracker(TrackerConfig(track_timeout_us=500_000))
        tracker.update(
            (observation("cube", "power_cube", 0.0, 0.0, 1_000_000),),
            1_000_000,
        )

        self.assertEqual((), tracker.snapshot(1_500_001))

    def test_robot_prediction_has_twenty_steps_and_growing_covariance(self):
        tracker = MultiObjectTracker(
            TrackerConfig(position_gain=1.0, velocity_gain=1.0)
        )
        tracker.update(
            (observation("r", "robot", 0.0, 0.0, 1_000_000),),
            1_000_000,
        )

        track = tracker.update(
            (observation("r", "robot", 0.1, 0.0, 1_100_000),),
            1_100_000,
        )[0]

        self.assertEqual(20, len(track.predictions))
        self.assertAlmostEqual(2.1, track.predictions[-1].x)
        self.assertGreater(track.predictions[-1].variance_x, track.variance_x)

    def test_snapshot_extrapolates_track_through_short_occlusion(self):
        tracker = MultiObjectTracker(
            TrackerConfig(position_gain=1.0, velocity_gain=1.0)
        )
        tracker.update(
            (observation("r", "robot", 0.0, 0.0, 1_000_000),),
            1_000_000,
        )
        observed = tracker.update(
            (observation("r", "robot", 0.1, 0.0, 1_100_000),),
            1_100_000,
        )[0]

        extrapolated = tracker.snapshot(1_300_000)[0]

        self.assertAlmostEqual(0.3, extrapolated.x)
        self.assertEqual(observed.last_seen_us, extrapolated.last_seen_us)
        self.assertGreater(extrapolated.variance_x, observed.variance_x)


if __name__ == "__main__":
    unittest.main()
