import unittest

from frc_world_model.model import MultiObjectTracker, Observation, TrackerConfig
from frc_world_model.world_store import WorldStore, WorldStoreError


def observation(
    detection_id: str,
    class_name: str,
    x: float,
    timestamp_us: int,
) -> Observation:
    return Observation(
        detection_id=detection_id,
        class_name=class_name,
        affiliation=0,
        x=x,
        y=0.0,
        yaw=0.0,
        size_x=0.33,
        size_y=0.33,
        confidence=0.9,
        variance_x=0.04,
        variance_y=0.04,
        timestamp_us=timestamp_us,
    )


class WorldStoreTest(unittest.TestCase):
    def test_ingest_commits_one_partitioned_version(self):
        store = WorldStore(
            MultiObjectTracker(TrackerConfig()),
            source_timeout_us=250_000,
        )

        snapshot = store.ingest(
            (
                observation("c", "power_cube", 1.0, 1_000_000),
                observation("r", "robot", 2.0, 1_000_000),
            ),
            source_timestamp_us=1_000_000,
            received_timestamp_us=1_020_000,
        )

        self.assertEqual(1, snapshot.version)
        self.assertEqual(1, len(snapshot.game_objects))
        self.assertEqual(1, len(snapshot.robots))
        self.assertTrue(snapshot.perception_fresh)
        self.assertEqual(20_000, snapshot.newest_source_age_us)

    def test_stale_source_keeps_tracks_but_marks_snapshot_unfresh(self):
        store = WorldStore(
            MultiObjectTracker(TrackerConfig(track_timeout_us=1_000_000)),
            source_timeout_us=250_000,
        )
        store.ingest(
            (observation("r", "robot", 0.0, 1_000_000),),
            source_timestamp_us=1_000_000,
            received_timestamp_us=1_010_000,
        )

        snapshot = store.snapshot(1_300_001)

        self.assertFalse(snapshot.perception_fresh)
        self.assertEqual(300_001, snapshot.newest_source_age_us)
        self.assertEqual(1, len(snapshot.robots))

    def test_rejects_non_monotonic_batch_without_incrementing_version(self):
        store = WorldStore(
            MultiObjectTracker(TrackerConfig()),
            source_timeout_us=250_000,
        )
        accepted = store.ingest(
            (observation("r", "robot", 1.0, 1_000_000),),
            source_timestamp_us=1_000_000,
            received_timestamp_us=1_010_000,
        )

        with self.assertRaises(WorldStoreError):
            store.ingest(
                (observation("old", "robot", 9.0, 900_000),),
                source_timestamp_us=900_000,
                received_timestamp_us=1_020_000,
            )

        current = store.snapshot(1_020_000)
        self.assertEqual(accepted.version, current.version)
        self.assertEqual(1.0, current.robots[0].x)

    def test_rejects_future_source_timestamp(self):
        store = WorldStore(
            MultiObjectTracker(TrackerConfig()),
            source_timeout_us=250_000,
        )

        with self.assertRaises(WorldStoreError):
            store.ingest(
                (observation("r", "robot", 0.0, 1_100_000),),
                source_timestamp_us=1_100_000,
                received_timestamp_us=1_000_000,
            )

        self.assertEqual(0, store.snapshot(1_000_000).version)


if __name__ == "__main__":
    unittest.main()
