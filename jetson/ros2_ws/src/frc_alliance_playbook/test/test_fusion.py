import unittest

from frc_alliance_playbook.fusion import MatchClock, ObservedAlly, StableAllyMatcher
from frc_alliance_playbook.reservations import (
    ReservationSample,
    ReservationTube,
)
from frc_alliance_playbook.schema import Pose2d


def tube(team_number: int, x: float) -> ReservationTube:
    return ReservationTube(
        plan_id="test",
        team_number=team_number,
        robot_label=f"ally-{team_number}",
        confidence=0.8,
        samples=(
            ReservationSample(0, Pose2d(x, 1.0, 0.0), 0.7),
            ReservationSample(1_000_000, Pose2d(x, 1.0, 0.0), 0.7),
        ),
    )


class MatchClockTest(unittest.TestCase):
    def test_starts_only_on_autonomous_transition_and_resets_between_matches(self):
        clock = MatchClock()

        self.assertFalse(clock.update("DISABLED", 1_000_000))
        self.assertIsNone(clock.elapsed_us(1_500_000))
        self.assertTrue(clock.update("AUTONOMOUS", 2_000_000))
        self.assertEqual(0, clock.elapsed_us(2_000_000))
        self.assertFalse(clock.update("AUTONOMOUS", 2_250_000))
        self.assertEqual(250_000, clock.elapsed_us(2_250_000))
        self.assertFalse(clock.update("DISABLED", 3_000_000))
        self.assertIsNone(clock.elapsed_us(3_100_000))
        self.assertTrue(clock.update("AUTONOMOUS", 4_000_000))
        self.assertEqual(100_000, clock.elapsed_us(4_100_000))

    def test_ros_time_rewind_starts_a_new_epoch_for_replay(self):
        clock = MatchClock()
        clock.update("AUTONOMOUS_SIM", 5_000_000)
        self.assertEqual(500_000, clock.elapsed_us(5_500_000))

        self.assertTrue(clock.update("AUTONOMOUS_SIM", 1_000_000))
        self.assertEqual(0, clock.elapsed_us(1_000_000))


class StableAllyMatcherTest(unittest.TestCase):
    def test_track_identity_does_not_swap_when_teammates_cross(self):
        matcher = StableAllyMatcher(missing_binding_grace_us=1_000_000)
        tubes = (tube(254, 1.0), tube(1678, 9.0))
        initial = (
            ObservedAlly("track-a", Pose2d(1.1, 1.0, 0.0)),
            ObservedAlly("track-b", Pose2d(8.9, 1.0, 0.0)),
        )
        crossed = (
            ObservedAlly("track-a", Pose2d(8.8, 1.0, 0.0)),
            ObservedAlly("track-b", Pose2d(1.2, 1.0, 0.0)),
        )

        first = matcher.assign(tubes, initial, 0)
        second = matcher.assign(tubes, crossed, 500_000)

        self.assertEqual("track-a", first[254].track_id)
        self.assertEqual("track-b", first[1678].track_id)
        self.assertEqual("track-a", second[254].track_id)
        self.assertEqual("track-b", second[1678].track_id)

    def test_missing_binding_is_released_only_after_grace_period(self):
        matcher = StableAllyMatcher(missing_binding_grace_us=500_000)
        tubes = (tube(254, 1.0),)
        original = ObservedAlly("track-a", Pose2d(1.0, 1.0, 0.0))
        replacement = ObservedAlly("track-b", Pose2d(1.0, 1.0, 0.0))
        matcher.assign(tubes, (original,), 0)

        during_grace = matcher.assign(tubes, (replacement,), 400_000)
        after_grace = matcher.assign(tubes, (replacement,), 600_000)

        self.assertEqual({}, during_grace)
        self.assertEqual("track-b", after_grace[254].track_id)


if __name__ == "__main__":
    unittest.main()
