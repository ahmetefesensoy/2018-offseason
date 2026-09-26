import math
import unittest

from frc_alliance_playbook.reservations import (
    ConfidenceTracker,
    ReservationSample,
    ReservationTube,
    build_reservations,
    find_conflicts,
    sample_reservation,
)
from frc_alliance_playbook.schema import (
    AlliancePlan,
    PlanSegment,
    Pose2d,
    RobotPlan,
    TimedPose,
)


def robot_plan(
    team_number: int,
    *,
    x_start: float,
    x_end: float,
    y: float = 1.0,
    start_us: int = 0,
    end_us: int = 200_000,
    confidence: float = 0.8,
) -> RobotPlan:
    start = Pose2d(x_start, y, 0.0)
    segment = PlanSegment(
        segment_id=f"route-{team_number}",
        task="CROSS_LINE",
        target_id="auto-line",
        earliest_start_us=start_us,
        latest_start_us=start_us,
        expected_duration_us=end_us - start_us,
        corridor_radius_m=0.2,
        fallback_segment_id="",
        path=(
            TimedPose(start_us, start),
            TimedPose(end_us, Pose2d(x_end, y, math.pi / 2.0)),
        ),
    )
    return RobotPlan(
        team_number=team_number,
        robot_label=f"ally-{team_number}",
        start_pose=start,
        footprint_length_m=0.8,
        footprint_width_m=0.6,
        max_velocity_mps=10.0,
        max_acceleration_mps2=100.0,
        initial_confidence=confidence,
        segments=(segment,),
    )


def alliance_plan(*robots: RobotPlan) -> AlliancePlan:
    return AlliancePlan(
        schema_version=1,
        field_version="2018-power-up-v1",
        alliance="blue",
        plan_id="reservation-test",
        robots=tuple(robots),
    )


class ReservationSamplingTest(unittest.TestCase):
    def test_samples_interpolated_path_radius_and_stable_order(self):
        plan = alliance_plan(
            robot_plan(1678, x_start=2.0, x_end=3.0),
            robot_plan(254, x_start=1.0, x_end=2.0),
        )

        tubes = build_reservations(plan, sample_period_us=100_000)

        self.assertEqual((254, 1678), tuple(tube.team_number for tube in tubes))
        first = tubes[0]
        self.assertEqual((0, 100_000, 200_000), tuple(p.time_us for p in first.samples))
        self.assertEqual((1.0, 1.5, 2.0), tuple(p.pose.x for p in first.samples))
        self.assertAlmostEqual(math.pi / 4.0, first.samples[1].pose.heading)
        self.assertTrue(all(math.isclose(0.7, p.radius_m) for p in first.samples))

    def test_detects_contiguous_temporal_conflict_interval(self):
        tubes = build_reservations(
            alliance_plan(
                robot_plan(254, x_start=1.0, x_end=2.0),
                robot_plan(1678, x_start=1.2, x_end=2.2),
            ),
            sample_period_us=100_000,
        )

        conflicts = find_conflicts(tubes)

        self.assertEqual(1, len(conflicts))
        conflict = conflicts[0]
        self.assertEqual((254, 1678), (conflict.first_team_number, conflict.second_team_number))
        self.assertEqual((0, 200_000), (conflict.start_us, conflict.end_us))
        self.assertAlmostEqual(-1.2, conflict.minimum_clearance_m)

    def test_does_not_conflict_when_same_space_is_reserved_at_different_times(self):
        tubes = (
            ReservationTube(
                plan_id="test",
                team_number=254,
                robot_label="first",
                confidence=0.8,
                samples=(
                    ReservationSample(0, Pose2d(1.0, 1.0, 0.0), 0.7),
                    ReservationSample(100_000, Pose2d(2.0, 1.0, 0.0), 0.7),
                ),
            ),
            ReservationTube(
                plan_id="test",
                team_number=1678,
                robot_label="second",
                confidence=0.8,
                samples=(
                    ReservationSample(200_000, Pose2d(1.0, 1.0, 0.0), 0.7),
                    ReservationSample(300_000, Pose2d(2.0, 1.0, 0.0), 0.7),
                ),
            ),
        )

        self.assertEqual((), find_conflicts(tubes))

    def test_sampling_outside_tube_holds_endpoint_pose(self):
        tube = build_reservations(
            alliance_plan(robot_plan(254, x_start=1.0, x_end=2.0)),
            sample_period_us=100_000,
        )[0]

        before = sample_reservation(tube, -100_000)
        after = sample_reservation(tube, 300_000)

        self.assertEqual((-100_000, 1.0), (before.time_us, before.pose.x))
        self.assertEqual((300_000, 2.0), (after.time_us, after.pose.x))


class ConfidenceTrackerTest(unittest.TestCase):
    def tracker(self) -> ConfidenceTracker:
        return ConfidenceTracker(
            {254: 0.8},
            corridor_tolerance_m=0.2,
            deviation_scale_m=1.0,
            missing_half_life_us=1_000_000,
        )

    def test_on_corridor_preserves_but_never_restores_confidence(self):
        tracker = self.tracker()
        planned = Pose2d(1.0, 1.0, 0.0)

        initial = tracker.update(254, planned, Pose2d(1.1, 1.0, 0.0), 0)
        deviated = tracker.update(254, planned, Pose2d(2.0, 1.0, 0.0), 100_000)
        returned = tracker.update(254, planned, planned, 200_000)

        self.assertEqual(0.8, initial)
        self.assertAlmostEqual(0.8 * math.exp(-0.8), deviated)
        self.assertEqual(deviated, returned)

    def test_missing_observations_decay_repeatedly_and_clamp(self):
        tracker = self.tracker()
        planned = Pose2d(1.0, 1.0, 0.0)
        tracker.update(254, planned, planned, 0)

        first = tracker.missing(254, 1_000_000)
        second = tracker.missing(254, 2_000_000)
        far_future = tracker.missing(254, 1_002_000_000)

        self.assertAlmostEqual(0.4, first)
        self.assertAlmostEqual(0.2, second)
        self.assertGreaterEqual(far_future, 0.0)
        self.assertLessEqual(far_future, 0.8)

    def test_equal_inputs_are_deterministic(self):
        planned = Pose2d(1.0, 1.0, 0.0)
        observed = Pose2d(1.65, 1.0, 0.0)
        first = self.tracker()
        second = self.tracker()

        first_values = (
            first.update(254, planned, observed, 10),
            first.missing(254, 500_010),
        )
        second_values = (
            second.update(254, planned, observed, 10),
            second.missing(254, 500_010),
        )

        self.assertEqual(first_values, second_values)


if __name__ == "__main__":
    unittest.main()
