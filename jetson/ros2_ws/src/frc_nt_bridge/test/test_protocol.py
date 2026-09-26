import math
import unittest

from frc_nt_bridge.protocol import (
    CommandRequest,
    FrameBuilder,
    ProtocolError,
    server_time_us,
)


class FrameBuilderTest(unittest.TestCase):
    def setUp(self):
        self.builder = FrameBuilder(
            max_translation_speed_mps=0.75,
            max_rotation_speed_radps=1.5,
            max_validity_us=150_000,
        )

    def test_sequences_are_strictly_increasing(self):
        first = self.builder.build(
            CommandRequest(True, 0.2, -0.1, 0.3, 100_000),
            session_id="boot-a",
            now_server_us=1_000_000,
        )
        second = self.builder.build(
            CommandRequest(True, 0.1, 0.0, 0.0, 80_000),
            session_id="boot-a",
            now_server_us=1_020_000,
        )

        self.assertEqual(1, first.sequence)
        self.assertEqual(2, second.sequence)
        self.assertGreater(second.sequence, first.sequence)

    def test_sequence_resumes_after_jetson_process_restart(self):
        self.builder.synchronize_accepted_sequence(41)

        frame = self.builder.build(
            CommandRequest(True, 0.1, 0.0, 0.0, 100_000),
            session_id="boot-a",
            now_server_us=1_000_000,
        )

        self.assertEqual(42, frame.sequence)

    def test_commit_sequence_is_the_final_publish_operation(self):
        frame = self.builder.build(
            CommandRequest(True, 0.2, 0.0, 0.1, 100_000),
            session_id="boot-a",
            now_server_us=1_000_000,
        )

        operations = frame.publish_operations()

        self.assertEqual("commit_sequence", operations[-1][0])
        self.assertEqual(frame.sequence, operations[-1][1])
        self.assertEqual(frame.sequence, dict(operations)["sequence"])

    def test_disarmed_frame_forces_zero_velocity(self):
        frame = self.builder.build(
            CommandRequest(False, 0.4, -0.3, 1.0, 100_000),
            session_id="boot-a",
            now_server_us=1_000_000,
        )

        self.assertFalse(frame.armed)
        self.assertEqual((0.0, 0.0, 0.0), (frame.vx_mps, frame.vy_mps, frame.omega_radps))

    def test_invalid_values_and_limits_fail_closed(self):
        invalid_requests = (
            CommandRequest(True, math.nan, 0.0, 0.0, 100_000),
            CommandRequest(True, 0.75, 0.01, 0.0, 100_000),
            CommandRequest(True, 0.0, 0.0, 1.51, 100_000),
            CommandRequest(True, 0.0, 0.0, 0.0, 150_001),
            CommandRequest(True, 0.0, 0.0, 0.0, 0),
        )

        for request in invalid_requests:
            with self.subTest(request=request):
                with self.assertRaises(ProtocolError):
                    self.builder.build(request, "boot-a", 1_000_000)

    def test_missing_session_fails_closed(self):
        with self.assertRaises(ProtocolError):
            self.builder.build(
                CommandRequest(True, 0.0, 0.0, 0.0, 100_000),
                session_id="",
                now_server_us=1_000_000,
            )


class ServerTimeTest(unittest.TestCase):
    def test_applies_nt4_server_offset_to_monotonic_clock(self):
        self.assertEqual(1_000_125, server_time_us(1_000_000, 125))

    def test_requires_completed_nt4_time_sync(self):
        with self.assertRaises(ProtocolError):
            server_time_us(1_000_000, None)


if __name__ == "__main__":
    unittest.main()
