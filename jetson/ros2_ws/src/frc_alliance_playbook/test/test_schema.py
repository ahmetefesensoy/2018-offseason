import copy
import json
import unittest

from frc_alliance_playbook.schema import (
    PlanValidationError,
    canonical_document,
    content_sha256,
    parse_plan,
)


def valid_plan_dict():
    return {
        "schema_version": 1,
        "field_version": "2018-power-up-v1",
        "alliance": "blue",
        "plan_id": "qualification-12-blue",
        "robots": [
            {
                "team_number": 254,
                "robot_label": "ally-left",
                "start_pose": {"x": 1.0, "y": 1.0, "heading": 0.0},
                "footprint": {"length_m": 0.90, "width_m": 0.90},
                "max_velocity_mps": 2.0,
                "max_acceleration_mps2": 4.0,
                "initial_confidence": 0.85,
                "segments": [
                    {
                        "segment_id": "cross-line",
                        "task": "CROSS_LINE",
                        "target_id": "auto-line",
                        "earliest_start_us": 0,
                        "latest_start_us": 0,
                        "expected_duration_us": 1_000_000,
                        "corridor_radius_m": 0.20,
                        "fallback_segment_id": "wait-safe",
                        "path": [
                            {"time_us": 0, "x": 1.0, "y": 1.0, "heading": 0.0},
                            {"time_us": 1_000_000, "x": 2.0, "y": 1.0, "heading": 0.0},
                        ],
                    },
                    {
                        "segment_id": "wait-safe",
                        "task": "WAIT",
                        "target_id": "safe-zone",
                        "earliest_start_us": 1_000_000,
                        "latest_start_us": 1_000_000,
                        "expected_duration_us": 1_000_000,
                        "corridor_radius_m": 0.20,
                        "fallback_segment_id": "",
                        "path": [
                            {"time_us": 1_000_000, "x": 2.0, "y": 1.0, "heading": 0.0},
                            {"time_us": 2_000_000, "x": 2.0, "y": 1.0, "heading": 0.0},
                        ],
                    },
                ],
            }
        ],
    }


class AlliancePlanSchemaTest(unittest.TestCase):
    def test_canonical_round_trip_and_hash_are_order_independent(self):
        first = parse_plan(valid_plan_dict())
        reordered = parse_plan(json.loads(json.dumps(valid_plan_dict(), sort_keys=True)))
        modified_data = valid_plan_dict()
        modified_data["plan_id"] = "qualification-13-blue"
        modified = parse_plan(modified_data)

        self.assertEqual(first, reordered)
        self.assertEqual(content_sha256(first), content_sha256(reordered))
        self.assertEqual(64, len(content_sha256(first)))
        self.assertNotEqual(content_sha256(first), content_sha256(modified))
        self.assertEqual(content_sha256(first), canonical_document(first)["content_sha256"])

    def test_declared_hash_must_match_canonical_content(self):
        data = valid_plan_dict()
        data["content_sha256"] = "0" * 64

        with self.assertRaises(PlanValidationError):
            parse_plan(data)

    def test_rejects_invalid_plans_atomically(self):
        cases = []

        unknown_schema = valid_plan_dict()
        unknown_schema["schema_version"] = 2
        cases.append(("schema", unknown_schema))

        unknown_field = valid_plan_dict()
        unknown_field["field_version"] = "2026-rebuilt"
        cases.append(("field", unknown_field))

        unknown_alliance = valid_plan_dict()
        unknown_alliance["alliance"] = "green"
        cases.append(("alliance", unknown_alliance))

        unknown_task = valid_plan_dict()
        unknown_task["robots"][0]["segments"][0]["task"] = "RAM_OPPONENT"
        cases.append(("task", unknown_task))

        invalid_limit = valid_plan_dict()
        invalid_limit["robots"][0]["max_velocity_mps"] = 0.0
        cases.append(("velocity limit", invalid_limit))

        invalid_confidence = valid_plan_dict()
        invalid_confidence["robots"][0]["initial_confidence"] = 1.1
        cases.append(("confidence", invalid_confidence))

        outside_field = valid_plan_dict()
        outside_field["robots"][0]["segments"][0]["path"][1]["x"] = 17.0
        cases.append(("field bounds", outside_field))

        repeated_time = valid_plan_dict()
        repeated_time["robots"][0]["segments"][0]["path"][1]["time_us"] = 0
        cases.append(("path time", repeated_time))

        overspeed = valid_plan_dict()
        overspeed["robots"][0]["segments"][0]["path"][1]["x"] = 4.0
        cases.append(("speed", overspeed))

        overlap = valid_plan_dict()
        overlap["robots"][0]["segments"][1]["path"][0]["time_us"] = 900_000
        cases.append(("segment overlap", overlap))

        missing_fallback = valid_plan_dict()
        missing_fallback["robots"][0]["segments"][0]["fallback_segment_id"] = "missing"
        cases.append(("missing fallback", missing_fallback))

        fallback_cycle = valid_plan_dict()
        fallback_cycle["robots"][0]["segments"][1]["fallback_segment_id"] = "cross-line"
        cases.append(("fallback cycle", fallback_cycle))

        duplicate_team = valid_plan_dict()
        second_robot = copy.deepcopy(duplicate_team["robots"][0])
        second_robot["robot_label"] = "ally-right"
        duplicate_team["robots"].append(second_robot)
        cases.append(("duplicate team", duplicate_team))

        for name, data in cases:
            with self.subTest(name=name):
                with self.assertRaises(PlanValidationError):
                    parse_plan(data)


if __name__ == "__main__":
    unittest.main()
