import json
from pathlib import Path
import unittest

from frc_alliance_playbook.importers import (
    ImportMetadata,
    PlanImportError,
    import_csv,
    import_pathplanner_auto,
    import_pathplanner_path,
)


FIXTURES = Path(__file__).parent / "fixtures"


def metadata() -> ImportMetadata:
    return ImportMetadata(
        plan_id="qualification-12-blue",
        alliance="blue",
        team_number=254,
        robot_label="ally-left",
        footprint_length_m=0.90,
        footprint_width_m=0.90,
        initial_confidence=0.80,
        corridor_radius_m=0.20,
        default_task="CROSS_LINE",
        default_target_id="auto-line",
        max_velocity_mps=2.0,
        max_acceleration_mps2=4.0,
    )


class CsvImporterTest(unittest.TestCase):
    def test_imports_seconds_degrees_velocity_and_event(self):
        text = """time,x,y,heading,vx,vy,omega,event
0.0,1.0,1.0,0,0,0,0,CROSS_LINE:auto-line
0.5,1.5,1.0,45,1,0,0,
1.0,2.0,1.0,90,1,0,0,
"""

        plan = import_csv(text, metadata())
        segment = plan.robots[0].segments[0]

        self.assertEqual("CROSS_LINE", segment.task)
        self.assertEqual("auto-line", segment.target_id)
        self.assertEqual(500_000, segment.path[1].time_us)
        self.assertAlmostEqual(1.5707963267948966, segment.path[-1].pose.heading)

    def test_rejects_non_monotonic_or_velocity_inconsistent_csv(self):
        non_monotonic = """time,x,y,heading,vx,vy,omega,event
0.0,1.0,1.0,0,0,0,0,CROSS_LINE
0.0,2.0,1.0,0,1,0,0,
"""
        inconsistent = """time,x,y,heading,vx,vy,omega,event
0.0,1.0,1.0,0,0,0,0,CROSS_LINE
1.0,2.0,1.0,0,0,0,0,
"""

        for text in (non_monotonic, inconsistent):
            with self.subTest(text=text):
                with self.assertRaises(PlanImportError):
                    import_csv(text, metadata())


class PathPlannerImporterTest(unittest.TestCase):
    def test_samples_current_path_format_with_exact_endpoints(self):
        data = json.loads((FIXTURES / "Pickup.path").read_text(encoding="utf-8"))

        plan = import_pathplanner_path(data, metadata())
        path = plan.robots[0].segments[0].path

        self.assertEqual(21, len(path))
        self.assertEqual((1.0, 1.0), (path[0].pose.x, path[0].pose.y))
        self.assertEqual((3.0, 1.0), (path[-1].pose.x, path[-1].pose.y))
        self.assertGreater(path[-1].time_us, path[0].time_us)

    def test_rejects_unsupported_version_and_missing_anchor(self):
        base = json.loads((FIXTURES / "Pickup.path").read_text(encoding="utf-8"))
        unsupported = dict(base)
        unsupported["version"] = "2024.0"
        missing_anchor = json.loads(json.dumps(base))
        del missing_anchor["waypoints"][1]["anchor"]

        for data in (unsupported, missing_anchor):
            with self.subTest(data=data):
                with self.assertRaises(PlanImportError):
                    import_pathplanner_path(data, metadata())

    def test_imports_sequential_auto_paths_and_waits(self):
        auto = json.loads((FIXTURES / "Example.auto").read_text(encoding="utf-8"))

        def load_path(name):
            return json.loads((FIXTURES / f"{name}.path").read_text(encoding="utf-8"))

        plan = import_pathplanner_auto(auto, load_path, metadata())
        segments = plan.robots[0].segments

        self.assertEqual(("CROSS_LINE", "WAIT", "CROSS_LINE"), tuple(value.task for value in segments))
        self.assertEqual(segments[0].path[-1].time_us, segments[1].path[0].time_us)
        self.assertEqual(1_000_000, segments[1].expected_duration_us)
        self.assertEqual(segments[1].path[-1].time_us, segments[2].path[0].time_us)

    def test_auto_rejects_missing_path_and_parallel_drive_structure(self):
        auto = json.loads((FIXTURES / "Example.auto").read_text(encoding="utf-8"))
        with self.assertRaises(PlanImportError):
            import_pathplanner_auto(
                auto,
                lambda name: (_ for _ in ()).throw(FileNotFoundError(name)),
                metadata(),
            )

        parallel = json.loads(json.dumps(auto))
        parallel["command"]["type"] = "parallel"
        with self.assertRaises(PlanImportError):
            import_pathplanner_auto(parallel, lambda name: {}, metadata())


if __name__ == "__main__":
    unittest.main()
