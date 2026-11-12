import itertools
import unittest

from frc_strategy.game_data import parse_game_data, resolve_field_sides
from frc_strategy.task_generator import TaskGenerator
from test_task_generator import TARGETS, snapshot


class GameDataTest(unittest.TestCase):
    def test_all_eight_field_layouts_preserve_alliance_relative_order(self):
        for letters in itertools.product("LR", repeat=3):
            result = parse_game_data("".join(letters))
            expected = tuple("LEFT" if c == "L" else "RIGHT" for c in letters)
            self.assertEqual((result.near_switch, result.scale, result.far_switch), expected)

    def test_missing_or_malformed_fms_data_never_uses_manual_scoring_sides(self):
        generator = TaskGenerator(TARGETS)
        for message in ("", "L", "LR", "LRLL", "XYZ", "lrl", "LR ", None):
            self.assertIsNone(parse_game_data(message))
            switch, scale = resolve_field_sides(message, "fms", "LEFT", "LEFT")
            tasks = generator.generate(snapshot(has_cube=True, our_switch_side=switch, scale_side=scale))
            self.assertFalse(any(t.task_type.startswith("SCORE") for t in tasks))
            self.assertTrue(any(t.task_type == "CROSS_LINE" for t in tasks))

    def test_valid_message_changes_the_scoring_target(self):
        switch, scale = resolve_field_sides("RLL", "fms", "LEFT", "RIGHT")
        tasks = TaskGenerator(TARGETS).generate(snapshot(has_cube=True, our_switch_side=switch, scale_side=scale))
        self.assertIn("switch_right", [task.target_id for task in tasks])
        self.assertNotIn("switch_left", [task.target_id for task in tasks])

    def test_manual_mode_requires_explicit_selection(self):
        self.assertEqual(resolve_field_sides("", "parameters", "RIGHT", "LEFT"), ("RIGHT", "LEFT"))
        with self.assertRaises(ValueError):
            resolve_field_sides("LLL", "typo", "LEFT", "LEFT")
