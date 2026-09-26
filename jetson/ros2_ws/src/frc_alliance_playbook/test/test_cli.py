import json
from pathlib import Path
import tempfile
import unittest

from frc_alliance_playbook.cli import main
from frc_alliance_playbook.schema import parse_plan


PACKAGE_ROOT = Path(__file__).parents[1]
FIXTURES = Path(__file__).parent / "fixtures"


class PlaybookCliTest(unittest.TestCase):
    def test_validates_hashed_canonical_plan(self):
        result = main(
            ["validate", str(PACKAGE_ROOT / "config" / "example-plan.json")]
        )

        self.assertEqual(0, result)

    def test_import_path_writes_a_hashed_canonical_document(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "normalized.json"
            result = main(
                [
                    "import-path",
                    str(FIXTURES / "Pickup.path"),
                    str(output),
                    "--plan-id",
                    "cli-test",
                    "--alliance",
                    "blue",
                    "--team",
                    "254",
                    "--label",
                    "ally-left",
                ]
            )

            document = json.loads(output.read_text(encoding="utf-8"))
            plan = parse_plan(document)
            self.assertEqual(0, result)
            self.assertEqual(document["content_sha256"], plan.content_sha256)


if __name__ == "__main__":
    unittest.main()
