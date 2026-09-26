import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from frc_perception.model_contract import ModelContractError, load_model_contract
from frc_perception.projection import BoundingBox, project_detection


class PerceptionCoreTest(unittest.TestCase):
    def test_model_contract_requires_matching_artifact_hash_and_classes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "cube.onnx"
            artifact.write_bytes(b"deterministic-model")
            metadata = root / "model.json"
            document = {
                "schema_version": 1,
                "backend": "onnxruntime",
                "artifact_sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                "classes": ["power_cube", "robot", "switch_plate", "scale_plate", "vault_opening"],
                "input_width": 640,
                "input_height": 640,
                "confidence_threshold": 0.45,
            }
            metadata.write_text(json.dumps(document), encoding="utf-8")

            contract = load_model_contract(metadata, artifact)
            self.assertEqual(contract.backend, "onnxruntime")

            document["artifact_sha256"] = "0" * 64
            metadata.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaises(ModelContractError):
                load_model_contract(metadata, artifact)

    def test_depth_projection_uses_median_valid_depth(self):
        depth = [[0 for _ in range(8)] for _ in range(8)]
        depth[3][3] = 2000; depth[3][4] = 2100; depth[4][3] = 1900

        point = project_detection(
            BoundingBox(2, 2, 6, 6), depth,
            fx=400.0, fy=400.0, cx=4.0, cy=4.0, depth_scale=0.001,
        )

        self.assertAlmostEqual(point.z, 2.0)
        self.assertAlmostEqual(point.x, 0.0)
        self.assertAlmostEqual(point.y, 0.0)
        self.assertGreater(point.variance, 0.0)


if __name__ == "__main__":
    unittest.main()
