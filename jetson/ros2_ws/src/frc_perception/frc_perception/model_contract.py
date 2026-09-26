"""Fail-closed metadata contract for ONNX and TensorRT model artifacts."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path


REQUIRED_CLASSES = (
    "power_cube", "robot", "switch_plate", "scale_plate", "vault_opening",
)


class ModelContractError(ValueError):
    pass


@dataclass(frozen=True)
class ModelContract:
    backend: str
    artifact_sha256: str
    classes: tuple[str, ...]
    input_width: int
    input_height: int
    confidence_threshold: float


def load_model_contract(metadata_path: Path | str, artifact_path: Path | str) -> ModelContract:
    metadata_path = Path(metadata_path)
    artifact_path = Path(artifact_path)
    try:
        document = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ModelContractError(f"cannot read model metadata: {error}") from error
    if document.get("schema_version") != 1:
        raise ModelContractError("unsupported model metadata schema")
    backend = document.get("backend")
    if backend not in {"onnxruntime", "tensorrt"}:
        raise ModelContractError("backend must be onnxruntime or tensorrt")
    classes = tuple(document.get("classes", ()))
    if classes != REQUIRED_CLASSES:
        raise ModelContractError("model class order does not match the autonomy contract")
    try:
        artifact_hash = _sha256(artifact_path)
    except OSError as error:
        raise ModelContractError(f"cannot read model artifact: {error}") from error
    declared_hash = str(document.get("artifact_sha256", "")).lower()
    if len(declared_hash) != 64 or declared_hash != artifact_hash:
        raise ModelContractError("model artifact SHA-256 mismatch")
    width = document.get("input_width"); height = document.get("input_height")
    threshold = document.get("confidence_threshold")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ModelContractError("model input dimensions must be positive integers")
    if isinstance(threshold, bool) or not isinstance(threshold, (int, float)) or not math.isfinite(threshold) or not 0.0 < threshold <= 1.0:
        raise ModelContractError("confidence threshold must be finite and in (0, 1]")
    return ModelContract(backend, artifact_hash, classes, width, height, float(threshold))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
