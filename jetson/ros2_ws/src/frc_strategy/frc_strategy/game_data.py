"""POWER UP FMS data in alliance-relative near-switch/scale/far-switch order."""
from dataclasses import dataclass


@dataclass(frozen=True)
class FieldSides:
    near_switch: str
    scale: str
    far_switch: str


def parse_game_data(message: str) -> FieldSides | None:
    if not isinstance(message, str) or len(message) != 3 or any(c not in "LR" for c in message):
        return None
    return FieldSides(*("LEFT" if c == "L" else "RIGHT" for c in message))


def resolve_field_sides(message: str, source: str, manual_switch: str, manual_scale: str) -> tuple[str, str]:
    if source == "fms":
        sides = parse_game_data(message)
        return (sides.near_switch, sides.scale) if sides else ("UNKNOWN", "UNKNOWN")
    if source == "parameters":
        valid = {"LEFT", "RIGHT"}
        return (manual_switch if manual_switch in valid else "UNKNOWN",
                manual_scale if manual_scale in valid else "UNKNOWN")
    raise ValueError("field_side_source must be fms or parameters")
