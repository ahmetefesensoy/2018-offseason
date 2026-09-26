"""Command-line validation and normalization for teammate strategy files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .importers import (
    ImportMetadata,
    import_csv,
    import_pathplanner_auto,
    import_pathplanner_path,
)
from .schema import canonical_document, parse_plan


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="playbook")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate", help="validate canonical JSON")
    validate.add_argument("source", type=Path)

    csv_parser = subparsers.add_parser("import-csv", help="normalize route CSV")
    csv_parser.add_argument("source", type=Path)
    csv_parser.add_argument("output", type=Path)
    _add_metadata_arguments(csv_parser)

    path_parser = subparsers.add_parser(
        "import-path",
        help="normalize a PathPlanner 2025.0 path",
    )
    path_parser.add_argument("source", type=Path)
    path_parser.add_argument("output", type=Path)
    _add_metadata_arguments(path_parser)

    auto_parser = subparsers.add_parser(
        "import-auto",
        help="normalize a sequential PathPlanner 2025.0 auto",
    )
    auto_parser.add_argument("source", type=Path)
    auto_parser.add_argument("output", type=Path)
    auto_parser.add_argument(
        "--paths",
        type=Path,
        required=True,
        help="directory containing referenced .path files",
    )
    _add_metadata_arguments(auto_parser)
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "validate":
            plan = parse_plan(_load_json(args.source))
            print(
                f"valid plan_id={plan.plan_id} robots={len(plan.robots)} "
                f"sha256={plan.content_sha256}"
            )
            return 0

        metadata = _metadata(args)
        if args.command == "import-csv":
            plan = import_csv(args.source.read_text(encoding="utf-8-sig"), metadata)
        elif args.command == "import-path":
            plan = import_pathplanner_path(_load_json(args.source), metadata)
        else:
            auto_data = _load_json(args.source)

            def load_path(name: str):
                return _load_json(args.paths / f"{name}.path")

            plan = import_pathplanner_auto(auto_data, load_path, metadata)
        _write_plan(args.output, plan)
        print(f"wrote {args.output} sha256={plan.content_sha256}")
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"playbook: error: {error}", file=sys.stderr)
        return 2


def _add_metadata_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--plan-id", required=True)
    parser.add_argument("--alliance", choices=("blue", "red"), required=True)
    parser.add_argument("--team", type=int, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--footprint-length", type=float, default=0.9)
    parser.add_argument("--footprint-width", type=float, default=0.9)
    parser.add_argument("--confidence", type=float, default=0.75)
    parser.add_argument("--corridor-radius", type=float, default=0.2)
    parser.add_argument("--task", default="CROSS_LINE")
    parser.add_argument("--target", default="auto-line")
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-acceleration", type=float, default=4.0)


def _metadata(args) -> ImportMetadata:
    return ImportMetadata(
        plan_id=args.plan_id,
        alliance=args.alliance,
        team_number=args.team,
        robot_label=args.label,
        footprint_length_m=args.footprint_length,
        footprint_width_m=args.footprint_width,
        initial_confidence=args.confidence,
        corridor_radius_m=args.corridor_radius,
        default_task=args.task.upper(),
        default_target_id=args.target,
        max_velocity_mps=args.max_velocity,
        max_acceleration_mps2=args.max_acceleration,
    )


def _load_json(path: Path):
    with path.open("r", encoding="utf-8-sig") as source:
        return json.load(source)


def _write_plan(path: Path, plan) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(canonical_document(plan), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    raise SystemExit(main())
