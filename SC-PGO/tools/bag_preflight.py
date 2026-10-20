#!/usr/bin/env python3
"""Validate a rosbag2 metadata file against a named rover topic contract."""

import argparse
import json
from pathlib import Path
import sys

import yaml


def metadata_path(bag_path: Path) -> Path:
    if bag_path.is_dir():
        return bag_path / "metadata.yaml"
    if bag_path.name == "metadata.yaml" or bag_path.suffix in {".yaml", ".yml"}:
        return bag_path
    return bag_path.parent / "metadata.yaml"


def read_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected a YAML mapping in {path}")
    return value


def topic_inventory(metadata: dict) -> tuple[dict, float]:
    info = metadata.get("rosbag2_bagfile_information", metadata)
    duration = info.get("duration", 0)
    if isinstance(duration, dict):
        duration = duration.get("nanoseconds", 0)
    duration_seconds = float(duration or 0) / 1e9

    inventory = {}
    for entry in info.get("topics_with_message_count", []):
        topic = entry.get("topic_metadata", {})
        name = topic.get("name")
        if not name:
            continue
        count = int(entry.get("message_count", 0))
        inventory[name] = {
            "type": topic.get("type", ""),
            "message_count": count,
            "average_hz": count / duration_seconds if duration_seconds > 0 else None,
        }
    return inventory, duration_seconds


def evaluate(metadata: dict, contract: dict, profile: str) -> dict:
    profiles = contract.get("profiles", {})
    if profile not in profiles:
        raise ValueError(
            f"unknown profile '{profile}'; available: {', '.join(sorted(profiles))}"
        )
    expected = profiles[profile]
    inventory, duration_seconds = topic_inventory(metadata)
    missing = []
    empty = []
    type_mismatches = []
    present = {}
    for name, expected_type in expected.items():
        actual = inventory.get(name)
        if actual is None:
            missing.append(name)
            continue
        present[name] = actual
        if actual["type"] != expected_type:
            type_mismatches.append(
                {"topic": name, "expected": expected_type, "actual": actual["type"]}
            )
        if actual["message_count"] <= 0:
            empty.append(name)
    return {
        "profile": profile,
        "ok": not missing and not empty and not type_mismatches,
        "duration_seconds": duration_seconds,
        "present": present,
        "missing": missing,
        "empty": empty,
        "type_mismatches": type_mismatches,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory, metadata.yaml, db3, or mcap")
    parser.add_argument("--profile", default="rtk_fusion")
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "bag_contract.yaml",
    )
    parser.add_argument("--output", type=Path, help="also write the JSON report here")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        report = evaluate(
            read_yaml(metadata_path(args.bag)), read_yaml(args.contract), args.profile
        )
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(json.dumps({"ok": False, "error": str(error)}, indent=2), file=sys.stderr)
        return 2
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
