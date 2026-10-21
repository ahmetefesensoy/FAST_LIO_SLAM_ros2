#!/usr/bin/env python3
"""Replay one PGO-input bag across keyframe gaps and emit machine-readable evidence."""

import argparse
import csv
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from bag_preflight import evaluate, metadata_path, read_yaml


def bundled_config(name: str) -> Path:
    candidates = [
        TOOLS_DIR.parent / "config" / name,
        TOOLS_DIR.parents[1] / "share" / "aloam_velodyne" / "config" / name,
    ]
    return next((path for path in candidates if path.exists()), candidates[0])


def gap_slug(gap: float) -> str:
    return f"{gap:g}".replace(".", "p")


def create_plan(bag: Path, output: Path, params_file: Path, gaps: list[float]) -> list[dict]:
    plan = []
    for gap in gaps:
        name = f"keyframe_{gap_slug(gap)}m"
        run_dir = output / name
        metrics = run_dir / "pgo_metrics.json"
        telemetry = run_dir / "telemetry"
        pgo_argv = [
            "ros2", "run", "aloam_velodyne", "sc_pgo", "--ros-args",
            "--params-file", str(params_file),
            "-p", f"keyframe_meter_gap:={gap}",
            "-p", "use_sim_time:=true",
            "-p", f"save_directory:={run_dir / 'map'}",
            "-p", f"metrics_output_path:={metrics}",
        ]
        record_argv = [
            "ros2", "bag", "record", "-o", str(telemetry),
            "/aft_pgo_odom", "/pgo/diagnostics", "/tf", "/tf_static",
        ]
        play_argv = ["ros2", "bag", "play", str(bag), "--clock"]
        plan.append({
            "name": name,
            "keyframe_meter_gap": gap,
            "run_directory": str(run_dir.relative_to(output)),
            "metrics_file": str(metrics.relative_to(output)).replace("\\", "/"),
            "telemetry_bag": str(telemetry.relative_to(output)).replace("\\", "/"),
            "pgo_argv": pgo_argv,
            "record_argv": record_argv,
            "play_argv": play_argv,
            "pgo_command": " ".join(pgo_argv),
        })
    return plan


def blocked_report(bag: Path, reason: str) -> dict:
    return {
        "status": "blocked_by_missing_data",
        "results_available": False,
        "bag": str(bag),
        "reason": reason,
    }


def stop_process(process: subprocess.Popen, timeout: float = 10.0) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def process_sample(pid: int) -> tuple[float, int] | None:
    result = subprocess.run(
        ["ps", "-p", str(pid), "-o", "%cpu=,rss="],
        check=False, capture_output=True, text=True,
    )
    fields = result.stdout.split()
    if len(fields) != 2:
        return None
    return float(fields[0]), int(fields[1])


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int((len(ordered) - 1) * fraction))
    return ordered[index]


def run_one(run: dict) -> dict:
    run_dir = Path(run["run_directory_absolute"])
    run_dir.mkdir(parents=True, exist_ok=False)
    logs = {name: (run_dir / f"{name}.log").open("w", encoding="utf-8")
            for name in ("pgo", "record", "play")}
    processes = []
    started = time.monotonic()
    cpu_samples = []
    rss_samples = []
    try:
        pgo = subprocess.Popen(run["pgo_argv"], stdout=logs["pgo"], stderr=subprocess.STDOUT)
        processes.append(pgo)
        time.sleep(2.0)
        recorder = subprocess.Popen(
            run["record_argv"], stdout=logs["record"], stderr=subprocess.STDOUT
        )
        processes.append(recorder)
        time.sleep(1.0)
        player = subprocess.Popen(
            run["play_argv"], stdout=logs["play"], stderr=subprocess.STDOUT
        )
        processes.append(player)
        while player.poll() is None:
            sample = process_sample(pgo.pid)
            if sample:
                cpu_samples.append(sample[0])
                rss_samples.append(sample[1])
            time.sleep(0.2)
        play_exit_code = player.returncode
        time.sleep(2.0)
    finally:
        for process in reversed(processes):
            stop_process(process)
        for stream in logs.values():
            stream.close()

    metrics_path = Path(run["metrics_file_absolute"])
    pgo_metrics = None
    if metrics_path.exists():
        pgo_metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    return {
        "name": run["name"],
        "keyframe_meter_gap": run["keyframe_meter_gap"],
        "status": "completed" if play_exit_code == 0 and pgo_metrics else "incomplete",
        "elapsed_seconds": time.monotonic() - started,
        "bag_play_exit_code": play_exit_code,
        "cpu_percent_mean": sum(cpu_samples) / len(cpu_samples) if cpu_samples else None,
        "cpu_percent_p95": percentile(cpu_samples, 0.95),
        "rss_kib_max": max(rss_samples) if rss_samples else None,
        "pgo_metrics": pgo_metrics,
        "telemetry_bag": run["telemetry_bag"],
        "accuracy": {
            "status": "not_computed",
            "reason": "ATE/RPE requires a ground-truth trajectory",
        },
    }


def write_report(output: Path, report: dict) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "benchmark_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not report.get("runs"):
        return
    with (output / "benchmark_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "name", "keyframe_meter_gap", "status", "elapsed_seconds",
            "cpu_percent_mean", "cpu_percent_p95", "rss_kib_max",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["runs"])


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--params-file", type=Path, default=bundled_config("pgo_ros2.yaml"))
    parser.add_argument("--contract", type=Path, default=bundled_config("bag_contract.yaml"))
    parser.add_argument("--gaps", nargs="+", type=float, default=[0.5, 1.0, 2.0])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    metadata = metadata_path(args.bag)
    if not metadata.exists():
        report = blocked_report(args.bag, f"metadata.yaml not found at {metadata}")
        write_report(args.output, report)
        print(json.dumps(report, indent=2))
        return 2
    preflight = evaluate(read_yaml(metadata), read_yaml(args.contract), "rtk_fusion")
    if not preflight["ok"]:
        report = blocked_report(args.bag, "rtk_fusion topic preflight failed")
        report["preflight"] = preflight
        write_report(args.output, report)
        print(json.dumps(report, indent=2))
        return 2

    plan = create_plan(args.bag, args.output, args.params_file, args.gaps)
    if args.dry_run:
        report = {
            "status": "planned",
            "results_available": False,
            "bag": str(args.bag),
            "preflight": preflight,
            "plan": plan,
        }
        write_report(args.output, report)
        print(json.dumps(report, indent=2))
        return 0

    missing_commands = [name for name in ("ros2", "ps") if shutil.which(name) is None]
    if missing_commands:
        report = blocked_report(args.bag, f"missing commands: {', '.join(missing_commands)}")
        write_report(args.output, report)
        print(json.dumps(report, indent=2))
        return 2

    runs = []
    for item in plan:
        item["run_directory_absolute"] = str(args.output / item["run_directory"])
        item["metrics_file_absolute"] = str(args.output / item["metrics_file"])
        runs.append(run_one(item))
    report = {
        "status": "completed" if all(run["status"] == "completed" for run in runs) else "incomplete",
        "results_available": all(run["status"] == "completed" for run in runs),
        "bag": str(args.bag),
        "preflight": preflight,
        "runs": runs,
    }
    write_report(args.output, report)
    print(json.dumps(report, indent=2))
    return 0 if report["results_available"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
