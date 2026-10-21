import importlib.util
from pathlib import Path
import tempfile
import unittest
import json


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "run_bag_benchmark.py"


class BenchmarkRunnerTest(unittest.TestCase):
    def load_module(self):
        spec = importlib.util.spec_from_file_location("run_bag_benchmark", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_plan_contains_three_isolated_keyframe_runs(self):
        runner = self.load_module()
        with tempfile.TemporaryDirectory() as directory:
            plan = runner.create_plan(
                bag=Path("/data/erc_run"),
                output=Path(directory),
                params_file=Path("/config/pgo.yaml"),
                gaps=[0.5, 1.0, 2.0],
            )

        self.assertEqual([run["keyframe_meter_gap"] for run in plan], [0.5, 1.0, 2.0])
        self.assertEqual(plan[0]["name"], "keyframe_0p5m")
        self.assertEqual(plan[2]["name"], "keyframe_2m")
        self.assertIn("keyframe_meter_gap:=1.0", plan[1]["pgo_command"])
        self.assertEqual(plan[1]["metrics_file"], "keyframe_1m/pgo_metrics.json")
        self.assertEqual(plan[1]["telemetry_bag"], "keyframe_1m/telemetry")

    def test_missing_bag_report_is_explicitly_blocked(self):
        runner = self.load_module()

        report = runner.blocked_report(Path("/missing/bag"), "metadata.yaml not found")

        self.assertEqual(report["status"], "blocked_by_missing_data")
        self.assertFalse(report["results_available"])
        self.assertEqual(report["bag"], "/missing/bag")
        self.assertIn("metadata.yaml", report["reason"])

    def test_cli_writes_blocked_report_when_bag_is_missing(self):
        runner = self.load_module()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "report"
            exit_code = runner.main([
                "/missing/erc_bag",
                "--output", str(output),
            ])
            report = json.loads(
                (output / "benchmark_report.json").read_text(encoding="utf-8")
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(report["status"], "blocked_by_missing_data")
        self.assertFalse(report["results_available"])


if __name__ == "__main__":
    unittest.main()
