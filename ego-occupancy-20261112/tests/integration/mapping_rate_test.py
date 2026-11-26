import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TOOL = Path(__file__).resolve().parents[2] / "scripts/validation/measure_mapping_rate.py"
NANOSECONDS = 1_000_000_000
START = 123_456_789_000_000_001


class MappingRateTest(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.root = Path(scratch.name)

    def write_run(self, elapsed, events, *, failure="", mode="live", has_snapshot=True):
        accepted = sum(row[1] for row in events)
        published = sum(row[2] for row in events)
        end = START + int(elapsed * NANOSECONDS)
        fields = {
            "mode": mode,
            "window_start_monotonic_ns": f'"{START}"',
            "window_end_monotonic_ns": f'"{end}"',
            "mapping_target_hz": "10.",
            "fused": f"{accepted}.",
            "snapshot_publish_skipped": f"{accepted - published}.",
            "has_snapshot": int(has_snapshot),
            "failure": f'"{failure}"',
        }
        (self.root / "summary.yml").write_text(
            "%YAML:1.0\n---\n" + "".join(f"{key}: {value}\n" for key, value in fields.items())
        )
        with (self.root / "mapping.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("accepted", "published", "completed_monotonic_ns"))
            for seconds, fused, published in events:
                writer.writerow((fused, published, START + int(seconds * NANOSECONDS)))

    def run_report(self, seconds):
        return subprocess.run(
            [sys.executable, str(TOOL), str(self.root), "--seconds", str(seconds)],
            capture_output=True,
            text=True,
        )

    def test_fusion_and_publication_counts_exclude_window_end_and_drain(self):
        events = [(0.1 * i, 1, int(i < 6)) for i in range(8)]
        events.extend([(1.2, 0, 0), (2.0, 1, 1), (2.1, 1, 1)])
        self.write_run(2.15, events)
        result = self.run_report(2)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["duration_seconds"], 2)
        self.assertEqual(report["fused_hz"], 4)
        self.assertEqual(report["published_hz"], 3)
        self.assertEqual(report["published_per_second"], [6, 0])
        self.assertEqual(report["zero_update_seconds"], 1)

    def test_early_exit_reports_the_actual_observation_window(self):
        self.write_run(45, [(1, 1, 1), (2, 1, 1)], failure="no RGB-D frames: timed out")
        result = self.run_report(60)
        self.assertNotEqual(result.returncode, 0)
        report = json.loads(result.stdout)
        self.assertEqual(report["status"], "incomplete")
        self.assertEqual(report["duration_seconds"], 45)
        self.assertAlmostEqual(report["published_hz"], 2 / 45)
        self.assertIn("no RGB-D frames", report["failure"])

    def test_clean_early_exit_explains_short_window(self):
        self.write_run(1, [(0.2, 1, 1)])
        result = self.run_report(2)
        self.assertEqual(result.returncode, 1, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["duration_seconds"], 1)
        self.assertIn("ended before", report["incomplete_reason"])

    def test_no_updates_still_uses_the_full_duration(self):
        self.write_run(2, [], has_snapshot=False)
        result = self.run_report(2)
        self.assertNotEqual(result.returncode, 0)
        report = json.loads(result.stdout)
        self.assertEqual(report["published_hz"], 0)
        self.assertEqual(report["published_per_second"], [0, 0])
        self.assertEqual(report["zero_update_seconds"], 2)

    def test_legacy_log_reports_missing_timestamps(self):
        self.write_run(2, [(0.1, 1, 1)])
        (self.root / "mapping.csv").write_text("accepted,published\n1,1\n")
        result = self.run_report(2)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("completed_monotonic_ns", result.stderr)

    def test_replay_is_rejected(self):
        self.write_run(2, [(0.1, 1, 1)], mode="replay")
        result = self.run_report(2)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("live", result.stderr)


if __name__ == "__main__":
    unittest.main()
