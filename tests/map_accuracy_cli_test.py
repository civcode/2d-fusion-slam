#!/usr/bin/env python3
"""Standalone CTest: native fixture -> FUSMAP2D import -> ground truth JSON."""
import json
import subprocess
import sys
import unittest


class GroundTruthCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executable, cls.fixture = sys.argv[1:3]

    def run_cli(self, scene, filename=None):
        return subprocess.run(
            [self.executable, scene, filename or self.fixture],
            capture_output=True, text=True, check=False, timeout=90,
        )

    def test_apartment_json_and_determinism(self):
        first = self.run_cli("apartment")
        self.assertEqual(first.returncode, 0, first.stderr)
        second = self.run_cli("apartment")
        self.assertEqual(first.stdout, second.stdout)
        report = json.loads(first.stdout)
        self.assertEqual(report["format"], "FUSMAPACC1")
        self.assertEqual(report["reference_class"], "synthetic_exact")
        self.assertEqual(report["reference_scene"], "synthetic-apartment-v1")
        self.assertEqual(report["alignment"], "declared_same_metric_map_frame_no_fitting")
        counts = report["counts"]
        self.assertGreater(counts["truth_wall"], 0)
        self.assertGreater(counts["predicted_wall"], 0)
        self.assertEqual(counts["known"] + counts["unknown"],
                         report["width"] * report["height"])
        self.assertEqual(counts["truth_wall"],
                         report["confusion"]["tp"] + report["confusion"]["fn_observed_free"]
                         + report["confusion"]["unknown_wall"])
        metrics = report["metrics"]
        self.assertGreaterEqual(metrics["occupied_iou"], 0)
        self.assertLessEqual(metrics["occupied_iou"], 1)
        self.assertGreater(metrics["known_coverage"], 0)
        self.assertLessEqual(metrics["known_coverage"], 1)
        self.assertGreaterEqual(metrics["symmetric_mean_wall_distance_m"], 0)

    def test_wrong_reference_refused(self):
        result = self.run_cli("warehouse")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertIn("refused", result.stderr.lower())

    def test_nonexistent_bundle_refused(self):
        result = self.run_cli("apartment", self.fixture + ".not-present")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: test_map_accuracy_cli.py <binary> <fixture.slammap>")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
