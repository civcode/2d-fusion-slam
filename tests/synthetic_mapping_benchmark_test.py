#!/usr/bin/env python3
"""Native synthetic front-end -> authoritative FUSMAP2D -> independent accuracy.

Does not substitute oracle-pose map construction; the benchmark executable
must generate estimated poses from perturbed relative odometry and scan data.
"""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


class SyntheticMappingBenchmarkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.benchmark, cls.evaluator, cls.build_dir = sys.argv[1:4]

    def benchmark_scene(self, scene, root):
        directory = pathlib.Path(root) / scene
        run = subprocess.run(
            [self.benchmark, scene, str(directory)],
            capture_output=True, text=True, timeout=120, check=False,
        )
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertTrue((directory / "baseline.slammap").is_file())
        self.assertTrue((directory / "estimated.slammap").is_file())
        report = json.loads((directory / "benchmark.json").read_text())
        self.assertEqual(report["format"], "FUSSYNTHBENCH1")
        self.assertEqual(report["truth_class"], "synthetic_exact")
        self.assertEqual(report["scene"], f"synthetic-{scene}-v1")
        self.assertEqual(report["method"],
                         "offline_even_training_odd_heldout_scan_to_map")
        self.assertIs(report["mapping_truth_poses_used"], False)
        self.assertEqual(report["pose_anchor"], "first_pose_only")
        self.assertGreater(report["scan_count"], 10)
        self.assertGreater(report["query_attempted"], 0)
        self.assertEqual(report["query_attempted"],
                         report["scan_count"] // 2,
                         "odd-index scans must be held out from matcher training")
        self.assertEqual(report["even_training_updates"],
                         (report["scan_count"] + 1) // 2)
        config = report["configuration"]
        self.assertEqual(config["scan_beams"], 360)
        self.assertEqual(config["odometry_forward_scale"], 1.018)
        self.assertEqual(config["matcher_min_supported_hits"], 18)
        self.assertEqual(report["query_attempted"],
                         report["query_accepted"] + report["query_rejected"])
        self.assertGreater(report["even_training_updates"], 0)
        self.assertGreater(report["query_accepted"], 0,
                           "held-out scan matching must actually run and accept")
        self.assertGreater(report["baseline_pose_error"]["translation_rmse_m"], 0)

        for key, filename in [("baseline_map", "baseline.slammap"),
                              ("candidate_map", "estimated.slammap")]:
            embedded = report[key]
            self.assertEqual(embedded["format"], "FUSMAPACC1")
            self.assertEqual(embedded["reference_scene"], report["scene"])
            self.assertEqual(embedded["reference_class"], "synthetic_exact")
            count = embedded["counts"]
            self.assertEqual(count["known"] + count["unknown"],
                             embedded["width"] * embedded["height"])
            self.assertGreater(count["truth_wall"], 0)
            self.assertGreater(count["predicted_wall"], 0)
            self.assertIsNotNone(embedded["metrics"]["occupied_iou"])
            self.assertGreaterEqual(embedded["metrics"]["occupied_iou"], 0)
            self.assertLessEqual(embedded["metrics"]["occupied_iou"], 1)
            self.assertLessEqual(embedded["metrics"]["known_coverage"], 1)
            out = subprocess.run(
                [self.evaluator, scene, str(directory / filename)],
                capture_output=True, text=True, timeout=120, check=False,
            )
            self.assertEqual(out.returncode, 0, out.stderr)
            self.assertEqual(json.loads(out.stdout), embedded,
                             "independent CLI agrees with benchmark JSON")
        return report, (directory / "benchmark.json").read_bytes()

    def test_apartment_and_warehouse_independently(self):
        with tempfile.TemporaryDirectory(prefix="fusion-slam-benchmark-") as root:
            for scene in ("apartment", "warehouse"):
                with self.subTest(scene=scene):
                    report, _ = self.benchmark_scene(scene, root)
                    self.assertGreaterEqual(report["candidate_pose_error"]
                                            ["translation_rmse_m"], 0)

    def test_deterministic_result_and_no_clobber(self):
        with tempfile.TemporaryDirectory(prefix="fusion-slam-repeat-") as root:
            a = pathlib.Path(root) / "first"
            b = pathlib.Path(root) / "second"
            runs = []
            for p in (a, b):
                process = subprocess.run(
                    [self.benchmark, "apartment", str(p)],
                    capture_output=True, text=True, timeout=120, check=False,
                )
                self.assertEqual(process.returncode, 0, process.stderr)
                runs.append((p / "benchmark.json").read_bytes())
            self.assertEqual(*runs, "benchmark result must be reproducible")
            refusal = subprocess.run(
                [self.benchmark, "apartment", str(a)],
                capture_output=True, text=True, timeout=5, check=False,
            )
            self.assertNotEqual(refusal.returncode, 0)
            self.assertIn("refuse", refusal.stderr)
            self.assertEqual(runs[0], (a / "benchmark.json").read_bytes(),
                             "existing benchmark must remain unchanged")

    def test_invalid_scene(self):
        with tempfile.TemporaryDirectory() as root:
            bad = subprocess.run(
                [self.benchmark, "fr079", str(pathlib.Path(root) / "bad")],
                capture_output=True, text=True, timeout=5, check=False,
            )
            self.assertNotEqual(bad.returncode, 0)
            self.assertIn("unknown", bad.stderr.lower())


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit("usage: test.py <benchmark-binary> <accuracy-binary> <build-dir>")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
