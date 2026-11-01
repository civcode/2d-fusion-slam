#!/usr/bin/env python3
"""Exercise real C++ fusion CLI, 5%-grid map metrics and fail-closed outputs.

Never claims the synthetic experiment is independent real hardware evidence.
"""
from __future__ import annotations
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest

EXE = None
SCORE = None


def run(*argv, timeout=180):
    return subprocess.run(argv, capture_output=True, text=True,
                          timeout=timeout, check=False)


class OfflineFusionExperiment(unittest.TestCase):
    def scene(self, scene, root):
        folder = pathlib.Path(root) / scene
        folder.parent.mkdir(parents=True, exist_ok=True)
        result = run(EXE, scene, str(folder))
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((folder / "fusion_report.json").read_text())
        self.assertEqual(report["format"], "FUSFUSION1")
        self.assertEqual(report["scene"], f"synthetic-{scene}-v1")
        self.assertFalse(report["map_pose_truth_used"])
        self.assertFalse(report["physical_gyro_calibration_verified"])
        self.assertTrue(report["full_se2_deskew_verified"])
        self.assertEqual(report["sensor_clock"], "synthetic_exact_single_epoch")
        self.assertEqual(report["stationary_calibration_ticks"], 12)
        self.assertGreater(report["frames"], 20)
        pose_export = json.loads((folder / "trajectories.json").read_text())
        self.assertEqual(pose_export["format"], "FUSPOSES1")
        self.assertEqual(pose_export["clock_domain"], 101)
        self.assertEqual(pose_export["clock_epoch"], "1")
        self.assertEqual(pose_export["reference_class"], "synthetic_exact")
        self.assertEqual(len(pose_export["timestamps_ns"]), report["frames"])
        self.assertEqual(len(pose_export["synthetic_truth_for_scoring"]),
                         report["frames"])
        if scene == "warehouse":
            self.assert_warehouse_route_clear(pose_export["synthetic_truth_for_scoring"])
        self.assertEqual(set(pose_export["paths"]), set(report["results"]))
        for path in pose_export["paths"].values():
            self.assertEqual(len(path), report["frames"])
            self.assertTrue(all(len(p) == 3 for p in path))

        trace = json.loads((folder / "fusion_replay.json").read_text())
        self.assertEqual(trace["format"], "FUSFUSREPLAY1")
        self.assertEqual(trace["scene"], report["scene"])
        self.assertEqual(trace["matcher"], report["matcher"])
        self.assertEqual(trace["frame_count"], report["frames"])
        self.assertEqual(trace["reference_class"], "synthetic_exact")
        self.assertEqual(trace["scan_frame"], "laser_midpoint_deskewed_full_se2")
        self.assertEqual(len(trace["frames"]), report["frames"])
        accepted = rejected = loop_accepted = 0
        for i, frame in enumerate(trace["frames"]):
            self.assertEqual(frame["index"], i)
            self.assertEqual(frame["time_ns"], pose_export["timestamps_ns"][i])
            self.assertLessEqual(len(frame["points_laser"]), frame["hits"])
            self.assertTrue(frame["mapping"]["fused_scan_matched_inserted"])
            self.assertEqual(frame["matching"]["attempted"], i % 2 == 1)
            if i % 2:
                accepted += int(frame["matching"]["accepted"])
                rejected += int(not frame["matching"]["accepted"])
            self.assertEqual(frame["matching"]["cumulative_accepted"], accepted)
            self.assertEqual(frame["matching"]["cumulative_rejected"], rejected)
            loop_accepted += sum(event["decision"] == "accepted"
                                 for event in frame["loop_events"])
            self.assertEqual(frame["cumulative_accepted_loops"], loop_accepted)
        self.assertEqual(accepted, report["heldout_accepted"])
        self.assertEqual(rejected, report["heldout_rejected"])
        self.assertEqual(loop_accepted, report["graph_accepted_loops"])

        self.assertGreater(report["heldout_attempted"], 10)
        self.assertEqual(report["heldout_attempted"],
                         report["heldout_accepted"] + report["heldout_rejected"])
        for prefix in ("heldout", "lidar_only"):
            reasons = report[prefix + "_rejection_reasons"]
            self.assertEqual(sum(reasons.values()), report[prefix + "_rejected"])
            self.assertTrue(all(isinstance(count, int) and count > 0 for count in reasons.values()))
            self.assertLessEqual(set(reasons), {"insufficient_support", "degenerate",
                                               "no_improvement", "invalid_input", "qualification_gate"})
        self.assertEqual(report["lidar_only_accepted"] + report["lidar_only_rejected"],
                         report["frames"] - 1)
        self.assertIn(report["matcher"],
                      ("ceres", "geometry_reference_fallback"))
        self.assertEqual(report["loop_measurements"], "none")
        self.assertEqual(report["graph_loop_candidates"], 0)
        self.assertEqual(report["graph_accepted_loops"], 0)
        self.assertGreater(report["gyro_bias_estimate_rad_s"], .01)
        self.assertLess(report["gyro_bias_estimate_rad_s"], .04)

        labels = ("wheel_only", "lidar_only", "gyro_wheel", "fused_scan_matched")
        if report["graph_status"] != "gtsam_not_built":
            labels += ("graph_optimized",)
        for label in labels:
            with self.subTest(label=label):
                entry = report["results"][label]
                self.assertGreaterEqual(entry["translation_rmse_m"], 0)
                self.assertGreaterEqual(entry["translation_p95_m"], 0)
                self.assertGreaterEqual(entry["yaw_rmse_rad"], 0)
                map_score = entry["map"]
                self.assertEqual(map_score["format"], "FUSMAPACC1")
                self.assertEqual(map_score["reference_scene"], report["scene"])
                self.assertEqual(map_score["reference_class"], "synthetic_exact")
                self.assertGreater(map_score["counts"]["truth_wall"], 0)
                self.assertGreater(map_score["counts"]["predicted_wall"], 0)
                self.assertGreater(map_score["counts"]["known"], 0)
                occupied_iou = map_score["metrics"]["occupied_iou"]
                self.assertIsNotNone(occupied_iou)
                self.assertGreaterEqual(occupied_iou, 0)
                self.assertLessEqual(occupied_iou, 1)
                exported = folder / f"{label}.slammap"
                self.assertTrue(exported.is_file(), exported)
                score = run(SCORE, scene, str(exported), timeout=120)
                self.assertEqual(score.returncode, 0, score.stderr)
                independent = json.loads(score.stdout)
                self.assertEqual(independent["map_fingerprint"],
                                 map_score["map_fingerprint"])
                self.assertEqual(independent["confusion"], map_score["confusion"])
                self.assertEqual(independent["metrics"], map_score["metrics"])
        if report["graph_status"] == "gtsam_not_built":
            self.assertNotIn("graph_optimized", report["results"])
            self.assertFalse((folder / "graph_optimized.slammap").exists())
        else:
            self.assertIn("graph_optimized", report["results"])
            self.assertTrue((folder / "graph_optimized.slammap").is_file())
            self.assertIn("gtsam_", report["graph_status"])
        return report, (folder / "fusion_report.json").read_bytes()

    def assert_warehouse_route_clear(self, poses):
        # Independent shelf-solid check, including motion between scan poses.
        # Protect both a 0.2 m-radius base and the offset laser origin.
        shelves = [(x, x+1, 4., 16.) for x in (6., 12., 18., 24.)]
        for first, second in zip(poses, poses[1:]):
            steps = max(1, math.ceil(math.hypot(second[0]-first[0], second[1]-first[1])/.02))
            for i in range(steps + 1):
                u = i / steps
                x, y = (first[j] + u*(second[j]-first[j]) for j in range(2))
                self.assertGreaterEqual(min(x, y, 30-x, 20-y), .2)
                for left, right, bottom, top in shelves:
                    distance = math.hypot(max(left-x, 0, x-right), max(bottom-y, 0, y-top))
                    self.assertGreaterEqual(distance, .2, "reference robot intersects a shelf")
        for x, y, yaw in poses:
            lx = x + .12*math.cos(yaw) - .01*math.sin(yaw)
            ly = y + .12*math.sin(yaw) + .01*math.cos(yaw)
            self.assertTrue(0 < lx < 30 and 0 < ly < 20)
            self.assertFalse(any(l <= lx <= r and b <= ly <= t for l, r, b, t in shelves))

    def test_apartment(self):
        with tempfile.TemporaryDirectory() as root:
            self.scene("apartment", root)

    def test_warehouse(self):
        with tempfile.TemporaryDirectory() as root:
            self.scene("warehouse", root)

    def test_explicit_independent_synthetic_loop_source(self):
        # No fake loop closures should be silently generated by default.
        with tempfile.TemporaryDirectory() as root:
            baseline, _ = self.scene("apartment", pathlib.Path(root) / "baseline")
            if baseline["graph_status"] == "gtsam_not_built":
                self.skipTest("optional GTSAM backend not compiled")
            new = pathlib.Path(root) / "simulated-loops"
            run_result = run(EXE, "apartment", str(new),
                             "--synthetic-independent-loops")
            self.assertEqual(run_result.returncode, 0, run_result.stderr)
            report = json.loads((new / "fusion_report.json").read_text())
            self.assertEqual(report["loop_measurements"],
                             "independently_simulated_relative_pose")
            self.assertEqual(report["graph_status"],
                             "gtsam_with_simulated_independent_loops")
            self.assertLessEqual(report["graph_accepted_loops"],
                                 report["graph_loop_candidates"])
            self.assertIn("graph_optimized", report["results"])

    def test_repeatability_refusal_and_provenance(self):
        with tempfile.TemporaryDirectory() as root:
            first, bytes_first = self.scene("apartment", pathlib.Path(root) / "one")
            # Because the parent directory has to exist, repeat with a
            # different sibling folder under the same parent.
            folder = pathlib.Path(root) / "two"
            folder.mkdir()
            second, bytes_second = self.scene("apartment", folder)
            self.assertEqual(bytes_first, bytes_second)
            self.assertEqual(first["results"]["wheel_only"],
                             second["results"]["wheel_only"])
            refused = run(EXE, "apartment", str(pathlib.Path(root) / "one" / "apartment"))
            self.assertNotEqual(refused.returncode, 0)
            self.assertIn("refuse", refused.stderr)
            missing = run(EXE, "not-a-scene", str(pathlib.Path(root) / "bad"))
            self.assertNotEqual(missing.returncode, 0)
            if first["graph_status"] == "gtsam_not_built":
                invalid = run(EXE, "apartment", str(pathlib.Path(root) / "no-gtsam"),
                              "--synthetic-independent-loops")
                self.assertNotEqual(invalid.returncode, 0)
                self.assertIn("GTSAM", invalid.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: offline_fusion_test.py <fusion-CLI> <accuracy-CLI>")
    EXE, SCORE = sys.argv[1:]
    unittest.main(argv=[sys.argv[0]], verbosity=2)
