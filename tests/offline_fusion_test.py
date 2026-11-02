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

        profiling = json.loads((folder / "fusion_profile.json").read_text())
        self.assertEqual(profiling["format"], "FUSPROFILE1")
        self.assertEqual(profiling["scene"], report["scene"])
        self.assertEqual(profiling["matcher"], report["matcher"])
        self.assertEqual(profiling["frames"], report["frames"])
        self.assertFalse(profiling["realtime_capability_verified"])
        self.assertAlmostEqual(profiling["scan_period_ms"],
                               report["scan_period_ns"] * 1e-6, places=6)
        self.assertAlmostEqual(profiling["input_rate_fps"],
                               1000 / profiling["scan_period_ms"], places=6)
        if profiling["profiled"]:
            self.assertGreater(profiling["fusion_compute_ms"], 0)
            self.assertGreater(profiling["effective_processing_fps"], 0)
            self.assertGreater(profiling["frame_proxy_p95_ms"], 0)
            self.assertLessEqual(profiling["frame_proxy_p50_ms"],
                                 profiling["frame_proxy_p95_ms"])
            self.assertLessEqual(profiling["frame_proxy_p95_ms"],
                                 profiling["frame_proxy_max_ms"])
            for name, calls in (("sensor_integration", 1),
                                ("scan_deskew", report["frames"]),
                                ("heldout_scan_match",
                                 report["heldout_attempted"]),
                                ("fused_map_insert", report["frames"]),
                                ("fused_map_build", 1)):
                self.assertEqual(profiling["stages"][name]["calls"], calls)
                self.assertGreaterEqual(profiling["stages"][name]["total_ms"], 0)
        else:
            self.assertIsNone(profiling["fusion_compute_ms"])
            self.assertIsNone(profiling["effective_processing_fps"])
            self.assertIsNone(profiling["frame_proxy_p95_ms"])

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
        audit = json.loads((folder / "lidar_only_match_audit.json").read_text())
        self.assertEqual(audit["format"], "FUSLIDARAUDIT1")
        self.assertFalse(audit["ground_truth_used"])
        self.assertFalse(audit["support_probe_read_only"])
        self.assertEqual(audit["attempts"],
                         report["lidar_only_accepted"] + report["lidar_only_rejected"])
        self.assertEqual(len(audit["frames"]), audit["attempts"])
        self.assertEqual(sum(int(f["accepted"]) for f in audit["frames"]),
                         report["lidar_only_accepted"])
        self.assertEqual(
            sum(f["status"] == "insufficient_support" and
                not f["candidate_evaluated"] and
                f["solver_residual_blocks"] == 0 for f in audit["frames"]),
            report["lidar_only_before_solver_insufficient_support"])
        self.assertEqual(
            sum(f["status"] == "insufficient_support" and
                f["candidate_evaluated"] for f in audit["frames"]),
            report["lidar_only_after_solver_insufficient_support"])
        self.assertTrue(all(f["input_beams"] > 0 and
                            0 <= f["seed_overlap"] <= 1 and
                            f["seed_supported_hits"] <= f["input_beams"]
                            for f in audit["frames"]))
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

    def test_matching_options_apply_to_all_synthetic_maps(self):
        # These are both available scenes; mapping stride affects all maps,
        # matching cadence/stride affects both held-out and LiDAR-only paths.
        for scene in ("apartment", "warehouse"):
            with self.subTest(scene=scene), tempfile.TemporaryDirectory() as root:
                folder = pathlib.Path(root) / "tuned"
                result = run(
                    EXE, scene, str(folder),
                    "--match-every", "3", "--field-every", "7",
                    "--map-stride", "2", "--match-stride", "2",
                    "--distance-truncation", "0.5")
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads((folder / "fusion_report.json").read_text())
                trace = json.loads((folder / "fusion_replay.json").read_text())
                self.assertEqual(report["match_every"], 3)
                self.assertEqual(report["field_every"], 7)
                self.assertEqual(report["map_stride"], 2)
                self.assertEqual(report["match_stride"], 2)
                self.assertEqual(report["match_offset"], 0)
                self.assertEqual(report["distance_truncation_m"], 0.5)
                self.assertEqual(trace["match_every"], 3)
                self.assertEqual(trace["field_every"], 7)
                self.assertEqual(trace["map_stride"], 2)
                self.assertEqual(trace["match_stride"], 2)
                self.assertEqual(report["heldout_attempted"] +
                                 report["heldout_skipped"], report["frames"] // 2)
                self.assertEqual(report["lidar_only_accepted"] +
                                 report["lidar_only_rejected"] +
                                 report["lidar_only_skipped"], report["frames"] - 1)
                self.assertGreater(report["lidar_only_field_rebuilds"], 1)
                self.assertLess(report["lidar_only_field_rebuilds"],
                                report["frames"] - 1)
                for i, frame in enumerate(trace["frames"]):
                    attempted = i % 2 == 1 and ((i-1)//2) % 3 == 0
                    self.assertEqual(frame["matching"]["attempted"], attempted)
                    if i % 2 and not attempted:
                        self.assertEqual(frame["matching"]["status"], "skipped")
                profile = json.loads((folder / "fusion_profile.json").read_text())
                if profile["profiled"]:
                    self.assertEqual(
                        profile["stages"]["heldout_scan_match"]["calls"],
                        report["heldout_attempted"])
                for name in report["results"]:
                    self.assertTrue((folder / f"{name}.slammap").is_file())

    def test_lidar_only_support_probe_is_read_only(self):
        with tempfile.TemporaryDirectory() as root:
            for tag, flags in (("baseline", ()), ("probed", ("--lidar-only-support-probe",))):
                result = run(EXE, "apartment", str(pathlib.Path(root) / tag), *flags)
                self.assertEqual(result.returncode, 0, result.stderr)
            baseline = json.loads(
                (pathlib.Path(root) / "baseline" / "fusion_report.json").read_text())
            probed = json.loads(
                (pathlib.Path(root) / "probed" / "fusion_report.json").read_text())
            self.assertFalse(baseline["lidar_only_support_probe"])
            self.assertTrue(probed["lidar_only_support_probe"])
            self.assertEqual(baseline["lidar_only_accepted"],probed["lidar_only_accepted"])
            self.assertEqual(baseline["lidar_only_rejected"],probed["lidar_only_rejected"])
            for name in baseline["results"]:
                self.assertEqual(
                    baseline["results"][name]["map"]["map_fingerprint"],
                    probed["results"][name]["map"]["map_fingerprint"])
            audit = json.loads(
                (pathlib.Path(root) / "probed" / "lidar_only_match_audit.json").read_text())
            self.assertTrue(audit["support_probe_read_only"])
            self.assertEqual(len(audit["frames"]),probed["frames"]-1)
            self.assertTrue(all(f["probe_poses"] == 13*13*9
                                for f in audit["frames"][:3]))
            self.assertTrue(all(f["probe_poses"] == 0
                                for f in audit["frames"][3:]))
            self.assertTrue(all(f["probe_ranked_poses"] <=
                                f["probe_supported_poses"] <= f["probe_poses"]
                                for f in audit["frames"]))

    def test_lidar_only_scan_to_scan_bootstrap(self):
        # Compare each bootstrapped experiment to the old estimator, keeping
        # both scenes' gyro/wheel, held-out and graph paths unchanged.
        for scene in ("apartment", "warehouse"):
            with self.subTest(scene=scene), tempfile.TemporaryDirectory() as root:
                before = pathlib.Path(root) / "without"
                after = pathlib.Path(root) / "with"
                for folder, flags in (
                    (before, ()),
                    (after, ("--lidar-only-bootstrap",)),
                ):
                    process = run(EXE, scene, str(folder), *flags)
                    self.assertEqual(process.returncode, 0, process.stderr)
                old = json.loads((before / "fusion_report.json").read_text())
                new = json.loads((after / "fusion_report.json").read_text())
                self.assertFalse(old["lidar_only_bootstrap"])
                self.assertTrue(new["lidar_only_bootstrap"])
                self.assertEqual(old["lidar_only_accepted"], 0)
                self.assertEqual(old["lidar_only_bootstrap_accepted"], 0)
                self.assertGreater(new["lidar_only_bootstrap_attempted"], 0)
                self.assertGreater(new["lidar_only_bootstrap_accepted"], 0)
                self.assertEqual(new["lidar_only_bootstrap_attempted"],
                                 new["lidar_only_bootstrap_accepted"] +
                                 new["lidar_only_bootstrap_rejected"])
                self.assertEqual(new["lidar_only_accepted"],
                                 new["lidar_only_edt_accepted"] +
                                 new["lidar_only_bootstrap_accepted"])
                self.assertEqual(new["lidar_only_accepted"] +
                                 new["lidar_only_rejected"] +
                                 new["lidar_only_skipped"], new["frames"]-1)
                self.assertGreater(
                    new["results"]["lidar_only"]["map"]["counts"]["known"],
                    old["results"]["lidar_only"]["map"]["counts"]["known"])
                # Independent simulator reference is used only for scoring,
                # not registration. The prior baseline is frozen at its first
                # pose, so any bootstrapping that worsens that trajectory
                # materially must fail regression rather than count as fixed.
                self.assertLess(
                    new["results"]["lidar_only"]["translation_rmse_m"],
                    .9 * old["results"]["lidar_only"]["translation_rmse_m"])
                for name in ("wheel_only", "gyro_wheel", "fused_scan_matched"):
                    self.assertEqual(
                        old["results"][name]["map"]["map_fingerprint"],
                        new["results"][name]["map"]["map_fingerprint"])
                old_paths = json.loads((before / "trajectories.json").read_text())
                new_paths = json.loads((after / "trajectories.json").read_text())
                for name, path in old_paths["paths"].items():
                    if name != "lidar_only":
                        self.assertEqual(path,new_paths["paths"][name])
                audit = json.loads(
                    (after / "lidar_only_match_audit.json").read_text())
                self.assertTrue(audit["lidar_only_bootstrap"])
                self.assertEqual(
                    sum(row["registration_method"] in
                        ("scan_to_scan","edt_refined") for row in audit["frames"]),
                    new["lidar_only_bootstrap_accepted"])
                self.assertTrue(all(
                    row["registration_forward_overlap"]>=.65 and
                    row["registration_reverse_overlap"]>=.65 and
                    row["registration_error_m"]<=.20 and
                    row["registration_translation_m"]<=.90 and
                    row["registration_yaw_rad"]<=.32
                    for row in audit["frames"]
                    if row["registration_method"] in ("scan_to_scan","edt_refined")))
                self.assertTrue(all(row["accepted"] ==
                                    (row["status"] == "accepted")
                                    for row in audit["frames"]))

    def test_lidar_prediction_precedes_edt_and_is_isolated(self):
        # These controlled scenes can be evaluated after localization; their
        # synthetic truth is never given to a registration or motion predictor.
        for scene in ("apartment", "warehouse"):
            with self.subTest(scene=scene), tempfile.TemporaryDirectory() as root:
                prior=pathlib.Path(root)/"prior"
                predicted=pathlib.Path(root)/"predict"
                common=("--lidar-only-bootstrap", "--match-stride", "1")
                for folder, flags in (
                    (prior,common),
                    (predicted,common+("--lidar-only-predict-first",)),
                ):
                    process=run(EXE,scene,str(folder),*flags)
                    self.assertEqual(process.returncode,0,process.stderr)
                old=json.loads((prior/"fusion_report.json").read_text())
                new=json.loads((predicted/"fusion_report.json").read_text())
                self.assertFalse(old["lidar_only_predict_first"])
                self.assertTrue(new["lidar_only_predict_first"])
                self.assertTrue(new["lidar_only_bootstrap"])
                self.assertGreater(new["lidar_only_accepted"],0)
                self.assertEqual(
                    new["lidar_only_accepted"],
                    new["lidar_only_edt_accepted"]+
                    new["lidar_only_bootstrap_accepted"])
                self.assertEqual(
                    new["lidar_only_bootstrap_attempted"],
                    new["lidar_only_bootstrap_accepted"]+
                    new["lidar_only_bootstrap_rejected"])
                self.assertLessEqual(
                    new["lidar_only_predicted_edt_accepted"],
                    new["lidar_only_edt_accepted"])
                for name in ("wheel_only","gyro_wheel","fused_scan_matched"):
                    self.assertEqual(
                        old["results"][name]["map"]["map_fingerprint"],
                        new["results"][name]["map"]["map_fingerprint"])
                paths_before=json.loads((prior/"trajectories.json").read_text())
                paths_after=json.loads((predicted/"trajectories.json").read_text())
                for name in paths_before["paths"]:
                    if name!="lidar_only":
                        self.assertEqual(
                            paths_before["paths"][name],paths_after["paths"][name])
                audit=json.loads(
                    (predicted/"lidar_only_match_audit.json").read_text())
                self.assertTrue(audit["lidar_only_predict_first"])
                self.assertEqual(audit["attempts"],new["frames"]-1)
                self.assertTrue(all(
                    row["used_scan_prediction"] for row in audit["frames"]))
                self.assertEqual(
                    sum(row["registration_method"]=="edt_predicted"
                        for row in audit["frames"]),
                    new["lidar_only_predicted_edt_accepted"])
                self.assertEqual(
                    sum(row["edt_prediction_disagreed"] for row in audit["frames"]),
                    new["lidar_only_edt_prediction_disagreements"])
                self.assertTrue(all(
                    row["predicted_edt_translation_delta_m"]<=.12+1e-9 and
                    row["predicted_edt_yaw_delta_rad"]<=.075+1e-9
                    for row in audit["frames"]
                    if row["registration_method"]=="edt_predicted"))
                # The first warehouse scan's independent synthetic motion
                # is ~0.303 m. It is checked AFTER the run, never seeded.
                if scene=="warehouse":
                    t=paths_after["synthetic_truth_for_scoring"]
                    e=paths_after["paths"]["lidar_only"]
                    actual=math.hypot(t[1][0]-t[0][0],t[1][1]-t[0][1])
                    recovered=math.hypot(e[1][0]-e[0][0],e[1][1]-e[0][1])
                    self.assertAlmostEqual(actual,.303,delta=.02)
                    self.assertAlmostEqual(recovered,actual,delta=.10)

    def test_continuous_cost_for_both_scenes_or_fail_closed(self):
        for scene in ("apartment", "warehouse"):
            with self.subTest(scene=scene), tempfile.TemporaryDirectory() as root:
                folder = pathlib.Path(root) / "continuous"
                result = run(EXE, scene, str(folder),
                             "--continuous-cost", "--match-every", "2",
                             "--match-stride", "2")
                if result.returncode != 0:
                    self.assertIn("Ceres-enabled", result.stderr)
                    self.assertFalse(folder.exists())
                    continue
                report = json.loads((folder / "fusion_report.json").read_text())
                trace = json.loads((folder / "fusion_replay.json").read_text())
                self.assertEqual(report["matcher"], "ceres_continuous_cost")
                self.assertEqual(trace["matcher"], report["matcher"])
                self.assertEqual(report["match_every"], 2)
                self.assertEqual(report["match_stride"], 2)
                self.assertEqual(report["heldout_attempted"] +
                                 report["heldout_skipped"], report["frames"] // 2)

    def test_tuning_rejects_invalid_options_before_writing(self):
        bad = (("--lidar-only-predict-first",), ("--match-every", "0"), ("--match-every", "201"),
               ("--match-stride", "0"), ("--match-stride", "65"),
               ("--map-stride", "-1"), ("--field-every", "0"),
               ("--match-offset", "-1"), ("--match-offset", "64"),
               ("--match-offset", "00"),
               ("--distance-truncation", "nan"),
               ("--distance-truncation", "3.1"))
        with tempfile.TemporaryDirectory() as root:
            for i, flags in enumerate(bad):
                with self.subTest(flags=flags):
                    dest = pathlib.Path(root) / f"invalid-{i}"
                    result = run(EXE, "apartment", str(dest), *flags)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertFalse(dest.exists())

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
