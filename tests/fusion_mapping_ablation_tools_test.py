#!/usr/bin/env python3
"""Standalone mapping-ablation report validation (no native SLAM required)."""
from __future__ import annotations

import contextlib
import io
import json
import math
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import fusion_mapping_ablation


class MappingAblationToolsTest(unittest.TestCase):
    def fixture(self, directory, *, scene="warehouse_asymmetric", region=False):
        directory.mkdir()
        scene_id = f"synthetic-{scene}-v1"
        n = 195
        cut = n - math.ceil(.12 * n)
        (directory / "mapping_ablation").mkdir()
        (directory / "fusion_report.json").write_text(json.dumps({
            "scene": scene_id,
            "frames": n,
            "lidar_loop_closure": True,
            "graph_status": "gtsam_with_lidar_scan_loop_closure",
            "graph_accepted_loops": 2,
            "matcher": "ceres_continuous_cost",
            "match_every": 1, "match_stride": 1, "field_every": 15,
            "map_stride": 1, "lidar_only_bootstrap": True,
            "lidar_only_predict_first": True, "distance_truncation_m": .7,
            "results": {"graph_optimized": {
                "map": {"map_fingerprint": "12345"}
            }}
        }))
        rows = []
        for window, begin, end in (("full", 0, n), ("early", 0, cut),
                                   ("late", cut, n)):
            for deskew in ("gyro_wheel_original", "graph_corrected_motion_model"):
                for evidence, hit, free in (
                    ("baseline", .85, -.4), ("stronger_hit", 1.2, -.4),
                    ("gentler_free", .85, -.2), ("combined", 1.2, -.2),
                ):
                    identifier = f"{window}_{deskew}_{evidence}"
                    filename = f"mapping_ablation/{identifier}.slammap"
                    (directory / filename).write_bytes(b"FUSMAP2D")
                    rows.append({
                        "id": identifier, "window": window,
                        "begin_frame": begin,
                        "end_frame_exclusive": end,
                        "deskew": deskew, "evidence": evidence,
                        "hit_log_odds": hit, "free_log_odds": free,
                        "map_path": filename,
                        "map": {
                            "format": "FUSMAPACC1",
                            "reference_scene": scene_id,
                            "map_fingerprint": "12345" if identifier ==
                                "full_gyro_wheel_original_baseline" else "different",
                            "metrics": {
                                "occupied_iou": .06, "precision": .16,
                                "recall": .10, "symmetric_mean_wall_distance_m": .37,
                                "symmetric_p95_wall_distance_m": 2.45,
                            }
                        },
                        "wall_tolerance_scores": [
                            {"tolerance_m": value, "precision": q,
                             "recall": q / 2, "f1": 2 * q / 3}
                            for value, q in ((.10, .20), (.25, .40), (.50, .60))
                        ]
                    })
                    if region:
                        variant = rows[-1]
                        covered = {"full": 2400, "early": 2000,
                                   "late": 800}[window]
                        variant["region_footprint"] = {
                            "source": "original_baseline_rays",
                            "padding_m": .25,
                            "cells": covered,
                            "total_grid_cells": 5000,
                            "fingerprint": str(covered * 789 + 123),
                        }
                        variant["region_map"] = {
                            **variant["map"],
                            "counts": {"truth_wall": 200,
                                       "known": covered - 100,
                                       "unknown": 100},
                        }
                        variant["region_wall_tolerance_scores"] = [
                            dict(b) for b in variant["wall_tolerance_scores"]]
        payload = {
            "format": "FUSMAPABL1",
            "source": "graph_optimized_fixed_estimated_poses",
            "reference_usage": "posthoc_scoring_only",
            "scene": scene_id,
            "region_mask_source": "original_deskew_baseline_ray_observation_padded",
            "region_padding_m": .25,
            "variants": rows
        }
        (directory / "mapping_ablation.json").write_text(json.dumps(payload))
        return payload

    def test_summary_csv_and_window_separation(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = pathlib.Path(tmp) / "run"
            self.fixture(run)
            csv_path = pathlib.Path(tmp) / "summary.csv"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                fusion_mapping_ablation.summarize(run, csv_path)
            self.assertTrue(csv_path.is_file())
            self.assertEqual(len(csv_path.read_text().splitlines()), 25)
            text = output.getvalue()
            for substring in ("scans 0..194", "scans 0..170", "scans 171..194",
                              "F1 10cm", "F1 25cm", "F1 50cm",
                              "FULL reference", "map fingerprint verified"):
                self.assertIn(substring, text)

    def test_region_mask_comparison_and_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = pathlib.Path(tmp) / "region"
            payload = self.fixture(run, region=True)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                fusion_mapping_ablation.summarize(run)
            self.assertIn("Region-masked", output.getvalue())
            self.assertIn("original-ray footprint", output.getvalue())
            values = fusion_mapping_ablation.normalized_row(payload["variants"][0])
            self.assertEqual(values["region_cells"], 2400)
            self.assertAlmostEqual(values["region_iou"], .06)
            payload["variants"][1]["region_footprint"]["cells"] = 2300
            (run / "mapping_ablation.json").write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "footprint changed"):
                fusion_mapping_ablation.load(run)
            payload["variants"][1]["region_footprint"]["cells"] = 2400
            payload["variants"][1]["region_footprint"]["fingerprint"] = "987654321"
            (run / "mapping_ablation.json").write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "footprint changed"):
                fusion_mapping_ablation.load(run)
            payload["variants"][1]["region_footprint"]["fingerprint"] = str(
                2400 * 789 + 123)
            payload["variants"][0]["region_map"]["counts"]["known"] = 100
            (run / "mapping_ablation.json").write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "mask cardinality"):
                fusion_mapping_ablation.load(run)

    def test_reject_mismatched_geometry_or_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = pathlib.Path(tmp) / "run"
            payload = self.fixture(run)
            payload["variants"][0]["wall_tolerance_scores"][1]["f1"] = 0.001
            (run / "mapping_ablation.json").write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "nonmonotone"):
                fusion_mapping_ablation.load(run)
            payload["variants"][0]["wall_tolerance_scores"][1]["f1"] = 2 * .4 / 3
            payload["variants"][0]["end_frame_exclusive"] = 194
            (run / "mapping_ablation.json").write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "window"):
                fusion_mapping_ablation.load(run)


if __name__ == "__main__":
    unittest.main()
