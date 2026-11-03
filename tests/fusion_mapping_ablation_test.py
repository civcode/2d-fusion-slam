#!/usr/bin/env python3
"""End-to-end fixed-graph mapping ablation, separate from fast fusion smoke."""
from __future__ import annotations

import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest


EXE = sys.argv.pop() if len(sys.argv) > 1 else None


def run(scene, output, *flags):
    return subprocess.run([EXE, scene, str(output), *flags],
                          text=True, capture_output=True, timeout=480)


class MappingAblationIntegrationTest(unittest.TestCase):
    @unittest.skipIf(EXE is None, "provide slam_offline_fusion executable")
    def test_fixed_graph_24_map_reconstructions(self):
        with tempfile.TemporaryDirectory() as work:
            root=pathlib.Path(work)
            baseline=root/"baseline"
            experiment=root/"ablation"
            flags=("--lidar-loop-closure","--match-stride","1")
            base=run("apartment",baseline,*flags)
            if base.returncode and "GTSAM" in base.stderr:
                self.assertFalse(baseline.exists())
                self.skipTest("GTSAM offline graph not built")
            self.assertEqual(base.returncode,0,base.stderr)
            changed=run("apartment",experiment,*flags,"--mapping-ablation")
            self.assertEqual(changed.returncode,0,changed.stderr)
            old_report=json.loads((baseline/"fusion_report.json").read_text())
            new_report=json.loads((experiment/"fusion_report.json").read_text())
            old_poses=json.loads((baseline/"trajectories.json").read_text())
            new_poses=json.loads((experiment/"trajectories.json").read_text())
            self.assertEqual(old_report,new_report,
                             "mapping-only flag must not change any standard metric")
            self.assertEqual(old_poses,new_poses,
                             "mapping-only flag must not change estimated poses")
            for method in old_report["results"]:
                self.assertEqual(
                    old_report["results"][method]["map"]["map_fingerprint"],
                    new_report["results"][method]["map"]["map_fingerprint"])

            data=json.loads((experiment/"mapping_ablation.json").read_text())
            self.assertEqual(data["format"],"FUSMAPABL1")
            self.assertEqual(data["reference_usage"],"posthoc_scoring_only")
            self.assertTrue(data["original_trajectory_and_maps_unchanged"])
            self.assertEqual(len(data["variants"]),24)
            self.assertEqual(len({v["id"] for v in data["variants"]}),24)
            n=new_report["frames"]
            cut=n-math.ceil(n*.12)
            self.assertTrue(
                {v["deskew"] for v in data["variants"]} ==
                {"gyro_wheel_original","graph_corrected_motion_model"})
            self.assertTrue(
                {v["window"] for v in data["variants"]} ==
                {"full","early","late"})
            self.assertTrue(
                {v["evidence"] for v in data["variants"]} ==
                {"baseline","stronger_hit","gentler_free","combined"})
            masks_by_window={}
            self.assertEqual(data["region_padding_m"],.25)
            self.assertEqual(
                data["region_mask_source"],
                "original_deskew_baseline_ray_observation_padded")
            for v in data["variants"]:
                self.assertEqual(v["map"]["format"],"FUSMAPACC1")
                self.assertEqual(v["map"]["reference_scene"],new_report["scene"])
                if v["window"]=="full":
                    self.assertEqual((v["begin_frame"],v["end_frame_exclusive"]),(0,n))
                elif v["window"]=="early":
                    self.assertEqual((v["begin_frame"],v["end_frame_exclusive"]),(0,cut))
                else:
                    self.assertEqual((v["begin_frame"],v["end_frame_exclusive"]),(cut,n))
                self.assertEqual([round(b["tolerance_m"],2)
                                  for b in v["wall_tolerance_scores"]],
                                 [.10,.25,.50])
                for field in ("precision","recall","f1"):
                    values=[s[field] for s in v["wall_tolerance_scores"]]
                    self.assertTrue(all(0<=x<=1 for x in values))
                    self.assertEqual(values,sorted(values))
                footprint=v["region_footprint"]
                self.assertEqual(footprint["source"],"original_baseline_rays")
                self.assertEqual(footprint["padding_m"],.25)
                self.assertGreater(footprint["cells"],0)
                self.assertRegex(footprint["fingerprint"],r"^[0-9]+$")
                self.assertLessEqual(footprint["cells"],
                                     footprint["total_grid_cells"])
                self.assertEqual(masks_by_window.setdefault(v["window"],footprint),
                                 footprint,
                                 "evidence/deskew changes must use same spatial mask")
                local=v["region_map"]
                self.assertEqual(local["format"],"FUSMAPACC1")
                self.assertEqual(local["reference_scene"],new_report["scene"])
                self.assertEqual(local["map_fingerprint"],
                                 v["map"]["map_fingerprint"])
                self.assertEqual(
                    local["counts"]["known"]+local["counts"]["unknown"],
                    footprint["cells"])
                self.assertGreater(local["counts"]["truth_wall"],0)
                self.assertLessEqual(local["counts"]["truth_wall"],
                                     footprint["cells"])
                for value in ("precision","recall","f1"):
                    scores=[t[value] for t in v["region_wall_tolerance_scores"]]
                    self.assertEqual(scores,sorted(scores))
                self.assertEqual(
                    [round(t["tolerance_m"],2)
                     for t in v["region_wall_tolerance_scores"]],
                    [.10,.25,.50])
                artifact=experiment/v["map_path"]
                self.assertTrue(artifact.is_file(),str(artifact))
                self.assertGreater(artifact.stat().st_size,100)
            exact=next(v for v in data["variants"] if
                v["id"]=="full_gyro_wheel_original_baseline")
            self.assertEqual(
                exact["map"]["map_fingerprint"],
                new_report["results"]["graph_optimized"]["map"]["map_fingerprint"])
            self.assertEqual(
                exact["map"]["metrics"],
                new_report["results"]["graph_optimized"]["map"]["metrics"])
            self.assertFalse((baseline/"mapping_ablation.json").exists())



if __name__ == "__main__":
    unittest.main()
