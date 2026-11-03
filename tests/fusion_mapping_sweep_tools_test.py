#!/usr/bin/env python3
"""No native solver needed: validate seven-scene score aggregation and guardrails."""
from __future__ import annotations

import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import fusion_mapping_sweep as sweep
from fusion_mapping_ablation_tools_test import MappingAblationToolsTest


class SevenSceneSweepTest(unittest.TestCase):
    def populated(self, root: Path, prefix="mapping-eval-001", *,
                  grid=False):
        fixture = MappingAblationToolsTest()
        for scene in sweep.SCENES:
            fixture.fixture(root / f"{prefix}-{scene}",
                            scene=scene, region=True, grid=grid)
        return prefix

    def test_all_scenes_and_region_csv_are_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            prefix=self.populated(root)
            full_csv=root/"full.csv"
            region_csv=root/"region.csv"
            stdout=io.StringIO()
            with contextlib.redirect_stdout(stdout):
                sweep.summarize(root,prefix,full_csv,region_csv)
            self.assertIn("Seven-scene LiDAR-loop detector",stdout.getvalue())
            self.assertIn("Cross-scene macro averages",stdout.getvalue())
            self.assertIn("No evidence setting is automatically promoted",
                          stdout.getvalue())
            with full_csv.open() as f:
                rows=list(csv.DictReader(f))
            with region_csv.open() as f:
                regional=list(csv.DictReader(f))
            self.assertEqual(len(rows),7*4)
            self.assertEqual(len(regional),7*3*4)
            self.assertEqual({r["scene"] for r in rows},set(sweep.SCENES))
            self.assertEqual({r["evidence"] for r in rows},set(sweep.EVIDENCE))
            self.assertEqual({r["window"] for r in regional},set(sweep.WINDOWS))
            self.assertTrue(all(float(r["mask_fraction"])>0 for r in regional))
            self.assertTrue(all(r["accepted_loops"]=="2" for r in rows))

    def test_nine_settings_ranked_and_penalty_guardrails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            prefix=self.populated(root, grid=True)
            full_csv=root/"nine-full.csv"
            regional_csv=root/"nine-region.csv"
            ranking_csv=root/"nine-ranking.csv"
            stdout=io.StringIO()
            with contextlib.redirect_stdout(stdout):
                sweep.summarize(root,prefix,full_csv,regional_csv,ranking_csv)
            self.assertIn("full 3x3",stdout.getvalue())
            self.assertIn("worst ΔP",stdout.getvalue())
            self.assertIn("5% gate",stdout.getvalue())
            with full_csv.open() as f:
                rows=list(csv.DictReader(f))
            with regional_csv.open() as f:
                regional=list(csv.DictReader(f))
            with ranking_csv.open() as f:
                rankings=list(csv.DictReader(f))
            self.assertEqual(len(rows),7*9)
            self.assertEqual(len(regional),7*3*9)
            self.assertEqual(len(rankings),9)
            self.assertEqual({r["evidence"] for r in rankings},
                             set(sweep.EVIDENCE))
            self.assertEqual(
                {float(r["hit"]) for r in rankings}, {.85,1.0,1.2})
            self.assertEqual(
                {float(r["free"]) for r in rankings}, {-.4,-.3,-.2})
            self.assertTrue(all(
                r["precision_guardrail_5pct"] in ("PASS","FAIL","UNDEFINED")
                for r in rankings))
            numeric_iou=[float(r["mean_iou"]) for r in rankings]
            self.assertEqual(numeric_iou,sorted(numeric_iou,reverse=True))
            # Independent controlled ranking example: higher IoU is not
            # automatically "eligible" when a scene's precision worsens.
            full,_=sweep.collect(root,prefix)
            for row in full:
                if row["evidence"]=="combined":
                    row["iou"]=.3
                    row["precision"]=.12
                elif row["evidence"]=="h100_f030":
                    row["iou"]=.2
                    row["precision"]=.157
            checked=sweep.rank_evidence(full)
            self.assertEqual(checked[0]["evidence"],"combined")
            self.assertEqual(checked[0]["precision_guardrail_5pct"],"FAIL")
            self.assertEqual(checked[0]["precision_loss_gt_5pct_scenes"],7)
            compromise=next(x for x in checked if x["evidence"]=="h100_f030")
            self.assertEqual(compromise["precision_guardrail_5pct"],"PASS")
            self.assertEqual(compromise["precision_loss_gt_5pct_scenes"],0)

    def test_reject_mixed_legacy_and_new_scene_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            import shutil
            root=Path(tmp)
            prefix=self.populated(root,grid=True)
            target=root/f"{prefix}-industrial"
            shutil.rmtree(target)
            MappingAblationToolsTest().fixture(
                target,scene="industrial",region=True,grid=False)
            with self.assertRaisesRegex(ValueError,"mixed legacy"):
                sweep.collect(root,prefix)

    def test_refuse_partial_or_changed_detector(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            prefix=self.populated(root)
            broken=root/f"{prefix}-atrium"/"fusion_report.json"
            report=json.loads(broken.read_text())
            report["match_stride"]=2
            broken.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,"unexpected sweep configuration"):
                sweep.collect(root,prefix)
            report["match_stride"]=1
            report["lidar_loop_closure"]=False
            broken.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,"LiDAR-loop GTSAM run"):
                sweep.collect(root,prefix)

    def test_run_is_local_explicit_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            binary=root/"slam_offline_fusion"
            binary.write_text("fake binary")
            with mock.patch.object(sweep.subprocess,"run") as native:
                sweep.run_all(binary,root,"test-v1")
            self.assertEqual(native.call_count,7)
            for call in native.call_args_list:
                argv=call.args[0]
                self.assertIn("--mapping-ablation",argv)
                self.assertIn("--lidar-loop-closure",argv)
                self.assertIn("--continuous-cost",argv)
                self.assertTrue(call.kwargs["check"])
            (root/"test-v1-apartment").mkdir()
            with mock.patch.object(sweep.subprocess,"run") as native:
                with self.assertRaises(FileExistsError):
                    sweep.run_all(binary,root,"test-v1")
                native.assert_not_called()
            with self.assertRaises(ValueError):
                sweep.checked_prefix("../unsafe")


if __name__=="__main__":
    unittest.main()
