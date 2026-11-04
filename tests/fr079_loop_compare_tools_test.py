#!/usr/bin/env python3
"""No native deps: enforce exact FR079 matcher pairing for loop scorecards."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fr079_loop_compare import compare


class Fr079LoopPairingTest(unittest.TestCase):
    def make(self, root):
        base=root/"noloop"
        loop=root/"loop"
        loop.mkdir()
        sha="a"*64
        report={
            "format":"FUSFR079SLAM1","source_sha256":sha,
            "matcher":"ceres_continuous_cost","distance_truncation_m":.5,
            "ablation_mode":"none","match_every":5,"field_rebuild_every":15,
            "map_beam_stride":1,"match_beam_stride":4,"match_beam_phase":0,
            "frames":3,"attempts":1,"accepted":1,"rejected":0,
            "inserted_scans":3,"map":{"fingerprint":"55","revision":3},
            "loop_closures":0,"pose_graph_optimized":False,
        }
        pose={
            "format":"FUSFR079POSE1","source_sha256":sha,
            "poses":[{"estimated":[float(i),0,0]} for i in range(3)]}
        for prefix in (base,loop/"loop"):
            Path(str(prefix)+".trajectory.json").write_text(json.dumps(pose))
        Path(str(base)+".report.json").write_text(json.dumps(report))
        report["pose_graph_optimized"]=True
        report["loop_mode"]="fr079_lidar_scan_registration_gtsam_v1"
        report["loop_closures"]=1
        report["loop_candidates"]=1
        report["graph_control_map"]={"fingerprint":"100","revision":3}
        report["graph_optimized_map"]={"fingerprint":"101","revision":3}
        Path(str(loop/"loop")+".report.json").write_text(json.dumps(report))
        graph={
            "format":"FUSFR079LOOP1","source_sha256":sha,
            "independent_ground_truth":False,"first_pose_fixed":True,
            "loop_evidence":"correlated_raw_lidar_reciprocal_scan_registration",
            "proposed_loops":1,"accepted_loops":1,
            "nearby_pairs":2,"registration_attempts":2,
            "geometry_verified":1,
            "loop_decisions":[{"from":0,"to":2,"decision":"accepted"}],
            "odometry_graph_poses":[[float(i),0,0] for i in range(3)],
            "loop_graph_poses":[[float(i),0,0] for i in range(3)],
            "maximum_graph_shift_m":.1,"mean_graph_shift_m":.05,
            "maximum_graph_shift_yaw_rad":.02,
            "graph_control_initial_cost":20,"graph_control_final_cost":10,
            "graph_loop_initial_cost":30,"graph_loop_final_cost":12,
            "graph_shift_within_safety_limit":True,
        }
        (loop/"loop.loops.json").write_text(json.dumps(graph))
        return base,loop

    def test_paired_graph_and_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            base,loop=self.make(Path(temp))
            row=compare(base,loop)
            self.assertEqual(row["comparison_status"],
                             "LOOPS_ACCEPTED_REVIEW_MAPS")
            self.assertTrue(row["frontend_trajectory_identical"])
            self.assertFalse(row["map_accuracy_measured"])
            self.assertEqual(row["loops_accepted"],1)

    def test_refuses_changing_matching_map_and_source(self):
        with tempfile.TemporaryDirectory() as temp:
            base,loop=self.make(Path(temp))
            file=loop/"loop.report.json"
            original=json.loads(file.read_text())
            for changed, error in (
                ({"match_every":7},"setting or count changed"),
                ({"map":{"fingerprint":"56","revision":3}},
                 "frontend map changed"),
                ({"source_sha256":"b"*64},"source hash mismatch"),
            ):
                file.write_text(json.dumps({**original,**changed}))
                with self.assertRaisesRegex(ValueError,error):
                    compare(base,loop)
            file.write_text(json.dumps(original))
            poses=loop/"loop.trajectory.json"
            obj=json.loads(poses.read_text())
            obj["poses"][2]["estimated"][0]+=2
            poses.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError,"frontend trajectory changed"):
                compare(base,loop)

    def test_rejects_unverified_provenance_and_marks_zero_loops(self):
        with tempfile.TemporaryDirectory() as temp:
            base,loop=self.make(Path(temp))
            graph_file=loop/"loop.loops.json"
            obj=json.loads(graph_file.read_text())
            obj["independent_ground_truth"]=True
            graph_file.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError,"graph provenance"):
                compare(base,loop)
            obj["independent_ground_truth"]=False
            obj["accepted_loops"]=0
            obj["proposed_loops"]=0
            obj["loop_decisions"]=[]
            graph_file.write_text(json.dumps(obj))
            report_file=loop/"loop.report.json"
            report=json.loads(report_file.read_text())
            report["loop_closures"]=0
            report["loop_candidates"]=0
            report_file.write_text(json.dumps(report))
            self.assertEqual(compare(base,loop)["comparison_status"],
                             "NO_ACCEPTED_LOOP_CLOSURES")


if __name__ == "__main__":
    unittest.main()
