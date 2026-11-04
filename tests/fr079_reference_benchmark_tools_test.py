#!/usr/bin/env python3
"""Pure Python FR079 scan-matched-reference benchmark regression tests."""
from __future__ import annotations

import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from fr079_reference_eval_lib import (compose,relative,rigid_xy_align,
    trajectory_metrics,stats,laser_records)
from fr079_reference_benchmark import evaluate


class ReferenceBenchmarkTest(unittest.TestCase):
    def fixture(self,root:Path):
        raw,sm = root/"raw.log.gz",root/"scanmatched.log.gz"
        n=22
        raw_rows,ref_rows = [],[]
        for i in range(n):
            stamp=1710000000.+i*.23
            ranges=["2.000"]*360
            recorded_odometry=(.1*i,0.,.02*i)
            reference_pose=(.1*i,0.,.02*i)
            def line(laser_pose):
                return ("FLASER 360 "+" ".join(ranges)+" "+
                    " ".join(str(v) for v in (*laser_pose,*recorded_odometry))+
                    f" {stamp:.6f} testhost {stamp:.6f}")
            raw_rows.append(line((999.,999.,0.))) # must NOT be evaluator truth
            ref_rows.append(line(reference_pose))
        rawbytes=("\n".join(raw_rows)+"\n").encode()
        refbytes=("\n".join(ref_rows)+"\n").encode()
        with gzip.open(raw,"wb") as handle: handle.write(rawbytes)
        with gzip.open(sm,"wb") as handle: handle.write(refbytes)
        sha=hashlib.sha256(rawbytes).hexdigest()
        baseline=root/"noloop"
        optimized=root/"optimized"
        optimized.mkdir()
        frontend=[];controls=[];loops=[];matches=[]
        for i in range(n):
            stamp=1710000000.+i*.23
            initial=[.13*i,0.,.025*i]
            improved=[.11*i,0.,.021*i]
            frontend.append({"index":i,"t":stamp,"odom":[.1*i,0.,.02*i],
                             "prior":initial,"estimated":initial})
            controls.append(initial)
            loops.append(improved)
            matches.append({"index":i,"accepted":False,
                            "status":"not_attempted","field_ms":i*.3})
        report={
            "format":"FUSFR079SLAM1","source_sha256":sha,
            "frames":n,"attempts":0,"accepted":0,"rejected":0,
            "inserted_scans":n,"pose_graph_optimized":False,
            "loop_closures":0,"independent_ground_truth":False,
            "matcher":"ceres_continuous_cost","distance_truncation_m":.5,
            "ablation_mode":"none","match_every":5,"field_rebuild_every":15,
            "map_beam_stride":1,"match_beam_stride":4,"match_beam_phase":0,
            "map":{"fingerprint":"123","revision":n},
        }
        def save(prefix,suffix,obj):
            Path(str(prefix)+suffix).write_text(json.dumps(obj))
        for prefix in (baseline,optimized/optimized.name):
            save(prefix,".trajectory.json",{
                "format":"FUSFR079POSE1","source_sha256":sha,"poses":frontend})
            save(prefix,".matches.json",{
                "format":"FUSFR079MATCH1","source_sha256":sha,"frames":matches})
        save(baseline,".report.json",report)
        report=dict(report,pose_graph_optimized=True,loop_closures=2,
            loop_mode="fr079_frontend_relative_gtsam_loops_v2",
            sequential_factor_source="frontend_scanmatched_relative_correlated_v1")
        save(optimized/optimized.name,".report.json",report)
        save(optimized/optimized.name,".loops.json",{
            "format":"FUSFR079LOOP1","source_sha256":sha,
            "independent_ground_truth":False,"first_pose_fixed":True,
            "sequential_factor_source":"frontend_scanmatched_relative_correlated_v1",
            "accepted_loops":2,
            "graph_control_poses":controls,"loop_graph_poses":loops,
        })
        return raw,sm,baseline,optimized

    def test_se2_and_yaw_wrapping(self):
        a=(1.,2.,math.pi-.01)
        b=(2.,3.,-math.pi+.01)
        self.assertAlmostEqual(relative(a,b)[2],.02)
        self.assertAlmostEqual(compose(a,relative(a,b))[0],2.)
        ref=[(1.,2.,0.),(2.,2.,0.),(3.,2.,0.)]
        est=[(4.,5.,.2),(5.,5.,.2),(6.,5.,.2)]
        aligned=rigid_xy_align(ref,est)
        self.assertLess(max(math.hypot(p[0]-q[0],p[1]-q[1])
                            for p,q in zip(ref,aligned)),1e-9)
        self.assertLess(trajectory_metrics(ref,est,align="rigid_xy")[
            "ate_translation_m"]["rmse"],1e-9)
        self.assertAlmostEqual(stats([1.,2.,3.])["p95"],2.9)

    def test_paired_estimates_improve_correlated_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            raw,sm,base,opt=self.fixture(Path(tmp))
            result=evaluate(raw,sm,base,opt,beam_stride=8,
                            verify_published_reference=False)
            self.assertFalse(result["reference_archive_published_md5_verified"])
            self.assertEqual(result["frames"],22)
            self.assertFalse(result["independent_ground_truth"])
            self.assertFalse(result["reference_used_in_estimation"])
            self.assertFalse(result["occupied_endpoint_map"]["reference_map_archive_used"])
            for key in ("first_pose","rigid_xy"):
                traj=result["trajectory"][key]
                self.assertLess(traj["ceres_gtsam_loops"]["ate_translation_m"]["rmse"],
                                traj["ceres"]["ate_translation_m"]["rmse"])
                self.assertGreater(traj["ate_rmse_change_m"]["percent_reduction"],0)
                self.assertGreater(result["occupied_endpoint_map"][key]["ceres"][
                    "candidate_occupied_endpoint_cells"],0)
                self.assertGreater(result["occupied_endpoint_map"][key]["ceres_gtsam_loops"][
                    "candidate_occupied_endpoint_cells"],0)

    def test_refuse_wrong_graph_and_mismatched_scans(self):
        with tempfile.TemporaryDirectory() as tmp:
            raw,sm,base,opt=self.fixture(Path(tmp))
            with self.assertRaisesRegex(ValueError,"published FR079 MD5"):
                evaluate(raw,sm,base,opt)
            report=opt/(opt.name+".report.json")
            obj=json.loads(report.read_text())
            obj["sequential_factor_source"]="logged_odometry_scan_time_v1"
            report.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError,"requires original Ceres"):
                evaluate(raw,sm,base,opt,verify_published_reference=False)
            obj["sequential_factor_source"]="frontend_scanmatched_relative_correlated_v1"
            report.write_text(json.dumps(obj))
            different=gzip.open(sm,"rb").read().replace(b"2.000",b"2.100",1)
            with gzip.open(sm,"wb") as stream:stream.write(different)
            with self.assertRaisesRegex(ValueError,"LiDAR ranges differ"):
                evaluate(raw,sm,base,opt,verify_published_reference=False)
            with self.assertRaisesRegex(ValueError,"same raw and reference"):
                evaluate(raw,raw,base,opt,verify_published_reference=False)

    def test_refuse_different_ceres_control_and_matches(self):
        with tempfile.TemporaryDirectory() as tmp:
            raw,sm,base,opt=self.fixture(Path(tmp))
            graph=opt/(opt.name+".loops.json")
            obj=json.loads(graph.read_text())
            obj["graph_control_poses"][10][0]+=1.
            graph.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError,"does not preserve Ceres"):
                evaluate(raw,sm,base,opt,verify_published_reference=False)
            obj["graph_control_poses"][10][0]-=1.
            graph.write_text(json.dumps(obj))
            decisions=opt/(opt.name+".matches.json")
            rows=json.loads(decisions.read_text())
            rows["frames"][10]["accepted"]=True
            decisions.write_text(json.dumps(rows))
            with self.assertRaisesRegex(ValueError,"matching decision audit"):
                evaluate(raw,sm,base,opt,verify_published_reference=False)


if __name__=="__main__":
    unittest.main()
