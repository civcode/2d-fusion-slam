#!/usr/bin/env python3
"""Evaluate FR079 Ceres vs Ceres+GTSAM loops against fr079-sm.log.gz.

This is agreement with the published scan-matched estimate, NOT surveyed
ground-truth trajectory or physical map accuracy. The reference is read only
AFTER the two SLAM estimates have been produced.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from fr079_reference_eval_lib import (
    pose, wrap, relative, laser_records, verify_reference_scans,
    trajectory_metrics, occupied_endpoints, map_overlap, improvement,
)
from review_artifacts import review_files

SETTINGS = ("matcher","distance_truncation_m","ablation_mode",
            "match_every","field_rebuild_every","map_beam_stride",
            "match_beam_stride","match_beam_phase")
COUNTS = ("frames","attempts","accepted","rejected","inserted_scans")
FRONTEND_SOURCE = "frontend_scanmatched_relative_correlated_v1"
LOOP_MODE = "fr079_frontend_relative_gtsam_loops_v2"


def json_at(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def run_prefix(path: Path) -> Path:
    path = Path(path)
    return path / path.name if path.is_dir() else path


def evaluate(raw_log: Path, reference_sm_log: Path, baseline: Path,
             optimized: Path, *, resolution: float = .10,
             beam_stride: int = 4, max_range: float = 18.) -> dict:
    if not (.02 <= resolution <= .50 and 1 <= beam_stride <= 64 and
            1. <= max_range <= 40.):
        raise ValueError("invalid map proxy resolution/beam stride/max range")
    a, b = run_prefix(baseline),run_prefix(optimized)
    br = json_at(Path(str(a)+".report.json"))
    lr = json_at(Path(str(b)+".report.json"))
    bp = json_at(Path(str(a)+".trajectory.json"))
    lp = json_at(Path(str(b)+".trajectory.json"))
    g = json_at(Path(str(b)+".loops.json"))
    if br.get("format") != "FUSFR079SLAM1" or lr.get("format") != "FUSFR079SLAM1":
        raise ValueError("benchmark expects FUSFR079SLAM1 reports")
    if bp.get("format") != "FUSFR079POSE1" or lp.get("format") != "FUSFR079POSE1":
        raise ValueError("benchmark expects FUSFR079POSE1 pose timelines")
    if g.get("format") != "FUSFR079LOOP1":
        raise ValueError("benchmark expects FUSFR079LOOP1 graph diagnostics")
    if (br.get("pose_graph_optimized") is not False or br.get("loop_closures") != 0 or
            lr.get("pose_graph_optimized") is not True or
            lr.get("loop_mode") != LOOP_MODE or
            lr.get("sequential_factor_source") != FRONTEND_SOURCE or
            g.get("sequential_factor_source") != FRONTEND_SOURCE or
            g.get("independent_ground_truth") is not False or
            g.get("first_pose_fixed") is not True or
            lr.get("independent_ground_truth") is not False):
        raise ValueError("requires original Ceres baseline and new correlated-frontend GTSAM graph")
    if not br.get("matcher","").startswith("ceres"):
        raise ValueError("requires real Ceres matching, not geometry fallback")
    if g.get("accepted_loops") != lr.get("loop_closures"):
        raise ValueError("loop closure count differs between graph and report")
    for field in SETTINGS+COUNTS:
        if br.get(field) != lr.get(field):
            raise ValueError(f"frontend configuration or count changed: {field}")
    if br["map"]["fingerprint"] != lr["map"]["fingerprint"]:
        raise ValueError("Ceres frontend occupancy map fingerprint changed")
    if bp["poses"] != lp["poses"]:
        raise ValueError("Ceres frontend pose sequence changed")

    raw, raw_hash = laser_records(raw_log)
    reference_scans, ref_hash = laser_records(reference_sm_log)
    if raw_hash == ref_hash:
        raise ValueError("same raw and reference file; reference is not independent")
    for source in (br.get("source_sha256"),lr.get("source_sha256"),
                   bp.get("source_sha256"),lp.get("source_sha256"),
                   g.get("source_sha256")):
        if source != raw_hash:
            raise ValueError("raw log SHA-256 mismatch with SLAM artifacts")
    n = len(raw)
    if any(size != n for size in (br["frames"],lr["frames"],
               len(bp["poses"]),len(lp["poses"]),len(g["graph_control_poses"]),
               len(g["loop_graph_poses"]))):
        raise ValueError("scan and pose timeline counts do not match")
    left_match = json_at(Path(str(a)+".matches.json"))
    right_match = json_at(Path(str(b)+".matches.json"))
    durations = ("field_ms","match_ms","insertion_ms","frame_ms")
    def untimed(m):
        return [{k:v for k,v in row.items() if k not in durations}
                for row in m["frames"]]
    if (left_match.get("format") != "FUSFR079MATCH1" or
            right_match.get("format") != "FUSFR079MATCH1" or
            left_match.get("source_sha256") != raw_hash or
            right_match.get("source_sha256") != raw_hash or
            len(left_match["frames"]) != n or len(right_match["frames"]) != n or
            untimed(left_match) != untimed(right_match)):
        raise ValueError("frontend matching decision audit does not match")

    verify_reference_scans(raw, reference_scans,
                           [record["t"] for record in bp["poses"]])
    frontend = [pose(p["estimated"],"Ceres estimate") for p in bp["poses"]]
    sequential = [pose(p,"frontend graph control")
                  for p in g["graph_control_poses"]]
    loop = [pose(p,"GTSAM loop estimate") for p in g["loop_graph_poses"]]
    if any(math.hypot(x[0]-y[0],x[1]-y[1]) > 1e-4 or
           abs(wrap(x[2]-y[2])) > 1e-4
           for x,y in zip(frontend,sequential)):
        raise ValueError("no-loop GTSAM graph does not preserve Ceres frontend")
    # Reference comes only from the scan-matched recording. Never the raw log's
    # 'laser pose' fields, the logged odometry, or a known map used in SLAM.
    reference = [relative(reference_scans[0]["laser"],p["laser"])
                 for p in reference_scans]

    report = {
        "format":"FUSFR079REFERENCEBENCH1",
        "raw_source_sha256":raw_hash,
        "reference_source_sha256":ref_hash,
        "reference_kind":"fr079_sm_log_scanmatched_logged_laser_pose_estimate",
        "reference_used_in_estimation":False,
        "independent_ground_truth":False,
        "frames":n,"accepted_loops":g["accepted_loops"],
        "sequential_factor_source":g["sequential_factor_source"],
        "scan_pairing":"same_index_and_timestamp_and_all_lidar_ranges_verified",
        "note":"Reference shares LiDAR scans with estimators. Scores measure agreement with estimated scan-matched poses, NOT surveyed truth. Occupied-endpoint map score is a reference-derived raster proxy, not physical map error.",
        "trajectory":{},
        "occupied_endpoint_map":{
            "method":"same_raw_lidar_hits_rasterized_at_reference_or_estimated_poses",
            "reference_map_archive_used":False,
            "is_occupancy_grid_ground_truth":False,
            "resolution_m":resolution,
            "beam_stride":beam_stride,
            "hits_required_per_cell":2,
        },
    }
    # Project the same unmodified raw hits into a shared reference coordinate
    # frame. Both estimators receive precisely the same observations.
    reference_hits = occupied_endpoints(raw,reference,resolution,beam_stride,max_range)
    for alignment in ("first_pose","rigid_xy"):
        cr = trajectory_metrics(reference,frontend,align=alignment)
        gr = trajectory_metrics(reference,loop,align=alignment)
        ceres_aligned, graph_aligned = cr.pop("aligned_poses"),gr.pop("aligned_poses")
        ate_before,ate_after = cr["ate_translation_m"]["rmse"],gr["ate_translation_m"]["rmse"]
        arm = {
            "ceres":cr,"ceres_gtsam_loops":gr,
            "ate_rmse_change_m":improvement(ate_before,ate_after),
        }
        if n > 10:
            arm["rpe_10_rmse_change_m"] = improvement(
                cr["rpe_scan_intervals"]["10"]["translation_m"]["rmse"],
                gr["rpe_scan_intervals"]["10"]["translation_m"]["rmse"])
        report["trajectory"][alignment] = arm
        ch = occupied_endpoints(raw,ceres_aligned,resolution,beam_stride,max_range)
        gh = occupied_endpoints(raw,graph_aligned,resolution,beam_stride,max_range)
        cm = map_overlap(reference_hits,ch)
        gm = map_overlap(reference_hits,gh)
        report["occupied_endpoint_map"][alignment] = {
            "ceres":cm,"ceres_gtsam_loops":gm,
            "strict_iou_gain":gm["strict_occupied_endpoint_iou"] -
                              cm["strict_occupied_endpoint_iou"],
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-log",type=Path,required=True)
    parser.add_argument("--reference-sm-log",type=Path,required=True)
    parser.add_argument("--baseline",type=Path,required=True)
    parser.add_argument("--optimized",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--map-resolution",type=float,default=.1)
    parser.add_argument("--map-beam-stride",type=int,default=4)
    args = parser.parse_args()
    report = evaluate(args.raw_log,args.reference_sm_log,
                      args.baseline,args.optimized,
                      resolution=args.map_resolution,
                      beam_stride=args.map_beam_stride)
    json_path = Path(str(args.out)+".json")
    csv_path = Path(str(args.out)+".csv")
    review_path = Path(str(args.out)+".review.json")
    if any(p.exists() for p in (json_path,csv_path,review_path)):
        raise FileExistsError("benchmark output exists; choose a new prefix")
    json_path.parent.mkdir(parents=True,exist_ok=True)
    with json_path.open("x",encoding="utf-8") as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write("\n")

    row = {"frames":report["frames"],
           "accepted_loops":report["accepted_loops"],
           "reference_is_ground_truth":False}
    for frame in ("first_pose","rigid_xy"):
        block = report["trajectory"][frame]
        for arm in ("ceres","ceres_gtsam_loops"):
            for metric in ("ate_translation_m","absolute_yaw_deg"):
                for stat in ("rmse","median","p95","max"):
                    row[f"{frame}_{arm}_{metric}_{stat}"] = block[arm][metric][stat]
        row[f"{frame}_ate_rmse_reduction_percent"] = block[
            "ate_rmse_change_m"]["percent_reduction"]
        for arm in ("ceres","ceres_gtsam_loops"):
            map_score = report["occupied_endpoint_map"][frame][arm]
            for metric in ("strict_occupied_endpoint_iou",
                           "tolerant_precision","tolerant_recall"):
                row[f"{frame}_{arm}_{metric}"] = map_score[metric]
    with csv_path.open("x",encoding="utf-8",newline="") as stream:
        writer = csv.DictWriter(stream,fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    review_files(review_path,"fr079_scanmatched_reference_benchmark",[
        ("reference_benchmark_json",json_path,True),
        ("reference_benchmark_csv",csv_path,True)])
    print("FR079 scan-matched ESTIMATED-reference comparison:")
    for frame,block in report["trajectory"].items():
        print(f"{frame}: Ceres ATE RMSE={block['ceres']['ate_translation_m']['rmse']:.3f} m, "
              f"Ceres+loops={block['ceres_gtsam_loops']['ate_translation_m']['rmse']:.3f} m")
    print("Reference is NOT independently surveyed ground truth.")


if __name__ == "__main__":
    main()
