#!/usr/bin/env python3
"""Compare FR079 no-loop vs GTSAM LiDAR-loop experiments without ground truth.

Refuse unrelated input logs, matching settings or frontend maps. Compare the
loop graph against the *same run's odometry-only GTSAM graph control*; never
present any graph-cost decrease or source-derived map agreement as ATE/IoU.
All files are read-only, output files are exclusive.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from review_artifacts import review_files


SETTINGS = ("matcher", "distance_truncation_m", "ablation_mode",
            "match_every", "field_rebuild_every", "map_beam_stride",
            "match_beam_stride", "match_beam_phase")
FRONTEND_COUNTS = ("frames", "attempts", "accepted", "rejected",
                   "inserted_scans")


def load(prefix: Path):
    prefix = Path(prefix)
    if prefix.is_dir():
        prefix = prefix / prefix.name
    return (json.loads(Path(str(prefix) + ".report.json").read_text()),
            json.loads(Path(str(prefix) + ".trajectory.json").read_text()))


def compare(base: Path, loop: Path) -> dict:
    b, bp = load(base)
    l, lp = load(loop)
    graph_prefix = loop / loop.name if loop.is_dir() else loop
    graph_file = Path(str(graph_prefix) + ".loops.json")
    g = json.loads(graph_file.read_text())
    if b.get("format") != "FUSFR079SLAM1" or l.get("format") != "FUSFR079SLAM1":
        raise ValueError("both runs require FUSFR079SLAM1 reports")
    if bp.get("format") != "FUSFR079POSE1" or lp.get("format") != "FUSFR079POSE1":
        raise ValueError("both runs require FUSFR079POSE1 pose traces")
    if g.get("format") != "FUSFR079LOOP1":
        raise ValueError("loop run requires FUSFR079LOOP1 diagnostics")
    if b.get("pose_graph_optimized") is not False or b.get("loop_closures") != 0:
        raise ValueError("baseline must be incremental SLAM without a pose graph")
    if l.get("pose_graph_optimized") is not True or l.get("loop_mode") != (
            "fr079_lidar_scan_registration_gtsam_v1"):
        raise ValueError("comparison run must explicitly enable GTSAM LiDAR loops")
    sha = b.get("source_sha256")
    if (not isinstance(sha, str) or len(sha) != 64 or
            any(source != sha for source in
                (l.get("source_sha256"), bp.get("source_sha256"),
                 lp.get("source_sha256"), g.get("source_sha256")))):
        raise ValueError("FR079 source hash mismatch between runs")
    for key in SETTINGS + FRONTEND_COUNTS:
        if b.get(key) != l.get(key):
            raise ValueError(f"frontend setting or count changed: {key}")
    if b["map"]["fingerprint"] != l["map"]["fingerprint"]:
        raise ValueError("incremental frontend map changed; cannot attribute result to loops")
    if bp["poses"] != lp["poses"]:
        raise ValueError("incremental frontend trajectory changed; run configurations not paired")
    base_prefix = base / base.name if base.is_dir() else base
    loop_prefix = loop / loop.name if loop.is_dir() else loop
    bm = json.loads(Path(str(base_prefix) + ".matches.json").read_text())
    lm = json.loads(Path(str(loop_prefix) + ".matches.json").read_text())
    if (bm.get("format") != "FUSFR079MATCH1" or
            lm.get("format") != "FUSFR079MATCH1" or
            bm.get("source_sha256") != sha or
            lm.get("source_sha256") != sha):
        raise ValueError("invalid frontend match audit provenance")
    if len(bm["frames"]) != l["frames"] or len(lm["frames"]) != l["frames"]:
        raise ValueError("frontend matching audit frame count mismatch")
    times = {"field_ms", "match_ms", "insertion_ms", "frame_ms"}
    def untimed(audit):
        return [{key:value for key,value in row.items() if key not in times}
                for row in audit["frames"]]
    if untimed(bm) != untimed(lm):
        raise ValueError("frontend matching decisions changed between runs")
    if (g.get("independent_ground_truth") is not False or
            g.get("loop_evidence") !=
                "correlated_raw_lidar_reciprocal_scan_registration" or
            g.get("first_pose_fixed") is not True):
        raise ValueError("invalid graph provenance or gauge")
    n = l["frames"]
    if len(g["odometry_graph_poses"]) != n or len(g["loop_graph_poses"]) != n:
        raise ValueError("graph pose count mismatch")
    if (g["accepted_loops"] != l["loop_closures"] or
            g["proposed_loops"] != l["loop_candidates"]):
        raise ValueError("graph loop counts disagree with report")
    if b["map"]["revision"] != l["map"]["revision"]:
        raise ValueError("incremental map revision changed")
    control, optimized = l.get("graph_control_map"), l.get("graph_optimized_map")
    if not isinstance(control, dict) or not isinstance(optimized, dict):
        raise ValueError("missing graph-control or optimized-map metadata")
    if any(not math.isfinite(float(v)) for v in
           (g["maximum_graph_shift_m"], g["mean_graph_shift_m"],
            g["maximum_graph_shift_yaw_rad"],
            g["graph_control_final_cost"], g["graph_loop_final_cost"])):
        raise ValueError("nonfinite graph diagnostic")
    if g["graph_shift_within_safety_limit"] is not True:
        status = "REVIEW_GRAPH_SHIFT_EXCEEDED_LIMIT"
    elif g["accepted_loops"] == 0:
        status = "NO_ACCEPTED_LOOP_CLOSURES"
    elif control["fingerprint"] == optimized["fingerprint"]:
        status = "ACCEPTED_LOOPS_BUT_SAME_GRAPH_MAP"
    else:
        status = "LOOPS_ACCEPTED_REVIEW_MAPS"
    return {
        "format": "FUSFR079LOOPCOMPARE1",
        "source_sha256": sha,
        "independent_ground_truth": False,
        "readme": ("Control is the GTSAM graph with logged odometry only. "
                   "Loop arm adds correlated reciprocal LiDAR factors. "
                   "Neither graph objective nor map fingerprint measures physical accuracy."),
        "comparison_status": status,
        "frontend_trajectory_identical": True,
        "frontend_matching_audit_identical": True,
        "frontend_map_identical": True,
        "frontend_matched": l["accepted"],
        "frontend_attempts": l["attempts"],
        "frames": n,
        "baseline_incremental_compute_ms": b.get("profiling", {}).get("slam_compute_ms"),
        "loop_run_compute_including_graph_ms": l.get("profiling", {}).get("slam_compute_ms"),
        "graph_compute_ms": l.get("profiling", {}).get("graph_compute_ms"),
        "loop_nearby_pairs": g["nearby_pairs"],
        "loop_registrations": g["registration_attempts"],
        "loop_geometry_verified": g["geometry_verified"],
        "loops_proposed": g["proposed_loops"],
        "loops_accepted": g["accepted_loops"],
        "loop_rejected_events": len([d for d in g["loop_decisions"]
                                     if d["decision"] != "accepted"]),
        "baseline_frontend_map_fingerprint": b["map"]["fingerprint"],
        "loop_run_frontend_map_fingerprint": l["map"]["fingerprint"],
        "graph_odometry_control_fingerprint": control["fingerprint"],
        "graph_lidar_loops_fingerprint": optimized["fingerprint"],
        "maximum_loop_vs_control_shift_m": g["maximum_graph_shift_m"],
        "mean_loop_vs_control_shift_m": g["mean_graph_shift_m"],
        "maximum_loop_vs_control_yaw_rad": g["maximum_graph_shift_yaw_rad"],
        "graph_control_initial_cost": g["graph_control_initial_cost"],
        "graph_control_final_cost": g["graph_control_final_cost"],
        "graph_loop_initial_cost": g["graph_loop_initial_cost"],
        "graph_loop_final_cost": g["graph_loop_final_cost"],
        "map_accuracy_measured": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True,
                        help="flat prefix or FR079 run folder")
    parser.add_argument("--loop", type=Path, required=True,
                        help="flat prefix or FR079 loop run folder")
    parser.add_argument("--out", type=Path, required=True,
                        help="new report prefix, usually reports/fr079-...-comparison")
    args = parser.parse_args()
    report = compare(args.baseline, args.loop)
    result_path = Path(str(args.out) + ".json")
    csv_path = Path(str(args.out) + ".csv")
    manifest = Path(str(args.out) + ".review.json")
    if any(p.exists() for p in (result_path,csv_path,manifest)):
        raise FileExistsError("comparison output exists; use a new prefix")
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(report, indent=2) + "\n")
    with csv_path.open("x", newline="") as handle:
        row = {key:value for key,value in report.items()
               if key != "readme" and not isinstance(value, (dict,list))}
        writer = csv.DictWriter(handle, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    review_files(manifest, "fr079_loop_comparison", [
        ("comparison_json", result_path, True),
        ("comparison_csv", csv_path, True),
    ])
    print("Status:", report["comparison_status"])
    print("Accepted loop closures:", report["loops_accepted"])
    print("Maximum loop-versus-control trajectory shift (m):",
          report["maximum_loop_vs_control_shift_m"])
    print("No independent trajectory/map accuracy has been established.")


if __name__ == "__main__":
    main()
