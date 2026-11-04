#!/usr/bin/env python3
"""Compare FR079 no-loop vs GTSAM LiDAR-loop experiments without ground truth.

Refuse unrelated input logs, matching settings or frontend maps. Report
three separately labeled trajectory differences in the shared first-scan
gauge (Ceres-to-loop, odometry-graph-to-loop, and Ceres-to-odometry).
Never present graph-cost decrease or correlated map agreement as ATE/IoU.
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



def _quantile(sorted_values: list[float], fraction: float) -> float:
    """Linearly interpolate between sorted sample positions, incl. both ends."""
    if not sorted_values:
        raise ValueError("trajectory comparison contains no poses")
    index = fraction * (len(sorted_values) - 1)
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (index - lo)


def trajectory_differences(reference: list, candidate: list) -> dict:
    """Absolute same-index pose changes in their shared first-scan SE(2) gauge.

    These are paired estimate-to-estimate displacements, NOT physical
    positioning errors or independent trajectory accuracy measurements.
    """
    if not reference or len(reference) != len(candidate):
        raise ValueError("trajectory pose count mismatch or empty trajectory")
    position, heading = [], []
    for idx, (a, b) in enumerate(zip(reference, candidate)):
        if (not isinstance(a, (list, tuple)) or
                not isinstance(b, (list, tuple)) or
                len(a) != 3 or len(b) != 3 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in (*a, *b))):
            raise ValueError(f"nonfinite or malformed SE(2) pose at scan {idx}")
        position.append(math.hypot(b[0] - a[0], b[1] - a[1]))
        heading.append(abs(math.remainder(b[2] - a[2], 2 * math.pi)) *
                       180 / math.pi)
    def describe(values: list[float]) -> dict:
        ordered = sorted(values)
        return {
            "mean": math.fsum(values) / len(values),
            "median": _quantile(ordered, .50),
            "p95": _quantile(ordered, .95),
            "max": max(values),
            "max_scan_index": max(range(len(values)), key=values.__getitem__),
        }
    return {
        "samples": len(position),
        "translation_m": describe(position),
        "yaw_deg": describe(heading),
    }


def scorecard_csv_row(report: dict) -> dict:
    """Keep existing scalar fields; add unambiguous metrics for both arms."""
    row = {key: value for key, value in report.items()
           if key != "readme" and not isinstance(value, (dict, list))}
    for comparison, summary in report["trajectory_comparisons"].items():
        for dimension in ("translation_m", "yaw_deg"):
            for stat, value in summary[dimension].items():
                row[f"{comparison}_{dimension}_{stat}"] = value
    return row


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
    frontend_poses = [p["estimated"] for p in bp["poses"]]
    odom_poses = g["odometry_graph_poses"]
    loop_poses = g["loop_graph_poses"]
    comparisons = {
        "ceres_frontend_to_lidar_loop": trajectory_differences(
            frontend_poses, loop_poses),
        "odometry_graph_to_lidar_loop": trajectory_differences(
            odom_poses, loop_poses),
        "ceres_frontend_to_odometry_graph": trajectory_differences(
            frontend_poses, odom_poses),
    }
    odom_loop = comparisons["odometry_graph_to_lidar_loop"]
    if (not math.isclose(odom_loop["translation_m"]["max"],
                         g["maximum_graph_shift_m"], abs_tol=1e-7) or
            not math.isclose(odom_loop["translation_m"]["mean"],
                             g["mean_graph_shift_m"], abs_tol=1e-7) or
            not math.isclose(odom_loop["yaw_deg"]["max"],
                             math.degrees(g["maximum_graph_shift_yaw_rad"]),
                             abs_tol=1e-7)):
        raise ValueError("graph control comparison inconsistent with saved loop diagnostics")
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
        "readme": ("Three paired estimate comparisons in the shared first-scan gauge: "
                   "Ceres frontend versus loop GTSAM, odometry-only GTSAM "
                   "versus loop GTSAM, and Ceres versus odometry-only GTSAM. "
                   "The legacy 3 m / 0.9 rad post-hoc warning uses odometry-only "
                   "GTSAM as its reference, NOT Ceres. None of these differences "
                   "is independent position or map accuracy."),
        "comparison_status": status,
        "frontend_trajectory_identical": True,
        "frontend_matching_audit_identical": True,
        "frontend_map_identical": True,
        "trajectory_comparisons": comparisons,
        "trajectory_comparison_frame": "shared_first_scan_gauge",
        "trajectory_comparison_not_ground_truth": True,
        "legacy_graph_shift_warning_reference": "odometry_graph_to_lidar_loop",
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
        row = scorecard_csv_row(report)
        writer = csv.DictWriter(handle, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    review_files(manifest, "fr079_loop_comparison", [
        ("comparison_json", result_path, True),
        ("comparison_csv", csv_path, True),
    ])
    print("Status:", report["comparison_status"])
    print("Accepted loop closures:", report["loops_accepted"])
    for name, summary in report["trajectory_comparisons"].items():
        print(f"{name}:")
        for dimension in ("translation_m", "yaw_deg"):
            s = summary[dimension]
            print(f"  {dimension}: median={s['median']:.3f}, "
                  f"p95={s['p95']:.3f}, max={s['max']:.3f} "
                  f"(scan {s['max_scan_index']})")
    print("Legacy shift warning compares GTSAM odometry-only versus loops, "
          "not Ceres versus loops.")
    print("No independent trajectory/map accuracy has been established.")


if __name__ == "__main__":
    main()
