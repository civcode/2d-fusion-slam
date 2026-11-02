#!/usr/bin/env python3
"""Compare independently scored synthetic fused-graph loop closure A/B runs.

Scoring-only simulator truth is read AFTER both estimators finish; it is
never used for candidate generation, graph factors, or acceptance gates.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def load(folder: Path):
    return (json.loads((folder / "fusion_report.json").read_text()),
            json.loads((folder / "trajectories.json").read_text()),
            json.loads((folder / "fusion_replay.json").read_text()))


def final_error(poses, truth):
    a, b = poses[-1], truth[-1]
    return math.hypot(a[0] - b[0], a[1] - b[1])


def window_statistics(before, after, truth, begin, end):
    """Synthetic truth is used for post-hoc comparison only."""
    old_errors = [
        math.hypot(before[i][0] - truth[i][0],
                   before[i][1] - truth[i][1])
        for i in range(begin, end)
    ]
    new_errors = [
        math.hypot(after[i][0] - truth[i][0],
                   after[i][1] - truth[i][1])
        for i in range(begin, end)
    ]
    displacements = [
        math.hypot(after[i][0] - before[i][0],
                   after[i][1] - before[i][1])
        for i in range(begin, end)
    ]
    def rmse(vals):
        return math.sqrt(sum(value*value for value in vals)/len(vals))
    return (rmse(old_errors), rmse(new_errors),
            max(old_errors), max(new_errors),
            max(displacements), sum(displacements)/len(displacements))


def compare(baseline: Path, with_loops: Path, *, allow_loop_baseline=False):
    old, old_paths, _ = load(baseline)
    new, new_paths, new_replay = load(with_loops)
    if old["scene"] != new["scene"] or old["frames"] != new["frames"]:
        raise ValueError("different scene or trajectory length")
    if not new.get("lidar_loop_closure") or (
            old.get("lidar_loop_closure") and not allow_loop_baseline):
        raise ValueError("expected baseline without and experiment with LiDAR loops; "
                         "pass --allow-loop-baseline to compare two closure versions")
    for method in ("wheel_only", "lidar_only", "gyro_wheel",
                   "fused_scan_matched"):
        if (old["results"][method]["map"]["map_fingerprint"] !=
                new["results"][method]["map"]["map_fingerprint"] or
                old_paths["paths"][method] != new_paths["paths"][method]):
            raise ValueError(f"non-graph baseline modified: {method}")
    if old_paths["synthetic_truth_for_scoring"] != new_paths["synthetic_truth_for_scoring"]:
        raise ValueError("different post-hoc scoring reference")
    reference = new_paths["synthetic_truth_for_scoring"]
    events = [e for f in new_replay["frames"] for e in f["loop_events"]]
    accepted = [e for e in events if e["decision"] == "accepted"]
    if len(accepted) != new["graph_accepted_loops"]:
        raise ValueError("graph loop replay/report count mismatch")
    print(f"Scene: {new['scene']} · frames: {new['frames']}")
    print("Non-graph paths and map fingerprints: unchanged")
    if old.get("lidar_loop_closure"):
        print(f"Previous loop-enabled run: {old['graph_accepted_loops']} "
              "accepted constraints (algorithm-version comparison)")
    else:
        print("Baseline loop detector: disabled (controlled no-loop comparison)")
    print(f"Pose-nearby revisit pairs: {new.get('graph_lidar_loop_nearby_pairs', 'not recorded')}")
    print(f"LiDAR ICP pair checks: {new['graph_lidar_loop_registration_attempted']}")
    print(f"Strict geometry passes: {new.get('graph_lidar_loop_geometry_verified', 'not recorded')}")
    print(f"Redundant proposals suppressed: {new.get('graph_lidar_loop_redundant_suppressed', 'not recorded')}")
    print(f"Nonredundant graph proposals: {new['graph_lidar_loop_registration_verified']}")
    print(f"Graph accepted loops: {new['graph_accepted_loops']} / {new['graph_loop_candidates']}")
    old_label = "Earlier loop mode" if old.get("lidar_loop_closure") else "No LiDAR loops"
    print(f"{'Metric':38} {old_label:>16} {'New LiDAR loops':>16}")
    for label, method in (("Graph translation RMSE (m)", "translation_rmse_m"),
                          ("Graph map occupied IoU", "occupied_iou")):
        old_value = (old["results"]["graph_optimized"].get(method)
                     if method != "occupied_iou" else
                     old["results"]["graph_optimized"]["map"]["metrics"][method])
        new_value = (new["results"]["graph_optimized"].get(method)
                     if method != "occupied_iou" else
                     new["results"]["graph_optimized"]["map"]["metrics"][method])
        print(f"{label:38} {old_value:16.5f} {new_value:16.5f}")
    print(f"{'Graph final position error (m)':38} "
          f"{final_error(old_paths['paths']['graph_optimized'], reference):16.5f} "
          f"{final_error(new_paths['paths']['graph_optimized'], reference):16.5f}")
    # Distinguish propagation of a correction from actual localization
    # improvement: a changed pose may still be wrong. Inspect the last
    # 12% AND the interval between its start and the final closure.
    n = len(reference)
    first = n - max(1, math.ceil(n*.12))
    old_graph = old_paths["paths"]["graph_optimized"]
    new_graph = new_paths["paths"]["graph_optimized"]
    windows = [("Early / middle", 0, first),
               ("Final 12%", first, n)]
    late_loops = sorted(e["to"] for e in accepted
                        if first <= e["to"] < n-1)
    if late_loops:
        last = late_loops[-1]
        if first < last:
            windows.append(("Up to final closure", first, last+1))
        if last+1 < n:
            windows.append(("After final closure", last+1, n))
    print("\nGraph corrections and synthetic pose errors by frame window:")
    print("Window                 Frames   RMSE old -> new (m)   "
          "Max error old -> new (m)   Mean/max pose shift (m)")
    for label, begin, end in windows:
        if begin >= end:
            continue
        a, b, c, d, max_shift, avg_shift = window_statistics(
            old_graph, new_graph, reference, begin, end)
        print(f"{label:22} {begin:3}-{end-1:<3} "
              f"{a:7.3f} -> {b:7.3f}   "
              f"{c:7.3f} -> {d:7.3f}   "
              f"{avg_shift:.3f} / {max_shift:.3f}")
    for index in sorted({first, *late_loops, n-1}):
        old_error = math.hypot(
            old_graph[index][0]-reference[index][0],
            old_graph[index][1]-reference[index][1])
        new_error = math.hypot(
            new_graph[index][0]-reference[index][0],
            new_graph[index][1]-reference[index][1])
        shift = math.hypot(
            new_graph[index][0]-old_graph[index][0],
            new_graph[index][1]-old_graph[index][1])
        print(f"Frame {index:3} | old error {old_error:.3f} m, "
              f"new error {new_error:.3f} m, graph shift {shift:.3f} m")

    if not accepted:
        print("No loop was accepted; do not attribute any change to loop closure.")
    else:
        print("Accepted loop frames: " + ", ".join(
            f"{item['from']}->{item['to']}" for item in accepted))
        for event in accepted:
            print("  pair {:3}->{:3}: overlap={}, ICP RMS={} m".format(
                event["from"],event["to"],
                event.get("overlap", "not recorded"),
                event.get("registration_error_m", "not recorded")))
    print("These are independent synthetic POST-HOC scoring metrics, not loop-factor evidence.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--loops", type=Path, required=True)
    parser.add_argument("--allow-loop-baseline", action="store_true",
                        help="Explicitly compare an older loop-enabled run "
                             "with a newer loop-enabled run; not a no-loop A/B")
    args = parser.parse_args()
    compare(args.baseline, args.loops,
            allow_loop_baseline=args.allow_loop_baseline)


if __name__ == "__main__":
    main()
