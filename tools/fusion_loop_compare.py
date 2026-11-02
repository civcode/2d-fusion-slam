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


def compare(baseline: Path, with_loops: Path):
    old, old_paths, _ = load(baseline)
    new, new_paths, new_replay = load(with_loops)
    if old["scene"] != new["scene"] or old["frames"] != new["frames"]:
        raise ValueError("different scene or trajectory length")
    if old.get("lidar_loop_closure") or not new.get("lidar_loop_closure"):
        raise ValueError("expected baseline without and experiment with LiDAR loops")
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
    print(f"LiDAR ICP loop checks: {new['graph_lidar_loop_registration_attempted']}")
    print(f"Geometrically verified proposals: {new['graph_lidar_loop_registration_verified']}")
    print(f"Graph accepted loops: {new['graph_accepted_loops']} / {new['graph_loop_candidates']}")
    print(f"{'Metric':38} {'No LiDAR loops':>16} {'LiDAR loops':>16}")
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
    if not accepted:
        print("No loop was accepted; do not attribute any change to loop closure.")
    else:
        print("Accepted loop frames: " + ", ".join(
            f"{item['from']}->{item['to']}" for item in accepted))
    print("These are independent synthetic POST-HOC scoring metrics, not loop-factor evidence.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--loops", type=Path, required=True)
    args = parser.parse_args()
    compare(args.baseline, args.loops)


if __name__ == "__main__":
    main()
