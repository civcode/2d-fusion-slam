#!/usr/bin/env python3
"""Integration smoke of diverse synthetic map generators and scoring.

The test runs the real binary. Exact geometry/truth stays confined to the
generator and the independently invoked scorer, not LiDAR-only matching.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile

SCENES = ("office", "loft", "atrium", "industrial", "warehouse_asymmetric")


def run(*args):
    result = subprocess.run(args, text=True, capture_output=True, timeout=180)
    if result.returncode:
        raise AssertionError(f"command failed {args}:\n{result.stderr}\n{result.stdout}")
    return result


def check_scene(binary, scorer, root, name, offset):
    folder = root / f"{name}-p{offset}"
    run(binary, name, str(folder), "--match-stride", "4",
        "--match-offset", str(offset), "--lidar-only-bootstrap",
        "--lidar-only-predict-first")
    report = json.loads((folder / "fusion_report.json").read_text())
    replay = json.loads((folder / "fusion_replay.json").read_text())
    audit = json.loads((folder / "lidar_only_match_audit.json").read_text())
    trajectories = json.loads((folder / "trajectories.json").read_text())

    expected_name = f"synthetic-{name}-v1"
    assert report["scene"] == expected_name
    assert replay["scene"] == expected_name
    assert report["match_stride"] == replay["match_stride"] == 4
    assert report["match_offset"] == replay["match_offset"] == offset
    assert audit["match_offset"] == offset
    assert report["lidar_only_predict_first"]
    assert report["lidar_only_bootstrap"]
    assert report["map_pose_truth_used"] is False
    assert audit["ground_truth_used"] is False
    assert len(audit["frames"]) == report["frames"]-1
    assert len(trajectories["synthetic_truth_for_scoring"]) == report["frames"]
    assert report["lidar_only_accepted"] + report["lidar_only_rejected"] == report["frames"]-1
    for method in ("wheel_only", "gyro_wheel", "fused_scan_matched", "lidar_only"):
        entry = report["results"][method]
        output = run(scorer, name, str(folder / f"{method}.slammap"))
        independent = json.loads(output.stdout)
        assert independent["map_fingerprint"] == entry["map"]["map_fingerprint"]
        assert independent["confusion"] == entry["map"]["confusion"]
    return report


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: fusion_scene_diversity_test.py EXE MAP_SCORER")
    binary, scorer = sys.argv[1:]
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for name in SCENES:
            initial = check_scene(binary, scorer, root, name, 0)
            # Test that changing the matching-hit phase never changes
            # wheel/gyro baselines for exactly the same simulated scan stream.
            if name == "office":
                shifted = check_scene(binary, scorer, root, name, 1)
                for method in ("wheel_only", "gyro_wheel"):
                    assert (initial["results"][method]["map"]["map_fingerprint"] ==
                            shifted["results"][method]["map"]["map_fingerprint"])
        print("five new synthetic scenes, independent scoring and phase controls passed")


if __name__ == "__main__":
    main()
