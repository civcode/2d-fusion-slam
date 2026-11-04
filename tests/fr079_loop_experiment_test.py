#!/usr/bin/env python3
"""End-to-end opt-in FR079 loop run and paired no-loop graph comparison.

Uses a synthetic CARMEN-format log with only scan-time odometry; this tests
data provenance and graph exports, not actual Freiburg loop performance.
No GTSAM simulation truth can enter the estimator.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fr079_offline_test import carmen_fixture, run
from fr079_loop_compare import compare


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: fr079_loop_experiment_test.py FR079_BINARY")
    binary = Path(sys.argv[1]).resolve(strict=True)
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        log = root / "repeated-rectangle.carmen.log"
        log.write_text(carmen_fixture(frames=184))
        base = root / "noloop"
        loop = root / "loop"
        flags = ("--reference", "--run-directory",
                 "--match-every", "3", "--field-every", "5",
                 "--map-stride", "3", "--match-stride", "4")
        run(binary,log,base,*flags)
        output = run(binary,log,loop,*flags,"--lidar-loop-closure")
        assert "loop_decisions=" in output.stdout
        for suffix in (".slammap",".graph_control.slammap",
                       ".graph_optimized.slammap",".loops.json",
                       ".report.json",".trajectory.json",".review.json"):
            path = loop / (loop.name + suffix)
            assert path.is_file(), path
            assert path.stat().st_size > 100, path
        verified = compare(base,loop)
        assert verified["frontend_trajectory_identical"]
        assert verified["frontend_map_identical"]
        assert verified["frames"] == 184
        graph = json.loads((loop/(loop.name+".loops.json")).read_text())
        assert graph["format"] == "FUSFR079LOOP1"
        assert graph["independent_ground_truth"] is False
        assert graph["first_pose_fixed"] is True
        assert graph["proposed_loops"] >= graph["accepted_loops"]
        assert len(graph["odometry_graph_poses"]) == 184
        assert len(graph["loop_graph_poses"]) == 184
        assert (loop/(loop.name+".graph_control.slammap")).read_bytes()[:7] == b"FUSMAP2"
        manifest=json.loads((loop/(loop.name+".review.json")).read_text())
        assert manifest["format"]=="SLAM_REVIEW1"
        assert {x["role"] for x in manifest["files"]} >= {
            "report","matching_decisions","loop_decisions_and_graph_poses",
            "odometry_only_graph_map","loop_optimized_map"}
        print("FR079 loop/native graph-control integration and pairing passed")
        print("Loop closures on toy CARMEN fixture (not Freiburg):",
              graph["accepted_loops"])


if __name__ == "__main__":
    main()
