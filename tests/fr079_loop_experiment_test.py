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
        corrected = root / "frontend-sequential"
        run(binary,log,corrected,*flags,
            "--lidar-loop-closure","--frontend-sequential-graph")
        new_report=json.loads((corrected/(corrected.name+".report.json")).read_text())
        new_graph=json.loads((corrected/(corrected.name+".loops.json")).read_text())
        new_trajectory=json.loads((corrected/(corrected.name+".trajectory.json")).read_text())
        assert new_report["loop_mode"] == "fr079_frontend_relative_gtsam_loops_v2"
        assert new_report["sequential_factor_source"] == (
            "frontend_scanmatched_relative_correlated_v1")
        assert new_graph["sequential_factor_source"] == (
            "frontend_scanmatched_relative_correlated_v1")
        assert new_graph["first_pose_fixed"] is True
        assert len(new_graph["graph_control_poses"]) == 184
        assert len(new_graph["loop_graph_poses"]) == 184
        # Native no-loop graph must preserve the input frontend trajectory.
        for ctrl, frame in zip(new_graph["graph_control_poses"],
                               new_trajectory["poses"]):
            pose = frame["estimated"]
            assert sum((x-y)**2 for x,y in zip(ctrl[:2],pose[:2])) < 1e-8
            assert abs((ctrl[2]-pose[2]+3.141592653589793) % (
                2*3.141592653589793)-3.141592653589793) < 1e-4
        corrected_manifest=json.loads(
            (corrected/(corrected.name+".review.json")).read_text())
        assert "frontend_sequential_graph_control_map" in {
            entry["role"] for entry in corrected_manifest["files"]}
        # Opt-in full-heading pose-prior scan matcher. This mode must be
        # separately selectable, preserving the original loop experiment.
        pose_prior = root / "pose-prior"
        run(binary,log,pose_prior,*flags,
            "--lidar-loop-closure","--frontend-sequential-graph",
            "--loop-registration","pose-prior-scan")
        pose_report=json.loads(
            (pose_prior/(pose_prior.name+".report.json")).read_text())
        pose_loops=json.loads(
            (pose_prior/(pose_prior.name+".loops.json")).read_text())
        assert pose_report["loop_registration_mode"] == "pose_prior_scan_to_scan_v1"
        assert pose_loops["registration_mode"] == "pose_prior_scan_to_scan_v1"
        assert pose_loops["sequential_factor_source"] == (
            "frontend_scanmatched_relative_correlated_v1")
        assert pose_loops["large_heading_nearby_pairs"] >= (
            pose_loops["large_heading_registration_attempts"])
        assert pose_loops["large_heading_registration_attempts"] >= (
            pose_loops["large_heading_geometry_verified"])
        assert pose_loops["large_heading_geometry_verified"] >= (
            pose_loops["large_heading_accepted_loops"])
        assert all("predicted_relative_yaw_rad" in entry
                   for entry in pose_loops["loop_decisions"])
        assert len(pose_loops["graph_control_poses"]) == 184
        assert len(pose_loops["loop_graph_poses"]) == 184
        assert (pose_prior/(pose_prior.name+".graph_optimized.slammap")).is_file()
        # A loop matcher change must not affect frontend scan matching/maps.
        prior_timeline=json.loads(
            (pose_prior/(pose_prior.name+".trajectory.json")).read_text())
        assert prior_timeline["poses"] == new_trajectory["poses"]
        assert pose_report["map"]["fingerprint"] == new_report["map"]["fingerprint"]
        # Opt-in local frozen-submap matcher. Zero loops is a valid result
        # on this small fixture; invariants and graph provenance still apply.
        submaps = root / "submap"
        run(binary,log,submaps,*flags,
            "--lidar-loop-closure","--frontend-sequential-graph",
            "--loop-registration","scan-to-submap")
        submap_report=json.loads(
            (submaps/(submaps.name+".report.json")).read_text())
        submap_graph=json.loads(
            (submaps/(submaps.name+".loops.json")).read_text())
        submap_trajectory=json.loads(
            (submaps/(submaps.name+".trajectory.json")).read_text())
        assert submap_report["loop_registration_mode"] == (
            "correlative_scan_to_submap_v1")
        assert submap_graph["registration_mode"] == (
            "correlative_scan_to_submap_v1")
        assert submap_graph["loop_evidence"] == (
            "correlated_raw_lidar_frozen_submap_correlative_registration")
        assert submap_graph["sequential_factor_source"] == (
            "frontend_scanmatched_relative_correlated_v1")
        assert submap_graph["submap_registration_attempts"] >= (
            submap_graph["submap_geometry_verified"])
        assert submap_graph["submap_registration_attempts"] == (
            submap_graph["registration_attempts"])
        assert submap_graph["submap_geometry_verified"] >= (
            submap_graph["accepted_loops"])
        assert submap_graph["large_heading_accepted_loops"] <= (
            submap_graph["accepted_loops"])
        assert submap_graph["first_pose_fixed"] is True
        assert len(submap_graph["graph_control_poses"]) == 184
        assert len(submap_graph["loop_graph_poses"]) == 184
        assert all("submap_translation_curvature_min" in entry
                   for entry in submap_graph["loop_decisions"])
        assert submap_trajectory["poses"] == new_trajectory["poses"]
        assert submap_report["map"]["fingerprint"] == new_report["map"]["fingerprint"]
        assert (submaps/(submaps.name+".graph_optimized.slammap")).is_file()
        print("FR079 loop/native graph-control integration and pairing passed")
        print("Loop closures on toy CARMEN fixture (not Freiburg):",
              graph["accepted_loops"])


if __name__ == "__main__":
    main()
