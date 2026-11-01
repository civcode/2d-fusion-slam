#!/usr/bin/env python3
"""Native end-to-end FR079 LiDAR+odom regression, no external fixture needed.

Default: generate a bounded CARMEN rectangle traversal and verify genuine
unmapped-start reconstruction plus laser-pose leakage resistance.
Optional second argv: real FR079 .log.gz, run full recording once.
"""
from __future__ import annotations
import gzip
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


def carmen_fixture(*, corrupt_laser=False, perturb_odom=False,
                   reverse_time=False) -> str:
    lines = ["PARAM fixture=offline_slam"]
    walls = (0.0, 6.0, 0.0, 5.0)
    for i in range(55):
        a = 2 * math.pi * i / 55
        x, y = 3 + 1.25 * math.cos(a), 2.5 + 1.0 * math.sin(a)
        yaw = a + math.pi / 2
        ox = x + 0.012 * i + (1.0 if perturb_odom and i > 25 else 0.0)
        oy = y + 0.006 * i
        oyaw = yaw + 0.0015 * i
        ranges = []
        for j in range(360):
            theta = yaw - math.pi / 2 + math.pi * j / 360
            dx, dy = math.cos(theta), math.sin(theta)
            hits = []
            if abs(dx) > 1e-12:
                for wx in walls[:2]:
                    t = (wx - x) / dx
                    yy = y + t * dy
                    if t > 0 and walls[2] <= yy <= walls[3]:
                        hits.append(t)
            if abs(dy) > 1e-12:
                for wy in walls[2:]:
                    t = (wy - y) / dy
                    xx = x + t * dx
                    if t > 0 and walls[0] <= xx <= walls[1]:
                        hits.append(t)
            dist = min(hits) if hits else 0.0
            if j % 47 == 0:
                dist = 0.0  # explicit invalid / no-return sample
            ranges.append(f"{dist:.6f}")
        t = 1710000000 + (54 - i if reverse_time else i) * 0.15
        # Logged laser poses are deliberately not relied upon as truth.
        lx = x + (200.0 if corrupt_laser else 0.0)
        ly = y - (300.0 if corrupt_laser else 0.0)
        lines.append("FLASER 360 " + " ".join(ranges) +
                     f" {lx:.6f} {ly:.6f} {yaw:.6f}"
                     f" {ox:.6f} {oy:.6f} {oyaw:.6f}"
                     f" {t:.6f} testhost {t:.6f}")
    return "\n".join(lines) + "\n"


def run(binary: Path, source: Path, target: Path, *flags: str,
        expected_success: bool = True):
    proc = subprocess.run(
        [str(binary), str(source), str(target), *flags], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=600,
        check=False)
    if (proc.returncode == 0) != expected_success:
        raise AssertionError(
            f"run status {proc.returncode}, expected_success={expected_success}:\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}")
    return proc


def validate(prefix: Path, expected_scans: int):
    report = json.loads(prefix.with_suffix(".report.json").read_text())
    timeline = json.loads(prefix.with_suffix(".trajectory.json").read_text())
    matches = json.loads(prefix.with_suffix(".matches.json").read_text())
    replay = json.loads(prefix.with_suffix(".slamreplay.json").read_text())
    assert report["format"] == "FUSFR079SLAM1"
    assert report["frames"] == expected_scans
    assert report["initial_map_empty"] is True
    assert report["external_map_loaded"] is False
    assert report["uses_logged_laser_pose_as_input"] is False
    assert report["laser_extrinsic"] == "assumed_identity_NOT_CALIBRATED"
    assert report["independent_ground_truth"] is False
    assert report["map_frame_verified"] is False
    assert report["per_beam_time_verified"] is False
    assert report["pose_graph_optimized"] is False
    assert report["loop_closures"] == 0
    assert report["realtime_deadlines_verified"] is False
    assert report["map"]["revision"] == report["inserted_scans"]
    assert report["accepted"] + report["rejected"] == report["attempts"]
    assert 0 < report["inserted_scans"] <= expected_scans
    assert 0 < report["attempts"] < expected_scans
    assert len(timeline["poses"]) == expected_scans
    assert len(matches["frames"]) == expected_scans
    assert len(replay["frames"]) == expected_scans
    for artifact in (timeline, matches):
        assert artifact["source_sha256"] == report["source_sha256"]
    assert replay["source"]["source_sha256"] == report["source_sha256"]
    assert replay["source"]["pose_kind"] == \
        "estimated_fr079_odometry_scan_matched_not_ground_truth"
    assert replay["source"]["per_beam_time_known"] is False
    for i, (f, decision, pose) in enumerate(zip(
            replay["frames"], matches["frames"], timeline["poses"])):
        assert f["index"] == decision["index"] == pose["index"] == i
        assert f["pose"] == pose["estimated"]
        assert f["odom"] == pose["odom"]
        assert f["matching"] is None
        assert 0 <= f["hit_count"] <= f["beam_count"] <= 4096
        assert len(f["points"]) <= f["hit_count"]
        assert decision["accepted"] <= decision["attempted"]
        if decision["accepted"]:
            assert decision["status"] == "accepted"
    p = report["profiling"]
    assert p["slam_compute_ms"] > 0
    assert p["estimated_offline_fps"] > 0
    assert p["frame_latency"]["count"] == expected_scans
    assert p["frame_latency"]["p50_ms"] <= p["frame_latency"]["p95_ms"]
    assert p["frame_latency"]["p95_ms"] <= p["frame_latency"]["max_ms"]
    assert prefix.with_suffix(".slammap").stat().st_size > 100
    assert prefix.with_suffix(".pgm").read_bytes().startswith(b"P5\n")
    assert prefix.with_suffix(".yaml").is_file()
    assert prefix.with_suffix(".slammap").read_bytes().find(
        b"fr079-carmen-odom-lidar-incremental-v1") != -1
    return report


def main():
    binary = Path(sys.argv[1]).resolve(strict=True)
    with tempfile.TemporaryDirectory(prefix="fr079-offline-regression-") as temp:
        root = Path(temp)
        if len(sys.argv) >= 3:
            dataset = Path(sys.argv[2]).resolve(strict=True)
            flags = ("--reference",) if "--reference" in sys.argv[3:] else ()
            prefix = root / "real"
            run(binary, dataset, prefix, *flags)
            report = validate(prefix, 3118)
            print("real FR079 end-to-end:", report["frames"],
                  "scans", report["accepted"], "accepted matches")
            return

        source = root / "fixture.log.gz"
        source.write_bytes(gzip.compress(carmen_fixture().encode()))
        prefix = root / "original"
        run(binary, source, prefix, "--reference", "--match-every", "3",
            "--field-every", "5")
        original = validate(prefix, 55)

        altered = root / "changed-laser-pose.log"
        altered.write_text(carmen_fixture(corrupt_laser=True))
        alt_prefix = root / "laser-perturbed"
        run(binary, altered, alt_prefix, "--reference", "--match-every", "3",
            "--field-every", "5")
        alternate = validate(alt_prefix, 55)
        assert original["source_sha256"] != alternate["source_sha256"]
        assert original["map"]["fingerprint"] == alternate["map"]["fingerprint"], \
            "logged laser poses leaked into reconstructed map"
        assert prefix.with_suffix(".slammap").read_bytes() == \
            alt_prefix.with_suffix(".slammap").read_bytes(), \
            "laser pose changes influenced authoritative occupancy data"

        shifted = root / "changed-odom.log"
        shifted.write_text(carmen_fixture(perturb_odom=True))
        odom_prefix = root / "odom-perturbed"
        run(binary, shifted, odom_prefix, "--reference", "--match-every", "3",
            "--field-every", "5")
        other = validate(odom_prefix, 55)
        assert original["map"]["fingerprint"] != other["map"]["fingerprint"], \
            "changing raw odometry had no effect on map"

        reversed_log = root / "reversed.log"
        reversed_log.write_text(carmen_fixture(reverse_time=True))
        run(binary, reversed_log, root / "bad-order", "--reference",
            expected_success=False)
        run(binary, source, prefix, "--reference", expected_success=False)
        print("FR079 end-to-end native mapping, replay, provenance, "
              "laser-pose nonleakage, odometry sensitivity and "
              "failure gates passed")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("Usage: fr079_offline_test.py EXECUTABLE [FR079_LOG] [--reference]")
    main()
