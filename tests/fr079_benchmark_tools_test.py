#!/usr/bin/env python3
"""No-dataset benchmark CLI parsing and CSV aggregation regression."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def invoke(*args: str, success=True) -> subprocess.CompletedProcess:
    result = subprocess.run([sys.executable, *map(str, args)], capture_output=True,
                            text=True, timeout=30)
    if (result.returncode == 0) != success:
        raise AssertionError(f"unexpected exit {result.returncode}: "
                             f"{result.stdout}\n{result.stderr}")
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    sweep = root / "tools/fr079_truncation_sweep.py"
    summary = root / "tools/fr079_capture_summary.py"
    with tempfile.TemporaryDirectory(prefix="fr079-bench-tools-") as tmp:
        folder = Path(tmp)
        log = folder / "test.log"
        log.write_text("test")
        out = folder / "future"
        a = invoke(sweep, "--repo", root, "--log", log,
                   "--out-dir", out, "--binary", sys.executable,
                   "--truncations", "0.7", "1.0", "--dry-run")
        assert "--distance-truncation 0.700" in a.stdout
        assert "--distance-truncation 1.000" in a.stdout
        assert "--match-every 1 --field-every 15" in a.stdout
        assert not out.exists(), "dry-run created artifacts"
        invoke(sweep, "--repo", root, "--log", log, "--out-dir", out,
               "--binary", sys.executable, "--truncations", ".1",
               "--dry-run", success=False)
        invoke(sweep, "--repo", root, "--log", log, "--out-dir", out,
               "--binary", sys.executable, "--truncations", ".7", ".700",
               "--dry-run", success=False)
        invoke(sweep, "--repo", root, "--log", log, "--out-dir", out,
               "--binary", sys.executable, "--truncations", "1.0",
               "--capture", "--dry-run", success=False)
        b = invoke(sweep, "--repo", root, "--log", log, "--out-dir", out,
                   "--binary", sys.executable, "--truncations", "0.7", "1.0",
                   "--capture", "--capture-max-frames", "3", "--dry-run")
        assert b.stdout.count("--capture ") == 1
        guarded = invoke(sweep, "--repo", root, "--log", log,
                         "--out-dir", out, "--binary", sys.executable,
                         "--truncations", "0.7", "1.0", "--map-stride", "1",
                         "--support-aware", "--dry-run")
        assert guarded.stdout.count("--support-aware") == 2
        assert guarded.stdout.count("--map-stride 1") == 2

        capture = folder / "capture.csv"
        records = []
        for idx, recovered in enumerate((1, 0)):
            records.append({
                "format": "FUSFR079CAPTURE1",
                "source_sha256": "1"*64,
                "anchor_kind": "same_run_accepted_pose_proxy_not_ground_truth",
                "truncation_m": "0.7", "search_bound_xy_m": "0.12",
                "initial_xy_offset_m": "0.1",
                "initial_yaw_offset_deg": "2",
                "recovered_proxy": str(recovered),
                "accepted_but_far_from_proxy": str(1-recovered),
                "initial_overlap": "0.6",
                "ceres_invalid_residual_evaluations": str(idx),
                "ceres_invalid_unknown_cells": str(idx),
                "ceres_invalid_truncated": "0",
                "ceres_invalid_outside_grid": "0",
                "ceres_unsuccessful_steps": str(idx),
                "trial_ms": "1.5",
                "support_fraction_0_3m": "0.9",
                "support_fraction_3_7m": "0.7",
                "support_fraction_7_12m": "0.4",
                "support_fraction_12_18m": "0.1",
            })
        with capture.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
        target = folder / "capture-summary"
        invoke(summary, "--capture-csv", capture, "--out", target, "--no-plot")
        report = json.loads(target.with_suffix(".json").read_text())
        assert report["format"] == "FUSFR079CAPTURESUMMARY1"
        assert report["independent_ground_truth"] is False
        group = report["grouped"][0]
        assert group["trials"] == 2
        assert group["proxy_recovery_fraction"] == .5
        assert group["invalid_unknown_cells_total"] == 1
        assert group["fraction_with_invalid_residual_evaluations"] == .5
        assert len(list(csv.DictReader(target.with_suffix(".csv").open()))) == 1
        invoke(summary, "--capture-csv", capture, "--out", target, "--no-plot",
               success=False)
    print("FR079 truncation CLI, dry-run, capture sampling and no-truth CSV "
          "summary regression passed")


if __name__ == "__main__":
    main()
