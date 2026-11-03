#!/usr/bin/env python3
"""Seven-scene fixed-graph occupancy evidence sweep and independent scoring.

Native run mode calls the normal offline fusion binary sequentially. The
mapping ablation holds each scene's graph poses fixed across all evidence
settings; it does NOT force the same number of loop closures in every scene.
Synthetic reference scores are read only after the native estimator finishes.
No Python packages beyond the standard library or CI runners are needed.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
from pathlib import Path

import fusion_mapping_ablation as ablation


SCENES = ("apartment", "warehouse", "warehouse_asymmetric",
          "office", "loft", "atrium", "industrial")
EVIDENCE = ("baseline", "stronger_hit", "gentler_free", "combined")
WINDOWS = ("full", "early", "late")
DESKEW = "gyro_wheel_original"
FUSION_FLAGS = (
    "--continuous-cost", "--match-every", "1",
    "--match-stride", "1", "--field-every", "15", "--map-stride", "1",
    "--lidar-only-bootstrap", "--lidar-only-predict-first",
    "--lidar-loop-closure", "--mapping-ablation",
)
CONTROL_KEYS = (
    "matcher", "match_every", "match_stride", "field_every",
    "map_stride", "lidar_only_bootstrap", "lidar_only_predict_first",
    "lidar_loop_closure", "distance_truncation_m",
)


def checked_prefix(prefix: str) -> str:
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}", prefix):
        raise ValueError("run prefix must be 1-80 ASCII letters/digits/_/-")
    return prefix


def run_all(binary: Path, runs_dir: Path, prefix: str) -> None:
    checked_prefix(prefix)
    if not binary.is_file():
        raise FileNotFoundError(f"native fusion binary not found: {binary}")
    if not runs_dir.is_dir():
        raise FileNotFoundError(f"existing output root required: {runs_dir}")
    destinations = [runs_dir / f"{prefix}-{scene}" for scene in SCENES]
    occupied = [str(path) for path in destinations if path.exists()]
    if occupied:
        raise FileExistsError("refusing to overwrite existing runs: " +
                              ", ".join(occupied))
    for scene, directory in zip(SCENES, destinations):
        command = [str(binary.resolve()), scene, str(directory.resolve()),
                   *FUSION_FLAGS]
        print(f"\n[{scene}] {directory}", flush=True)
        subprocess.run(command, check=True)  # local native process, never Actions
    print("\nCompleted seven fixed-detector scene runs.")


def _finite(value, label: str) -> float:
    if (not isinstance(value, (int, float)) or
            isinstance(value, bool) or
            not math.isfinite(value)):
        raise ValueError(f"missing/nonfinite score: {label}")
    return float(value)


def _value(row: dict, name: str) -> float | None:
    value = row["map"]["metrics"].get(name)
    return None if value is None else _finite(value, name)


def collect(runs_dir: Path, prefix: str):
    checked_prefix(prefix)
    rows, region_rows = [], []
    config = None
    for scene in SCENES:
        directory = runs_dir / f"{prefix}-{scene}"
        report, variants = ablation.load(directory)
        if report.get("scene") != f"synthetic-{scene}-v1":
            raise ValueError(f"scene/run mismatch for {scene}")
        if (report.get("graph_status") != "gtsam_with_lidar_scan_loop_closure" or
                report.get("lidar_loop_closure") is not True):
            raise ValueError(f"{scene}: expected configured LiDAR-loop GTSAM run")
        required = {
            "matcher": "ceres_continuous_cost",
            "match_every": 1, "match_stride": 1,
            "field_every": 15, "map_stride": 1,
            "lidar_only_bootstrap": True,
            "lidar_only_predict_first": True,
            "distance_truncation_m": .7,
        }
        if any(report.get(key) != value for key, value in required.items()):
            raise ValueError(f"{scene}: unexpected sweep configuration")
        current = tuple(report.get(key) for key in CONTROL_KEYS)
        if config is None:
            config = current
        elif current != config:
            raise ValueError(f"{scene}: cross-scene estimator settings changed")
        if not all("region_map" in row for row in variants):
            raise ValueError(f"{scene}: missing observation-footprint region metrics")
        for window in WINDOWS:
            by_evidence = {
                row["evidence"]: row for row in variants
                if row["window"] == window and row["deskew"] == DESKEW
            }
            if set(by_evidence) != set(EVIDENCE):
                raise ValueError(f"{scene}: missing original-deskew evidence levels")
            for evidence in EVIDENCE:
                variant = by_evidence[evidence]
                m = variant["map"]["metrics"]
                region = variant["region_map"]["metrics"]
                row = {
                    "scene": scene, "frames": report["frames"],
                    "accepted_loops": report["graph_accepted_loops"],
                    "window": window, "evidence": evidence,
                    "hit": variant["hit_log_odds"],
                    "free": variant["free_log_odds"],
                    "iou": _finite(m["occupied_iou"], "iou"),
                    "precision": _finite(m["precision"], "precision")
                    if m["precision"] is not None else None,
                    "recall": _finite(m["recall"], "recall")
                    if m["recall"] is not None else None,
                    "mean_wall_m": _value(variant, "symmetric_mean_wall_distance_m"),
                    "p95_wall_m": _value(variant, "symmetric_p95_wall_distance_m"),
                    "f1_10cm": variant["wall_tolerance_scores"][0]["f1"],
                    "f1_25cm": variant["wall_tolerance_scores"][1]["f1"],
                    "f1_50cm": variant["wall_tolerance_scores"][2]["f1"],
                }
                if window == "full":
                    rows.append(row)
                mask = variant["region_footprint"]
                region_rows.append({
                    **{key: row[key] for key in (
                        "scene", "frames", "accepted_loops", "window",
                        "evidence", "hit", "free")},
                    "mask_cells": mask["cells"],
                    "mask_fraction": mask["cells"] / mask["total_grid_cells"],
                    "reference_wall_cells": variant["region_map"]["counts"]["truth_wall"],
                    "region_iou": _finite(region["occupied_iou"], "region iou"),
                    "region_precision": (None if region["precision"] is None
                                         else _finite(region["precision"], "region precision")),
                    "region_recall": (None if region["recall"] is None
                                      else _finite(region["recall"], "region recall")),
                    "region_wall_mean_m": (None if region["symmetric_mean_wall_distance_m"] is None
                                           else _finite(region["symmetric_mean_wall_distance_m"], "region mean")),
                    "region_f1_10cm": variant["region_wall_tolerance_scores"][0]["f1"],
                    "region_f1_25cm": variant["region_wall_tolerance_scores"][1]["f1"],
                    "region_f1_50cm": variant["region_wall_tolerance_scores"][2]["f1"],
                })
    return rows, region_rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError("cannot export an empty sweep")
    # Don't write inside root-owned Docker run folders; call with a writable
    # host-local destination such as reports/fusion_mapping_sweep.csv.
    if not path.parent.is_dir():
        raise FileNotFoundError(f"CSV parent directory does not exist: {path.parent}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def summarize(runs_dir: Path, prefix: str, csv_path: Path | None,
              region_csv_path: Path | None) -> None:
    rows, regions = collect(runs_dir, prefix)
    by_scene = {(r["scene"], r["evidence"]): r for r in rows}
    print("Seven-scene LiDAR-loop detector; graph poses fixed within each scene.")
    print("Loop counts can differ across scenes. No simulator truth entered mapping.")
    print("Full-run 5cm exact occupied IoU: original deskew, fixed 4 evidence levels.\n")
    print(f"{'Scene':23} {'loops':>5} {'IoU base':>9} {'IoU comb':>9} "
          f"{'P base':>8} {'P comb':>8} {'mean wall base':>14} {'mean wall comb':>14}")
    for scene in SCENES:
        a = by_scene[scene, "baseline"]
        b = by_scene[scene, "combined"]
        def fmt(value):
            return f"{value:.4f}" if value is not None else "undefined"
        print(f"{scene:23} {a['accepted_loops']:5} {a['iou']:9.4f} "
              f"{b['iou']:9.4f} {fmt(a['precision']):>8} "
              f"{fmt(b['precision']):>8} {fmt(a['mean_wall_m']):>14} "
              f"{fmt(b['mean_wall_m']):>14}")

    print("\nCross-scene macro averages (unweighted, one vote per scene):")
    print(f"{'Evidence':16} {'mean IoU':>10} {'mean precision':>15} "
          f"{'IoU regressions':>15} {'precision <-10%':>16} {'wall regressions':>17}")
    for evidence in EVIDENCE:
        group = [by_scene[scene, evidence] for scene in SCENES]
        originals = [by_scene[scene, "baseline"] for scene in SCENES]
        def average(key):
            values = [x[key] for x in group]
            return sum(values)/len(values) if all(v is not None for v in values) else None
        iou_bad = sum(row["iou"] + 1e-12 < base["iou"]
                      for row, base in zip(group, originals))
        precision_bad = sum(
            row["precision"] is not None and base["precision"] is not None and
            row["precision"] + 1e-12 < .9 * base["precision"]
            for row, base in zip(group, originals)
        )
        distance_bad = sum(
            row["mean_wall_m"] is not None and
            base["mean_wall_m"] is not None and
            row["mean_wall_m"] > base["mean_wall_m"] + 1e-12
            for row, base in zip(group, originals)
        )
        avg_iou=average("iou")
        avg_p=average("precision")
        p_label = f"{avg_p:.5f}" if avg_p is not None else "undefined"
        print(f"{evidence:16} {avg_iou:10.5f} "
              f"{p_label:>15} "
              f"{iou_bad:15} {precision_bad:16} {distance_bad:17}")
    print("\nRegional scores use the SAME original-ray +25cm footprint for all "
          "evidence variants in each scene/window.")
    print("Early and late observation areas can overlap; do not add or directly "
          "compare their reference-wall recall.")
    print("No evidence setting is automatically promoted to a production default.")

    if csv_path is not None:
        write_csv(csv_path, rows)
        print(f"Full-scene CSV: {csv_path}")
    if region_csv_path is not None:
        write_csv(region_csv_path, regions)
        print(f"Spatially masked per-window CSV: {region_csv_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run the same two-loop detector on all 7 scenes")
    run.add_argument("--binary", required=True, type=Path)
    run.add_argument("--runs-dir", required=True, type=Path)
    run.add_argument("--prefix", required=True)
    analysis = sub.add_parser("summarize", help="read all 7 finished runs; never run SLAM")
    analysis.add_argument("--runs-dir", required=True, type=Path)
    analysis.add_argument("--prefix", required=True)
    analysis.add_argument("--csv", type=Path)
    analysis.add_argument("--region-csv", type=Path)
    args = parser.parse_args()
    if args.command == "run":
        run_all(args.binary, args.runs_dir, args.prefix)
    else:
        summarize(args.runs_dir, args.prefix, args.csv, args.region_csv)


if __name__ == "__main__":
    main()
