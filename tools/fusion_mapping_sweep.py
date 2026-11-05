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
from review_artifacts import review_files, host_path


SCENES = ("apartment", "warehouse", "warehouse_asymmetric",
          "office", "loft", "atrium", "industrial")  # immutable 2026 legacy suite
DIVERSE_SCENES = ("apartment_explore", "warehouse", "warehouse_asymmetric",
                  "workshop", "loft", "atrium", "industrial")
SCENE_SUITES = {"legacy_v1": SCENES, "diverse_v2": DIVERSE_SCENES}


def suite_scenes(suite: str) -> tuple[str, ...]:
    try:
        return SCENE_SUITES[suite]
    except KeyError as exc:
        raise ValueError(f"unknown benchmark suite: {suite}") from exc

EVIDENCE = tuple(ablation.GRID_EVIDENCE)  # complete 3x3, not an estimator input
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


def run_all(binary: Path, runs_dir: Path, prefix: str,
            suite: str = "legacy_v1") -> None:
    checked_prefix(prefix)
    scenes = suite_scenes(suite)
    if not binary.is_file():
        raise FileNotFoundError(f"native fusion binary not found: {binary}")
    if not runs_dir.is_dir():
        raise FileNotFoundError(f"existing output root required: {runs_dir}")
    destinations = [runs_dir / f"{prefix}-{scene}" for scene in scenes]
    occupied = [str(path) for path in destinations if path.exists()]
    if occupied:
        raise FileExistsError("refusing to overwrite existing runs: " +
                              ", ".join(occupied))
    for scene, directory in zip(scenes, destinations):
        command = [str(binary.resolve()), scene, str(directory.resolve()),
                   *FUSION_FLAGS]
        print(f"\n[{scene}] {directory}", flush=True)
        subprocess.run(command, check=True)  # local native process, never Actions
    print(f"\nCompleted seven fixed-detector scene runs (suite={suite}).")
    # The native generator prints each run's review files. Also provide one
    # discoverable index of scene reports for upload without opening 7 folders.
    reviews = [(f"{scene}_report", directory / "fusion_report.json", True)
               for scene, directory in zip(scenes, destinations)]
    if all(file.is_file() for _, file, _ in reviews):
        review_files(runs_dir / f"{prefix}-runs.review.json",
                     f"fusion_mapping_sweep:{suite}", reviews)


def _finite(value, label: str) -> float:
    if (not isinstance(value, (int, float)) or
            isinstance(value, bool) or
            not math.isfinite(value)):
        raise ValueError(f"missing/nonfinite score: {label}")
    return float(value)


def _value(row: dict, name: str) -> float | None:
    value = row["map"]["metrics"].get(name)
    return None if value is None else _finite(value, name)


def collect(runs_dir: Path, prefix: str, suite: str = "legacy_v1"):
    checked_prefix(prefix)
    scenes = suite_scenes(suite)
    rows, region_rows = [], []
    config = None
    expected_evidence = None
    for scene in scenes:
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
        scene_evidence = {
            row["evidence"] for row in variants
            if row["window"] == "full" and row["deskew"] == DESKEW
        }
        if scene_evidence not in (set(ablation.LEGACY_EVIDENCE),
                                  set(ablation.GRID_EVIDENCE)):
            raise ValueError(f"{scene}: unsupported evidence settings")
        if expected_evidence is None:
            expected_evidence = scene_evidence
        elif scene_evidence != expected_evidence:
            raise ValueError("mixed legacy and three-by-three sweep runs")
        for window in WINDOWS:
            by_evidence = {
                row["evidence"]: row for row in variants
                if row["window"] == window and row["deskew"] == DESKEW
            }
            if set(by_evidence) != expected_evidence:
                raise ValueError(f"{scene}: missing original-deskew evidence levels")
            for evidence in ablation.GRID_EVIDENCE:
                if evidence not in expected_evidence:
                    continue
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
    if suite != "legacy_v1":
        rows = [{"suite": suite, **row} for row in rows]
        region_rows = [{"suite": suite, **row} for row in region_rows]
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


def rank_evidence(rows: list[dict],
                  scenes: tuple[str, ...] = SCENES) -> list[dict]:
    """Rank only complete comparable scene groups; never promote defaults."""
    by_scene = {(r["scene"], r["evidence"]): r for r in rows}
    if len(by_scene) != len(rows):
        raise ValueError("duplicate scene/evidence rows in ranking")
    expected_scenes = set(scenes)
    present = {r["evidence"] for r in rows}
    if present not in (set(ablation.LEGACY_EVIDENCE), set(ablation.GRID_EVIDENCE)):
        raise ValueError("ranking requires all four legacy or all nine grid settings")
    if {r["scene"] for r in rows} != expected_scenes or len(rows) != len(scenes)*len(present):
        raise ValueError("ranking must include every scene and evidence setting")
    ranked = []
    for evidence in ablation.GRID_EVIDENCE:
        if evidence not in present:
            continue
        group = [by_scene[scene, evidence] for scene in scenes]
        original = [by_scene[scene, "baseline"] for scene in scenes]
        prec_changes = []
        unavailable = False
        for row, base in zip(group, original):
            a, b = base["precision"], row["precision"]
            if a is None or a <= 0 or b is None:
                unavailable = True
            else:
                prec_changes.append(100. * (b/a - 1.))
        iou_regressions = sum(r["iou"] + 1e-12 < b["iou"]
                              for r, b in zip(group, original))
        wall_regressions = sum(
            r["mean_wall_m"] is not None and b["mean_wall_m"] is not None
            and r["mean_wall_m"] > b["mean_wall_m"] + 1e-12
            for r, b in zip(group, original))
        all_prec = [r["precision"] for r in group]
        result = {
            "evidence": evidence,
            "hit": ablation.GRID_EVIDENCE[evidence][0],
            "free": ablation.GRID_EVIDENCE[evidence][1],
            "mean_iou": sum(r["iou"] for r in group) / len(group),
            "mean_precision": (sum(all_prec)/len(all_prec)
                               if all(v is not None for v in all_prec) else None),
            "worst_precision_change_pct": (min(prec_changes)
                                            if not unavailable else None),
            "precision_loss_gt_5pct_scenes":
                sum(x < -5. - 1e-10 for x in prec_changes)
                if not unavailable else None,
            "precision_loss_gt_10pct_scenes":
                sum(x < -10. - 1e-10 for x in prec_changes)
                if not unavailable else None,
            "iou_regression_scenes": iou_regressions,
            "wall_distance_regression_scenes": wall_regressions,
            "precision_guardrail_5pct": (
                "PASS" if not unavailable and all(x >= -5.-1e-10 for x in prec_changes)
                else "UNDEFINED" if unavailable else "FAIL"
            ),
        }
        ranked.append(result)
    ranked.sort(key=lambda r: (-r["mean_iou"], r["evidence"]))
    return ranked


def summarize(runs_dir: Path, prefix: str, csv_path: Path | None,
              region_csv_path: Path | None,
              ranking_csv_path: Path | None = None,
              suite: str = "legacy_v1") -> None:
    scenes = suite_scenes(suite)
    rows, regions = collect(runs_dir, prefix, suite)
    rankings = rank_evidence(rows, scenes)
    if suite != "legacy_v1":
        rankings = [{"suite": suite, **row} for row in rankings]
    by_scene = {(r["scene"], r["evidence"]): r for r in rows}
    print(f"Seven-scene LiDAR-loop detector (suite={suite}); graph poses fixed within each scene.")
    print("Loop counts can differ across scenes. No simulator truth entered mapping.")
    print("Full-run 5cm exact occupied IoU; original deskew, fixed hit/free evidence.")
    print(f"Evidence levels: {len(rankings)} "
          f"({'full 3x3' if len(rankings)==9 else 'legacy 2x2'})\n")
    print(f"{'Scene':23} {'loops':>5} {'IoU base':>9} {'IoU comb':>9} "
          f"{'P base':>8} {'P comb':>8} {'mean wall base':>14} {'mean wall comb':>14}")
    for scene in scenes:
        a = by_scene[scene, "baseline"]
        b = by_scene[scene, "combined"]
        def fmt(value):
            return f"{value:.4f}" if value is not None else "undefined"
        print(f"{scene:23} {a['accepted_loops']:5} {a['iou']:9.4f} "
              f"{b['iou']:9.4f} {fmt(a['precision']):>8} "
              f"{fmt(b['precision']):>8} {fmt(a['mean_wall_m']):>14} "
              f"{fmt(b['mean_wall_m']):>14}")

    print("\nCross-scene macro averages (unweighted; ranked by mean occupied IoU):")
    print(f"{'Rank':>4} {'Evidence':16} {'hit':>5} {'free':>6} "
          f"{'mean IoU':>9} {'mean P':>8} {'worst ΔP':>9} "
          f"{'P loss>5%':>9} {'P loss>10%':>10} "
          f"{'IoU down':>8} {'wall down':>9} {'5% gate':>9}")
    for index, r in enumerate(rankings,1):
        def fmt(value, precision=2):
            return "N/A" if value is None else f"{value:.{precision}f}"
        print(f"{index:4} {r['evidence']:16} {r['hit']:5.2f} {r['free']:6.2f} "
              f"{r['mean_iou']:9.5f} {fmt(r['mean_precision'],5):>8} "
              f"{fmt(r['worst_precision_change_pct']):>9} "
              f"{str(r['precision_loss_gt_5pct_scenes']):>9} "
              f"{str(r['precision_loss_gt_10pct_scenes']):>10} "
              f"{r['iou_regression_scenes']:8} "
              f"{r['wall_distance_regression_scenes']:9} "
              f"{r['precision_guardrail_5pct']:>9}")
    print("\nPrecision penalties are relative percentages against each scene's "
          "+0.85/-0.40 baseline, not percentage points.")
    print("5% gate = no scene loses more than 5% precision; it is a "
          "diagnostic criterion, NOT automatic promotion.")
    print("Regional scores use the SAME original-ray +25cm footprint for all "
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
    if ranking_csv_path is not None:
        write_csv(ranking_csv_path, rankings)
        print(f"Cross-scene evidence ranking CSV: {ranking_csv_path}")
    exports = [
        ("full_scene_csv", csv_path, True),
        ("region_csv", region_csv_path, True),
        ("ranking_csv", ranking_csv_path, True),
    ]
    exports = [(name, path, upload) for name, path, upload in exports
               if path is not None]
    if exports:
        manifest = exports[0][1].parent / f"{prefix}-summary.review.json"
        if not manifest.exists():
            review_files(manifest, f"fusion_mapping_summary:{suite}", exports)
        else:
            print("Review manifest already exists:", host_path(manifest) or manifest)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run the same two-loop detector on all 7 scenes")
    run.add_argument("--binary", required=True, type=Path)
    run.add_argument("--runs-dir", required=True, type=Path)
    run.add_argument("--prefix", required=True)
    run.add_argument("--suite", choices=SCENE_SUITES, default="legacy_v1")
    analysis = sub.add_parser("summarize", help="read all 7 finished runs; never run SLAM")
    analysis.add_argument("--runs-dir", required=True, type=Path)
    analysis.add_argument("--prefix", required=True)
    analysis.add_argument("--suite", choices=SCENE_SUITES, default="legacy_v1")
    analysis.add_argument("--csv", type=Path)
    analysis.add_argument("--region-csv", type=Path)
    analysis.add_argument("--ranking-csv", type=Path,
                          help="optional ranked nine-setting comparison export")
    args = parser.parse_args()
    if args.command == "run":
        run_all(args.binary, args.runs_dir, args.prefix, args.suite)
    else:
        summarize(args.runs_dir, args.prefix, args.csv, args.region_csv,
                  args.ranking_csv, args.suite)


if __name__ == "__main__":
    main()
