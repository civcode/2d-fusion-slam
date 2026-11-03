#!/usr/bin/env python3
"""Audit fixed-graph occupancy experiments without changing any SLAM estimate.

Window-specific maps retain full-scene scores for backwards compatibility.
New runs also include a shared fixed-ray observation-footprint region score,
so evidence variants can be compared fairly within early/late windows.
All exact synthetic reference metrics are independent post-hoc scoring only.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path


def metric(value):
    if value is None:
        return None
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("non-finite mapping ablation metric")
    return float(value)


def load(run: Path):
    report = json.loads((run / "fusion_report.json").read_text())
    payload = json.loads((run / "mapping_ablation.json").read_text())
    if (payload.get("format") != "FUSMAPABL1" or
            payload.get("scene") != report.get("scene") or
            payload.get("reference_usage") != "posthoc_scoring_only" or
            payload.get("source") != "graph_optimized_fixed_estimated_poses"):
        raise ValueError("mapping-ablation format/source/scene mismatch")
    rows = payload.get("variants")
    if not isinstance(rows, list) or len(rows) != 24:
        raise ValueError("expected 24 fixed-graph mapping variants")
    expected = set()
    for window in ("full", "early", "late"):
        for deskew in ("gyro_wheel_original", "graph_corrected_motion_model"):
            for odds in ("baseline", "stronger_hit", "gentler_free", "combined"):
                expected.add(f"{window}_{deskew}_{odds}")
    if {row["id"] for row in rows} != expected:
        raise ValueError("missing/duplicated mapping ablation variants")
    baseline = next(row for row in rows
                    if row["id"] == "full_gyro_wheel_original_baseline")
    if (baseline["map"]["map_fingerprint"] !=
            report["results"]["graph_optimized"]["map"]["map_fingerprint"]):
        raise ValueError("original graph map fingerprint differs from mapping-only baseline")
    region_enabled = any("region_map" in row for row in rows)
    if region_enabled and not all(
            "region_map" in row and "region_footprint" in row and
            "region_wall_tolerance_scores" in row for row in rows):
        raise ValueError("incomplete region-masked mapping metrics")
    footprints = {}
    for row in rows:
        if row.get("evidence") not in (
                "baseline", "stronger_hit", "gentler_free", "combined"):
            raise ValueError("unknown occupancy evidence variant")
        if row["map"]["format"] != "FUSMAPACC1":
            raise ValueError("unverified map scoring format")
        if row["map"]["reference_scene"] != report["scene"]:
            raise ValueError("map reference mismatch")
        expected_range = ((0, report["frames"]) if row["window"] == "full" else
                          (0, report["frames"] - math.ceil(report["frames"] * .12))
                          if row["window"] == "early" else
                          (report["frames"] - math.ceil(report["frames"] * .12),
                           report["frames"]))
        if ((row["begin_frame"], row["end_frame_exclusive"]) != expected_range):
            raise ValueError("incorrect scan contribution window")
        if (not row["map_path"].startswith("mapping_ablation/") or
                Path(row["map_path"]).is_absolute() or
                ".." in Path(row["map_path"]).parts or
                not (run / row["map_path"]).is_file()):
            raise ValueError("mapping ablation map artifact missing/unsafe")
        bands = row["wall_tolerance_scores"]
        if [round(x["tolerance_m"], 2) for x in bands] != [.10, .25, .50]:
            raise ValueError("wall tolerance score thresholds missing")
        for key in ("precision", "recall", "f1"):
            vals = [metric(x[key]) for x in bands]
            if not all(0 <= x <= 1 for x in vals) or vals != sorted(vals):
                raise ValueError("nonmonotone wall distance agreement")
        if region_enabled:
            footprint = row["region_footprint"]
            if (footprint.get("source") != "original_baseline_rays" or
                    footprint.get("padding_m") != .25 or
                    not isinstance(footprint.get("cells"), int) or
                    not 0 < footprint["cells"] <= footprint.get("total_grid_cells", 0) or
                    not isinstance(footprint.get("fingerprint"), str) or
                    not footprint["fingerprint"].isdigit()):
                raise ValueError("invalid fixed observation footprint")
            previous = footprints.setdefault(row["window"], footprint)
            if previous != footprint:
                raise ValueError("region footprint changed across mapping variants")
            masked = row["region_map"]
            if (masked["format"] != "FUSMAPACC1" or
                    masked["reference_scene"] != report["scene"] or
                    masked["map_fingerprint"] != row["map"]["map_fingerprint"] or
                    masked["counts"]["known"] + masked["counts"]["unknown"] !=
                    footprint["cells"] or
                    not 0 < masked["counts"]["truth_wall"] <= footprint["cells"]):
                raise ValueError("region metric provenance or mask cardinality mismatch")
            region_bands = row["region_wall_tolerance_scores"]
            if [round(x["tolerance_m"], 2) for x in region_bands] != [.10, .25, .50]:
                raise ValueError("region tolerance scoring thresholds missing")
            for key in ("precision", "recall", "f1"):
                values = [metric(x[key]) for x in region_bands]
                if not all(0 <= v <= 1 for v in values) or values != sorted(values):
                    raise ValueError("nonmonotone region wall tolerance agreement")
    return report, rows


def normalized_row(row):
    m = row["map"]["metrics"]
    near = row["wall_tolerance_scores"]
    return {
        "variant": row["id"],
        "window": row["window"],
        "deskew": row["deskew"],
        "evidence": row["evidence"],
        "hit": row["hit_log_odds"],
        "free": row["free_log_odds"],
        "occupied_iou": metric(m["occupied_iou"]),
        "precision": metric(m["precision"]),
        "recall": metric(m["recall"]),
        "wall_mean_m": metric(m["symmetric_mean_wall_distance_m"]),
        "wall_p95_m": metric(m["symmetric_p95_wall_distance_m"]),
        "f1_10cm": near[0]["f1"],
        "f1_25cm": near[1]["f1"],
        "f1_50cm": near[2]["f1"],
        "region_cells": row.get("region_footprint", {}).get("cells"),
        "region_truth_wall": row.get("region_map", {}).get("counts", {}).get("truth_wall"),
        "region_iou": metric(row["region_map"]["metrics"]["occupied_iou"])
            if "region_map" in row else None,
        "region_precision": metric(row["region_map"]["metrics"]["precision"])
            if "region_map" in row else None,
        "region_recall": metric(row["region_map"]["metrics"]["recall"])
            if "region_map" in row else None,
        "region_wall_mean_m":
            metric(row["region_map"]["metrics"]["symmetric_mean_wall_distance_m"])
            if "region_map" in row else None,
        "region_f1_25cm": row["region_wall_tolerance_scores"][1]["f1"]
            if "region_wall_tolerance_scores" in row else None,
        "map_path": row["map_path"],
    }


def summarize(run: Path, csv_output: Path | None = None):
    report, variants = load(run)
    rows = [normalized_row(row) for row in variants]
    print(f"Scene: {report['scene']}; fixed graph poses: {report['frames']} scans")
    print("Full-scene reference scoring retained; optional spatially masked "
          "region metrics follow below.")
    print("Synthetic truth is used only after map reconstruction.")
    print("Original graph map fingerprint verified against baseline reconstruction.")
    for window in ("full", "early", "late"):
        subset = [row for row in rows if row["window"] == window]
        n = next(v for v in variants if v["window"] == window)
        print(f"\n== {window} (scans {n['begin_frame']}..{n['end_frame_exclusive']-1}) ==")
        if window != "full":
            print("  Subset map evaluated against FULL reference, not region-local truth.")
        print(f"{'deskew / evidence':50} {'IoU':>7} {'P':>7} {'R':>7} "
              f"{'wall mean':>10} {'F1 10cm':>9} {'F1 25cm':>9} {'F1 50cm':>9}")
        for row in subset:
            def fmt(key):
                return "undefined" if row[key] is None else f"{row[key]:.4f}"
            print(f"{(row['deskew'] + '/' + row['evidence']):50} "
                  f"{fmt('occupied_iou'):>7} {fmt('precision'):>7} "
                  f"{fmt('recall'):>7} {fmt('wall_mean_m'):>10} "
                  f"{fmt('f1_10cm'):>9} {fmt('f1_25cm'):>9} "
                  f"{fmt('f1_50cm'):>9}")
        if all(row["region_cells"] is not None for row in subset):
            print("  Region-masked (shared original-ray footprint + 25cm; "
                  "reference walls outside it are excluded):")
            print(f"  {'deskew / evidence':48} {'cells':>8} {'walls':>7} "
                  f"{'IoU':>7} {'P':>7} {'R':>7} {'wall mean':>10} {'F1 25cm':>9}")
            for row in subset:
                def fmt_region(key):
                    return ("undefined" if row[key] is None
                            else f"{row[key]:.4f}")
                print(f"  {(row['deskew'] + '/' + row['evidence']):48} "
                      f"{row['region_cells']:8} {row['region_truth_wall']:7} "
                      f"{fmt_region('region_iou'):>7} "
                      f"{fmt_region('region_precision'):>7} "
                      f"{fmt_region('region_recall'):>7} "
                      f"{fmt_region('region_wall_mean_m'):>10} "
                      f"{fmt_region('region_f1_25cm'):>9}")
        # Ranking only among identical-frame-window variants; report
        # observed scores, never feed synthetic scoring back into SLAM.
        best = max(subset, key=lambda x: x["f1_25cm"])
        print(f"  Highest post-hoc F1@25cm in this window: {best['variant']}")
    if csv_output:
        with csv_output.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nCSV: {csv_output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True,
                        help="new synthetic run with --mapping-ablation")
    parser.add_argument("--csv", type=Path,
                        help="optional human-readable metric table export")
    args = parser.parse_args()
    summarize(args.run, args.csv)


if __name__ == "__main__":
    main()
