#!/usr/bin/env python3
"""Summarize frozen-prior FR079 capture trials into recovery heatmaps.

Recovery means returning near the same run's accepted estimate (a correlated
proxy, NOT independently measured ground truth or a false-positive rate).
"""
from __future__ import annotations

from review_artifacts import review_files

import argparse
import csv
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture-csv", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True, help="new output basename")
    p.add_argument("--no-plot", action="store_true", help="do not require matplotlib")
    args = p.parse_args()
    results = [args.out.with_suffix(".json"), args.out.with_suffix(".csv")]
    if not args.no_plot:
        results.append(args.out.with_suffix(".png"))
    if any(path.exists() for path in results):
        p.error("refusing to overwrite existing capture summary")
    with args.capture_csv.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        p.error("no capture rows")
    required = {
        "format", "source_sha256", "anchor_kind", "truncation_m",
        "search_bound_xy_m", "initial_xy_offset_m", "initial_yaw_offset_deg",
        "recovered_proxy", "accepted_but_far_from_proxy", "initial_overlap",
        "ceres_invalid_residual_evaluations", "ceres_invalid_unknown_cells",
        "ceres_invalid_truncated", "ceres_invalid_outside_grid",
        "ceres_unsuccessful_steps", "trial_ms",
        "support_fraction_0_3m", "support_fraction_3_7m",
        "support_fraction_7_12m", "support_fraction_12_18m"
    }
    if not required.issubset(rows[0]):
        p.error("missing required capture columns: " + repr(sorted(required - rows[0].keys())))
    if {row["format"] for row in rows} != {"FUSFR079CAPTURE1"}:
        p.error("unexpected benchmark format")
    if {row["anchor_kind"] for row in rows} != {
            "same_run_accepted_pose_proxy_not_ground_truth"}:
        p.error("capture data does not provide the expected proxy provenance")
    sources = {row["source_sha256"] for row in rows}
    if len(sources) != 1:
        p.error("input scans differ within benchmark")
    groups = defaultdict(list)
    details = defaultdict(list)
    for row in rows:
        key = (float(row["truncation_m"]), float(row["search_bound_xy_m"]))
        groups[key].append(row)
        details[key + (float(row["initial_xy_offset_m"]),
                        float(row["initial_yaw_offset_deg"]))].append(row)

    def summary(cases):
        def average(field):
            return mean(float(c[field]) for c in cases)
        return {
            "trials": len(cases),
            "proxy_recovery_fraction": average("recovered_proxy"),
            "accepted_but_far_from_proxy_fraction": average(
                "accepted_but_far_from_proxy"),
            "initial_supported_fraction": average("initial_overlap"),
            "fraction_with_invalid_residual_evaluations": mean(
                int(c["ceres_invalid_residual_evaluations"]) > 0 for c in cases),
            "invalid_residual_evaluations_total": sum(
                int(c["ceres_invalid_residual_evaluations"]) for c in cases),
            "invalid_unknown_cells_total": sum(
                int(c["ceres_invalid_unknown_cells"]) for c in cases),
            "invalid_truncated_total": sum(
                int(c["ceres_invalid_truncated"]) for c in cases),
            "invalid_outside_grid_total": sum(
                int(c["ceres_invalid_outside_grid"]) for c in cases),
            "unsuccessful_solver_steps_total": sum(
                int(c["ceres_unsuccessful_steps"]) for c in cases),
            "mean_trial_ms": average("trial_ms"),
            "mean_support_by_range": {
                key: average(field) for key, field in (
                    ("0_3m", "support_fraction_0_3m"),
                    ("3_7m", "support_fraction_3_7m"),
                    ("7_12m", "support_fraction_7_12m"),
                    ("12_18m", "support_fraction_12_18m"))
            },
        }
    compact = []
    for (trunc, bound), cases in sorted(groups.items()):
        compact.append({"truncation_m": trunc, "search_bound_xy_m": bound,
                        **summary(cases)})
    matrix = []
    for (trunc, bound, xy, angle), cases in sorted(details.items()):
        matrix.append({"truncation_m": trunc, "search_bound_xy_m": bound,
                       "initial_xy_offset_m": xy, "initial_yaw_offset_deg": angle,
                       **summary(cases)})
    report = {
        "format": "FUSFR079CAPTURESUMMARY1",
        "source_sha256": next(iter(sources)),
        "anchor_kind": "same_run_accepted_pose_proxy_not_ground_truth",
        "independent_ground_truth": False,
        "interpretation": "Ceres returns close to its own run's accepted pose on a frozen earlier-scan map, not independent ATE/RPE",
        "grouped": compact, "heatmap_cells": matrix
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    with args.out.with_suffix(".csv").open("w", newline="") as stream:
        cols = ["truncation_m", "search_bound_xy_m", "trials",
                "proxy_recovery_fraction", "accepted_but_far_from_proxy_fraction",
                "initial_supported_fraction", "fraction_with_invalid_residual_evaluations",
                "invalid_residual_evaluations_total", "invalid_unknown_cells_total",
                "invalid_truncated_total", "invalid_outside_grid_total",
                "unsuccessful_solver_steps_total", "mean_trial_ms"]
        writer = csv.DictWriter(stream, fieldnames=cols, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(compact)
    if not args.no_plot:
        import matplotlib.pyplot as plt

        cutoffs = sorted({key[0] for key in groups})
        bounds = sorted({key[1] for key in groups})
        translations = sorted({key[2] for key in details})
        angles = sorted({key[3] for key in details})
        fig, axes = plt.subplots(len(bounds), len(cutoffs),
                                 figsize=(3.3 * len(cutoffs), 3.4 * len(bounds)),
                                 squeeze=False, constrained_layout=True)
        lookup = { (x["truncation_m"], x["search_bound_xy_m"],
                    x["initial_xy_offset_m"], x["initial_yaw_offset_deg"]):
                  x["proxy_recovery_fraction"] for x in matrix}
        for row_index, bound in enumerate(bounds):
            for col_index, trunc in enumerate(cutoffs):
                ax = axes[row_index][col_index]
                z = [[lookup.get((trunc, bound, xy, yaw), float("nan"))
                      for yaw in angles] for xy in translations]
                image = ax.imshow(z, vmin=0, vmax=1, cmap="viridis", origin="lower")
                ax.set_title(f"EDT {trunc:g} m · bounds ±{bound:g} m", fontsize=10)
                ax.set_xticks(range(len(angles)), labels=[f"{v:g}" for v in angles])
                ax.set_yticks(range(len(translations)),
                              labels=[f"{v:g}" for v in translations])
                ax.set_xlabel("Heading perturbation (degrees)")
                ax.set_ylabel("Translation perturbation (metres)")
        fig.colorbar(image, ax=axes.ravel().tolist(),
                     label="Recovery to same-run pose proxy", shrink=0.8)
        fig.suptitle("FR079 frozen-map capture range — NOT independent ground truth")
        fig.savefig(args.out.with_suffix(".png"), dpi=150)
        plt.close(fig)
    print(json.dumps({"format": report["format"], "grouped": compact}, indent=2))
    review_entries = [
        ("capture_summary_json", args.out.with_suffix(".json"), True),
        ("capture_summary_csv", args.out.with_suffix(".csv"), True),
    ]
    if args.out.with_suffix(".png").is_file():
        review_entries.append(
            ("capture_heatmap_png", args.out.with_suffix(".png"), True))
    review_files(args.out.with_suffix(".review.json"),
                 "fr079_capture_summary", review_entries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
