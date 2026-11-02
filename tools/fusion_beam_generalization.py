#!/usr/bin/env python3
"""Summarize whether increasing LiDAR hit count generalizes across maps/phases.

Use independently scored fusion sweep scorecard.csv. This tool does not
modify matching, run experiments, or give simulator truth to the estimator.
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import median


def read(path):
    with Path(path).open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("empty scorecard")
    seen = set()
    for row in rows:
        key = (row["scene"], row["matcher"], int(row["match_stride"]),
               int(row.get("match_offset", "0")), row["map_method"])
        if key in seen:
            raise ValueError(f"duplicate scored configuration: {key}")
        seen.add(key)
    return rows


def summarize(rows, map_method="lidar_only"):
    groups = defaultdict(dict)
    for row in rows:
        if row["map_method"] != map_method:
            continue
        scene = row["scene"]
        matcher = row["matcher"]
        stride = int(row["match_stride"])
        phase = int(row.get("match_offset", "0"))
        groups[(scene, matcher, phase)][stride] = row

    if not groups:
        raise ValueError(f"no rows for map method {map_method!r}")

    print(f"Map method: {map_method}; groups: {len(groups)} scene/matcher/phase combinations")
    print("Each scene-phase is a paired test; phase offsets are not independent datasets.")
    summary = defaultdict(list)
    worsening = defaultdict(list)
    for (scene, matcher, phase), series in sorted(groups.items()):
        keys = sorted(series, reverse=True)  # coarse to dense: 8,6,4,3,2,1
        last = None
        for stride in keys:
            row = series[stride]
            rmse = float(row["translation_rmse_m"])
            iou = float(row["occupied_iou"])
            summary[(matcher, stride)].append((scene, phase, rmse, iou))
            if last is not None:
                poorer = rmse > last[1]+1e-9
                worsening[(matcher, last[0], stride)].append(poorer)
            last = (stride, rmse)

    for matcher in sorted({key[0] for key in summary}):
        print(f"\n=== {matcher.upper()} ===")
        print(f"{'Stride':>6} {'Nominal':>7} {'Pairs':>6} {'RMSE median':>12} "
              f"{'RMSE max':>10} {'IoU median':>11}")
        for _, stride in sorted((k for k in summary if k[0] == matcher),
                                key=lambda k: -k[1]):
            records = summary[(matcher, stride)]
            rmse = [record[2] for record in records]
            iou = [record[3] for record in records]
            print(f"{stride:>6} {240//stride:>7} {len(records):>6} "
                  f"{median(rmse):>12.4f} {max(rmse):>10.4f} {median(iou):>11.4f}")
        print("More-beams-worse paired transitions (same scene and phase):")
        for (m, before, after), flags in sorted(worsening.items(),
                                               key=lambda item: -item[0][1]):
            if m == matcher:
                print(f"  stride {before:>2} -> {after:>2}: "
                      f"{sum(flags)}/{len(flags)} worse")

    print("\n=== PAIRED 60 -> 240 BEAMS (stride 4 -> 1) ===")
    for matcher in sorted({key[1] for key in groups}):
        comparisons = []
        for (scene, m, phase), series in sorted(groups.items()):
            if m != matcher or 4 not in series or 1 not in series:
                continue
            coarse = float(series[4]["translation_rmse_m"])
            dense = float(series[1]["translation_rmse_m"])
            comparisons.append((scene, phase, coarse, dense))
        if not comparisons:
            print(f"{matcher}: no matched stride-4/stride-1 samples")
            continue
        better = sum(dense < coarse for _, _, coarse, dense in comparisons)
        ties = sum(abs(dense-coarse) <= 1e-9
                   for _, _, coarse, dense in comparisons)
        ratios = [dense/coarse for _, _, coarse, dense in comparisons if coarse > 0]
        # Count each scene once for a geometry-level conclusion, rather than
        # treating three phases from the same map as three different maps.
        by_scene = defaultdict(list)
        for scene, phase, coarse, dense in comparisons:
            if coarse > 0:
                by_scene[scene].append(dense/coarse)
        scene_wins = sum(median(ratios) < 1.0 for ratios in by_scene.values())
        scene_ratios = [median(ratios) for ratios in by_scene.values()]
        print(f"{matcher}: {better}/{len(comparisons)} phase-pairs improved"
              f", {ties} tied; scene-level {scene_wins}/{len(by_scene)} improved")
        if ratios:
            print(f"  phase-pair median RMSE ratio: {median(ratios):.3f}"
                  f"; per-scene median ratio: {median(scene_ratios):.3f}")
        for scene in sorted(by_scene):
            print(f"  {scene}: per-phase median 240/60 RMSE ratio "
                  f"{median(by_scene[scene]):.3f}")
    return groups


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scorecard", type=Path, required=True)
    p.add_argument("--map-method", default="lidar_only")
    a = p.parse_args()
    summarize(read(a.scorecard), a.map_method)


if __name__ == "__main__":
    main()
