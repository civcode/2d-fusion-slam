#!/usr/bin/env python3
"""Compare two completed synthetic fusion sweeps without rerunning experiments.

The report uses only withheld synthetic scoring outputs, never as estimator input.
Higher beam count is a hypothesis to test, not an assumption or a sorting rule.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def read(path: Path):
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path}: no scorecard data")
    table = {}
    for row in rows:
        key = (row["scene"], row["matcher"], int(row["match_stride"]),
               row["map_method"], int(row.get("match_offset", "0")))
        if key in table:
            raise ValueError(f"{path}: duplicate scorecard key {key}")
        table[key] = row
    return table


def compare(old, new):
    if old.keys() != new.keys():
        missing_old = new.keys() - old.keys()
        missing_new = old.keys() - new.keys()
        raise ValueError(f"sweep keys differ: new only={sorted(missing_old)}, "
                         f"old only={sorted(missing_new)}")
    for method in ("wheel_only", "gyro_wheel", "fused_scan_matched"):
        bad = [k for k in old if k[3] == method
               and old[k]["map_fingerprint"] != new[k]["map_fingerprint"]]
        if bad:
            raise ValueError(f"non-LiDAR-only control map changed: "
                             f"{method}, {len(bad)} cases")
    print("Controls: wheel_only, gyro_wheel, fused_scan_matched — PASS")

    groups = sorted({(k[0], k[1]) for k in old if k[3] == "lidar_only"})
    for scene, matcher in groups:
        keys = [k for k in old if k[0] == scene and k[1] == matcher
                and k[3] == "lidar_only"]
        keys.sort(key=lambda k: (int(new[k]["nominal_beams"]), k[4]))
        print(f"\n=== {scene.upper()} / {matcher.upper()} ===")
        print(f"{'Beams':>5} {'Phase':>5} {'Old RMSE':>10} {'New RMSE':>10} "
              f"{'Old IoU':>9} {'New IoU':>9} {'Accepted':>8} "
              f"{'Scan-pair':>9} {'Pred.EDT':>8} {'Conflicts':>9}")
        errors = []
        for k in keys:
            a, b = old[k], new[k]
            rmse = float(b["translation_rmse_m"])
            errors.append(rmse)
            print(f"{int(b['nominal_beams']):>5} {k[4]:>5} "
                  f"{float(a['translation_rmse_m']):>10.4f} {rmse:>10.4f} "
                  f"{float(a['occupied_iou']):>9.4f} "
                  f"{float(b['occupied_iou']):>9.4f} "
                  f"{int(b['lidar_only_accepted']):>8} "
                  f"{int(b.get('lidar_only_bootstrap_accepted', 0)):>9} "
                  f"{int(b.get('lidar_only_predicted_edt_accepted', 0)):>8} "
                  f"{int(b.get('lidar_only_edt_prediction_disagreements', 0)):>9}")
        # Only compare beam counts within the SAME hit-subsampling phase.
        # Aggregating independent phase sets would fabricate transitions.
        regression_total = comparisons = 0
        by_phase = {}
        for k in keys:
            by_phase.setdefault(k[4], []).append(k)
        for phase, phase_keys in by_phase.items():
            phase_keys.sort(key=lambda k: int(new[k]["nominal_beams"]))
            for left, right in zip(phase_keys, phase_keys[1:]):
                comparisons += 1
                regression_total += (float(new[right]["translation_rmse_m"]) >
                                     float(new[left]["translation_rmse_m"])+1e-9)
        print(f"More-beams-worse adjacent transitions (same phase): "
              f"{regression_total}/{comparisons}")
        best = min(keys, key=lambda k: float(new[k]["translation_rmse_m"]))
        print(f"Lowest RMSE: {int(new[best]['nominal_beams'])} beams, "
              f"{float(new[best]['translation_rmse_m']):.4f} m")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--old", type=Path, required=True,
                   help="previous sweep scorecard.csv")
    p.add_argument("--new", type=Path, required=True,
                   help="experimental sweep scorecard.csv")
    a = p.parse_args()
    compare(read(a.old), read(a.new))


if __name__ == "__main__":
    main()
