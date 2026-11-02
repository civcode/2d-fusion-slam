#!/usr/bin/env python3
"""Reproducible multi-geometry synthetic fusion matching-beam sweep.

Runs the real slam_offline_fusion executable; no simulation reference is used
by the estimator. Map metrics use independently generated synthetic scene
truth and are read only after each run.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

SCENES = ("apartment", "warehouse", "office", "loft",
          "atrium", "industrial", "warehouse_asymmetric")
MATCHERS = ("legacy", "continuous")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo", type=Path, default=Path.cwd())
    p.add_argument("--out-dir", type=Path, required=True,
                   help="must not exist yet; no results are overwritten")
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--docker", action="store_true")
    source.add_argument("--binary", type=Path)
    p.add_argument("--scenes", choices=SCENES, nargs="+", default=list(SCENES))
    p.add_argument("--matchers", choices=MATCHERS, nargs="+", default=list(MATCHERS))
    p.add_argument("--match-strides", type=int, nargs="+", default=[1, 2, 3, 4, 6, 8])
    p.add_argument("--match-offsets", type=int, nargs="+", default=[0],
                   help="starting positions among valid LiDAR hits; duplicate effective offsets are skipped")
    p.add_argument("--map-stride", type=int, default=1)
    p.add_argument("--match-every", type=int, default=1)
    p.add_argument("--field-every", type=int, default=1)
    p.add_argument("--distance-truncation", type=float, default=0.7)
    p.add_argument("--lidar-only-bootstrap", action="store_true",
                   help="opt-in gated scan-to-scan bootstrapping of LiDAR-only map")
    p.add_argument("--lidar-only-predict-first", action="store_true",
                   help="predict scan motion from LiDAR before EDT (requires bootstrap)")
    p.add_argument("--dry-run", action="store_true")
    return p


def validate(a, p):
    if a.lidar_only_predict_first and not a.lidar_only_bootstrap:
        p.error("--lidar-only-predict-first requires --lidar-only-bootstrap")
    if len(set(a.match_strides)) != len(a.match_strides):
        p.error("duplicate match stride")
    if len(set(a.match_offsets)) != len(a.match_offsets):
        p.error("duplicate match offset")
    if not all(0 <= phase <= 63 for phase in a.match_offsets):
        p.error("match offsets must be within [0,63]")
    if len(set(a.scenes)) != len(a.scenes) or len(set(a.matchers)) != len(a.matchers):
        p.error("duplicate scene or matcher")
    if not all(1 <= x <= 64 for x in [a.map_stride, *a.match_strides]):
        p.error("beam strides must be within [1,64]")
    if not 1 <= a.match_every <= 200 or not 1 <= a.field_every <= 200:
        p.error("cadences must be in [1,200]")
    if not 0.2 <= a.distance_truncation <= 3.0:
        p.error("EDT cutoff must be within [0.2,3.0]")
    a.repo = a.repo.resolve(strict=True)
    a.out_dir = a.out_dir.resolve()
    if a.binary:
        a.binary = a.binary.resolve(strict=True)
    if a.out_dir.exists():
        p.error("output directory exists; use a new name")


def command(a, scene, matcher, stride, label, offset=0):
    opts = ["--match-every", str(a.match_every),
            "--field-every", str(a.field_every),
            "--map-stride", str(a.map_stride),
            "--match-stride", str(stride),
            "--match-offset", str(offset),
            "--distance-truncation", f"{a.distance_truncation:.3f}"]
    if matcher == "continuous":
        opts.append("--continuous-cost")
    if a.lidar_only_bootstrap:
        opts.append("--lidar-only-bootstrap")
    if a.lidar_only_predict_first:
        opts.append("--lidar-only-predict-first")
    if a.docker:
        return [
            "docker", "compose", "run", "--rm",
            "-v", f"{a.out_dir}:/fusion",
            "--entrypoint", "./build-docker/slam_offline_fusion",
            "test", scene, f"/fusion/{label}", *opts,
        ]
    return [str(a.binary), scene, str(a.out_dir / label), *opts]


def score_rows(directory, scene, matcher, stride, label, opts, offset=0):
    report = json.loads((directory / "fusion_report.json").read_text())
    prof = json.loads((directory / "fusion_profile.json").read_text())
    replay = json.loads((directory / "fusion_replay.json").read_text())
    expected = "ceres_continuous_cost" if matcher == "continuous" else "ceres"
    if report["matcher"] != expected:
        raise ValueError(f"{label}: expected Ceres matcher {expected}, got {report['matcher']}")
    for k, expected_value in (
        ("match_every", opts.match_every),
        ("field_every", opts.field_every),
        ("map_stride", opts.map_stride),
        ("match_stride", stride),
        ("match_offset", offset),
    ):
        if report[k] != expected_value or replay[k] != expected_value:
            raise ValueError(f"{label}: unexpected {k}")
    if report["lidar_only_bootstrap"] != opts.lidar_only_bootstrap:
        raise ValueError(f"{label}: LiDAR-only bootstrap mode differs")
    if report["lidar_only_predict_first"] != opts.lidar_only_predict_first:
        raise ValueError(f"{label}: LiDAR-only motion prediction mode differs")
    if abs(report["distance_truncation_m"] - opts.distance_truncation) > 1e-9:
        raise ValueError(f"{label}: unexpected EDT truncation")
    if not report["frames"] == replay["frame_count"] == prof["frames"]:
        raise ValueError(f"{label}: frame counts differ")
    if report["heldout_attempted"] + report["heldout_skipped"] != report["frames"] // 2:
        raise ValueError(f"{label}: held-out matching accounting invalid")
    if report["lidar_only_accepted"] + report["lidar_only_rejected"] + report["lidar_only_skipped"] != report["frames"] - 1:
        raise ValueError(f"{label}: LiDAR-only matching accounting invalid")
    rows = []
    for map_name, entry in report["results"].items():
        m = entry["map"]["metrics"]
        row = {
            "run": label, "scene": scene, "matcher": matcher,
            "match_stride": stride,
            "match_offset": offset,
            "nominal_beams": (240 + stride - 1 - offset) // stride,
            "map_stride": opts.map_stride,
            "lidar_only_bootstrap": opts.lidar_only_bootstrap,
            "lidar_only_predict_first": opts.lidar_only_predict_first,
            "lidar_only_predicted_edt_accepted": report["lidar_only_predicted_edt_accepted"],
            "lidar_only_edt_prediction_disagreements": report["lidar_only_edt_prediction_disagreements"],
            "lidar_only_bootstrap_attempted": report["lidar_only_bootstrap_attempted"],
            "lidar_only_bootstrap_accepted": report["lidar_only_bootstrap_accepted"],
            "lidar_only_edt_accepted": report["lidar_only_edt_accepted"],
            "lidar_only_edt_refined_accepted": report["lidar_only_edt_refined_accepted"],
            "match_every": opts.match_every,
            "field_every": opts.field_every,
            "distance_truncation_m": opts.distance_truncation,
            "map_method": map_name, "frames": report["frames"],
            "heldout_attempted": report["heldout_attempted"],
            "heldout_mean_input_beams": report["heldout_matching_input_hits_total"] / max(1, report["heldout_attempted"]),
            "lidar_only_mean_input_beams": report["lidar_only_matching_input_hits_total"] / max(1, report["frames"] - 1 - report["lidar_only_skipped"]),
            "heldout_accepted": report["heldout_accepted"],
            "heldout_skipped": report["heldout_skipped"],
            "lidar_only_accepted": report["lidar_only_accepted"],
            "lidar_only_rejected": report["lidar_only_rejected"],
            "lidar_only_skipped": report["lidar_only_skipped"],
            "lidar_only_field_rebuilds": report["lidar_only_field_rebuilds"],
            "translation_rmse_m": entry["translation_rmse_m"],
            "translation_p95_m": entry["translation_p95_m"],
            "yaw_rmse_rad": entry["yaw_rmse_rad"],
            "occupied_iou": m["occupied_iou"],
            "offline_processing_fps": prof.get("effective_processing_fps"),
            "map_fingerprint": entry["map"]["map_fingerprint"],
        }
        rows.append(row)
    return rows


def main():
    p = parser()
    a = p.parse_args()
    validate(a, p)
    # Only distinct effective phases are scheduled per stride (e.g. stride 1
    # has one phase, stride 2 has two). Phase zero preserves the old sweep.
    specs = []
    for scene in a.scenes:
        for matcher in a.matchers:
            for stride in a.match_strides:
                offsets = list(dict.fromkeys(value % stride
                                             for value in a.match_offsets))
                for offset in offsets:
                    suffix = f"-p{offset:02d}" if (
                        len(a.match_offsets) > 1 or offset != 0) else ""
                    specs.append((scene, matcher, stride, offset,
                                  f"{scene}-{matcher}-s{stride:02d}{suffix}"))
    if a.dry_run:
        for scene, matcher, stride, offset, label in specs:
            print("+", " ".join(command(a, scene, matcher, stride, label, offset)))
        return
    a.out_dir.mkdir(parents=True)
    manifest = {
        "format": "FUSFUSIONBEAMSWEEP1",
        "source": "selected analytic synthetic maps/routes with independently withheld simulator truth",
        "commit_or_binary": str(a.binary) if a.binary else "Docker test image supplied by user",
        "scenes": a.scenes, "matchers": a.matchers,
        "match_strides": a.match_strides,
        "match_offsets_requested": a.match_offsets,
        "map_stride": a.map_stride, "match_every": a.match_every,
        "field_every": a.field_every,
        "distance_truncation_m": a.distance_truncation,
        "lidar_only_bootstrap": a.lidar_only_bootstrap,
        "lidar_only_predict_first": a.lidar_only_predict_first,
        "commands": [],
        "run_count": len(specs),
        "runs": [],
    }
    rows = []
    # These map variants do not use scan-matching corrections. They are a
    # negative control: their fingerprints must not depend on match stride.
    controls = {}
    for scene, matcher, stride, offset, label in specs:
        cmd = command(a, scene, matcher, stride, label, offset)
        manifest["commands"].append(cmd)
        manifest["runs"].append({"scene":scene,"matcher":matcher,
                                 "match_stride":stride,"match_offset":offset,
                                 "label":label})
        print("+", " ".join(cmd), flush=True)
        logfile = a.out_dir / (label + ".run.log")
        with logfile.open("w") as f:
            result = subprocess.run(cmd, cwd=a.repo, stdout=f,
                                    stderr=subprocess.STDOUT, check=False)
        if result.returncode:
            tail = "\n".join(logfile.read_text(errors="replace").splitlines()[-25:])
            raise RuntimeError(f"{label} failed ({result.returncode}):\n{tail}")
        run_rows = score_rows(a.out_dir/label, scene, matcher, stride, label, a, offset)
        for item in run_rows:
            # Fused matched maps legitimately vary with matching stride/phase.
            # Only wheel/gyro baselines are independent of LiDAR matching.
            if item["map_method"] not in ("wheel_only", "gyro_wheel"):
                continue
            key = (scene, item["map_method"])
            old = controls.setdefault(key, item["map_fingerprint"])
            if old != item["map_fingerprint"]:
                raise ValueError(
                    f"{label}: non-matching control map changed with match stride/phase")
        rows.extend(run_rows)
        print("Completed", label, flush=True)
    with (a.out_dir/"scorecard.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (a.out_dir/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print("Scorecard:", a.out_dir/"scorecard.csv")


if __name__ == "__main__":
    main()
