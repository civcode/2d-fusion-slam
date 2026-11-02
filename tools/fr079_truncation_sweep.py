#!/usr/bin/env python3
"""FR079 EDT truncation sweep: same input/scans, one variable at a time.

Outputs one immutable map/report per truncation plus a CSV scorecard.
The supplied scan-matched reference map is a correlated agreement proxy,
never independent ground truth. No GitHub Actions or remote execution.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

DEFAULT_TRUNCATIONS = (0.35, 0.5, 0.7, 1.0, 1.5, 2.0)


def truncations(values: list[str]) -> list[float]:
    result = []
    for raw in values:
        number = float(raw)
        if not math.isfinite(number) or not 0.2 <= number <= 3.0:
            raise ValueError("truncations must be finite and inside [0.2, 3.0] m")
        if int(number * 1000 + 0.5) in [int(x * 1000 + 0.5) for x in result]:
            raise ValueError("duplicate truncations at millimetre precision")
        result.append(number)
    if not result:
        raise ValueError("at least one truncation required")
    return result


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", type=Path, required=True,
                   help="local FR079 CARMEN .log.gz path")
    p.add_argument("--repo", type=Path, default=Path.cwd())
    p.add_argument("--out-dir", type=Path, required=True,
                   help="new directory for immutable runs and sweep.csv")
    p.add_argument("--reference-archive", type=Path,
                   help="optional FR079 reference-map tgz for geometric agreement")
    p.add_argument("--reference-cm", type=int, choices=(3, 5, 10), default=5)
    p.add_argument("--truncations", nargs="+",
                   default=[str(x) for x in DEFAULT_TRUNCATIONS])
    p.add_argument("--docker", action="store_true",
                   help="use 'docker compose run test' instead of a local binary")
    p.add_argument("--binary", type=Path, help="native slam_fr079_offline path")
    p.add_argument("--continuous-cost", action="store_true",
                   help="finite Ceres residuals through unknown/truncated EDT cells")
    p.add_argument("--support-aware", action="store_true",
                   help="opt-in certified Ceres local matcher, rather than legacy")
    p.add_argument("--map-stride", type=int, choices=(1, 3), default=3,
                   help="beam stride for occupancy insertion (1 or 3)")
    p.add_argument("--capture", action="store_true",
                   help="also run bounded perturbation capture trials per configuration")
    p.add_argument("--capture-every", type=int, default=120)
    p.add_argument("--capture-max-frames", type=int, default=12)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    args.truncations = truncations(args.truncations)
    if args.continuous_cost and args.support_aware:
        p.error("--continuous-cost and --support-aware cannot be combined")
    if args.docker and args.binary:
        p.error("choose --docker or --binary, not both")
    if not args.docker and not args.binary:
        p.error("pass --docker or --binary")
    if args.capture and not (1 <= args.capture_every <= 200 and
                             1 <= args.capture_max_frames <= 100):
        p.error("invalid capture sampling settings")
    if args.capture and not any(abs(t - 0.7) < 1e-9 for t in args.truncations):
        p.error("--capture requires 0.7 m in --truncations as the fixed anchor run")
    args.repo = args.repo.resolve(strict=True)
    args.log = args.log.resolve(strict=True)
    args.out_dir = args.out_dir.resolve()
    if args.reference_archive:
        args.reference_archive = args.reference_archive.resolve(strict=True)
    if args.binary:
        args.binary = args.binary.resolve(strict=True)
    if args.out_dir.exists():
        p.error("output directory already exists; use a new directory")
    return args


def command(args, tag, truncation, prefix):
    options = ["--match-every", "1", "--field-every", "15",
               "--map-stride", str(args.map_stride), "--match-stride", "4",
               "--distance-truncation", format(truncation, ".3f")]
    if args.support_aware:
        options.append("--support-aware")
    if args.continuous_cost:
        options.append("--continuous-cost")
    if args.capture and abs(truncation - 0.7) < 1e-9:
        options += ["--capture", "--capture-every", str(args.capture_every),
                    "--capture-max-frames", str(args.capture_max_frames)]
    if args.docker:
        return ["docker", "compose", "run", "--rm",
                "-v", f"{args.log}:/input/fr079.log.gz:ro",
                "-v", f"{args.out_dir}:/output",
                "--entrypoint", "./build-docker/slam_fr079_offline", "test",
                "/input/fr079.log.gz", f"/output/{tag}", *options]
    return [str(args.binary), str(args.log), str(prefix), *options]


def execute(command_line, cwd):
    print("+", " ".join(command_line), flush=True)
    subprocess.run(command_line, cwd=cwd, check=True)


def main():
    args = parse_args()
    runs = []
    for value in args.truncations:
        tag = "fr079-trunc-" + str(int(value * 1000 + 0.5)).zfill(4) + "mm"
        prefix = args.out_dir / tag
        for suffix in [".slammap", ".report.json", ".matches.json"]:
            if prefix.with_suffix(suffix).exists():
                raise ValueError("refusing to overwrite " + str(prefix.with_suffix(suffix)))
        runs.append((value, tag, prefix))
    if args.dry_run:
        for value, tag, prefix in runs:
            print(" ".join(command(args, tag, value, prefix)))
        return 0
    args.out_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    source_sha = None
    columns = [
        "truncation_m", "matcher", "map_beam_stride", "source_sha256", "map_fingerprint",
        "frames", "attempts", "accepted", "rejected",
        "ceres_invalid_residual_evaluations",
        "ceres_continuous_unknown_evaluations",
        "ceres_continuous_truncated_evaluations",
        "ceres_continuous_outside_evaluations",
        "ceres_unsuccessful_steps",
        "offline_fps", "frame_p95_ms", "match_p95_ms",
        "wall_f1_15cm", "symmetric_mean_wall_m", "generated_wall_p95_m",
        "occupied_iou", "reference_agreement_is_not_ground_truth",
        "slammap", "report", "comparison_json", "capture_csv"
    ]
    for value, tag, prefix in runs:
        execute(command(args, tag, value, prefix), args.repo)
        report_file = Path(str(prefix) + ".report.json")
        report = json.loads(report_file.read_text())
        if report["format"] != "FUSFR079SLAM1":
            raise ValueError("unexpected FR079 report format")
        if abs(report["distance_truncation_m"] - value) > 1e-9:
            raise ValueError("truncation was not applied")
        if report["match_every"] != 1 or report["field_rebuild_every"] != 15:
            raise ValueError("benchmark matching parameters unexpectedly changed")
        if report["map_beam_stride"] != args.map_stride or report["match_beam_stride"] != 4:
            raise ValueError("benchmark beam parameters unexpectedly changed")
        if source_sha is None:
            source_sha = report["source_sha256"]
        if report["source_sha256"] != source_sha:
            raise ValueError("CARMEN source mismatch between runs")
        invalid_breakdown = sum(report["ceres_invalid_" + reason] for reason in
                                ("unknown_cells", "truncated", "outside_grid", "other"))
        if invalid_breakdown != report["ceres_invalid_residual_evaluations"]:
            raise ValueError("Ceres invalid-evaluation breakdown does not reconcile")
        expected_matcher = ("ceres_continuous_cost" if args.continuous_cost
                            else "ceres_support_aware" if args.support_aware
                            else "ceres")
        if report["matcher"] != expected_matcher:
            raise ValueError("unexpected frontend matcher: " + str(report["matcher"]))
        comparison = {}
        comparison_file = ""
        if args.reference_archive:
            cmp_prefix = args.out_dir / (tag + "-reference")
            execute([sys.executable,
                     str(args.repo / "tools/fr079_map_compare.py"),
                     "--repo", str(args.repo),
                     "--slammap", str(prefix) + ".slammap",
                     "--reference-archive", str(args.reference_archive),
                     "--reference-cm", str(args.reference_cm),
                     "--out", str(cmp_prefix)], args.repo)
            comparison_file = str(cmp_prefix) + ".json"
            comparison = json.loads(Path(comparison_file).read_text())
            if comparison["independent_ground_truth"] is not False:
                raise ValueError("invalid reference-provenance claim")
        capture_file = (str(prefix) + ".capture.csv"
                        if args.capture and abs(value - 0.7) < 1e-9 else "")
        if capture_file:
            with open(capture_file, newline="") as h:
                if not next(csv.DictReader(h), None):
                    raise ValueError("no accepted snapshot anchors for capture benchmark")
        row = {
            "truncation_m": value,
            "matcher": report["matcher"],
            "map_beam_stride": args.map_stride,
            "source_sha256": source_sha,
            "map_fingerprint": report["map"]["fingerprint"],
            "frames": report["frames"],
            "attempts": report["attempts"],
            "accepted": report["accepted"],
            "rejected": report["rejected"],
            "ceres_invalid_residual_evaluations":
                report["ceres_invalid_residual_evaluations"],
            "ceres_continuous_unknown_evaluations":
                report["ceres_continuous_unknown_evaluations"],
            "ceres_continuous_truncated_evaluations":
                report["ceres_continuous_truncated_evaluations"],
            "ceres_continuous_outside_evaluations":
                report["ceres_continuous_outside_evaluations"],
            "ceres_unsuccessful_steps": report["ceres_unsuccessful_steps"],
            "offline_fps": report["profiling"]["estimated_offline_fps"],
            "frame_p95_ms": report["profiling"]["frame_latency"]["p95_ms"],
            "match_p95_ms": report["profiling"]["scan_matching"].get("p95_ms"),
            "wall_f1_15cm":
                comparison.get("wall_metrics", {}).get("wall_f1_within_0p15m"),
            "symmetric_mean_wall_m":
                comparison.get("wall_metrics", {}).get("symmetric_mean_m"),
            "generated_wall_p95_m":
                comparison.get("wall_metrics", {}).get(
                    "generated_to_reference", {}).get("p95_m"),
            "occupied_iou":
                comparison.get("occupancy_metrics", {}).get(
                    "occupied_iou_on_joint_known_cells"),
            "reference_agreement_is_not_ground_truth": True,
            "slammap": str(prefix) + ".slammap",
            "report": str(report_file),
            "comparison_json": comparison_file,
            "capture_csv": capture_file,
        }
        rows.append(row)
        # Keep a partial scorecard after each complete run; no re-run on resume.
        with (args.out_dir / "sweep.csv").open("w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        print("Completed", tag, "wall F1", row["wall_f1_15cm"],
              "invalid residuals", row["ceres_invalid_residual_evaluations"])
    with (args.out_dir / "manifest.json").open("w") as out:
        json.dump({
            "format": "FUSFR079TRUNCSWEEP1",
            "source_sha256": source_sha,
            "reference_is_independent_ground_truth": False,
            "parameters_held_fixed": {
                "match_every": 1, "field_rebuild_every": 15,
                "map_beam_stride": args.map_stride, "match_beam_stride": 4
            },
            "capture_anchor": "0.7m_run_same_run_accepted_pose_proxy_not_ground_truth"
                              if args.capture else None,
            "results": rows,
        }, out, indent=2)
    print("Scorecard:", args.out_dir / "sweep.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
