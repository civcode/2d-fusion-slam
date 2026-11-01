#!/usr/bin/env python3
"""Local, manual-only SLAM fusion benchmark; no Actions or pip dependencies.

Reports both external process wall time (includes simulation and IO) and the
instrumented fused-processing critical-path proxy (excludes baselines and IO).
Run the exact same binary, scene and host repeatedly; compare profiled and
unprofiled CMake builds to estimate probe overhead.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

FIELDS = (
    "scene", "trial", "frames", "process_wall_ms", "profiled",
    "input_rate_fps", "fusion_compute_ms", "effective_processing_fps",
    "frame_proxy_p50_ms", "frame_proxy_p95_ms", "frame_proxy_max_ms",
    "realtime_factor", "average_offline_budget_met", "matcher",
    "heldout_accepted", "heldout_attempted", "slowest_critical_stage",
    "slowest_critical_stage_ms",
)
CRITICAL = {
    "sensor_integration", "scan_deskew", "training_map_build",
    "heldout_scan_match", "fused_map_build", "graph_optimize",
}


def quantile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[math.ceil(fraction * len(ordered)) - 1]


def bench(binary: Path, scene: str, root: Path, trial: int) -> dict:
    run_dir = root / f"{scene}-{trial:04}"
    start = time.perf_counter()
    result = subprocess.run(
        [str(binary), scene, str(run_dir)], capture_output=True,
        text=True, check=False, timeout=600,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000
    if result.returncode:
        raise RuntimeError(
            f"{scene} trial {trial} exited {result.returncode}\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}"
        )
    profile = json.loads((run_dir / "fusion_profile.json").read_text())
    fusion = json.loads((run_dir / "fusion_report.json").read_text())
    if (profile["format"] != "FUSPROFILE1" or
            profile["scene"] != fusion["scene"] or
            profile["frames"] != fusion["frames"]):
        raise RuntimeError("profiling and fusion report identity mismatch")
    stages = profile["stages"]
    critical = sorted(
        ((stage, entry["total_ms"]) for stage, entry in stages.items()
         if stage in CRITICAL),
        key=lambda entry: entry[1], reverse=True,
    )
    top_name, top_ms = critical[0] if critical else ("unprofiled", None)
    return dict(
        scene=scene, trial=trial, frames=profile["frames"],
        process_wall_ms=elapsed_ms, profiled=profile["profiled"],
        input_rate_fps=profile["input_rate_fps"],
        fusion_compute_ms=profile["fusion_compute_ms"],
        effective_processing_fps=profile["effective_processing_fps"],
        frame_proxy_p50_ms=profile["frame_proxy_p50_ms"],
        frame_proxy_p95_ms=profile["frame_proxy_p95_ms"],
        frame_proxy_max_ms=profile["frame_proxy_max_ms"],
        realtime_factor=profile["realtime_factor"],
        average_offline_budget_met=profile["average_offline_budget_met"],
        matcher=profile["matcher"],
        heldout_accepted=fusion["heldout_accepted"],
        heldout_attempted=fusion["heldout_attempted"],
        slowest_critical_stage=top_name,
        slowest_critical_stage_ms=top_ms,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--scene", choices=("apartment", "warehouse", "both"),
                        default="both")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--csv", type=Path, default=None,
                        help="Create new CSV (fails if already present)")
    args = parser.parse_args()
    if not 1 <= args.runs <= 100 or not 0 <= args.warmup <= 20:
        parser.error("--runs must be 1..100 and --warmup must be 0..20")
    binary = args.binary.resolve(strict=True)
    scenes = ["apartment", "warehouse"] if args.scene == "both" else [args.scene]
    rows = []
    with tempfile.TemporaryDirectory(prefix="slam-fusion-bench-") as temp:
        root = Path(temp)
        for scene in scenes:
            for trial in range(-args.warmup, args.runs):
                row = bench(binary, scene, root,
                            trial if trial >= 0 else trial - 1000)
                if trial >= 0:
                    rows.append(row)
                    print(f"{scene} #{trial+1}: {row['process_wall_ms']:.2f} "
                          f"ms process; fusion={row['fusion_compute_ms']} ms; "
                          f"fps={row['effective_processing_fps']}", file=sys.stderr)
    if args.csv:
        with args.csv.open("x", newline="", encoding="utf8") as output:
            writer = csv.DictWriter(output, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        print(f"CSV: {args.csv}", file=sys.stderr)
    else:
        writer = csv.DictWriter(sys.stdout, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    for scene in scenes:
        group = [row for row in rows if row["scene"] == scene]
        wall = [row["process_wall_ms"] for row in group]
        print(f"{scene} process wall: p50={quantile(wall,.5):.2f} ms, "
              f"p95={quantile(wall,.95):.2f} ms "
              "(includes simulator, baselines, scoring and file output)",
              file=sys.stderr)
        fps = [row["effective_processing_fps"] for row in group
               if row["effective_processing_fps"] is not None]
        if fps:
            print(f"{scene} profiled offline fusion FPS: "
                  f"median={statistics.median(fps):.2f}, "
                  f"worst={min(fps):.2f}; "
                  f"required input={group[0]['input_rate_fps']:.3f} fps",
                  file=sys.stderr)
        else:
            print(f"{scene}: instrumentation disabled; use process wall "
                  "time to compare probe overhead", file=sys.stderr)
    print("NOTE: no live sensor deadlines, scheduler, thermal throttling "
          "or device I/O were verified.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"benchmark failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
