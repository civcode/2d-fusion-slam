#!/usr/bin/env python3
"""FR079 closed-loop Ceres factorial ablation and matching-beam/phase sweep.

All runs read the identical log, insert the same mapping-beam stride, and
change only matching mode, matching stride, or beam-selection phase.
Comparisons to FR079's supplied maps are correlated proxies, never truth.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

MODES = ("legacy", "finite-seed-legacy", "finite-all-legacy",
         "finite-seed-continuous", "continuous")
FIELDS = (
    "tag", "mode", "stride", "phase", "match_input_beams",
    "solver_residual_blocks", "seed_supported_hits",
    "candidate_supported_hits", "candidate_evaluated", "status", "accepted",
    "seed_strict_cost", "candidate_strict_cost", "strict_delta",
    "seed_finite_cost", "candidate_finite_cost", "finite_delta",
    "candidate_translation_m", "candidate_rotation_rad",
    "ceres_invalid_residual_evaluations", "ceres_continuous_unknown_evaluations",
)


def args_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo", type=Path, default=Path.cwd())
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True,
                   help="new output directory; existing output never overwritten")
    p.add_argument("--reference-archive", type=Path)
    p.add_argument("--reference-cm", type=int, choices=(3, 5, 10), default=5)
    backend = p.add_mutually_exclusive_group(required=True)
    backend.add_argument("--docker", action="store_true")
    backend.add_argument("--binary", type=Path)
    p.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    p.add_argument("--match-strides", nargs="+", type=int, default=[1, 2, 4, 8])
    p.add_argument("--phases", nargs="+", type=int, default=[0],
                   help="0-based original-beam offsets; invalid stride/phase pairs skipped")
    p.add_argument("--map-stride", type=int, default=1)
    p.add_argument("--field-every", type=int, default=15)
    p.add_argument("--distance-truncation", type=float, default=0.5)
    p.add_argument("--dry-run", action="store_true")
    return p


def validate(a, p):
    for name, values in (("modes", a.modes), ("match-strides", a.match_strides),
                         ("phases", a.phases)):
        if len(set(values)) != len(values):
            p.error(f"duplicate {name}")
    if any(s < 1 or s > 64 for s in a.match_strides):
        p.error("match strides must be in [1,64]")
    if any(phase < 0 or phase > 63 for phase in a.phases):
        p.error("phases must be in [0,63]")
    if not any(phase < stride for stride in a.match_strides for phase in a.phases):
        p.error("no valid stride/phase configurations")
    if not (1 <= a.map_stride <= 64 and 1 <= a.field_every <= 200):
        p.error("invalid mapping stride or field cadence")
    if not math.isfinite(a.distance_truncation) or not (
            0.2 <= a.distance_truncation <= 3.0):
        p.error("distance truncation outside [0.2,3.0]")
    a.repo = a.repo.resolve(strict=True)
    a.log = a.log.resolve(strict=True)
    if a.reference_archive:
        a.reference_archive = a.reference_archive.resolve(strict=True)
    if a.binary:
        a.binary = a.binary.resolve(strict=True)
    a.out_dir = a.out_dir.resolve()
    if a.out_dir.exists():
        p.error("output directory already exists; use a new one")


def command(a, tag, mode, stride, phase):
    options = ["--match-every", "1", "--field-every", str(a.field_every),
               "--map-stride", str(a.map_stride),
               "--match-stride", str(stride), "--match-phase", str(phase),
               "--distance-truncation", f"{a.distance_truncation:.3f}"]
    if mode != "legacy":
        options.append("--continuous-cost")
        if mode != "continuous":
            options += ["--ablation", mode]
    if a.docker:
        return ["docker", "compose", "run", "--rm",
                "-v", f"{a.log}:/input/fr079.log.gz:ro",
                "-v", f"{a.out_dir}:/output",
                "-e", f"SLAM_HOST_OUTPUT_DIR={a.out_dir}",
                "--entrypoint", "./build-docker/slam_fr079_offline", "test",
                "/input/fr079.log.gz", f"/output/{tag}", *options]
    return [str(a.binary), str(a.log), str(a.out_dir / tag), *options]


def execute(cmd, cwd, logfile=None):
    print("+", " ".join(cmd), flush=True)
    if logfile is None:
        subprocess.run(cmd, cwd=cwd, check=True)
        return
    # Legacy Ceres can emit hundreds of thousands of repeated warnings.
    # Keep the complete evidence without flooding the terminal.
    with logfile.open("w", encoding="utf-8") as out:
        process = subprocess.run(cmd, cwd=cwd, stdout=out,
                                 stderr=subprocess.STDOUT, check=False)
    if process.returncode:
        tail = "\n".join(logfile.read_text(errors="replace").splitlines()[-30:])
        raise RuntimeError(f"run failed ({process.returncode}), see {logfile}:\n{tail}")


def write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def audit_frames(tag, mode, stride, phase, frames, poses):
    if len(frames) != len(poses):
        raise ValueError(f"{tag}: matches and trajectory length mismatch")
    result = []
    for i, (frame, pose) in enumerate(zip(frames, poses)):
        if frame["index"] != i or pose["index"] != i:
            raise ValueError(f"{tag}: out-of-order frames")
        attempted = frame["attempted"]
        n = frame["match_input_beams"]
        blocks = frame["solver_residual_blocks"]
        if not (0 <= blocks <= n <= math.ceil((360-phase)/stride)):
            raise ValueError(f"{tag}: invalid residual cohort or beam count at {i}")
        if frame["accepted"] and not frame["candidate_evaluated"]:
            raise ValueError(f"{tag}: accepted correction with no candidate at {i}")
        if attempted and mode != "legacy" and frame["ceres_invalid_residual_evaluations"]:
            raise ValueError(f"{tag}: continuous solver emitted invalid residuals")
        candidate = frame["candidate_evaluated"]
        row = dict(tag=tag, mode=mode, stride=stride, phase=phase,
                   match_input_beams=n, solver_residual_blocks=blocks,
                   candidate_evaluated=candidate, status=frame["status"],
                   accepted=int(frame["accepted"]))
        for k in ("seed_supported_hits", "candidate_supported_hits",
                  "seed_strict_cost", "candidate_strict_cost",
                  "seed_finite_cost", "candidate_finite_cost",
                  "candidate_translation_m", "candidate_rotation_rad",
                  "ceres_invalid_residual_evaluations",
                  "ceres_continuous_unknown_evaluations"):
            row[k] = frame[k] if candidate or k in (
                "seed_supported_hits", "seed_strict_cost", "seed_finite_cost",
                "ceres_invalid_residual_evaluations",
                "ceres_continuous_unknown_evaluations") else ""
        row["strict_delta"] = (frame["candidate_strict_cost"] -
                               frame["seed_strict_cost"]) if candidate else ""
        row["finite_delta"] = (frame["candidate_finite_cost"] -
                               frame["seed_finite_cost"]) if candidate else ""
        row["index"] = i
        row["prior"] = pose["prior"]
        row["estimated"] = pose["estimated"]
        result.append(row)
    return result


def score(a, prefix, tag):
    if not a.reference_archive:
        return {}
    cmd = [sys.executable, str(a.repo / "tools/fr079_map_compare.py"),
           "--repo", str(a.repo), "--slammap", str(prefix) + ".slammap",
           "--reference-archive", str(a.reference_archive),
           "--reference-cm", str(a.reference_cm),
           "--out", str(a.out_dir / (tag + "-reference"))]
    execute(cmd, a.repo)
    report = json.loads((a.out_dir / (tag + "-reference.json")).read_text())
    return {
        "wall_f1_15cm": report["wall_metrics"]["wall_f1_within_0p15m"],
        "symmetric_wall_mean_m": report["wall_metrics"]["symmetric_mean_m"],
        "occupied_iou": report["occupancy_metrics"]["occupied_iou_on_joint_known_cells"],
        "reference_wall_recall": report["occupancy_metrics"][
            "reference_occupied_recall_including_unknown_as_missed"],
        "known_reference_coverage": report["occupancy_metrics"][
            "overlap_known_fraction_of_reference_known"],
    }


def summary(a, tag, mode, stride, phase, prefix):
    report = json.loads(prefix.with_suffix(".report.json").read_text())
    matches = json.loads(prefix.with_suffix(".matches.json").read_text())
    trajectory = json.loads(prefix.with_suffix(".trajectory.json").read_text())
    if (report["match_beam_stride"] != stride or
            report["match_beam_phase"] != phase or
            report["map_beam_stride"] != a.map_stride or
            report["field_rebuild_every"] != a.field_every or
            report["match_every"] != 1 or
            abs(report["distance_truncation_m"]-a.distance_truncation) > 1e-9 or
            report["ablation_mode"] != (mode if mode not in
                                        ("legacy", "continuous") else "none")):
        raise ValueError(f"{tag}: parameters did not match the manifest")
    if not report["frames"] == len(matches["frames"]) == len(trajectory["poses"]):
        raise ValueError(f"{tag}: inconsistent frame counts")
    if not (report["accepted"]+report["rejected"] == report["attempts"]):
        raise ValueError(f"{tag}: invalid acceptance accounting")
    if (matches["source_sha256"] != report["source_sha256"] or
            trajectory["source_sha256"] != report["source_sha256"]):
        raise ValueError(f"{tag}: inconsistent input provenance")
    frames = audit_frames(tag, mode, stride, phase, matches["frames"],
                          trajectory["poses"])
    attempts = [f for f in frames if matches["frames"][f["index"]]["attempted"]]
    candidates = [f for f in attempts if f["candidate_evaluated"]]
    accepted = [f for f in candidates if f["accepted"]]
    sum_invalid = sum(f["ceres_invalid_residual_evaluations"] for f in frames)
    if sum_invalid != report["ceres_invalid_residual_evaluations"]:
        raise ValueError(f"{tag}: invalid residual accounting mismatch")
    row = {
        "tag": tag, "mode": mode, "stride": stride, "phase": phase,
        "beam_nominal_max": math.ceil((360-phase)/stride),
        "beam_mean": sum(f["match_input_beams"] for f in attempts)/max(1,len(attempts)),
        "residual_blocks_mean": sum(f["solver_residual_blocks"] for f in attempts)/max(1,len(attempts)),
        "source_sha256": report["source_sha256"],
        "frames": report["frames"], "attempts": report["attempts"],
        "accepted": report["accepted"],
        "accept_fraction": report["accepted"]/max(1,report["attempts"]),
        "candidates": len(candidates),
        "accepted_strict_cost_regressions": sum(f["strict_delta"] > 1e-10 for f in accepted),
        "accepted_finite_cost_regressions": sum(f["finite_delta"] > 1e-10 for f in accepted),
        "accepted_support_losses": sum(f["candidate_supported_hits"] <
                                       f["seed_supported_hits"] for f in accepted),
        "invalid_residuals": sum_invalid,
        "finite_unknown_evaluations": report["ceres_continuous_unknown_evaluations"],
        "offline_fps": report["profiling"]["estimated_offline_fps"],
        "frame_p95_ms": report["profiling"]["frame_latency"]["p95_ms"],
        "matching_p95_ms": report["profiling"]["scan_matching"]["p95_ms"],
    }
    row.update(score(a, prefix, tag))
    return row, frames


def first_divergence(left, right):
    """Only the first differing frame has a common preceding SLAM trajectory."""
    for l, r in zip(left, right):
        if l["index"] != r["index"]:
            raise ValueError("unaligned paired frames")
        if l["prior"] != r["prior"]:
            return l["index"], "different_prior"
        if l["accepted"] != r["accepted"] or any(
                abs(float(x)-float(y)) > 1e-8
                for x, y in zip(l["estimated"], r["estimated"])):
            return l["index"], "first_correction_divergence"
    return None, "identical_trajectory"


def main():
    parser = args_parser()
    a = parser.parse_args()
    validate(a, parser)
    specs = [(f"{mode}-s{stride:02d}-p{phase:02d}", mode, stride, phase)
             for stride in a.match_strides for phase in a.phases if phase < stride
             for mode in a.modes]
    if a.dry_run:
        for tag, mode, stride, phase in specs:
            print("+", " ".join(command(a, tag, mode, stride, phase)))
        return
    a.out_dir.mkdir(parents=True)
    manifest = {
        "format": "FUSFR079MATCHABLATION1",
        "source_sha256_compressed_file": hashlib.sha256(a.log.read_bytes()).hexdigest(),
        "log": str(a.log), "reference_archive": str(a.reference_archive) if a.reference_archive else None,
        "reference_is_independent_ground_truth": False,
        "closed_loop_divergence_warning": "After first different correction, future maps and seeds are not paired",
        "modes": a.modes, "strides": a.match_strides, "phases": a.phases,
        "map_stride": a.map_stride, "field_every": a.field_every,
        "distance_truncation_m": a.distance_truncation,
        "commands": [], "runs": [],
    }
    summaries, audits, pairs = [], {}, []
    for tag, mode, stride, phase in specs:
        cmd = command(a, tag, mode, stride, phase)
        manifest["commands"].append(cmd)
        execute(cmd, a.repo, a.out_dir / (tag + ".run.log"))
        row, frames = summary(a, tag, mode, stride, phase, a.out_dir / tag)
        summaries.append(row)
        audits[(mode, stride, phase)] = frames
        manifest["runs"].append(tag)
        print("Completed", tag, "accepted", row["accepted"],
              "wall F1", row.get("wall_f1_15cm"), flush=True)
    if len({r["source_sha256"] for r in summaries}) != 1:
        raise ValueError("runs did not all use identical FR079 content")
    for stride in a.match_strides:
        for phase in a.phases:
            if phase >= stride:
                continue
            for left, right in zip(a.modes, a.modes[1:]):
                i, why = first_divergence(audits[(left,stride,phase)],
                                           audits[(right,stride,phase)])
                pairs.append(dict(stride=stride, phase=phase, left=left,
                                  right=right, first_divergent_index=i,
                                  reason=why))
    write_csv(a.out_dir/"scorecard.csv", summaries, list(summaries[0]))
    audit_rows = [f for seq in audits.values() for f in seq]
    write_csv(a.out_dir/"frame-audit.csv", audit_rows,
              ["index", *FIELDS, "prior", "estimated"])
    if pairs:
        write_csv(a.out_dir/"first-divergence.csv", pairs, list(pairs[0]))
    (a.out_dir/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("Scorecard:", a.out_dir/"scorecard.csv")


if __name__ == "__main__":
    main()
