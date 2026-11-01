"""Validate optional FUSMATCH1 sidecars emitted by the real Ceres C++ CLI.

Sidecars are visualization-only: neither frame labels nor map fingerprints
prove calibration, independence, metric truth or scan acquisition times.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import stat

from .replay import ReplayError, finite, valid_pose

MAX_RESULTS = 4 * 1024 * 1024
STATUSES = {"accepted", "insufficient_support", "degenerate", "no_improvement", "invalid_input"}
SIDE_SUFFIX = ".ceres.json"


def validate_results(result, session, replay_sha: str, map_metadata: dict):
    if not isinstance(result, dict) or set(result) != {
            "format", "matcher", "source_sha256", "replay_sha256", "map_name",
            "map_fingerprint", "map_frame_id", "map_revision", "map_origin_verified",
            "independent_ground_truth", "training_independence_verified", "per_beam_time_verified",
            "frames"} or result["format"] != "FUSMATCH1" or result["matcher"] != "ceres_scan_to_edt":
        raise ReplayError("invalid Ceres result schema")
    if (result["source_sha256"] != session["source"]["source_sha256"] or
            result["replay_sha256"] != replay_sha):
        raise ReplayError("Ceres results do not match recorded session bytes/source")
    for field in ("independent_ground_truth", "training_independence_verified", "per_beam_time_verified"):
        if result[field] is not False:
            raise ReplayError("unjustified Ceres source claim")
    if type(result["map_origin_verified"]) is not bool or result["map_origin_verified"] != map_metadata["metric_origin_verified"]:
        raise ReplayError("Ceres map-origin metadata mismatch")
    for f in ("map_fingerprint", "map_frame_id", "map_revision"):
        if result[f] != map_metadata["fingerprint" if f=="map_fingerprint" else "frame_id" if f=="map_frame_id" else "revision"]:
            raise ReplayError("stale/incorrect Ceres source map")
    if result["map_frame_id"] != session["source"]["frame_id"]:
        raise ReplayError("Ceres result frame differs from recorded frame")
    if result["map_name"] != map_metadata["name"]:
        raise ReplayError("Ceres result references a different map")
    rows = result["frames"]
    if not isinstance(rows, list) or len(rows) > len(session["frames"]):
        raise ReplayError("invalid Ceres row count")
    prev = -1
    for item in rows:
        if not isinstance(item, dict) or set(item) != {"index", "t", "seed", "estimate", "status", "iterations", "input_hits", "supported_hits", "overlap", "initial_robust_cost", "robust_cost", "mean_distance_m", "full_rank", "map_fingerprint"}:
            raise ReplayError("invalid Ceres match record")
        idx = item["index"]
        if type(idx) is not int or not prev < idx < len(session["frames"]):
            raise ReplayError("invalid Ceres frame index")
        prev=idx
        original=session["frames"][idx]
        if not finite(item["t"]) or abs(original["t"]-item["t"]) > 1e-6:
            raise ReplayError("Ceres timestamp does not match recording")
        if not valid_pose(item["seed"]) or not valid_pose(item["estimate"]):
            raise ReplayError("invalid Ceres pose")
        if any(abs(a-b)>1e-6 for a,b in zip(item["seed"],original["pose"])):
            raise ReplayError("Ceres seed does not match logged laser pose")
        if item["status"] not in STATUSES or item["map_fingerprint"] != result["map_fingerprint"]:
            raise ReplayError("unrecognized match status or source")
        if type(item["iterations"]) is not int or not 0 <= item["iterations"] <= 1000:
            raise ReplayError("invalid Ceres iterations")
        for field in ("input_hits", "supported_hits"):
            if type(item[field]) is not int or not 0 <= item[field] <= 4096:
                raise ReplayError("invalid match hit count")
        if item["supported_hits"] > item["input_hits"] or not 0 <= item["overlap"] <= 1 or not finite(item["overlap"]):
            raise ReplayError("invalid Ceres overlap")
        expected_overlap = item["supported_hits"] / item["input_hits"] if item["input_hits"] else 0.0
        if abs(item["overlap"] - expected_overlap) > 1e-9:
            raise ReplayError("mismatched overlap count")
        for field in ("initial_robust_cost", "robust_cost"):
            if not finite(item[field]) or item[field] < 0:
                raise ReplayError("invalid Ceres robust cost")
        if item["mean_distance_m"] is not None and (not finite(item["mean_distance_m"]) or item["mean_distance_m"] < 0):
            raise ReplayError("invalid Ceres mean distance")
        if type(item["full_rank"]) is not bool:
            raise ReplayError("invalid rank flag")
        if item["status"] == "accepted":
            if not item["full_rank"] or item["supported_hits"] < 12 or item["overlap"] < .65 or item["robust_cost"] >= item["initial_robust_cost"]:
                raise ReplayError("accepted Ceres match fails acceptance contract")
        elif any(abs(a-b)>1e-6 for a,b in zip(item["estimate"],item["seed"])):
            raise ReplayError("rejected Ceres match must retain the logged seed")
    return result


def load_results(sidecar: Path, session_path: Path, session, map_name: str, map_bundle):
    st=sidecar.lstat()
    if not stat.S_ISREG(st.st_mode) or st.st_size>MAX_RESULTS:
        raise ReplayError("Ceres results must be bounded regular files")
    payload=sidecar.read_bytes()
    if len(payload)>MAX_RESULTS:
        raise ReplayError("oversized Ceres results")
    try:
        result=json.loads(payload,parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
    except (ValueError,UnicodeError,RecursionError) as err:
        raise ReplayError("invalid Ceres sidecar JSON") from err
    metadata=dict(map_bundle.metadata())
    metadata["name"]=map_name
    return validate_results(result, session, hashlib.sha256(session_path.read_bytes()).hexdigest(), metadata)
