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
    required = {"format", "matcher", "source_sha256", "replay_sha256", "map_name",
                "map_fingerprint", "map_frame_id", "map_revision", "map_origin_verified",
                "independent_ground_truth", "training_independence_verified", "per_beam_time_verified",
                "frames"}
    if (not isinstance(result, dict) or
        set(result) not in (required, required | {"selection", "query_stride", "beam_stride"}) or
        result["format"] != "FUSMATCH1" or result["matcher"] != "ceres_scan_to_edt"):
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
    selection=result.get("selection", "sampled_unrestricted")
    if selection not in ("sampled_unrestricted", "fr079_heldout_mod5_verified"):
        raise ReplayError("invalid Ceres selection protocol")
    if selection == "fr079_heldout_mod5_verified":
        # The source ID and scan-index split identify the reproducible
        # training-map recipe. The native producer additionally recomputes
        # and fingerprint-checks the actual training map.
        if (map_metadata.get("source_id") != "fr079-training-index-mod5-mod6-v1" or
            result["map_frame_id"] != "fr079_logged_laser_frame_UNVERIFIED" or
            result["map_origin_verified"] is not False):
            raise ReplayError("held-out evaluation references a non-training map")
    if "selection" in result:
        for key, maximum in (("query_stride", 128), ("beam_stride", 64)):
            if type(result.get(key)) is not int or not 1 <= result[key] <= maximum:
                raise ReplayError("invalid Ceres sampling parameters")
    rows = result["frames"]
    if not isinstance(rows, list) or len(rows) > len(session["frames"]):
        raise ReplayError("invalid Ceres row count")
    if selection == "fr079_heldout_mod5_verified":
        if not rows or [r.get("index") for r in rows if isinstance(r, dict)] != list(range(0, len(session["frames"]), 5 * result["query_stride"])):
            raise ReplayError("held-out coverage/stride differs from native selection")
    elif "selection" in result and [r.get("index") for r in rows if isinstance(r,dict)] != list(range(0,len(session["frames"]), result["query_stride"])):
        raise ReplayError("Ceres sampling stride differs from native selection")
    prev = -1
    for item in rows:
        if not isinstance(item, dict) or set(item) != {"index", "t", "seed", "estimate", "status", "iterations", "input_hits", "supported_hits", "overlap", "initial_robust_cost", "robust_cost", "mean_distance_m", "full_rank", "map_fingerprint"}:
            raise ReplayError("invalid Ceres match record")
        idx = item["index"]
        if type(idx) is not int or not prev < idx < len(session["frames"]):
            raise ReplayError("invalid Ceres frame index")
        prev=idx
        if selection == "fr079_heldout_mod5_verified" and idx % 5 != 0:
            raise ReplayError("held-out evaluation includes a training-eligible scan")
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


def summarize(result: dict) -> dict:
    """Descriptive held-out geometry diagnostics; never ATE/RPE or independent truth."""
    if result.get("selection") != "fr079_heldout_mod5_verified":
        return {"mode": "unrestricted", "training_split_checked": False}
    rows=result["frames"]
    counts={status: sum(row["status"]==status for row in rows) for status in sorted(STATUSES)}
    n=len(rows)
    return {"mode": "fr079_heldout_mod5", "training_split_checked": True,
            "excluded_from_training": True, "ground_truth_available": False,
            "per_beam_time_verified": False, "map_alignment_verified": False,
            "query_stride": result.get("query_stride"), "beam_stride": result.get("beam_stride"),
            "evaluated_scans": n, "accepted": counts["accepted"],
            "acceptance_fraction": counts["accepted"]/n if n else None,
            "status_counts": counts,
            "mean_initial_robust_cost": sum(row["initial_robust_cost"] for row in rows)/n if n else None,
            "mean_final_robust_cost": sum(row["robust_cost"] for row in rows)/n if n else None,
            "mean_overlap": sum(row["overlap"] for row in rows)/n if n else None,
            "mean_edt_distance_m": (sum(row["mean_distance_m"] for row in rows if row["mean_distance_m"] is not None)/
                                    sum(row["mean_distance_m"] is not None for row in rows)
                                    if any(row["mean_distance_m"] is not None for row in rows) else None),
            "quality_metric": "scan-to-frozen-map EDT geometry; no position error"}
