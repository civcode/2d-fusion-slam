"""Bounded, read-only FUSREPLAY1 data and deterministic CARMEN FLASER conversion.

FR079 logged laser poses are estimates, NOT calibrated map alignment or truth.
Scan matching metrics are absent here: native Ceres writes a separate sidecar.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import stat

MAGIC = "FUSREPLAY1"
SUFFIX = ".slamreplay.json"
SAFE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,140}\.slamreplay\.json\Z")
MAX_BYTES = 32 * 1024 * 1024
MAX_FRAMES = 6000
MAX_POINTS = 720
MAX_BEAMS = 4096
MAX_LINE = 1024 * 1024


class ReplayError(ValueError):
    """Refused malformed or unsupported recorded-session data."""


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def valid_pose(value):
    return isinstance(value, list) and len(value) == 3 and all(finite(v) and abs(v) <= 1_000_000 for v in value)


def validate(data):
    if not isinstance(data, dict) or set(data) != {"format", "source", "frames"} or data["format"] != MAGIC:
        raise ReplayError("unsupported replay format")
    source = data["source"]
    if not isinstance(source, dict):
        raise ReplayError("invalid replay provenance")
    for key in ("dataset", "frame_id", "pose_kind", "timestamp_kind", "source_sha256"):
        if not isinstance(source.get(key), str) or not 1 <= len(source[key]) <= 256:
            raise ReplayError("missing replay provenance: " + key)
    if source["pose_kind"] not in ("logged_laser_estimate_not_ground_truth", "synthetic_exact"):
        raise ReplayError("unrecognized pose provenance")
    if source["timestamp_kind"] not in ("CARMEN_IPC_scan_level", "synthetic_scan_level"):
        raise ReplayError("unsupported clock provenance")
    if source.get("per_beam_time_known") is not False or source.get("map_alignment_verified") is not False:
        raise ReplayError("replay cannot claim beam timing or map alignment")
    if not re.fullmatch(r"[a-f0-9]{64}", source["source_sha256"]):
        raise ReplayError("invalid source SHA-256 provenance")
    frames = data["frames"]
    if not isinstance(frames, list) or not 1 <= len(frames) <= MAX_FRAMES:
        raise ReplayError("invalid replay frame count")
    last_t = -math.inf
    for idx, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != {"index", "t", "pose", "odom", "beam_count", "hit_count", "points", "matching"}:
            raise ReplayError("invalid replay frame schema")
        if type(frame["index"]) is not int or frame["index"] != idx:
            raise ReplayError("nonsequential replay index")
        if not finite(frame["t"]) or frame["t"] < last_t:
            raise ReplayError("invalid/nonmonotonic scan time")
        last_t = frame["t"]
        if not valid_pose(frame["pose"]) or not valid_pose(frame["odom"]):
            raise ReplayError("invalid recorded pose")
        n, hits, points = frame["beam_count"], frame["hit_count"], frame["points"]
        if type(n) is not int or type(hits) is not int or not 1 <= n <= MAX_BEAMS or not 0 <= hits <= n:
            raise ReplayError("invalid beam counts")
        if not isinstance(points, list) or len(points) > MAX_POINTS or len(points) > hits:
            raise ReplayError("invalid replay point count")
        if any(not isinstance(p, list) or len(p) != 2 or not all(finite(x) for x in p) for p in points):
            raise ReplayError("nonfinite or malformed scan point")
        # Diagnostics require a separately verified producer, not a guessed value.
        if frame["matching"] is not None:
            raise ReplayError("scan-matching diagnostics not supported by FUSREPLAY1 converter")
    return data


def read_replay(path: Path):
    st = path.lstat()
    if not stat.S_ISREG(st.st_mode) or st.st_size > MAX_BYTES:
        raise ReplayError("replays must be bounded regular files, not symlinks")
    with path.open("rb") as file:
        raw = file.read(MAX_BYTES + 1)
    if len(raw) > MAX_BYTES:
        raise ReplayError("replay exceeds 32 MiB")
    try:
        return validate(json.loads(raw))
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ReplayError("invalid replay JSON") from exc


def parse_flaser(line: str, *, stride: int):
    if len(line) > MAX_LINE:
        raise ReplayError("CARMEN record exceeds 1 MiB")
    parts = line.split()
    if not parts or parts[0] != "FLASER":
        return None
    try:
        count = int(parts[1])
        if not 1 <= count <= MAX_BEAMS or len(parts) != count + 11:
            raise ReplayError("invalid FLASER beam count or suffix")
        # CARMEN: FLASER N ranges... laser_pose(3) odom_pose(3) IPC host logger
        distances = [float(x) for x in parts[2:2 + count]]
        laser = [float(x) for x in parts[2 + count:5 + count]]
        odom = [float(x) for x in parts[5 + count:8 + count]]
        t = float(parts[8 + count])
        logger_t = float(parts[10 + count])
        if not parts[9 + count] or not all(map(math.isfinite, distances + laser + odom + [t, logger_t])):
            raise ReplayError("nonfinite CARMEN fields")
        hits = sum(0 < d <= 80.99 for d in distances)
        points = []
        for i in range(0, count, stride):
            d = distances[i]
            if 0 < d <= 80.99:
                angle = -math.pi / 2 + math.pi * i / count  # same as CarmenReader
                points.append([round(d * math.cos(angle), 5), round(d * math.sin(angle), 5)])
        return {"t": t, "pose": laser, "odom": odom, "beam_count": count,
                "hit_count": hits, "points": points, "matching": None}
    except (ValueError, IndexError, OverflowError) as exc:
        raise ReplayError("malformed CARMEN FLASER record") from exc


def convert_carmen(source: Path, stride: int = 6, max_frames: int = MAX_FRAMES,
                   frame_id: str = "carmen_logged_laser_frame_UNVERIFIED"):
    if not re.fullmatch(r"[A-Za-z0-9_./:-]{1,128}", frame_id):
        raise ReplayError("invalid declared frame ID")
    if not 1 <= stride <= 64 or not 1 <= max_frames <= MAX_FRAMES:
        raise ReplayError("invalid replay limits")
    if source.stat().st_size > 128 * 1024 * 1024:
        raise ReplayError("oversized CARMEN recording")
    digest = hashlib.sha256()
    with source.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    frames = []
    opener = gzip.open if source.suffix == ".gz" else open
    with opener(source, "rt", encoding="ascii", errors="strict") as log:
        for lineno, line in enumerate(log, 1):
            if lineno > 300_000:
                raise ReplayError("CARMEN log exceeds maximum record count")
            if len(line) > MAX_LINE:
                raise ReplayError(f"CARMEN line {lineno}: too long")
            try:
                frame = parse_flaser(line, stride=stride)
            except ReplayError as exc:
                raise ReplayError(f"CARMEN line {lineno}: {exc}") from exc
            if frame is None:
                continue
            if len(frames) >= max_frames:
                raise ReplayError("recording exceeds requested maximum frame count")
            frame["index"] = len(frames)
            frames.append(frame)
    data = {"format": MAGIC, "source": {
        "dataset": "CARMEN FLASER recording", "frame_id": frame_id,
        "pose_kind": "logged_laser_estimate_not_ground_truth", "timestamp_kind": "CARMEN_IPC_scan_level",
        "per_beam_time_known": False, "map_alignment_verified": False,
        "source_sha256": digest.hexdigest()}, "frames": frames}
    validate(data)
    if len(json.dumps(data, separators=(",", ":")).encode()) > MAX_BYTES:
        raise ReplayError("converted session exceeds 32 MiB")
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description="Convert CARMEN FLASER to bounded, read-only FUSREPLAY1 JSON")
    parser.add_argument("source", type=Path, help="CARMEN .log or .log.gz")
    parser.add_argument("output", type=Path, help="output .slamreplay.json (must not already exist)")
    parser.add_argument("--beam-stride", type=int, default=6)
    parser.add_argument("--frame-id", default="carmen_logged_laser_frame_UNVERIFIED",
                        help="descriptive source-frame name, NOT calibration or map registration")
    opts = parser.parse_args(argv)
    if not SAFE_NAME.fullmatch(opts.output.name):
        parser.error("output filename must end in .slamreplay.json")
    data = convert_carmen(opts.source, stride=opts.beam_stride, frame_id=opts.frame_id)
    payload = json.dumps(data, ensure_ascii=True, separators=(",", ":"), allow_nan=False).encode()
    # Exclusive creation: never replace a session or an authoritative map.
    with opts.output.open("xb") as out:
        out.write(payload)
    print(f"Exported {len(data['frames'])} scan frames ({len(payload)} bytes); poses are logged estimates, NOT ground truth")


if __name__ == "__main__":
    main()


def create_synthetic_replay():
    """Schematic scan sequence for an empty demo, NEVER a real recording."""
    from hashlib import sha256
    frames = []
    # A synthetic oval path, bounded inside the synthetic workshop.
    for i in range(56):
        angle = 2 * math.pi * i / 56
        x, y = 6 + 2.25 * math.cos(angle), 4.5 + 1.55 * math.sin(angle)
        yaw = math.atan2(1.55 * math.cos(angle), -2.25 * math.sin(angle))
        points = []
        for beam in range(64):
            a = yaw - math.pi / 2 + math.pi * beam / 64
            dx, dy = math.cos(a), math.sin(a)
            choices = []
            for wall, origin, d, other, slope, low, high in (
                    (0.7, x, dx, y, dy, 0.7, 8.3),
                    (11.3, x, dx, y, dy, 0.7, 8.3),
                    (0.7, y, dy, x, dx, 0.7, 11.3),
                    (8.3, y, dy, x, dx, 0.7, 11.3)):
                if abs(d) > 1e-10:
                    t = (wall - origin) / d
                    if t > 0 and low <= other + t * slope <= high:
                        choices.append(t)
            dist = min(choices) if choices else 5.0
            points.append([round(dist * math.cos(a - yaw), 5), round(dist * math.sin(a - yaw), 5)])
        frames.append({"index": i, "t": round(i * 0.12, 4), "pose": [x, y, yaw],
                       "odom": [x, y, yaw], "beam_count": 64, "hit_count": 64,
                       "points": points, "matching": None})
    source = {"dataset": "SYNTHETIC virtual workshop (not recorded LiDAR)",
              "frame_id": "synthetic_demo_frame", "pose_kind": "synthetic_exact",
              "timestamp_kind": "synthetic_scan_level", "per_beam_time_known": False,
              "map_alignment_verified": False,
              "source_sha256": sha256(b"synthetic virtual workshop replay v1").hexdigest()}
    return validate({"format": MAGIC, "source": source, "frames": frames})
