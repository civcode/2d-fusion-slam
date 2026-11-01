"""Read-only, strictly validated FUSMAP2D v1 viewer support.

This mirrors src/map_bundle.cpp and src/distance_field.cpp on purpose: a
web client must not give a corrupted map the appearance of validity.
FNV checks detect accidental damage, not tampering by an attacker.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import re
import stat
import struct
import zlib

MAGIC = b"FUSMAP2D"
MAX_BYTES = 150 * 1024 * 1024
MAX_CELLS = 16_000_000
MAX_LABEL = 256
FNV_START = 14695981039346656037
FNV_PRIME = 1099511628211
MASK = (1 << 64) - 1
LABEL = re.compile(rb"[A-Za-z0-9_./:\-]+\Z")


class BundleError(ValueError):
    """Malformed or unsupported FUSMAP2D file."""


def fnv_update(h: int, data: bytes | memoryview) -> int:
    for b in data:
        h = ((h ^ b) * FNV_PRIME) & MASK
    return h


def fnv_word(h: int, value: int) -> int:
    for shift in range(0, 64, 8):
        h = ((h ^ ((value >> shift) & 255)) * FNV_PRIME) & MASK
    return h


class Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def take(self, n: int) -> bytes:
        if n < 0 or self.pos + n > len(self.data):
            raise BundleError("truncated map bundle")
        chunk = self.data[self.pos : self.pos + n]
        self.pos += n
        return chunk

    def u(self, n: int) -> int:
        return int.from_bytes(self.take(n), "little")

    def f64(self) -> float:
        return struct.unpack("<d", self.take(8))[0]

    def label(self) -> str:
        length = self.u(4)
        if not (1 <= length <= MAX_LABEL):
            raise BundleError("invalid provenance label length")
        chunk = self.take(length)
        if not LABEL.fullmatch(chunk):
            raise BundleError("invalid provenance label characters")
        return chunk.decode("ascii")


class MapBundle:
    def __init__(self, raw: bytes):
        if len(raw) > MAX_BYTES:
            raise BundleError("map bundle exceeds 150 MiB limit")
        reader = Reader(raw)
        if reader.take(8) != MAGIC or reader.u(4) != 1:
            raise BundleError("unknown map magic/version")
        flags = reader.u(4)
        if flags not in (0, 1):
            raise BundleError("unsupported map flags")
        width, height = reader.u(4), reader.u(4)
        if not width or not height or width > MAX_CELLS or height > MAX_CELLS // width:
            raise BundleError("invalid map dimensions")
        resolution = reader.f64()
        origin_x, origin_y, yaw = reader.f64(), reader.f64(), reader.f64()
        hit_lo, free_lo, min_lo, max_lo = (reader.f64() for _ in range(4))
        clears = reader.u(4)
        if clears not in (0, 1):
            raise BundleError("unsupported no-return flag")
        revision, stored_fp = reader.u(8), reader.u(8)
        frame_id, source_id, truth_class = reader.label(), reader.label(), reader.label()
        count = reader.u(8)
        if count != width * height or len(raw) - reader.pos != count * 9 + 8:
            raise BundleError("cell payload length or trailing bytes invalid")
        config_nums = (resolution, origin_x, origin_y, yaw, hit_lo, free_lo, min_lo, max_lo)
        if (not all(math.isfinite(n) for n in config_nums) or resolution <= 0 or
            not math.isfinite(width * resolution) or not math.isfinite(height * resolution) or
            hit_lo <= 0 or free_lo >= 0 or min_lo >= 0 or max_lo <= 0 or min_lo >= max_lo):
            raise BundleError("invalid grid configuration")
        calc_footer = fnv_update(FNV_START, raw[:-8])
        if calc_footer != int.from_bytes(raw[-8:], "little"):
            raise BundleError("FUSMAP2D checksum mismatch")
        # Replicate MapSnapshot::fingerprint() from distance_field.cpp:
        # eight-byte LE for every config field, revision, and each cell.
        h = FNV_START
        for word in (width, height):
            h = fnv_word(h, word)
        for number in (resolution, origin_x, origin_y, yaw):
            h = fnv_word(h, struct.unpack("<Q", struct.pack("<d", number))[0])
        h = fnv_word(h, revision)
        cells_start = reader.pos
        cells = memoryview(raw)[cells_start : cells_start + count * 9]
        known = occupied = free = 0
        for offset in range(0, len(cells), 9):
            observed = cells[offset]
            if observed not in (0, 1):
                raise BundleError("invalid observed mask")
            lo_word = struct.unpack_from("<Q", cells, offset + 1)[0]
            lo = struct.unpack_from("<d", cells, offset + 1)[0]
            if not math.isfinite(lo) or not min_lo <= lo <= max_lo or (not observed and lo != 0.0):
                raise BundleError("invalid known/unknown occupancy value")
            h = fnv_word(fnv_word(h, lo_word), observed)
            if observed:
                known += 1
                if lo > 0:
                    occupied += 1
                elif lo < 0:
                    free += 1
        if h != stored_fp:
            raise BundleError("FUSMAP2D snapshot fingerprint mismatch")
        self.width, self.height, self.resolution = width, height, resolution
        self.origin = (origin_x, origin_y, yaw)
        self.revision, self.fingerprint = revision, stored_fp
        self.frame_id, self.source_id, self.truth_class = frame_id, source_id, truth_class
        self.metric_origin_verified = bool(flags)
        self.clear_no_returns = bool(clears)
        self.known, self.unknown = known, count - known
        self.occupied, self.free = occupied, free
        self.cells = cells
        self.min_log_odds, self.max_log_odds = min_lo, max_lo

    def cell(self, x: int, y: int) -> dict:
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            raise IndexError("cell outside map")
        offset = (y * self.width + x) * 9
        known = bool(self.cells[offset])
        log_odds = struct.unpack_from("<d", self.cells, offset + 1)[0]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        lx, ly = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        wx = self.origin[0] + c * lx - s * ly
        wy = self.origin[1] + s * lx + c * ly
        return {
            "x": x, "y": y, "world_x": wx, "world_y": wy,
            "observed": known, "log_odds": log_odds if known else None,
            "occupied_probability": 1 / (1 + math.exp(-log_odds)) if known else None,
        }

    def metadata(self) -> dict:
        return {
            "format": "FUSMAP2D v1", "width": self.width, "height": self.height,
            "resolution_m": self.resolution,
            "origin": {"x": self.origin[0], "y": self.origin[1], "yaw_rad": self.origin[2]},
            "frame_id": self.frame_id, "source_id": self.source_id,
            "truth_class": self.truth_class, "metric_origin_verified": self.metric_origin_verified,
            "revision": str(self.revision), "fingerprint": str(self.fingerprint),
            "known": self.known, "unknown": self.unknown,
            "occupied": self.occupied, "free": self.free,
            "neutral_known": self.known - self.occupied - self.free,
            "clear_trusted_no_returns": self.clear_no_returns,
            "integrity": "verified-corruption-only",
        }

    def png(self, max_side: int = 1800) -> bytes:
        # Highest-Y grid row appears at the top, matching C++ write_map_pgm.
        # Nearest-neighbor reduction is deliberate: no phantom interpolated evidence.
        step = max(1, math.ceil(max(self.width, self.height) / max_side))
        out_w = math.ceil(self.width / step)
        out_h = math.ceil(self.height / step)
        rgba = bytearray()
        for py in range(out_h):
            rgba.append(0)  # PNG filter type 0
            y = self.height - 1 - min(py * step, self.height - 1)
            for px in range(out_w):
                x = min(px * step, self.width - 1)
                offset = (y * self.width + x) * 9
                if not self.cells[offset]:
                    color = (21, 34, 54, 255)  # unknown is explicit
                else:
                    lo = struct.unpack_from("<d", self.cells, offset + 1)[0]
                    prob = 1.0 / (1.0 + math.exp(-lo))
                    if prob >= 0.65:
                        color = (70, 211, 184, 255)
                    elif prob <= 0.35:
                        color = (225, 237, 241, 255)
                    else:
                        color = (131, 155, 175, 255)
                rgba.extend(color)
        def chunk(tag: bytes, data: bytes) -> bytes:
            body = tag + data
            return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))
        return (b"\x89PNG\r\n\x1a\n" +
                chunk(b"IHDR", struct.pack(">2I5B", out_w, out_h, 8, 6, 0, 0, 0)) +
                chunk(b"IDAT", zlib.compress(bytes(rgba), 6)) + chunk(b"IEND", b""))


def read_bundle(path: Path) -> MapBundle:
    # O_NOFOLLOW prevents symlink leaf traversal. Directory confinement is
    # separately enforced by the web server using fixed file basenames.
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "rb") as f:
        st = os.fstat(f.fileno())
        if not stat.S_ISREG(st.st_mode) or st.st_size > MAX_BYTES:
            raise BundleError("map must be a regular file <= 150 MiB")
        data = f.read(MAX_BYTES + 1)
    return MapBundle(data)


def parse_registry(path: Path) -> dict:
    """Read FUSGEN1 pointer without changing state or selecting an orphan."""
    if not path.exists() and not path.is_symlink():
        return {"status": "absent", "active": None, "previous": None}
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "rb") as f:
        if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
            raise BundleError("registry must be a regular file")
        raw = f.read(257)
    if len(raw) > 256:
        raise BundleError("oversized registry")
    parts = raw.split(b"\n")
    if len(parts) != 5 or parts[-1] or parts[0] != b"FUSGEN1":
        raise BundleError("malformed FUSGEN1 header")
    prefix = b"\n".join(parts[:3]) + b"\n"
    def num(raw_number: bytes) -> int:
        if not raw_number or not re.fullmatch(rb"(?:0|[1-9][0-9]*)", raw_number):
            raise BundleError("noncanonical registry number")
        result = int(raw_number)
        if result > MASK:
            raise BundleError("registry number overflow")
        return result
    if fnv_update(FNV_START, prefix) != num(parts[3]):
        raise BundleError("registry checksum mismatch")
    def ref(row: bytes) -> dict | None:
        fields = row.split(b" ")
        if len(fields) != 2:
            raise BundleError("invalid registry pointer")
        gen, fp = (num(x) for x in fields)
        if gen == 0:
            if fp != 0:
                raise BundleError("zero id with nonzero fingerprint")
            return None
        return {"id": str(gen), "fingerprint": str(fp), "file": f"generation-{gen}.slammap"}
    active, previous = ref(parts[1]), ref(parts[2])
    if (previous and not active) or (active and previous and active["id"] == previous["id"]):
        raise BundleError("invalid active/previous pointers")
    return {"status": "present", "active": active, "previous": previous}
