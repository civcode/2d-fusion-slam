#!/usr/bin/env python3
"""Create visualization PGM/YAML from a verified FUSMAP2D map, without rerunning SLAM.

Use --out-prefix pointing to a new name; never overwrites historical previews.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "apps" / "demo"))
from web.bundle import read_bundle  # validates footer and snapshot fingerprint

SAFE_NAME = re.compile(r"[A-Za-z0-9_.-]+\Z")


def render_pgm(bundle) -> bytes:
    """Byte-identical to native write_map_pgm for a verified snapshot."""
    width, height = bundle.width, bundle.height
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    pixels = bytearray(width * height)
    pos = 0
    for y in range(height - 1, -1, -1):
        for x in range(width):
            offset = (y * width + x) * 9
            value = 205
            if bundle.cells[offset]:
                lo = struct.unpack_from("<d", bundle.cells, offset + 1)[0]
                probability = 1.0 / (1.0 + math.exp(-lo))
                value = min(255, max(0, math.floor(255.0 * (1.0 - probability) + .5)))
            pixels[pos] = value
            pos += 1
    return header + pixels


def render_yaml(bundle, pgm_name: str) -> str:
    if not SAFE_NAME.fullmatch(pgm_name):
        raise ValueError("unsafe PGM basename")
    ox, oy, yaw = bundle.origin
    return ("# Visualization only: authoritative known mask and log-odds are in .slammap\n"
            f"image: {pgm_name}\n"
            f"resolution: {bundle.resolution!r}\n"
            f"origin: [{ox!r}, {oy!r}, {yaw!r}]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n")


def export_pgm_yaml(slammap: Path, out_prefix: Path):
    slammap, out_prefix = Path(slammap), Path(out_prefix)
    pgm, yaml = Path(str(out_prefix) + ".pgm"), Path(str(out_prefix) + ".yaml")
    if any(p.exists() or p.is_symlink() for p in (pgm, yaml)):
        raise FileExistsError("preview exists; choose a new output prefix")
    if not out_prefix.parent.is_dir():
        raise FileNotFoundError("output directory must already exist")
    if not SAFE_NAME.fullmatch(pgm.name):
        raise ValueError("unsafe PGM basename")
    bundle = read_bundle(slammap)
    content = render_pgm(bundle)
    metadata = render_yaml(bundle, pgm.name)
    with pgm.open("xb") as stream:
        stream.write(content)
    try:
        with yaml.open("x", encoding="utf-8") as stream:
            stream.write(metadata)
    except BaseException:
        pgm.unlink()
        raise
    return pgm, yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--slammap", type=Path, required=True)
    p.add_argument("--out-prefix", type=Path, required=True)
    args = p.parse_args()
    pgm, yaml = export_pgm_yaml(args.slammap, args.out_prefix)
    print(f"Map preview: {pgm}\nMetadata: {yaml}")


if __name__ == "__main__":
    main()
