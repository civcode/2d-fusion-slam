#!/usr/bin/env python3
"""Independently verify CARMEN x-major/Y-up indexing against a source GIF.

Optional diagnostic only: requires Pillow; native C++ compilation has no Python
or Pillow dependency. The GIF is visualization evidence, not pose ground truth.
"""
import argparse
import gzip
import json
import struct
from pathlib import Path


def verify(map_path: Path, gif_path: Path) -> dict:
    from PIL import Image  # optional analysis dependency; not part of native SDK

    raw = gzip.decompress(map_path.read_bytes())
    prefix = b"CARMENMAPFILEv020 "
    section = b"GRIDMAP   "
    assert prefix in raw and section in raw, "unsupported map signature"
    start = raw.index(section) + len(section)
    width, height, resolution = struct.unpack_from("<IIf", raw, start)
    assert 0 < width <= 10000 and 0 < height <= 10000
    assert 0.001 <= resolution <= 1
    values = struct.unpack_from(f"<{width*height}f", raw, start + 12)
    with Image.open(gif_path) as source:
        picture = source.convert("L")
        assert picture.size == (width, height), "GIF/map shape mismatch"
        pixels = picture.load()
        occupied, free, unknown = 0, 0, 0
        occupied_matches, free_matches = 0, 0
        # File order x*height+y, visualization row = height - 1 - y.
        for x in range(width):
            for y in range(height):
                value = values[x * height + y]
                color = pixels[x, height - 1 - y]
                if value < 0:
                    unknown += 1
                elif value > 0.65:
                    occupied += 1
                    occupied_matches += color < 100
                elif value <= 0.001:
                    free += 1
                    free_matches += color >= 250
        result = {
            "width": width, "height": height, "resolution_metres": resolution,
            "occupied_checked": occupied, "occupied_pixel_agreement": occupied_matches / occupied,
            "free_checked": free, "free_pixel_agreement": free_matches / free,
            "unknown": unknown,
            "convention": "column-major x*height+y; visualization row = height-1-y",
            "independent_pose_truth": False,
        }
        assert result["occupied_pixel_agreement"] >= 0.999
        assert result["free_pixel_agreement"] >= 0.999
        return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path, help="compressed fr079-05cm.carmen.map")
    parser.add_argument("gif", type=Path, help="corresponding map05cm.gif (not path overlay)")
    args = parser.parse_args()
    print(json.dumps(verify(args.map, args.gif), sort_keys=True))


if __name__ == "__main__":
    main()
