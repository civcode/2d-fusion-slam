#!/usr/bin/env python3
"""Dependency-free SLAM map PGM/YAML conversion test."""
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "apps" / "demo"))
from slammap_to_pgm import export_pgm_yaml, render_pgm
from web.bundle import read_bundle
from web.example import create_example_bundle


def main():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        source = root / "sample.slammap"
        source.write_bytes(create_example_bundle())
        bundle = read_bundle(source)
        pgm, yaml = export_pgm_yaml(source, root / "sample.graph_optimized")
        assert pgm.read_bytes() == render_pgm(bundle)
        assert pgm.read_bytes().startswith(
            f"P5\n{bundle.width} {bundle.height}\n255\n".encode())
        assert "image: sample.graph_optimized.pgm" in yaml.read_text()
        assert "occupied_thresh: 0.65" in yaml.read_text()
        try:
            export_pgm_yaml(source, root / "sample.graph_optimized")
            raise AssertionError("overwrote an existing preview")
        except FileExistsError:
            pass
        broken = root / "corrupt.slammap"
        broken.write_bytes(source.read_bytes()[:-1] + b"X")
        try:
            export_pgm_yaml(broken, root / "bad")
            raise AssertionError("corrupt map was accepted")
        except ValueError:
            pass
        assert not (root / "bad.pgm").exists()
        assert not (root / "bad.yaml").exists()
    print("slammap_to_pgm_tools passed")


if __name__ == "__main__":
    main()
