#!/usr/bin/env python3
"""Compare C++ Workshop fixture surfaces to independent Python raster contours."""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"apps"/"demo"))
from web.example import WIDTH, HEIGHT, RESOLUTION, _cell


def expected_edges():
    states=[[_cell(x,y) for x in range(WIDTH)] for y in range(HEIGHT)]
    occupied=[[bool(known and odds>0) for known,odds in row]
              for row in states]
    free=[[bool(known and odds<0) for known,odds in row]
          for row in states]
    edges=[]
    for y in range(1,HEIGHT):
        start=None
        for x in range(WIDTH+1):
            edge=x<WIDTH and (
                (occupied[y-1][x] and free[y][x]) or
                (occupied[y][x] and free[y-1][x]))
            if edge and start is None:
                start=x
            if not edge and start is not None:
                edges.append((start*RESOLUTION,y*RESOLUTION,
                              x*RESOLUTION,y*RESOLUTION))
                start=None
    for x in range(1,WIDTH):
        start=None
        for y in range(HEIGHT+1):
            edge=y<HEIGHT and (
                (occupied[y][x-1] and free[y][x]) or
                (occupied[y][x] and free[y][x-1]))
            if edge and start is None:
                start=y
            if not edge and start is not None:
                edges.append((x*RESOLUTION,start*RESOLUTION,
                              x*RESOLUTION,y*RESOLUTION))
                start=None
    return sorted(edges)


def main():
    if len(sys.argv)!=2:
        raise SystemExit("usage: workshop_contour_parity_test.py NATIVE_DUMP")
    output=subprocess.check_output([sys.argv[1]], text=True, timeout=15)
    actual=sorted(tuple(float(v) for v in line.split())
                  for line in output.strip().splitlines())
    expected=expected_edges()
    assert len(actual)==len(expected)>0, (
        "native workshop diverged from Python fixture contour count",
        len(actual),len(expected))
    for a,b in zip(actual,expected):
        assert len(a)==len(b)==4
        assert all(math.isclose(x,y,abs_tol=1e-9) for x,y in zip(a,b)), (
            "native workshop diverged from Python raster surface",a,b)
    print(f"{len(actual)} Workshop contours match canonical UI raster")


if __name__=="__main__":
    main()
