#!/usr/bin/env python3
"""Versioned Apartment/Workshop native-route validation, independent of SLAM.

The native generator rejects wall intersections at 20 ms spacing. This test
independently verifies the emitted 300 ms truth path, door coverage, motion
continuity, workshop raster clearances, and scoring provenance. Truth is
used only by this post-hoc test and never by the estimator.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "demo"))
from web.example import _cell, WIDTH, HEIGHT, RESOLUTION
from fusion_scene_diversity_test import check_scene


def crosses(path, axis, coordinate, minimum, maximum):
    for a, b in zip(path, path[1:]):
        av, bv = a[axis], b[axis]
        if (av-coordinate)*(bv-coordinate) >= 0 or abs(bv-av)<1e-12:
            continue
        t=(coordinate-av)/(bv-av)
        other=a[1-axis]+t*(b[1-axis]-a[1-axis])
        if minimum+.12 < other < maximum-.12:
            return True
    return False


def clearance_to_cell(x, y, ix, iy):
    left, right = ix*RESOLUTION, (ix+1)*RESOLUTION
    bottom, top = iy*RESOLUTION, (iy+1)*RESOLUTION
    return math.hypot(max(left-x, 0, x-right),
                      max(bottom-y, 0, y-top))


def validate_workshop(path):
    # The authoritative schematic fixture is the independent raster oracle.
    blocked = {(x,y) for y in range(HEIGHT) for x in range(WIDTH)
               if (lambda cell: bool(cell[0] and cell[1]>0))(_cell(x,y))}
    for x,y,yaw in path:
        laser=(x+.12*math.cos(yaw)-.01*math.sin(yaw),
               y+.12*math.sin(yaw)+.01*math.cos(yaw))
        for px,py,required in ((x,y,.19),(*laser,.075)):
            cx,cy=int(math.floor(px/RESOLUTION)),int(math.floor(py/RESOLUTION))
            assert 0<=cx<WIDTH and 0<=cy<HEIGHT
            for iy in range(max(0,cy-4),min(HEIGHT,cy+5)):
                for ix in range(max(0,cx-4),min(WIDTH,cx+5)):
                    if (ix,iy) in blocked:
                        assert clearance_to_cell(px,py,ix,iy)>required, (
                            "workshop route too near occupied fixture cell", px,py)
    assert any(x<3 for x,y,_ in path)
    assert any(x>8 for x,y,_ in path)
    assert any(y>7 for x,y,_ in path)


def validate_route(name, truth):
    expected={"apartment_explore":360,"workshop":280}[name]
    assert len(truth)==expected
    for a,b in zip(truth,truth[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        rate=(b[2]-a[2]+math.pi)%(2*math.pi)-math.pi
        assert math.hypot(dx,dy)/.3 < .86, "discontinuous translation"
        assert abs(rate/.3)<1.3, "unbounded turn rate"
    assert math.dist(truth[0][:2],truth[{"apartment_explore":280,
                                        "workshop":210}[name]][:2])<.01, (
                                            "closed path must revisit origin")
    if name=="apartment_explore":
        assert crosses(truth,0,5.0,4.0,5.5)
        assert crosses(truth,0,10.0,7.0,8.5)
        assert crosses(truth,1,7.0,2.5,4.0)
        assert any(x<4.7 and y>8 for x,y,_ in truth)
        assert any(6<x<9 for x,y,_ in truth)
        assert any(x>11 for x,y,_ in truth)
    else:
        validate_workshop(truth)


def main():
    if len(sys.argv)!=3:
        raise SystemExit("usage: fusion_diverse_routes_test.py EXE MAP_SCORER")
    with tempfile.TemporaryDirectory() as temp:
        for name in ("apartment_explore","workshop"):
            result=check_scene(sys.argv[1],sys.argv[2],Path(temp),name,0)
            path=Path(temp)/f"{name}-p0"/"trajectories.json"
            import json
            data=json.loads(path.read_text())
            assert result["frames"]==len(data["synthetic_truth_for_scoring"])
            validate_route(name,data["synthetic_truth_for_scoring"])
    print("versioned native routes, clearance, doors, and scoring passed")


if __name__=="__main__":
    main()
