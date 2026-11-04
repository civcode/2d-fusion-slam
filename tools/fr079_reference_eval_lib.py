"""FR079 scan-matched reference benchmark math and CARMEN data validation.

The published scan-matched log is a correlated *estimated* reference, never
surveyed ground truth. No reference measurement is used by the SLAM pipeline.
"""
from __future__ import annotations

import gzip
import hashlib
import math
from collections import Counter
from pathlib import Path
from statistics import mean


def pose(value, label: str) -> tuple[float, float, float]:
    if (not isinstance(value, (list, tuple)) or len(value) != 3 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in value)):
        raise ValueError(f"invalid SE(2) pose: {label}")
    return tuple(float(v) for v in value)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca*b[0] - sa*b[1],
            a[1] + sa*b[0] + ca*b[1], wrap(a[2] + b[2]))


def inverse(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca*a[0] - sa*a[1], sa*a[0] - ca*a[1], wrap(-a[2]))


def relative(a, b):
    return compose(inverse(a), b)


def gauge_align(reference: list, estimated: list) -> list:
    transform = compose(reference[0], inverse(estimated[0]))
    return [compose(transform, p) for p in estimated]


def rigid_xy_align(reference: list, estimated: list) -> list:
    """SE(2) least-squares alignment: no scaling, reflection or time warping."""
    rx, ry = mean(p[0] for p in reference), mean(p[1] for p in reference)
    ex, ey = mean(p[0] for p in estimated), mean(p[1] for p in estimated)
    cross = dot = 0.
    for r, e in zip(reference, estimated):
        x, y = e[0]-ex, e[1]-ey
        u, v = r[0]-rx, r[1]-ry
        dot += x*u + y*v
        cross += x*v - y*u
    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    transform = (rx-c*ex+s*ey, ry-s*ex-c*ey, theta)
    return [compose(transform, p) for p in estimated]


def stats(values: list[float]) -> dict:
    if not values or not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite or empty evaluation metric")
    ordered = sorted(values)
    def quantile(q):
        idx = q*(len(ordered)-1)
        lo, hi = math.floor(idx), math.ceil(idx)
        return ordered[lo] + (ordered[hi]-ordered[lo])*(idx-lo)
    return {"count":len(values), "mean":math.fsum(values)/len(values),
            "rmse":math.sqrt(math.fsum(v*v for v in values)/len(values)),
            "median":quantile(.50), "p95":quantile(.95), "max":ordered[-1],
            "max_scan_or_edge_index":max(range(len(values)),
                                         key=values.__getitem__)}


def trajectory_metrics(reference: list, estimated: list, *, align: str) -> dict:
    if not reference or len(reference) != len(estimated):
        raise ValueError("trajectory/reference pose count mismatch")
    if align == "first_pose":
        aligned = gauge_align(reference, estimated)
    elif align == "rigid_xy":
        aligned = rigid_xy_align(reference, estimated)
    else:
        raise ValueError("unsupported evaluation alignment")
    positions = [math.hypot(r[0]-e[0], r[1]-e[1])
                 for r, e in zip(reference, aligned)]
    headings = [abs(math.degrees(wrap(r[2]-e[2])))
                for r, e in zip(reference, aligned)]
    rpe = {}
    for interval in (1,10,50):
        if len(reference) <= interval:
            continue
        translations, yaw = [], []
        for i in range(len(reference)-interval):
            r = relative(reference[i], reference[i+interval])
            e = relative(estimated[i], estimated[i+interval])
            err = relative(r, e)
            translations.append(math.hypot(err[0], err[1]))
            yaw.append(abs(math.degrees(err[2])))
        rpe[str(interval)] = {"translation_m":stats(translations),
                              "yaw_deg":stats(yaw)}
    return {"alignment":align, "ate_translation_m":stats(positions),
            "absolute_yaw_deg":stats(headings), "rpe_scan_intervals":rpe,
            "aligned_poses":aligned}


def laser_records(path: Path) -> tuple[list[dict], str]:
    """Parse FLASER/RLASER without trusting poses in the raw recording."""
    path = Path(path)
    with (gzip.open(path,"rb") if path.suffix == ".gz" else
          path.open("rb")) as source:
        body = source.read(128*1024*1024+1)
    if len(body) > 128*1024*1024:
        raise ValueError("CARMEN log exceeds 128 MiB")
    digest = hashlib.sha256(body).hexdigest()
    scans = []
    for lineno, line in enumerate(body.decode("utf-8").splitlines(), 1):
        fields = line.split()
        if not fields or fields[0] not in ("FLASER","RLASER"):
            continue
        try:
            count = int(fields[1])
            if count < 1 or count > 4096 or len(fields) != count+11:
                raise ValueError("bad beam count or record length")
            ranges = tuple(float(v) for v in fields[2:count+2])
            laser = pose(tuple(float(v) for v in fields[count+2:count+5]),
                         "logged laser")
            odom = pose(tuple(float(v) for v in fields[count+5:count+8]),
                        "logged odometry")
            time = float(fields[count+8])
            logger = float(fields[count+10])
            if not all(math.isfinite(v) for v in (*ranges,*laser,*odom,time,logger)):
                raise ValueError("nonfinite CARMEN field")
            if scans and time < scans[-1]["timestamp"]:
                raise ValueError("nonmonotonic CARMEN laser timestamp")
        except (ValueError,IndexError) as exc:
            raise ValueError(f"malformed laser line {lineno}: {exc}") from exc
        scans.append({"ranges":ranges,"laser":laser,"timestamp":time})
    if not scans or len(scans) > 6000:
        raise ValueError("missing/excessive CARMEN laser records")
    return scans, digest


def verify_reference_scans(raw: list, reference: list, timestamps: list,
                           tolerance_s: float = .02) -> None:
    """Refuse differing scans, hidden index shifts, or unrelated reference."""
    if len(raw) != len(reference) or len(timestamps) != len(raw):
        raise ValueError("raw/reference/frontend laser scan counts differ")
    for i, (a, b, stamp) in enumerate(zip(raw, reference, timestamps)):
        if (abs(a["timestamp"]-b["timestamp"]) > tolerance_s or
                abs(a["timestamp"]-stamp) > tolerance_s):
            raise ValueError(f"unpaired reference/frontend timestamp at scan {i}")
        if len(a["ranges"]) != len(b["ranges"]):
            raise ValueError(f"laser beam count differs at scan {i}")
        for j,(x,y) in enumerate(zip(a["ranges"],b["ranges"])):
            if abs(x-y) > 1e-4:
                raise ValueError(f"LiDAR ranges differ at scan {i}, beam {j}")


def occupied_endpoints(scans: list, poses: list, resolution: float,
                       stride: int, max_range: float) -> set[tuple[int,int]]:
    """Occupied hit-endpoint proxy, not a free/occupied/unknown SLAM grid."""
    counts = Counter()
    for scan, p in zip(scans,poses):
        c, s = math.cos(p[2]),math.sin(p[2])
        ranges = scan["ranges"]
        for beam in range(0,len(ranges),stride):
            distance = ranges[beam]
            if not .10 <= distance <= max_range:
                continue
            bearing = -math.pi/2 + math.pi*beam/len(ranges)
            x,y = distance*math.cos(bearing),distance*math.sin(bearing)
            gx,gy = p[0]+c*x-s*y,p[1]+s*x+c*y
            counts[(math.floor(gx/resolution),math.floor(gy/resolution))] += 1
    return {p for p, count in counts.items() if count >= 2}


def map_overlap(reference: set, candidate: set, tolerance_cells: int = 1) -> dict:
    if not reference or not candidate:
        raise ValueError("insufficient occupied endpoint evidence")
    offsets = [(dx,dy) for dx in range(-tolerance_cells,tolerance_cells+1)
                        for dy in range(-tolerance_cells,tolerance_cells+1)]
    def matched(src,dest):
        return sum(any((x+dx,y+dy) in dest for dx,dy in offsets)
                   for x,y in src)
    return {"reference_occupied_endpoint_cells":len(reference),
            "candidate_occupied_endpoint_cells":len(candidate),
            "strict_occupied_endpoint_iou":len(reference&candidate)/
                                           len(reference|candidate),
            "tolerant_precision":matched(candidate,reference)/len(candidate),
            "tolerant_recall":matched(reference,candidate)/len(reference),
            "tolerance_cells":tolerance_cells}


def improvement(before: float, after: float) -> dict:
    return {"before":before,"after":after,"lower_is_better":True,
            "absolute_reduction":before-after,
            "percent_reduction":100*(before-after)/before if before>0 else None}
