#!/usr/bin/env python3
"""FR079 multi-view LiDAR visibility consistency, DIAGNOSTIC ONLY.

Two disjoint scan windows are ray-traced in the *unchanged* optimized pose
frame. No registration is fitted, no graph factor is added, no Ceres/GTSAM
estimate or authoritative map is altered. Unknown ≠ free; opposing wall faces
need not have coincident occupied endpoints.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import sys
import zlib

from fr079_reference_eval_lib import laser_records, pose, wrap


def window(text: str, count: int) -> tuple[int, int]:
    """Half-open scan index range START:END."""
    try:
        start_s, end_s = text.split(':')
        start, end = int(start_s), int(end_s)
    except (ValueError, AttributeError) as exc:
        raise ValueError('window must be START:END (END exclusive)') from exc
    if start < 0 or end > count or end <= start:
        raise ValueError('window indices must be ordered and within the log')
    return start, end


def checked_poses(raw, sha: str, graph: dict):
    if (graph.get('format') != 'FUSFR079LOOP1' or
            graph.get('source_sha256') != sha or
            graph.get('independent_ground_truth') is not False):
        raise ValueError('graph provenance/format does not match the raw scan log')
    values = graph.get('loop_graph_poses')
    if not isinstance(values, list) or len(values) != len(raw):
        raise ValueError('scan/optimized-pose count mismatch')
    return [pose(v, f'optimized scan {i}') for i, v in enumerate(values)]


def region_arg(text: str | None):
    if text is None:
        return None
    try:
        values = tuple(float(v) for v in text.split(','))
    except ValueError as exc:
        raise ValueError('region must be XMIN,YMIN,XMAX,YMAX in metres') from exc
    if (len(values) != 4 or not all(math.isfinite(v) for v in values) or
            values[0] >= values[2] or values[1] >= values[3]):
        raise ValueError('invalid world-coordinate region bounds')
    return values


def cell(x, y, resolution):
    return math.floor(x / resolution), math.floor(y / resolution)


def inside(x, y, bounds):
    return bounds is None or (bounds[0] <= x <= bounds[2] and
                              bounds[1] <= y <= bounds[3])


def raster_window(scans, poses, bounds, interval, *, resolution=.10,
                  scan_stride=3, beam_stride=4, max_range=18.,
                  endpoint_guard_m=.20):
    """Count observed hit/free cells by scan, not by number of beams.

    A beam gives free evidence only before its endpoint guard. No-return or
    invalid beams are NEVER used as proof of free space. These are raw LiDAR
    rays in recorded scan coordinates, not simulated rays cast at the final map.
    """
    hits, frees = Counter(), Counter()
    used_scans, used_beams = 0, 0
    for index in range(interval[0], interval[1], scan_stride):
        values = scans[index]['ranges']
        x0, y0, yaw = poses[index]
        visited_free, visited_hit = set(), set()
        for beam in range(0, len(values), beam_stride):
            distance = values[beam]
            if not (.10 <= distance <= max_range):
                continue
            # Exactly the beam convention of fr079_reference_eval_lib.py.
            bearing = yaw - math.pi / 2 + math.pi * beam / len(values)
            vx, vy = math.cos(bearing), math.sin(bearing)
            ex, ey = x0 + distance * vx, y0 + distance * vy
            if inside(ex, ey, bounds):
                visited_hit.add(cell(ex, ey, resolution))
            length = distance - endpoint_guard_m
            if length > 0:
                steps = math.ceil(length / (resolution * .5))
                for step in range(steps):
                    travelled = length * step / steps
                    px, py = x0 + travelled * vx, y0 + travelled * vy
                    if inside(px, py, bounds):
                        visited_free.add(cell(px, py, resolution))
            used_beams += 1
        # Contradictory beams within the same scan do not vote twice.
        visited_free.difference_update(visited_hit)
        hits.update(visited_hit)
        frees.update(visited_free)
        used_scans += 1
    return {'hit': hits, 'free': frees, 'scans': used_scans, 'beams': used_beams}


def classify(raster: dict, minimum_observations: int):
    """Observed hit/free both => ambiguous, never forced to either label."""
    strong_hit = {p for p, v in raster['hit'].items()
                  if v >= minimum_observations}
    strong_free = {p for p, v in raster['free'].items()
                   if v >= minimum_observations}
    ambiguous = strong_hit & strong_free
    return strong_hit - ambiguous, strong_free - ambiguous, ambiguous


def compare(a, b, *, minimum_observations=2):
    ah, af, am = classify(a, minimum_observations)
    bh, bf, bm = classify(b, minimum_observations)
    both_occupied = ah & bh
    conflict_a_to_b = ah & bf
    conflict_b_to_a = bh & af
    # For directional rates the denominator is observed occupied cells with
    # known status in the other interval; unobserved is not a false negative.
    observed_a = ah & (bh | bf)
    observed_b = bh & (ah | af)
    joint_known = (ah | af) & (bh | bf)
    return {
        'a_occupied_cells': len(ah), 'b_occupied_cells': len(bh),
        'a_free_cells': len(af), 'b_free_cells': len(bf),
        'a_ambiguous_cells': len(am), 'b_ambiguous_cells': len(bm),
        'joint_known_cells': len(joint_known),
        'joint_occupied_cells': len(both_occupied),
        'a_occupied_b_free_cells': len(conflict_a_to_b),
        'b_occupied_a_free_cells': len(conflict_b_to_a),
        'a_occupied_observed_by_b': len(observed_a),
        'b_occupied_observed_by_a': len(observed_b),
        'a_to_b_contradiction_fraction': (
            len(conflict_a_to_b) / len(observed_a) if observed_a else None),
        'b_to_a_contradiction_fraction': (
            len(conflict_b_to_a) / len(observed_b) if observed_b else None),
        'symmetric_joint_occupied_fraction': (
            len(both_occupied) / len(joint_known) if joint_known else None),
    }, (ah, af, am, bh, bf, bm)


def rgba_png(classes, filename: Path, resolution: float, bounds=None, max_side=1600):
    ah, af, am, bh, bf, bm = classes
    points = ah | af | am | bh | bf | bm
    if not points:
        return None
    if bounds is not None:
        xmin, ymin, xmax, ymax = bounds
        lo_x, lo_y = cell(xmin, ymin, resolution)
        hi_x, hi_y = cell(xmax, ymax, resolution)
    else:
        xs, ys = zip(*points)
        lo_x, hi_x, lo_y, hi_y = min(xs)-2, max(xs)+2, min(ys)-2, max(ys)+2
    width, height = hi_x-lo_x+1, hi_y-lo_y+1
    if width > 16000 or height > 16000:
        raise ValueError('plot region exceeds 16k cells; supply --region')
    stride = max(1, math.ceil(max(width, height) / max_side))
    out_w, out_h = math.ceil(width / stride), math.ceil(height / stride)
    # Only observed cells are assigned colors. Unknown is neutral grey.
    background = (225, 226, 228)
    free = (249, 249, 248)
    a_color, b_color, both_color = (21, 122, 153), (204, 125, 48), (72, 69, 119)
    conflict = (224, 49, 50)
    ambiguous = (130, 130, 130)
    rows = bytearray()
    for py in range(out_h):
        rows.append(0)
        y = hi_y - py * stride
        for px in range(out_w):
            p = (lo_x + px * stride, y)
            color = background
            if p in af or p in bf:
                color = free
            if p in ah:
                color = a_color
            if p in bh:
                color = b_color
            if p in ah and p in bh:
                color = both_color
            if p in am or p in bm:
                color = ambiguous
            if p in ah and p in bf or p in bh and p in af:
                color = conflict
            rows.extend(color)
    def chunk(kind, payload):
        return (struct.pack('>I', len(payload)) + kind + payload +
                struct.pack('>I', zlib.crc32(kind+payload) & 0xffffffff))
    png = (b'\x89PNG\r\n\x1a\n' +
           chunk(b'IHDR', struct.pack('>IIBBBBB', out_w, out_h, 8, 2, 0, 0, 0)) +
           chunk(b'IDAT', zlib.compress(bytes(rows), 6)) + chunk(b'IEND', b''))
    filename.write_bytes(png)
    return {'width': out_w, 'height': out_h,
            'world_origin_cell': [lo_x, lo_y],
            'display_cell_stride': stride,
            'legend': {'unknown': '#e1e2e4', 'observed_free': '#f9f9f8',
                       'a_occupied': '#157a99', 'b_occupied': '#cc7d30',
                       'both_occupied': '#484577', 'contradiction': '#e03132',
                       'within_window_ambiguous': '#828282'}}


def suggest_pairs(poses, *, stride=16, radius=5., min_gap=80,
                  min_heading_degrees=60., max_results=16, half_window=24):
    """Exploratory scan-window hints; not place recognition or loop factors."""
    candidates = []
    for i in range(0, len(poses), stride):
        for j in range(0, i-min_gap+1, stride):
            dist = math.hypot(poses[i][0]-poses[j][0],
                              poses[i][1]-poses[j][1])
            heading = abs(math.degrees(wrap(poses[i][2]-poses[j][2])))
            if dist <= radius and heading >= min_heading_degrees:
                candidates.append((dist,j,i,heading))
    candidates.sort()
    chosen=[]
    for distance,j,i,heading in candidates:
        if any(abs(i-y['current_scan']) < half_window*2 or
               abs(j-y['historical_scan']) < half_window*2 for y in chosen):
            continue
        chosen.append({'historical_scan':j,'current_scan':i,
                       'distance_m':distance,'heading_difference_deg':heading,
                       'window_a':[max(0,j-half_window),min(len(poses),j+half_window)],
                       'window_b':[max(0,i-half_window),min(len(poses),i+half_window)]})
        if len(chosen)>=max_results:break
    return chosen


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-log', type=Path, required=True)
    parser.add_argument('--loops', type=Path, required=True,
                        help='unaltered gold-standard *.loops.json')
    parser.add_argument('--window-a', required=True, help='START:END scan indices')
    parser.add_argument('--window-b', required=True, help='START:END scan indices')
    parser.add_argument('--region', help='optional world-metric XMIN,YMIN,XMAX,YMAX')
    parser.add_argument('--out', type=Path, required=True,
                        help='new output prefix, refuses existing .json/.png')
    parser.add_argument('--scan-stride', type=int, default=3)
    parser.add_argument('--beam-stride', type=int, default=4)
    parser.add_argument('--resolution', type=float, default=.10)
    parser.add_argument('--max-range', type=float, default=18.)
    parser.add_argument('--min-observations', type=int, default=2)
    parser.add_argument('--suggest-pairs', type=int, default=16)
    args = parser.parse_args(argv)
    if (args.scan_stride < 1 or args.beam_stride < 1 or
        args.min_observations < 1 or not .03 <= args.resolution <= .5 or
        not 1 <= args.max_range <= 40 or not 0 <= args.suggest_pairs <= 100):
        parser.error('invalid resolution/stride/range/min-observations')
    report_file = Path(str(args.out)+'.json')
    png_file = Path(str(args.out)+'.png')
    control_png_file = Path(str(args.out)+'.control.png')
    if any(f.exists() or f.is_symlink() for f in
           (report_file, png_file, control_png_file)):
        parser.error('output exists: specify a new --out prefix')
    if not args.out.parent.is_dir():
        parser.error('output directory does not exist')
    scans, sha = laser_records(args.raw_log)
    graph = json.loads(args.loops.read_text(encoding='utf-8'))
    poses = checked_poses(scans, sha, graph)
    a = window(args.window_a, len(scans))
    b = window(args.window_b, len(scans))
    if a[0] < b[1] and b[0] < a[1]:
        parser.error('windows overlap: use disjoint groups of scans')
    bounds = region_arg(args.region)
    settings = {'resolution':args.resolution,'scan_stride':args.scan_stride,
                'beam_stride':args.beam_stride,'max_range':args.max_range,
                'minimum_observations_per_cell':args.min_observations}
    left = raster_window(scans,poses,bounds,a,resolution=args.resolution,
                         scan_stride=args.scan_stride,beam_stride=args.beam_stride,
                         max_range=args.max_range)
    right = raster_window(scans,poses,bounds,b,resolution=args.resolution,
                          scan_stride=args.scan_stride,beam_stride=args.beam_stride,
                          max_range=args.max_range)
    metrics, classes=compare(left,right,minimum_observations=args.min_observations)
    # The graph-only changes preserve the Ceres frontend pose history in
    # graph_control_poses. Run the SAME evidence protocol on those poses to
    # determine whether a local contradiction predates loop optimization.
    control_metric = control_image = control_classes = None
    control_values = graph.get('graph_control_poses')
    if control_values is not None:
        if not isinstance(control_values,list) or len(control_values)!=len(scans):
            raise ValueError('graph control poses must match raw scan count')
        control_poses=[pose(v,f'control scan {i}') for i,v in enumerate(control_values)]
        ctrl_a = raster_window(scans,control_poses,bounds,a,
                              resolution=args.resolution,
                              scan_stride=args.scan_stride,
                              beam_stride=args.beam_stride,max_range=args.max_range)
        ctrl_b = raster_window(scans,control_poses,bounds,b,
                              resolution=args.resolution,
                              scan_stride=args.scan_stride,
                              beam_stride=args.beam_stride,max_range=args.max_range)
        control_metric, control_classes = compare(
            ctrl_a,ctrl_b,minimum_observations=args.min_observations)
    # Validate all evidence/provenance before writing the two image outputs.
    image = rgba_png(classes,png_file,args.resolution,bounds)
    if control_classes is not None:
        control_image = rgba_png(control_classes,control_png_file,args.resolution,bounds)
    output = {
        'format':'FR079_MULTIVIEW_VISIBILITY_DIAGNOSTIC1',
        'kind':'observation_consistency_diagnostic_not_loop_verification',
        'independent_ground_truth':False,
        'read_only':True,'graph_factors_added':0,
        'source_sha256':sha,'graph_registration_mode':graph.get('registration_mode'),
        'graph_optimized_poses_used':True,'window_a':list(a),'window_b':list(b),
        'region_world_m':list(bounds) if bounds else None,
        'settings':settings,'a_scan_count':left['scans'],'b_scan_count':right['scans'],
        'a_hit_beams':left['beams'],'b_hit_beams':right['beams'],
        'metrics':metrics,'plot':image,
        'frontend_control_metrics':control_metric,'frontend_control_plot':control_image,
        'comparison_protocol':'same raw ray evidence in two frozen pose frames; no refitting',
        'suggested_pose_proximity_pairs':suggest_pairs(poses,max_results=args.suggest_pairs),
        'interpretation':('Only strong occupied-vs-free evidence is called a contradiction; '
                          'unknown or non-overlapping fields of view do not count as failures. '
                          'Separate wall faces can be geometrically valid. '
                          'Same-log pose errors, dynamic obstacles and self-calibration can '
                          'cause contradictions; scores are NOT ground-truth error or proof '
                          'of loop closure. No graph factor is created.'),
    }
    report_file.write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'json':str(report_file),'png':str(png_file) if image else None,
                      'frontend_control_png':str(control_png_file) if control_image else None,
                      'metrics':metrics,'frontend_control_metrics':control_metric},indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
