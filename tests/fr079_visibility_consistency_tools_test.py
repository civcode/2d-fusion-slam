#!/usr/bin/env python3
"""Deterministic geometry/provenance test: no reference trajectory or ground truth."""
from pathlib import Path
import hashlib
import json
import math
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from fr079_visibility_consistency import (
    cell, window, region_arg, checked_poses, raster_window, compare,
    rgba_png, suggest_pairs, main,
)


def assert_raises(typ, run):
    try:
        run()
    except typ:
        return
    raise AssertionError(f'{typ.__name__} not raised')


def scan(distance=2., beams=180):
    # Exactly one forward hit; all other beam readings invalid/no-return.
    ranges = [0.] * beams
    ranges[beams // 2] = distance
    return {'ranges': tuple(ranges)}


def build(n=12, a=(0., 0., 0.), b=(4., 0., math.pi), d=2.):
    scans = [scan(d) for _ in range(n)]
    poses = [a] * (n // 2) + [b] * (n - n // 2)
    settings = {'resolution':.10, 'scan_stride':1, 'beam_stride':1,
                'max_range':18.}
    aa = raster_window(scans, poses, None, (0,n//2),**settings)
    bb = raster_window(scans, poses, None, (n//2,n),**settings)
    return compare(aa, bb), (aa, bb)


def main_test():
    assert window('0:4', 8) == (0,4)
    for bad in ('0:0','-1:4','4:9','bad','4,5'):
        assert_raises(ValueError, lambda: window(bad,8))
    assert region_arg('0,1,3,4') == (0.,1.,3.,4.)
    assert_raises(ValueError,lambda:region_arg('2,0,1,4'))
    assert_raises(ValueError,lambda:region_arg('0,nan,1,4'))
    assert cell(-.01,-.01,.10) == (-1,-1)
    # Same surface observed from opposite sides should be classified as
    # same occupied cell, not contradiction (no free ray passes through it).
    (metric, classes), (aa, bb) = build()
    assert metric['joint_occupied_cells'] > 0, metric
    assert metric['a_occupied_b_free_cells'] == 0, metric
    assert metric['b_occupied_a_free_cells'] == 0, metric
    # Different physical wall faces can be legitimate: no occupied match
    # needed if free-space evidence is consistent.
    (metric, _), _ = build(b=(4.4,0.,math.pi))
    assert metric['a_occupied_b_free_cells'] == 0, metric
    assert metric['b_occupied_a_free_cells'] == 0, metric
    # Geometrically impossible crossing: B's ray travels through A's wall.
    (metric, classes), (aa, bb) = build(b=(3.6,0.,math.pi))
    assert metric['a_occupied_b_free_cells'] > 0, metric
    assert metric['a_to_b_contradiction_fraction'] > 0, metric
    # No cross-field observation must remain null, not invented zero score.
    (metric, _), _ = build(b=(11.,0.,math.pi))
    assert metric['a_occupied_observed_by_b'] == 0
    assert metric['a_to_b_contradiction_fraction'] is None
    with tempfile.TemporaryDirectory() as td:
        filename = Path(td) / 'contrast.png'
        detail = rgba_png(classes, filename, .10)
        assert detail['width'] > 0
        assert filename.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
        assert png_is_valid(filename)
    raw = [{'ranges':(2.,)}]*3
    good={'format':'FUSFR079LOOP1','source_sha256':'hash',
          'independent_ground_truth':False,
          'loop_graph_poses':[[0.,0.,0.]]*3}
    assert len(checked_poses(raw,'hash',good)) == 3
    assert_raises(ValueError,lambda:checked_poses(raw,'other',good))
    assert_raises(ValueError,lambda:checked_poses(raw[:2],'hash',good))
    poses=[(float(i),0.,0. if i<100 else math.pi) for i in range(200)]
    # The candidate generator is optional, deterministic, and does not
    # infer a loop when all far-apart poses are spatially distant.
    assert suggest_pairs(poses,stride=16,radius=.5,min_gap=80) == []
    # Full CLI exercise with a synthetic CARMEN log and a matching hashed graph.
    # No reference/GT poses enter the tool; the test declares its own poses.
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        n, beam_count = 12, 180
        ranges = ['0'] * beam_count
        ranges[beam_count // 2] = '2.0'
        lines = []
        for i in range(n):
            line = ['FLASER', str(beam_count), *ranges,
                    '0', '0', '0', '0', '0', '0', str(float(i)), 'host', '0']
            assert len(line) == beam_count + 11
            lines.append(' '.join(line))
        data = ('\n'.join(lines) + '\n').encode()
        rawfile = root / 'synthetic.log'
        rawfile.write_bytes(data)
        graphfile = root / 'synthetic.loops.json'
        graph = {
            'format': 'FUSFR079LOOP1',
            'source_sha256': hashlib.sha256(data).hexdigest(),
            'independent_ground_truth': False,
            'registration_mode': 'global_scan_to_submap_v2',
            'loop_graph_poses': [[0, 0, 0]] * 6 + [[3.6, 0, math.pi]] * 6,
        }
        graphfile.write_text(json.dumps(graph))
        out = root / 'visibility'
        assert main(['--raw-log', str(rawfile), '--loops', str(graphfile),
                     '--window-a', '0:6', '--window-b', '6:12',
                     '--scan-stride', '1', '--beam-stride', '1',
                     '--out', str(out)]) == 0
        report = json.loads((root / 'visibility.json').read_text())
        assert report['graph_factors_added'] == 0
        assert report['source_sha256'] == hashlib.sha256(data).hexdigest()
        assert report['metrics']['a_occupied_b_free_cells'] > 0
        assert png_is_valid(root / 'visibility.png')
        assert_raises(SystemExit, lambda: main([
            '--raw-log', str(rawfile), '--loops', str(graphfile),
            '--window-a', '0:6', '--window-b', '5:12',
            '--out', str(root / 'other')]))
    print('FR079 multi-view visibility geometry/provenance regression passed')


def png_is_valid(filename):
    import struct, zlib
    raw = filename.read_bytes()
    assert raw[:8] == b'\x89PNG\r\n\x1a\n'
    size = struct.unpack('>I', raw[8:12])[0]
    assert raw[12:16] == b'IHDR' and size == 13
    assert zlib.crc32(raw[12:29]) & 0xffffffff == struct.unpack('>I', raw[29:33])[0]
    return True


if __name__ == '__main__':
    main_test()
