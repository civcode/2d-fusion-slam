"""Synthetic fallback replay does not affect any existing user evidence."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps'/'demo'))
from web.example import WIDTH, HEIGHT, create_example_bundle, data_dir_or_example
from web.bundle import MapBundle
from web.replay import _workshop_ray_distance, create_synthetic_replay
from web.server import Dashboard

def _runs(bundle, blocked):
    """Independent rectangle representation of the parsed raster (not DDA)."""
    result = []
    for y in range(bundle.height):
        start = None
        for x in range(bundle.width + 1):
            active = x < bundle.width and blocked(bundle.cell(x, y))
            if active and start is None:
                start = x
            if not active and start is not None:
                r = bundle.resolution
                result.append((start*r, x*r, y*r, (y+1)*r))
                start = None
    return result


def _first_surface(rectangles, x, y, dx, dy):
    """Slab intersection oracle, independent of the fixture's grid traversal."""
    nearest = math.inf
    for left, right, bottom, top in rectangles:
        enter, leave = 0.0, math.inf
        for origin, direction, low, high in ((x, dx, left, right), (y, dy, bottom, top)):
            if abs(direction) < 1e-12:
                if not low <= origin < high:
                    leave = -1
                    break
            else:
                a, b = sorted(((low-origin)/direction, (high-origin)/direction))
                enter, leave = max(enter, a), min(leave, b)
        # A corner-only touch has no occupied interior along the ray.
        if leave - enter > 1e-9:
            nearest = min(nearest, enter)
    return None if math.isinf(nearest) else nearest


def verify_workshop_replay(test, frames, bundle):
    occupied = _runs(bundle, lambda cell: cell['observed'] and cell['log_odds'] > 0)
    blocked = _runs(bundle, lambda cell: not cell['observed'] or cell['log_odds'] >= 0)
    interior_hits = 0
    # Bounds include the rasterized cell faces, not just schematic centers.
    objects = ((3., 4.3, 1.8, 4.4), (6.6, 8.2, 4., 5.4),
               (8.9, 10.2, 1.8, 3.7), (5.5, 6., 6.6, 8.1),
               (1.8, 2.2, 6.5, 7.8))
    seen = set()
    misses = 0
    for frame in frames:
        x, y, yaw = frame['pose']
        expected = []
        for beam in range(64):
            ray = -math.pi/2 + math.pi*beam/64
            dx, dy = math.cos(yaw+ray), math.sin(yaw+ray)
            distance = _first_surface(occupied, x, y, dx, dy)
            if distance is None:
                misses += 1
                continue
            expected.append((distance*math.cos(ray), distance*math.sin(ray)))
            hx, hy = x+distance*dx, y+distance*dy
            interior_hits += 1 < hx < 11 and 1 < hy < 8
            for i, (left, right, bottom, top) in enumerate(objects):
                if left-1e-6 <= hx <= right+1e-6 and bottom-1e-6 <= hy <= top+1e-6:
                    seen.add(i)
        test.assertEqual(frame['beam_count'], 64)
        test.assertEqual(frame['hit_count'], len(expected))
        test.assertEqual(len(frame['points']), len(expected))
        for actual, target in zip(frame['points'], expected):
            for value, truth in zip(actual, target):
                test.assertAlmostEqual(value, truth, delta=2e-5)
    test.assertGreater(interior_hits, 500, 'interior objects must occlude outer walls')
    test.assertEqual(seen, set(range(len(objects))), 'every visible interior object must be detected')
    test.assertGreater(misses, 0, 'unknown map openings must not fabricate range hits')

    # Check the swept circular body, not just the sampled laser origins.
    # Include the closing segment of the loop and all unknown cells as blocked.
    for first, second in zip(frames, frames[1:] + frames[:1]):
        x0, y0 = first['pose'][:2]
        x1, y1 = second['pose'][:2]
        steps = max(1, math.ceil(math.hypot(x1-x0, y1-y0)/.05))
        for i in range(steps + 1):
            x, y = x0+(x1-x0)*i/steps, y0+(y1-y0)*i/steps
            for left, right, bottom, top in blocked:
                distance = math.hypot(max(left-x, 0, x-right), max(bottom-y, 0, y-top))
                test.assertGreaterEqual(distance + 1e-9, .2, 'robot body intersects blocked map cells')


class SyntheticReplayTests(unittest.TestCase):
    def test_synthetic_session_and_timeline(self):
        data=create_synthetic_replay()
        self.assertEqual(data['format'],'FUSREPLAY1')
        self.assertEqual(len(data['frames']),56)
        self.assertEqual(data['frames'][0]['beam_count'],64)
        self.assertGreater(len(data['frames'][0]['points']),0)
        self.assertEqual(data['source']['pose_kind'],'synthetic_exact')
        self.assertFalse(data['source']['map_alignment_verified'])

    def test_raycast_axis_and_corner_cases(self):
        bundle = MapBundle(create_example_bundle())
        occupied = [c['observed'] and c['log_odds'] > 0
                    for y in range(HEIGHT) for x in range(WIDTH)
                    for c in [bundle.cell(x, y)]]
        rectangles = _runs(bundle, lambda c: c['observed'] and c['log_odds'] > 0)
        for dx, dy in ((1., 0.), (-1., 0.), (0., 1.), (0., -1.),
                       (math.sqrt(.5), math.sqrt(.5))):
            self.assertAlmostEqual(_workshop_ray_distance(occupied, 8.5, 4.5, dx, dy),
                                   _first_surface(rectangles, 8.5, 4.5, dx, dy))
        cells = [False] * (WIDTH * HEIGHT)
        cells[WIDTH + 1] = True
        self.assertAlmostEqual(_workshop_ray_distance(cells, .05, .05, math.sqrt(.5), math.sqrt(.5)),
                               math.sqrt(.005))
        cells[WIDTH + 1] = False
        cells[1] = True # A corner-only side contact is not a return.
        self.assertIsNone(_workshop_ray_distance(cells, .05, .05, math.sqrt(.5), math.sqrt(.5)))
        self.assertIsNone(_workshop_ray_distance(cells, .05, .05, -1., 0.))

    def test_scan_geometry_and_swept_body_match_map(self):
        verify_workshop_replay(self, create_synthetic_replay()['frames'],
                               MapBundle(create_example_bundle()))

    def test_example_mode_serves_virtual_session_without_user_file(self):
        with tempfile.TemporaryDirectory() as root:
            with data_dir_or_example(Path(root),True) as generated:
                self.assertNotEqual(generated,Path(root))
                viewer=Dashboard(generated)
                viewer.synthetic_replay=create_synthetic_replay()
                self.assertEqual(len(viewer.replays()['sessions']),1)
                self.assertEqual(len(viewer.replay(viewer.replays()['sessions'][0])['frames']),56)
            self.assertEqual(list(Path(root).iterdir()),[])

    def test_existing_recordings_never_hidden_by_synthetic_map(self):
        with tempfile.TemporaryDirectory() as root:
            (Path(root)/'recording.slamreplay.json').write_text('corrupt')
            with data_dir_or_example(Path(root),True) as selected:
                self.assertEqual(selected,Path(root))
                viewer=Dashboard(selected)
                self.assertEqual(viewer.replays()['sessions'],['recording.slamreplay.json'])

if __name__=='__main__':unittest.main()
