"""C++/Python FUSMAP2D v1 interoperability gate for the startup example."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'apps' / 'demo'))
from web.example import create_example_bundle
from web.bundle import MapBundle


def run(inspector: str):
    payload = create_example_bundle()
    python_map = MapBundle(payload)
    with tempfile.TemporaryDirectory(prefix='fusion-slam-native-check-') as d:
        artifact = Path(d) / 'sample-synthetic-workshop.slammap'
        artifact.write_bytes(payload)
        result = subprocess.run([inspector, str(artifact)], capture_output=True,
                                text=True, check=True)
        native = json.loads(result.stdout)
    for key in ('width', 'height', 'known', 'unknown', 'revision', 'fingerprint',
                'frame_id', 'source_id', 'truth_class', 'metric_origin_verified'):
        expected = python_map.metadata()[key]
        if key in ('revision', 'fingerprint'):
            expected = int(expected)
        if native[key] != expected:
            raise AssertionError(f'{key}: C++ {native[key]!r} versus Python {expected!r}')
    print('Synthetic FUSMAP2D fixture passes native C++ importer and Python parser')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('Usage: check_native_fixture.py <slam_map_inspect>')
    run(sys.argv[1])
