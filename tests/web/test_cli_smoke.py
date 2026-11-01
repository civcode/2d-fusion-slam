"""Run the actual demo CLI/API with an empty map folder (no UI mocks)."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
ENTRYPOINT = ROOT / 'apps/demo/main.py'


class DemoCLITests(unittest.TestCase):
    def test_empty_map_dir_starts_synthetic_example_without_writing_user_data(self):
        with tempfile.TemporaryDirectory() as d:
            user_dir = Path(d)
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0))
                port = sock.getsockname()[1]
            process = subprocess.Popen(
                [sys.executable, str(ENTRYPOINT), '--data-dir', d,
                 '--example-if-empty', '--port', str(port)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            try:
                endpoint = f'http://127.0.0.1:{port}'
                deadline = time.monotonic() + 6
                while True:
                    try:
                        with urlopen(endpoint + '/api/maps', timeout=.5) as response:
                            listing = json.load(response)
                        break
                    except OSError:
                        if process.poll() is not None or time.monotonic() >= deadline:
                            self.fail('demo CLI failed to start and serve the map index')
                        time.sleep(.05)
                self.assertEqual(len(listing['maps']), 1)
                name = listing['maps'][0]['id']
                self.assertEqual(name, 'sample-synthetic-workshop.slammap')
                with urlopen(endpoint + '/api/maps/' + name, timeout=2) as response:
                    metadata = json.load(response)
                self.assertEqual(metadata['truth_class'], 'synthetic_exact')
                self.assertFalse(metadata['metric_origin_verified'])
                with urlopen(endpoint + '/api/maps/' + name + '/image.png', timeout=2) as response:
                    self.assertTrue(response.read().startswith(b'\x89PNG\r\n\x1a\n'))
                self.assertEqual(list(user_dir.iterdir()), [])
            finally:
                process.terminate()
                process.wait(timeout=4)


if __name__ == '__main__':
    unittest.main()
