"""Native C++ dashboard HTTP/API parity and fail-closed reader regressions.

Python is only the test driver and fixture generator; the viewer runs entirely
as a compiled native process. No pytest/flask/pip/npm/Actions are required.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import socket
import struct
import zlib
import sys
import tempfile
import time
from urllib import error, request
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "apps" / "demo"))
from web.example import create_example_bundle
from web.bundle import MapBundle, read_bundle
from test_synthetic_replay import verify_workshop_replay
from web.replay import create_synthetic_replay


# The old web_ui_unit CTest discovers test_*.py with unittest discover.
# Native HTTP tests require an executable and are only activated by their
# dedicated native_web_api CTest. Do not treat "discover"/flags as a binary.
_NATIVE_EXE = sys.argv[1] if __name__ == "__main__" and len(sys.argv) == 2 else None


@unittest.skipUnless(_NATIVE_EXE is not None, "native HTTP tests run via native_web_api CTest")
class NativeViewerApi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executable = _NATIVE_EXE

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.map = self.root / "record.slammap"
        self.map.write_bytes(create_example_bundle())
        self.session = create_synthetic_replay()
        self.replay = self.root / "record.slamreplay.json"
        self.replay.write_text(json.dumps(self.session, separators=(",", ":")))
        self.child = None
        self.start()

    def tearDown(self):
        if self.child:
            self.child.terminate()
            try:
                self.child.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.communicate(timeout=3)
        self.directory.cleanup()

    def start(self, extra=None):
        self.child = subprocess.Popen(
            [self.executable, "--data-dir", str(self.root),
             "--port", "0", *(extra or [])],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        line = self.child.stdout.readline().strip()
        match = re.search(r"http://127\.0\.0\.1:(\d+)", line)
        self.assertIsNotNone(match, (line, self.child.poll()))
        self.url = f"http://127.0.0.1:{match[1]}"
        for _ in range(20):
            try:
                self.api("/api/health")
                return
            except OSError:
                time.sleep(.025)
        self.fail("native viewer did not start")

    def get(self, path, method="GET"):
        return request.urlopen(
            request.Request(self.url + path, method=method), timeout=8,
        )

    def api(self, path):
        with self.get(path) as response:
            self.assertIn("object-src", response.headers["Content-Security-Policy"])
            self.assertIn("img-src 'self' data: blob:;",
                          response.headers["Content-Security-Policy"])
            self.assertEqual(response.headers["X-Content-Type-Options"], "nosniff")
            return json.load(response)

    def test_maps_and_png_preserve_existing_api(self):
        self.assertEqual(self.api("/api/health")["mode"], "read-only-offline")
        listing = self.api("/api/maps")
        self.assertEqual(listing["maps"][0]["id"], "record.slammap")
        self.assertEqual(listing["registry"]["status"], "absent")
        metadata = self.api("/api/maps/record.slammap")
        truth = read_bundle(self.map).metadata()
        for key in ("format", "width", "height", "resolution_m", "origin",
                    "revision", "fingerprint", "known", "unknown", "occupied",
                    "free", "neutral_known", "frame_id", "source_id",
                    "truth_class", "metric_origin_verified",
                    "clear_trusted_no_returns"):
            self.assertEqual(metadata[key], truth[key], key)
        for x, y in ((0, 0), (40, 30), (119, 89)):
            actual = self.api(f"/api/maps/record.slammap/cell?x={x}&y={y}")
            expected = read_bundle(self.map).cell(x, y)
            for key in ("x", "y", "observed", "log_odds"):
                self.assertEqual(actual[key], expected[key], key)
            for key in ("world_x", "world_y", "occupied_probability"):
                if expected[key] is None:
                    self.assertIsNone(actual[key], key)
                else:
                    self.assertAlmostEqual(actual[key], expected[key], places=12, msg=key)
        with self.get("/api/maps/record.slammap/image.png") as r:
            self.assertEqual(r.headers["Content-Type"], "image/png")
            png = r.read()
            self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(png[12:16], b"IHDR")
            width, height = struct.unpack(">II", png[16:24])
            self.assertEqual((width, height), (120, 90))
            self.assertEqual(png[24:29], b"\x08\x06\x00\x00\x00")
            self.assertEqual(png[-8:-4], b"IEND")
            length = struct.unpack(">I", png[33:37])[0]
            self.assertEqual(png[37:41], b"IDAT")
            uncompressed = zlib.decompress(png[41:41+length])
            self.assertEqual(len(uncompressed), 90 * (120 * 4 + 1))
            self.assertEqual(uncompressed[:5], bytes([0, 21, 34, 54, 255]))
            center = 45 * (120 * 4 + 1) + 1 + 60 * 4
            self.assertEqual(uncompressed[center:center+4],
                             bytes([225, 237, 241, 255]))
        # Repeated PNG and cell inspection must reuse an intact cached map.
        with self.get("/api/maps/record.slammap/image.png") as r:
            self.assertEqual(r.read(), png)
        self.assertEqual(self.api("/api/maps/record.slammap")["fingerprint"],
                         metadata["fingerprint"])
        # Replacing the file must invalidate both snapshot and PNG caching.
        original_bytes = self.map.read_bytes()
        try:
            self.map.write_bytes(b"invalid replacement")
            with self.assertRaises(error.HTTPError) as exc:
                self.get("/api/maps/record.slammap")
            self.assertEqual(exc.exception.code, 422)
            with self.assertRaises(error.HTTPError) as exc:
                self.get("/api/maps/record.slammap/image.png")
            self.assertEqual(exc.exception.code, 422)
        finally:
            self.map.write_bytes(original_bytes)
        self.assertEqual(self.api("/api/maps/record.slammap")["fingerprint"],
                         metadata["fingerprint"])
        for route, mark in (("/", b"<!DOCTYPE html"), ("/app.js", b"fetch("),
                            ("/replay", b"replay"), ("/replay.js", b"replay"),
                            ("/replay.css", b"replay")):
            with self.get(route) as r:
                self.assertIn(mark.lower(), r.read().lower())
                # Map/replay pages decode fetched PNGs using blob object URLs.
                policy = r.headers["Content-Security-Policy"]
                self.assertIn("img-src 'self' data: blob:;", policy)
                self.assertIn("script-src 'self';", policy)

    def test_full_recorded_replay_and_ceres_sidecar(self):
        rows = self.api("/api/replays")
        self.assertEqual(rows["sessions"], ["record.slamreplay.json"])
        s = self.api("/api/replays/record.slamreplay.json")
        self.assertEqual(s["format"], "FUSREPLAY1")
        self.assertEqual(s["frame_count"], 56)
        self.assertEqual(s["ceres"]["status"], "absent")
        batch = self.api("/api/replays/record.slamreplay.json/frames")
        self.assertEqual(batch["format"], "FUSREPLAYFRAMES1")
        self.assertEqual(batch["source_sha256"], s["source"]["source_sha256"])
        self.assertEqual(len(batch["frames"]), s["frame_count"])
        self.assertEqual(batch["frames"][0]["matching_state"], "absent")
        self.assertIsNone(batch["frames"][0]["matching"])
        f = self.api("/api/replays/record.slamreplay.json/frame/0")
        self.assertEqual(batch["frames"][0], f)
        self.assertEqual(f["matching_state"], "absent")
        self.assertIsNone(f["matching"])
        self.assertEqual(f["points"], self.session["frames"][0]["points"])

        meta = read_bundle(self.map).metadata()
        source = self.session["source"]
        row = {
            "index": 0, "t": self.session["frames"][0]["t"],
            "seed": self.session["frames"][0]["pose"],
            "estimate": [self.session["frames"][0]["pose"][0] + .05,
                         *self.session["frames"][0]["pose"][1:]],
            "status": "accepted", "iterations": 4, "input_hits": 16,
            "supported_hits": 16, "overlap": 1.,
            "initial_robust_cost": 5., "robust_cost": 3.,
            "mean_distance_m": .1, "full_rank": True,
            "map_fingerprint": meta["fingerprint"],
        }
        sidecar = {
            "format": "FUSMATCH1", "matcher": "ceres_scan_to_edt",
            "source_sha256": source["source_sha256"],
            "replay_sha256": hashlib.sha256(self.replay.read_bytes()).hexdigest(),
            "map_name": self.map.name, "map_fingerprint": meta["fingerprint"],
            "map_frame_id": meta["frame_id"], "map_revision": meta["revision"],
            "map_origin_verified": False,
            "independent_ground_truth": False,
            "training_independence_verified": False,
            "per_beam_time_verified": False, "frames": [row],
        }
        side = self.root / (self.replay.name + ".ceres.json")
        side.write_text(json.dumps(sidecar))
        active = self.api("/api/replays/record.slamreplay.json")
        self.assertEqual(active["ceres"]["status"], "valid")
        self.assertEqual(active["ceres"]["accepted"], 1)
        matched = self.api("/api/replays/record.slamreplay.json/frame/0")
        self.assertEqual(matched["matching"]["status"], "accepted")
        batch = self.api("/api/replays/record.slamreplay.json/frames")
        self.assertEqual(batch["frames"][0], matched)
        self.assertEqual(batch["frames"][1]["matching_state"], "valid")
        self.assertIsNone(batch["frames"][1]["matching"])

        sidecar["replay_sha256"] = "0" * 64
        side.write_text(json.dumps(sidecar))
        rejected = self.api("/api/replays/record.slamreplay.json")
        self.assertEqual(rejected["ceres"]["status"], "invalid")
        self.assertIsNone(self.api(
            "/api/replays/record.slamreplay.json/frame/0")["matching"])
        invalid_batch = self.api("/api/replays/record.slamreplay.json/frames")
        self.assertEqual(invalid_batch["frames"][0]["matching_state"], "invalid")
        self.assertIsNone(invalid_batch["frames"][0]["matching"])

        # A forged held-out selection must never present source-index
        # separation without the actual frozen FR079 training-map source.
        sidecar["replay_sha256"] = hashlib.sha256(self.replay.read_bytes()).hexdigest()
        sidecar["selection"] = "fr079_heldout_mod5_verified"
        sidecar["query_stride"] = 10
        sidecar["beam_stride"] = 4
        side.write_text(json.dumps(sidecar))
        heldout = self.api("/api/replays/record.slamreplay.json")
        self.assertEqual(heldout["ceres"]["status"], "invalid")
        self.assertIsNone(self.api(
            "/api/replays/record.slamreplay.json/frame/0")["matching"])

    def test_read_only_symlinks_and_bad_files(self):
        for path in ("/api/maps/record.slammap/cell?x=-1&y=0",
                     "/api/maps/record.slammap/cell?x=4&x=5",
                     "/api/replays/record.slamreplay.json/frame/6001",
                     "/api/replays/../etc/passwd"):
            with self.subTest(path=path), self.assertRaises(error.HTTPError):
                self.get(path)
        with self.assertRaises(error.HTTPError) as e:
            self.get("/api/maps", method="POST")
        self.assertEqual(e.exception.code, 405)
        self.assertFalse((self.root / "CURRENT").exists())
        alias = self.root / "alias.slammap"
        alias.symlink_to(self.map)
        with self.assertRaises(error.HTTPError) as e:
            self.get("/api/maps/alias.slammap")
        self.assertEqual(e.exception.code, 422)
        corrupt = self.root / "broken.slammap"
        corrupt.write_bytes(b"garbage")
        with self.assertRaises(error.HTTPError) as e:
            self.get("/api/maps/broken.slammap")
        self.assertEqual(e.exception.code, 422)
        previous = self.replay.read_bytes()
        try:
            self.replay.write_bytes(b'{"format":"FUSREPLAY1","source":{},"frames":[]}')
            with self.assertRaises(error.HTTPError) as e:
                self.get("/api/replays/record.slamreplay.json")
            self.assertEqual(e.exception.code, 422)
        finally:
            self.replay.write_bytes(previous)
        link = self.root / "linked.slamreplay.json"
        link.symlink_to(self.replay)
        with self.assertRaises(error.HTTPError) as e:
            self.get("/api/replays/linked.slamreplay.json")
        self.assertEqual(e.exception.code, 422)

    def test_oversized_http_headers_fail_closed(self):
        endpoint = self.url.removeprefix("http://")
        host, port = endpoint.split(":")
        with socket.create_connection((host, int(port)), timeout=4) as sock:
            sock.settimeout(4)
            payload = ("GET /api/maps HTTP/1.1\r\n" +
                       "X-Long: " + "A" * 9000 + "\r\n\r\n")
            sock.sendall(payload.encode("ascii"))
            first = sock.recv(512)
            self.assertIn(b"400", first.split(b"\r\n", 1)[0])

    def test_synthetic_only_if_folder_empty(self):
        self.child.terminate()
        self.child.communicate(timeout=3)
        self.child = None
        self.map.unlink()
        self.replay.unlink()
        self.start(["--example-if-empty"])
        maps = self.api("/api/maps")["maps"]
        self.assertEqual(maps[0]["id"], "sample-synthetic-workshop.slammap")
        m = self.api("/api/maps/sample-synthetic-workshop.slammap")
        self.assertEqual(m["truth_class"], "synthetic_exact")
        self.assertFalse(m["metric_origin_verified"])
        bundle = MapBundle(create_example_bundle())
        self.assertEqual(m["fingerprint"], bundle.metadata()["fingerprint"])
        self.assertEqual(self.api("/api/replays")["sessions"],
                         ["sample-synthetic-workshop.slamreplay.json"])
        name = "sample-synthetic-workshop.slamreplay.json"
        metadata = self.api(f"/api/replays/{name}")
        expected = create_synthetic_replay()
        self.assertEqual(metadata["source"], expected["source"])
        self.assertEqual(metadata["frame_count"], len(expected["frames"]))
        frames = [self.api(f"/api/replays/{name}/frame/{i}")
                  for i in range(metadata["frame_count"])]
        verify_workshop_replay(self, frames, bundle)
        # Both implementations must produce the same scan geometry and poses.
        for actual, target in zip(frames, expected["frames"]):
            self.assertEqual(actual["hit_count"], target["hit_count"])
            for value, truth in zip(actual["pose"], target["pose"]):
                self.assertAlmostEqual(value, truth, places=9)
            for point, truth in zip(actual["points"], target["points"]):
                for value, component in zip(point, truth):
                    self.assertAlmostEqual(value, component, delta=1e-5)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: test_native_viewer.py <native executable>")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
