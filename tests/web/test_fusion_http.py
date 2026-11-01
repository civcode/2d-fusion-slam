#!/usr/bin/env python3
"""Native FUSFUSREPLAY1 HTTP end-to-end tests; executed locally by CTest only.

Runs the ACTUAL synthetic C++ fusion estimator and tests its finished artifact
against the native viewer, without starting any solver in the HTTP process.
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import unittest
from urllib import request, error
from urllib.parse import quote


_FUSION_BINARIES = (__name__ == "__main__" and len(sys.argv) == 3)

@unittest.skipUnless(_FUSION_BINARIES, "native fusion HTTP tests require explicit CTest executables")
class FusionHTTP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.viewer, cls.cli = sys.argv[1:]
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        cls.run_dir = cls.root / "apartment_001"
        result = subprocess.run(
            [cls.cli, "apartment", str(cls.run_dir)],
            capture_output=True, text=True, timeout=180,
        )
        if result.returncode:
            raise AssertionError("native fusion CLI failed:\n" + result.stderr)
        cls.child = subprocess.Popen(
            [cls.viewer, "--data-dir", str(cls.root),
             "--fusion-dir", str(cls.root), "--port", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        line = cls.child.stdout.readline()
        match = re.search(r"http://127\.0\.0\.1:(\d+)", line)
        if not match:
            raise AssertionError("native viewer unavailable: " + repr(line))
        cls.base = "http://127.0.0.1:" + match[1]
        for _ in range(60):
            try:
                cls.get_json("/api/health")
                break
            except OSError:
                time.sleep(.05)
        else:
            raise AssertionError("native viewer health never returned")

    @classmethod
    def tearDownClass(cls):
        if getattr(cls, "child", None):
            cls.child.terminate()
            try:
                cls.child.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                cls.child.kill()
                cls.child.communicate(timeout=3)
        if getattr(cls, "tmp", None):
            cls.tmp.cleanup()

    @classmethod
    def get_json(cls, path):
        with request.urlopen(cls.base + path, timeout=15) as response:
            assert "no-store" in response.headers["Cache-Control"]
            return json.load(response)

    def test_discovery_report_and_real_frame_events(self):
        listed = self.get_json("/api/fusion/runs")
        self.assertEqual(listed["mode"], "read-only-offline")
        self.assertEqual([x["id"] for x in listed["runs"]], ["apartment_001"])
        base = "/api/fusion/runs/apartment_001"
        report = self.get_json(base)
        profiling = self.get_json(base + "/profile")
        self.assertEqual(profiling["format"], "FUSPROFILE1")
        self.assertEqual(profiling["frames"], report["frames"])
        self.assertEqual(profiling["scene"], report["scene"])
        self.assertEqual(profiling["matcher"], report["matcher"])
        self.assertFalse(profiling["realtime_capability_verified"])
        if profiling["profiled"]:
            self.assertGreater(profiling["effective_processing_fps"], 0)
            self.assertGreaterEqual(profiling["frame_proxy_p95_ms"], 0)
        else:
            self.assertIsNone(profiling["effective_processing_fps"])
        poses = self.get_json(base + "/trajectories")
        replay = self.get_json(base + "/replay")
        self.assertEqual(report["format"], "FUSFUSION1")
        self.assertEqual(poses["format"], "FUSPOSES1")
        self.assertEqual(replay["format"], "FUSFUSREPLAY1")
        self.assertEqual(replay["source"], report["source"])
        self.assertEqual(replay["scene"], report["scene"])
        self.assertEqual(replay["matcher"], report["matcher"])
        self.assertFalse(report["map_pose_truth_used"])
        self.assertEqual(len(replay["frames"]), report["frames"])
        self.assertEqual(len(poses["timestamps_ns"]), report["frames"])
        self.assertEqual(len(poses["synthetic_truth_for_scoring"]), report["frames"])
        self.assertEqual(set(poses["paths"]), set(report["results"]))
        accepted = rejected = 0
        for i, frame in enumerate(replay["frames"]):
            self.assertEqual(frame["index"], i)
            self.assertEqual(frame["time_ns"], poses["timestamps_ns"][i])
            self.assertGreater(frame["hits"], 0)
            self.assertGreater(len(frame["points_laser"]), 0)
            self.assertLessEqual(len(frame["points_laser"]), frame["hits"])
            self.assertEqual(frame["matching"]["attempted"], i % 2 == 1)
            self.assertTrue(frame["mapping"]["fused_scan_matched_inserted"])
            if i % 2 == 1:
                if frame["matching"]["accepted"]:
                    accepted += 1
                else:
                    rejected += 1
            self.assertEqual(frame["matching"]["cumulative_accepted"], accepted)
            self.assertEqual(frame["matching"]["cumulative_rejected"], rejected)
        self.assertEqual(accepted, report["heldout_accepted"])
        self.assertEqual(rejected, report["heldout_rejected"])
        self.assertEqual(self.get_json(base + "/frame/0"), replay["frames"][0])
        last = report["frames"] - 1
        self.assertEqual(self.get_json(base + f"/frame/{last}"),
                         replay["frames"][last])

    def test_map_fingerprint_png_assets_and_no_mutation(self):
        base = "/api/fusion/runs/apartment_001"
        report = self.get_json(base)
        merged = self.get_json("/api/maps")
        fusion_rows = [x for x in merged["maps"]
                       if x["role"] == "fusion"]
        self.assertEqual(len(fusion_rows), len(report["results"]))
        self.assertEqual({x["id"] for x in fusion_rows},
                         {"fusion@apartment_001@" + method
                          for method in report["results"]})
        for method in report["results"]:
            metadata = self.get_json(base + f"/maps/{method}")
            map_id = "fusion@apartment_001@" + method
            # The real browser uses encodeURIComponent(), including for @.
            explorer = "/api/maps/" + quote(map_id, safe="")
            self.assertEqual(self.get_json("/api/maps/" + map_id), metadata)
            self.assertEqual(self.get_json(explorer), metadata)
            cell = self.get_json(explorer + "/cell?x=0&y=0")
            self.assertEqual((cell["x"], cell["y"]), (0, 0))
            self.assertIn("observed", cell)
            self.assertEqual(metadata["truth_class"], "synthetic_estimated")
            self.assertEqual(metadata["frame_id"], "map")
            self.assertEqual(metadata["source_id"], report["scene"])
            self.assertEqual(metadata["fingerprint"],
                             str(report["results"][method]["map"]["map_fingerprint"]))
            with request.urlopen(self.base + explorer + "/image.png") as r:
                self.assertEqual(r.headers["Content-Type"], "image/png")
                self.assertEqual(r.read(8), b"\x89PNG\r\n\x1a\n")
            with request.urlopen(self.base + base + f"/maps/{method}/image.png") as r:
                self.assertEqual(r.headers["Content-Type"], "image/png")
                self.assertEqual(r.read(8), b"\x89PNG\r\n\x1a\n")
        for path, marker in (("/fusion", b"FUSION"), ("/fusion.js", b"fusion"),
                             ("/fusion.css", b"fusion"), ("/fusion.html", b"replay")):
            with request.urlopen(self.base + path) as r:
                self.assertIn(marker.lower(), r.read().lower())
        with self.assertRaises(error.HTTPError) as raised:
            request.urlopen(request.Request(self.base + base, method="POST"))
        self.assertEqual(raised.exception.code, 405)
        self.assertFalse((self.root / "CURRENT").exists())

    def test_map_listing_keeps_valid_maps_when_another_run_is_broken(self):
        broken = self.root / "broken_run"
        broken.mkdir()
        files = [broken / name for name in
                 ("fusion_report.json", "trajectories.json", "fusion_replay.json")]
        try:
            for file in files:
                file.write_text("{}")
            report = self.get_json("/api/fusion/runs/apartment_001")
            expected = {"fusion@apartment_001@" + method for method in report["results"]}
            for _ in range(3):
                listing = self.get_json("/api/maps")
                self.assertEqual({row["id"] for row in listing["maps"]
                                  if row["role"] == "fusion"}, expected)
            with self.assertRaises(error.HTTPError) as raised:
                request.urlopen(self.base + "/api/fusion/runs/broken_run")
            self.assertEqual(raised.exception.code, 422)
        finally:
            for file in files:
                file.unlink(missing_ok=True)
            broken.rmdir()

    def test_fail_closed_symlink_paths_and_stale_artifacts(self):
        base = "/api/fusion/runs/apartment_001"
        bad_routes = (
            "/api/fusion/runs/../fusion_report.json",
            "/api/fusion/runs/apartment_001/maps/not_a_method",
            "/api/fusion/runs/apartment_001/frame/10000",
            "/api/fusion/runs/apartment_001/frame/-1",
            "/api/fusion/runs/apartment_001/replay?unexpected=1",
            "/api/maps/fusion%40apartment_001%2F..%40wheel_only",
            "/api/maps/fusion%2540apartment_001%2540wheel_only",
            "/api/maps/fusion%40apartment_001%40wheel_only%00",
            "/api/maps/fusion%40apartment_001%40wheel_only%2Fimage.png",
        )
        for route in bad_routes:
            with self.subTest(route=route), self.assertRaises(error.HTTPError):
                request.urlopen(self.base + route)
        bad_dir = self.root / "aliased"
        bad_dir.symlink_to(self.run_dir, target_is_directory=True)
        with self.assertRaises(error.HTTPError) as raised:
            request.urlopen(self.base + "/api/fusion/runs/aliased")
        self.assertEqual(raised.exception.code, 422)
        trace_file = self.run_dir / "fusion_replay.json"
        previous = trace_file.read_bytes()
        try:
            trace = json.loads(previous)
            trace["frames"][1]["matching"]["cumulative_accepted"] = 10000
            trace_file.write_text(json.dumps(trace))
            with self.assertRaises(error.HTTPError) as raised:
                request.urlopen(self.base + base + "/frame/1")
            self.assertEqual(raised.exception.code, 422)
        finally:
            trace_file.write_bytes(previous)
        map_file = self.run_dir / "wheel_only.slammap"
        previous_map = map_file.read_bytes()
        try:
            map_file.write_bytes(b"corrupt")
            with self.assertRaises(error.HTTPError) as raised:
                request.urlopen(self.base + base + "/maps/wheel_only/image.png")
            self.assertEqual(raised.exception.code, 422)
        finally:
            map_file.write_bytes(previous_map)
        profile_file = self.run_dir / "fusion_profile.json"
        previous_profile = profile_file.read_bytes()
        try:
            invalid = json.loads(previous_profile)
            invalid["frames"] = 999999
            profile_file.write_text(json.dumps(invalid))
            with self.assertRaises(error.HTTPError) as raised:
                request.urlopen(self.base + base + "/profile")
            self.assertEqual(raised.exception.code, 422)
        finally:
            profile_file.write_bytes(previous_profile)
        self.assertEqual(self.get_json(base + "/profile")["format"],
                         "FUSPROFILE1")
        self.assertEqual(self.get_json(base + "/frame/1")["index"], 1)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: test_fusion_http.py <fusion_slam_demo> <slam_offline_fusion>")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
