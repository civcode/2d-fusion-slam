#!/usr/bin/env python3
"""Source-independent regression for late-frame loop propagation diagnostics."""
from __future__ import annotations

import contextlib
import io
import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import fusion_loop_compare


class LoopCompareTest(unittest.TestCase):
    def fixture(self, folder, loops):
        folder.mkdir()
        truth = [[i * 0.1, 0.0, 0.0] for i in range(20)]
        paths = {name: list(truth) for name in
                 ("wheel_only", "lidar_only", "gyro_wheel",
                  "fused_scan_matched")}
        scale = 0.10 if not loops else 0.03
        paths["graph_optimized"] = [[p[0] + i * scale / 19, 0.0, 0.0]
                                    for i, p in enumerate(truth)]
        report = {
            "scene": "synthetic-office-v1", "frames": 20,
            "lidar_loop_closure": loops,
            "graph_lidar_loop_registration_attempted": int(loops),
            "graph_lidar_loop_registration_verified": int(loops),
            "graph_accepted_loops": int(loops), "graph_loop_candidates": int(loops),
            "results": {
                name: {
                    "translation_rmse_m": 0.1,
                    "map": {"map_fingerprint": "123",
                            "metrics": {"occupied_iou": .07 if loops else .05}}
                }
                for name in paths
            }
        }
        frames = [{"loop_events": []} for _ in truth]
        if loops:
            frames[18]["loop_events"] = [
                {"from": 0, "to": 18, "decision": "accepted",
                 "source": "lidar_scan_registration"}]
        (folder / "fusion_report.json").write_text(json.dumps(report))
        (folder / "fusion_replay.json").write_text(json.dumps({"frames": frames}))
        (folder / "trajectories.json").write_text(json.dumps({
            "paths": paths, "synthetic_truth_for_scoring": truth}))
        return paths

    def test_final_window_and_closure_boundary(self):
        with tempfile.TemporaryDirectory() as work:
            root = pathlib.Path(work)
            old = root / "base"
            new = root / "loops"
            self.fixture(old, False)
            self.fixture(new, True)
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                fusion_loop_compare.compare(old, new)
            output = buffer.getvalue()
            self.assertIn("Non-graph paths and map fingerprints: unchanged", output)
            self.assertIn("Final 12%", output)
            self.assertIn("Up to final closure", output)
            self.assertIn("After final closure", output)
            self.assertIn("Frame  18", output)
            self.assertIn("Frame  19", output)
            self.assertIn("0->18", output)

    def test_explicit_old_loop_version_comparison(self):
        with tempfile.TemporaryDirectory() as work:
            root = pathlib.Path(work)
            old = root / "old-loop"
            new = root / "multi-loop"
            self.fixture(old, False)
            self.fixture(new, True)
            report_file = old / "fusion_report.json"
            report = json.loads(report_file.read_text())
            report["lidar_loop_closure"] = True
            report["graph_accepted_loops"] = 1
            report_file.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "allow-loop-baseline"):
                fusion_loop_compare.compare(old, new)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                fusion_loop_compare.compare(
                    old, new, allow_loop_baseline=True)
            self.assertIn("algorithm-version comparison", output.getvalue())
            self.assertIn("Earlier loop mode", output.getvalue())

    def test_source_method_not_allowed_to_change(self):
        with tempfile.TemporaryDirectory() as work:
            root = pathlib.Path(work)
            old = root / "base"
            new = root / "loops"
            self.fixture(old, False)
            self.fixture(new, True)
            new_poses = new / "trajectories.json"
            payload = json.loads(new_poses.read_text())
            payload["paths"]["lidar_only"][5] = [123, 123, 0]
            new_poses.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "non-graph baseline modified"):
                fusion_loop_compare.compare(old, new)


if __name__ == "__main__":
    unittest.main()
