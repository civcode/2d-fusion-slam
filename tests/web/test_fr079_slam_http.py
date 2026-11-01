#!/usr/bin/env python3
"""Native HTTP smoke test: real FR079 offline SLAM export -> Map Explorer/Replay."""
from __future__ import annotations
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from urllib import request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fr079_offline_test import carmen_fixture, run  # noqa: E402


def main():
    if len(sys.argv) != 3:
        raise SystemExit("Usage: test_fr079_slam_http.py VIEWER_EXE FR079_SLAM_EXE")
    viewer = Path(sys.argv[1]).resolve(strict=True)
    slam = Path(sys.argv[2]).resolve(strict=True)
    with tempfile.TemporaryDirectory(prefix="fr079-slam-native-http-") as temp:
        root = Path(temp)
        log = root / "sample.log"
        log.write_text(carmen_fixture())
        run(slam, log, root / "fr079-estimated", "--reference",
            "--match-every", "3", "--field-every", "5")
        proc = subprocess.Popen(
            [str(viewer), "--data-dir", str(root), "--fusion-dir", str(root),
             "--port", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            assert proc.stdout is not None
            first = proc.stdout.readline()
            found = re.search(r"http://127\.0\.0\.1:(\d+)", first)
            assert found, f"native server failed to listen: {first!r}"
            base = f"http://127.0.0.1:{found.group(1)}"

            def get(path):
                with request.urlopen(base + path, timeout=15) as result:
                    assert result.status == 200
                    return result.read(), result.headers.get("Content-Type", "")

            maps = json.loads(get("/api/maps")[0])
            assert "fr079-estimated.slammap" in [m["id"] for m in maps["maps"]]
            meta = json.loads(get("/api/maps/fr079-estimated.slammap")[0])
            assert meta["truth_class"] == \
                "estimated_trajectory_no_independent_ground_truth"
            assert meta["metric_origin_verified"] is False
            img, mime = get("/api/maps/fr079-estimated.slammap/image.png")
            assert mime == "image/png" and img.startswith(b"\x89PNG\r\n\x1a\n")
            sessions = json.loads(get("/api/replays")[0])
            assert "fr079-estimated.slamreplay.json" in sessions["sessions"]
            replay = json.loads(get(
                "/api/replays/fr079-estimated.slamreplay.json")[0])
            assert replay["frame_count"] == 55
            assert replay["source"]["frame_id"] == meta["frame_id"]
            assert replay["source"]["pose_kind"] == \
                "estimated_fr079_odometry_scan_matched_not_ground_truth"
            assert replay["ceres"]["status"] in ("absent", "unavailable", "none")
            point = json.loads(get(
                "/api/replays/fr079-estimated.slamreplay.json/frame/10")[0])
            assert point["index"] == 10
            assert point["matching"] is None
            assert point["pose"] == replay["trajectory"][10][1:]
            assert "text/html" in get("/replay")[1]
            print("FR079 causal offline SLAM map + replay HTTP integration passed")
        finally:
            proc.terminate()
            try:
                proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate(timeout=5)


if __name__ == "__main__":
    main()
