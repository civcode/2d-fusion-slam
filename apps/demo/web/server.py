"""Local, read-only HTTP dashboard for FUSMAP2D v1 and FUSGEN1.

No third-party packages. Bound to loopback only; no writes, shell commands,
operator endpoints, file uploads, or websocket robot-status pretensions.
"""
from __future__ import annotations

import argparse
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import stat
import threading
from urllib.parse import parse_qs, urlsplit

from .bundle import BundleError, parse_registry, read_bundle
from .example import data_dir_or_example
from .replay import SAFE_NAME as SAFE_REPLAY, read_replay, ReplayError, create_synthetic_replay, validate
from .ceres_results import SIDE_SUFFIX, MAX_RESULTS, load_results

BASE = Path(__file__).resolve().parent
STATIC = BASE / "static"
SAFE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,180}\.slammap\Z")
MAP_ENDPOINT = re.compile(r"/api/maps/([A-Za-z0-9][A-Za-z0-9_.-]{0,180}\.slammap)(?:/(image\.png|cell))?\Z")
MAX_ENTRIES = 512
REPLAY_ENDPOINT = re.compile(r"/api/replays/([A-Za-z0-9][A-Za-z0-9_.-]{0,140}\.slamreplay\.json)(?:/frame/([0-9]{1,4}))?\Z")


class Dashboard:
    def __init__(self, data_dir: Path, replay_dir: Path | None = None):
        root = data_dir.resolve(strict=True)
        if not root.is_dir():
            raise ValueError("--data-dir must be an existing directory")
        self.root = root
        self.replay_root = (replay_dir or data_dir).resolve(strict=True)
        if not self.replay_root.is_dir():
            raise ValueError("--replay-dir must be an existing directory")
        self.lock = threading.Lock()
        self.synthetic_replay = None

    @lru_cache(maxsize=12)
    def _read_cached(self, name: str, inode: int, size: int, mtime_ns: int, ctime_ns: int):
        del inode, size, mtime_ns, ctime_ns
        return read_bundle(self.root / name)

    def bundle(self, name: str):
        if not SAFE_NAME.fullmatch(name):
            raise FileNotFoundError("invalid map filename")
        p = self.root / name
        st = p.lstat()
        if not stat.S_ISREG(st.st_mode):
            raise BundleError("maps must be regular files, not symlinks")
        # Key incorporates identity and freshness across atomic generation
        # replacements. Each cached bundle is immutable and fully verified.
        with self.lock:
            return self._read_cached(name, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)

    @lru_cache(maxsize=3)
    def _replay_cached(self, name: str, inode: int, size: int, mtime_ns: int, ctime_ns: int):
        del inode, size, mtime_ns, ctime_ns
        return read_replay(self.replay_root / name)

    def replay(self, name: str):
        if not SAFE_REPLAY.fullmatch(name):
            raise FileNotFoundError("invalid replay filename")
        if name == "sample-synthetic-workshop.slamreplay.json" and self.synthetic_replay is not None:
            return self.synthetic_replay
        p = self.replay_root / name
        st = p.lstat()
        if not stat.S_ISREG(st.st_mode):
            raise ReplayError("replay must be a regular file, not a symlink")
        with self.lock:
            return self._replay_cached(name, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)

    def ceres(self, name: str, session: dict):
        """Read optional Ceres diagnostics; never execute a solver over HTTP."""
        if self.synthetic_replay is not None and name == "sample-synthetic-workshop.slamreplay.json":
            return {"status": "absent", "processed": 0}, {}
        sidecar = self.replay_root / (name + SIDE_SUFFIX)
        try:
            st = sidecar.lstat()
        except FileNotFoundError:
            return {"status": "absent", "processed": 0}, {}
        try:
            if not stat.S_ISREG(st.st_mode) or st.st_size > MAX_RESULTS:
                raise ReplayError("Ceres sidecar must be a bounded regular file")
            # Extract only a safe map basename before performing full validation.
            with sidecar.open("rb") as stream:
                preliminary = json.load(stream)
            if not isinstance(preliminary, dict):
                raise ReplayError("invalid Ceres sidecar")
            map_name = preliminary.get("map_name")
            if not isinstance(map_name, str) or not SAFE_NAME.fullmatch(map_name):
                raise ReplayError("invalid Ceres referenced map")
            mapped = self.bundle(map_name)
            session_file = self.replay_root / name
            matched = load_results(sidecar, session_file, session, map_name, mapped)
            rows = {row["index"]: row for row in matched["frames"]}
            accepted = sum(row["status"] == "accepted" for row in rows.values())
            return {"status": "valid", "processed": len(rows), "accepted": accepted,
                    "map_name": map_name, "map_fingerprint": matched["map_fingerprint"],
                    "map_frame_id": matched["map_frame_id"],
                    "independent_ground_truth": False,
                    "training_independence_verified": False}, rows
        except (OSError, ValueError, ReplayError) as exc:
            return {"status": "invalid", "processed": 0,
                    "error": "Ceres results refused: " + str(exc)}, {}

    def replays(self) -> dict:
        names = sorted(p.name for p in self.replay_root.iterdir()
                       if SAFE_REPLAY.fullmatch(p.name) and p.is_file() and not p.is_symlink())
        if self.synthetic_replay is not None and not names:
            names = ["sample-synthetic-workshop.slamreplay.json"]
        if len(names) > MAX_ENTRIES:
            raise ReplayError("too many replay sessions")
        return {"sessions": names, "read_only": True, "live_tracking": False}

    def registry(self) -> dict:
        try:
            registry = parse_registry(self.root / "CURRENT")
        except (OSError, BundleError, ValueError) as e:
            return {"status": "invalid", "active": None, "previous": None,
                    "error": f"Registry refused: {e}"}
        if registry["status"] != "present":
            return registry
        # A pointer is not healthy merely because its metadata checksum works.
        # Verify both the referenced snapshot and its declared fingerprint.
        for role in ("active", "previous"):
            pointer = registry[role]
            if not pointer:
                continue
            try:
                b = self.bundle(pointer["file"])
                if str(b.fingerprint) != pointer["fingerprint"]:
                    raise BundleError(f"{role} fingerprint mismatch")
                pointer["integrity"] = "valid"
            except (OSError, BundleError, ValueError) as e:
                pointer["integrity"] = "invalid"
                registry["status"] = "degraded"
                registry["error"] = f"{role.capitalize()} map refused: {e}. Run native GenerationStore::recover() explicitly."
        return registry

    def index(self) -> dict:
        names = sorted(p.name for p in self.root.iterdir()
                       if SAFE_NAME.fullmatch(p.name) and p.is_file() and not p.is_symlink())
        # Avoid unbounded directory listings in the read-only local service.
        if len(names) > MAX_ENTRIES:
            raise BundleError("too many maps in one directory (max 512)")
        registry = self.registry()
        maps = []
        for name in names:
            role = "file"
            for candidate_role in ("active", "previous"):
                pointer = registry.get(candidate_role)
                if pointer and pointer["file"] == name:
                    role = candidate_role
                    break
            maps.append({"id": name, "name": name, "role": role})
        return {"maps": maps, "registry": registry, "mode": "read-only-offline",
                "live_tracking": False, "publication": False}


class Handler(BaseHTTPRequestHandler):
    server_version = "FusionSLAMViewer/1"

    @property
    def viewer(self) -> Dashboard:
        return self.server.viewer  # type: ignore[attr-defined]

    def log_message(self, format_string: str, *args):
        # Local-only API; do not log whole request bodies or file contents.
        super().log_message(format_string, *args)

    def _send(self, status: int, payload: bytes, mime: str):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, status: int, obj: dict):
        self._send(status, json.dumps(obj, ensure_ascii=True, separators=(",", ":")).encode("utf-8"),
                   "application/json; charset=utf-8")

    def do_GET(self):
        try:
            self._route()
        except (FileNotFoundError, IsADirectoryError):
            self._json(404, {"error": "map not found"})
        except (BundleError, ReplayError, ValueError, IndexError, OSError) as e:
            self._json(422, {"error": str(e)})
        except Exception:
            self.log_error("unexpected viewer exception")
            self._json(500, {"error": "internal viewer error"})

    def do_POST(self):
        self._json(405, {"error": "read-only dashboard: no state-changing HTTP methods"})

    do_PUT = do_POST
    do_DELETE = do_POST
    do_PATCH = do_POST

    def _route(self):
        parsed = urlsplit(self.path)
        path = parsed.path
        if path == "/api/health":
            return self._json(200, {"status": "ok", "mode": "read-only-offline"})
        if path == "/api/replays":
            return self._json(200, self.viewer.replays())
        replay_match = REPLAY_ENDPOINT.fullmatch(path)
        if replay_match:
            name, frame_text = replay_match.groups()
            replay = self.viewer.replay(name)
            match_info, match_rows = self.viewer.ceres(name, replay)
            if frame_text is not None:
                idx = int(frame_text)
                if idx >= len(replay["frames"]):
                    raise ValueError("frame index out of range")
                response = dict(replay["frames"][idx])
                response["matching"] = match_rows.get(idx)
                response["matching_state"] = match_info["status"]
                return self._json(200, response)
            frames = replay["frames"]
            return self._json(200, {"name": name, "format": replay["format"],
                "source": replay["source"], "ceres": match_info, "frame_count": len(frames),
                "first_t": frames[0]["t"], "last_t": frames[-1]["t"],
                "trajectory": [[f["t"], *f["pose"]] for f in frames]})
        if path == "/api/maps":
            return self._json(200, self.viewer.index())
        match = MAP_ENDPOINT.fullmatch(path)
        if match:
            name, subroute = match.group(1), match.group(2)
            m = self.viewer.bundle(name)
            if subroute == "image.png":
                return self._send(200, m.png(), "image/png")
            if subroute == "cell":
                query = parse_qs(parsed.query, keep_blank_values=True)
                if set(query) != {"x", "y"} or len(query["x"]) != 1 or len(query["y"]) != 1:
                    raise ValueError("cell request requires x and y")
                if not re.fullmatch(r"(?:0|[1-9][0-9]{0,7})", query["x"][0]) or not re.fullmatch(r"(?:0|[1-9][0-9]{0,7})", query["y"][0]):
                    raise ValueError("cell coordinates must be nonnegative integers")
                return self._json(200, m.cell(int(query["x"][0]), int(query["y"][0])))
            return self._json(200, m.metadata())
        if path in ("/", "/index.html", "/app.js", "/styles.css", "/replay", "/replay.html", "/replay.js", "/replay.css"):
            filename = "index.html" if path == "/" else ("replay.html" if path == "/replay" else path[1:])
            mime = {"index.html": "text/html; charset=utf-8",
                    "app.js": "text/javascript; charset=utf-8",
                    "styles.css": "text/css; charset=utf-8",
                    "replay.html": "text/html; charset=utf-8",
                    "replay.js": "text/javascript; charset=utf-8",
                    "replay.css": "text/css; charset=utf-8"}[filename]
            return self._send(200, (STATIC / filename).read_bytes(), mime)
        return self._json(404, {"error": "not found"})


def run(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, required=True,
                        help="directory containing FUSMAP2D .slammap files or FUSGEN1 CURRENT")
    parser.add_argument("--replay-dir", type=Path, default=None,
                        help="directory containing .slamreplay.json sessions (default: map data directory)")
    parser.add_argument("--example-if-empty", action="store_true",
                        help="show a disposable labeled synthetic map only when the map directory is empty")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", choices=("127.0.0.1", "0.0.0.0"), default="127.0.0.1",
                        help="bind interface; defaults to loopback only")
    parser.add_argument("--allow-network", action="store_true",
                        help="required for 0.0.0.0, e.g. Docker with host-side loopback port binding")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    if args.host == "0.0.0.0" and not args.allow_network:
        parser.error("binding to all interfaces requires explicit --allow-network")
    with data_dir_or_example(args.data_dir, args.example_if_empty) as directory:
        viewer = Dashboard(directory, args.replay_dir)
        # The synthetic map and virtual scans are both temporary and served
        # only when the explicit --example-if-empty path was selected.
        if args.example_if_empty and directory != args.data_dir.resolve() and args.replay_dir is None:
            viewer.synthetic_replay = create_synthetic_replay()
        server = ThreadingHTTPServer((args.host, args.port), Handler)
        server.viewer = viewer
        print(f"Read-only SLAM map dashboard: http://127.0.0.1:{server.server_port}", flush=True)
        if args.host == "0.0.0.0":
            print("WARNING: HTTP has no authentication; bind Docker host port to 127.0.0.1 only.", flush=True)
        if args.example_if_empty and directory != args.data_dir.resolve():
            print("SYNTHETIC EXAMPLE ONLY: no user map or CURRENT was found.", flush=True)
        print(f"Map directory: {viewer.root}", flush=True)
        print("No live robot status, robot commands, map publication or recovery actions exposed.", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


if __name__ == "__main__":
    run()
