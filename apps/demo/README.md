# Fusion SLAM map explorer demo

The application is a **local read-only viewer** for existing FUSMAP2D
`.slammap` files and the active/previous `FUSGEN1` generation registry.
It does **not** stream robot poses or control SLAM.

From the repository root, build the standalone C++20 server (needs
Eigen3, zlib and OpenSSL development headers, but **not** Ceres/GTSAM):

```sh
cmake -S . -B build-demo -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_GTSAM=OFF -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_BUILD_NATIVE_DEMO=ON -DSLAM_REQUIRE_NATIVE_DEMO=ON \
  -DSLAM_ENABLE_CATCH2=OFF
cmake --build build-demo --target fusion_slam_demo --parallel 2
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
ctest --test-dir build-demo -R 'native_web_api' --output-on-failure
```

The original Python viewer remains available for comparison and rollback,
not as the default runtime:

```sh
python3 apps/demo/main.py --data-dir apps/demo/maps --example-if-empty
```

Open http://127.0.0.1:8765. To use a different map folder, pass its
path to `--data-dir`.

Like `civcode/hybrid-astar-cpp`, this repository also exposes a native
C++ Docker demo, built without the heavyweight solver libraries:

```sh
docker compose up --build demo
```

The Compose port is bound to **host loopback only**. Files inside
`apps/demo/maps` are mounted read-only at `/maps`. The container listens
on all of its own interfaces solely so Docker's loopback port forwarding
works; **do not publish the HTTP port publicly**, because the viewer
has no authentication. To run native SLAM tests manually:

```sh
docker compose run --build --rm test
```

For map format, CLI options and API details see
[the Web UI guide](../../docs/web-ui.md).

## Interactive recorded LiDAR replay

Open **http://127.0.0.1:8765/replay** after starting the demo. With
`--example-if-empty` and an empty map folder, a **disposable synthetic
session** is provided alongside the virtual workshop map. This is not
recorded laser data.

Convert an existing CARMEN/FR079 FLASER recording into the bounded,
read-only replay format:

```sh
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
```

The converter preserves CARMEN scan times and logged laser/odometry
poses, but **does not run Ceres or infer scan-match diagnostics**.
FR079 pose estimates are not verified ground truth. Map overlays require
matching declared frame IDs, and their alignment is still *unverified*.
Scan points are downsampled for display while valid-hit/beam counts
remain attached to every frame. The UI supports play/pause, variable
speed, seek, stepping, zoom and pan. Existing user files take precedence
over examples. See [recorded replay format and limits](../../docs/recorded-replay.md).

## Offline Ceres scan-matching diagnostics

With a **separately frozen** `.slammap` in this directory and the original
CARMEN recording, build the native `slam_ceres_replay_export` executable,
then generate a `FUSMATCH1` sidecar with `python3 apps/demo/match_ceres.py`.
The replay will display genuine Ceres overlap, robust cost, rejected matches
and optional orange accepted-pose overlays. The browser never runs Ceres.
See [Ceres replay instructions](../../docs/ceres-replay.md).

## Verified held-out Ceres diagnostics

Use `--verified-fr079-holdout` in `apps/demo/match_ceres.py` to evaluate only
scans excluded from the actual source-rebuilt training map. The viewer
shows a held-out summary (status counts, EDT overlap and cost) without
claiming ground-truth positioning accuracy. The command also emits an
exclusive `FUSHOLDOUT_REPORT1` JSON report. See
[held-out evaluation instructions](../../docs/heldout-ceres-evaluation.md).

## Native implementation and compatibility

The C++20 offline server is `apps/demo/native/main.cpp` and
`apps/demo/native/data.cpp`; it imports authoritative `FUSMAP2D`
maps with the existing C++ library, validates the `FUSGEN1` pointer
without recovery/writes, renders PNG with zlib, and parses validated
`FUSREPLAY1` and `FUSMATCH1` without running the solver.

The HTML/CSS/JavaScript under `apps/demo/web/static` is unchanged.
The existing read-only API endpoints, metadata/cell formats,
trajectory playback and optional Ceres overlays are preserved. Both
the native server and the original Python server refuse unsigned,
unregistered or stale sources; metadata labels are still declarations,
not independent real-world truth. The native binary requires no Python
interpreter at runtime; the Python conversion scripts and test suite
are retained.

The native HTTP server is intentionally a **local offline single-threaded
viewer** with bounded request size and timeouts; it is not a real-time
SLAM service and must not be internet-exposed. HTTP writes remain
disabled, and paths are fixed basenames or bundled asset allowlists.
See [the Web UI guide](../../docs/web-ui.md) for verification status.
