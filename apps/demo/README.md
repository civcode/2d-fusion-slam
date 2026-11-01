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


## Fusion process replay (native C++)

The **Fusion Replay** page at **http://127.0.0.1:8765/fusion** visualizes
an actual completed `slam_offline_fusion` run, rather than pretending
to show a live robot. It supports timeline scrub/play/pause/step, speed
adjustment, 2D zoom/pan, optional final-map overlay, method-by-method
trajectories, sampled moving LiDAR returns, cumulative scan-hit
observations, held-out Ceres/reference matcher status, source-exclusion
decisions, GTSAM independent-sensor loop events and per-method final
trajectory RMSE/map IoU. Graph and synthetic truth overlays are explicitly
labeled as post-run information.

Generate a run from the repository root, with an output directory that
does **not** already exist:

~~~sh
cmake --build build --target slam_offline_fusion
./build/slam_offline_fusion apartment apps/demo/fusion/apartment-001

# Native demo on the host; no Python required for the server:
./build-demo/fusion_slam_demo \
  --data-dir apps/demo/maps --fusion-dir apps/demo/fusion --port 8765
# Open http://127.0.0.1:8765/fusion
~~~

Alternatively, `docker compose up --build demo` mounts
`apps/demo/fusion` at `/fusion:ro` as well as the separate map
directory at `/maps:ro`. Generate any number of clearly named run
subdirectories and select them in the Fusion Replay dropdown. Allowed
run IDs contain ASCII letters, digits, underscores or hyphens.

New read-only native endpoints:

- `GET /api/fusion/runs`: discover direct child run directories
- `GET /api/fusion/runs/{id}`: validated FUSFUSION1 report
- `GET /api/fusion/runs/{id}/trajectories`: validated FUSPOSES1
- `GET /api/fusion/runs/{id}/replay`: full validated FUSFUSREPLAY1
- `GET /api/fusion/runs/{id}/frame/{index}`: one validated replay frame
- `GET /api/fusion/runs/{id}/maps/{method}`: authoritative map metadata
- `GET /api/fusion/runs/{id}/maps/{method}/image.png`: authoritative
  fingerprint-checked *final* occupancy PNG

All artifacts are read from disk, not generated in HTTP handlers. The
server refuses symlinked run directories/files, missing or contradictory
metadata/clock/match events, changed map fingerprints and path traversal.
No HTTP POST/PUT controls are supported. The file formats provide
integrity/provenance checks but **not cryptographic authenticity**.

**Interpretation:** the replay projects precomputed deskewed scan samples
using the scan-matched trajectory. The accumulating green dots represent
**observed hit endpoints, not the time-varying occupancy probability
grid**. An optional occupancy raster shows the *final* map, and GTSAM
poses show the *post-run* graph solution; those final results did not
necessarily exist at the replayed instant. The browser never reruns
Ceres, GTSAM or SLAM and does not claim real hardware calibration.

Native CTest `fusion_replay_http` runs the real C++ offline fusion CLI,
loads its output through the native HTTP server, checks frame/matcher
totals and map fingerprints, and verifies corrupted files, symlinks and
writes fail closed. See [Fusion Replay protocol](../../docs/fusion-replay.md).


## Fusion results in the Map Explorer and SLAM profiling

The main / Map Explorer now lists the final authoritative occupancy
maps for every valid run inside --fusion-dir. For example,
"apartment-001 / fused_scan_matched" and "warehouse-001 / graph_optimized"
can be opened with the normal PNG viewer and cell inspector. Results are
read from the run directory, **not copied into the map directory**.
The C++ server verifies map fingerprints against the matching fusion
report; the workshop fallback is not preferred when a fusion map exists.

The recorded-LiDAR /replay page remains dedicated to FUSREPLAY1 recorded
sessions; apartment and warehouse simulated scans and matcher decisions
are available in /fusion, with links between the views.

New offline runs export fusion_profile.json with measured native processing
stage timings (integration, deskew, matching, graph, and occupancy
building), p50/p95, a synthetic offline compute FPS estimate, and an
input-rate budget comparison. /fusion displays the profile from the
read-only /api/fusion/runs/{id}/profile endpoint. Missing profiles on
older runs do not prevent map/replay viewing; regenerate the run for
profiling data.

Manual Docker benchmarking with BUILD_JOBS=28:

~~~sh
export BUILD_JOBS=28 SLAM_DOCKER_JOBS=28 SLAM_PROFILE_FUSION=ON
docker compose build test demo
mkdir -p benchmarks
docker compose run --rm -v "$PWD/benchmarks:/results" \
  --entrypoint python3 test scripts/benchmark-fusion.py \
  --binary ./build-docker/slam_offline_fusion \
  --scene both --warmup 1 --runs 5 \
  --csv /results/fusion-profiled-001.csv
~~~

This is NOT an online scheduling/real-time validation. See
[profiling methodology](../../docs/fusion-profiling.md).
