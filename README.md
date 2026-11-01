# 2D Fusion SLAM

**Status: tested synthetic GTSAM 4.3 Pose2 batch graph, versioned offline map export, and FR079 geometric proxy benchmark.** The C++20 core now parses Freiburg FR079 CARMEN logs, converts scans to matcher-native laser points and supports offline Catch2 3.16.1 and Ceres 2.2.0 dependencies. **This is not yet a complete SLAM system or a validated real-dataset SLAM benchmark.** See [implementation status](docs/implementation-status.md).

The planned system uses an **LD19 2D LiDAR and MPU6050 gyroscope**, with optional **actually measured** wheel odometry. Three target modes share a portable core:

| Mode | Platform | Function |
| --- | --- | --- |
| Offline SLAM | Powerful workstation | Ceres scan matching, occupancy mapping and GTSAM batch optimization |
| Known-map localization | Raspberry Pi 5 | Read-only verified map and bounded pose tracking, no initial GTSAM |
| Online SLAM | Raspberry Pi 5 (later) | Resource-limited mapping, designed from benchmark evidence |

**Development strategy:** Run initial implementation/tests/benchmarks on synthetic and verified opt-in public data. Develop real LD19/MPU6050 Pi acquisition/recording **in parallel**, without blocking synthetic/public tests. Optimizations and online graph complexity are selected using measurements, not assumptions.

## What works today

- `slam_core`: checked signed 64-bit nanosecond arithmetic with explicit clock domains/epochs, SE(2) compose/inverse/transform/exp/log, constant-body-twist integration.
- `slam_fixtures`: deterministic apartment (15 × 12 m, doorway gaps) and warehouse (30 × 20 m, repeated shelves) with exact ground-truth moving-ray acquisition times and hit/no-return classification.
- `slam_sensors`: time-aware, source/calibration-consistent angular-seam assembly, full/partial classification, actual acquisition bounds and observed-sector coverage; invalid/missing/no-return remain distinct.
- `slam_mapping`: bounded rotated-origin log-odds occupancy with independent known mask, clipping, one update per touched cell, hit-over-free priority and default-safe no-return policy. Callers supply trusted world-space rays; no automatic deskew is implied.
- `slam_ld19`: 47-byte LD19 packet decoding, CRC-8, corrupted-stream recovery, raw frame preservation, bounded 30-second counter unwrapping and **provisional** angle/speed timing. Receipt time and acquisition time remain distinct.
- `slam_motion`: sampled SE(2) interpolation and midpoint deskew diagnostics with laser mounting offset, explicit gyro-only/full-SE(2) capability and acquisition-time validity gates.
- `slam_distance_field`: deep-copy immutable map snapshot with revision/content fingerprint, full-field exact centre-to-centre Euclidean distance transform, strict four-cell known-mask interpolation, finite gradients and source-version pinning.
- `slam_match_geometry`: geometry-only robust Huber scan-to-EDT reference matcher with synthetic pose recovery, overlap and LiDAR Hessian-rank diagnostics, and conservative rejection of unknown/truncated evidence.
- `slam_offline_graph` (optional, workstation only): GTSAM 4.3 Pose2 batch optimizer with fixed first pose, sequential verified odometry, robust and provenance-gated loop closures, rejected-loop diagnostics and deterministic synthetic regression. Its GTSAM backend was compiled and **2/2 tests passed locally**, including strict warnings/UBSan. **No FR079 or real-wheel graph-factor accuracy is claimed.**
- `slam_offline_generation` (optional with GTSAM): source-qualified frontend loop admission, graph optimization, two immutable occupancy/map-bundle rebuilds, **separate synthetic held-out map quality evaluation**, and fail-closed candidate selection that keeps the incumbent on rejection. GTSAM, map export and mapping do not run on the Pi known-map localizer.
- `slam_map_registry` (Linux): **durable FUSMAP2D map-generation selection** with atomic file/pointer publication, expected-incumbent checks, restart recovery, corruption fallback and explicit rollback. The optional offline graph bridge publishes only after the existing quality gate accepts the candidate; see [map-generation registry](docs/map-generation-registry.md).
- **`apps/demo/` offline map console:** a synthetic-example-capable read-only **C++20 native web UI** for authoritative `.slammap` exports and the `FUSGEN1` active/previous registry. It renders known/free/occupied/unknown cells, pans/zooms, checks data integrity, shows map provenance and supports cell inspection. **No live pose, robot commands, map promotion or fabricated sensor status.** See [Web UI guide](docs/web-ui.md).
- `slam_match_ceres` (optional): Ceres cost-function/solver adapter. Its byte-identical source compiled and was **exercised at runtime** against Ceres 2.2.0 using an isolated analytic distance-field harness; the *complete repository Ceres CTest remains unverified*.
- `slam_carmen`, `slam_carmen_geometry` and `slam_carmen_inspect`: strict FR079 `FLASER`/`RLASER`/`ODOM` parsing, 180° half-open angular layout, source scan IPC/logger times, conservative invalid-range classification, and native geometry-only matcher input without inventing per-beam times.
- **Catch2 3.16.1**: user-provided release extracted, compiled and used for CARMEN parsing/geometry tests. CMake accepts `SLAM_CATCH2_SOURCE_DIR`, otherwise uses pinned upstream commit `08092139210881f01a34e60c5f2fd6afa0ea4024`.
- **Ceres 2.2.0**: user-provided release compiled into a local static library with minimal logging; a standalone solver consumer passed. The byte-identical repository Ceres adapter compiled against Ceres 2.2 headers/library in an isolated build. **The full repository Ceres regression is not yet executed.**
- `slam_fr079_alignment` / `slam_fr079_alignment_inspect`: deterministic training-only frame-hypothesis search and held-out query diagnostics against the archived reference map, with explicit reflection detection, without asserting independent ground truth or an SE(2) pose.
- `slam_fr079_train_only`: a **separate** training-only 0.1 m occupancy/EDT map built from FR079 scans, frozen before Ceres or reference matching of withheld queries. A leakage test modifies held-out scans and checks the training map fingerprint is unchanged. Results are **geometric-fit proxies, not independent pose accuracy**.
- **FR079 public dataset**: both uploaded complete logs parsed in local CTest (3,118 scans and 5,761 odometry records each). `slam_carmen_map` parses CARMENMAPFILEv020 reference grids at 3, 5 and 10 cm; the optional `slam_carmen_map_inspect` checks the real 5 cm grid. Archived CARMEN binary maps now decode with preserved known/unknown cells; map-to-trajectory registration and conversion into authoritative map snapshots remain unverified. See [FR079 setup](datasets/fr079/README.md).

LD19 frame/CRC/rollover tests use **synthetically constructed packets**, not verified recordings from the physical device. The LD19 packet timestamp's exact physical beam-reference time remains **unverified**; its current beam timing model is provisional and deliberately not accepted as calibrated moving-scan map evidence. There is **no full end-to-end SLAM pipeline**, independently validated real-data graph factor source, Pi driver, recorder, ROS adapter or **live robot Web UI** implemented. The available web console is an offline map viewer only.

## Deterministic offline map export

- **slam_map_bundle** imports and exports a versioned FUSMAP2D v1 binary map with the full known/unknown mask, exact log-odds values, grid frame, revision/fingerprint, metadata and corruption checks.
- **slam_map_inspect** verifies and reports a saved bundle without requiring Ceres or GTSAM.
- **slam_fr079_map_export** (zlib required) builds a map from the training-only FR079 split and writes a verified .slammap bundle plus visualization-only .pgm and .yaml files. The original logged-laser map frame is explicitly **unverified**, not calibrated ground truth.
- See [map bundle format and usage](docs/map-bundle-v1.md) and [FR079 reproduction](datasets/fr079/README.md).

## Build and test

Install CMake 3.25+, C++20 compiler, Eigen3 (3.3+), Ninja (recommended). Network-free smoke tests:

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

For the pinned Catch2 v3 suite (may download Catch2 if not already cached):

~~~sh
cmake -S . -B build-catch -G Ninja -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON
cmake --build build-catch
ctest --test-dir build-catch --output-on-failure
~~~

If Eigen3 is not discoverable, supply `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3`. No public dataset download, hardware or GitHub Actions are required.

## Reusable offline dependency cache

A verified **11.6 MB Linux x86-64 Debian 13/GCC 14** dependency bundle
preserves installed Catch2 3.16.1, Ceres 2.2.0, GTSAM 4.3.0 and Eigen3 3.4.0
across chat/workspace resets. The repo tracks the pinned source checksums,
build scripts, restore verification, CMake smoke test and exact archive SHA-256,
but **not** the compiled archive. See [recovery and release instructions](docs/dependency-cache.md).
The [versioned dependency Release](https://github.com/civcode/2d-fusion-slam/releases/tag/deps-debian13-x86_64-gcc14-v1) is published for recovery in its compatible Debian 13 environment.

CMake prefers an installed Catch2 package; it does not fetch Catch2 if
the restored package is available.

## Interactive recorded LiDAR replay — available now

The [apps/demo replay viewer](docs/recorded-replay.md) now supports
**recorded FLASER sessions** with a timeline, play/pause/speed, frame-by-frame
logged trajectory, LiDAR endpoints, valid-hit counts, optional explicitly
unverified occupancy-map overlay, and recorded source provenance.

```sh
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
# Open http://127.0.0.1:8765/replay
```

The default Docker demo also supplies **a synthetic replay** when its map
folder is completely empty. The FR079 converter does **not** execute Ceres,
derive scan-matching quality, deskew scans, verify map registration or
produce ground truth. All recorded poses remain explicitly labeled
**logged estimates**.

## FR079 recorded LiDAR + odometry: end-to-end incremental SLAM

A new **slam_fr079_offline** native C++20 executable processes all
FR079 CARMEN FLASER scans in timestamp order, predicts poses using
embedded scan-time odometry, corrects selected scans using a bounded
Ceres scan-to-historical-map matcher, and inserts every valid scan
into an initially empty FUSMAP2D occupancy grid. The logged laser-pose
estimates are **not** used to generate its trajectory or map. It exports
a validated binary .slammap, a browser-readable estimated-trajectory
.slamreplay.json, per-frame matcher evidence, and p50/p95/p99 native
processing times with offline processing FPS.

The result appears in the **standard** Map Explorer and LiDAR Replay
when written under apps/demo/maps; it is NOT a synthetic Fusion Replay
run. FR079 supplies no verified IMU stream or per-beam scan timings,
and the first experiment assumes identity laser extrinsics.
**There is no independently verified map-frame registration, loop
closure, pose-graph result or FR079 ground-truth IoU/ATE.**

~~~sh
export BUILD_JOBS=28 SLAM_DOCKER_JOBS=28
docker compose build test demo
export FR079_LOG=/absolute/path/to/fr079.log.gz
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-slam
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker -R fr079_offline_slam --output-on-failure
docker compose up -d demo
~~~

See the full [FR079 offline SLAM protocol](docs/fr079-offline-slam.md)
for artifact formats, provenance and limitations.

## Apartment/warehouse maps and native SLAM performance

The main **Map Explorer** at http://127.0.0.1:8765/ now discovers
all completed fusion-run maps under the configured fusion directory:
apartments, warehouses, and each existing wheel/LiDAR/scan-matched/
graph-optimized method. It opens those maps directly with the existing
cell inspector, **without copying** FUSMAP2D files to apps/demo/maps.
When a fusion run exists it prefers a scan-matched result over the
synthetic workshop fallback. The Fusion Replay page at /fusion also
links directly to the selected final map in the main Map Explorer.

Native profiling (SLAM_PROFILE_FUSION=ON by default) now records
steady-clock **per-stage wall times, p50/p95, offline effective fusion
FPS and the 300 ms/scan budget comparison** into fusion_profile.json.
The Fusion Replay details panel shows these measured results and the
largest stages; scripts/benchmark-fusion.py repeats apartment and
warehouse runs and produces CSV output for comparing real workstation
or Docker builds:

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

The reported FPS is a **synthetic offline throughput estimate**, not
proof of real-time behavior for LD19/MPU6050 hardware. Compare the
same experiments with instrumentation disabled in a separate build,
then evaluate actual sensor deadlines and backpressure on target
hardware. See [profiling methodology](docs/fusion-profiling.md).

## Fusion Replay — watch completed SLAM runs in the browser

The native C++20 dashboard now has a dedicated **[Fusion Replay](docs/fusion-replay.md)**
page at **http://127.0.0.1:8765/fusion**. It replays the real *recorded
estimator events* of a completed synthetic offline run: per-frame sampled
deskewed LiDAR returns, held-out Ceres/reference matcher decisions and
rank/overlap diagnostics, map insertion events, optional independent
synthetic loop closures, and interactive wheel-only/LiDAR-only/gyro-wheel/
scan-matched/graph trajectories. It also shows completed method metrics
and optional **final** FUSMAP2D occupancy overlays.

~~~sh
# Build the native offline pipeline and the standalone C++ Web UI.
cmake --build build --target slam_offline_fusion fusion_slam_demo
./build/slam_offline_fusion apartment apps/demo/fusion/apartment-001
./build/fusion_slam_demo --data-dir apps/demo/maps \
  --fusion-dir apps/demo/fusion --example-if-empty
# Open http://127.0.0.1:8765/fusion
~~~

Docker Compose mounts apps/demo/fusion read-only and serves the same page.
A FUSFUSREPLAY1 trace is written alongside the existing fusion report,
timed trajectories and maps. **Accumulated dots are sampled measured
endpoints, not a replay of intermediate occupancy probabilities**; maps
and optimized graph paths are labeled final post-run artifacts. The native
HTTP layer never runs the estimator or accepts robot commands.

New CTest **fusion_replay_http** runs the real native pipeline and checks
run discovery, replay/timeline parity, map fingerprints, corruption and
symlink refusal. Full workstation verification remains required;
see [Fusion Replay protocol](docs/fusion-replay.md).

## Moving-scan offline gyro/wheel SLAM experiment

The new **C++20 `slam_offline_fusion`** executable connects verified
timestamped synthetic gyroscope and wheel observations to full-SE(2)
trajectory interpolation, moving LiDAR beam deskew, frozen-map Ceres scan
matching (a geometry-only fallback is reported if Ceres is not built),
and optional independently sourced GTSAM graph optimization. All mapping
uses estimated poses; the exact simulator reference is reserved for
sensor generation and after-the-fact accuracy/ATE scoring.

Compared methods: **wheel-only**, **LiDAR-only without deskew**,
**gyro + wheel**, **gyro + wheel + source-excluded scan matching**, and,
when GTSAM is installed, **graph-optimized**. Each produces its own
authoritative FUSMAP2D snapshot; the run writes an auditable
`fusion_report.json` and time-indexed `trajectories.json`.
Loop factors are refused by default: LiDAR scan-to-map pose corrections
are **not** independent relative measurements. An opt-in *separately
simulated relative-pose sensor* can exercise loop gating in synthetic
tests, but no such hardware sensor is claimed.

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON
cmake --build build --target slam_offline_fusion \
  slam_gyro_wheel_test slam_map_accuracy_cli --parallel 2
ctest --test-dir build -R 'gyro_wheel_fusion|offline_fusion' \
  --output-on-failure

./build/slam_offline_fusion apartment /tmp/slam-fusion-apartment-a
./build/slam_offline_fusion warehouse /tmp/slam-fusion-warehouse-a
~~~

The exact expected Ceres/GTSAM results and numerical map accuracy must be
measured by running the native executable; **the complete repository native
build remains unverified in this environment**. No physical LD19/MPU6050
clock mapping, stationary calibration, real-wheel source or real loop
sensor is asserted. See [offline sensor fusion protocol](docs/offline-fusion.md).

## Estimated-pose synthetic mapping benchmark

\`slam_synthetic_mapping_benchmark\` now generates apartment and warehouse
laser scans along deterministic synthetic routes, estimates all poses **after
the first anchor** using biased relative odometry, and improves eligible
held-out scans with the actual immutable-EDT geometry matcher. It rebuilds
and exports **two** FUSMAP2D maps (odometry baseline versus matched candidate)
and evaluates both against exact synthetic wall truth. Reports include
occupancy IoU/coverage/wall distances and independently scored trajectory
RMSE/p95; **regressions are reported, not hidden**.

The scan matcher uses an immutable **even-scan training map** and queries
only **odd scans**, without using per-frame truth poses for mapping. The
reference geometry is used only for synthetic ray generation and the
after-the-fact accuracy evaluation. This is a reproducible local
scan-to-map front-end experiment, **not** full sensor-fused GTSAM SLAM or
measured real-world accuracy.

\`\`\`sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_ENABLE_GTSAM=OFF
cmake --build build --target slam_synthetic_mapping_benchmark \
  slam_map_accuracy_cli --parallel 2
./build/slam_synthetic_mapping_benchmark apartment /tmp/slam-apartment-run1
./build/slam_synthetic_mapping_benchmark warehouse /tmp/slam-warehouse-run1
\`\`\`

See [the synthetic mapping benchmark protocol](docs/synthetic-mapping-benchmark.md)
for the held-out split, FUSSYNTHBENCH1 JSON and the new CTests.

## Ground-truth synthetic map accuracy

`slam_map_accuracy_cli` compares an exported FUSMAP2D binary map to the
**exact synthetic apartment/warehouse wall layout** and emits a deterministic
`FUSMAPACC1` JSON report: occupied IoU, precision, recall, known/unknown
coverage, and symmetric mean/p95 wall distance in metres. It refuses cropped,
mismatched or unverified frames; FR079 is **not** accepted as independent
ground truth. The separate oracle-pose fixture exercises the full export/import
path but is **not** a SLAM-accuracy benchmark. See
[map accuracy instructions](docs/map-accuracy.md).

## Local web UI — available now

The offline map console uses a **native C++20 executable**, not Ceres, GTSAM,
ROS or a compiled web framework. After exporting a FUSMAP2D map:

~~~sh
mkdir -p /tmp/slam-maps
./build/slam_fr079_map_export /path/to/fr079.log.gz /tmp/slam-maps/fr079-training
./build-demo/fusion_slam_demo --data-dir /tmp/slam-maps --port 8765
# For an empty directory only, demonstrate the map UI with fake geometry:
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
~~~

Open **http://127.0.0.1:8765/**. You can also point `--data-dir`
at an existing persistent generation directory containing `CURRENT` and
`generation-*.slammap`. The console is **read-only**, checks binary
integrity and does not mutate, recover or roll back map generations.
For a Docker-only launch with no Python on the host and for security
notes, see [Web UI guide](docs/web-ui.md).


### Manual native viewer verification

From a full source checkout, use `bash scripts/verify-native-demo.sh`
to build the standalone C++ viewer and execute `native_viewer_json`,
`native_web_api`, `demo_fixture_native` and the existing
`web_ui_unit` compatibility tests. It never dispatches GitHub Actions
or downloads dependencies. The original Python server is retained
for rollback while native Docker and HTTP tests await execution.

### Native C++ demo build

The existing HTML/CSS/JavaScript is unchanged. A separate POSIX C++20
server now provides read-only maps, replay, PNG and optional validated
Ceres sidecars, without duplicating the authoritative C++ map bundle
reader or requiring Python at runtime. Python remains only for offline
dataset conversion, test drivers and the preserved fallback viewer.

~~~sh
cmake -S . -B build-demo -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_GTSAM=OFF -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_BUILD_NATIVE_DEMO=ON -DSLAM_REQUIRE_NATIVE_DEMO=ON
cmake --build build-demo --target fusion_slam_demo --parallel 2
ctest --test-dir build-demo -R '^native_web_api

Following the app and Docker layout in
[hybrid-astar-cpp](https://github.com/civcode/hybrid-astar-cpp), the
read-only Web UI now lives in [apps/demo](apps/demo/README.md), and
the [root Dockerfile](Dockerfile) provides separate **demo** and
**native test** targets. The demo runs native C++ without Python; the separate
native test target uses Ubuntu APT Ceres/GTSAM/Catch2 dependencies.

```sh
# Out of the box: opens a clearly labeled synthetic workshop map.
docker compose up --build demo           # http://127.0.0.1:8765
# Add *.slammap files to apps/demo/maps/ for real offline map inspection.
docker compose run --build --rm test    # C++ + Python CTest
```

Compose publishes port 8765 on host loopback only and mounts the map
directory read-only. No robot control, authentication or live tracking
is provided, so do not expose this service publicly.

## Actual Ceres diagnostics in LiDAR replay

The replay can now overlay **real offline Ceres scan-to-EDT matching results**
against a frozen, fingerprint-pinned map. Build the native
`slam_ceres_replay_export` target, then use
`python3 apps/demo/match_ceres.py` to produce a separately validated
`FUSMATCH1` sidecar. The UI shows accepted/rejected frames, EDT overlap,
mean distances and robust-cost changes. This is **not ground truth or
independently measured localization accuracy**; the browser never runs Ceres.
See [Ceres replay instructions](docs/ceres-replay.md).

## Docker / APT build and tests

For a self-contained **Ubuntu 24.04** build, install Docker on the host
and run `bash scripts/docker-build-test.sh test` from the repository.
The image uses Ubuntu APT packages (Ceres 2.2, GTSAM 4.2 and Catch2 3.4),
requires all solvers and detailed tests, and keeps build outputs under
`.build/docker-ubuntu24.04-apt/`. It does **not** need the Debian 13
binary dependency Release or host-installed development libraries.
See [Docker development and version compatibility](docs/docker-development.md). The legacy `docker/Dockerfile.apt` was replaced by the root multi-stage Dockerfile; the development wrapper now builds its `dependencies` target.

## Manual GitHub build and tests

The [Manual C++ build and tests](.github/workflows/manual-build-test.yml) workflow runs **only when manually started** from **Actions → Manual C++ build and tests → Run workflow**. It does not run on pushes, pull requests or a schedule, and it was not dispatched when added.

On Ubuntu 24.04 it installs Eigen3, Ceres 2.2.0 and zlib, uses the pinned Catch2 3.16.1 source, builds with CMake/Ninja and runs CTest, including the Ceres matcher test. CMake requires Ceres so the run cannot silently skip that backend. FR079 full-recording tests require local dataset files outside the repository and are **not included in this workflow**; synthetic and source-level importer tests are included.

## Documentation

- [Architecture](docs/architecture-specification.md)
- [Technical specification](docs/technical-specification.md)
- [Implementation contract](docs/implementation-contract.md)
- [Benchmarks and datasets](docs/benchmarks-and-datasets.md)
- [Design choices OD-001–OD-025](docs/open-design-choices.md)
- [Implementation status and local test evidence](docs/implementation-status.md)
- [FR079 dataset setup, hashes and limitations](datasets/fr079/README.md)
- [FR079 map-frame diagnostic evidence](datasets/fr079/alignment-evidence.json)
- [FR079 training-only benchmark evidence](datasets/fr079/train-only-evidence.json)
- [Air-gapped Catch2 and Ceres setup](docs/offline-source-dependencies.md)
- [Offline dependency bundle, restore and Release setup](docs/dependency-cache.md)
- [Docker build/test using Ubuntu APT packages](docs/docker-development.md)
- [Offline GTSAM 4.3 pose-graph integration](docs/offline-pose-graph.md)
- [Offline map candidate rebuilding, comparison and rollback](docs/offline-map-generation.md)
- [Persistent map-generation registry and recovery](docs/map-generation-registry.md)
- [Offline map Web UI: launch, Docker and API](docs/web-ui.md)
- [Native Ceres replay diagnostics and limitations](docs/ceres-replay.md)
- [Training-excluded FR079 Ceres evaluation and report](docs/heldout-ceres-evaluation.md)
- [Interactive recorded LiDAR replay and FUSREPLAY1](docs/recorded-replay.md)
- [P2 read-only known-map tracking trust gate (synthetic foundation)](docs/known-map-tracker.md)
- [Ground-truth map accuracy: synthetic IoU, coverage and wall distance](docs/map-accuracy.md)
- [Estimated-pose synthetic mapping benchmark and reproducible truth comparisons](docs/synthetic-mapping-benchmark.md)
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** source-rebuilt train-only map evaluation now checks that every Ceres query scan was excluded from the FR079 training map. See [held-out Ceres evaluation](docs/heldout-ceres-evaluation.md). This still does not provide independent physical ground truth, verified extrinsics or per-beam timestamps. A calibrated independent traversal is the next real-data gate.

**Previously planned:** verify the new durable map registry in the complete Docker/Ubuntu build and add independent, calibrated sensor packet lineage before attempting real-data map promotion. Then add measured fault-injection/power-loss tests and retention policy. The current graph/map quality gate is still synthetic-only; real-world accuracy and FR079 map-frame truth remain unresolved.

 --output-on-failure
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
~~~

**Verification:** The native C++ source and HTTP compatibility tests are
committed; full-checkout CMake/CTest and Docker runtime execution remain
unverified in the current environment. The original Python server is
retained as fallback until native tests pass; Docker's demo runtime is
configured for C++. See [web UI guide](docs/web-ui.md).

## Browser demo and native Docker tests

Following the app and Docker layout in
[hybrid-astar-cpp](https://github.com/civcode/hybrid-astar-cpp), the
read-only Web UI now lives in [apps/demo](apps/demo/README.md), and
the [root Dockerfile](Dockerfile) provides separate **demo** and
**native test** targets. The demo runs native C++ without Python; the separate
native test target uses Ubuntu APT Ceres/GTSAM/Catch2 dependencies.

```sh
# Out of the box: opens a clearly labeled synthetic workshop map.
docker compose up --build demo           # http://127.0.0.1:8765
# Add *.slammap files to apps/demo/maps/ for real offline map inspection.
docker compose run --build --rm test    # C++ + Python CTest
```

Compose publishes port 8765 on host loopback only and mounts the map
directory read-only. No robot control, authentication or live tracking
is provided, so do not expose this service publicly.

## Actual Ceres diagnostics in LiDAR replay

The replay can now overlay **real offline Ceres scan-to-EDT matching results**
against a frozen, fingerprint-pinned map. Build the native
`slam_ceres_replay_export` target, then use
`python3 apps/demo/match_ceres.py` to produce a separately validated
`FUSMATCH1` sidecar. The UI shows accepted/rejected frames, EDT overlap,
mean distances and robust-cost changes. This is **not ground truth or
independently measured localization accuracy**; the browser never runs Ceres.
See [Ceres replay instructions](docs/ceres-replay.md).

## Docker / APT build and tests

For a self-contained **Ubuntu 24.04** build, install Docker on the host
and run `bash scripts/docker-build-test.sh test` from the repository.
The image uses Ubuntu APT packages (Ceres 2.2, GTSAM 4.2 and Catch2 3.4),
requires all solvers and detailed tests, and keeps build outputs under
`.build/docker-ubuntu24.04-apt/`. It does **not** need the Debian 13
binary dependency Release or host-installed development libraries.
See [Docker development and version compatibility](docs/docker-development.md). The legacy `docker/Dockerfile.apt` was replaced by the root multi-stage Dockerfile; the development wrapper now builds its `dependencies` target.

## Manual GitHub build and tests

The [Manual C++ build and tests](.github/workflows/manual-build-test.yml) workflow runs **only when manually started** from **Actions → Manual C++ build and tests → Run workflow**. It does not run on pushes, pull requests or a schedule, and it was not dispatched when added.

On Ubuntu 24.04 it installs Eigen3, Ceres 2.2.0 and zlib, uses the pinned Catch2 3.16.1 source, builds with CMake/Ninja and runs CTest, including the Ceres matcher test. CMake requires Ceres so the run cannot silently skip that backend. FR079 full-recording tests require local dataset files outside the repository and are **not included in this workflow**; synthetic and source-level importer tests are included.

## Documentation

- [Architecture](docs/architecture-specification.md)
- [Technical specification](docs/technical-specification.md)
- [Implementation contract](docs/implementation-contract.md)
- [Benchmarks and datasets](docs/benchmarks-and-datasets.md)
- [Design choices OD-001–OD-025](docs/open-design-choices.md)
- [Implementation status and local test evidence](docs/implementation-status.md)
- [FR079 dataset setup, hashes and limitations](datasets/fr079/README.md)
- [FR079 map-frame diagnostic evidence](datasets/fr079/alignment-evidence.json)
- [FR079 training-only benchmark evidence](datasets/fr079/train-only-evidence.json)
- [Air-gapped Catch2 and Ceres setup](docs/offline-source-dependencies.md)
- [Offline dependency bundle, restore and Release setup](docs/dependency-cache.md)
- [Docker build/test using Ubuntu APT packages](docs/docker-development.md)
- [Offline GTSAM 4.3 pose-graph integration](docs/offline-pose-graph.md)
- [Offline map candidate rebuilding, comparison and rollback](docs/offline-map-generation.md)
- [Persistent map-generation registry and recovery](docs/map-generation-registry.md)
- [Offline map Web UI: launch, Docker and API](docs/web-ui.md)
- [Native Ceres replay diagnostics and limitations](docs/ceres-replay.md)
- [Training-excluded FR079 Ceres evaluation and report](docs/heldout-ceres-evaluation.md)
- [Interactive recorded LiDAR replay and FUSREPLAY1](docs/recorded-replay.md)
- [P2 read-only known-map tracking trust gate (synthetic foundation)](docs/known-map-tracker.md)
- [Ground-truth map accuracy: synthetic IoU, coverage and wall distance](docs/map-accuracy.md)
- [Estimated-pose synthetic mapping benchmark and reproducible truth comparisons](docs/synthetic-mapping-benchmark.md)
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** source-rebuilt train-only map evaluation now checks that every Ceres query scan was excluded from the FR079 training map. See [held-out Ceres evaluation](docs/heldout-ceres-evaluation.md). This still does not provide independent physical ground truth, verified extrinsics or per-beam timestamps. A calibrated independent traversal is the next real-data gate.

**Previously planned:** verify the new durable map registry in the complete Docker/Ubuntu build and add independent, calibrated sensor packet lineage before attempting real-data map promotion. Then add measured fault-injection/power-loss tests and retention policy. The current graph/map quality gate is still synthetic-only; real-world accuracy and FR079 map-frame truth remain unresolved.

