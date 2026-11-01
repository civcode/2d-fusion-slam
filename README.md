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
- `slam_match_ceres` (optional): Ceres cost-function/solver adapter. Its byte-identical source compiled and was **exercised at runtime** against Ceres 2.2.0 using an isolated analytic distance-field harness; the *complete repository Ceres CTest remains unverified*.
- `slam_carmen`, `slam_carmen_geometry` and `slam_carmen_inspect`: strict FR079 `FLASER`/`RLASER`/`ODOM` parsing, 180° half-open angular layout, source scan IPC/logger times, conservative invalid-range classification, and native geometry-only matcher input without inventing per-beam times.
- **Catch2 3.16.1**: user-provided release extracted, compiled and used for CARMEN parsing/geometry tests. CMake accepts `SLAM_CATCH2_SOURCE_DIR`, otherwise uses pinned upstream commit `08092139210881f01a34e60c5f2fd6afa0ea4024`.
- **Ceres 2.2.0**: user-provided release compiled into a local static library with minimal logging; a standalone solver consumer passed. The byte-identical repository Ceres adapter compiled against Ceres 2.2 headers/library in an isolated build. **The full repository Ceres regression is not yet executed.**
- `slam_fr079_alignment` / `slam_fr079_alignment_inspect`: deterministic training-only frame-hypothesis search and held-out query diagnostics against the archived reference map, with explicit reflection detection, without asserting independent ground truth or an SE(2) pose.
- `slam_fr079_train_only`: a **separate** training-only 0.1 m occupancy/EDT map built from FR079 scans, frozen before Ceres or reference matching of withheld queries. A leakage test modifies held-out scans and checks the training map fingerprint is unchanged. Results are **geometric-fit proxies, not independent pose accuracy**.
- **FR079 public dataset**: both uploaded complete logs parsed in local CTest (3,118 scans and 5,761 odometry records each). `slam_carmen_map` parses CARMENMAPFILEv020 reference grids at 3, 5 and 10 cm; the optional `slam_carmen_map_inspect` checks the real 5 cm grid. Archived CARMEN binary maps now decode with preserved known/unknown cells; map-to-trajectory registration and conversion into authoritative map snapshots remain unverified. See [FR079 setup](datasets/fr079/README.md).

LD19 frame/CRC/rollover tests use **synthetically constructed packets**, not verified recordings from the physical device. The LD19 packet timestamp's exact physical beam-reference time remains **unverified**; its current beam timing model is provisional and deliberately not accepted as calibrated moving-scan map evidence. There is **no full end-to-end SLAM pipeline**, independently validated real-data graph factor source, Pi driver, recorder, ROS adapter or Web UI implemented.

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
**The Release asset is prepared but not yet published** through GitHub.

CMake prefers an installed Catch2 package; it does not fetch Catch2 if
the restored package is available.

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
- [Offline GTSAM 4.3 pose-graph integration](docs/offline-pose-graph.md)
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** connect verified frontend measurements to the new offline batch graph without double-counting sources, benchmark false-loop rejection, and regenerate a *new* map generation after graph optimization only when held-out quality supports it. The reference pose graph currently uses synthetic factors, not unverified FR079 odometry. GTSAM remains excluded from the Raspberry Pi known-map localizer.
