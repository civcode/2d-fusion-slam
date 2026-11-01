# 2D Fusion SLAM

**Status: synthetic matching baseline plus initial FR079 public-data import.** The C++20 core now parses Freiburg FR079 CARMEN logs, converts scans to matcher-native laser points and supports offline Catch2 3.16.1 and Ceres 2.2.0 dependencies. **This is not yet a complete SLAM system or a validated real-dataset SLAM benchmark.** See [implementation status](docs/implementation-status.md).

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
- `slam_match_ceres` (optional): Ceres cost-function/solver adapter using the same geometry and fail-closed support policy; **not compiled or runtime-verified** in the current environment because Ceres is not installed.
- `slam_carmen`, `slam_carmen_geometry` and `slam_carmen_inspect`: strict FR079 `FLASER`/`RLASER`/`ODOM` parsing, 180° half-open angular layout, source scan IPC/logger times, conservative invalid-range classification, and native geometry-only matcher input without inventing per-beam times.
- **Catch2 3.16.1**: user-provided release extracted, compiled and used for CARMEN parsing/geometry tests. CMake accepts `SLAM_CATCH2_SOURCE_DIR`, otherwise uses pinned upstream commit `08092139210881f01a34e60c5f2fd6afa0ea4024`.
- **Ceres 2.2.0**: user-provided release compiled into a local static library with minimal logging; a standalone solver consumer passed. The byte-identical repository Ceres adapter compiled against Ceres 2.2 headers/library in an isolated build. **The full repository Ceres regression is not yet executed.**
- **FR079 public dataset**: both uploaded complete logs parsed in local CTest (3,118 scans and 5,761 odometry records each). `slam_carmen_map` parses CARMENMAPFILEv020 reference grids at 3, 5 and 10 cm; the optional `slam_carmen_map_inspect` checks the real 5 cm grid. Archived CARMEN binary maps now decode with preserved known/unknown cells; map-to-trajectory registration and conversion into authoritative map snapshots remain unverified. See [FR079 setup](datasets/fr079/README.md).

LD19 frame/CRC/rollover tests use **synthetically constructed packets**, not verified recordings from the physical device. The LD19 packet timestamp's exact physical beam-reference time remains **unverified**; its current beam timing model is provisional and deliberately not accepted as calibrated moving-scan map evidence. There is **no fully verified Ceres-based SLAM pipeline**, GTSAM optimizer, Pi driver, recorder, ROS adapter or Web UI implemented.

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

## Documentation

- [Architecture](docs/architecture-specification.md)
- [Technical specification](docs/technical-specification.md)
- [Implementation contract](docs/implementation-contract.md)
- [Benchmarks and datasets](docs/benchmarks-and-datasets.md)
- [Design choices OD-001–OD-025](docs/open-design-choices.md)
- [Implementation status and local test evidence](docs/implementation-status.md)
- [FR079 dataset setup, hashes and limitations](datasets/fr079/README.md)
- [Air-gapped Catch2 and Ceres setup](docs/offline-source-dependencies.md)
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** run the full in-repository Ceres matcher regression against the assembled offline dependencies, validate reference-map axis/frame registration, then create a reproducible FR079 scan-to-map benchmark with a held-out evaluation split. Logged CARMEN pose fields are estimates, not ground truth. Follow with map export and offline GTSAM. Hardware acquisition progresses independently.
