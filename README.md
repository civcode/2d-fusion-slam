# 2D Fusion SLAM

**Status: C0 foundations and initial C1 sensor/motion primitives implemented.** The portable core now includes synthetic scenes, scan assembly, an occupancy grid, an LD19 packet decoder and a minimal trajectory/deskew library. **This is not yet a working SLAM system.** See [implementation status](docs/implementation-status.md).

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
- Local CMake/Ninja build and **four** network-free CTest tests pass under GCC 14, strict warnings, UBSan and ASan+UBSan. Pinned Catch2 v3 remains **unverified in this offline environment**.

LD19 frame/CRC/rollover tests use **synthetically constructed packets**, not verified recordings from the physical device. The LD19 packet timestamp's exact physical beam-reference time remains **unverified**; its current beam timing model is provisional and deliberately not accepted as calibrated moving-scan map evidence. There is **no Ceres/GTSAM pipeline, Pi driver, recorder, ROS adapter or Web UI** implemented.

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
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** connect the physical sensor acquisition and time-calibration path in parallel, and implement immutable map snapshots/EDT and synthetic Ceres scan matching. A tested gyro predictor and verified translational motion source remain future work, followed by workstation GTSAM and known-map localization. A minimal Web Lab follows once mapped state and replay are available.
