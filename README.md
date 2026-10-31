# 2D Fusion SLAM

**Status: C0 bootstrap underway.** The portable C++20 time/SE(2) core and deterministic synthetic apartment/warehouse scan fixtures are implemented; **this is not yet a working SLAM system**. See [implementation status](docs/implementation-status.md).

The planned system uses an **LD19 2D LiDAR and MPU6050 gyroscope**, with optional **actually measured** wheel odometry. Three target modes share a portable core:

| Mode | Platform | Function |
| --- | --- | --- |
| Offline SLAM | Powerful workstation | Ceres scan matching, occupancy mapping and GTSAM batch optimization |
| Known-map localization | Raspberry Pi 5 | Read-only verified map and bounded pose tracking, no initial GTSAM |
| Online SLAM | Raspberry Pi 5 (later) | Resource-limited mapping, designed from benchmark evidence |

**Development strategy:** Run initial implementation/tests/benchmarks on synthetic and verified opt-in public data. Develop real LD19/MPU6050 Pi acquisition/recording **in parallel**, without blocking synthetic/public tests. Optimizations and online graph complexity are selected using measurements, not assumptions.

## What works today

- \`slam_core\`: checked signed 64-bit nanosecond arithmetic with explicit clock domains/epochs, SE(2) compose/inverse/transform/exp/log, constant-body-twist integration.
- \`slam_fixtures\`: deterministic apartment (15 × 12 m, doorway gaps) and warehouse (30 × 20 m, repeated shelves) with exact ground-truth moving-ray acquisition times and hit/no-return classification.
- Local CMake/Ninja build and two network-free CTest smoke tests. Additional Catch2 v3 tests are available by pinned FetchContent but **not yet verified in this offline environment**.

The synthetic fixtures are initial geometry test inputs, not an LD19 packet simulator or an accuracy benchmark yet. There is **no Ceres/GTSAM pipeline, Pi driver, recorder, ROS adapter or Web UI** implemented.

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

If Eigen3 is not discoverable, supply \`-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3\`. No public dataset download, hardware or GitHub Actions are required.

## Documentation

- [Architecture](docs/architecture-specification.md)
- [Technical specification](docs/technical-specification.md)
- [Implementation contract](docs/implementation-contract.md)
- [Benchmarks and datasets](docs/benchmarks-and-datasets.md)
- [Design choices OD-001–OD-025](docs/open-design-choices.md)
- [Implementation status and local test evidence](docs/implementation-status.md)
- [Specification migration notes](docs/documentation-migration-notes.md)

**Next:** sensor/scan data contracts, time-aware scan assembly and occupancy mapping; then Ceres matching, offline GTSAM and known-map localization. A minimal Web Lab follows once mapped state and replay are available.
