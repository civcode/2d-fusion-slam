# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Repository branch:** `main`  
**Scope:** C0 foundation; this is not a completed SLAM application.

## Implemented

| Target | Implemented behavior | Current verification |
| --- | --- | --- |
| `slam_core` | C++20 native `Timestamp` (nanoseconds/domain/epoch), checked arithmetic; `Pose2` and SE(2) transform/exp/log/integration | GCC 14 native build, `core_smoke` passed via CTest |
| `slam_fixtures` | Noiseless, deterministic apartment/warehouse line-segment ray cast; moving sensor pose at exact acquisition time; hit/no-return | `fixtures_smoke` passed via CTest |
| CMake project | `slam_core`, `slam_fixtures`, network-free CTest and an opt-in verified-commit Catch2 v3 FetchContent declaration | CMake 3.31.6/Ninja native configuration and build worked with a local Eigen header location |

**Local command used for the verified smoke suite:**

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF -DSLAM_EIGEN_INCLUDE_DIR=<local-Eigen3-include-root>
cmake --build build
ctest --test-dir build --output-on-failure
~~~

**Result:** 2/2 smoke tests passed both in the normal build and in a second local build using `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined -fno-sanitize-recover=undefined` (GCC 14). `SLAM_ENABLE_CATCH2=ON` is the normal detailed-test path, pinned to Catch2 v3.8.1 upstream commit `2b60af89e23d28eefc081bc930831ee9d45ea58b`. **Its fetch/build/test was not verified** because local GitHub DNS access was unavailable. No attempt was made to run GitHub Actions.

## Not implemented / not verified

- Public dataset manifests/import/conversion, complete B01–B18 benchmark suite or quantitative mapping/localization accuracy.
- LD19 native 47-byte UART packet parsing, MPU6050 I2C acquisition, clock drift modeling, MCAP raw recording or real-device timing calibration.
- Source capability model/scan assembly, gyro-only or wheel-fusion estimator, deskew validation, occupancy grid/submaps, EDT/Ceres scan matching or GTSAM graph.
- Portable map bundle, Pi known-map localization, online SLAM, semantic seek, Web Lab and optional ROS/Pico integration.
- Raspberry Pi 5, ARM64, Docker image, real-hardware or thermal/performance validation.

## Next checkpoints

1. Implement sensor/beam validity classes and scan assembly; test missing/invalid/partial data and trustworthy ray rules (**B02/B03**).
2. Implement a simple 2D occupancy known-mask/log-odds grid with checked ray insertion; synthetic exact fixture tests (**B06**).
3. Establish geometry-only Ceres scan-match baseline against immutable map generations; compare predictions on held-out synthetic and later validated public data (**B07/B08**).
4. Build a workstation one-pass graph/map bundle, then benchmark known-map localization on identical prepared data before any Pi-specific optimizations.

A minimal hardware acquisition path may develop **in parallel**, but it does not gate these initial synthetic/public tests.
