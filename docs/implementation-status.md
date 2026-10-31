# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Branch:** `main`  
**Scope:** Synthetic scan-to-map baseline with portable solver interface; **not complete offline SLAM**.

## Implemented and locally exercised

| Target | Working functionality | Evidence |
| --- | --- | --- |
| `slam_core` | Checked nanosecond/epoch time and SE(2) geometry | `core_smoke` |
| `slam_fixtures` | Deterministic exact-time apartment/warehouse synthetic ray casting | `fixtures_smoke` |
| `slam_sensors` and `slam_ld19` | Beam timing/quality, partial scan assembly, 47-byte LD19/CRC/frame/rollover parsing | `scan_mapping`, `ld19_motion` |
| `slam_motion` | Sampled SE(2) trajectory interpolation and deskew diagnostic safety | `ld19_motion` |
| `slam_mapping` | Known-mask log-odds grid and safe ray updates | `scan_mapping` |
| **`slam_distance_field`** | Immutable deep-copy occupancy snapshot with pinned revision and non-cryptographic fingerprint, exact 2-pass EDT, map-frame bilinear gradients, known-mask and nontruncated residual support | **`distance_field`** |
| **`slam_match_geometry`** | LiDAR geometry-only Huber robust EDT scoring and a small damped reference optimizer; synthetic pose recovery with overlap, rank/eigenvalues and source-coherence diagnostics | **`match_geometry`** |
| `slam_match_ceres` (optional) | A private Ceres single-scalar-per-beam cost-function adapter, bounded local solve, fail-closed unsupported-region evaluation and identical output quality gate | **Source committed, NOT compiled/tested** without Ceres |

**Test evidence:** GCC 14, CMake 3.31.6, Ninja, provisioned Eigen3 on Linux x86-64. **6/6 CTest tests passed** with network-free `SLAM_ENABLE_CATCH2=OFF`, including the normal build and a strict build using `-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-sanitize-recover=all`. The EDT tests compare all cells to exact Euclidean reference distances, test rotated grids, immutable source revisions, unknown stencils and truncation. The matching tests check geometry-only convergence toward synthetic truth, overlap, rank, cost reduction and unsupported-region rejection. These are **unit/synthetic tests**, not a public-dataset accuracy benchmark.

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Use `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3` if the Eigen package is not automatically found. If Ceres is installed, `SLAM_ENABLE_CERES=ON` discovers it, builds `slam_match_ceres`, and registers a seventh `match_ceres` synthetic test. **That conditional build/test was not run** in this environment; passing 6/6 does not validate Ceres. Pinned FetchContent Catch2 v3 was also not fetched here because direct GitHub DNS is unavailable. No GitHub Actions were created or executed.

## Guarantees and limitations

- **Snapshots are immutable after publication**, but capture is only coherent if the caller serializes copying with map insertion. There is no multi-threaded map owner/rebuild scheduler yet.
- **EDT is exact at cell centres**, with bilinear interpolation using four observed cells and entirely nontruncated stencil values. Unknown/out-of-map/saturated support is **not** a false zero residual. The derived cache is not map authority.
- **Matcher is a local, geometry-only baseline** seeded near the map pose. Its synthetic test uses independent, known synthetic landmark geometry but is not a full SLAM trajectory/loop benchmark. No relocalization/global search or verified sensor bias/clock estimator is implemented.
- The information matrix and its eigenvalues are **observability diagnostics**, not a calibrated independent covariance. A returned tracking pose is not an automatically graph-eligible factor. A local matcher can still have map/deskew correlations.
- **Ceres backend awaits a toolchain test.** An unsupported trial evaluation fails rather than yielding an artificial zero-distance residual. Its actual convergence, false-lock and boundary behavior must be profiled on synthetic and validated public recordings before deployment.
- LD19 packet time-reference, MPU6050 gyro bias, real acquisition/MCAP, actual Pi performance, public dataset import and full online/offline SLAM remain unverified.

## Next checkpoint

First verify/benchmark the optional Ceres target and its Jacobians in a real Ceres-enabled toolchain. Then add **deterministic full-map replay/map export and a simple offline pose-only GTSAM reference graph**, with conservative factor provenance and false-loop checks. Independently progress real Pi LD19/MPU6050 acquisition and data calibration; initial algorithm experiments continue using synthetic and verified public data.
