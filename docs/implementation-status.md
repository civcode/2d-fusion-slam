# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Branch:** `main`  
**Scope:** Synthetic scan matching plus FR079 public CARMEN data ingest, Catch2 source integration and a locally built Ceres dependency; **not complete offline SLAM or a quantitative public-data SLAM benchmark**.

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
| `slam_match_ceres` (optional) | Ceres single-scalar-per-beam cost-function adapter and bounded solver | Repository adapter file **compiled** in an isolated Ceres 2.2 consumer with interface-compatible headers; full repository Ceres regression still not run |
| `slam_carmen` + `slam_carmen_geometry` | Strict CARMEN `FLASER`/`ODOM` parsing and conservative native laser-point conversion (scan timestamp only) | Catch2 importer/geometry tests passed |
| `slam_carmen_inspect` | Local `.log`/`.log.gz` summary and optional expected scan count; zlib remains CLI-only | Both user-uploaded complete FR079 logs parsed in CTest |

**Test evidence:** GCC 14, CMake 3.31.6, Ninja, provisioned Eigen3 on Linux x86-64. **6/6 pre-existing CTest tests passed** with network-free `SLAM_ENABLE_CATCH2=OFF`, including the normal build and a strict build using `-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-sanitize-recover=all`. The EDT tests compare all cells to exact Euclidean reference distances, test rotated grids, immutable source revisions, unknown stencils and truncation. The matching tests check geometry-only convergence toward synthetic truth, overlap, rank, cost reduction and unsupported-region rejection. These are **unit/synthetic tests**, not a public-dataset accuracy benchmark.

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Use `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3` if the Eigen package is not automatically found. Ceres 2.2.0 was compiled and installed locally from the uploaded source with Eigen3/miniglog; a standalone Ceres least-squares consumer solved successfully. The byte-identical `src/match_ceres.cpp` compiled against that installation in an isolated adapter build, **but the full repository Ceres CTest has not yet been run**. Catch2 3.16.1 was also built from the uploaded source; 4/4 additional local integration CTests passed (CARMEN parser, two complete log imports, native laser geometry). That additional suite does **not replace** the original six-test regression suite. No GitHub Actions were created or executed.

## Guarantees and limitations

- **Snapshots are immutable after publication**, but capture is only coherent if the caller serializes copying with map insertion. There is no multi-threaded map owner/rebuild scheduler yet.
- **EDT is exact at cell centres**, with bilinear interpolation using four observed cells and entirely nontruncated stencil values. Unknown/out-of-map/saturated support is **not** a false zero residual. The derived cache is not map authority.
- **Matcher is a local, geometry-only baseline** seeded near the map pose. Its synthetic test uses independent, known synthetic landmark geometry but is not a full SLAM trajectory/loop benchmark. No relocalization/global search or verified sensor bias/clock estimator is implemented.
- The information matrix and its eigenvalues are **observability diagnostics**, not a calibrated independent covariance. A returned tracking pose is not an automatically graph-eligible factor. A local matcher can still have map/deskew correlations.
- **Ceres toolchain builds locally; whole-program solver validation remains.** An unsupported trial evaluation fails rather than yielding an artificial zero-distance residual. The repository matcher end-to-end convergence, false-lock and boundary behavior still need a full Ceres-linked CTest and independent recordings.
- LD19 packet time-reference, MPU6050 gyro bias, real acquisition/MCAP, actual Pi performance, public-dataset SLAM accuracy and full online/offline SLAM remain unverified. FR079 log parsing is verified, **not** full SLAM benchmarking. FR079 reference maps are not yet read by the occupancy mapper.

## Next checkpoint

Run the complete project Catch2/Ceres CTests against the supplied offline dependencies, implement a reproducible FR079 scan-to-map evaluation with held-out frames and truthful source timing, then add **deterministic map export and a simple offline pose-only GTSAM reference graph**, with conservative factor provenance and false-loop checks. Independently progress real Pi LD19/MPU6050 acquisition and data calibration; initial algorithm experiments continue using synthetic and verified public data.
