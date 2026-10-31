# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Branch:** `main`  
**Scope:** C0 foundation and early C1 observation/occupancy primitives; **not a complete SLAM system**.

## Implemented

| Target | Working functionality | Verification |
| --- | --- | --- |
| `slam_core` | Checked int64 nanosecond time with explicit domains/epochs and SE(2) geometry | `core_smoke` |
| `slam_fixtures` | Exact-time deterministic synthetic apartment/warehouse ray casting | `fixtures_smoke` |
| `slam_sensors` | Per-beam timestamps and return kinds, angular-seam scan assembly, CW/CCW support, midpoint from actual acquired bounds, invalid/missing/partial coverage, source/calibration revision checks | `scan_mapping` |
| `slam_mapping` | Rotated-origin bounded 2D log-odds grid, independent known mask, ray clipping, deduplicated hit-over-free updates, opt-in trusted no-return clearing and revision counter | `scan_mapping` |

**Local test evidence:** GCC 14 / CMake 3.31.6 / Ninja / locally provisioned Eigen3. Network-free `SLAM_ENABLE_CATCH2=OFF` configuration built successfully. **3/3 CTest tests passed**, both normally and with `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined -fno-sanitize-recover=undefined` enabled. Tests exercise epoch/source/calibration rejection, scan wrap and partial coverage, invalid/missing rays, hit precedence, no-return opt-in, rotated origins, clipping, neutral-but-known cells, clamping, and a stationary synthetic apartment scan.

Reproduce without external downloads:

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Pass `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3` if Eigen3 is not autodetected. The pinned Catch2 v3 FetchContent tests are **not locally verified** because direct GitHub network/DNS is unavailable. No GitHub Actions were used.

## Scope limitations

- **Scan assembly is generic beam-level processing**, not an LD19 47-byte UART/CRC parser or validated per-beam physical clock model. Source IDs and calibration revisions can be checked; clock drift fitting and calibration workflows remain future work.
- **Occupancy integration accepts already valid world-space rays**, not arbitrary raw moving beams. It performs **no deskew, trajectory interpolation or sensor time alignment**. The synthetic mapping test uses a truly stationary scan; passing it is not evidence that a moving sensor's map is correct.
- The grid is mutable, not yet an immutable revisioned snapshot/submap or portable map bundle. The revision counter does not substitute for the future map-source provenance ledger.
- Missing and invalid returns are never occupancy evidence. No-return free-space clearing is disabled unless explicitly enabled for a sensor with verified semantics.
- B01/B03/B06 have **partial unit/integration coverage**, not complete benchmark acceptance or independent mapping accuracy evidence.

## Not implemented or validated

LD19 packet parser and raw recording; MPU6050/Pi acquisition; packet/gyro clock models; full scan deskew/trajectory provider; submap lifecycle; immutable EDT caches; Ceres matching; GTSAM batch mapping; map persistence; public dataset import/benchmark runs; Pi runtime; Web Lab; ROS; online SLAM; real-hardware calibration/measurements.

## Next milestone

Implement native LD19 packet parsing and source time/beam modeling with synthetic packet/CRC/rollover fixtures (**B02**); support a defensible scan-to-world trajectory interface and deskew quality modes (**B04/B05**) before inserting moving scans. Follow with immutable occupancy snapshots/EDT and the first Ceres matcher (**B07/B08**). Hardware capture may advance independently, while **synthetic and validated public datasets remain the initial algorithm acceptance basis**.
