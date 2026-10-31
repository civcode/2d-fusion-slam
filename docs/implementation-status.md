# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Branch:** `main`  
**Scope:** C0 and initial C1 native primitives; **not yet a SLAM pipeline**.

## Implemented, with local tests

| Target | Working implementation | Local test |
| --- | --- | --- |
| `slam_core` | Checked signed nanosecond time, clock domain/epoch, SE(2) exp/log/compose | `core_smoke` |
| `slam_fixtures` | Deterministic apartment/warehouse exact-time synthetic moving-beam ray casting | `fixtures_smoke` |
| `slam_sensors` | Angular seam scan assembly, partial coverage, acquisition midpoint, source/calibration/time-quality consistency | `scan_mapping` |
| `slam_mapping` | Rotated bounded known-mask/log-odds grid, conservative ray insertion and hit precedence | `scan_mapping` |
| `slam_ld19` | 47-byte native packet parsing, CRC-8 (poly 0x4D), raw-frame and CRC-failure events, fragmented UART stream resynchronization, nominal 30-second device-counter unwrap, provisional beam angles/relative timing | `ld19_motion` |
| `slam_motion` | Interpolation-only sampled SE(2) trajectory, strict time coverage, lever-arm midpoint deskew of valid hits, explicit gyro-only/full-SE(2) and provisional/verified timing safeguards | `ld19_motion` |

**Verified locally:** GCC 14 / CMake 3.31.6 / Ninja / provisioned Eigen3. **4/4 tests pass** with network-free `SLAM_ENABLE_CATCH2=OFF` in all three configurations: normal, `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined`, and the same strict warnings with `-fsanitize=address,undefined -fno-sanitize-recover=all`. Tests cover frame fields, independent CRC vector, packet corruption and resync, source receive clock discontinuity, device counter wrap/reset/gaps, provisional timing, timestamp overflow, exact synthetic twist interpolation, laser-mount lever arm, moving-world-point deskew and safe refusal of unsupported mapping quality.

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Set `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3` if needed. Pinned Catch2 v3 FetchContent tests remain **unverified locally** because direct network access to GitHub is unavailable. No GitHub Actions were run.

## Explicit limitations and safety boundaries

- **Raw byte recording is not yet implemented.** The stream parser retains recognized 47-byte candidates (even when CRC fails), but this is *not* an authoritative lossless UART capture. A real acquisition/MCAP recorder must preserve all incoming bytes separately.
- **Device counters are not canonical acquisition time.** The 30,000 ms unwrap policy is a nominal model and rejects ambiguous gaps, resets and discontinuities. Actual LD19 packet timestamp reference, transport delay, host clock drift and true beam-time error remain unmeasured. No wall/receipt timestamp is silently treated as physical beam acquisition.
- **Relative beam timing is provisional.** Angle and speed produce relative offsets under an explicitly assumed last-beam reference. Calibrated/synthetic exact anchors require an explicit physical-reference-validation flag, and scan quality carries the provisional/calibrated distinction. Real calibration evidence must precede claims about validated per-beam timing.
- **No gyro estimator or measured XY source yet.** The trajectory class interpolates **supplied** poses; it does not read MPU6050, integrate gyro, estimate bias, or infer wheel velocity. Gyro-only capability does not permit metric XY translation or full moving-scan mapping. Provisional timing is diagnostic-only by default.
- **No direct raw-scan-to-map insertion.** Deskew returns reference-frame hit points with a mapping-suitability check. The existing occupancy grid accepts separately validated world-space rays; this milestone does not secretly compose these into a production mapping pipeline.
- Physical LD19 firmware variation, packet timing and serial noise have **not** been tested on real hardware. Public dataset conversions, independent accuracy measurements and Pi performance have not been tested either.

## Benchmark evidence

B01–B06 have **partial synthetic unit/integration coverage**, not full acceptance: B02 lacks physical timing/real LD19 recordings, B04 lacks gyro bias/wheel source measurements, B05 lacks sensor/dataset comparisons, B06 lacks submap lifecycle. B07–B18 are unexecuted.

## Next checkpoint

Implement a synthetic/verified-public-data frontend using immutable occupancy snapshots and a baseline Euclidean distance field (B07), followed by Ceres geometric scan matching and quality diagnostics (B08). In parallel, continue Pi LD19/MPU6050 raw acquisition and calibration work, plus measured gyro motion and time alignment. Continue to prefer synthetic/public benchmark evidence before optimization.
