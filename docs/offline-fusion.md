# Offline LiDAR + gyro + wheel SLAM experiment

**Scope:** a runnable, deterministic **C++20 synthetic sensor-fusion
pipeline** using the existing moving LiDAR simulator, clock/SE(2) trajectory
machinery, Ceres matcher when installed, optional GTSAM graph, authoritative
FUSMAP2D maps and exact synthetic map-accuracy evaluator. This is a
**research/validation executable**, not yet an LD19/MPU6050 device driver,
real-world measurement or deployed online SLAM.

## Build and run

The portable fusion frontend and reference geometry matcher build with Eigen3.
Ceres and GTSAM are optional workstation backends; the report records which
ones actually ran. Enable both for the full requested solver path:

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON
cmake --build build --target \
  slam_offline_fusion slam_gyro_wheel_test slam_map_accuracy_cli --parallel 2
ctest --test-dir build -R 'gyro_wheel_fusion|offline_fusion' --output-on-failure

./build/slam_offline_fusion apartment /tmp/fusion-apartment-a
./build/slam_offline_fusion warehouse /tmp/fusion-warehouse-a

# Only in a GTSAM-enabled build: an extra independently simulated
# relative-pose loop SENSOR (not derived from SLAM map matching).
./build/slam_offline_fusion apartment /tmp/fusion-apartment-loops \
  --synthetic-independent-loops
~~~

To run without heavy solvers, set both `SLAM_ENABLE_CERES=OFF` and
`SLAM_ENABLE_GTSAM=OFF`. The CLI then reports
`matcher=geometry_reference_fallback` and `graph_status=gtsam_not_built`
rather than pretending that either solver ran.

The CLI refuses an existing output directory; choose a new path for each run.

## Input and honest sensor model

- Synthetic apartment/warehouse fixtures generate **moving** 240-beam scans
  with **exact simulated per-beam acquisition timestamps** and a nonzero
  laser mounting translation/yaw; timestamps are in one explicit
  `clock_domain=101, epoch=1`. Synthetic truth is used *by the simulator*
  to generate physical measurements, not by the estimator to reconstruct
  map poses.
- A deterministic 40 Hz gyro/wheel stream supplies angular rate with
  **0.025 rad/s injected gyro bias**, wheel forward velocity with a
  2.5% scale bias and independently biased wheel yaw rate. The first
  12 externally designated *stationary* ticks calibrate gyro bias;
  this status is an explicit simulator attestation, not inferred from
  gyro samples alone. The first exact synthetic pose establishes a single
  global gauge/initial anchor.
- The portable `slam::fusion::integrate_sensors` library refuses mixed
  clock domain/epoch, nonmonotonic timestamps, gaps above the configured
  maximum, absent/implausible gyro, unsupported stationary calibration,
  out-of-bound bias, and missing wheel translation. Its **optional**
  gyro-only diagnostics mode never claims XY motion or moving-map readiness.
  It never interpolates missing measurements.
- The estimator uses the library's timestamped, integrated full-SE(2)
  odometry trajectory to deskew each hit with the existing
  `slam::motion::deskew_hits` and the simulated laser extrinsics.
  All scan and beam calibration/source/timing claims must agree;
  provisional LD19 packet timing would be refused for mapping.

The synthetic motion is a smooth repeatable interior-room circle in the
apartment and a 14 x 9 m-radius perimeter ellipse in the warehouse. The
warehouse route is regression-checked against shelf collisions; the former
13 x 8 m ellipse intersected shelves and is no longer used. This **is not a
complete environment exploration/coverage guarantee**, and occupancy
quality may remain low because some walls are never observed.

## Processing and evidence provenance

1. Generate synthetic gyro, forward wheel and wheel-yaw observations.
   Estimate **both** a gyro+wheel trajectory and independent wheel-only
   baseline from the same raw stream. No oracle pose enters integration.
2. Deskew the moving LiDAR hits with each measured full-SE(2) trajectory.
   Build the **frozen even-index scan training map** from the predicted
   gyro+wheel poses, then match *only odd-index scans* against that immutable
   map. The CLI dispatches the existing Ceres matcher when it is linked,
   otherwise names the explicit reference-geometry fallback.
3. Accept only a positive matcher status with a matching training-map
   revision/fingerprint, adequate rank/hits/overlap and bounded correction.
   Keep rejected predictions, and rebuild the full result map from
   **the corrected estimated trajectory**. A matcher status is *not* a
   measurement of absolute accuracy.
4. A separate **LiDAR-only** baseline uses an initial map-frame anchor and
   matches each raw scan to its own previously accepted occupancy map.
   It uses **neither gyro nor wheel odometry**. It cannot deskew moving
   scans, so is explicitly a disadvantaged stationary-scan approximation;
   critically, rejected scans are not silently inserted into the map.
5. When the offline GTSAM target is built, create sequential
   `PoseEdge` factors from the *pre-matching* gyro/wheel stream (not from
   accepted map-match pose deltas). Graph initial poses may be
   scan-matched predictions, but edge provenance remains independent of
   those predictions. No LiDAR matcher result is promoted to an independent
   loop measurement or an empirically measured covariance.
6. By default the graph contains **no loop closure**, since LD19/MPU6050
   alone do not provide an independent relative-pose sensor. The explicit
   `--synthetic-independent-loops` option generates a separately sourced
   relative-pose sensor **inside the simulator**; its measurements,
   not the oracle trajectory, enter the graph. A candidate still requires
   eligible source-excluded LiDAR geometry and is subject to GTSAM's
   source-independence, separation and consistency gates. The synthetic
   relative sensor is a simulated extra instrument, not a claim about
   existing real hardware.

Sequential odometry sigma and synthetic loop sigma in the graph are
**configured experiment noise-model assumptions**, not estimated
posterior covariance. The graph's objective reduction is not itself a
ground-truth position metric.

## Outputs and metric comparisons

The new directory contains:

| File | Contents |
| --- | --- |
| `wheel_only.slammap` | Raw wheel-only odometry map, with wheel-derived moving-scan deskew |
| `lidar_only.slammap` | Raw, non-deskewed, accepted-only LiDAR scan-to-local-map baseline |
| `gyro_wheel.slammap` | Bias-calibrated gyro + wheel estimated-pose map |
| `fused_scan_matched.slammap` | Gyro + wheel + accepted Ceres/reference scan corrections |
| `graph_optimized.slammap` | Optional GTSAM-optimized map when GTSAM is compiled |
| `trajectories.json` | FUSPOSES1 timestamped estimated pose paths, nanosecond timestamps as strings and separately labeled synthetic reference for scoring |
| `fusion_report.json` | FUSFUSION1 with fixed-gauge translation RMSE/p95, yaw RMSE, FUSMAPACC1 metrics, and held-out/LiDAR-only rejection-reason histograms |
| `fusion_replay.json` | FUSFUSREPLAY1 actual per-frame source-excluded match decisions, sampled deskewed laser rays, map insertion flags, cumulative counts and optional independent synthetic GTSAM loop events |

The generated map's provenance is `synthetic_estimated`, and the
separate **reference** is `synthetic_exact`. Evaluation uses the exact
fixture walls and common declared map frame with **no best-fit map
registration**. The report preserves regressions, so there are no
fabricated improvement percentages or real-world accuracy claims.

CTest `gyro_wheel_fusion` exercises gyro bias, wheel-only yaw, gyro-only
missing-translation trust, clock epoch/domain failures and missing
samples. The Python-driven native `offline_fusion` CTest executes the
C++ program in both scenes and compares its embedded map scores to
those produced independently by `slam_map_accuracy_cli`, checks
deterministic repeatability and tests exclusive-output refusal. No
GitHub Actions need to be run.

## Important limitations

- The simulator provides already synchronized, exact per-beam timing and
  ideal extrinsics. **No physical LD19/MPU6050 calibration, clock mapping,
  packet-to-gyro synchronization or hardware recorder exists here yet.**
- Stationarity is caller-attested, not detected robustly. A real pipeline
  must calibrate gyro scale/axis, bias drift and temperature, extrinsics and
  device clocks before the trust flag can be true.
- A continuous ellipse is a controlled geometric fixture. It does not
  exercise arbitrary stop/start movement, multipath, dynamic obstacles,
  hardware dropouts or real-wheel skid. Integration deliberately fails
  closed on missing data rather than inventing poses.
- A map-to-map fixed-frame evaluator and trajectory RMSE against synthetic
  truth are meaningful **only for this exact synthetic reference**.
- A real independent loop-closure measurement has not been acquired.
  Geometric match confidence alone does **not** justify creating one.
- The front end is a **batch replay** of timestamped observations,
  not a low-latency Pi SLAM daemon. The LiDAR-only baseline assumes
  stationary scans and carries known limitations.

**Local verification:** Executed in Ubuntu 24.04 Docker with Ceres 2.2 and
GTSAM 4.2; all 31 CTests passed with 28-way parallelism. Both scenes were run
with and without the extra simulated loop sensor. All 20 exported maps were
re-scored through the accuracy CLI, trajectory errors were independently
recomputed, and primary outputs repeated byte-for-byte. Results reveal failed
LiDAR-only bootstrap and poor warehouse map accuracy despite successful program
execution. See [measured results and limitations](slam-evaluation.md). These
runs do not consume the web UI workshop replay and do not validate hardware.


To visualize the completed process, see [the read-only Fusion Replay Web UI](fusion-replay.md) at `/fusion`. Its final occupancy image and optimized graph path are explicitly post-run results; the accumulated points are sampled observations, not an intermediate map reconstruction.
