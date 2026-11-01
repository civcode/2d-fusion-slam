# FR079 LiDAR + odometry offline SLAM (incremental, no prior map)

This experiment processes the recorded CARMEN FR079 LiDAR and odometry
**in timestamp order**, beginning with an empty occupancy grid. Unlike
the earlier FR079 training-only diagnostic, **no archived reference map
or logged laser-pose estimate is used as the trajectory or map input**.

## Build and run

The full Ubuntu 24.04 Docker test image has Ceres, zlib and OpenSSL.
Choose an output basename that does not already exist. The directory
must exist and must be the **same directory used by the native viewer**
if you want both the new map and scan replay in the Web UI.

~~~sh
export BUILD_JOBS=28
export SLAM_DOCKER_JOBS="$BUILD_JOBS"
docker compose build test demo

# Replace with your actual local dataset location.
export FR079_LOG=/absolute/path/to/fr079.log.gz
mkdir -p apps/demo/maps

docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  --entrypoint ./build-docker/slam_fr079_offline \
  test /input/fr079.log.gz /output/fr079-slam

docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker -R fr079_offline_slam --output-on-failure

docker compose up -d demo
~~~

View the new file at **http://127.0.0.1:8765/** (Map Explorer, select
fr079-slam.slammap) and **http://127.0.0.1:8765/replay** (select
fr079-slam.slamreplay.json). Do not select the unrelated workshop demo
or legacy FR079 logged-pose replay when evaluating reconstructed maps.

Native local build:

~~~sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON
cmake --build build --target slam_fr079_offline --parallel 28
./build/slam_fr079_offline /path/to/fr079.log.gz apps/demo/maps/fr079-slam
ctest --test-dir build -R fr079_offline_slam --output-on-failure
~~~

If Ceres is intentionally unavailable, add **--reference** to use
the bounded local geometry optimizer. The report identifies the actual
backend; no hidden solver fallback is claimed. Optional parameters:
--match-every N (default 5), --field-every N (default 15),
--map-stride N (default 3), --match-stride N (default 4).
All are deterministic, strictly bounded integer settings.
Remove old output files or use a new prefix before repeating a run.

## Actual pipeline

1. Read and validate the complete CARMEN file, including the 360-beam
   FLASER records and scan-level IPC timestamps. The embedded per-scan
   **logged odometry** supplies the motion prior. Separate ODOM rows
   are validated but are not independently fused/interpolated.
2. Set the **first odometry pose** as the gauge origin. Use odometry
   *increments* to predict each successive laser pose; do **not**
   substitute logged laser pose (which may already include previous
   SLAM correction) or the public FR079 reference occupancy map.
3. Allocate a bounded grid from the **odometry-only extent**, with a
   conservative sensor-range margin. This is an offline memory-layout
   prepass, not advance occupancy information. Online mapping would
   need growing map tiles instead.
4. At configurable scan intervals, match actual range hits using Ceres
   (or explicitly requested reference optimizer) against a distance
   field captured **before the query scan is inserted**. The field
   refreshes at a bounded cadence. Accept only rank, support, overlap,
   source-revision and bounded-innovation gated corrections.
5. Insert the current scan's valid in-range LiDAR rays at the accepted
   pose or odometry prediction. Unknown, zero and over-range returns
   are never invented as wall or free-space evidence.
6. Carry the corrected pose into the next odometry prediction. Export
   an authoritative FUSMAP2D occupancy map plus reproducible, ordered
   trajectory and per-frame match artifacts.

## Output files

Using output prefix fr079-slam creates:

| Name | Meaning |
| --- | --- |
| fr079-slam.slammap | Authoritative occupancy grid, FUSMAP2D, read-back checked |
| fr079-slam.pgm / .yaml | Lossy ROS-style visualization / metadata |
| fr079-slam.slamreplay.json | FUSREPLAY1 using *estimated SLAM* poses, not recorded laser poses |
| fr079-slam.trajectory.json | FUSFR079POSE1: odometry, prior, corrected estimate per scan |
| fr079-slam.matches.json | FUSFR079MATCH1: decisions, support, source-excluded matching times |
| fr079-slam.report.json | FUSFR079SLAM1: counts, settings, map fingerprint, per-frame p50/p95/p99, offline processing FPS |

The replay carries a SHA-256 of the **uncompressed input file
contents** and an explicit pose_kind ending in
not_ground_truth. The .slammap metadata labels its origin
fr079_odom_scan_frame_UNVERIFIED and
metric_origin_verified=false, even though the viewer can draw the
map and estimated trajectory together. Shared strings do *not*
supply independent calibration evidence.

## What has NOT been established

- **Not** LiDAR + gyro + wheel fusion: FR079 contains no validated
  MPU6050 gyro stream or per-beam acquisition timing. The estimator
  uses only LiDAR and recorded odometry, with no motion deskew.
- **Uncalibrated laser extrinsics:** the first experiment explicitly
  assumes laser and odometry frames coincide. This is **not** a
  measured mounting transform. Metric map/odom frame alignment is
  unverified; corrected pose and map geometry must be assessed with
  that limitation in mind.
- **No loop closure or GTSAM result:** the CARMEN data does not supply
  independently verified loop constraints. No correlated scan-to-map
  measurement is silently promoted to an independent graph factor.
  This is an end-to-end **incremental SLAM frontend**, not a complete
  independently validated global loop-closing system.
- **No independent accuracy ground truth:** logged laser poses are
  estimates, the FR079 reference occupancy map was generated from
  related observations, and its metric registration is unverified.
  No ATE/RPE or ground-truth IoU is reported.
- **No live real-time guarantee:** reported mean processing FPS and
  individual native scan-work latencies come from a batch replay.
  There is no paced input queue, deadline/drop policy or robot device
  synchronization yet.
- No map publication or trust promotion: the exported occupancy
  remains a research artifact with unverified physical provenance.

## Regression and acceptance

fr079_offline_slam synthesizes a CARMEN scan sequence with walls,
checks the actual native SLAM executable and FUSMAP2D/replay/report
exports, mutates the **logged laser pose fields only** and demands
byte-identical reconstructed maps, changes raw odometry and expects
the map to change, and rejects malformed timestamp order and
overwriting existing output. With SLAM_FR079_LOG configured,
fr079_offline_real_data runs the full 3,118-scan recording separately.

The authoritative experiment must still be built, run and measured
locally on the actual data. No unpublished FR079 accuracy or measured
speedup is assumed by this document. Tests are manual; no GitHub
Actions are enabled or executed.
