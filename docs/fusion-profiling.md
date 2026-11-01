# Fusion SLAM native profiling and reproducible benchmarks

The offline C++ sensor-fusion pipeline exports **fusion_profile.json**
(FUSPROFILE1) next to its maps and replay files. Measurements use
std::chrono::steady_clock around actual processing, not browser animation
or fabricated time estimates. The independent source bench is inspired by
the profiling-ON / profiling-OFF discipline in hybrid-astar-cpp.

## What's measured

| Stage | Measurement | In fused-path throughput? |
| --- | --- | --- |
| synthetic_generation | Fixture/sensor synthesis | No |
| sensor_integration | Calibrated gyro + wheel integration | Yes |
| scan_deskew | Per-scan trajectory interpolation and LiDAR deskew, all scans | Yes |
| training_map_build | Immutable even-index source map and distance-field construction | Yes |
| heldout_scan_match | Ceres or reference matcher + qualification, odd-index scans | Yes |
| lidar_only_baseline | Independent rolling LiDAR-only matching and insertions | No |
| wheel_map_build | Final wheel-only occupancy | No |
| lidar_map_build | Final LiDAR-only occupancy | No |
| gyro_map_build | Gyro + wheel map before scan matching | No |
| fused_map_build | Final scan-corrected occupancy map | Yes |
| fused_map_insert | Per-frame work *nested within* fused_map_build | Counted through parent, not twice |
| graph_optimize | Optional solver graph (or empty fallback) | Yes |
| graph_map_build | Optional independent post-optimization map | No |

Every reported stage includes a call count, wall time total, mean,
p50, p95, and maximum sample time (milliseconds). Timing instrumentation
is **coarse**—there is no timer per LiDAR point.

* effective_processing_fps = frame_count * 1000 / fused_critical_path_ms.
* realtime_factor = scan_time_span_budget / fused_critical_path_ms.
* average_offline_budget_met is the coarse average throughput budget
  comparison against 300 ms/scan (3.3333 Hz input rate).
* frame_proxy_p95_ms is a **proxy** combining each frame's measured
  deskew, match and map-insertion times with amortized batch overhead.
  It is *not* observed end-to-end per-scan live latency.

These timings are measured on **one offline batch execution**, including
a frozen-map training phase that would not be performed this way by a
real online frontend. They exclude synthetic input generation, alternative
baselines, evaluation and file I/O. **Offline budget met never proves a
real-time-capable robot**: that requires wall-clock input arrivals,
online incremental SLAM, real LiDAR/gyro sync, queue/backpressure
instrumentation, jitter and target-device thermal testing. Note also
that stage times are nested (fused_map_insert inside fused_map_build),
so **never add all stage totals together**.

## Native build and test

~~~sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DSLAM_PROFILE_FUSION=ON
cmake --build build --parallel 28 --target \
  slam_offline_fusion slam_fusion_profiler_test fusion_slam_demo
ctest --test-dir build -R 'fusion_profiler|offline_fusion|fusion_replay_http' \
  --output-on-failure
./build/slam_offline_fusion apartment apps/demo/fusion/apartment-profile-001
# Open /fusion with the native server to inspect the new profiling panel
~~~

For a controlled **unprofiled** build, use a separate build directory
with -DSLAM_PROFILE_FUSION=OFF. No stage timers are started in that build;
the profile JSON explicitly reports profiled=false with null performance
metrics (not misleading zeros).

## Repeated benchmarks with CSV

The manual benchmark driver generates temporary output directories for
each trial. One optional warmup is discarded, then each scene is run
repeatedly through the real native executable. Its CSV includes external
**whole-process wall time** (simulator, all baselines, scoring and I/O),
the profiled fused compute work and stage hotspots, so those two
different interpretations are not mixed:

~~~sh
python3 scripts/benchmark-fusion.py \
  --binary ./build/slam_offline_fusion --scene both \
  --warmup 1 --runs 5 --csv fusion-performance.csv
~~~

The CSV is **exclusive-create**. Pick a new name for each run. The
command also prints per-scene median and p95 process duration and
median/worst offline fused FPS. Use matching input scenes, release
settings and hardware for comparisons, with several independent runs.
Avoid running other CPU-intensive jobs concurrently.

## Docker and BUILD_JOBS=28

~~~sh
export BUILD_JOBS=28
export SLAM_DOCKER_JOBS="$BUILD_JOBS"
export SLAM_PROFILE_FUSION=ON
docker compose build test demo

docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --parallel "$BUILD_JOBS" \
  -R 'fusion_profiler|offline_fusion|fusion_replay_http|native_web_api' \
  --output-on-failure

mkdir -p benchmarks
docker compose run --rm \
  -v "$PWD/benchmarks:/results" \
  --entrypoint python3 test \
  scripts/benchmark-fusion.py \
  --binary ./build-docker/slam_offline_fusion \
  --scene both --warmup 1 --runs 5 \
  --csv /results/fusion-profiled-001.csv
~~~

To compare profiling overhead, repeat with
SLAM_PROFILE_FUSION=OFF docker compose build test and a **different**
output filename. OFF builds produce external process-time values but
null stage/FPS values by design. Do not use these profiled synthetic
numbers as calibrated Raspberry Pi or live robotics performance.

## Web UI and consistency

The browser Fusion Replay has a **Native Processing Profile** panel,
fetching /api/fusion/runs/{id}/profile. It displays FPS, input rate,
the averaged input budget, frame-proxy p95, critical compute time and
sorted expensive stages. Missing profiles from older generated runs
are shown as unavailable and do not break map or trajectory display.

A run's profile must agree with its FUSFUSION1 scene, matcher, frame
count and scan period. The native API rejects modified or non-finite
data and does not claim live real-time verification.

**Verification status:** Complete native end-to-end CTest and Docker
execution must be performed on a full checkout. This ChatGPT runtime
cannot clone GitHub. No GitHub Actions have been invoked.
