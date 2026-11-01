# Recorded replay and FR079 map performance fixes

The FR079 recorded replay used to issue an HTTP GET for each displayed
scan. The native single-threaded viewer re-opened, re-parsed and
re-validated the *whole* FUSREPLAY1 file, including all 3,118 scans,
every time. At playback speed 4x, multiple asynchronous fetch requests
overlapped and their older responses were ignored. This caused the
Play button to show Pause while the frame number did not advance.
The same queued replay requests delayed occupancy-map loading.

## Corrected behavior

- GET /api/replays/{name}/frames provides FUSREPLAYFRAMES1 with
  source_sha256 and the full validated frame collection, including
  verified optional Ceres sidecar results. The existing per-frame
  endpoint remains available for older clients and tests.
- The browser fetches metadata and the complete frame batch once at
  session load time. It verifies the SHA-256 identity, count, and first/
  last timestamps, then seeks and plays entirely in memory.
  All animation is based on recorded scan timestamps and the selected
  speed, not per-frame HTTP latency. The UI displays an explicit
  loading state while the one-time batch arrives.
- Session changes and map selection invalidate earlier in-flight
  responses. Replaying from the final frame starts over at frame 1.
- The native Map Explorer caches one fully validated immutable
  occupancy snapshot and its rendered PNG. A map is still checked
  using lstat on each request; cached content is invalidated on
  file device, inode, size, modification-time or change-time changes,
  and symlinks are refused. Map PNG uses faster zlib compression and
  classifies the same 0.65/0.35 occupancy thresholds directly in
  log-odds space without computing exp() per cell.
- The older Python viewer supports the same batch API because its
  browser assets are shared with the native viewer.
- The recorded replay now distinguishes **FR079 estimated SLAM poses
  and final reconstructed occupancy maps** from legacy logged pose
  sessions or independent reference maps. Faster playback controls
  include 8x, 16x and 32x for a long FR079 session.

## Update your existing Docker viewer

From the repository root after pulling main:

~~~sh
export BUILD_JOBS=28
export SLAM_DOCKER_JOBS="$BUILD_JOBS"
docker compose build demo test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'native_web_api|fr079_offline_http|web_ui_unit|replay_browser_runtime'
docker compose up -d --force-recreate demo
~~~

Refresh http://127.0.0.1:8765/replay and select the existing
fr079-slam.slamreplay.json. The browser will load all 3,118 scans once;
after that Play, step and dragging the timeline do not generate
per-frame API traffic. In the optional map dropdown choose
fr079-slam.slammap, which is the final reconstructed map from that
same run. Its frame alignment remains **unverified**. The output
map and session do not need to be regenerated.

Native regression tests now check /frames and /frame/N parity,
accepted/rejected optional Ceres diagnostics, map cache invalidation
after mutation, and FR079 replay provenance. When Node is already
installed, CMake also registers replay_browser_runtime to simulate
Play, Pause, restart and time-based frame advances with a browser-like
DOM, asserting only one batch request.

**Validation:** Browser JavaScript syntax and an actual V8 browser
mock passed, advancing from first to last frame without per-frame
requests. Complete C++ CTest/Docker execution and measured FR079 map
load latency remain unverified in this session because the local
container cannot clone the repository. No GitHub Actions were run.
