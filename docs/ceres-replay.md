# Native Ceres diagnostics in recorded LiDAR replay

**Scope:** Actual offline Ceres scan matching against a frozen FUSMAP2D
occupancy map. The output is a **diagnostic visualization**, not a verified
localization accuracy measurement, independent loop closure, map promotion,
or live robot pose. Logged FR079 laser poses are the initial **seeds**,
not ground truth.

## Build and produce results

Build the project with Ceres 2.2, zlib, and Eigen, or use the root Docker
native build stage:

```sh
cmake -S . -B build -G Ninja -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON
cmake --build build --target slam_ceres_replay_export --parallel 2
```

First convert a CARMEN recording to the existing FUSREPLAY1 format:

```sh
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED
```

Place the **separately frozen training-map** export
`fr079-training.slammap` alongside the replay file in `apps/demo/maps`,
then create an independent diagnostic sidecar:

```sh
python3 apps/demo/match_ceres.py \
  /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json \
  apps/demo/maps/fr079-training.slammap \
  --executable build/slam_ceres_replay_export \
  --scan-stride 20 --beam-stride 4

python3 apps/demo/main.py --data-dir apps/demo/maps
# Open http://127.0.0.1:8765/replay
```

The raw log must have the **same SHA-256** as the original recording from
which the replay was converted. The frozen map and replay must declare the
**same frame label**; that is necessary for visual overlay but does **not**
prove metric alignment. Do not use the original FR079 reference map unless
its transform into the logged laser-pose frame is independently established.

## What the native exporter really does

`slam_ceres_replay_export` loads a validated immutable `.slammap`, constructs
its Euclidean distance field once, and parses original CARMEN FLASER data
(plain or gzip). At the specified scan and beam strides it calls the
repository's **`slam::matching::match_ceres`** with each logged laser pose
as the initial estimate. It uses the same overlap, support, rank and
robust-cost improvement gates as the real Ceres frontend. It neither
updates the map nor creates graph factors. Display-only scans use the
separate FUSREPLAY1 beam-sampling policy; Ceres reads the original beams.

The Python wrapper validates native output indices, scan timestamps,
logged-pose seeds, current map fingerprint, map revision and Ceres
acceptance contract before exclusively writing a
`fr079.slamreplay.json.ceres.json` sidecar. It never rewrites a
`.slamreplay.json`, `.slammap`, or persistent generation registry.

FUSMATCH1 sidecars are **at most 4 MiB**, store sparse results, and
include the original recording SHA-256, replay-file SHA-256, map frame,
map fingerprint and revision. The viewer checks them again against the
current files before returning any match through the HTTP API. Changed
replay bytes, mismatched source data, missing/corrupt maps, symlinks and
invalid solver results make the sidecar **invalid**, not authoritative.
No HTTP request runs optimization.

## Browser behavior and caveats

The replay details show actual Ceres status, iteration count, supported
EDT hits, overlap, mean distance and robust-cost change. Frames not
processed at the chosen stride are labeled **not processed**. The
optional orange overlay draws only **accepted** matched poses and
reprojected scan endpoints; rejected frames remain on the original
logged trajectory. The overlay is disabled by default.

This interface explicitly records
`independent_ground_truth=false`,
`training_independence_verified=false` and
`per_beam_time_verified=false`. In particular, FR079's frozen training
map can contain the scans being queried; lower residuals against such a
map are **not** held-out accuracy or proof of robust localization.
No calibrated LiDAR/IMU timing, covariance or independent graph
measurements are invented. The viewer is read-only and unauthenticated;
keep it bound to host loopback.

## Validation

- Local native **Ceres 2.2** exporter compiled and ran on the real FR079
  recording and frozen training map. A bounded run with scan stride 64,
  beam stride 10 processed **49 scans**: 44 accepted, 3 insufficient
  support, 2 no improvement. These are *solver statuses only*.
- **35/35 Python/HTTP tests** passed, including source integrity,
  stale replay/map, rejected-pose and forged-acceptance safeguards.
- The native cross-language fixture passed: C++ Ceres → FUSMATCH1 →
  Python format validation on synthetic data.
- Chromium displayed actual FR079 Ceres overlap/cost information after
  seeking to an accepted scan and enabling the orange overlay (fixture
  browser API; HTTP tested separately).

The full Ubuntu Docker build and repository-wide CTest matrix remain
unverified in this environment. GitHub Actions were not run.
