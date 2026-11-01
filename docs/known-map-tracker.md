# Read-only known-map localization trust gate (P2 foundation)

**Scope:** `slam_known_map` is a portable C++20 state/trust layer on top of an
immutable `DistanceField` and the existing geometry matcher. It does **not**
implement robot acquisition, an independently calibrated sensor, global recovery,
a survey-grade absolute pose, or a Pi latency benchmark.

## What is implemented

- Consumes an explicitly supplied **absolute map anchor** and a **same-time
  odometry pose**. It never invents an initial pose or motion. The caller attests
  map provenance, verified laser extrinsics and an independently anchored pose.
  These booleans are **not** signatures or independent proof.
- Starts LOST and publishes **no pose** merely from receiving an anchor. Only
  a subsequent accepted, well-supported scan can enter TRACKING.
- Uses an immutable `DistanceField`; keeps revision/fingerprint pinned, checks
  matcher acceptance, input/support accounting, overlap, LiDAR-only rank and
  finite cost, and bounds pose innovation. The default matching backend is the
  deterministic reference matcher. Consumers built with Ceres may inject
  `slam::matching::match_ceres` as the `Matcher` callback.
- Creates `T_map_base` from matched `T_map_laser * inverse(T_base_laser)` and
  `T_map_odom` from the **same observation's** `T_odom_base`. It does not
  jump the supplied odom trajectory or pretend a wheel source exists.
- Returns current scan global poses **only for an accepted update**. A rejected
  scan enters UNCERTAIN, a rejection streak enters LOST, and invalid
  calibration/epoch/domain/order or host-steady freshness destroys the anchor.
  The watchdog does not return an old pose as a fresh one.
- Limits input scan hit count and source-time gap before invoking the matcher.
  `request_relocalization()` explicitly reports RELOCALIZING but does not
  implement a coarse/global search or autonomously certify a map lock.

## Build and run synthetic test

From the repository root:

```sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF -DSLAM_ENABLE_CERES=OFF
cmake --build build --target slam_known_map_test
ctest --test-dir build -R '^known_map_tracker$' --output-on-failure
```

Eigen3 must be installed, or supply `-DSLAM_EIGEN_INCLUDE_DIR`. This
CTest uses a deterministic, deliberately synthetic occupancy/EDT scene for one
actual `match_reference` integration and scripted matcher diagnostics for
failure/clock/anchor/watchdog paths. Tests never require real hardware,
recordings, a GTSAM graph or GitHub Actions.

## Trust and safety contract

`Anchor` must contain `stamp`, `map_from_base`, `odom_from_base` and a
nonzero `calibration_revision`. The caller must set all three explicit
provenance flags **only after independent verification**. False attestations
are outside this component's ability to detect; map checksums and
fingerprints are not cryptographic authentication.

Each `Observation` supplies scan-level source timestamp, host **steady**
receipt timestamp, same-time odometry, calibration revision, laser endpoints
and evidence flags. `acquisition_time_verified` must be false where source
timing is provisional; uncalibrated FR079, raw LD19 inferred beam timing and
unregistered maps must not be relabelled verified to activate tracking.
`odom_valid` means a physically supported odometry pose, **not** a
fabricated zero-translation wheel estimate.

A consumer must call `watchdog(now_steady_ns)` while no scans arrive, and
must treat `Estimate.globally_valid()==false` as no authoritative current
global pose. The returned timestamp belongs to the observation; freshness
uses an independent steady clock. The default 500 ms timeout, 1 s scan-gap,
720 hit cap, 0.35 m / 0.35 rad innovation, and two-reject limit are **test
settings**, not Pi-validated safety or reliability thresholds. The component
has no real-time execution deadline: Ceres/runtime load still needs Pi
profiling and externally bounded worker scheduling.

This is the **first P2 known-map tracking substrate**, not the complete
FR-10 acceptance milestone. The remaining gates are trusted sensor
acquisition/clock/extrinsic calibration, a real independent map anchor,
verified runtime source and bounded queue processing, hardware fault tests,
and independent localization error measurement.
