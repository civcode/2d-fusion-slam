# 2D Fusion SLAM — Benchmarks and Datasets

**Status:** Proposed / dataset candidates not yet provisioned  
**Version:** 0.1  
**Date:** 2026-10-30  
**Related:** [Technical Specification](technical-specification.md) · [Implementation Contract](https://github.com/civcode/2d-fusion-slam/blob/docs/implementation-contract/docs/implementation-contract.md) (proposed in [PR #1](https://github.com/civcode/2d-fusion-slam/pull/1))

> **Purpose:** Establish a small, reproducible benchmark suite from the beginning of the project, covering 2D LiDAR, yaw-rate IMU, wheel odometry, scan deskew, offline mapping, and known-map live localization. The suite MUST distinguish exact simulated ground truth, independent measured ground truth, LiDAR-derived reference trajectories, and sequences with no ground truth.

This document specifies the **benchmark concept and acceptance requirements**. Dataset files, download scripts, Docker services, benchmark executables, and the manifest registry below are **planned**, not yet implemented or verified locally. External datasets MUST NOT be downloaded automatically by normal builds or tests.

## 1. Goals and scope

The first benchmark suite MUST be **small, understandable, deterministic where possible, and fast enough to run frequently in Docker**. Scenarios should represent **apartment- or small-warehouse-sized planar environments** and the sensor modalities of the deployment robot:

- 2D rotating LiDAR (initial target: LDROBOT LD19, ~10 Hz and ~450 points/revolution).
- Gyroscope / IMU (initial target: MPU6050, ~200 Hz).
- Measured wheel-derived motion / odometry (nominal 100–200 Hz where available).
- Ground-truth SE(2) poses when a suitable reference exists.

The suite must serve **four distinct purposes**: (1) deterministic mathematical and regression tests, (2) sensor-fusion and offline SLAM comparison, (3) localization against a frozen map including recovery, and (4) latency/CPU/memory measurements on Raspberry Pi 5 and x86-64.

**Not goals:** downloading entire public corpora; making synthetic performance look like real-world performance; requiring ROS/ROS 2 in the C++20 core; or reporting absolute pose accuracy from wheel odometry / scan-matched self-reference as though it were independent ground truth.

## 2. Initial dataset selection (agreed baseline)

| ID | Selection and role | Required modalities | Duration/size information | Ground truth | Adoption status |
| --- | --- | --- | --- | --- | --- |
| `synthetic-apartment-v1` | **Mandatory always-available** deterministic unit/regression fixture | Simulated 2D LiDAR, IMU yaw rate, wheel readings | Compact, generated locally; small test subset committed | **Exact simulated trajectory** | **Required to implement** |
| `synthetic-warehouse-v1` | **Mandatory always-available** ambiguity/loop-closure fixture | Same as apartment | Compact, generated locally | **Exact simulated trajectory** | **Required to implement** |
| `rosbot-indoor-full` | **Preferred compact real-world fusion candidate** | 2D scan, IMU, wheel-derived odometry | Publisher reports **65.42 s** and **5,395 messages** for the indoor/full recording; **download bytes not yet confirmed** | No independent trajectory reference verified | **Candidate — access/contents/size verification required** |
| `mit-stata-selected` | **Established 3-sensor cross-check** | 2D scan, IMU, wheel odometry | Select **one** reasonable ROS 1 bag/session; individual size and final crop not yet pinned | Floorplan/LiDAR-derived trajectory for some bags; **not independent LiDAR ground truth** | **Candidate — sequence/hash/size selection required** |
| `cartographer-museum-149s` | **Compact LiDAR + IMU smoke baseline** | Horizontal 2D LiDAR and IMU | Published bag `b0-2014-07-11-10-58-16.bag`: **149 s, 38 MB** | No independent reference verified for this sequence | **Documented public choice — extraction/checksum still pending** |
| `iilabs-small-mocap` | **Deferred accuracy benchmark** | Hokuyo 2D LiDAR, Xsens IMU, wheel odometry and encoder readings | Full collection ~**350 GB**; single sequence/sensor download must be scoped and sized | **OptiTrack motion-capture trajectory** (independent) | **Later stage — not in mandatory download/test set** |

**Decision:** Ship the **two synthetic scenarios** as the mandatory default. The **ROSBOT indoor/full** sequence is the first real-world *fusion* candidate but must pass the dataset validation gate before promotion. **MIT Stata** is a second independent-source three-sensor cross-check. The **38 MB Cartographer** bag is a practical, small LiDAR–IMU smoke case, **not** a full-fusion benchmark. Introduce **IILABS later**, when quantitative trajectory accuracy is the priority.

The real-world candidates do **not** replicate the LD19's native UART packet timestamps or the MPU6050's specific noise characteristics. A short **recording from the actual robot** is therefore required eventually (Section 6).

## 3. Dataset-specific source notes and selection criteria

### 3.1 Synthetic apartment: primary correctness fixture

**Proposed generated geometry:** approximately **15 m × 12 m** with several rectangular rooms, narrow doorways, short corridors, and at least one loop. Include a clear turn, one in-place rotation, and nonzero translational and angular motion during scans. Exact walls, trajectory, rates, and seed will be frozen in a versioned fixture once implemented.

**Purpose:** Verify frame transforms, scan geometry, scan-to-submap registration, gyro preintegration, wheel/gyro integration, loop closure, deterministic replay, and accurate full SE(2) deskew against a mathematically known trajectory.

At least one easy/noiseless preset MUST allow strict numerical assertions. A separate noisy preset should add gyro bias/drift, Gaussian gyro noise, wheel-scale errors, occasional wheel slip, and LiDAR range noise/outliers, with specified seeds and units.

### 3.2 Synthetic warehouse: ambiguous-geometry fixture

**Proposed generated geometry:** approximately **30 m × 20 m**, repeating shelves/aisles, a perimeter loop, long parallel walls, and partially symmetric regions. Exact geometry and canonical path will be committed as fixture data.

**Purpose:** Exercise scan matching in degenerate corridors, loop closure gating, alternative global-localization hypotheses, relocalization after an artificial pose reset, occlusions, and dynamic-object rejection. Require at least one false-but-plausible scan-matching candidate to demonstrate why residual minimization alone cannot certify correct localization.

Noiseless accuracy and noisy robustness MUST be evaluated separately; synthetic map truth and the map exposed to the localizer must be clearly differentiated.

### 3.3 ROSBOT Indoor Full (short real-world fusion candidate)

**Source:** [Indoor–Outdoor Synchronized Multi-Sensor Dataset for Mobile Robot Navigation and SLAM](https://github.com/MBarah10/indoor-outdoor-multisensor-robot-dataset). The project's README describes a ROSBOT PLUS with LSLIDAR M10 Plus, wheel-derived `/odom`, and IMU messages; indoor/full topics include `/scan`, `/odom`, `/imu/data`, `/imu/data_raw`, `/tf`, `/tf_static`, and `/cmd_vel`. Formats reported are ROS 2 `.db3` bags and CSV. Publisher metadata reports 65.42 s and 5,395 messages.

**Important caveats:** The available `/odom` represents **wheel-derived odometry**, not necessarily raw left/right encoder ticks. It must not be labeled as raw encoder data or assumed to be statistically independent of any fused state without inspecting its generation path. An independently measured reference trajectory, accurate sensor extrinsics, exact per-beam capture times, and the direct download file size are **not yet confirmed**. The published Zenodo DOI is listed in the source README, but the actual artifact/download and license MUST be independently checked before we pin a manifest.

**Promotion gate:** verify downloadable artifact URL, license, byte size, SHA-256, required topics/types, topic timestamp order, relevant TF, scan angle/time conventions, wheel-odom provenance, and a successful Docker-only conversion/replay. If download access fails or the published bag lacks essential content, **retain as candidate and do not block mandatory tests**.

### 3.4 MIT Stata Center (three-sensor established reference)

**Source:** [MIT Stata dataset downloads](https://projects.csail.mit.edu/stata/downloads.html) and [project caveats](https://projects.csail.mit.edu/stata/). Listed topics include `/base_scan` (2D scan, **40 Hz**), `/base_odometry/odm` (raw wheel odometry, **44 Hz**), and `/torso_lift_imu/data` (**100 Hz**). Original recordings are ROS 1 bags, often with camera streams that we do not need.

**Use:** pick one manageable session (the `2012-04-06-11-15-29.bag` session is documented in published work as a possible starting point), then define a deterministic time-window and topic-only derived subset. Verify that the specific chosen session includes **all** required topics; source-level topic listings are not enough.

**Ground-truth warning:** The maintainers explicitly report changes in hardware/calibration and time-synchronization defects. Their reference poses were aligned with building floor plans using LiDAR scan matching and factor-graph refinement; **they are not an independent LiDAR-blind truth source**. Report accuracy relative to this reference as a **reference-trajectory comparison**, clearly identified as such, rather than independent absolute accuracy.

**Promotion gate:** fix a specific bag/session, subset window, data rights, original and derived SHA-256, sensor extrinsics/topic remappings, a time-base check, and a documented usable ROS 1 import path in the Docker image.

### 3.5 Cartographer Deutsches Museum (lightweight LiDAR–IMU)

**Source:** [Cartographer ROS public data](https://google-cartographer-ros.readthedocs.io/en/latest/data.html), specifically `b0-2014-07-11-10-58-16.bag` (published **38 MB / 149 s**). It contains IMU, horizontal 2D LiDAR, and additional vertical LiDAR; only the **horizontal** scanner plus IMU belong in the baseline.

**Use:** early parser/import, frame handling, LiDAR–IMU synchronization, scan matching, and mapping smoke tests. It is **not eligible** for comparisons claiming improvement from real wheel odometry, since this public dataset lacks wheel odometry.

**Promotion gate:** download only the nominated bag with a pinned source URL and SHA-256, validate topics/frame names, preserve original bag, create a topic-filtered derived subset, and verify that the dataset can run with wheel input disabled without fabricated zero-motion readings.

### 3.6 IILABS (later independent ground-truth accuracy)

**Source:** [IILABS 3D indoor benchmark dataset](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/). Despite its name and additional 3D sensors, it also has **Hokuyo UST-10LX 2D LiDAR** (`/eve/scan`, **40 Hz**), **Xsens MTi-630 IMU** (`/eve/imu/data`, **400 Hz**), **wheel odometry** (`/eve/odom`, **100 Hz**), and a custom encoder topic (`/eve/motors_enc`); it provides calibration and OptiTrack motion-capture ground truth.

**Use:** later accuracy benchmarking, after the converter can select only needed sensor streams and a *single short sequence* with synchronized motion-capture truth. The full corpus is ~350 GB and MUST NOT become a required download. The project offers a selective-download toolkit, but actual per-sequence/per-sensor artifact size and the usability of encoder custom-message definitions must be verified before selection.

**Ground-truth caveat:** The publisher describes external MoCap ground truth, which is much better suited to independent LiDAR accuracy scoring than the MIT reference; temporal/frame alignment and coverage still require validation.

## 4. Synthetic data model and generator contract

The generator MUST be deterministic given **scenario version + generator version + integer random seed + parameter file**. Generator output includes geometry, an exact continuous SE(2) trajectory, per-beam LiDAR measurements/times, IMU samples, wheel measurements, sensor transforms, and a metadata record.

Proposed defaults to be pinned in fixture configuration:

| Quantity | Starting target | Requirement |
| --- | --- | --- |
| 2D occupancy resolution | 0.05 m | Map origin/row convention matches native ROS-compatible grid semantics. |
| LiDAR revolution rate | 10 Hz | Sequential beam sampling, not one simultaneous rigid scan. |
| Measurements/revolution | About 450 | Actual packet/beam model configurable; preserve point order. |
| IMU gyro rate | 200 Hz | Gyro yaw + configured bias/noise; no fabricated absolute yaw. |
| Wheel measurement rate | 100–200 Hz | Wheel ticks/increments or clearly identified wheel-derived odometry. |
| Frames | `map`, `odom`, `base_link`, `laser`, `imu_link` | REP-103/105-compatible axes and explicit extrinsics. |
| Reference | Exact `T_map_base(t)` | Can interpolate at each beam's acquisition time. |
| Time origin | Explicit integer/monotonic | Deterministic timestamp ordering; no dependence on host wall clock. |

**Generator algorithm:** ray cast against a known planar wall/obstacle model at each simulated beam time `t_i` using `T_map_base(t_i) * T_base_laser`. Sample angle, range, optional intensity/validity, and scan reference time. Generate gyro output as true body yaw rate plus bias/noise and wheel output using the selected differential-drive or Ackermann model and its error settings. Include defined missing packets and nonuniform scan angular speed only in separate robustness presets.

Required test presets:

1. **Ideal:** zero noise, known calibration, exact timestamps, deterministic point/trajectory alignment.
2. **Motion distortion:** fast turn + translation within a revolution; compare no deskew, gyro-only deskew, and full wheel+gyro SE(2) deskew.
3. **Bias and slip:** gyro bias drift plus independent wheel-scale/slip errors; check uncertainty and correction logic.
4. **Ambiguity:** repeating warehouse aisles, corridor degeneracy, false loop candidate, kidnapping/relocalization.
5. **Faults:** missing IMU packet, timestamp wrap, corrupted LiDAR packet, time offset, outliers/dynamic obstacles.

LD19 **raw 47-byte packet parsing** MUST have its own deterministic protocol fixture tests (CRC, timestamp rollover, angular convention). Under **accepted OD-003**, add a synthetic packet/transport generator with known **angle/speed-derived beam acquisition times**, an independent configurable **packet-reference delay**, modulo-30-second source ticks, a distinct host-monotonic receive clock, injected drift/latency/jitter and variable motor speed. Verify relative timing against exact simulated truth and report **clock alignment combined-offset estimates**, not an invented independent physical-delay estimate. Generating ideal `LaserScan`-like beams alone **does not** validate the real LD19 timing behavior.

The real LD19 physical packet-reference event remains unverified: test an actual sensor across controlled rotations/translations, host buffering and device restarts to **determine or bound** the acquisition-to-reported-timestamp delay before treating absolute timestamps as calibrated. Preserve original source bytes/ticks, Pi receive times, timing quality, model parameters and calibration revision in each prepared dataset.

Small canonical fixture files MAY be committed when redistribution is allowed and their total size remains practical; keep full generated data out of Git.

## 5. Benchmark categories and experiments

| ID | Benchmark | Inputs | Required observations |
| --- | --- | --- | --- |
| B01 | Planar geometry and frame transforms | Synthetic geometry, known `T_base_laser` | Inverse/compose errors, rotated map-origin correctness, yaw/angle sign. |
| B02 | Gyro and wheel prediction | Gyro/wheel streams; truth where available | Relative pose error, drift over distance/time, yaw-bias estimation. |
| B03 | Full 2D LiDAR deskew | Beam times, trajectory, scan | Endpoint alignment error and scan-match residual for **none vs gyro-only vs SE(2)** deskew. |
| B04 | Ceres scan-to-map | Frozen submap, deskewed scans, pose priors | Position/yaw error, convergence basin, failure rate, runtime, degeneracy. |
| B05 | Submap/offline SLAM + GTSAM | Full sequence / loop | Loop consistency, trajectory accuracy category, map quality, runtime. |
| B06 | Frozen-map live localization | Saved map and held-out traversal | Pose drift / error, tracking dropout, confidence calibration, recovery. |
| B07 | Global relocalization | Unknown/perturbed pose and frozen map | Initialization success, kidnapped-robot recovery, time to stable lock. |
| B08 | Runtime/portability | Same exact dataset+config on ARM64 and x86-64 | p50/p95/p99 latency, throughput, CPU/RAM/thermal, missed deadlines. |
| B09 | Fault tolerance | Dropped points/IMU, timing jumps, wheel slip, dynamics | State transitions, reported uncertainty, graceful recovery and invalid-data rejection. |
| B10 | Benchmark comparison | Multiple deterministic configs | Reproducible side-by-side Web Lab views and machine-readable reports. |
| B11 | LD19 timing reconstruction and alignment (OD-003) | Synthetic device packets + host receive jitter/clock drift; real LD19 capture | Relative per-beam error, clock slope/combined offset, jitter/uncertainty coverage, unknown physical-delay bound, wrap/reset recovery and deskew impact. |

Compare **LiDAR-only**, **LiDAR + wheel odometry**, and **LiDAR + wheel + gyro** wherever the dataset supports them. A `no-wheel` dataset must not be included in wheel-fusion averages. If a dataset provides only processed odometry and not raw ticks, label the experiment accordingly; don't imply independent source availability.

For localization, build the map using a **mapping split** and measure on a **different held-out traversal** wherever data permits. An offline map built from the same scan sequence being localized creates data leakage and MUST NOT be presented as an independent held-out localization result. Synthetic fixtures should provide separate training/map-building and evaluation paths; real sequences with only one run should be labeled self-localization tests.

## 6. Real robot benchmark (required eventually)

Collect a short, deliberate indoor dataset from the project's actual platform using **LD19 + MPU6050 + measured encoders** (if available), ideally with:

- A small room/apartment layout or warehouse aisle; repeatable straight runs, 90°/180° turns, a full loop, and an in-place rotation if kinematics allow.
- Stationary startup segment for gyro bias, varied angular speeds, and some operation near maximum practical translational/rotational speed.
- Raw LD19 UART packets, raw gyro/accelerometer samples, wheel tick counts, original timestamps, receive timestamps, and calibration snapshot.
- Optional manually marked fiducial/reference checkpoints or external measurement. Do not claim exact global ground truth without an independent measurement system.
- A second traversal (same map) to test live localization on **held-out data** and deliberately moved obstacles or a kidnapped-robot scenario.

Use this dataset to validate the **accepted OD-003** angle/speed beam model, 30-second timestamp wrap, LD19-to-Pi drift and receive-time combined offset, **experimentally measured or bounded physical packet-reference delay**, host UART buffering, MPU6050 bias/noise, mounting extrinsics, and the Pi 5 acquisition-to-pose pipeline. The exact sensor hardware is not faithfully represented by the public datasets.

Robot-specific recordings MUST be stored outside Git by default. Record any privacy-sensitive capture details before sharing publicly.

## 7. Canonical data interface and import pipeline

All data must pass through a single ROS-independent C++20 representation. The core supports native dataset replay with **no ROS installation**; import adapters may run with ROS tooling inside dedicated Docker containers.

~~~text
 External ROS 1 bag / ROS 2 bag / CSV / raw LD19 UART
                             |
                      dataset adapter
                (Docker / optional ROS image)
                             |
        validated synchronized native dataset / MCAP
                             |
                deterministic replay engine
                             |
      shared motion -> deskew -> Ceres -> GTSAM
                             |
       poses, map bundles, diagnostics, benchmark metrics
                             |
                    Web Lab and reports
~~~

Required native sample attributes:

- Timestamp and clock domain, with conversion provenance and uncertainty.
- Frame ID, units, sensor extrinsics/calibration ID, measurement validity/quality.
- 2D LiDAR range/angle/intensity and per-beam acquisition timing where known; mark **estimated/unavailable** timing explicitly.
- Raw gyro rate (rad/s), optionally other IMU axes, and meaningful covariance.
- Raw wheel readings **or** separately typed wheel-derived odometry; never silently convert fused EKF odometry to raw wheel measurements.
- Optional independent ground truth, including the source method and exact reference frame.

ROS-compatible frames (REP-103/REP-105), map cell indexing, origin rotation, and angular conventions MUST be preserved. Conversion MUST reject missing required topics, invalid covariance, inconsistent timestamp order, unknown frame transforms, and ambiguous topic provenance unless an explicit permitted fallback is configured.

### 7.1 Proposed folder structure (not yet created)

~~~text
benchmarks/
  datasets/
    synthetic_apartment.yaml
    synthetic_warehouse.yaml
    rosbot_indoor.yaml
    mit_stata.yaml
    cartographer_museum.yaml
    iilabs_optional.yaml
  tools/
    prepare_dataset.py
    validate_dataset.py
    evaluate_trajectory.py
  fixtures/
    # Small committed synthetic samples / golden expectations
  reports/
    # Generated locally; not checked into Git
~~~

External raw and converted datasets MUST reside under ignored, mounted data volumes such as `/data/raw` and `/data/prepared`. Only small manifests, generator definitions, validation fixtures, and published summary reports may be committed. Retain source and derived checksums separately.

### 7.2 Manifest schema (proposed)

Every registry entry MUST be machine-readable and have at least:

~~~yaml
schema_version: 1
id: rosbot-indoor-full
status: candidate  # candidate | verified | enabled | deferred
name: ROSBOT Indoor Full
source:
  project_url: https://github.com/MBarah10/indoor-outdoor-multisensor-robot-dataset
  download_url: null           # must be verified before enabling
  version_or_record_id: null   # pin upstream artifact
  license: unverified         # verify actual artifact rights before fetching
  expected_bytes: null        # unknown, do not invent
  sha256: null                # fill from inspected download, not guessed
format: ros2-sqlite-bag
environment: indoor
sensors:
  lidar_2d: {topic: /scan, required: true}
  imu: {topic: /imu/data_raw, required: true}
  wheel_odometry: {topic: /odom, required: true, kind: derived_odom}
ground_truth:
  type: none                  # simulated | independent | derived | none
calibration:
  extrinsics: to_verify
  clock_alignment: to_verify
selection:
  start_seconds: 0
  end_seconds: null
preparation:
  importer: ros2_bag
  expected_output: native_mcap
verification:
  topic_contract: pending
  timestamps: pending
  permissions: pending
~~~

This YAML is a **schema illustration**, not an enabled download registry or proven file layout. A real implementation MUST choose and document whether all keys are required or whether nested fields have defaults. Synthetic manifests instead include generator version, seed, geometry version, truth trajectory, and noise preset. Downloading by ID is allowed **only** when `status: verified` or `enabled` and a pinned URL/hash have been reviewed.

### 7.3 Download / conversion policy

1. No remote data downloads during CMake configuration, normal builds, normal unit tests, or Docker image build unless an **explicitly requested provisioning command** requires it.
2. Preparation tools MUST take a named dataset ID and an explicit `--download` flag; print candidate URL, license, and expected size before fetching.
3. Verify artifact size and SHA-256 before extraction; reject mismatches. Limit expansion size and reject unsafe archive paths or symlinks escaping the dataset root.
4. Allow offline conversion from pre-downloaded, read-only mounted bags and CSV without network access.
5. ROS 1 bag conversion may require dedicated ROS 1/bridge-compatible tooling or a parser, but MUST NOT make the core ROS-dependent. ROS 2 db3 import is also an adapter responsibility.
6. Preserve original timestamps/frames/covariances, log any transformations (scan decimation, dropped camera topics, time crops, frame remapping), and produce a versioned conversion report.
7. Do not perform undocumented scan downsampling or alter sensor rates to simulate LD19. Sensor fidelity differences MUST remain visible in the report.
8. Reuse cached hashes/artifacts. Dataset preparation and evaluation should run in Docker (and must **not** consume GitHub Actions minutes).

## 8. Benchmark scoring and ground-truth integrity

### 8.1 Truth classes

| Truth class | Meaning | Permitted report labels |
| --- | --- | --- |
| `simulated_exact` | Generator's exact time-indexed SE(2) state | Absolute and relative error **against synthetic truth**. |
| `independent_measured` | Separate MoCap/externally surveyed reference with validated alignment | Absolute trajectory error (ATE), relative pose error (RPE), localization error, and uncertainty calibration. |
| `derived_reference` | Trajectory inferred from the benchmarked sensor family and/or floor plans | **Reference-trajectory discrepancy**, not independent truth error. |
| `none` | Only wheel/IMU/LiDAR measurements or a generated SLAM map | Relative drift proxies, loop closure/endpoint consistency, map self-consistency, timing and recovery statistics; **no ground-truth ATE**. |

The evaluator MUST state whether an SE(2) reference alignment was applied (initial-pose alignment or estimated rigid transform) and how it was estimated. No Sim(2) scale fit for metric LiDAR trajectories by default. Always align trajectory samples by timestamp using documented interpolation and maximum allowed time gap; report evaluation coverage.

### 8.2 Required metrics and their limitations

| Metric | Definition / usage | Applicable datasets |
| --- | --- | --- |
| ATE translational RMSE, median, 95th percentile | Time-aligned estimate vs reference in metres, with stated SE(2) alignment | Synthetic or independently measured truth; derived reference must be labeled as such. |
| Relative pose error (RPE) | Translation (m) and yaw (rad/deg) over fixed time/distance horizons | Synthetic/independent truth; reference-only with caveats. |
| Deskew endpoint error | Difference to ideal ray-cast geometry at reference time | Synthetic truth; controlled real fixture where external geometry is surveyed. |
| Scan-matcher quality | Inlier fraction, mean/robust residual, iterations, convergence and rejection status | All datasets; **not** a substitute for absolute accuracy. |
| Loop consistency | Closure transform residual and verified closure acceptance/rejection | Real/synthetic loops; not independent global position error. |
| Map quality | Occupancy agreement / geometric overlap where a real truth map exists; otherwise qualitative/consistency proxy | Synthetic; real if independent surveyed map exists. |
| Localization health | Tracking dropout rate, uncertain/lost fractions, initial-convergence and kidnapping-recovery success/time | Known-map and held-out or clearly self-evaluated sequences. |
| Computation | p50/p95/p99 deskew, scan match, map update, GTSAM backend and capture-to-pose latency; deadline misses | Pi 5 / x86-64 on identical prepared input. |
| Resource use | Peak RSS, CPU utilisation, thermal throttle events, processed scans/s | Target hardware; log architecture/build/image digest. |

A benchmark report MUST include success/failure and refused/invalid measurements as well as the successful subset; excluding failures without disclosure is prohibited. Record actual sensor frequencies, dropped message counts and number of evaluated scans. A missing metric must be **null/unavailable with a reason**, never zero.

## 9. Comparison matrix and experimental methodology

Every configuration comparison MUST use a pinned dataset, scan selection, calibration, generator seed, software commit, Docker image, solver settings, initial-pose perturbation, and reference map version. Repeat stochastic trials with fixed published seeds and summarize sample size and dispersion; differentiate warm-up from timed runs.

| Comparison | Configurations | Interpretation |
| --- | --- | --- |
| Deskew | none / gyro-only / full SE(2) wheel+gyro | Is explicit beam timing correcting motion distortion? |
| Fusion | LiDAR-only / LiDAR+wheel / LiDAR+wheel+IMU | Does each sensor add stability or accuracy on **supported** datasets? |
| Scan matching | Ceres local optimization / coarse+fine or alternative baseline | Convergence basin, speed, failure/degeneracy trade-offs. |
| Mapping | Ceres frontend + GTSAM batch / incremental online | Offline refinement benefit vs live resource cost. |
| Localization | Ceres map tracker / AMCL or Beluga baseline | Stable tracking, ambiguous cases, kidnapping recovery. |
| Platform | Raspberry Pi 5 ARM64 / experimental x86-64 | Target-architecture efficiency; never compare unlike workloads. |

Use exactly the same input data for variants. If gyro is used to deskew LiDAR and also appears as a GTSAM factor, report that shared conditioning and avoid inventing statistically independent duplicate priors. Evaluate live localization on a frozen map; mapping and evaluation traversals should be separated where possible.

**Benchmark acceptance is evidence-driven:** numeric thresholds for ATE, latency, and recovery will be set after the first validated fixtures/hardware run, not guessed from architecture proposals.

## 10. Docker, CLI, automation and Web Lab

The implementation contract's Docker-first rule applies: dataset preparation, conversion, tests, replay, benchmark scoring, and Web Lab backend MUST run in containers. The base C++20 core MUST NOT depend on ROS messages; use a separately pinned ROS adapter/test image for bag conversion.

The following commands are **proposed API shapes**, not runnable yet:

~~~sh
# Always-available synthetic fixtures; no network:
docker compose run --rm test ctest --output-on-failure

# Opt-in real dataset provisioning after manifest verification:
docker compose run --rm dataset-tool \
  prepare --dataset rosbot-indoor-full --download --output /data/prepared

# Offline benchmark processing:
docker compose run --rm benchmark \
  run --dataset /data/prepared/synthetic-apartment-v1 \
  --config /config/offline-mapping.yaml --report /data/reports/apartment.json

# Summarize comparable recorded runs:
docker compose run --rm benchmark \
  compare --reports /data/reports --output /data/reports/summary.json
~~~

When implemented, cxxopts MUST be used for C++ application CLIs. The benchmark runner MUST provide `--help` and predictable nonzero exit status for failed validation/measurements. A container must not download datasets implicitly at startup; ordinary tests work offline.

### Web Lab benchmark explorer

The optional Web Lab SHOULD present:

- Dataset selector with **synthetic / real**, sensor presence, ground-truth class, validation state, download size, and provenance.
- Map and scan overlays, raw vs gyro-only vs full SE(2) deskew, wheel/gyro trajectory, Ceres output, GTSAM optimized trajectory, and optional ground truth.
- Acquisition-time playback, pause, step-by-scan, seek and rate control with correct reset/replay semantics.
- Experiment parameters, side-by-side saved result selection, residual / error/latency plots, localization state and dropped-sensor diagnostics.
- Exportable report including run IDs and SHA-256 dataset provenance, calibration/map version, configuration, software revision, and hardware metrics.

As in the implementation contract, the browser MUST display **the real C++ backend results**. JavaScript MAY render and interact with results but MUST NOT calculate a separate ground-truth SLAM pipeline. Benchmark runs and map edits must have isolated, versioned state.

## 11. Report format and release criteria

Every benchmark run MUST create machine-readable JSON (and optionally human-readable Markdown/plots) containing:

~~~json
{
  "schema_version": 1,
  "run_id": "example-only",
  "dataset_id": "synthetic-apartment-v1",
  "dataset_sha256": null,
  "dataset_manifest_version": 1,
  "ground_truth_class": "simulated_exact",
  "source_revision": "example-git-commit",
  "docker_image": "example-image-digest",
  "architecture": "arm64-or-x86_64",
  "mode": "offline_slam",
  "configuration_sha256": null,
  "seed": 42,
  "scan_count_processed": null,
  "scan_count_rejected": null,
  "metrics": {
    "ate_translation_rmse_m": null,
    "rpe_translation_rmse_m": null,
    "rpe_yaw_rmse_rad": null,
    "scan_match_p95_ms": null,
    "capture_to_pose_p95_ms": null
  },
  "unavailable_reasons": {},
  "status": "example_not_executed"
}
~~~

The example deliberately uses `null` for **unmeasured** fields: it is a reporting contract, **not fabricated benchmark results**. Include raw/processed dataset checksums, versions of all converters, topic remaps, frame transforms, metrics definition version, errors, and dependency/toolchain details in the final production schema.

### 11.1 Acceptance and delivery phases

**D0 — Dataset architecture / first fixtures:** Commit scenario definitions, deterministic generator, compact truth/scan/IMU/wheel fixtures, manifest schema and Catch2 tests. Runs offline and completely within Docker. No external dataset required.

**D1 — Compact public baseline:** Verify and pin the 38 MB Cartographer bag; implement horizontal-LiDAR/IMU import and standalone LiDAR–IMU smoke replay. Do not call it a full 3-sensor benchmark.

**D2 — Full real-world fusion:** Verify and pin ROSBOT indoor/full; import wheel-derived odometry, gyro and 2D scans. Select a manageable MIT Stata session as cross-check; preserve synchronization/reference limitations.

**D3 — Benchmark evaluator / Web Lab:** Comparable error and resource reports, replay and overlays, experiment provenance and no-ground-truth scoring safeguards. Run on Pi 5 and x86-64 with the same inputs.

**D4 — Independent accuracy:** Select **one** IILABS sequence/sensor subset with OptiTrack truth, verify actual storage footprint, frames, timing and calibration; add quantitative held-out validation.

**D5 — Own hardware:** Record LD19 + MPU6050 + encoder datasets with the project's own recorder. Characterize the packet-reference physical delay and time-source uncertainty; verify OD-003 clock alignment, beam timing, rollover and real capture-to-pose latency in the deployment Docker runtime.

The synthetic fixtures are mandatory for development; external/public downloads are **opt-in and never a prerequisite for normal unit tests**. If real-data access is unavailable, that milestone is reported as pending rather than substituting invented samples.

## 12. Dataset attribution and reference links

Dataset licenses and the right to redistribute derived artifacts MUST be reviewed for each pinned recording. Cite original authors in benchmark reports and keep URLs and license decisions in the dataset manifest. The following are the **publisher/maintainer sources** for the candidate facts above:

1. **ROSBOT PLUS** — [dataset repository](https://github.com/MBarah10/indoor-outdoor-multisensor-robot-dataset); publisher-described indoor/full topics and duration. Actual download and byte count **not verified** in this specification.
2. **MIT Stata Center** — [downloads/topic table](https://projects.csail.mit.edu/stata/downloads.html), [project notices](https://projects.csail.mit.edu/stata/), and [ground-truth generation method](https://projects.csail.mit.edu/stata/groundtruth.html).
3. **Cartographer Deutsches Museum** — [public data index](https://google-cartographer-ros.readthedocs.io/en/latest/data.html), nominated `b0-2014-07-11-10-58-16.bag` (149 s, 38 MB).
4. **IILABS** — [dataset, sensors, custom encoder topic, ground truth, and selective-download instructions](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/).
5. **Relevant conventions** — [ROS REP-103](https://reps.openrobotics.org/rep-0103/), [REP-105](https://reps.openrobotics.org/rep-0105/), [MCAP](https://mcap.dev/).

**Re-evaluate URLs, license status, upstream topic metadata, bytes and checksums before implementing the dataset registry.** Public online descriptions are not substitutes for inspecting the actual recording.
