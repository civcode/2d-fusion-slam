# 2D Fusion SLAM — Benchmarks and Datasets

**Status:** Proposed / dataset candidates not yet provisioned  
**Version:** 0.1  
**Date:** 2026-10-30  
**Related:** [Technical Specification](technical-specification.md) · [Implementation Contract](implementation-contract.md)

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
| Reference | Exact `T_map_base(t)` | Can be evaluated at each beam's true acquisition timestamp and at the temporal scan midpoint, independently of the estimated trajectory. |
| Time origin | Explicit integer/monotonic | Deterministic timestamp ordering; no dependence on host wall clock. |

**Generator algorithm:** ray cast against a known planar wall/obstacle model at each simulated beam time `t_i` using `T_map_base(t_i) * T_base_laser`. Compute the scan reference as **`t_mid = t_start + floor((t_end - t_start)/2)`**, where bounds come from time-valid acquired beam slots even when their range measurements are invalid. Sample angle, range, optional intensity/validity, and scan reference time. Generate gyro output as true body yaw rate plus bias/noise and wheel output using the selected differential-drive or Ackermann model and its error settings. Include defined missing packets and nonuniform scan angular speed only in separate robustness presets.

Required test presets:

1. **Ideal:** zero noise, known calibration, exact timestamps, deterministic point/trajectory alignment.
2. **Motion distortion:** fast turn + translation within a revolution; compare no deskew, gyro-only deskew, and full wheel+gyro SE(2) deskew.
3. **Bias and slip:** gyro bias drift plus independent wheel-scale/slip errors; check uncertainty and correction logic.
4. **Ambiguity:** repeating warehouse aisles, corridor degeneracy, false loop candidate, kidnapping/relocalization.
5. **Faults:** missing IMU packet, timestamp wrap, corrupted LiDAR packet, time offset, outliers/dynamic obstacles.

LD19 **raw 47-byte packet parsing** MUST have its own deterministic protocol fixture tests (CRC, timestamp rollover, angular convention). Under **accepted OD-003**, add a synthetic packet/transport generator with known **angle/speed-derived beam acquisition times**, an independent configurable **packet-reference delay**, modulo-30-second source ticks, a distinct host-monotonic receive clock, injected drift/latency/jitter and variable motor speed. Verify relative timing against exact simulated truth and report **clock alignment combined-offset estimates**, not an invented independent physical-delay estimate. Generating ideal `LaserScan`-like beams alone **does not** validate the real LD19 timing behavior.

The real LD19 physical packet-reference event remains unverified: test an actual sensor across controlled rotations/translations, host buffering and device restarts to **determine or bound** the acquisition-to-reported-timestamp delay before treating absolute timestamps as calibrated. Preserve original source bytes/ticks, Pi receive times, timing quality, model parameters and calibration revision in each prepared dataset.

Small canonical fixture files MAY be committed when redistribution is allowed and their total size remains practical; keep full generated data out of Git.

### 4.1 Accepted OD-004 trajectory and deskew fixture requirements

The generator MUST produce continuous **SE(2) ground-truth** constant-body-twist segments with controlled wheel/gyro measurements; the first estimator implementation reconstructs pose from these streams using the **SE(2) exponential**, not independent linear x/y/yaw interpolation. Preserve the exact truth at every beam time and the scan's **temporal midpoint**.

The synthetic apartment and warehouse fixtures MUST include at least:

1. Pure translation, in-place yaw, constant-curvature turning, and nonzero LiDAR-to-base translation, with SE(2) exponential/small-angle numerical tests.
2. Asynchronous encoder and gyro timestamps, nonuniform LD19 beam times, odd nanosecond scan durations, invalid range values that retain valid acquisition timing, and source-clock/epoch changes.
3. A **complete, quality-valid interpolation interval** covering the scan bounds and midpoint. Only this case qualifies for **ordinary map insertion**; assert correct midpoint world geometry and that raw timestamps remain unchanged.
4. Missing/reordered encoder/gyro data, ambiguous acquisition interval and incomplete trajectory coverage. A bounded delayed-processing run must either obtain valid coverage or mark the scan deferred/rejected, **never fill missing motion with silent extrapolation**.
5. Optional live prediction at **9.9 ms**, **10 ms** and **10.1 ms** beyond a valid trajectory boundary. The first two are allowed **only when quality gates pass**, while the last is always rejected under the initial 10 ms ceiling. Confirm `EXTRAPOLATED` status, covariance growth, domain/epoch checks and **no extrapolated ordinary map insertion**.
6. A mock or alternative trajectory provider feeding the same deskew tests without LD19, MPU6050, wheel encoder, Pico or ROS libraries. Future spline benchmarking uses the identical public trajectory interface.

For all B03/B12 results, publish endpoint error against exact synthetic truth, trajectory coverage fraction, deferred/rejected scans, uncertainty/error calibration, interpolation/extrapolation counts, and p50/p95/p99 scan processing/processing-delay latency. **Do not confuse frame time `t_mid` with the time at which computation finished.** Run tests and benchmarks in Docker; Pi 5 timing must be measured on actual target hardware.

### 4.2 Accepted OD-005 scan-assembly and safe occupancy fixtures

The deterministic generator MUST support **LD19-like 12-beam packets that straddle an angular seam**, variable motor speed, 0°/configurable seam, forward/backward/raw clockwise direction, partial first/last revolutions, packet loss, incomplete sectors, corrupt packet CRC, and motor/clock discontinuity. Generate **exact observed beam acquisition times** and the true coverage of the simulated rotating sensor; never add a beam for missing sectors merely to complete a regular scan.

Fixture variants include:

1. Full 360° coverage with all valid hits, fixed seam crossing, and exactly-once beam assignment across packets.
2. A **partial scan with adequate distributed valid hits** that passes configurable match/insertion gates; verify OD-004 midpoint equals the **actual observed** acquisition-time midpoint and full interpolated trajectory coverage enables insertion.
3. **Insufficient angular support or a large missing wedge**: verify calculated **circular** angular coverage and that matching/insertion gate rejection reasons are correct even when minimum and maximum observed angles appear nearly a full revolution apart.
4. Mixed **VALID_HIT**, **VALID_NO_RETURN**, **INVALID_RETURN** and **MISSING** data. Under all invalid/unknown/missing cases, assert **exactly zero free-space and occupied updates**; only explicitly validated, enabled no-return policy may produce bounded free rays. Preserve intensity/status and time-valid invalid-range slots.
5. Dropped packets across the seam, ambiguous gaps spanning multiple possible revolutions, sensor reset and out-of-order packets: no phantom scans, fake beam timestamps, inferred occupied endpoints or clearing.
6. Stalled acquisition watchdog event: marked partial/timeout recovery. In nominal continuous acquisition, verify **only angular seam crossings** delimit revolutions, independent of wall-clock timeouts.
7. Repeat the same scenarios with changing packet/beam reception schedules and ensure identical assembly result given the same ordered acquisition samples. Track latency, packet loss, partial-scan acceptance and mapping error.

Publish B13 outputs: **observed angular coverage fraction, largest missing sector (degrees), missing intervals, valid/invalid/no-return counts, accepted vs rejected partial scans, match/insertion eligibility reasons, malformed-packet count, watchdog recoveries, and free/occupied writes due to invalid/missing measurements (must be zero)**. Include Docker test image/version and report profile thresholds; measure Raspberry Pi 5 impact when real LD19 recordings are available. The Web Lab SHOULD render coverage and missing sectors distinctly, never as inferred free space.

### 4.3 Accepted OD-006 wheel/gyro motion and bias fixtures

Provide a deterministic **minimal wheel-translation + gyro-yaw integrator** fixture with exact SE(2) trajectory truth, configurable body-frame twist, true gyro Z bias, bias random walk/temperature variation, gyro measurement noise, wheel scale/slip/encoder quantization, timing errors and valid/invalid motion evidence. Generate **measured** wheel/steering inputs according to an explicitly selected platform model; do not label steering commands or already-fused odometry as independent raw encoder evidence.

Required cases:

1. **Nominal:** Known wheel translation, zero/constant yaw and constant-curvature motion with asynchronous gyro/wheel samples; compare reconstructed SE(2) path against truth and OD-004 midpoint deskew without Ceres or GTSAM.
2. **Kinematic selection:** Synthetic differential-drive **only with independently measured left/right wheels**; Ackermann **only with measured steering** and calibrated wheelbase; unknown/unconfigured drivetrain and missing steering MUST fail or degrade explicitly, never silently default to differential drive.
3. **Stationary calibration:** Inject known fixed Z-rate bias and noise over a verified stationary startup window; measure bias mean error, estimated variance and accepted-calibration rate. Moving startup, insufficient samples or ambiguous stationary evidence MUST reject a false trusted bias.
4. **Gated online bias:** Separate true stationary bias drift from moving robot and wheel-slip cases. Confirm allowed bounded corrections, rejected updates during motion/slip, logged revision changes and continuous previously published odometry.
5. **Slip/disagreement:** Inject asymmetric wheel slip, encoder scale mismatch and gyro bias separately. When independent wheel yaw exists, verify covariance-aware wheel/gyro disagreement and increased uncertainty; when absent, report the monitor unavailable. Do not turn slip into a bias update.
6. **Uncertainty and data gaps:** Compare pose/yaw/bias covariance and cross-covariances with Monte Carlo exact simulated truth, long bias drift intervals, dropped wheel/gyro messages, bad clock epochs and the OD-004 10 ms extrapolation limit. Missing observations must not be interpreted as measured zero motion.
7. **Architecture/performance:** Run the same dataset through the minimal integrator and a mock alternative provider with the same timestamped trajectory interface (future EKF/smoother candidate); ensure generic C++20 core works without ROS/drivers, and record Raspberry Pi 5 p50/p95/p99 integration/deskew latency, CPU and memory.

**Report** bias estimation error and readiness fraction, integration RPE/ATE **only with exact synthetic or independent ground truth**, yaw drift, slip detection precision/recall where labeled, covariance-vs-error consistency, missing-data coverage, and estimator time/resource cost. Each result must record kinematic model/geometry, sensor input provenance, configured bias/slip/noise parameters, algorithm revision, seed and Docker image. The ROSBOT, MIT and Cartographer datasets support **different measured-source and ground-truth claims**: a bag with only `/odom` is not automatically a pair of raw wheel encoders.

### 4.4 Accepted OD-007 EDT/Ceres objective fixture and controlled ablations

The initial Ceres scan-matching benchmark MUST use a **known authoritative occupancy grid**, derive a **truncated unsigned EDT from confidently occupied cells**, and match **OD-004 midpoint-deskewed, OD-005 valid-hit points** using per-beam independent Huber robustification, bilinear interpolation, an OD-006 uncertainty-weighted pose prior and explicit out-of-map boundary penalties. Use the synthetic apartment and warehouse as exact geometry/trajectory references; keep raw beams, prior and map revisions identical across variants.

Required independent test conditions:

1. **EDT metric:** Known single wall/point/parallel-wall grids, valid occupied confidence thresholds, Euclidean metre distances, truncation at `D_max`, empty occupied map, shifted/rotated origin and cell-centered ROS-compatible indexing. The EDT cache must identify the immutable source occupancy revision/checksum and unknown/validity mask.
2. **Interpolation:** Bilinear values within sampled extrema, finite gradient on cell interiors, expected gradient changes at cell boundaries, no silent interpolation across unknown stencils or missing support. Compare bicubic **only as an optional later ablation**, inspecting overshoot and Pi cost.
3. **Per-beam Huber:** Inject one extreme range outlier among clean endpoints. Assert that separate scalar Ceres `HuberLoss` residual blocks reduce **the outlier independently**, unlike a combined single-block Huber. Confirm residual `sqrt(w_i) * D / sigma_i` and loss scaling rather than double-squaring the distance.
4. **Odometry prior:** Compare no prior, calibrated weak prior, calibrated tight prior and deliberately wrong prior. Check whitening with SE(2) Lie Log and prediction covariance, distinct LiDAR/prior costs and acceptance results. A tight prior may stabilize pose but MUST NOT falsely improve **LiDAR-only observability** or turn a wrong geometrical match into a trusted correction.
5. **Unknown and boundary:** Place endpoints within confidently observed map, unknown interior cells and outside the rotated grid; require separate classification, **finite outward-increasing boundary gradient**, rejection of unsupported/boundary-dominated matches and no unknown-as-free/obstacle assumption.
6. **Overlap/degeneracy:** Observe long corridor, sparse angular support, repetitive warehouse shelves, symmetric false minima, low inliers, dynamic obstacles, saturated EDT plateau and high residual. Verify gate reasons, LiDAR-only geometric Hessian eigenvalues and false-lock rates independent of odometry-prior strength.
7. **Graph information accounting:** Store prior source wheel/gyro IDs and `Sigma_pred`, LiDAR-only score/information, posterior pose, source occupancy/EDT revision, boundary cost and acceptance. Assert that passing the prior-regularized pose to GTSAM **as an independent LiDAR-only factor** is disallowed in the integration contract; compare acceptable factor extraction strategies under OD-008/013 later.
8. **Backend and resource boundary:** Match with a generic native `ScanMatcher` interface and private Ceres target, ensuring public C++20 headers build without Ceres and ROS. Measure p50/p95/p99 latency, convergence/reject reasons, CPU/peak RSS on x86-64 and Pi 5 when present.

**Report B15:** scan midpoint reference, dataset/map/EDT/calibration/config revisions; number of valid/masked/unknown/out-of-grid/informative beams; raw and Huber LiDAR-only residual distributions; prior and boundary costs; posterior pose, geometric information eigenvalues and rejection reasons; ground-truth translation/yaw error only when true reference exists; solve iterations and latency. Record `D_max`, Huber δ, confidence threshold, `w_i`/`sigma_i` policy, prior covariance scale, boundary scale, interpolation mode and solver budget. Separate **LiDAR-only / with-prior / no-boundary** ablations; do not compare unlike datasets or conflate small total cost with correct localization.

### 4.5 Accepted OD-008 LiDAR-only information, degeneracy and uncertainty-calibration fixtures

Build **benchmark B16** on the versioned OD-007 authoritative occupancy/EDT maps and midpoint-deskewed scans, with deliberately **separated LiDAR geometric residuals**, OD-006 prior residuals and nonmeasurement boundary penalties. The evaluator must measure trustworthiness of both **accepted and rejected** poses, not only report Ceres-converged fits.

**Exact synthetic and held-out reference cases:**

1. **Known-information algebra:** Construct 1×3 residual Jacobians with known **rank 0, 1, 2 and 3**, normalized per-beam confidence already absorbed in `J_i`, Huber influence `alpha_i=rho'(r_i²)` and outliers. Verify the LiDAR-only `H_L=sum_i alpha_i J_i^T J_i` is PSD, symmetric and does not double-weight beam confidence or add prior/boundary information.
2. **Numerical Ceres comparison:** Compare independent LiDAR-only analytic/automatic-differentiation Jacobians to finite differences in the correct SE(2) tangent basis. Verify `rho'` robust-influence approximation versus `rho''` solver curvature and optional Ceres covariance cross-check; neither is treated as independently calibrated covariance.
3. **Physical-axis observability:** Use documented `S` scales for metres and radians, decompose `S^T H_L S`, report eigenvalues, rank, condition and observable/null bases. Rotate reference frames and verify the transformed geometric directions and covariance. Rank must be **unchanged** when the OD-007 odometry prior is tightened.
4. **Corridor / open room / corner:** Test corridor along-axis translation degeneracy, low-information open area, observable corner and mixed/partial scan geometry. A converged tight-prior result may be accurate but **must not** claim LiDAR-only information where it is absent. The evaluator must project only supported components and recheck quality **or reject**, never report a falsely finite 3D covariance.
5. **Warehouse repeated structure:** Include two nearly equal geometric local minima, a deliberately wrong odometry prediction, dynamic obstacle/outlier beams, poor map overlap and heavily truncated EDT samples. Measure false acceptance/rejection, solver failure, null-space projection quality and independent information rank.
6. **Calibration training/test split:** Select `K`, PSD `Sigma_floor` and quality/innovation/rank thresholds on deterministic **calibration** traversals/scenes, then evaluate **different held-out** synthetic traversals and independently referenced real paths where available. Prevent using the same mapping trajectory/reference to calibrate and claim independent accuracy. Store calibration ID, thresholds, dataset/source hashes, scenario, seed and fitted uncertainty policy.
7. **Uncertainty consistency:** For fully observable rank-3 fits, compare reported 3×3 covariance with exact/independent position-yaw errors: normalized innovation/estimation error (including NEES when valid), empirical confidence-region coverage, median/p95 error, overconfidence and underconfidence. For rank<3, report rank and observable-space uncertainty **only**, null directions unavailable, projected correction accuracy and rejection rates—not a fabricated full-rank NEES.
8. **Sensor-quality variation:** Move at slow and fast speeds, inject LD19 clock drift/beam-time uncertainty (OD-003), 10 ms OD-004 prediction/gap limits, asymmetric encoder slip and gyro-bias errors (OD-006), poor OD-005 angular coverage and changing map/EDT revisions. Verify quality gating and covariance response.
9. **Offline resampling:** Optional bootstrap/perturbation sensitivity study to cross-check robust information; preserve beam correlation and map/trajectory dependence. Do not add bootstrap to the initial Pi 5 online pipeline.
10. **Module and target performance:** Ceres-linked solver and **Ceres-free MatchQualityEvaluator** each have Docker Catch2 tests. Measure p50/p95/p99 evaluation overhead and total scan latency on x86-64 and Pi 5 when available. ROS-independent native result serialization must retain rank, tangent scales, axes, uncertainty status and reason codes for Web Lab and optional ROS adapters.

**Report B16:** source map/calibration/clock/EDT/software revisions, beam counts and observed-map overlap, LiDAR-only robust residual and Jacobian information, `H_L` and normalized eigenspectrum/rank, observable and null directions, convergence and innovation, full-rank calibrated covariance **or** observable-only uncertainty, acceptance/projection/rejection rationale, independent reference truth class, confidence coverage/false-lock rate and per-stage CPU/latency. Compare unweighted/raw, robust-only, robust+calibrated, and ablated observability gates using identical inputs. Explicitly differentiate **exact synthetic**, **independent measured ground truth**, **LiDAR-derived reference** and **unavailable truth**; only the first two can establish absolute uncertainty calibration.

### 4.6 Accepted OD-009 correlative-search and multi-resolution BnB fixtures

**Benchmark B17** uses the small exact-truth synthetic apartment and repetitive warehouse with **immutable occupancy-derived correlation score pyramids**, OD-004 midpoint-deskewed/OD-005 quality-accepted scans, OD-006 covariance hypotheses, OD-007 Ceres refinement and OD-008 quality/rank gating. Search is a **candidate generator**; measure actual accepted global pose rather than equating the highest coarse score with truth.

**Test matrix:**

1. **Normal healthy tracking:** OD-006 correct/high-confidence prediction -> direct Ceres with **zero coarse-search work** (no search-tree build/scan-time search); verify localization performance and the 10 Hz scan path.
2. **Degraded search:** Position offsets near **±0.3 m**, yaw near **±10°**, 10–20 ms proposed Pi search budget; vary covariance eigenstructure, clip windows, and record clipping/failures when truth is outside the cap.
3. **Recovery search:** Offsets near **±1.0 m**, yaw **±30°**, 30–50 ms proposed per-invocation budget; compare complete/partial multiresolution BnB and OD-007 refined poses against ground truth.
4. **Global kidnapped robot:** Full eligible frozen submap(s) and **360°** yaw, asynchronous **100–200 ms processing slices** with resumable/cancelled/incomplete progress. Measure total processing time to true stable relock and number of slices; a per-slice budget does **not** imply full-map completion in that time.
5. **Admissible pruning:** Small exact synthetic grids with known/support-mask regions, multiple map origins including yaw and repeated shelves. Compare upper bounds, candidate scores and best completed BnB result **against exhaustive discrete fine-resolution search**; test unknown/outside cases and score quantization. An inadmissible heuristic cannot claim exhaustive branch-and-bound correctness.
6. **Angular discretization:** Derive yaw increments from grid resolution and valid beam range. Sweep short/long beam scenes and angle seams; ensure candidate sampling is fine enough to find true peaks without excessive candidate count.
7. **Multiple hypotheses:** Ambiguous warehouse aisles, apartment symmetry and open corridor. Keep spatially separated top-K candidates (nonmaximum suppression); measure recall@K, distinct peaks, false-lock rate and multi-scan confirmation, not just rank-1 coarse score.
8. **Prior and OD-008 gates:** Deliberately **wrong, overconfident** OD-006 prior at a distant true pose; compare stale tracking-prior Ceres vs mode-relaxed/no-prior recovery refinement. High coarse correlation or low posterior cost MUST NOT bypass OD-008 overlap/residual/observability, state-aware innovation or OD-020/022 multi-scan relock.
9. **Resource and concurrency:** Feed deterministic **10 Hz** input/OD-006 integration while async BnB runs; vary CPU load/map size/pyramid depth/cache reuse and verify no sensor acquisition or recording blockage. Measure p50/p95/p99 scan-path latency, CPU/RAM/thermal, per-slice budget overshoot, node expansions/pruning, queue lag, cancellation latency, stale map revision handling and no map insertion while LOST.
10. **Map/version consistency:** Compare immutable pyramids built for occupancy versions A/B; reuse version A across identical scans, invalidate when authoritative map/unknown-mask changes, and confirm candidates cannot mix occupancy/EDT/clock/calibration revisions.

**Comparison and metrics:** OD-007 direct-Ceres-only, bounded local correlation + Ceres, full multiresolution BnB + Ceres and optional future persistent-particle baseline under OD-020, **using identical sensor/map splits**. Report search mode/window/covariance, angular step, map/pyramid revisions, search nodes/candidate coverage, bound admissibility, complete/partial/cancel state, candidate count and recall@K, Ceres/OD-008 acceptance, true location error only with **exact synthetic or independent measured truth**, time to robust relock, map insertion suppression and full CPU/latency cost.

**Measured performance required before fixing defaults:** Proposed 0/10–20/30–50/100–200 ms budgets are **initial Pi 5 targets only**; collect on-device benchmark results and classify any unmet timing targets honestly. No external large dataset is needed for B17's correctness fixture. Keep all automated tests inside Docker and no GitHub Actions execution.

### 4.7 Accepted OD-010 log-odds occupancy and overlapping frozen-map fixtures

**Benchmark B18** uses the synthetic apartment and warehouse exact 2D geometry and OD-003–005 timed/quality-assessed scans to test the **authoritative 0.05 m log-odds occupancy grid**, two overlapping submap lifecycle, probabilistic hit/free rays, motion filter, memory bounds and frozen immutability. OD-007 EDT and OD-009 correlation pyramids are **derived versioned caches** of the same occupancy source.

**Required cases and observations:**

1. **Deterministic scan-ID lifecycle:** Feed at least 280 accepted eligible scans; assert A receives IDs **1–180**, B IDs **91–270**, C starts at **181**, each submap receives a scan ID at most once, and two writable maps maximum. Validate exactly 90 overlapping IDs for nominal complete adjacent maps. Check boundary ordering **after** scan 90 and **after** scan 180 and first/last partial maps. A discarded/untrusted scan must not increment *any* insertion count.
2. **Empty-map bootstrap:** Start with an empty authoritative occupancy raster and no usable EDT occupied cells. Verify that the first geometrically eligible scans can seed a declared submap-local gauge using trusted OD-006 trajectory **without pretending Ceres provided a match**. Mark insertion state `BOOTSTRAP_UNVERIFIED_MAP_MATCH`, then transition to OD-007/008 pre-insertion geometric matching once map support becomes informative; a later LOST state must not restart bootstrap to write unsafe occupancy. Confirm a new overlapping submap inherits an already tracked pose frame.
3. **Filter and quality:** Produce stationary near-duplicate scans, pure rotations, varied translations, a delayed/low-quality scan and false localization. Configure documented translation/yaw/acquisition-time motion filter thresholds. Assert skipped scans are still recorded and OD-006 motion advances but submap counters/cells remain unchanged. Report accepted/rejected/filtered counts and reasons.
4. **Probability and unknown:** Check `logit(0.55)` and `logit(0.49)` log-odds increments, `logit(0.10)`/`logit(0.90)` clamping, odds/probability round trips, and the distinction between untouched **UNKNOWN** `p=0.50` and **observed-known-neutral** `p≈0.50` after contradictory observations.
5. **Ray and duplicate semantics:** Cast crossing hits, duplicate beams and rays that traverse other hits in one scan. For each scan/submap, enforce unique hit/free cell sets, **hits win**, and each cell changes **at most once**. Valid hits produce occupied endpoints and free cells strictly before; OD-005 `INVALID_RETURN`/`MISSING` (including missing wedges) produce **zero map updates**. Explicit validated no-return policy is off initially and, when enabled, may clear bounded ray only without a fake hit.
6. **Geometry/resolution:** Repeat same truth scene at **0.025 m, 0.05 m and 0.10 m**, with rotated/translating local submap origins and calibrated laser lever arm. Quantify obstacle-boundary error, structural consistency, free/occupied/unknown precision against truth, connected-wall artifacts and number of allocated cells/RAM. For equal physical area, note expected **~4×/~¼** cell count vs 5 cm, not a guarantee of end-to-end memory/cost ratio.
7. **Memory and spatial limits:** Force cell cap, extent cap, allocator refusal and oversized ray. Require bounded RAM, early freeze with **actual insertion count** or safe insertion stop, no fictitious boundary obstacle/free ray, no unbounded growth and no blocking of raw sensor acquisition. Record peak RSS, allocated cells, map footprint, early stop reason and reduced overlap when needed.
8. **Freezing and graph moves:** Hash local occupancy/known mask before/after freezing and after multiple simulated GTSAM `T_map_submap` global pose corrections. Source hash and local EDT/correlation caches remain unchanged; separately versioned global pose/spatial indexes change. Readers cannot see partially mutated active maps; a published new active snapshot yields new content/cache revision.
9. **Replay and consistency:** Replay identical scans/accepted flags through the same filter/config under different packet arrival schedules. Compare local map hashes, per-submap scan ID lists, lifecycle transitions, known masks, cache inputs and serialized/loaded maps. No map self-matching. Keep dynamic aging **disabled** on frozen maps; optional transient overlay must not change frozen source hashes.
10. **Pi 5 runtime:** Measure occupancy insertion p50/p95/p99, two-active-submap CPU and peak RAM, snapshot/freeze and derived-cache build cost, scan gating load, tile allocation rate and sustained ~10 Hz acquisition under pressure. x86 results cannot substitute for hardware profiling; no GitHub Actions usage.

**Configuration sweep:** Hold map-building and evaluation traversals distinct, vary **grid resolution (2.5/5/10 cm)**, hit/miss probabilities **(0.55/0.49 starting point)**, probability clamp **(0.10–0.90 starting point)**, new/freeze insertions **(90/180 starting point)**, motion filter thresholds, confidence-to-occupied threshold, physical map cap and robot motion. Label them as **unvalidated initial settings** and report parameter source. Do not use ground truth from the evaluation path to tune the mapping run.

**Report B18:** source dataset and truth class, map/submap local frame, grid resolution/origin, configured and *actual* accepted insertion counts, scan ID intersection/overlap by count, known/free/occupied cells, hit/miss/invalid/no-return counters, unique-cell write counts, motion-filter skips, map boundary/coverage error where exact ground truth exists, freeze/graph pose/cache revision IDs, early cap reasons, CPU/memory/latency. Include **zero invalid/missing ray updates** and **zero duplicate per-insertion cell writes** as mandatory regression invariants. If no independent truth exists, report geometric consistency rather than invented absolute map accuracy.

### 4.8 Accepted OD-011 snapshot, exact EDT tiles and frozen cache fixtures

**Benchmark B19** measures correctness, consistency and scheduling of immutable OD-010 occupancy-derived **OD-007 truncated unsigned EDT** and **OD-009 correlative-score pyramids**. Compare the **first implementation's complete full-field rebuild on a bounded refresh cadence** with the **target 32×32 dirty-tile active EDT** under identical source revisions and configs. Cached frozen fields should be reusable across scans and global pose changes; benchmark on-demand evictions/reloads separately from the online scan loop.

**Initial fixture settings:** 0.05 m/cell, unsigned EDT `R=0.30 m`, 32×32 interior cells (**1.6×1.6 m**), source halo **≥0.30 m + interpolation support** (≥6 cells plus at least one interpolation stencil cell at 0.05 m). Also test resolution 0.025/0.10 m, different map origins/rotations and alternative radii. **These parameters are not yet hardware-tuned**; a 40 m × 40 m map contains **640,000 cells** and one float32 field takes **2.56 MB decimal / ~2.44 MiB**, excluding all other caches/masks/versions.

**Mandatory cases:**

1. **Full EDT reference:** Generate occupancy grids with exact single-wall, points, corridors, dense warehouse geometry and observed/unknown masks. Produce a complete exact Euclidean distance transform, truncate at `R` and verify known nearest-obstacle distances (with empty occupied map and edge cases). Felzenszwalb–Huttenlocher is a suitable exact linear-time reference.
2. **Occupied-source additions and removals:** Change confident occupied cells into/out of threshold, including several updates within one insertion, moving obstacles, repeated updates and sources near tile edges. Recompute only dirty tiles using a sufficiently expanded source neighborhood and confirm **identical truncated output** versus the full reference, within numeric tolerance; this must hold **after removal of the previous nearest obstacle**.
3. **Support masks and interpolation:** Change known↔unknown status without changing the occupied source set; test EDT mask/stencil readiness, bilinear samples/derivatives and correct invalidation at tile boundaries and map edges. Never substitute unsupported unknown cells for clear rays or zero-distance obstacles.
4. **Separate OD-009 scores:** Change occupancy probability **without crossing the EDT occupied threshold**, including score decreases and known-mask transitions. Verify pyramid refresh and **admissible upper bounds against the identical fine-level score**, checked against exhaustive small-map enumeration. A clean EDT does **not** imply a current pyramid.
5. **Snapshot coherence:** Insert a new scan into active occupancy while the Ceres matcher pins snapshot N and a BnB worker pins another complete revision. Publish N+1 atomically with matching EDT/pyramid source IDs/config hashes; assert **no torn tile, no mixed versions and no use-after-free**. Explicitly test overdue worker completion that must not overwrite newer ready snapshots.
6. **Staleness gates:** Delay the background rebuild. Match on a previous complete map within a configurable maximum **host-steady age AND accepted insertion lag** while reporting degraded freshness, then exceed either cap and verify `STALE`/defer/reject. No map self-matching, silent latest-version claim or blocking of sensor recording.
7. **Frozen and graph pose cache:** Freeze local occupancy and precompute both derived fields; update global `T_map_submap` repeatedly and assert the local source hash/fields are unchanged. Test unpinned least-recently-used eviction, still-pinned reader survival, wrong-cache-config hash invalidation, on-demand reload/reconstruction and cache memory ceiling.
8. **Oversized dirty region/fallback:** Force source changes over many tiles, map growth/recentering, wrong resolution/origin and very large halo. Verify safe **full rebuild fallback**, bounded scratch and no partially published output.
9. **Scheduling and Pi resources:** Feed a deterministic **~10 Hz** LD19/IMU/encoder recording stream while active/frozen caches update, BnB searches and frozen loads execute. Report **p50/p95/p99** per scan matching, EDT rebuild/tile work, pyramid build, worker queue lag, pinned snapshot age, cache hit/miss/eviction, CPU/peak RSS and timestamp jitter. Exceed resource caps and verify acquisition/recording/OD-006 prediction continue unblocked.
10. **Portable API and replay:** The `MapSnapshotProvider` and cache builder compile and test **without ROS/Ceres public headers or drivers**. Replay with identical accepted insertion IDs but altered wall-clock scheduling must yield the same semantic occupancy/EDT/pyramid hashes (and documented deterministic publication policy) even when latency/cache hit statistics differ.

**Comparison protocol:** Baseline full exact EDT at a bounded cadence; on-demand full rebuild; tiled EDT with 32-cell or alternative tile widths; frozen cache eviction strategies; score pyramid full versus incremental rebuilds. Compare under **identical authoritative occupancy revisions, masks, configuration and benchmark seeds**, not only average cost. Correctness and consistency gates must pass **before** considering the tiled approach a performance improvement.

**B19 report:** map and grid/known-mask revisions, source hash and EDT/pyramid configuration hashes, build strategy, tile size/halo, dirty tiles, exact-vs-tiled max/mean error and bilinear error, score-bound validity, snapshot publication/age/insertion lag, search/map readiness, cache reuse and memory pressure outcomes, full rebuild fallback frequency, per-stage latency and CPU/RAM, and measured impact on Pi acquisition. Do not claim deterministic scheduling times, real-time deadlines or validated incremental performance from x86-only figures.

### 4.9 Accepted OD-012 native bundle, cross-platform persistence and crash recovery fixtures

**Benchmark B20** evaluates **JSON manifest + independently Zstandard-compressed portable binary authoritative submaps/graph** in a versioned **directory generation**, with OD-011 EDT/correlation caches intentionally **optional and regenerable**. Use the exact synthetic apartment/warehouse OD-010 5 cm log-odds maps, observed masks and graph-pose/constraint reference. The comparison is **lossless authoritative state round-trip**, not only a visually similar PGM preview.

**Mandatory test cases:**

1. **Authoritative round-trip:** Save/read frozen submaps with known free, occupied, **unknown and observed-neutral (~p=0.50)** states, rotated local grid origins, 5/2.5/10 cm resolution, 90/180 insertion histories/overlap, graph nodes/poses/constraints with covariance/information and factor IDs, supported calibration and OD-001 clock session/epoch mapping. Verify source log-odds and known-bit content hashes, IDs, graph references and pose convention exactly.
2. **Manifest compatibility:** Major/minor schema and required/optional features, minimum reader version, invalid/missing fields, duplicate submap/graph IDs, reference to nonexistent file or graph node, unsupported mandatory codec, known optional extra field and explicit migration to **new generation** (old source remains usable). Distinguish **LOCALIZATION_MAP** and **RESUMABLE_CHECKPOINT**.
3. **Binary portability:** Golden fixed-width little-endian record vectors independent of C++ object layouts, padding, host `size_t` and Eigen/GTSAM memory. Decode on Docker x86-64 and ARM64 where available, with platform-independent field/unit ordering. Include negative/malformed lengths, integer overflow and nonfinite pose/covariance rejection.
4. **Compression and lazy submaps:** Each frozen submap is independently zstd-compressed and loadable by ID without decoding all other submap files. Measure file-count, compressed/uncompressed sizes, zstd-window and streaming-memory caps, first-map and later-submap p50/p95/p99 load time. A large/hostile compressed frame cannot trigger unbounded allocation.
5. **Hash and trust boundaries:** Corrupt required occupancy bytes, observed mask, graph or manifest index/size; expected result is **no published partial runtime map**, with explicit error or verified prior-generation fallback. Reject SHA-256 mismatch, truncation, missing files, unsupported required features, absolute/path traversal/symlink escape and duplicate file IDs. SHA-256 is an **integrity** check, not cryptographic source authentication.
6. **Optional caches:** Corrupt, remove or make version-incompatible an OD-007 EDT and OD-009 score pyramid; the authoritative map/graph still loads, cache is reported `STALE`/`NOT_READY` and rebuilt on OD-011's bounded worker. Change only a submap's **global graph pose** and confirm its unchanged local occupancy hash/EDT source validity.
7. **Atomic checkpoint injection:** Deliberately stop the writer after staged payload writes, per-file fsync, directory sync, generation rename, temporary `CURRENT` write/sync, pointer rename and generation garbage collection. The reader pins **one complete generation**, validates required data and either loads the most recent valid generation, recovers a **retained prior complete** one, or returns error. It must **never** mix graph N with submaps N+1 or declare a staged directory valid. Test disk-full, interrupted rename, insufficient space, filesystem-specific sync failures and directory existing/nonempty conditions; distinguish visibility atomicity from durability.
8. **Map versus resumable:** `LOCALIZATION_MAP` supports read-only frozen-map localization but MUST reject attempting to resume active mapping; `RESUMABLE_CHECKPOINT` includes active submap contents/counters, estimator gyro bias/covariance, graph continuation, replay cursor/source identity and clock-domain/session mapping. Simulate reboot: prior Pi monotonic clock number cannot be treated as a new session's event epoch.
9. **ROS adapter and declared loss:** Export/roundtrip conventional Nav2 YAML/image or `OccupancyGrid` through separate adapter. Produce a loss report for graph constraints, exact log odds, observed-neutral mask, clock/replay metadata and overlapping submaps; do not claim a lossy image can reconstruct native checkpoints. Native writer/reader must build/test **without ROS, Ceres, GTSAM object serialization or proprietary drivers**.
10. **Pi 5 stress/benchmark:** Generate many individually compressed submaps in realistic apartment/warehouse variants and measure save/checkpoint wall time, CPU, p95/p99 capture latency while saving, peak memory, compressed ratio, SD-card/SSD bytes written, lazy random submap access and OD-011 frozen cache rebuild cost. Bound work in background and confirm **10 Hz acquisition, IMU/encoder ingestion and raw recording remain unblocked**.

**Implementation comparison:** Compare **directory generations** with and without optional derived caches, compression levels, file count and independently loaded submaps. A single-file bundle and SQLite/WAL-based database are *deferred design alternatives*, not baseline implementations or required benchmark infrastructure. Rebuildable caches must never change authoritative hash verification.

**Report B20:** format major/minor and required features; map/generation ID, `bundle_kind` and source/clock/calibration IDs; required-file count, size, zstd ratio, SHA validation status and cache readiness; exact authoritative round-trip and graph reference check; lazy loading/memory; simulated power-loss phase and recovered generation; safety-limit rejections, Pi/x86-64 architecture and compression dependency version; save/load p50/p95/p99 and measured effect on acquisition. Mark untested filesystem durability conditions explicitly—**do not claim general crash-proof operation without real fault injection and supported sync semantics**.

### 4.10 Accepted OD-013 keyframe/submap graph and correlation-consistency fixtures

**Benchmark B21** validates a **GTSAM batch SE(2) graph of keyframe global poses `X_k` and submap global poses `S_j`**, with one OD-006 **fused wheel/gyro relative-motion constraint per keyframe interval**, one primary **LiDAR-only local keyframe-to-submap constraint** from separately validated OD-007/008 evidence and OD-018 candidate/OD-009 search/OD-008-verified inter-submap loops. The OD-010 overlapping submaps, OD-011 source revision and OD-012 portable graph factor schema must share stable identifiers. High-rate bias/deskew remain **outside** the graph initially.

**Exact-truth deterministic fixtures:**

1. **Graph topology and gauge:** Build known keyframe/submap poses on apartment/warehouse loops, with local submap frames deliberately rotated/translated relative to map. Verify `Z_jk=S_j^-1 X_k`, Lie residual/Jacobians and finite graph rank after exactly **one gauge reference**. Compare with a simple keyframe pose-only graph baseline using identical underlying constraints; ensure submap poses are not artificially pinned by duplicate anchor factors.
2. **One fused motion likelihood:** Generate wheel and gyro time-series and OD-006 interval preintegration. Admit **one fused SE(2) motion factor per keyframe interval**; intentionally add raw wheel, gyro yaw and fused factors covering the **same samples** and assert duplicate-evidence admission blocks them. Missing modalities must not produce false 3-DoF certainty. Track bias calibration revision and data intervals.
3. **OD-007 prior contamination:** Choose a deliberately wrong **overconfident** odometry prior and LiDAR geometry that implies a different pose. Verify that the posterior Ceres result **does not enter GTSAM as an independent LiDAR factor**. Test a separate LiDAR-only refinement in observable directions with OD-008 noise calibration, and safe **omit-factor** behavior when geometry is insufficient. Subtracting a prior Hessian alone must fail the independent-evidence admission rule.
4. **Overlapping submap memberships:** Run a 90/180 OD-010 lifecycle; scans **91–180** enter A and B and have both membership records. Assert at most **one primary local LiDAR factor per keyframe/encounter**, even with two candidate submap matches; check stable map snapshots and source ID tracking.
5. **No self-loops or repeated information:** Attempt a loop from query scan `q` to a submap containing `q`: **reject**. Repeat the same scan/target match/loop creation 100 times: factor count/information must not grow. Send near-duplicate loops against overlapping A/B with shared historical scan sets: suppress or conservatively handle as correlated groups rather than 100 independent samples.
6. **Motion-conditioned deskew:** Generate high-speed arcs and in-place rotation with known wheel/gyro bias/slip and timing error. Deskew scans using the same noisy OD-006 interval that generated fused odometry factors. Compare independent-factor covariance assumptions versus calibrated **conservative OD-008 LiDAR inflation** against exact synthetic truth; quantify overconfidence and note that inflation does **not** remove the actual covariance cross term.
7. **Degenerate geometry and incorrect loops:** Corridor with weak along-axis LiDAR information, empty/open space, corner, repeated warehouse shelving with symmetric false loop, dynamic moving obstacle and many redundant proposals. Reject OD-008 rank-deficient **full 3×3** LiDAR factors until a valid observable-subspace factor exists; apply geometric, innovation and duplicate-evidence gates **before** robust GTSAM loop loss. Measure false accepted loops and graph deformation.
8. **Optimization and immutable local maps:** Close true loops, run batch LM/Gauss–Newton, compare `X_k` and `S_j` to exact truth, compute `T_map_odom` correction and assert **continuous OD-006 local odom**; frozen occupancy/EDT/pyramid local hashes remain unchanged under global pose changes. Graph versions and global spatial index update without mutating underlying source fields.
9. **Portable graph state and replay:** Serialize native factor records with exact node/submap/encounter/source/scan membership, covariance/OD-008 calibration, map-snapshot versions/correlation groups through OD-012. Roundtrip in Docker, load the same graph after interrupted generation recovery, and check deterministic factor-admission decisions on identical input IDs. No ROS/Ceres/GTSAM object types in public graph serialization.
10. **Compute and implementation alternatives:** Compare initial batch GTSAM keyframe+submap graph to pose-only baseline and optionally future iSAM2 without inventing online performance claims. Measure node/submap/factor count, loop candidate/dedup counts, solve p50/p95/p99 latency, peak RAM, p95 sensor capture lag under background graph work, graph sparsity and Pi 5 practicality when hardware is available. Do not dispatch GitHub Actions.

**Calibration and reference truth:** The synthetic apartment/warehouse trajectories are the **exact truth** source. Real world confidence/NEES and ATE/RPE require **independently verified reference**, not LiDAR-derived self-evaluation. Use separate calibration and held-out trajectories for OD-008 inflation and OD-013 loop/admission settings, including high yaw, slip, deskew timing error, partial scans, long corridors and short repeated-aisle traversals. Report when source modalities are unavailable; a bag's already-fused `/odom` cannot be represented as independent raw encoders plus gyro.

**B21 report:** Source dataset/revision/truth class, graph keyframes/submaps/factors, gauge, local anchor revisions and OD-010 membership intersections, fused wheel/gyro sample time ranges and OD-006 bias version, Ceres prior-vs-LiDAR geometry evidence, OD-008 rank/calibration and correlation inflation, OD-011 map snapshots, source/encounter dedup groups, loop acceptance/rejection/false loop rate, rank/covariance coverage, ATE/RPE with independent truth, frozen-grid hash invariance, graph pose version and batch solve CPU/RAM. Explicitly quantify information overconfidence when naïvely duplicating factors; that ablation MUST NOT be included as a recommended production configuration.

### 4.11 Accepted OD-014 selective iterative refinement and deterministic rollback fixtures

**Benchmark B22** compares the first deterministic **one-pass `R0` full replay/map/OD-013 batch graph result** against **up to three additional `R1–R3` selective refinement candidates**. Inputs must include original immutable LD19 packets with OD-003 time model, IMU and wheel events, OD-001 session/epoch, OD-004 deskew inputs, OD-005 validity, OD-010 insertion/membership, and OD-012 accepted map generations. **Never use a rendered/loaded map bundle as a substitute for the raw acquisition stream** needed to rerun beam timing and trajectory reconstruction.

**Fixtures and evaluation:**

1. **Baseline and iteration bound:** Generate a complete apartment/warehouse R0 map from unchanged raw synthetic inputs, then permit **0, 1, 2 or 3** additional iterations under fixed seeds/config/source IDs. Assert no fourth outer candidate, transparent early stopping and stable accepted generation hashes on deterministic repeat.
2. **Pure rigid graph correction:** Apply known global `T_map_submap` and common local gauge shifts after correct loops. Assert **zero beam re-deskew, zero log-odds map reinsertions, zero local EDT/correlation cache rebuilds**, immutable source hashes unchanged and only graph/global-index pose revision updated.
3. **Relative scan-local pose shift:** Change scan midpoint `T_submap_scan` for some keyframes but keep their within-scan relative motion identical. Correctly rebuild only affected OD-010 submaps from raw rays, **without redundant re-deskew**, including both submap memberships and stable insertion IDs.
4. **Intra-scan motion/bias correction:** Inject gyro Z bias, wheel/slip trajectory errors and high-speed arcs/in-place rotations. Compute maximum **submap-local endpoint displacement** for valid beams. Test values below/at/above the unvalidated **0.025 m (half 5 cm cell)** trigger, especially long beams and calibrated laser lever arms; re-deskew and remap only affected cases.
5. **Timing calibration:** Inject LD19 epoch-aware offset/drift, packet-counter wrap and unknown acquisition timestamp delay. Refit only when trusted independent timing evidence bounds the parameter; **reject unobservable or unsupported offset/drift/bias updates** even if same-data Ceres cost drops. Invalid time/trajectory coverage overrides displacement threshold.
6. **Deterministic source/occupancy safety:** Rebuild from original OD-005 valid hits, explicitly missing/invalid sectors and actual encoder/IMU time series. Verify known/unknown mask, ray hit-over-free precedence, one update per cell per accepted scan/submap, proper 90/180 membership and exact log-odds full-replay consistency; no raster warping/double-insertion.
7. **Derived cache and graph dependency replacement:** Changed occupancy source revision invalidates dependent OD-011 EDT and OD-009 pyramids, affected OD-007/008 geometric matches and OD-013 local/loop factors. Assert **old factor IDs are superseded, not appended**; scan-to-map self-matching and shared overlapping-submap evidence remain prohibited. A rigid-only update preserves cached local geometry.
8. **Quality acceptance vs overfitting:** Simulate a false loop and a candidate with **lower in-sample graph/matching cost but worse held-out trajectory/scan truth**, covariance coverage or geometric consistency. Reject/regress such a candidate, keep its diagnostics but **restore the previous accepted OD-012 generation**. Do not compare total graph objective directly if candidate factor/map membership differs.
9. **Atomic publication/recovery:** Fault inject staging/disk-full/incorrect source hash/invalid cache/aborted save before OD-012 `CURRENT` update. Reader must load the last accepted complete iteration; provenance includes raw MCAP/input hashes, calibration, timestamp domains/epochs, scan IDs, graph correlation groups and benchmark configuration.
10. **Resource/quality comparison:** One-pass only versus selective iterations versus **unconditional full submap rebuild after every graph pass** on identical input/held-out splits. Measure selected re-deskew beam/scan counts, changed submaps/cells/cache bytes, factor regeneration, accepted/rejected iterations and stop reasons, graph/scan residual quality, loop false positives, ATE/RPE **only with exact or independent truth**, OD-008 uncertainty coverage/NEES when valid, p50/p95/p99 work time and peak RAM. Profile on workstation and Pi 5 when practical; offline work must not block any live recorder.

**Reporting constraints:** Clearly distinguish **accepted iterations versus rejected candidates**, unchanged rigid corrections from local ray changes, time/bias parameter calibration backed by independent evidence from map-fit-only conjectures, and exact/independent truth from LiDAR-derived self-reference. Record threshold sweeps around 0.5 cell, candidate CPU/memory and the held-out quality criterion **before** observing the benchmark result. One-pass `R0` is always the comparison baseline and first deliverable; the three-pass extension is opt-in and not a prerequisite for initial SLAM functionality.

### 4.12 Accepted OD-015 motion keyframes, independent submaps and optional bias-knot fixtures

**Benchmark B23** evaluates the **native motion/uncertainty-aware keyframe selector**, separate OD-010 occupancy insertion/submap lifecycle, OD-008 LiDAR factor gates, and **zero initial GTSAM bias variables** against optional **5–10 s offline scalar yaw-bias random-walk knots**. Use exact-truth synthetic apartment and repetitive warehouse motion/scan/IMU/wheel streams with OD-001 canonical acquisition nanosecond timestamps, OD-004 midpoint poses, OD-005 scan eligibility and OD-006 uncertainty. **Do not conflate keyframe selection with Cartographer's scan insertion motion filter.**

**Mandatory controlled fixtures:**

1. **Threshold equality and ordering:** Generate scan midpoints with translation `0.20 m±ε`, yaw `5°±ε`, moving elapsed time `2.0 s±ε` and normal minimum interval `0.2 s±ε`. Test just-before/equal/after, tie-breaking, shortest yaw across ±π and 359°↔0°, pure rotation, pure translation, mixed motion, monotonic time and OD-001 reset/paused `/clock`.
2. **No stationary flood:** Replay stationary robot for 2 s, 30 s and minutes with noisy gyro while OD-006 continues bias calibration; require no time-only graph keyframe explosion. Small but meaningful motion below distance/yaw thresholds eventually yields a **bounded** moving-time fallback when 2 s elapses, provided 0.2 s minimum interval is met.
3. **Minimum interval and deferred source accounting:** Bursts of high rotational motion that exceed 5° within 0.2 s, missing scans, sparse source measurements and quality-deferred observations. Verify `DEFERRED_MIN_INTERVAL` reasons, eventual re-evaluation, and exactly one fused wheel/gyro factor covering each actual adjacent-keyframe interval, not duplicated short intervals.
4. **Uncertainty and observability:** Gradually increase OD-006 predicted covariance and perturb OD-008 LiDAR-only rank/support in long corridor, open area, informative corner and cluttered warehouse aisle. A high-uncertainty low-rank scan should trigger OD-009/022 degraded/recovery **without** spurious confident LiDAR factors or runaway node growth; an informative scan may trigger a bounded additional keyframe. Report accepted keyframe count separately from accepted geometry factor count.
5. **Submap lifecycle independence:** Feed ≥280 eligible OD-010 map insertions under several keyframe policies (e.g. 0.1/0.2/0.4 m and 2.5/5/10°). Submaps must still follow **A IDs 1–180, B IDs 91–270**, spawn at 90 and freeze at 180 **per submap**, even if graph keyframe counts differ. Preserve overlapping scan membership but at most one local primary LiDAR factor per encounter.
6. **Submap anchors:** New submap creation requires association with an existing timestamped keyframe or forced `SUBMAP_ANCHOR`, including a forced event inside 0.2 s interval with explicitly logged exception. Test anchored `S_j` global Pose2 optimization without duplicated anchor sensor likelihood, and correct splitting/recomputation of fused motion source intervals.
7. **No initial bias graph knots:** Run first OD-013 batch Pose2 graph with `X_k` and `S_j` only, retaining OD-006 evolving yaw bias. Compare metrics across keyframe densities before introducing new bias variables; quantify graph node count, memory/CPU, trajectory drift and information rank.
8. **Optional scalar offline bias knots:** With OD-014 optional experiments, introduce time-indexed `b_j` every **5 and 10 seconds** on valid same-epoch traces; inject known slow yaw-bias drift, temperature-dependent changes, wheel slip and unobservable segments. Verify `Var(b_(j+1)-b_j)=q_b*Delta_t_j`, source time/units, random-walk and bias-aware planar wheel+gyro factor Jacobians, observability gates and hold/fixed bias under ambiguity. **Replace** the original fused interval motion factors; never add bias-aware and baseline factors as independent evidence.
9. **Remapping and provenance:** When optional bias changes OD-004 intra-scan trajectory, verify OD-014 0.025 m geometry trigger and selective re-deskew/rebuild of affected OD-010 submaps, OD-011 caches and OD-013 factors; untouched rigid-only maps/caches stay immutable. Retain raw gyro/encoder intervals, scan midpoint and source clock session IDs and OD-012 graph/iteration revisions for deterministic reconstruction.
10. **Held-out error and resource comparison:** Compare motion-only vs uncertainty-adaptive keyframes, alternative thresholds, baseline zero-bias graph vs sparse 5/10 s offline knots, and optional information-gain proposals only when implemented. Measure graph nodes/factors per metre and minute, geometry overlap, rank/OD-008 covariance consistency, ATE/RPE against **exact or independent ground truth**, false loops, odometry yaw bias, graph solve p50/p95/p99, CPU/peak RSS on x86 and Pi 5 if available. Separate training/calibration from held-out validation; high-rate scanning does not imply high keyframe rate.

**B23 report:** All initial threshold values, equality semantics, min-interval exceptions, keyframe and no-LiDAR-factor outcomes/reasons, scan IDs/time domain, source fusion interval IDs, submap anchor/membership graph, accepted geometry factors and observed ranks, initial `bias_knots=0`, optional knot timestamps/`q_b` and factor replacement, OD-014 reconstruction decisions, graph convergence and Pi/host resource costs. Record when parameter values are unmeasured; do not claim benefits from a 5–10 s bias model unless held-out evidence supports it.

### 4.13 Accepted OD-016 four-process IPC, scheduling and recording-loss isolation

**Benchmark B24** verifies the **four independently managed Raspberry Pi 5 Linux processes** (`acquisition`, `recorder`, `slam`, `web_lab`), **bounded versioned `SOCK_SEQPACKET` Unix IPC**, fixed SLAM workers and explicit data-loss/stale-state accounting. Use deterministic synthetic LD19 packet/beam, 200 Hz IMU and wheel encoder streams with OD-001 canonical nanosecond clocks, source/boot/epoch identifiers, OD-002/003 time calibrations, and OD-023 raw MCAP (when implemented). Report a **complete/incomplete truth flag** per recording; raw losslessness cannot be assumed from IPC ordering.

**Mandatory fault and performance cases:**

1. **Four-process lifecycle:** Launch each process separately in Docker/Compose or equivalent manual local Linux setup, validate exclusive hardware-descriptor ownership and that headless off-line C++ core CLI/tests can still run with **no IPC/Web Lab**. Graceful stop/restart, supervisor crash loops, reconnect and restored run/clock epochs must not mix sessions.
2. **Wire protocol correctness:** Exercise connected `AF_UNIX/SOCK_SEQPACKET` ordering, boundary preservation and version/required feature negotiation, serialized exact field sizes/endian, bad payload length, `MSG_TRUNC`, `EAGAIN`, peer close/half-broken connection, duplicate/out-of-order/lost per-source sequence and explicit source/clock-generation discontinuity. Verify any gap is detected; socket delivery is not replay/durability.
3. **Recorder pressure and durability:** Inject slow consumer, disk-full, fsync error and process kill across raw packets, clock-calibration observations and configuration changes. Require a **declared gap or `INCOMPLETE`/STOP_RECORDING session**; never assert lossless after missing raw sequence. Distinguish received/enqueued/written/**durably finalized** source high-water marks and segments. Replay OD-014 raw provenance as far as complete data actually exists.
4. **Acquisition nonblocking fan-out:** Stall both recorder and SLAM independently and together; verify bounded **count+byte** per-edge queues, monotonic source timestamps, no indefinite acquisition-thread block, measured jitter, explicit enqueue failure and gap IDs. Independent SLAM overload cannot covertly corrupt a healthy recorder stream, and failed recorder cannot freeze acquisition; report actual drops, not a universal lossless guarantee.
5. **SLAM worker contention:** Feed ~10 Hz scan revolutions and high-rate gyro/wheel inputs while OD-007 Ceres is delayed, OD-009 global BnB scans, OD-011 EDT full rebuilds, OD-012 disk saves and OD-013 large batch graph run in bounded lower-priority workers. Test queue coalescing, cancellation, graph/map snapshot lifetime, stale revision rejection and continuous OD-006 odom with only valid `map→odom` corrections.
6. **Web Lab isolation:** Send slow/disconnected telemetry subscribers, very large frames, malformed/unauthorized control messages and high-rate requests. Validate last-value-only UI dropping, bounded Web Lab memory/CPU, authenticated command and applied-config/version acknowledgment; no scan/recorder latency regression from UI work.
7. **Restart/failover:** Crash acquisition, recorder, SLAM and Web Lab **one at a time**. Acquisition reboot increments source session/epoch, recorder rolls or flags incomplete segment, SLAM reloads validated OD-012 state or begins a new identity, Web Lab reconnects; stale graph results/clock events are never accepted. Report lost sensor interval length and operator-visible state.
8. **Scheduling experiment:** Compare default **SCHED_OTHER** workers and only explicitly configured bounded acquisition scheduling under CPU contention/thermal throttling, measuring starvation of recorder and graph, precise timestamp jitter and process CPU/RSS. Do not assert a `SCHED_FIFO` benefit without measurement and watchdog/privilige safeguards.
9. **Optional shared snapshots:** When enabled, compare `memfd_create` + `SCM_RIGHTS` sealed immutable FD passing against ordinary copied seqpacket batches for versioned frozen occupancy/EDT/graph; enforce matching hashes, ownership, pin counts, FD/byte limits and no mutation. Treat a disabled memfd path as correct Stage I behavior.
10. **Resource and acceptance:** Repeat on Docker x86/ARM64 plus **actual Pi 5** when available, with slow SD/SSD, normal load, bursty packets, CPU throttling and loss windows. Publish p50/p95/p99 acquisition-to-pose, per-source sequencing/drop rate, queue max count/bytes/age, deadline misses, graph lag, recorder finalized bytes/high-watermark, memory/FD use, CPU/thermal, recovery time and Web UI update age. Differentiate measured hardware results from simulated ones.

**B24 report and policy:** Display a four-process health timeline and IPC protocol/session revision, source event/durable recording boundaries, raw-to-SLAM divergence, OD-022 localization degradation, kernel send/receive errors and stale immutable graph/cache revisions. **No magic default queue sizes or durable fsync interval are chosen here**; OD-021 must specify and benchmark concrete caps and stream-specific drop/shed thresholds, OD-023 the MCAP schema. The benchmark should demonstrate *bounded failure and observability*, not promise zero lost samples under arbitrary disk stalls. Local Docker/Catch2 integration tests only; do not dispatch GitHub Actions.

**External protocol references:** [Linux unix(7)](https://man7.org/linux/man-pages/man7/unix.7.html), [Linux memfd_create(2)](https://man7.org/linux/man-pages/man2/memfd_create.2.html).

### 4.14 Accepted OD-017 target boundaries and clean installed SDK consumer

**Benchmark B25** verifies the initial **eight host C++20 libraries**, **four public/installable `slam::*` SDK targets** and **three native host daemons**, with **Web Lab and Pico firmware built independently**. Test the **installed/exported CMake package** as an actual external product, not merely an in-tree alias that happens to compile. Use Docker with pinned CMake ≥3.25, Ninja/toolchains, Catch2 v3/cxxopts and separately pinned solver libraries. No sensor hardware, ROS2, Pico toolchain or browser environment is required for portable SDK unit testing.

**Mandatory build/packaging fixtures:**

1. **Target inventory:** Configure project and enumerate all host production targets; require precisely `slam_core`, `slam_mapping`, `slam_engine`, `slam_storage`, `slam_frontend`, `slam_match_ceres`, `slam_graph_gtsam`, `slam_platform_linux` and `slam_acquisitiond`, `slam_recorderd`, `slam_localizerd`. Test CMake target graph for duplication/undeclared additions. Catch2 tests/optional tools do not affect the baseline 11 target count.
2. **Exact SDK exports:** Install into a fresh prefix, parse `FusionSLAMConfig.cmake`, `FusionSLAMConfigVersion.cmake` and `FusionSLAMTargets.cmake`; require **only four exported SDK targets** `slam::core`, `slam::mapping`, `slam::engine`, `slam::storage`. Internal four target names and headers **must not be exported** as supported public SDK targets.
3. **Real out-of-tree consumer:** Configure separate minimal project with `find_package(FusionSLAM CONFIG REQUIRED)`, include `slam/core/time.hpp`, `slam/mapping/occupancy_grid.hpp`, `slam/engine/slam_engine.hpp`, `slam/storage/map_bundle.hpp` and link all four targets. Build/run outside original source/build trees, using **only installed headers and declared public dependencies**; repeat after relocating install prefix. Reject missing imported target or incorrect version behavior.
4. **Public header isolation:** Compile representative/all public headers in isolation without `ceres::`, `gtsam::`, `rclcpp`/ROS messages, `sys/socket.h`, Linux device/Pico headers, internal worker/cache/graph headers, cxxopts, Catch2 or private Zstd/MCAP implementation types. Scan header and imported `INTERFACE_LINK_LIBRARIES` for dependency leakage. Eigen is allowed only when properly declared as an intentional public dependency.
5. **Acyclic dependency graph:** Verify `slam_core` has no host project deps, `slam_mapping` only core, `slam_storage` portable core/mapping, and platform only in executables; frontend does not link back into solver/graph implementation cyclically. Ceres and GTSAM remain `PRIVATE` implementation backends, and ROS2 adapter only consumes SDK. Test transitive include/compile/link visibility (`PUBLIC`/`PRIVATE`/`INTERFACE`) rather than assuming intended visibility.
6. **Static-link export trap:** Attempt clean installed package consumer for both supported shared and static configuration variants, if offered. A private internal static link can emit `$<LINK_ONLY:slam_match_ceres>` or similar unresolved names. Require **successful link/loader test** via valid shared façade/pImpl or an equivalently complete packaging approach, with **no unexported internal target** in the public install export. Report required private runtime libraries and license/package versions rather than claiming no runtime Ceres/GTSAM use.
7. **Optional build isolation:** `BUILD_TESTING=OFF` does not download/compile Catch2; `BUILD_TESTING=ON` builds library-specific Catch2 tests and CTest discovery. cxxopts appears only with daemon CLIs. Independent Web Lab and Pico SDK projects do not inject additional host targets, browser dependencies or firmware crosscompiler into portable SDK. Optional ROS adapter defaults OFF and builds separately only when available.
8. **Three executable smoke tests:** Each native executable `slam_acquisitiond`, `slam_recorderd`, `slam_localizerd` handles `--help` and invalid arguments correctly, uses appropriate private adapter/SDK links, and starts in a safe simulated/fake-sensor mode without unexpected hardware assumptions. Four OD-016 runtime processes result when separately built Web Lab is launched; test IPC compatibility, recorder and SLAM restarts using B24.
9. **Portability/resource:** Repeat clean build, install and external-consumer runs on Linux x86-64 and Raspberry Pi 5 ARM64 or applicable ARM64 Docker/toolchain. Report CMake configure/build/test/install durations, compiler/ABI/package version, installed bytes, target counts, public-header parse/build latency, dynamic library dependencies, startup RSS and container image size. Do not infer Pi 5 performance from x86-only runs.
10. **No Actions/minutes:** Use local/Docker `cmake --build`, `ctest`, `cmake --install` and external consumer executable. If GitHub Actions files are ever added, require manual-only `workflow_dispatch` unless user explicitly authorizes otherwise; do not dispatch or enable them as a substitute for local tests.

**Acceptance report B25:** Actual target counts, imported target/public header inventory, package major/minor and ABI policy, dependency DAG and any `LINK_ONLY` leakage, external consumer configure/build/link/run status, runtime private solver dependencies, `BUILD_TESTING`/optional project isolation, platform/environment and complete command logs with failure reasons. Passing in-tree tests **does not satisfy** the installed-package consumer criterion. This benchmark is a future requirement; **no source build or tests have yet been performed** for OD-017.

**Reference:** [CMake install/export and package guide](https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html).

### 4.15 Accepted OD-018 staged frozen-submap loop retrieval and conservative verification

**Benchmark B26** validates **Stage I covariance-aware spatial retrieval**, OD-009 admissible branch-and-bound, OD-007 independent LiDAR geometry refinement and OD-008/013 loop factor admission, followed by **Stage II rotation-tolerant 2D occupancy/observed-space frozen-submap signature** candidate retrieval. Compare against exact-truth apartment and repetitive warehouse worlds, including OD-010 two-active-submap shared scan IDs, OD-011 immutable snapshot hashes and OD-013 keyframe/submap graph poses. Run in Docker Catch2/CTest first, then profile Pi 5 CPU/queue/matching latency; neither Stage II descriptor results nor Pi budget compliance are claimed implemented.

**Mandatory deterministic experiments:**

1. **Drifted true spatial loop:** Drive an apartment loop with known accumulated OD-006 wheel/gyro yaw/translation drift, including a large `T_map_submap` disagreement. Vary covariance-aware spatial search radius and OD-009 search mode under bounded windows. A true geometrically corroborated loop may pass **despite large uncorrected odometry innovation**; evaluate retrieval recall@K, final truth SE(2) and map/graph error without disabling covariance/consistency gates.
2. **False positive spatial environment:** Repetitive warehouse shelves, symmetric long corridor, open-space weak geometry, dynamic boxes and OD-005 partial-sector scans. Confirm wrong high-scoring candidates fail overlap, distinctiveness, OD-008 eigen-rank, residual or graph consistency even when BnB score is high. Measure **false accepted loop count and precision**, not only recall.
3. **Source-scan/self-evidence:** A newly acquired query scan already present in a frozen target, A/B overlapping submaps with shared accepted scan IDs 91–180, successive same-encounter keyframes and repeated candidate retrieval. **Do not create a self-loop, correlated duplicate loop or independent second measurement from repeated use of the same historical rays**; factor count/information must not grow artificially.
4. **OD-009 coarse search validity:** Compare multi-resolution pyramid bounds and BnB accepted candidates to exhaustive small-grid correlation scores. Test angular wrap and **360°** search, multiple spatially distinct top hypotheses, node/candidate cap, per-invocation **20–50 ms** target, timeout/cancel, stale pinned map and truthful `INCOMPLETE`/`BUDGET_EXHAUSTED` result; an incomplete search may not be reported as global-optimal.
5. **Ceres prior separation:** Inject a strong **incorrect odometry prior** conflicting with accurate known LiDAR geometry. Refine BnB candidates with OD-007 unsigned EDT/bilinear/per-beam Huber on coherent OD-011 map; require **separate LiDAR-only or defensibly conditioned** OD-008 information/rank/covariance before OD-013 factor emission. Reject any attempt to reuse posterior-plus-prior pose as an independent LiDAR graph observation or to remove contamination by Hessian subtraction alone.
6. **2D frozen-submap signature Stage II:** Construct planar polar occupancy/known-mask descriptors on frozen local 5 cm grids. Apply global translations, local yaw rotation/circular bin shift, different known-space support, partial scans, unknown wedges and map resolution changes. Test rotation-tolerant ranking, descriptor ambiguity, observed-vs-free semantics and algorithm/config/source-hash invalidation. A **global graph pose change alone must not rebuild local signatures**; update global spatial index instead.
7. **Cross-stage ranking/dedup:** Merge spatial and global-signature candidate sets and retain **top 3–5 distinct** hypotheses after time/trajectory and scan-set overlap filters. Compare Stage I and Stage II candidate recall@K and unique true revisits, with cost of missed/false candidates and separately reported descriptor rank versus final geometric acceptance.
8. **Graph admission and rollback:** Use independent exact trajectory truth for a distant valid loop, a visually plausible but geometrically incorrect repeated-aisle candidate, and many correlated overlap candidates. Require OD-008 calibrated geometry/observable rank, OD-013 shared-source/correlation admission and **provisional graph consistency**. Robust losses alone do not legitimize wrong closures; reject/map-rollback candidates that deform verified trajectory/submap relationships and preserve prior OD-012/013 accepted graph generations.
9. **Concurrency/replay:** While OD-011 frozen snapshots remain pinned, publish graph-pose and map cache revisions, run OD-014 selective local remapping and cancel background loop jobs. Verify stale query membership, changed source hash or factor version triggers explicit revalidation or discard; rebuilt-map loop records and descriptors do not coexist as accepted evidence with superseded source data. Deterministic replay preserves candidate/rejection provenance and no duplicate loops.
10. **Pi 5 budget and dependency boundary:** Run synthetic **~10 Hz LD19** plus IMU/encoder input while the OD-016 lower-priority verifier operates. Report initial **spatial 1–5 ms**, **Stage II descriptor 5–20 ms**, **BnB 20–50 ms/candidate**, **top 3–5**, and **100–250 ms background batch** as **unvalidated targets**, with Ceres/refinement included in total. Measure p50/p95/p99 candidate/search/refine/graph latency, CPU/RSS, queue length/cancellation, acquisition/recorder/localizer jitter, precision-recall and ATE/RPE under independent truth. Verify portable native interface builds without ROS/Ceres/GTSAM/IPC headers and no ninth OD-017 library or extra daemon.

**B26 report:** Query/target stable IDs and source scan intersections; OD-001 clock/scan status; frozen occupancy, score and descriptor hash; graph-pose/spatial-index revisions; spatial covariances and retrieval method; top-K rank/scores; BnB exactness/budget and completeness; Ceres convergence and wrong-prior handling; OD-008 overlap/robust residual/Hessian rank/calibrated covariance; OD-013 correlation/admission/rejection, graph trial and rollback; held-out true/false loop precision, recall@K and ATE/RPE; per-stage time, total CPU/RSS and Pi process responsiveness. Keep **Stage I acceptance distinct from Stage II gains**; do not let a good descriptor rank count as a verified loop. No GitHub Actions.

### 4.16 Accepted OD-019 sparse global iSAM2, optional fixed-lag separator and offline cleanup fixtures

**Benchmark B27** tests the **initial Stage I persistent submap-only global Pose2 iSAM2** with OD-006's independent local wheel/gyro estimator, and compares it to a complete **offline OD-013 keyframe/submap batch graph**. A second **Stage II bounded local fixed-lag GTSAM smoother** (30–60 s, ~150–200 recent keyframes, 2 active+recently frozen submaps) is **conditional and must be evaluated separately**; it must not be silently counted as part of the initial online graph. Use exact-truth apartment, warehouse, corridor and long-duration loop traversals with immutable OD-005/010 scan IDs, OD-011 revisions and OD-013/018 correlation/loop provenance.

**Mandatory tests and comparisons:**

1. **Stage I graph topology:** Under identical OD-010 insertion and OD-015 keyframe inputs, build online global iSAM2 **only with submap `S_j` variables**, admitted relative-submap summaries and validated OD-018 loops. OD-006 `T_odom_base` stays continuous and high-rate; there is no initial local GTSAM smoother, no per-beam keyframe graph updates and **zero graph gyro-bias knots**. Compare submap poses/ATE/RPE and residual quality against offline OD-013 `X_k,S_j` full batch with exact independent truth.
2. **Summarized evidence provenance:** Build input pairs where fused wheel/gyro odometry, odometry-conditioned deskewed LiDAR and two OD-010 overlapping submaps share raw sources. Test accurate joint summary or conservative factor deferral; repeated overlapping and duplicated source group proposals **cannot produce independent extra submap factors** or overconfident covariance. Compare credible info/rank against full batch and explicit correlation-aware synthetic truth.
3. **Stage II Schur correctness (optional):** Construct synthetic linearized local keyframe/submap normal system with off-diagonal correlations and local gauge. Eliminate `m` and retain full joint separator `b`, testing **both `H_bb-H_bm solve(H_mm,H_mb)` and reduced RHS** against direct batch elimination. Pairwise-diagonalized summaries MUST visibly fail consistency comparison; nonlinear marginal summaries record a linearization point and do not invent nullspace information.
4. **Stage II window and overlap:** Exercise **30 and 60 s** windows, target **150/200 keyframe cap**, two active plus recently frozen submaps, OD-010 90/180 insertion independent of keyframe selection, stable source/membership IDs and valid boundary pose/gauge. Check keyframe expiry produces information-preserving marginalization, not factor deletion; absent a valid marginalization implementation, test explicit failure/deferred smoothing.
5. **Large loop and relinearization drift:** Correct a long synthetic trajectory via OD-018 delayed loop, including a very large historical submap pose shift and previously marginalized keyframes. Evaluate linearization-dependent summary error and the **correct offline original-factor reconstruction** against held-out map/trajectory truth. Recompute/discard stale summaries rather than reusing inconsistent old linearization or mixing superseded factor versions.
6. **GTSAM factor removal distinction:** Exercise `ISAM2UpdateParams::removeFactorIndices` on verified-invalid loop(s) with correct actual solver factor indices, then compare to properly marginalized/condensed factor information. Test source IDs versus solver slots, relinearization thresholds/skip, reorder/fill-in diagnostics, unused factor slot reuse, gauge/conditioning/invalid covariance and batch rebuild on incompatible graph revision.
7. **Worker budget and safe completion:** In OD-016 background worker, inject large graph batches/loop corrections with **global ordinary p95 <50 ms** and **optional Stage II local p95 <20 ms** initial targets. Measure actual completed update p50/p95/p99 and CPU; do **not forcibly interrupt an active `ISAM2::update`** at deadline. Verify bounded factor queues, coalescing compatible requests, dropping obsolete prework, pending/cancel state and **atomic publish only after numerical success/version validation**.
8. **Graph growth over hours:** Sustained 10 Hz *received* scans with sensitivity to accepted insertion rate (0–10/s), new submaps after 90 accepted inserts and ~400/h only when sustained 10 accepted/s, several hours of graph expansion and revisits to old frozen submaps. Track all historic `S_j` and correct long-loop handling, graph node/factor count, Bayes-tree cliques/nonzeros, memory/RSS, graph lag and trigger for future regional scaling. No arbitrary pruning of valid constraints to win budget.
9. **OD-012/014 offline batch cleanup:** Replay original raw MCAP and portable factor/membership/provenance, reconstruct full OD-013 `X_k,S_j` graph excluding rejected loops and superseded summary factors, optionally bounded selective OD-014 re-deskew/remap and new OD-011 fields. Validate against independent truth, clock/epoch consistency, accepted loop quality and covariance; publish **new immutable OD-012 generation** or rollback after injected stale concurrent iSAM2 updates, disk/persistence or graph validation failure.
10. **Pi 5 noninterference and portable API:** Under native OD-016 four-process synthetic LD19/IMU/encoder load with large loop optimization and offline batch job, measure p95/p99 capture, local Ceres tracking, recording completeness, map/graph update age, CPU peak RSS and thermal throttling. Internal OD-017 `slam_graph_gtsam` alone uses GTSAM; compile native graph-budget/summary/separator interfaces without GTSAM/ROS/Ceres/IPC. Run Docker Catch2/CTest and Pi hardware tests where available, **not GitHub Actions**.

**B27 report:** Stage I vs optional Stage II vs offline baseline; complete source IDs and graph/submap/OD-015 node counts; true accepted scans/s vs assumed 10 Hz, graph growth per hour; original vs summary factor status, OD-013 correlation group and separator linearization/Schur block dimension/rank/information error; iSAM2 factor removal/relinearization settings and Bayes-tree fill; validated loop accuracy, held-out ATE/RPE/covariance coverage; completed update latency p50/p95/p99, queue age/in-flight, CPU/peak RSS/thermal; stale and rejected candidate counts, offline rebuild and OD-012 rollback trace. Numerical 20/50 ms targets and 150–200 keyframes are **unverified engineering starting points**, not guarantees. Stage II is optional and requires evidence that its extra complexity improves quality or bound.

## 5. Benchmark categories and experiments

| ID | Benchmark | Inputs | Required observations |
| --- | --- | --- | --- |
| B01 | Planar geometry and frame transforms | Synthetic geometry, known `T_base_laser` | Inverse/compose errors, rotated map-origin correctness, yaw/angle sign. |
| B02 | Gyro and wheel prediction | Gyro/wheel streams; truth where available | Relative pose error, drift over distance/time, yaw-bias estimation. |
| B03 | Full 2D LiDAR deskew | Beam times, trajectory, scan | Endpoint alignment error and scan-match residual for **none vs gyro-only vs SE(2)** deskew. |
| B04 | Ceres scan-to-map | Frozen authoritative occupancy map + derived OD-007 EDT, midpoint deskewed scans, odometry predictions/covariances | Pose error, Ceres robust residual and prior contributions, accepted scan overlap/degeneracy, false-locks, runtime; no invalid/missing points. |
| B05 | OD-013/015 offline Pose2 graph and OD-019 sparse online submap iSAM2 | Immutable OD-010 frozen submaps and full original OD-013 factors; OD-018 verified loops, optional OD-014 refinement and OD-019 summaries | B21 full graph/provenance, B26 loop validity, **B27** persistent submap graph/correlation-aware marginalization, global pose consistency and held-out accuracy. |
| B06 | Frozen-map live localization | **OD-012 independently loaded LOCALIZATION_MAP**, held-out traversal and OD-011 optional rebuilt cache | Native map integrity/readiness and localization pose error, dropout, confidence calibration and recovery. |
| B07 | Global relocalization | Unknown/perturbed pose and frozen map, OD-009 multi-resolution BnB candidates | Initialization success, kidnapped-robot recovery, OD-008 validated false-lock/observability, multi-scan confirmation and time to stable lock. |
| B08 | Runtime/portability | Same exact dataset+config on ARM64 and x86-64 | p50/p95/p99 latency, throughput, CPU/RAM/thermal, missed deadlines. |
| B09 | Fault tolerance | Dropped points/IMU, timing jumps, wheel slip, dynamics | State transitions, reported uncertainty, graceful recovery and invalid-data rejection. |
| B10 | Benchmark comparison | Multiple deterministic configs | Reproducible side-by-side Web Lab views and machine-readable reports. |
| B11 | LD19 timing reconstruction and alignment (OD-003) | Synthetic device packets + host receive jitter/clock drift; real LD19 capture | Relative per-beam error, clock slope/combined offset, jitter/uncertainty coverage, unknown physical-delay bound, wrap/reset recovery and deskew impact. |
| B12 | Midpoint SE(2) trajectory and coverage (OD-004) | Exact arc trajectory, asynchronous wheel/gyro streams, nonuniform timed beams, missing samples and extrapolation cases | Midpoint pose/endpoint error; full-coverage map insertion acceptance; quality-gated 9.9/10/10.1 ms extrapolation; interpolation vs delay latency and covariance growth. |
| B13 | OD-005 angular-seam partial scan and invalid-return safety | LD19 packet/seam fixtures, missing wedges, invalid/no-return states, motion coverage | Circular coverage and gap correctness; partial matching/insertion gates; **zero free or occupied updates from invalid/missing returns**; timeout-only recovery, no phantom beams. |
| B14 | OD-006 minimal motion estimation and gyro bias | Exact synthetic SE(2), wheel/gyro streams, stationary calibration, slip and gaps; verified kinematics fixtures | Pose/yaw error, bias error/readiness, bias-update gating, covariance consistency, slip detection and missing-source status; Pi runtime. |
| B15 | Accepted OD-007 EDT Ceres objective, prior and boundary safety | Known occupancy/EDT maps, midpoint scans, synthetic apartment/corridor/warehouse, controlled odom prior/outliers/unknown regions | Per-beam Huber correctness, bilinear/EDT metric/gradients, overlap and LiDAR-only observability despite strong prior, boundary correction, false-locks, provenance/latency. |
| B16 | Accepted OD-008 robust LiDAR-only rank, calibrated covariance and quality gate | Rank-0/1/2/3 Jacobian fixtures, corridor/corner/warehouse midpoint scans, held-out independent truth, degraded sensors | PSD/normalized eigendirections, true rank, calibration/coverage, projected/rejected degenerate poses, false accepts, uncertainty status and runtime. |
| B17 | OD-009 covariance-aware correlative search and async BnB recovery | Synthetic apartment/repetitive warehouse, frozen occupancy pyramids, perturbed/corrupted odom, kidnapped robot | Complete vs partial BnB correctness, bound pruning, candidates recall@K, OD-007/008 accepted accuracy/false-lock, time to relock, Pi per-slice budgets and no capture blocking. |
| B18 | OD-010 5 cm probabilistic occupancy, two overlapping 90/180 submaps and immutable freezing | Exact apartment/warehouse ray traces, controlled motion-filter and memory faults, frozen maps/pose-graph revisions | Log-odds/unknown correctness; hit priority and no invalid clearing; 90 shared IDs, freeze hash/graph-pose separation, map quality vs 2.5/10 cm, CPU/RAM and safety caps. |
| B19 | OD-011 immutable coherent map snapshots, full EDT baseline vs incremental tiles and frozen cache | Exact occupancy/known-mask revisions, added/removed obstacles, 32-cell tiles and halos, concurrent readers, frozen eviction, Pi load | Exact tiled/full EDT equivalence, admissible score pyramid bounds, atomic revision consistency, snapshot staleness/cancellation, cache memory/p95 latency and no acquisition blocking. |
| B20 | OD-012 versioned JSON/zstd map bundles, safe loader and atomic generation recovery | Log-odds/known masks, graph factors/poses, corrupt optional/required payloads, injected power-loss, lazy frozen submaps and cross-platform golden bytes | Exact authority roundtrip, SHA/version checks, regeneration of derived caches, map-vs-checkpoint semantics, valid previous-generation recovery, Pi save/load CPU/RAM/latency. |
| B21 | OD-013 GTSAM keyframe/submap Pose2 graph and correlation-safe admission | Exact apartment/warehouse with overlapping A/B, fused wheel/gyro intervals, LiDAR prior contamination, loops, rank-deficient geometry and shared deskew noise | Gauge/Jacobian correctness, no duplicated motion/scan/loop information, OD-008 covariance consistency, true/false loop gating, global pose vs immutable raster, held-out accuracy, batch solve cost. |
| B22 | OD-014 bounded selective offline refinement, beam geometry threshold and immutable rollback | Raw apartment/warehouse with bias/clock/trajectory perturbations, rigid/relative submap transforms, overlapping scans and wrong loops | R0 + ≤3 candidate iterations, zero rebuild on rigid-only changes, 2.5 cm displacement trigger, correct factor/cache invalidation, held-out improvement, deterministic generation and rollback, CPU/RAM. |
| B23 | OD-015 motion/uncertainty keyframes, independent submap anchors and optional offline yaw-bias knots | Synthetic apartment/warehouse scans with exact SE(2), 0.20 m/5°/2 s/0.2 s edge cases, noisy gyro, degraded matching and 5/10 s bias drift | Deterministic node/factor admission, no stationary flooding, unchanged OD-010 90/180, source-unique motion factors, bias-knot random-walk/observability and held-out accuracy/CPU. |
| B24 | OD-016 four-process versioned Unix seqpacket IPC, worker isolation and recording-integrity faults | Deterministic 10 Hz LD19/200 Hz IMU + wheel feeds, independent recorder/SLAM/Web Lab crashes, disk/CPU/queue overload and graph/map jobs | Per-source gap and durable-high-watermark reporting, **INCOMPLETE** recording on loss, no indefinite acquisition blocking, stale correction rejection, bounded queues/CPU/RAM, restart/session validity, Pi p95/p99 latency. |
| B25 | OD-017 eight CMake libraries, four exported SDK targets and three thin host daemons | Clean Docker C++20 build/install, out-of-tree find_package consumer, public-header isolation, target DAG, optional Web Lab/Pico builds | Exact 8/4/3 inventory, relocatable FusionSLAMConfig exports, no solver/IPC leaks, no static LINK_ONLY target errors, BUILD_TESTING/ROS isolation, successful x86/ARM64 CLI smoke and packaging tests. |
| B26 | OD-018 spatial and rotation-tolerant frozen-submap signature retrieval with BnB/Ceres loop verification | Exact apartment loop with drift, repeated warehouse false loops, A/B shared scans, partial LD19 geometry, OD-011 snapshots and OD-013 graph | Stage I/II recall@K/precision, no self/duplicate loop factors, OD-009 BnB completeness, OD-008 rank/calibrated geometry, OD-013 graph consistency/rollback, Pi CPU/latency and no acquisition blocking. |
| B27 | OD-019 sparse global submap iSAM2, optional correlation-preserving fixed-lag and offline batch cleanup | Exact synthetic long loops, 90/180 submaps, shared source evidence, Schur separator matrices/RHS, delayed loops and multi-hour 10 Hz streams | Stage I one global graph/OD-006 continuity, optional Stage II 30–60 s/150–200 bounded keyframes, no factor double counting, full-history submaps, 20/50 ms p95 completed-update targets, offline rebuild and rollback. |

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
| Deskew endpoint error | Difference to ideal ray-cast geometry at **temporal midpoint**, plus coverage and insertion eligibility | Synthetic truth; controlled real fixture where external geometry is surveyed. |
| Scan-matcher quality | Inlier fraction, mean/robust residual, iterations, convergence and rejection status | All datasets; **not** a substitute for absolute accuracy. |
| OD-008 information/uncertainty | Robust LiDAR-only rank and normalized eigendirections, full/subspace/unavailable covariance, coverage/NEES when independent truth permits, false accepts/rejects | Synthetic exact and verified independent truth for calibration; other datasets for geometry/quality diagnostics only. |
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

**D0 — Dataset architecture / first fixtures:** Commit scenario definitions, deterministic generator, compact truth/scan/IMU/wheel fixtures, OD-004 midpoint SE(2)/10 ms gating, OD-005 seam/invalid-return fixtures, OD-006 gyro bias/slip/kinematics/uncertainty and **OD-007 EDT/bilinear/Huber/prior/unknown/boundary** fixtures, **OD-008 rank-deficiency/robust-information/held-out covariance fixtures**, **OD-009 BnB admissibility, wrong-odometry global recovery and budget/cancellation fixtures**, **OD-010 B18 log-odds/known-mask/90–180 submap and frozen-cache fixtures**, **OD-011 B19 pinned snapshot/full-vs-tiled EDT/cache-coherence fixtures**, **OD-012 B20 versioned JSON/binary/zstd roundtrip, corrupted-file and generation-crash fixtures**, **OD-013 B21 graph gauge/fused-motion uniqueness/submap-overlap/correlation and wrong-loop fixtures**, **OD-014 B22 rigid-vs-relative/local motion, half-cell geometry, recalibration observability, selective factor remap and rollback fixtures**, **OD-015 B23 motion keyframe boundary/uncertainty/admission, independent anchor and optional 5–10 s bias-knot fixtures**, **OD-016 B24 four-process seqpacket version/sequence/restart, recorder-disk-loss and bounded queue/worker fixtures**, **OD-017 B25 CMake target-count/export/clean SDK consumer and public-header isolation fixtures**, **OD-018 B26 spatial/frozen 2D descriptor recall, BnB completeness, overlapping source dedup, Ceres prior separation, OD-008/013 loop graph rejection fixtures**, **OD-019 B27 sparse iSAM2 growth, exact joint Schur separator, factor supersession, p95 update and offline batch-rebuild fixtures**, manifest schema and Catch2 tests. Runs offline and completely within Docker. No external dataset required.

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
