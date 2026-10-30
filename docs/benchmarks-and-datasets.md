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
2. **Filter and quality:** Produce stationary near-duplicate scans, pure rotations, varied translations, a delayed/low-quality scan and false localization. Configure documented translation/yaw/acquisition-time motion filter thresholds. Assert skipped scans are still recorded and OD-006 motion advances but submap counters/cells remain unchanged. Report accepted/rejected/filtered counts and reasons.
3. **Probability and unknown:** Check `logit(0.55)` and `logit(0.49)` log-odds increments, `logit(0.10)`/`logit(0.90)` clamping, odds/probability round trips, and the distinction between untouched **UNKNOWN** `p=0.50` and **observed-known-neutral** `p≈0.50` after contradictory observations.
4. **Ray and duplicate semantics:** Cast crossing hits, duplicate beams and rays that traverse other hits in one scan. For each scan/submap, enforce unique hit/free cell sets, **hits win**, and each cell changes **at most once**. Valid hits produce occupied endpoints and free cells strictly before; OD-005 `INVALID_RETURN`/`MISSING` (including missing wedges) produce **zero map updates**. Explicit validated no-return policy is off initially and, when enabled, may clear bounded ray only without a fake hit.
5. **Geometry/resolution:** Repeat same truth scene at **0.025 m, 0.05 m and 0.10 m**, with rotated/translating local submap origins and calibrated laser lever arm. Quantify obstacle-boundary error, structural consistency, free/occupied/unknown precision against truth, connected-wall artifacts and number of allocated cells/RAM. For equal physical area, note expected **~4×/~¼** cell count vs 5 cm, not a guarantee of end-to-end memory/cost ratio.
6. **Memory and spatial limits:** Force cell cap, extent cap, allocator refusal and oversized ray. Require bounded RAM, early freeze with **actual insertion count** or safe insertion stop, no fictitious boundary obstacle/free ray, no unbounded growth and no blocking of raw sensor acquisition. Record peak RSS, allocated cells, map footprint, early stop reason and reduced overlap when needed.
7. **Freezing and graph moves:** Hash local occupancy/known mask before/after freezing and after multiple simulated GTSAM `T_map_submap` global pose corrections. Source hash and local EDT/correlation caches remain unchanged; separately versioned global pose/spatial indexes change. Readers cannot see partially mutated active maps; a published new active snapshot yields new content/cache revision.
8. **Replay and consistency:** Replay identical scans/accepted flags through the same filter/config under different packet arrival schedules. Compare local map hashes, per-submap scan ID lists, lifecycle transitions, known masks, cache inputs and serialized/loaded maps. No map self-matching. Keep dynamic aging **disabled** on frozen maps; optional transient overlay must not change frozen source hashes.
9. **Pi 5 runtime:** Measure occupancy insertion p50/p95/p99, two-active-submap CPU and peak RAM, snapshot/freeze and derived-cache build cost, scan gating load, tile allocation rate and sustained ~10 Hz acquisition under pressure. x86 results cannot substitute for hardware profiling; no GitHub Actions usage.

**Configuration sweep:** Hold map-building and evaluation traversals distinct, vary **grid resolution (2.5/5/10 cm)**, hit/miss probabilities **(0.55/0.49 starting point)**, probability clamp **(0.10–0.90 starting point)**, new/freeze insertions **(90/180 starting point)**, motion filter thresholds, confidence-to-occupied threshold, physical map cap and robot motion. Label them as **unvalidated initial settings** and report parameter source. Do not use ground truth from the evaluation path to tune the mapping run.

**Report B18:** source dataset and truth class, map/submap local frame, grid resolution/origin, configured and *actual* accepted insertion counts, scan ID intersection/overlap by count, known/free/occupied cells, hit/miss/invalid/no-return counters, unique-cell write counts, motion-filter skips, map boundary/coverage error where exact ground truth exists, freeze/graph pose/cache revision IDs, early cap reasons, CPU/memory/latency. Include **zero invalid/missing ray updates** and **zero duplicate per-insertion cell writes** as mandatory regression invariants. If no independent truth exists, report geometric consistency rather than invented absolute map accuracy.

## 5. Benchmark categories and experiments

| ID | Benchmark | Inputs | Required observations |
| --- | --- | --- | --- |
| B01 | Planar geometry and frame transforms | Synthetic geometry, known `T_base_laser` | Inverse/compose errors, rotated map-origin correctness, yaw/angle sign. |
| B02 | Gyro and wheel prediction | Gyro/wheel streams; truth where available | Relative pose error, drift over distance/time, yaw-bias estimation. |
| B03 | Full 2D LiDAR deskew | Beam times, trajectory, scan | Endpoint alignment error and scan-match residual for **none vs gyro-only vs SE(2)** deskew. |
| B04 | Ceres scan-to-map | Frozen authoritative occupancy map + derived OD-007 EDT, midpoint deskewed scans, odometry predictions/covariances | Pose error, Ceres robust residual and prior contributions, accepted scan overlap/degeneracy, false-locks, runtime; no invalid/missing points. |
| B05 | Submap/offline SLAM + GTSAM | OD-010 authoritative occupancy, frozen 90/180 submaps and graph loop constraints | Frozen map hash/pose separation, loop consistency, ground-truth accuracy category, map quality, CPU/RAM and runtime. |
| B06 | Frozen-map live localization | Saved map and held-out traversal | Pose drift / error, tracking dropout, confidence calibration, recovery. |
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

**D0 — Dataset architecture / first fixtures:** Commit scenario definitions, deterministic generator, compact truth/scan/IMU/wheel fixtures, OD-004 midpoint SE(2)/10 ms gating, OD-005 seam/invalid-return fixtures, OD-006 gyro bias/slip/kinematics/uncertainty and **OD-007 EDT/bilinear/Huber/prior/unknown/boundary** fixtures, **OD-008 rank-deficiency/robust-information/held-out covariance fixtures**, **OD-009 BnB admissibility, wrong-odometry global recovery and budget/cancellation fixtures**, **OD-010 B18 log-odds/known-mask/90–180 submap and frozen-cache fixtures**, manifest schema and Catch2 tests. Runs offline and completely within Docker. No external dataset required.

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
