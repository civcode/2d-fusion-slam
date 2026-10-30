# 2D Fusion SLAM — Technical Specification

**Status:** Draft v0.1  
**Date:** 2026-10-30  
**Repository:** https://github.com/civcode/2d-fusion-slam  
**Scope:** Sensor acquisition, offline 2D mapping, online localization, and real-time 2D SLAM.

> This document defines an intended architecture and initial engineering targets, not a claim that these features are already implemented. All rates, dimensions, and performance targets below must be validated with recorded datasets and hardware benchmarks.

## 1. Purpose and design principles

2D Fusion SLAM is a modular, cross-platform C++ system for planar mobile robots equipped with a 2D LiDAR, an IMU, and optionally wheel encoders. It shall support the same sensing and geometric front end in three operation modes:

1. **Record and map offline:** Capture raw measurements on a robot, then replay, calibrate, map, close loops, and optimize on another computer.
2. **Localize online in a frozen map:** Run lightweight, low-latency pose tracking on the robot; recover from uncertain or lost localization.
3. **Perform online SLAM:** Extend the mapping pipeline with incremental submaps, loop closure, and global optimization on the robot.

Primary development target: **Raspberry Pi 5 / comparable ARM64 Linux SBC**. Secondary experimental target: **AWS DeepRacer EVO / x86-64 Linux**, subject to confirming available encoder and steering telemetry.

Principles:

- Keep estimation algorithms independent of ROS and hardware-specific drivers.
- Store raw sensor data and timing metadata; derived scans and poses must be reproducible.
- Separate continuous local odometry from discontinuous global map corrections.
- Use the simplest estimator that meets observable accuracy and reliability requirements.
- Treat sensor uncertainties, calibration, timing errors, and degeneracy as first-class concerns.
- Permit offline batch optimization and deterministic dataset replay without real-time deadlines.
- Do not assume a given differential-drive or Ackermann motion model for all platforms.

### 1.1 Initial non-goals

The initial version will not provide 3D SLAM, vision-based SLAM, learned features, high-integrity navigation certification, or global GPS navigation. Full 3D inertial position preintegration is out of scope for the initial planar MPU6050-based system. Motor control and autonomous route planning are external to the SLAM core.

## 2. Functional requirements

| ID | Requirement |
| --- | --- |
| FR-01 | Parse, validate, timestamp, and record raw LD19 data, IMU measurements, and available wheel encoder measurements. |
| FR-02 | Reconstruct a common monotonic timeline and report timestamp quality, transport delays, gaps, and dropped packets. |
| FR-03 | Calibrate sensor extrinsics, gyro bias, timing offsets, and wheel parameters. |
| FR-04 | Produce continuous local SE(2) odometry from wheel and gyro measurements. |
| FR-05 | Correct LiDAR scan motion distortion using point acquisition times and a time-indexed SE(2) trajectory. |
| FR-06 | Match deskewed scans against occupancy-grid submaps using a Ceres-based geometric frontend. |
| FR-07 | Build overlapping local occupancy-grid submaps and generate quality-checked LiDAR constraints. |
| FR-08 | Fuse independent LiDAR, wheel, and gyroscope information using a GTSAM factor graph, including gyro-bias evolution. |
| FR-09 | Support offline global mapping, loop closure, batch optimization, and refined map export. |
| FR-10 | Support low-latency localization against a previously built, read-only map. |
| FR-11 | Detect uncertain/lost localization and support global initialization or relocalization. |
| FR-12 | Support incremental online SLAM with an asynchronous GTSAM backend. |
| FR-13 | Replay recorded sessions deterministically and compare estimator configurations using identical data. |
| FR-14 | Build and test core algorithms on ARM64 and x86-64 Linux. |

## 3. Hardware and sensing assumptions

| Sensor / compute | Initial choice | Engineering notes |
| --- | --- | --- |
| 2D LiDAR | LDROBOT LD19 | ~10 Hz revolution, ~4,500 points/s, approximately 450 beams/revolution; raw UART packet processing preferred. |
| IMU | MPU6050 | Initial 200 Hz target; use yaw-rate integration, bias estimation, stationary detection, and optional tilt checks. |
| Wheeled odometry | Encoder-based, where available | 100–200 Hz acquisition target; support differential-drive and Ackermann configurations. |
| Main onboard compute | Raspberry Pi 5 or comparable | ARM64 Linux; front end and localization must fit a 100 ms LiDAR period. |
| Optional timestamp coprocessor | Raspberry Pi Pico, RP2040 initially | **Future** custom Pico SDK firmware for IMU/encoder capture and Pi↔Pico clock sync; no micro-ROS; PIO only if justified by measurements. |
| Experimental compute | DeepRacer EVO / x86 | Confirm actual odometry telemetry before enabling wheel factors; steering commands are not encoder measurements. |
| Offline workstation | Linux x86-64 or ARM64 | Batch optimization may use more memory and computation than the onboard target. |

The MPU6050 is a six-axis accelerometer/gyroscope, **not an absolute yaw reference**. Gyro integration drifts; LiDAR geometry and independent encoder yaw measurements help correct it. Avoid double integration of accelerometer data for planar translation in the initial implementation.

### 3.1 LD19 packet and per-beam timing requirements (accepted OD-003)

The LD19 uses ~230400-baud UART packets of **47 bytes** (header, version/count, speed, start angle, 12 range/intensity samples, end angle, 16-bit timestamp, CRC). The documented **speed is degrees/s**, angles **0.01°**, ranges **millimetres**, and device packet timestamp **milliseconds with a nominal 30-second rollover**. The manual describes a **packet timestamp** but does not establish the exact physical acquisition event to which it refers.

**The LD19 acquisition adapter MUST:**

- Validate full frames and CRC, resynchronize after corruption, track missing/suspicious packets, and preserve raw bytes, receive timestamp, device tick, angle, speed, range and intensity.
- Unwrap start/end packet angles across 0°/360°, verify the device's rotation direction, and reconstruct per-point angles using the documented packet interpolation. Convert to native REP-103 CCW-positive laser coordinates only after the sensor sign convention is validated.
- Compute **relative per-beam times** from unwrapped start/end angles and measured angular speed; guard against near-zero speed, implausible angle/speed disagreement, nonuniform motor motion and missing data. The assumed reference angle/event for a packet timestamp **MUST NOT** silently default to the first or last beam.
- Unwrap the device timestamp's **~30-second millisecond counter**. Distinguish normal wraps from out-of-order packets, bad frames and restarts; after sufficiently long gaps where the number of wraps is ambiguous, begin a new device epoch or invalidate source-time alignment.
- Record a **Pi-monotonic** UART reception timestamp and the exact host event it represents (byte/packet arrival vs driver read completion). Account for serial transmission duration and buffered I/O before comparing with the device clock.
- Estimate a calibrated **LD19-to-Pi clock offset and drift** from multiple device-counter / host-reception observations, with fitting outlier rejection, residuals, validity interval, calibration revision, uncertainty, and reset/relock diagnostics.
- **Explicitly preserve the one-way offset/delay ambiguity:** UART reception measurements alone cannot separate a constant source clock offset from a constant physical sensor/serial acquisition-to-arrival delay. The acquisition-to-packet reference offset MUST be **experimentally determined or conservatively bounded**, not advertised as exact from a regression.
- Propagate timestamp granularity (1 ms device counter), rotation-speed/angle estimation, UART latency and jitter, drift, host scheduling, unknown acquisition delay and shared per-packet systematic uncertainty into quality-rated **signed 64-bit canonical nanosecond beam timestamps**.
- Produce a **generic, ordered ROS-independent timed scan** with beam timestamps and quality indicators. Record both source and derived times so that improved offline clock calibration can re-run deskew without modifying the original raw packets. Do not timestamp an entire revolution as a single instantaneous scan.

**Experimentally validate** (1) 30-second rollover, variable angular velocity, angle seams, missing frames and resets, (2) clock fit under offset/drift/jitter/quantization, and (3) the physical beam-to-reported-timestamp delay using controlled motion, optional external timing references, and gyro/scan consistency. Until the physical delay is validated or bounded, expose timing as *estimated/uncertain*. The scan reference time used by the deskewer remains a distinct **OD-004** choice.

This adapter's implementation and its LD19-specific timing calibration belong at the **sensor acquisition boundary**. Neither the portable C++20 SLAM core nor the optional ROS 2 interface may contain LD19-specific packet timing calculations. See [accepted OD-003](open-design-choices.md) and [Architecture §4.1](architecture-specification.md).

## 4. Coordinates, clocks, and calibration

### 4.1 Frames

Use these semantic frames:

- **map:** globally consistent, possibly discontinuously corrected world frame.
- **odom:** smooth local frame, continuous during navigation.
- **base_link:** robot body frame.
- **laser:** LD19 frame, fixed relative to base_link.
- **imu:** MPU6050 frame, fixed relative to base_link.

Convention: body X forward, Y left, Z up; positive planar yaw is counterclockwise about +Z. The wheel/gyro predictor owns **odom -> base_link**; map localization owns **map -> odom**. Define explicit transforms **T_base_laser** and **T_base_imu**, including rotation and translation.

All rigid transforms must have documented source/target frame names and multiplication order. Calibration changes shall invalidate derived artifacts that relied on older calibration.

### 4.2 Time synchronization

**Accepted time representation (OD-001):** use **signed 64-bit nanoseconds** in the native C++20 timestamp and duration APIs. Every comparable timestamp carries or is associated with an explicit **clock domain and epoch**, and conversions happen at sensor, dataset, and ROS 2 boundaries. Do not use wall/system time for physical motion integration; use a stable acquisition timeline for live sensors and separately preserve host steady time for scheduling, latency and hardware timeouts.

**Accepted acquisition staging (OD-002):** The initial implementation timestamps LD19 UART frames, MPU6050 samples and wheel encoder events **directly on the Raspberry Pi**, as close to a known sampling or receive event as practical, with a monotonic host clock. Preserve actual read/packet timing, receipt time, device ticks (if any), quality, jitter and time uncertainty; a UART packet receive time is not necessarily the LiDAR beam's acquisition time. The system MUST record and replay successfully with **no Pico connected**.

Architect an optional **Raspberry Pi Pico firmware backend** for later hardware-near acquisition of IMU and wheel-encoder events. The Pico uses **custom Pico SDK C/C++ firmware**, **not micro-ROS**, DDS or ROS message serialization. A small versioned, checksummed native framing protocol (initial transport candidate USB CDC, UART optional) carries source timestamps, sample data, sequence numbers, status and Pico boot/session identity. The Pi estimates a Pico-to-host offset and drift from repeated bidirectional timestamp exchanges (host t1/t4, Pico t2/t3), including latency/asymmetry and mapping uncertainty, and converts source time into the accepted canonical signed 64-bit nanosecond timeline. On Pico reboot/reconnect, invalidate the previous clock mapping and start a new epoch.

The RP2040 Pico hardware timer provides 64-bit **microseconds since boot**. Converting its ticks to nanoseconds does **not** create nanosecond precision. Pico **PIO** may improve deterministic encoder edge counting or event capture, but it does not automatically produce samples stamped with the hardware timer; the capture path must be designed, measured and compared with a simpler IRQ/timer implementation before selecting PIO. The LD19 may remain on Pi UART. Its **accepted angle/speed and source-clock reconstruction method (OD-003)** stays in the LD19 adapter; the physical packet-to-beam delay requires experimental calibration. Keep Pico physical time independent of ROS 2 `/clock`, even during simulation. See [Architecture `3.5](architecture-specification.md) for source/timestamp interfaces.

Maintain per-source clock alignment: host receipt and hardware/sample time where available, clock offset, drift, uncertainty, rollover, fit version, validity interval and acquisition event definition. Do not compare unconverted timestamps from different clock domains or epochs.

**ROS 2 simulation compatibility:** the ROS-independent core exposes a clock-injection/time-event interface capable of consuming a controlled simulated nanosecond timeline. An optional ROS 2 adapter honors `use_sim_time` and `/clock` (`rosgraph_msgs/msg/Clock`), converting to/from ROS message timestamps exactly using integer arithmetic and checking ranges. When simulation time is uninitialized, do not publish a valid timed estimate. Pauses must not advance estimator integration; backward jumps, replay seeks, source changes and other discontinuities invalidate state/trajectory interpolation across the boundary and trigger reset or compatible checkpoint restoration. Use the independent host steady clock for CPU deadlines rather than `/clock`. A simulated timestamp need not be monotonic across epochs.

Time calibration must account for UART buffering, LiDAR packet timestamp convention, encoder interrupt/sample latency, IMU output/filter latency, and data-ready/FIFO semantics. Reject or flag unsupported interpolation across missing IMU samples. **OD-002 selects the staged acquisition architecture** and **OD-003 selects the LD19 beam-time reconstruction method**, not their final measured precision or calibration constants. Scan reference/interpolation (OD-004) and replay checkpoint/jump policy (OD-024) remain open.

Reference: [ROS 2 Clock and Time design](https://design.ros2.org/articles/clock_and_time.html).

### 4.3 Required calibration

1. LiDAR-to-body SE(2) (and measured tilt with respect to the ground plane).
2. IMU-to-body orientation and gyro-axis sign.
3. Gyro zero-rate bias, noise, and approximate bias temperature sensitivity.
4. Encoder scale, wheel radii, baseline for differential drive, or steering/wheelbase parameters for Ackermann.
5. Sensor clock offsets and timing uncertainty.
6. Optional wheel slip / motion-model noise parameters, estimated from repeated experiments.

Store calibration as a versioned artifact and associate its identifier with every recording and exported map.

## 5. Motion estimation and scan deskew

### 5.1 High-rate planar motion predictor

Fuse wheel-derived forward distance and bias-corrected gyro yaw increments to construct a time-indexed trajectory in SE(2). For differential drive, with left/right wheel increments ds_L and ds_R and track width B:

- Forward increment: **ds = (ds_R + ds_L) / 2**
- Independent wheel yaw increment: **dtheta_w = (ds_R - ds_L) / B**
- Gyro yaw increment: **dtheta_g = integral(omega_z - bias_g) dt**

For a short constant-curvature interval, update from pose (x, y, theta) with dtheta = dtheta_g:

~~~text
x'     = x + ds * sinc(dtheta/2) * cos(theta + dtheta/2)
y'     = y + ds * sinc(dtheta/2) * sin(theta + dtheta/2)
theta' = wrap(theta + dtheta)
sinc(a) = sin(a)/a  (with sinc(0) = 1)
~~~

For Ackermann steering, a nominal model is **yaw_rate = v * tan(steering_angle) / wheelbase**, when actual steering measurements are available. Offer interchangeable kinematic models behind a common prediction interface. Detect wheel/gyro disagreement and increase uncertainty when slip or model mismatch is suspected.

The predictor should provide **pose(t)** and uncertainty interpolation across a short history, and must remain smooth even when global map localization adjusts.

### 5.2 Yaw-only gyroscope preintegration

Start with a planar yaw integrator rather than full 3D position/velocity IMU preintegration:

~~~text
delta_theta_ij(b_ref) = sum_k ((omega_z,k - b_ref) * delta_t_k)
delta_theta_ij(b_i) ~= delta_theta_ij(b_ref) - (b_i - b_ref) * (t_j - t_i)
~~~

A custom GTSAM gyro factor may connect poses X_i, X_j and bias b_i:

~~~text
r_yaw = wrap(theta_j - theta_i
             - delta_theta_ij(b_ref)
             + (b_i - b_ref) * (t_j - t_i))
~~~

Use a gyro bias random walk with variance increasing with elapsed time. Include sensor measurement noise and bias uncertainty in factor covariance. Check analytical factor Jacobians against numerical derivatives.

### 5.3 Per-beam full SE(2) deskew

For a beam measured at t_i, express it in the laser frame at scan reference time t_ref:

~~~text
p_laser_ref =
    inverse(T_world_base(t_ref) * T_base_laser)
    * (T_world_base(t_i) * T_base_laser)
    * p_laser_i
~~~

Use translation from wheel odometry and rotation from gyro integration. Choose and document a consistent scan reference timestamp (for example scan end or midpoint). Explicitly handle the LiDAR lever arm from a noncentral sensor mounting position.

Retain raw points; deskew is a derived view that can be recomputed if offline optimization changes time alignment, gyro bias, or trajectory. Initially, keep bias fixed during one scan and update between scans.

## 6. Shared geometric frontend (Ceres)

### 6.1 Submaps and map representation

Maintain probabilistic occupancy-grid submaps, with log-odds or another documented inverse-sensor model. Construct distance-to-obstacle fields for local registration. Default *experiment starting points*, not fixed product guarantees:

| Parameter | Initial setting |
| --- | --- |
| Occupancy-grid resolution | 0.05 m |
| Submap duration | 30–100 scans, configurable |
| Active submaps | 1–2 overlapping submaps |
| New scan rate | ~10 Hz |
| Match variables | Pose2: x, y, yaw |

Only insert a scan after successful localization against the existing map; never match a scan against a submap that has already incorporated that very scan. Handle unknown cells and maximum-range returns separately from occupied returns. Keep dynamic-object suppression and map aging configurable.

### 6.2 Scan-to-submap optimization

Let D_M(q) be an interpolated submap distance-to-obstacle field and q_i the deskewed laser points. Optimize pose xi in SE(2):

~~~text
E_lidar(xi) = sum_i rho( D_M(T(xi) * q_i)^2 / sigma_lidar^2 )
~~~

Use Ceres with smooth field interpolation, a robust loss, proper residual scaling, and bounded iterations. Implement coarse correlative search for bad initial guesses and loop-closure verification. Do not interpret a converged nonlinear optimizer as proof of a correct localization.

The frontend returns optimized pose, reference submap ID, inlier/overlap statistics, residual score, convergence status, and an uncertainty/information estimate. Detect insufficient structure and ill-conditioning (such as translation along an empty corridor). Noise estimates must not be unrealistically tight because scan endpoints share a common map and may be correlated.

**Information accounting:** Use wheel/gyro prediction to initialize Ceres. If independent wheel and gyro factors are also added to GTSAM, do not simply add the identical fused prediction again as an independent Ceres-derived prior/factor. Deskew also creates some residual measurement correlation that should be documented and, where necessary, modeled conservatively.

## 7. Graph backend (GTSAM)

Use the GTSAM **Pose2 / SE(2)** model for planar robot poses. Primary graph variables are:

- **X_k:** robot keyframe poses.
- **B_k:** scalar yaw-rate gyro bias (possibly fewer bias nodes than pose nodes).
- **S_j:** global poses of submaps when using a submap graph.

Planned factors:

| Factor | Role |
| --- | --- |
| Initial pose prior | Fix global gauge freedom. |
| Wheel odometry | Encoder-derived relative pose / distance with calibrated uncertainty. |
| Yaw preintegration | Relative yaw measurement dependent on bias. |
| Gyro bias evolution | Random walk / temporal consistency. |
| LiDAR scan-to-submap | Geometric pose constraint from the Ceres matcher. |
| Loop closure | Verified relative pose or submap-to-submap constraint with robust loss. |

Use **batch Levenberg–Marquardt / Gauss–Newton** for offline processing and **iSAM2** for incremental online SLAM. A **fixed-lag smoother** is optional during map-based localization, not a requirement for the first live-localizer release.

Do not simultaneously introduce overlapping LiDAR constraints as independent evidence without accounting for shared scan/map observations; avoid overconfidence from redundant relative-pose and scan-to-submap edges. Implement robust loop-closure gating and rollback/rejection of inconsistent constraints.

## 8. Operating modes

### 8.1 Mode A — Record on robot, build map offline (first priority)

**On the robot:** acquire sensors, record packets/samples with original timing, optionally publish high-rate local odometry, and record diagnostics. Full map creation is not required while collecting data.

**Offline pipeline:**

1. Load and validate dataset; reconstruct source clocks and calibration.
2. Estimate/initialize wheel and gyro parameters and bias.
3. Build a continuous trajectory and deskew LiDAR measurements.
4. Match scans in Ceres, build overlapping submaps, and create graph constraints.
5. Detect and geometrically verify loop closures.
6. Optimize globally with GTSAM batch optimization.
7. Re-deskew and rebuild selected submaps when bias/trajectory corrections are material; iterate with convergence criteria.
8. Export a versioned map plus evaluation report.

Offline replay must support deterministic scan selection, configurable processing rate, and the ability to compare parameter changes against the same raw data.

### 8.2 Mode B — Live localization in a previously built map (second priority)

Load a frozen map or indexed map tiles plus precomputed distance fields. Keep high-rate wheel/gyro odometry continuous and match each deskewed scan using Ceres in the nearby map region. Publish global correction via **map -> odom**, while **odom -> base_link** remains continuous.

The localization state machine has three states:

| State | Behavior |
| --- | --- |
| TRACKING | Predict using wheel/gyro, then locally refine with Ceres. |
| UNCERTAIN | Increase uncertainty, widen correlative search, evaluate competing candidates. |
| LOST | Attempt global relocalization using a particle filter (AMCL/Beluga as benchmark or implementation option) or wide-area correlative search; verify consistency across multiple scans before committing. |

The initial localizer can use an EKF-style covariance update or simpler conservative prediction/measurement covariances; GTSAM fixed-lag smoothing may be added if experiments show benefit. A local optimizer alone does **not** solve global or kidnapped-robot localization.

A lost/stale pose must be explicitly signaled to downstream navigation/control; it must not silently present an old pose as current.

### 8.3 Mode C — Full online SLAM (third priority)

Reuse acquisition, high-rate prediction, deskewing, Ceres frontend, submaps, and graph definitions. Run graph optimization and loop closure asynchronously so sensor collection and local control are not blocked. Generate new global map versions while preserving a continuous odom trajectory.

## 9. Recording, API boundaries, and saved artifacts

### 9.1 Recording format

Prefer **MCAP**, directly or via ROS 2 rosbag2, with a simple documented schema for:

- Raw LD19 UART frames (or exact raw packet bytes plus receive timestamps).
- Decoded ranges, intensities, per-beam angle/time estimates and validity flags.
- MPU6050 raw gyro/accelerometer samples, timestamps, and acquisition status.
- Wheel tick counts or measured wheel increments, timestamps, and steering angle where available.
- Calibration snapshot, frame definitions, clock/sync metadata, firmware/driver versions, and recording health statistics.

A dataset must remain usable after decoders or deskew algorithms change. Avoid storing only angle-sorted LaserScan messages or already-fused odometry.

### 9.2 Native map artifact

Proposed logical contents (exact binary format to be selected after prototype):

~~~text
map_name/
  metadata.json
  calibration.yaml
  occupancy_grid.*
  submaps.*
  distance_fields.*
  pose_graph.*
  optimized_trajectory.*
~~~

Map metadata shall include format version, grid origin/resolution, occupied/unknown encoding, frame conventions, created-from-dataset IDs, calibration version, scan-matcher parameters, and checksum/integrity information. Precompute distance fields offline where practical. Localization is read-only; map edits require an explicitly new version.

### 9.3 C++ interfaces

Keep interfaces implementation-independent:

~~~text
SensorSample -> TimestampAligner -> MotionPredictor
LidarPacket  -> ScanAssembler -> Deskewer
DeskewedScan + MapView + PosePrior -> ScanMatcher -> MatchResult
MatchResult + WheelDelta + GyroDelta -> FactorGraphBackend
RecordedDataset -> OfflineMapper -> MapBundle
MapBundle + LiveSamples -> Localizer -> PoseEstimate
~~~

Every pose output should carry a reference frame, source timestamp, covariance/quality indicator, and tracking state.

## 10. Software stack and portability

- **Language:** C++20, CMake.
- **Math:** Eigen; explicit SE(2) utilities and derivative tests.
- **Local optimization:** Ceres Solver.
- **Graph optimization:** GTSAM.
- **Recording:** MCAP; ROS 2 adapters optional.
- **Maps:** Native versioned map bundle with occupancy grid, submaps, and distance fields.
- **Platforms:** Linux ARM64 and x86-64 with portable core algorithms.
- **Testing:** Unit tests, synthetic-sensor tests, deterministic dataset replay, and local benchmark executable.

Suggested source layout:

~~~text
include/fusion_slam/
src/
  sensors/
  time_sync/
  calibration/
  motion/
  deskew/
  mapping/
  frontend_ceres/
  backend_gtsam/
  localization/
  io/
apps/
  recorder/
  offline_mapper/
  live_localizer/
  online_slam/
  replay_evaluator/
tests/
docs/
~~~

Do not add automatic GitHub Actions triggers by default; prefer local tests and builds. If CI workflows are introduced, keep them manual (**workflow_dispatch**) unless explicitly requested otherwise.

## 11. Performance goals and resource policy

These are **target budgets to benchmark**, not measured hardware performance:

| Workload | Target |
| --- | --- |
| IMU and encoder sampling | Nominal 100–200 Hz, acquisition never blocked by mapping. |
| Predicted local pose | At least 100 Hz when sensor timing supports it. |
| LD19 frontend | Process approximately one 450-beam revolution every 100 ms. |
| Scan deskew | Aim for < 2 ms per scan on Raspberry Pi 5. |
| Ceres local scan matching | Aim for < 20 ms per scan (typical tracking case) on Raspberry Pi 5. |
| Occupancy submap update | Aim for < 5 ms per scan. |
| Scan-to-map correction | 10 Hz when valid scans are available; measure p50/p95/p99 latency. |
| Graph optimization and loop closure | Asynchronous, bounded CPU and memory use; no acquisition starvation. |

Measure end-to-end **capture-to-pose latency**, not only Ceres solve time. Use bounded queues, drop policies with diagnostics, and retained raw acquisition timestamps. Report missed deadlines and thermal throttling on the Pi. Online SLAM is allowed to defer global optimization when compute is constrained.

## 12. Testing and acceptance strategy

### 12.1 Unit and integration tests

- LD19 frame parsing, CRC errors, truncated packets, 30-second timestamp rollover, angle/speed beam-time reconstruction, device-to-host offset/drift, physical-delay calibration/uncertainty, angle wrapping, and serial resynchronization.
- Clock offset/drift estimation and timing uncertainty; missing or late IMU/encoder samples.
- SE(2) composition/inverse, curvature integration, Jacobians, and laser/IMU extrinsics.
- Synthetic moving-wall scenario demonstrating rotational and translational deskew.
- Ceres residual gradients, convergence basin, robust outlier rejection, and degeneracy detection.
- GTSAM custom factor Jacobians, bias recovery, covariance scaling, loop-closure gating.
- Map-bundle round-trip, version rejection, artifact integrity, and replay determinism.

### 12.2 Recorded-data benchmarks

Collect controlled sequences with straight walls, in-place rotation, long corridors, loops, dynamic obstacles, wheel slip, starts at uncertain pose, and intentional kidnapped-robot events. Compare:

1. No deskew vs gyro-only vs wheel+gyro deskew.
2. LiDAR-only vs wheel+LiDAR vs wheel+gyro+LiDAR.
3. Ceres local map tracking vs AMCL/Beluga baseline.
4. Offline batch optimization vs incremental online backend.
5. ARM64 Raspberry Pi and x86 experimental runs on the same recorded sessions.

Metrics: absolute trajectory error (where ground truth is available), relative pose error, loop-closure error, localization convergence/recovery rate and time, tracking dropouts, map consistency, estimated covariance calibration, CPU/RAM usage, and p50/p95/p99 compute/end-to-end latency.

### 12.3 Milestone acceptance gates

- **M1 Recorder:** Loss/CRC/timing metrics exposed; raw recording replays without changing measurement order or timestamps.
- **M2 Offline mapping:** Loop-bearing datasets produce consistent maps and optimized trajectories; deskew produces a measurable improvement on controlled moving scans.
- **M3 Online localization:** Frozen-map tracking runs at the LiDAR update rate on Pi 5 and reports uncertainty/lost state rather than publishing false confidence.
- **M4 Online SLAM:** Incremental mapping and loop closure operate without blocking acquisition, with bounded memory and a documented degradation policy.

Numerical accuracy thresholds should be set only after obtaining repeatable hardware datasets and reference measurements.

## 13. Risks and unresolved decisions

| Issue | Risk / planned response |
| --- | --- |
| LD19 beam timing not fully hardware-synchronized | Accepted OD-003 angle/speed reconstruction and affine clock alignment; experimentally determine or bound physical packet timestamp delay. Do not claim sub-ms precision without evidence. |
| MPU6050 gyro bias and temperature drift | Stationary startup calibration, graph bias estimation, noise characterization, optional temperature model. |
| Acceleration and wheel slip | Conservative wheel noise, gyro disagreement monitoring, robust LiDAR constraints. |
| Corridors / repetitive geometry | Degeneracy-aware scan matching, multiple pose hypotheses in recovery, properly scaled covariances. |
| Moving obstacles / map changes | Robust residuals, dynamic rejection, read-only localized map with versioned remapping. |
| Ceres result correlation with IMU deskew | Avoid duplicating motion priors; conservatively estimate front-end covariance. |
| DeepRacer wheel telemetry availability | Verify actual hardware signals before specifying encoder factors; support Ackermann model. |
| Online graph growth | Asynchronous backend, keyframes, submap pruning, optional fixed-lag local estimation. |
| Offline/map-version mismatch | Version artifacts and reject mismatched calibration or coordinates. |

**Open decisions:** final robot drive type; availability of measured wheel speeds/steering angle on DeepRacer; Pico transport and clock-model tuning (the Pi-first/Pico-future approach is accepted under OD-002); exact LiDAR/IMU mounting; ROS 2 integration boundary; map binary serialization; and whether global relocalization uses a custom particle filter, Beluga, or hierarchical correlative matching.

## 14. Implementation milestones

1. **Foundation:** portable build, sensor schemas, clocks, raw recorder, deterministic replay, test harness.
2. **Motion and deskew:** gyro bias calibration, encoder/gyro continuous SE(2), LiDAR point timing, full deskew.
3. **Ceres frontend:** occupancy grids, distance fields, scan-to-submap matching, residual diagnostics.
4. **Offline GTSAM mapping:** wheel/gyro/LiDAR factors, submap graph, loop closure, batch refinement and versioned export.
5. **Live fixed-map localization:** Pi 5 tracking, state machine, global relocalization, map->odom correction, baselines.
6. **Online SLAM:** incremental GTSAM, asynchronous loop closing, bounded resource policy.
7. **Validation and tuning:** cross-platform reproducible benchmarks, calibration tools, sensor failure handling.

## 15. References

- [LD19 Development Manual v2.5 (protocol and timestamping)](https://device.report/m/258656c429c028a5e2f9f741a2cc1afd961fc02c06a36e51b5e0b80c7cb9341c)
- [LDROBOT LiDAR SDK](https://github.com/ldrobotSensorTeam/ldlidar_sdk)
- [Ceres Solver documentation](https://ceres-solver.readthedocs.io/)
- [GTSAM documentation](https://gtsam.org/)
- [GTSAM fixed-lag smoother example](https://borglab.github.io/gtsam/fixedlagsmootherexample/)
- [Google Cartographer algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)
- [Beluga adaptive Monte Carlo localization](https://github.com/Ekumen-OS/beluga)
- [MCAP file format](https://mcap.dev/)
