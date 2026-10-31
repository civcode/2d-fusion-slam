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
| FR-08 | Fuse LiDAR geometry and OD-006 wheel/gyro motion in OD-013 keyframe/submap GTSAM Pose2 graph; gyro bias remains in the local estimator initially, with OD-015 sparse offline graph bias knots optional after validation. |
| FR-09 | Support offline global mapping, loop closure, batch optimization, and refined map export. |
| FR-10 | Support low-latency localization against a previously built, read-only map. |
| FR-11 | Detect uncertain/lost localization; recover globally using OD-009/018 BnB search and OD-020 bounded persistent top-K multi-scan verification. Never accept ambiguous global fixes or map while untrusted. |
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

### 3.2 Accepted OD-005 — Partial revolutions, angular coverage and return safety

After validated LD19 packet decoding and **OD-003 individual beam timestamp reconstruction**, group generic observed beam slots into scans at **crossings of a fixed configurable angular seam**, using unwrapped angles and the validated rotation direction. A packet may straddle the seam; split between its observed beam slots without duplicating or fabricating readings. Do not assume a fixed number of points, constant rotation speed or a UART packet boundary.

- Support **partial scans** at start/stop, after packet loss, and during acquisition recovery. Preserve actual canonical acquisition times, start/end angular status, observed beam order, angular support, missing-sector intervals, largest gap, coverage fraction, source clock/epoch, intensity and validity metadata, packet errors, complete/partial flag and termination reason.
- Coverage MUST be computed **circularly against 360°** with a documented angular support model (bins or intervals); simple min/max angular spread is not sufficient. Missing sectors must remain missing, not filled with artificial range values or timestamps. If an outage can hide multiple angular seam crossings, invalidate/relock rather than invent turns.
- Classify each observed return as `VALID_HIT`, explicitly documented `VALID_NO_RETURN`, or `INVALID_RETURN`. A missing slot/sector is `MISSING` metadata and not a measurement. Keep geometry validity separate from timestamp validity: a time-valid invalid-range slot can contribute to **OD-004 midpoint time bounds** but never to geometric endpoints.
- **Only valid hits** may create occupied endpoints and freespace along the measured ray up to that hit. **Invalid and missing measurements MUST NOT clear cells.** A valid, device-documented no-return MAY clear a bounded ray only with an explicit calibrated sensor policy and enabled configuration. Undocumented zero, saturated, malformed and out-of-range measurements are **invalid by default**, not automatic max-range clear rays.
- Provide independent, configurable **scan-matching** and **map-insertion** quality gates using observed angular coverage, largest gap, spatial hit distribution/count, packet/time continuity, alignment uncertainty and pose quality. A partial scan MAY pass either gate when sufficient real data exists. Report the measured values and reasons for rejection; no fixed universal thresholds are claimed before benchmarking.
- For **normal mapping**, apply these gates and require **OD-004 interpolated motion coverage over the entire actual scan time interval**, including valid acquisition times of invalid-range slots. No extrapolated or clock-incoherent scan is inserted. Do not match a scan against a map containing itself.
- **Time-based scan segmentation is prohibited in normal operation.** A host-steady watchdog timeout MAY flush/abort and mark an incomplete scan for **stalled UART, disconnect, reset or shutdown recovery only**; such records must be explicitly marked partial/timeout and cannot masquerade as complete revolutions.

The **LD19 adapter** exclusively owns packet framing, CRC, device-angle and clock handling. The **portable scan assembler** consumes generic timed beams and gap events, and exposes quality-assessed `AssembledScan` records to the deskewer/front end. Neither portable algorithm code nor ROS 2 integration requires LD19 packet structures. For nonuniform partial data, do not fabricate uniform `sensor_msgs/LaserScan` fields; a correctly described point-cloud or native timed-scan representation is preferable.

See [accepted OD-005](open-design-choices.md), [Architecture `4.1.1](architecture-specification.md) and [OD-004 midpoint deskew](open-design-choices.md).

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

Time calibration must account for UART buffering, LiDAR packet timestamp convention, encoder interrupt/sample latency, IMU output/filter latency, and data-ready/FIFO semantics. Reject or flag unsupported interpolation across missing IMU samples. **OD-002 selects staged acquisition**, **OD-003 selects LD19 beam timing**, and **accepted OD-004 selects midpoint deskew and SE(2) exponential trajectory interpolation**. Exact clock calibration, trajectory gap/uncertainty tuning and replay checkpoint/jump policy (OD-024) remain open.

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

**Accepted OD-004:** The predictor MUST provide a **timestamp-addressable, continuous SE(2) odometry trajectory** behind a **ROS- and sensor-independent trajectory-provider interface**. It consumes timestamped wheel and bias-corrected gyro/IMU inputs, constructs **piecewise-constant body-frame twist** segments, and evaluates intermediate poses with the **SE(2) exponential map**:

~~~text
T_odom_base(t) =
  T_odom_base(t_k) * Exp_SE2(((t - t_k) in seconds) * hat(xi_k))
  for t_k <= t <= t_{k+1}
~~~

Here `xi_k = (v_x, v_y, omega)` is expressed in the robot body frame and rates are converted to SI units; each timestamp comes from the OD-001 canonical signed-nanosecond clock. Evaluate the exact constant-twist/curvature arc with a stable zero-yaw-rate limit, not linear interpolation of x, y and yaw. Preserve timestamp-domain/epoch identity, measurement validity, pose covariance and trajectory revision. The local `odom` trajectory remains smooth when the global map correction changes.

The provider MUST report whether a time or interval is **fully interpolatable**, extrapolated or unavailable. Consumers only query poses, coverage and uncertainty; they MUST NOT directly depend on LD19, MPU6050, Pico, encoders or ROS message types. A continuous SE(2) spline MAY later implement the same interface, but is not part of the initial algorithm.

### 5.1.1 Accepted OD-006 — Minimal motion estimator, bias and kinematics

**Initial estimator:** Use **measured wheel-derived translation plus bias-corrected gyro Z-rate**, maintaining a native state containing `T_odom_base ∈ SE(2)`, body twist `(v_x,v_y,omega)`, gyro yaw-rate bias `b_g` and **uncertainty**. A pose+bias error state `(δx,δy,δyaw,δb_g)` with 4×4 covariance including **pose–bias cross terms** is a suitable first model; verify covariance propagation and frame/tangent conventions. Bias evolves according to a calibrated noise/random-walk model. Integrate using the accepted OD-004 **piecewise-constant body twist and SE(2) exponential**, at actual asynchronous acquisition times.

- **Replaceable kinematics:** A generic native kinematic interface converts observed encoder/wheel/steering measurements into body translation, optional independent wheel yaw, quality and covariance. **Only use differential drive after verifying the real robot has independently driven/measured left/right wheels.** Ackermann requires valid **measured steering feedback**, not merely a steering command. When drivetrain or telemetry is unknown, report `UNCONFIGURED`/`DEGRADED` rather than secretly assuming a model. Preserve the provenance of raw wheel readings versus wheel-derived or already-fused odometry.
- **Stationary startup gyro calibration:** Estimate mean/robust Z-rate bias, variance and readiness from a verified stationary sample window, cross-checking wheel no-motion, gyro stability and accelerometer/gravity cues where available. Log duration, samples, timestamp, temperature if available and calibration revision. If stationary confidence fails, mark bias uninitialized/uncertain, grow yaw covariance and gate affected uses.
- **Carefully gated online correction:** Update the bias only when an independently confirmed stationary period or separately justified observable correction provides evidence. Use bounded rate/magnitude, consistency/hysteresis checks and covariance updates. Do **not** train gyro bias from wheel slip or gyro/wheel disagreement alone, and do not retroactively jump published local odometry; corrected offline reintegration produces a new versioned trajectory.
- **Wheel/gyro consistency:** Only if the kinematic model supplies **independent wheel yaw**, compare that yaw with bias-corrected gyro yaw over the **same aligned interval**, using uncertainty-aware thresholds. Slip, encoder faults or calibration errors must **increase uncertainty**, flag degraded state and be logged. Do not invent wheel yaw for unavailable sensors.
- **Missing data:** Absent wheel travel or gyro rate is not a measured zero; expose invalid/degraded trajectory coverage and propagate uncertainty. Respect OD-004 full interpolation for normal map insertion and optional 10 ms quality-gated prediction.
- **Independence and future replacement:** The estimator implements the ROS-free `MotionEstimator`/`TrajectoryProvider` interface without exposing MPU6050, Pico, encoder, LD19 or ROS types. EKF/error-state filtering and recent-state smoothing are deferred alternatives behind the **same API**, to be adopted only if benchmarks demonstrate value.

**Graph measurement accounting:** The fused predictor is used for Ceres initialization and beam deskew; it MUST NOT also enter GTSAM as a statistically independent prior if its source wheel/gyro measurements already contribute factors. Preserve provenance and conservatively handle deskew-conditioned LiDAR correlations.

**Verification:** Docker Catch2 tests for stationary/moving bias calibration, bounded bias updates, supported/unconfigured kinematics, wheel slip, missing sensors, covariance growth/cross-correlation, exact SE(2) arcs, replay resets and an interchangeable mock trajectory provider. Calibrated sensor/noise thresholds and actual robot drive layout remain unverified.

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

### 5.3 Per-beam full SE(2) deskew into scan midpoint

**Accepted OD-004** defines the scan reference as the **temporal midpoint** of all time-valid acquired beam slots in a scan (regardless of whether their range return is usable):

~~~text
t_start = min(valid beam acquisition timestamp)
t_end   = max(valid beam acquisition timestamp)
t_mid   = t_start + floor((t_end - t_start)/2)
~~~

Use checked 64-bit nanosecond arithmetic and require a common time-domain/epoch. The assembler must report coverage and partial-scan state; unknown or missing time boundaries are not silently replaced by first/last packet arrival.

For beam `p_i` in the instantaneous laser frame at `t_i`, express it in the laser frame at `t_mid` using the **continuous local odom trajectory**:

~~~text
p_laser_mid =
    inverse(T_odom_base(t_mid) * T_base_laser)
    * (T_odom_base(t_i) * T_base_laser)
    * p_laser_i
~~~

This must handle full translation and rotation, including the LiDAR-to-base extrinsic lever arm. Deskew is a derived view of the immutable raw observations; recalibration or offline trajectory/bias corrections can trigger re-deskew and new map revisions (OD-014).

**Full interpolation is mandatory for ordinary map insertion:** the OD-005 quality/insertion gate must pass for the assembled scan (including partial scans); then every timestamp-valid observed beam and the scan midpoint must be covered by valid **interpolated** trajectory segments across `[t_start,t_end]` with no unsupported sensor gap or clock-epoch change. If the motion history has not reached `t_end`, **prefer bounded delayed scan processing** to waiting forever or forecasting future motion. Incomplete coverage, invalid boundary timing or any extrapolated point prevents normal occupancy insertion; report the reason, and retain the raw scan.

**Optional bounded extrapolation:** Explicit latency-sensitive localization/prediction or diagnostics MAY request extrapolation; the default is **disabled**. Initial maximum horizon is **10 ms (10,000,000 ns)** beyond the earliest/latest trusted trajectory boundary. This is an upper bound subject to covariance, gyro/encoder freshness, slip/bias, source timestamp uncertainty and clock/calibration quality gates. Never extrapolate across a discontinuity. Return distinct `INTERPOLATED`, `EXTRAPOLATED` or `UNAVAILABLE` status with horizon and uncertainty. An extrapolated pose MUST NOT silently become valid map-insertion evidence. Choose buffering/quality thresholds by benchmark, not assumed device performance.

The deskewer depends only on the portable timestamp-addressable trajectory provider, timed beams, and extrinsics. The **LD19 adapter (OD-003)** handles packet timing and clock alignment separately; neither the deskewer nor trajectory provider imports sensor-specific or ROS types. A future SE(2) spline may replace the piecewise-exponential provider without changing the deskewer API.

## 6. Shared geometric frontend (Ceres)

### 6.1 Accepted OD-010 — 5 cm log-odds occupancy and overlapping submaps

**Map authority:** Use **0.05 m** cells (5 cm) with **explicit observed/unknown flags and log-odds probabilities**, in bounded local SE(2) grids with origin position **and yaw**. An observed cell may return to `p≈0.50`; it MUST NOT become indistinguishable from an untouched **unknown** cell. **OD-007 EDT and OD-009 correlation pyramids are derived versioned caches**, never substitutes for the occupancy grid.

| Parameter | Accepted initial setting | Validation |
| --- | --- | --- |
| Grid resolution | **0.05 m** | Compare optional 0.025 m (≈4× cell count) and 0.10 m (≈¼) |
| Writable active maps | **2 overlapping**, 1 during startup | Never more than 2 normally |
| New submap | After **90 accepted scan insertions** | Configurable |
| Freeze old submap | After **180 per-submap accepted insertions** | Configurable, safety may shorten |
| Overlap | **90 shared accepted IDs / 180**, ≈50% by insertion count | No implied physical 50% overlap |
| Prior `p` / cell state | **0.50**, `UNKNOWN` until first real update | Separate known mask required |
| Hit increment probability | **0.55** | Initial, not calibrated |
| Free increment probability | **0.49** | Initial, not calibrated |
| Probability clamp | **0.10 to 0.90** (observed cells) | Initial |
| Motion filter | Configurable minimum translation/yaw and elapsed time | No unverified numeric thresholds |
| Map spatial/RAM budget | **Hard bounded dynamic allocation** | Benchmark on Pi 5 |
| Frozen-map aging | **Disabled** | Separate future transient overlay only |

**Exact lifecycle:** After scan IDs 1–90 have been accepted and inserted into A, create B. Insert 91–180 into **both** A and B; then freeze A at 180 and create C. B receives IDs 91–270 and freezes at 180 insertions; C starts at 181. Only **accepted, quality-valid, motion-filter-passing scan insertions** advance a submap's own counter. A single scan may count once in **each** of two active maps, not twice in one map; rejected/filtered scans continue to be recorded and contribute **zero** insertions. Store stable scan-to-submap IDs, true insertion counts and transition order so scan 90/180 cannot be duplicated. Always match against a **pre-insertion map snapshot**, not the scan's own freshly added cells.

**Map bootstrap:** The first scan of an empty occupancy map cannot match against its own non-existent EDT. Initialize an explicit submap-local gauge pose and require valid, initialized OD-006 odometry/trajectory, OD-003 beam time and OD-004/005 scan geometry (or an external documented initial pose). Mark initial insertions **BOOTSTRAP_UNVERIFIED_MAP_MATCH** and do not claim LiDAR-observed global localization. Once sufficient mapped occupied support exists, use ordinary OD-007/008 pre-insertion matching. This is a narrowly scoped empty-map startup policy, never a fallback for an actively LOST robot. A newly overlapping map starts in the already established tracked frame.

**Inverse-sensor update:**

~~~text
logit(p) = log(p/(1-p))
L_prior = logit(0.50) = 0;  observed=false
Delta_hit  = logit(0.55)
Delta_free = logit(0.49)
L_next = clamp(L + Delta_hit_or_free, logit(0.10), logit(0.90))
observed=true only on a valid actual update
~~~

From an accepted, OD-004 midpoint-deskewed scan, transform the observed laser origin/endpoints to **each active submap's local frame**. For `VALID_HIT`, cast **free only strictly before** a measured endpoint and mark its cell **hit**. `VALID_NO_RETURN` may clear **only a measured, bounded ray** after documented and enabled sensor semantics. `INVALID_RETURN` and `MISSING` (OD-005) generate **zero free and occupied updates**, including lost sectors or invalid zero/out-of-range returns.

**Within a single scan insertion into one submap**, accumulate unique hit-cell and traversed-free-cell sets, give **hit priority** when a cell is in both, then apply **at most one update per cell**. Crossing rays or duplicate beams must not overcount or erase a hit. A later accepted scan may update it again. Maintain intensity/range diagnostics without promoting an unsupported return to a free-space observation.

**Motion filter and bounded allocation:** Apply configurable pose translation/yaw changes and optional maximum elapsed **acquisition** time relative to the last *accepted map insertion*. Filtering only affects mapping, never acquisition/recording, OD-006 motion, or pose outputs. Support capped dynamically allocated map extents/tiles, maximum cells/RAM and allocation-safe stop. On cap/travel/allocation failure, **early freeze with explicit actual count/overlap or stop insertion safely**, not unlimited growth or false obstacle/cell creation at boundaries. Hardware thresholds are pending measurement; do not assert every 10 Hz LD19 scan qualifies for insertion.

**Immutability and graph:** Freezing makes the **local occupancy raster, observed mask, local coordinate origin and insertion history immutable**. Graph optimization changes a separately versioned global `T_map_submap`, not frozen occupancy. Cached local EDT/correlation fields remain valid while source raster/threshold/known-mask revisions match; global indexes may update with pose changes. Active matchers read **accepted OD-011 coherently published immutable snapshots**, with bounded staleness, full-rebuild baseline and eventual incremental EDT; localization remains read-only. No dynamic aging of frozen data; a future transient overlay must have independent lifetime and artifacts.

**Modules/tests:** Native `OccupancyGrid`/`SubmapBuilder`/`FrozenSubmap`/`MapSnapshot` are **ROS-, Ceres- and GTSAM-independent** with typed SE(2) frames and versioned provenance. Docker Catch2 tests must cover insertion sequence 90/180 and overlapping IDs, motion filter skipped scans, logit arithmetic/clamp, known 0.5 vs unknown, invalid/missing/no-return no-clearing, per-insertion hit precedence, rotated ray geometry, 0.05/0.025/0.10 m accuracy/memory, hard caps/early freeze, frozen map hashes despite pose-graph updates, EDT/pyramid source revisions and no self-matching. Benchmark B18 measures map accuracy, CPU/RAM and lifecycle reliability on apartment/warehouse and Pi 5. See [OD-010](open-design-choices.md) and [Architecture Section 6](architecture-specification.md).

### 6.1.1 Accepted OD-011 — Coherent map snapshots and incremental EDT cache contract

The **OD-010 5 cm log-odds occupancy/observed mask** remains authoritative. A versioned **truncated unsigned EDT (OD-007)** and **separate correlative-score pyramid (OD-009)** are derived, rebuildable representations for matching. **Final design:** active-submap dirty-region tracking and correct incremental **32×32 output tiles**, reusable immutable frozen EDT/pyramids, bounded evict/reload. **Implementation stage 1:** publish atomically coherent snapshots after a **bounded periodic full exact EDT and score-pyramid rebuild** (not a synchronous every-scan rebuild). Stage 2 adds tiled updates only if B19/Pi profiling demonstrates a benefit.

| Parameter | Initial value |
| --- | --- |
| Occupancy grid resolution | **0.05 m** (OD-010) |
| **Unsigned EDT** truncation `R` | **0.30 m** (configurable); not a signed TSDF |
| EDT interior tile | **32 × 32 cells**, 1.6 m × 1.6 m at 5 cm |
| Tile source halo | **≥R plus bilinear interpolation support**; ≥6 cells plus ≥1 stencil cell at 5 cm, with coordinate rounding/boundaries |
| Build execution | Background task, bounded CPU/queue/memory budget |
| Full rebuild fallback | On-demand or bounded refresh cadence, not every raw scan |
| Stale complete snapshots | Explicit maximum steady-clock age **and** accepted insertion lag; numerical limits to benchmark |
| Frozen data | Immutable local map and cached fields; evict only unpinned derived fields, reload/rebuild as needed |

**Dirty updates:** After an accepted OD-010 insertion, track occupancy-source threshold transitions **into and out of** confidently occupied and observed/unknown support changes. Every changed source can affect truncated EDT output within **R metres**; mark all output tiles intersecting the expanded change region, accounting for interpolation stencils. For each dirty interior, recompute against the **current occupancy snapshot**, using a source halo `≥R + interpolation` so obstacles outside the tile can influence values. **Removals are not just negative insertions**: if a former nearest obstacle disappears, find the new nearest source from all remaining nearby candidates. On map extent/origin/confidence threshold/algorithm changes or excessive dirty work, fall back to full rebuild. Validate against a full exact Euclidean distance transform (e.g. Felzenszwalb–Huttenlocher linear-time algorithm).

**Separate correlation-pyramid dirty logic:** A changed log-odds score can invalidate OD-009 coarse matching even when no cell changes occupied threshold and the EDT stays the same. Update affected fine scores and all dependent multi-resolution upper-bound aggregates, including **decreases** and known-mask changes. Incomplete or inadmissible upper bounds MUST never be used for a claimed exact branch-and-bound result.

**Atomic snapshot composition:** A native `MapSnapshot` includes submap ID, monotonically increasing occupancy revision/content hash, immutable occupancy and known-space support, compatible EDT and pyramid **source revision/configuration hashes and readiness**, local origin/resolution, insertion high-watermark, and separately versioned or pinned graph pose. Publish only **ready, coherent versions** atomically; **do not use new occupancy N+1 with EDT N**. Readers pin one immutable handle for the entire Ceres optimization or asynchronous OD-009 search. Partially built data stays private, and stale background builds cannot overwrite newer snapshots. Operation-specific `NOT_READY` is valid when a required cache is missing.

**Staleness and memory:** A complete *older* snapshot may be used within configured **maximum steady-clock age and accepted insertion count lag**, recording age/lag and applying OD-008 quality gates. Beyond either bound, defer/reject map matching/insertion rather than fabricate current cache data. Maintain OD-010 **match-before-insert**, so the current scan cannot be included in its own matching map. Account for pinned readers, tile copy-on-write pages, builder scratch, two active maps and frozen fields in bounded memory; no eviction of live pinned snapshots. Overload causes lower-priority build/search jobs to yield/cancel, while **sensor acquisition, recording and OD-006 odometry continue**.

**Frozen fields and graph:** When OD-010 freezes occupancy, enqueue final EDT and OD-009 pyramid generation on a bounded worker; expose final frozen matching snapshot only when coherent and ready. Keep frozen derived fields immutable; cache frequently accessed entries and evict unpinned cold copies. Reconstruct/reload on demand when missing or hash/config mismatches. A change to global `T_map_submap` does **not** invalidate local-grid fields; global spatial indexes and pinned transforms must reflect the pose revision separately.

**Hardware and domain boundaries:** `MapSnapshotProvider`/`SnapshotHandle` and cache metadata are **native ROS-/Ceres-independent** interfaces. Ceres/OD-009 implementations consume immutable versioned data; neither owns map-mutation locking. At **40×40 m and 0.05 m**, there are **640,000 cells**, requiring **2,560,000 bytes (~2.56 MB, 2.44 MiB)** for one float32 EDT alone. Include other masks/pyramids and concurrent snapshots when sizing Pi RAM; measure full rebuild vs incremental tiles before optimization.

**Tests:** Docker Catch2 compares tiled and exact full EDT after hit-source **addition/removal**, tile seam/edge and known-mask transitions, different grid origins/resolutions and bilinear interpolation; OD-009 pyramid score changes without EDT dirty events, admissible upper bounds, snap coherence, concurrent pinning/cancel/reload, age/insertion staleness caps, freeze graph pose changes and memory/CPU backpressure. Benchmark **B19** measures Pi 5 CPU/RAM and matching p95/p99, 10 Hz capture interference, cache reuse, tile regeneration and full rebuild fallback. See [OD-011](open-design-choices.md), [architecture `6.3.1](architecture-specification.md) and [Felzenszwalb–Huttenlocher](https://toc.cs.uchicago.edu/articles/v008a019/).

### 6.2 Accepted OD-007 — Ceres EDT scan-to-submap optimization

The native **occupancy grid is the authoritative map representation**. A **truncated unsigned Euclidean distance field (EDT)** is a **derived, versioned immutable scan-matching cache**, built from a configurable confidence threshold applied to **occupied cells**. Unknown cells are **not** obstacles or evidence of free space; separately track the known/supported map mask and mask invalid interpolation stencils. Preserve the grid's translation/rotation origin, resolution, occupancy probabilities and revision. Cache metadata records source occupancy revision/checksum, confidence threshold, interpolation mode, truncation and support-mask policy. **Accepted OD-010** determines authoritative log-odds and freezing; **accepted OD-011** requires atomic coherent snapshots, active-tile target/fallback full builds and frozen cache eviction/reload. Exact work/staleness budgets remain unmeasured.

For OD-005 valid-hit beams deskewed via OD-004 to the **midpoint laser frame**, optimize the **midpoint robot pose** `T` with laser extrinsic `T_base_laser`, a pinned map-frame EDT `D`, and the OD-006 odometry prediction `T_pred` and covariance `Sigma_pred` (same frame/time/tangent convention):

~~~text
q_i(T) = inverse(T_map_grid_or_submap) * T * T_base_laser * p_i
D(q)   = clamp( unsigned_Euclidean_distance_to_confident_occupied(q), 0, D_max )
r_i(T) = sqrt(w_i) / sigma_i * D(q_i(T))

e_prior = Log( inverse(T_pred) * T )
L^T L   = inverse(Sigma_pred)

E_Ceres(T) = 0.5 * sum_i Huber_delta( r_i(T)^2 )
           + 0.5 * || L * e_prior ||^2
           + 0.5 * sum_i r_boundary_i(T)^2
~~~

- `w_i` is validated nonnegative beam-quality weight and `sigma_i > 0` is effective endpoint/map uncertainty in **metres**. `D_max` is a configurable metre-scale truncation. Huber `delta` is positive, in normalized residual units; calibrate its actual value with synthetic and real data.
- **One scalar Ceres residual block and one `ceres::HuberLoss(delta)` per beam**. A single residual block containing all beams would robustify the *whole vector*, not each outlier. Emit **unsquared `sqrt(w_i)/sigma_i * D`**; Ceres internally squares and robustifies each block. Ceres's global factor 1/2 is conventional.
- Implement a **separate three-component whitened SE(2) motion-prior residual** using a checked positive-definite predicted covariance and a documented tangent-space convention. The odometry prior is part of the **frontend regularized objective**, not merely initialization. Its contribution and underlying sensor provenance MUST be reported separately from LiDAR-only residuals.
- **Out-of-grid samples:** Add a **separate, finite, differentiable or piecewise differentiable outward-increasing boundary penalty** (e.g. scaled hinge on metric outside distance) in map-local coordinates, with gradients leading back into support. Do not clamp outside coordinates to a fixed EDT edge value or treat inside-grid unknown cells as confident matches. Unknown regions are masked, not treated as zero distance/free space.
- **Bilinear interpolation** is the initial choice. It is continuous and bounded between neighbor values, though derivatives change at cell boundaries. Explicitly validate rotated grid origins, cell-edge derivatives and unknown-neighbor masking; bicubic `BiCubicInterpolator` remains a future **optional** benchmarked alternative, as do occupancy-probability, signed-distance and feature objectives behind the same public native `ScanMatcher` interface.
- **Ceres must remain a private implementation dependency:** shared domain-level scan/match/result/map interfaces MUST NOT contain Ceres classes, cost functions, loss functions or ROS 2 types. Avoid introducing Boost without case-by-case approval.

**Acceptance:** Require sufficient valid observed-map overlap, enough nontruncated/spatially distributed beams, acceptable **LiDAR-only** residual statistics, successful solver termination, consistency against the predicted motion, and sufficiently constrained **LiDAR-only geometric Hessian** directions. Inspect observability **excluding** both odometry-prior and artificial boundary information; strong odometry cannot mask scan degeneracy. Reject empty occupied EDTs, unsupported/unknown-dominated scans, boundary-dominated solutions, nonfinite covariance and poor-quality corrections, with typed reasons. **Accepted OD-008** defines robust LiDAR-only rank/uncertainty and held-out calibration; numeric thresholds and inflation remain empirical.

**Backend information accounting:** The optimized pose is a **wheel/gyro-prior-regularized posterior**. If GTSAM also has wheel/gyro factors from the same samples, **do not add that posterior as an independent LiDAR factor**. Preserve separate LiDAR geometry/information, odometry prior pose/covariance/source IDs, boundary penalties and immutable map/EDT revisions; **OD-008** governs quality and rank-aware measurement information; **Accepted OD-013** now fixes keyframe/submap topology, one fused motion factor per interval and conservative LiDAR geometry admission; correlated uncertainty remains empirically validated. Do not assume subtracting prior Hessian creates an exact independent LiDAR covariance.

**Test requirements:** Docker Catch2/Ceres integration tests must cover EDT Euclidean metrics, truncation and map yaw; bilinear values/derivatives/masks; one-block-per-beam Huber outliers; weight/sigma scaling, Lie Log prior whitening; out-of-grid boundary gradient; unknown and empty maps; overlap/degeneracy and strong-prior non-observability; synthetic apartment and symmetric warehouse false locks; repeatability and Pi 5 runtime. See [accepted OD-007](open-design-choices.md) and [Architecture Section 5](architecture-specification.md).

### 6.3 Accepted OD-008 — Robust LiDAR information, degeneracy and covariance

**Ceres owns numeric optimization and Jacobian extraction; a separate `MatchQualityEvaluator` owns acceptance, information-rank classification and uncertainty calibration.** The quality module operates only on native C++20/Eigen types, with **no Ceres, ROS or device dependency**. Its input is the OD-007 optimized SE(2) pose and **per-beam LiDAR-only normalized scalar residual/Jacobian** in an explicitly documented tangent frame; prior and artificial map-boundary residuals are NOT part of the LiDAR evidence.

For each valid, map-supported OD-005 hit, let `r_i=sqrt(w_i^beam)*D(q_i)/sigma_i` and `J_i=∂r_i/∂delta_xi` evaluated at the optimized midpoint pose. Construct a **PSD robust Gauss–Newton information approximation**:

~~~text
alpha_i = rho_delta'(r_i^2)  // individual Huber influence, not beam weight
H_L = sum_i alpha_i * J_i^T * J_i
~~~

`w_i^beam` already appears **inside** `r_i`/`J_i` and MUST NOT be applied again. Exclude OD-007 motion prior, out-of-grid boundary penalty, unknown cells, invalid returns and noninformative/truncated endpoints. The robust second-derivative `rho''` term is omitted to maintain a **PSD information diagnostic**, not because it is mathematically identical to the solver's true Hessian. Evaluate/manifold-transform Jacobians correctly; Ceres `Covariance` is a cross-check, not an automatically calibrated measurement error.

**Normalize physical axes before eigenanalysis**: choose and record `delta_xi=S*z` with configured metre/radian tangent scales, and decompose `H_norm=S^T H_L S = V Lambda V^T`. Publish eigenvalues, rank (0–3), condition, eigenvectors mapped back through `S`, normalized scales and tangent convention. Evaluate absolute/relative rank gates and uncertainty. Unscaled mixed-unit eigenvalues MUST NOT be used as a universal observability threshold.

- **Fully observable rank 3:** `Sigma_raw≈H_L^-1`, then `Sigma_match=K Sigma_raw K^T + Sigma_floor` using empirically calibrated inflation and symmetric PSD floors, with source/reference calibration revision. Verify finite positive-semidefinite/positive-definite result and correct frame conversion; this approximation does not fully account for correlated beams, map error, OD-004 wheel/gyro deskew conditioning or model mismatch.
- **Rank 0–2:** Return explicit observable/null bases and rank-aware information or observable-only covariance. Null directions have **unknown/unbounded uncertainty**, not zero variance. **Never** publish a pseudo-inverse with zero null variance or pad a covariance floor into a fictitious full-rank LiDAR constraint. An optionally **projected** pose correction may retain only observable components relative to OD-006 prediction in normalized tangent coordinates, then be revalidated. Otherwise **reject** and retain odom-only state with degraded global pose. OD-013 must define any rank-aware graph factor separately.
- **Independent acceptance gates:** Require solver convergence/finite state; enough spatially distributed hit returns and observed-map overlap; acceptable raw/Huber LiDAR residual distribution and outlier fraction; meaningful nontruncated information; rank/conditioning; conservative prediction innovation with **correlated** deskew/odom uncertainty; and valid timestamps, gyro bias/slip, calibration and occupancy/EDT revisions. Strong OD-007 odometry regularization **cannot** make a corridor LiDAR-observable.
- **Native result:** Return pose, `ACCEPT_FULL` / `ACCEPT_PROJECTED` / `REJECT` / `UNAVAILABLE`, full covariance **only when credible rank 3**, otherwise observable subspace and unavailable/null status, plus eigen-directions, rank, overlap/residual/innovation metrics, timing and versioned covariance calibration. Rejected matches retain diagnostics. A ROS adapter must avoid inventing confident covariance for unknown directions.
- **Calibration:** Fit `K`, `Sigma_floor` and gating thresholds using held-out synthetic apartment/warehouse and genuine independently referenced trajectories, with corners, corridors, open scenes, partial scans, varied speeds and degraded IMU/encoders/LD19 time. Track coverage/NEES (when defensible), false-accept/reject, rank fidelity and holdout leakage. Optional bootstrapping is an offline validation aid and must account for beam/map correlations.

**Tests:** Docker Catch2 unit tests of known-rank 3×3 Jacobians, robust influence/weight distinction, PSD, normalization/frame conversion, strong prior without LiDAR rank, null-space projection/rejection, covariance floors only in observable rank-3 cases, poor time/gyro/mapping quality and consistency metrics. The native evaluator must build without Ceres; separate Ceres-linked tests verify Jacobian extraction and optional Ceres covariance comparison. See [OD-008](open-design-choices.md) and [Architecture `5.3.1](architecture-specification.md).

### 6.4 Accepted OD-009 — Mode-dependent correlative initialization and BnB recovery

**Purpose:** OD-007 Ceres continues to provide precise local scan-to-EDT refinement; OD-009 supplies **robust initialization and recovery** using a separate, ROS-independent correlative score on **versioned immutable occupancy-derived map pyramids**. The coarse score is independent of the Ceres EDT objective and is **not** itself an accepted pose measurement. All candidates MUST be refined under the mode-appropriate OD-007 prior and validated by **OD-008** residual, geometric observability, overlap, innovation and uncertainty gates before global correction.

| Mode | Initial search range | Target CPU budget | Execution |
| --- | --- | --- | --- |
| Normal TRACKING | OD-006 predicted midpoint pose -> Ceres, no coarse search | **0 ms coarse** | Synchronous local Ceres only |
| DEGRADED/UNCERTAIN | **±0.3 m X/Y and ±10° yaw** | **10–20 ms** when invoked | Bounded local correlative sampling |
| RECOVERY | **±1.0 m X/Y and ±30° yaw** | **30–50 ms** per invocation | Multiresolution branch-and-bound |
| GLOBAL_RELOCALIZATION | Full frozen candidate submap(s), **360° yaw** | **100–200 ms per slice** | Asynchronous, cancellable/resumable BnB |

These are **unmeasured Pi 5 starting targets** around 10 Hz LD19, not guaranteed deadlines. Measure host **steady clock** (never pausable ROS simulated `/clock`) and cap evaluated candidates, CPU work, memory, queue occupancy, and cancellation separately. Report search `COMPLETE`, `PARTIAL_BUDGET`, `CANCELLED` and `INVALID`, with progress and evaluated/pruned nodes. **Incomplete search does not mean no match exists**.

**Search windows:** Derive local candidate extents from the OD-006 covariance in a documented SE(2) tangent frame, clip to each mode cap and report clipping. A stale/singular predictor triggers degraded/global policy, not false certainty. Select angular steps based on map spatial resolution and useful scan range (approximate `delta_yaw <= spatial_step / representative_range` for small yaw). Treat ± windows as pose-search bounds, not sensor range bounds. Avoid sampling sparse yaw so an informative endpoint skips entire occupancy cells.

**Search pyramids:** Build precomputed multiresolution **correlative-score** pyramids from **supported occupancy probability** on immutable map/submap revisions, with known/unknown masks, full map origin/yaw, score policy and source revision/checksum. These are separate from OD-007's truncated unsigned EDT cache and may have a different score objective. **Reuse** snapshot pyramids across scans, invalidate/rebuild when the underlying occupancy revision changes, and cap memory/build work. Do not let unknown/out-of-map, OD-005 invalid/missing beams or extrapolated OD-004 motion generate false support.

**Branch-and-bound correctness:** When claiming exact BnB pruning, pyramid cells MUST provide **admissible upper bounds of the fine-resolution correlation score** for all child translations at each sampled rotation. A max-over-children/precomputation strategy is suitable if proven with known masks, truncation and quantization; an average score is not necessarily an upper bound. Test pruning and best candidate against exhaustive search over a small bounded discrete grid. Budget-terminated search does not inherit complete-search optimality guarantees.

**Multiple hypotheses:** Keep a configurable number of **spatially distinct** top-scoring candidates, deduplicated by SE(2) separation, for repeated shelving, corner symmetry and kidnapping. Candidate scores are ranking aids, **not covariance or OD-008 acceptance**. In global recovery, perform **multiple-scan temporal confirmation** under OD-020/022 before publishing a stable map correction.

**Important prior distinction:** OD-007 includes an odometry prior during normal tracking. In global recovery, a falsely tight stale odometry covariance can pull correct distant hypotheses back toward the wrong pose. Use **explicitly mode-dependent relaxed/invalid/no-prior refinement** with OD-008 state-aware innovation and multi-scan consistency, rather than imposing a tracking prior on a kidnapped robot.

**Asynchronous execution and safety:** Run large searches in bounded workers with pinned immutable map/score pyramid, scan, calibration and source-clock epoch. A partial frontier can resume only with matching revisions; otherwise cancel/restart. Acquisition, recording, local odometry, normal local scan matching and ROS callbacks MUST NOT wait on a global search. While LOST/RELOCALIZING or without validated OD-008 correction, keep odom publishing, mark global pose uncertain/unavailable, and **disable map insertion**.

The **native `PoseCandidateSearch`** interface returns ranked candidates, search mode, score/overlap, map/time/calibration revisions, budget/progress and status. It depends on no Ceres types, ROS messages or hardware drivers. Its adapter/coordinator calls OD-007 Ceres and OD-008 quality evaluator separately. **Accepted OD-020** adds bounded persistent top-K temporal verification after BnB; particle filtering remains a conditional later backend only if B28 demonstrates shortcomings.

**Verification:** Docker Catch2/CTest: covariance bounds, angular sampling, rotation/map origin and 360° wrap, per-scan zero coarse work when TRACKING, precomputation cache reuse/invalidation, admissible BnB bounds vs exhaustive small search, top-K distinct candidates in repeated aisles, wrong-odometry-prior recovery, OD-008 quality rejection, partial-budget/yield/cancel and no blocked acquisitions. Pi execution budgets need on-device measurement, not inferred Docker x86 times. See [accepted OD-009](open-design-choices.md), [architecture `5.5](architecture-specification.md) and [Cartographer 2D fast correlative matching](https://google-cartographer.github.io/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.html).

## 7. Accepted OD-013 — Keyframe and submap SE(2) GTSAM pose graph

### 7.1 Initial graph state, residuals and ownership

The offline backend MUST optimize **global keyframe poses `X_k=T_map_base(t_k)` and global frozen/active-submap poses `S_j=T_map_submap_j`** as planar `Pose2` nodes. Keyframe timestamps refer to **OD-004 scan midpoints** and **accepted OD-015** motion/quality selection; submap frames refer to **OD-010 immutable local occupancy coordinates**. All graph measurements use explicit SE(2) Lie-tangent/covariance conventions. Choose **one gauge reference** to avoid a singular graph, without counting a submap frame-definition/anchor relationship as an independent sensor observation.

- **One fused wheel/gyro relative-motion factor per keyframe interval** from OD-006, with proper covariance and source samples. No simultaneous raw encoder, separate gyro-yaw, and already-fused odom factors from the same interval. Unavailable wheel/gyro evidence must not silently become full 3-DoF certainty.
- **At most one primary local LiDAR keyframe-to-submap factor per keyframe/encounter**, admitted only from **separate LiDAR-only geometric evidence**, not OD-007's odometry-conditioned posterior pose. Preserve both insertion memberships when scans are included in two active OD-010 submaps.
- **Validated loops** to submaps **not containing the query scan**, with overlap/shared-evidence deduplication, OD-008 geometry/observability and robust loss after validation.
- **No GTSAM bias state or joint trajectory** initially; OD-006 handles stationary/online gyro bias, high-rate motion and OD-004 deskew outside the graph. Full joint state, optional **OD-015** 5–10 s sparse scalar yaw-bias knots and planar bias-aware motion factors are deferred offline extensions requiring OD-014 validation; live iSAM2 remains OD-019.
- **OD-013 full keyframe/submap graph remains batch-first offline**. **Accepted OD-019** adds an online sparse global **submap-only iSAM2** graph as Stage I; the separate local fixed-lag GTSAM smoother is conditional.

For an admitted local LiDAR relative pose `Z_jk = T_submap_j_base(t_k)`:

~~~text
Z_pred(j,k) = inverse(S_j) * X_k
r_lidar(j,k) = Log( inverse(Z_jk) * inverse(S_j) * X_k )

Z_odom(k,k+1) = OD-006 fused wheel/gyro relative pose
r_odom = Log( inverse(Z_odom) * inverse(X_k) * X_(k+1) )
~~~

Use rank-aware covariances in **the same tangent frame as each residual**, tested Jacobians and frame conversions. Submap frame origin/anchor keyframe is **deterministic initialization/bookkeeping**, not an extra statistically independent likelihood. An anchored submap must still be allowed to move globally when loops optimize `S_j`.

### 7.1.1 Accepted OD-015 — Keyframe admission, independent submaps and future bias knots

**Initial keyframe selection:** Use native, ROS-/GTSAM-independent, deterministic `KeyframeSelector` on OD-004 **scan-midpoint** SE(2) estimates with valid OD-001 nanosecond clock-domain/epoch identity and OD-005 scan eligibility. Since the **last accepted graph keyframe**, compute relative pose translation `||trans(T_last^-1*T_now)||` and shortest yaw `abs(wrap(yaw))`. Request a keyframe if any of the **0.20 m distance**, **5° (pi/36 rad) rotation**, **2.0 s elapsed during meaningful motion** or a bounded uncertainty/information-supported trigger is met. Enforce **0.2 s minimum interval** for ordinary admission; log deferred candidates and test again when the interval permits. Initial gauge/root and explicit OD-010 submap-anchor keyframes may override the floor with a recorded exception. These are **configurable initial values, not empirically optimized defaults**.

**Stationarity/time semantics:** The 2 s fallback requires verified meaningful motion above a configurable noise floor; **no repeated time-only keyframes while stationary**. OD-006 raw IMU bias calibration continues independently. A paused `/clock` is not host wall-time motion; never calculate a keyframe `Delta_t` across distinct OD-001 clock epochs or a missing midpoint pose. Reject/regate invalid temporal data; preserve original raw scan/motion IDs for deterministic OD-014 reconstruction.

**Separate node creation and factor quality:** OD-008 overlap/residual, robust LiDAR-only rank/observable directions and covariance calibration control whether an admitted `X_k` can carry a LiDAR graph factor. If OD-006 uncertainty rises but OD-008 geometry is degenerate, avoid graph-node flooding and activate OD-009/022 degraded/recovery, while still preserving fused relative odometry continuity where necessary. One **fused wheel/gyro motion constraint per actual adjacent-keyframe interval** is computed from contributing source samples, with no duplicate factors after a deferred/forced node. A keyframe with rejected LiDAR geometry may retain a well-supported odom factor and a typed `NO_LIDAR_FACTOR` reason, not an artificial rank-3 observation. Only one primary local keyframe-to-submap scan factor per source encounter as in OD-013.

**Submap association independent of keyframe density:** OD-010 starts new submaps at **90 accepted map insertions** and freezes at **180 per-submap insertions**, **not** every N keyframes. Each submap has its own freely optimizable `S_j` with one associated anchor keyframe; at its birth timestamp associate a suitable existing `X_k` or create a separately labeled `SUBMAP_ANCHOR` even if the normal 0.2 s minimum would defer it. Anchor relationship is frame initialization/bookkeeping, **not an extra sensor likelihood**. Maintain both A/B scan memberships and one primary LiDAR constraint. Varying keyframe thresholds may change graph node counts, but MUST NOT change the map insertion count solely through graph scheduling or create artificial overlapping evidence.

**Bias state in initial graph:** Exactly **zero GTSAM gyro-bias knots**; evolving yaw gyro bias remains in OD-006 local state and is reflected in the fused interval motion estimate, OD-004 deskew and covariance. Future OD-014 *optional offline* joint model may introduce sparse scalar knots `b_j` initially every **5–10 s** only with independently observable drift/temperature dynamics:

~~~text
b_(j+1) = b_j + w_j
w_j ~ Normal(0, q_b * Delta_t_j)
Var(w_j) = q_b * Delta_t_j     // q_b in (rad/s)^2 / s
~~~

The future knot random walk MUST be accompanied by a **planar wheel/gyro bias-coupled relative-motion factor**, correct source time/source-ID integration and tested yaw/bias Jacobians; a random-walk-only bias variable is unidentifiable from pose. **Replace**, not duplicate, corresponding baseline OD-006 fused odometry factors if introducing bias-aware constraints. GTSAM's general IMU preintegration is not assumed equivalent to this planar wheel+gyro fusion. Maintain calibrated bias uncertainty and inter-factor motion/deskew correlations; do not let arbitrary bias knots absorb wheel slip or unknown LD19 time delay. Reject unobservable updates, benchmark against held-out truth, and use OD-014 selective re-deskew/remap and OD-012 versioning for any accepted refinement.

**Required native result and telemetry:** `KeyframeDecision` includes trigger type(s), SE(2) delta and midpoint time, min-interval decision, uncertainty/OD-008 support, admitted/dropped status and reason, original motion source interval, selected submap ID/anchor, membership and correlation group, bias model/config revision. Public domain contracts MUST NOT include ROS, GTSAM, Ceres or MPU6050/encoder device classes. **B23** and Docker Catch2 test exact 0.20 m/5°/2.0 s/0.2 s boundary cases, yaw wrap and clock jumps, static no-flood behavior, poor LiDAR factor abstention, two-submap anchor independence, one fused factor per interval and optional 5/10 s random-walk variance calibration.

### 7.2 Correlation and graph-factor admission

**Standard separate-factor noise does not imply independence.** When wheel/gyro motion is also used for LiDAR beam deskew, geometric match errors and fused odometry errors have cross-covariance:

~~~text
Sigma_joint = [ Sigma_odom     Sigma_cross ]
              [ Sigma_cross^T  Sigma_lidar ]
~~~

This correlation is not removed by providing `Sigma_odom` and `Sigma_lidar` separately. **Initial approximation:** retain source time intervals/calibration revisions that influenced each deskewed beam, use **OD-008 held-out-calibrated conservative LiDAR uncertainty inflation**, and gate on correlated evidence. Inflation can reduce overconfidence but **does not mathematically model the cross term**. If the remaining error is unacceptable, test a truly joint correlated factor or fuller state estimator as a separate future design.

**OD-007 Ceres prior contamination:** Ceres minimizes LiDAR geometry plus an **OD-006 wheel/gyro odometry prior** and boundary penalties. Its final optimized pose and posterior Hessian MUST NOT be exported as an independent LiDAR-only graph measurement beside the fused motion factor. Merely subtracting the prior Hessian does **not** isolate a statistically independent LiDAR covariance. Obtain a separately evaluated **LiDAR-only geometric relative pose**, e.g. via prior-free/observable-subspace refinement around a valid candidate, or a properly modeled conditioned observation; retain OD-008 residual/rank/covariance and map provenance. If not reliable, skip the LiDAR graph factor while permitting the frontend pose as a prediction/diagnostic.

**Overlap:** OD-010 inserts accepted scan IDs 91–180 into both A and B. Preserve this **two-submap membership** but choose **one primary local scan factor per keyframe** rather than counting both as independent. Multiple matches to overlapping maps and repeated encounters require source-scan intersection/grouping, deduplication and conservative covariance or rejection. The presence of a shared `S_j` variable accounts for submap-pose uncertainty, **not correlated occupancy-map raster errors**.

**Self-matching:** An OD-011 map pinned for local matching must be a **pre-insertion** source, not a raster already containing the current scan. New loop closures MUST exclude any target submap in the query scan's insertion membership. Reusing a scan against a map containing itself creates circular evidence, even if the geometry fits perfectly.

**Degeneracy:** OD-008 rank 0–2 information is not a finite full 3×3 pose covariance. **Reject** such LiDAR/loop factors by default until a tested **observable-subspace factor** constrains only supported directions; do not use a pseudo-inverse to fabricate nullspace confidence. OD-008 robust residual/overlap/innovation gates and additional loop checks apply **before** robustification. Repetitive aisles, dynamic clutter and loop inconsistency require rejection/rollback policies rather than assuming Huber/GTSAM robust losses solve false closures.

**Factor provenance:** Every native factor carries stable factor/keyframe/submap IDs, measurement type, timestamp interval and source raw encoder/gyro IDs, query scan IDs, insertion memberships, OD-004 deskew sample intervals, OD-007 prior/geometry model details, OD-008 calibration/rank, OD-010/011 map-snapshot revision/hash, correlation-group ID, duplicate/overlap classification and acceptance/rejection reason. OD-012 persists this portable graph state, **not native GTSAM factors**. Native `PoseGraphModel`/`GraphFactorSpec`/`GraphAdmissionResult`/`GraphOptimizationResult` interfaces are **GTSAM- and ROS-independent**.

### 7.3 Graph lifecycle, publication and acceptance

Graph factor admission occurs **after** OD-007 geometry refinement and OD-008 rank/quality checks; no raw optimizer output bypasses the provenance/duplication guard. A pose graph includes one documented gauge reference, keyed `X_k/S_j`, fused interval odometry, at most one primary local LiDAR factor per encounter and non-self verified loops. **Accepted OD-015** chooses motion/quality keyframe timing and independent anchors; **accepted OD-018** governs spatial/signature frozen-submap loop retrieval, OD-019 live iSAM2, and OD-014 optional versioned offline re-deskew/remap.

Run initial **GTSAM batch LM/Gauss–Newton** on an offline worker, not sensor acquisition threads. **Accepted OD-019** independently adds sparse global submap iSAM2 during online mapping; a second local graph remains optional. Verify finite errors and a consistent graph revision before publication. Global optimization may revise `X_k/S_j` and therefore `T_map_odom`, but **must not change continuous `T_odom_base` or any frozen local occupancy/EDT/correlation cache**. Map global correction can jump; its time, status and quality are tracked (OD-022). The resulting versioned graph and evidence references are saved by OD-012.

**Acceptance tests:** Docker Catch2/GTSAM tests for gauge/Jacobians, fused interval factor uniqueness, A/B overlapping submap memberships, scan self-match and duplicate-loop admission, OD-007 prior-regularized posterior rejection, OD-008 rank-deficient factor abstention, known synthetic corridor/loop truth, overlapping map correlation consistency, robust wrong-loop rejection and frozen hashes unchanged under graph pose updates. Measure B21 graph residual/ATE/RPE and held-out uncertainty calibration, solve time/memory, graph density, asynchronous no-blocking and native headers without GTSAM. See [accepted OD-013](open-design-choices.md) and [Architecture Section 7](architecture-specification.md).

### 7.3.1 Accepted OD-018 — Frozen-submap loop retrieval and verified constraints

**Stage I:** Retrieve candidate loops by **uncertainty-aware spatial proximity to frozen OD-010 submaps**; use OD-009 admissible multi-resolution branch-and-bound, OD-007 Ceres geometric refinement and OD-008/013 quality/consistency factor gates. **Stage II:** Once local spatial loops have validated precision/recall, add **rotation-tolerant 2D occupancy/known-space signatures on frozen submaps** to retrieve global revisits even where global pose has drifted. Individual keyframe descriptors remain deferred. No descriptor similarity or coarse correlation score becomes a GTSAM measurement by itself.

| Stage | Initial mechanism | Invariant |
| --- | --- | --- |
| Local retrieval | Pose proximity + covariance-bounded search radius/yaw window | Frozen submap, time/trajectory separation and OD-010 scan-membership exclusions |
| Global retrieval | Rotation-tolerant **2D frozen-submap occupancy/observed-mask signatures** | Explicit unknown vs free, cache source/content hash, circular-angle comparison |
| Candidate ranking | Merge sources; keep **top 3–5 distinct** (configurable) | Deduplicate same encounter, overlapping submaps and repeated source scans |
| Coarse verification | OD-009 **multi-resolution BnB** against immutable OD-011 pyramid | Admissible upper bounds, budget/completeness status; no false global-optimum claim on timeout |
| Fine verification | OD-007 unsigned EDT, bilinear, per-beam Huber Ceres | Same pinned OD-011 source revision, wrong odometry prior may not dominate distant loop |
| Acceptance | OD-008 geometry/rank/calibration; OD-013 correlation and graph check | Prior-conditioned Ceres posterior is **not** independent LiDAR; no self/duplicate factors |

**Local spatial retrieval:** Query a versioned global index of frozen submap bounds and poses using the current query keyframe/scan predicted pose and covariance; cap windows and candidate counts for OD-009's search budget. Require configurable minimum trajectory/time separation; refuse targets **containing the exact query scan**, as well as near-adjacent submaps whose historical insertion-ID sets substantially overlap. A frozen-submap-to-frozen-submap proposal must compare **both historical scan sets**, since adjacent A/B may share the same source evidence. A graph-pose-only optimization updates global spatial-index revision, **not** local occupancy, EDT, correlation pyramid or descriptor.

**Global submap signatures:** Compute a compact **planar rotation-tolerant descriptor** from known occupied geometry in frozen OD-010 local coordinates, e.g. radial/azimuthal occupancy histograms with circular angular shift and observed-space coverage normalization. Do not copy a **3D height-based** descriptor unchanged for LD19, or treat unobserved cells as observed free. Hash signature with exact OD-011 frozen occupancy source, threshold/known-mask/configuration and algorithm revision. A newly frozen submap can query historical signatures. Missing/stale entries rebuild using a **background, memory-bounded** worker; individual keyframe descriptors only after held-out comparisons.

**Ordered verification:** (1) validate query/target/source scan IDs, temporal separation, geometry support and immutable compatible source version; (2) coarse OD-009 BnB under mode-specific window and candidate budget, recording whether it was **complete, incomplete, timed out or canceled**, retaining multiple spatially distinct high-quality hypotheses as necessary; (3) OD-007 Ceres refine on pinned same-version map. Use a **distant-loop prior policy** that treats motion as an initialization/loose prior rather than an independent measured constraint; extract a **separately validated LiDAR-only geometry observation** (or explicitly conditioned model). Do not add a pose conditioned on OD-006 odometry as a second independent LiDAR graph factor; subtracting the prior Hessian is insufficient.

(4) OD-008 requires meaningful **independently observed** map overlap, robust LiDAR residual and inlier support, full or explicitly supported observability rank, calibrated covariance and time/deskew quality. A symmetric warehouse match with many equally plausible orientations/translations is **ambiguous**, even if correlation score is high; reject or require independent corroboration, not artificial rank-3 confidence. (5) OD-013 checks query-not-in-target, submap-source intersection, previous factor/query encounter IDs, shared motion/deskew dependence, and conservative factor correlation; then test consistency against accepted loops, graph innovation and **provisional optimization**. Since long-term odometry drifts, a large innovation alone must **not** veto a true well-corroborated loop; reject geometric false positives and loops inducing unexplained graph distortion. Robust graph losses are a backup after verification. Preserve **typed per-stage rejection diagnostics**, source/provenance and previous valid graph revision on failure.

**Scheduling:** OD-016 SLAM graph/recovery workers asynchronously process pinned OD-011 frozen snapshots, with bounded queue, count, CPU, memory and cancellation; they never hold acquisition/recording or local OD-007 tracking. Initial **unmeasured Pi 5 targets**: spatial shortlist **1–5 ms**, descriptor query Stage II **5–20 ms**, BnB **20–50 ms per candidate**, **top 3–5**, **100–250 ms** total background batch including Ceres refinement/validation. These are tunable work budgets, not guaranteed deadlines; report incomplete results and stale revisions, and do not assume worst-case per-candidate sums fit the batch cap. OD-021 determines concrete queue policy.

**ROS-/GTSAM-independent graph input:** A native `LoopCandidateRetriever`/`LoopVerifier`/`LoopDecision` (OD-017 domain interface) carries query/target/source membership, OD-001 time/epoch, OD-011 occupancy/score/descriptor config hashes, graph-pose revision, covariance/observability and validation state, not Ceres, GTSAM, ROS or IPC types. Implement spatial/descriptor retrieval in internal `slam_frontend`, Ceres in `slam_match_ceres` and GTSAM in `slam_graph_gtsam`; do not create a ninth initial library. OD-014 remapping invalidates and regenerates loop candidates, affected derived signatures and factors when local geometry actually changes, while global rigid corrections leave local descriptors unchanged.

**Acceptance (B26):** Docker Catch2 tests on exact synthetic apartment revisits with accumulated odom drift and repeated-shelving warehouse false loops; known/unknown and OD-005 partial scan descriptors; OD-010 A/B shared scans/self-match; OD-009 admissibility and top-K/time-limit results; OD-007 wrong-prior contamination; OD-008 LiDAR-only Hessian/rank/covariance and multimodal ambiguity; OD-013 overlap/graph provisional rejection and rollback; concurrent OD-011 version updates; Stage I vs Stage II recall@K/precision and unblocked Pi acquisition/localizer CPU/p95/p99. These are future tests, not measured outputs. See [accepted OD-018](open-design-choices.md), [Architecture Section 7.3.1](architecture-specification.md).

### 7.3.2 Accepted OD-019 — Sparse online submap iSAM2 and conditional fixed-lag smoothing

**Stage I (implement first):** Keep the OD-006 continuous wheel/gyro SE(2) estimator, bias correction and OD-004 deskew outside GTSAM. Add **ONE persistent global submap-only Pose2 iSAM2 graph** in the OD-016 SLAM process, using OD-017 internal `slam_graph_gtsam`. Retain every historical OD-010 submap pose `S_j` for global loop correction, and admit source-provenance-validated condensed inter-submap constraints and OD-018 verified loop factors. Do **not** add all keyframe states, an initial local GTSAM fixed-lag smoother or gyro-bias knots to this online graph. Keep the full OD-013 `X_k,S_j` batch graph independently reconstructible **offline**.

**Stage II only if validated:** A second, bounded GTSAM local smoother may keep **30–60 s** of OD-015 keyframes (target cap **150–200**) and **two active OD-010 submaps plus recently frozen boundary**. Add it only if held-out accuracy/uncertainty improves or bounded local graph complexity requires it; the initial Stage I system has no local GTSAM smoother to marginalize. OD-010's 90/180 scan insertion policy and OD-015 keyframe selection remain independent.

**Marginalization and correlated evidence:** For eliminated keyframes `m` and retained boundary/submap `b`:

~~~text
H = [ H_mm   H_mb ]    g = [ g_m ]
    [ H_bm   H_bb ]        [ g_b ]

H_reduced = H_bb - H_bm * solve(H_mm, H_mb)
g_reduced = g_b - H_bm * solve(H_mm, g_m)
~~~

Use stable factorization, appropriate gauge/rank treatment, the **full joint separator block and RHS**, preserved off-diagonal cross-information, original linearization point and source factor IDs. **Do not** split one dense multi-submap separator into fictitiously independent pairwise factors or convert nullspace into full-rank confidence. Nonlinear marginalization is a **linearization-dependent approximation**. Transfer a summary with explicit supersession/conditional information ownership: never let the global graph and local smoother count identical wheel/gyro/deskew/scan sources twice. GTSAM `ISAM2UpdateParams::removeFactorIndices` is a **factor removal mechanism, not statistical marginalization**; follow tested solver-version legality, Bayes-tree ordering and leaf restrictions.

**Summarized global constraints:** An OD-010 anchor and A/B overlapping submap membership are frame/source associations, **not independent measurements**. OD-013's fused odometry and LiDAR-only geometric evidence must be summarized with calibrated OD-008 rank/covariance, recorded OD-011 source hash, scan/gyro/wheel intervals and cross-correlation groups. A submap-only global graph **must not carry original keyframe likelihoods plus condensed copies**, or multiple independently weighted edges from substantially the same historical scan sets. **Defer an unsupported summary** rather than fabricate precision. Rejected/duplicate loops (OD-013/018) may be removed; valid factors must not be arbitrarily pruned solely to shrink the graph.

**Initial unmeasured Pi 5 compute/growth targets:**

| Metric | Initial budget | Applicable |
| --- | --- | --- |
| Local keyframe history | **30–60 s** | Stage II optional only |
| Active keyframes | **~150–200 cap** | Stage II; safe marginalization, not raw factor deletion |
| Local submaps | **Two active + recently frozen boundary** | Stage II |
| Historical submaps | **All remain in global graph** | Stage I/II |
| Global update | **Event-driven batched new factors** | Not per beam/scan |
| Local completed update p95 | **<20 ms** | Stage II only |
| Ordinary global completed update p95 | **<50 ms** | Stage I/II |
| Large loop closure | **Longer bounded background work** | Does not block acquisition/localizer |
| Work queue | **Bounded and coalesced where safe** | Exact size/overflow OD-021 |
| Cleanup | **On demand/offline checkpoint** | Full OD-013 batch graph |

The OD-010 new-submap period is ~90 *accepted* scan insertions; **10 accepted scans/s** gives **36000/90≈400 submaps per hour** as a hypothetical rate, not measured hardware acceptance. The global submap graph therefore grows and may eventually need explicitly designed regional partitioning; preserve historical `S_j` for OD-018 loops meanwhile.

**Safe iSAM2 scheduling:** OD-016 runs a lower-priority bounded graph worker, batches OD-013-admitted summaries/verified loops on events, and pins coherent OD-011/013 snapshot/source revisions. **Do not interrupt an active `ISAM2::update` midway** because its elapsed time reached 20/50 ms. Apply admission deadlines, max queued work and cancellation **before or between** solver updates, or discard independent candidate copies. Measure actual *completed* update latency. Tune `ISAM2Params` relinearization thresholds/skip/reordering, factor-slot reuse and memory after profiling, not by dropping valid constraints. A completed finite/consistent result may atomically publish a versioned global `T_map_odom` correction/`S_j` pose set after source/version checks; continuous OD-006 `T_odom_base` must not jump. Large-loop work and graph backlog are `DEGRADED`/resource diagnostics, not a reason to delay sensor capture.

**Offline rebuild and checkpoint:** Retain OD-005 original scan IDs and raw OD-006 wheel/gyro intervals, OD-008 geometry and uncertainty, OD-010 memberships, OD-011 versions, OD-013 original factor-source and correlation groups, OD-018 loop decisions and OD-015 keyframe time history. OD-012 bundles hold authoritative maps and graph state, **but OD-014 full re-deskew requires separate original MCAP/raw acquisitions and clock calibration**. At offline checkpoints, reconstruct the **full OD-013 keyframe/submap batch graph** from originals, excluding superseded summaries and rejected constraints, optionally run bounded OD-014 selective remap, validate held-out truth/consistency/loop quality and finite covariance, then publish a fresh immutable OD-012 generation. Reject stale cleanup outputs if live graph changes occurred; retain last accepted graph for rollback.

**Portable boundary/B27:** `GlobalSubmapGraph`, `SubmapSummaryFactor`, `SeparatorInformation`, `LocalWindowPolicy`, `GraphWorkBudget` and `GraphReconstructionPlan` use portable SE(2), calibrated cross-information, source IDs and revision metadata without ROS, Ceres, GTSAM or IPC types; only `slam_graph_gtsam` owns solver objects. Benchmark **B27** measures sparse Stage I against offline OD-013 batch truth, optional Stage II exact Schur vs linear batch, correlated source factor dedup and large-loop relinearization, 30/60 s–150/200 keyframe bounds, 20/50 ms p95 completed update goals, sustained graph growth/memory and offline rebuild/atomic rollback. See [OD-019](open-design-choices.md) and [architecture Section 7.3.2](architecture-specification.md).

## 8. Operating modes

### 8.1 Mode A — Record on robot, build map offline (first priority)

**On the robot:** acquire sensors, record packets/samples with original timing, optionally publish high-rate local odometry, and record diagnostics. Full map creation is not required while collecting data.

**Offline pipeline:**

1. Load and validate dataset; reconstruct source clocks and calibration.
2. Estimate/initialize wheel and gyro parameters and bias.
3. Build a continuous trajectory and deskew LiDAR measurements.
4. Match scans in Ceres, build overlapping submaps, and create graph constraints.
5. Use accepted OD-018 spatial frozen-submap retrieval, OD-009 BnB, OD-007 Ceres and OD-008/013 quality/correlation gates to detect and verify loop closures.
6. Admit OD-013 provenance-checked fused motion/local LiDAR/loop factors and optimize keyframe+submap poses globally with GTSAM batch optimization.
7. Apply **accepted OD-014** selective offline re-deskew and affected-submap reconstruction, only after quality/observability tests; start with one-pass R0 and permit at most three additional candidate iterations.
8. Publish only validated immutable OD-012 generation(s) and quality/iteration reports, preserving previous accepted outputs for rollback.

Offline replay must support deterministic scan selection, configurable processing rate, and the ability to compare parameter changes against the same raw data.

### 8.1.1 Accepted OD-014 — Bounded offline outer refinement

**Initial delivery is a one-pass mapper (`R0`)** using an immutable recording: OD-002/003 event-time reconstruction, OD-006 wheel/gyro trajectory with OD-004 full midpoint SE(2) deskew, OD-005 scan filtering, OD-007/008 matching, OD-010 submap insertion, OD-011 coherent caches and OD-013 batch keyframe/submap graph. Later support **at most 3 additional outer candidate iterations (`R1`, `R2`, `R3`)**, not an indefinite full-map loop. Runtime Pi sensor processing and live odometry do not depend on this feature.

**Selective change detection and update matrix:**

| Proposed correction | Re-deskew? | Reinsert occupancy rays? |
| --- | --- | --- |
| Rigid global GTSAM submap pose change | **No** | **No**, graph poses/global indexes only |
| Same rigid transform of every scan and local submap frame | **No** | **No** if `T_submap_scan` is unchanged |
| Scan midpoint pose relative to submap changes | **Only if within-scan motion also changes** | **Yes when material** |
| Intra-scan trajectory or calibrated gyro bias changes | **Yes when material** | **Yes** for affected member submaps |
| Validated OD-002/003 clock offset/drift/beam timing changes | **Yes when material** | **Yes** when resulting local beam geometry changes |
| Geometry unchanged within configurable tolerance | Generally no | Generally no; still assess quality/validity |

**Geometry threshold:** Compute maximum **submap-local Euclidean endpoint displacement**, comparing accepted and candidate beam geometry for all valid scans (or a defensibly conservative bound). Begin at **half of OD-010's 5 cm cell = 0.025 m**, with no claim of optimality; account for yaw error times beam range, laser extrinsics, OD-004 relative twist and OD-003 beam timestamps. Check **both** OD-010 membership submaps. Global loop-correction magnitude alone is not a remap signal. Relative scan pose may move rays even if within-scan deskew is unchanged: remap without pointless re-deskew. If clock epoch or time interpolation fails, a scan becomes invalid regardless of geometric threshold.

**Constrained calibration:** Only update OD-006 gyro bias when stationary/independent observations resolve bias from slip and noise; only update OD-002/003 clock offset/drift when timing is **observable** and physical LD19 timestamp delay remains respected/uncertain. Apply bounded changes/uncertainty and record clock domain/epoch/session. Do **not** tune time parameters solely to reduce LiDAR residuals on the same repeatedly rebuilt map or claim knowledge of the LD19 timestamp physical event.

**Rebuilding:** Construct affected local occupancy grids anew from the **original immutable raw packets, beam times, valid-hit/missing-return classifications and unchanged accepted scan IDs where appropriate**, preserving OD-010 one-cell-per-insertion hit precedence and known/unknown masks. Do not raster-warp accepted frozen maps or add log odds twice. Recompute OD-011 derived EDT/pyramid from the **new occupancy source revision**, then re-evaluate **all impacted OD-007/008 match constraints and OD-013 factors**, including loop candidates. **Replace** old factor IDs, never append duplicated information; apply OD-013 source membership, scan self-match and cross-correlation rules to each candidate. A globally rigid correction only changes `S_j`/graph pose revision, so its local raster/EDT/pyramid remain unchanged.

**Convergence, acceptance and rollback:** Each candidate has a full coherent map/graph/calibration/trajectory version and explicit dependency provenance. Check configurable **relative graph objective improvement on genuinely comparable factor sets**, pose-change tolerance, OD-008 geometry/residual/observable-rank and covariance consistency, loop support, and **held-out** scans/independent truth if available. A lower in-sample cost on a self-rebuilt map is **not** sufficient; do not compare total costs across different factor sets as though they have equal evidence. Reject a candidate for regression, false loop, nonfinite data, unobservable bias/time step, inconsistent cache/factor references or resource/budget violation. On rejection **retain the last accepted immutable generation** and stop or report; early stop when converged/no material local change; hard cap after 3 additional candidate iterations.

**Raw input and persistence:** OD-012 `LOCALIZATION_MAP` is insufficient for rerunning original deskew from scratch: retain **raw MCAP/packet acquisitions, encoder/IMU events, source-to-canonical time mappings, calibration provenance and input hashes** separately. Publish only validated candidates into **new immutable OD-012 map generations**, atomically changing `CURRENT` with recovery/rollback of prior version. Stable scan IDs, eligibility and submap membership are maintained wherever possible; any changes require reasoned new version IDs. Deterministic replay inputs, seeds, calibration, source hashes and dependency chains are included in iteration metadata; OD-024 later sets efficient replay seek/checkpoint behavior.

**Portable interface:** A ROS-/GTSAM-/Ceres-independent native `OfflineRefinementCoordinator` orchestrates model-neutral trajectory, map-builder, scan matcher, graph and artifact adapters. Ceres and GTSAM stay inside OD-007/013 targets. Tests in Docker Catch2 and **B22** cover pure rigid graph deformation with **zero** rebuilds, midpoint-only remap, high-yaw gyro/time re-deskew, half-cell bound, overlapping map membership, stale factor/cache deletion, recovery from aborted candidate generation, independent-truth quality gating and bounded iteration count. Profile selective/full rebuilding wall time, CPU/RAM and precision; avoid GitHub Actions.

### 8.2 Mode B — Live localization in a previously built map (second priority)

Load a frozen map or indexed map tiles plus precomputed distance fields. Keep high-rate wheel/gyro odometry continuous and match each deskewed scan using Ceres in the nearby map region. Publish global correction via **map -> odom**, while **odom -> base_link** remains continuous.

The localization state model includes these five states (exact hysteresis thresholds remain OD-022):

| State | Behavior |
| --- | --- |
| UNINITIALIZED | No trusted map pose; schedule full-map OD-020 candidate search and prohibit mapping. |
| TRACKING | Predict using wheel/gyro, then locally refine with Ceres. |
| UNCERTAIN | Enable bounded OD-009 local correlation (±0.3 m/±10°, initially 10–20 ms), refine selected candidates with Ceres and require OD-008 acceptance. |
| LOST | Revoke trusted global fix and suspend all mapping; schedule OD-020 map-wide OD-009/018 asynchronous BnB recovery with 360° yaw. |
| RELOCALIZING | Propagate persistent top-K hypotheses using continuous OD-006 odom; test across subsequent LiDAR scans, geometry, ambiguity and search coverage. Publish no global correction until all gates pass. |

The initial localizer uses the **accepted OD-006 minimal local predictor with conservative prediction/measurement uncertainty**. An EKF or GTSAM recent-state smoother may replace/enhance it later **only if benchmarks justify the complexity**. A local optimizer alone does **not** solve global or kidnapped-robot localization. **Accepted OD-009/020** supplies multiresolution branch-and-bound and persistent top-K multi-scan recovery. A persistent particle filter is **not** the initial backend; introduce or compare it only if B28 proves a need.

A lost/stale pose must be explicitly signaled to downstream navigation/control; it must not silently present an old pose as current.

### 8.2.1 Accepted OD-020 — Bounded global hypothesis tracking and publication

**Recovery input:** Given unknown global start, a suspected kidnapping or repeated local match failures, freeze map insertion and load immutable OD-010/011 eligible submaps with OD-012 generation identity. Do not constrain global search to an outdated OD-006 covariance/spatial vicinity. OD-018 signature retrieval may prioritize target submaps but must have deterministic full-map fallback over all eligible regions (including as-yet unloaded tiles) and 360° yaw. OD-009 BnB provides multiple spatially *and rotationally* distinct coarse poses; each is individually OD-007 refined with recovery-specific relaxed/no stale global pose prior and OD-008 LiDAR-only checked.

**Coverage and budget:** Maintain a per-snapshot complete eligible-domain ledger of considered, visited/evaluated, **admissibly pruned**, pending, unavailable and failed-load map regions/yaw cells; do not confuse retrieval recall with search completeness. Return COMPLETE, PARTIAL_BUDGET, CANCELLED, INVALID or RESOURCE_LIMIT, map/graph/score-cache/config/clock revision and work/node counts. Incomplete or unvisited credible regions may conceal a competing pose and block global acceptance. Run a bounded OD-016 async worker in OD-009's **unmeasured 100–200 ms work slices**, bounded by nodes, candidate K, queue age, pinned cache, CPU/RSS and total work; yield/cancel between slices, restart on revision changes and never stall acquisition/recording/local odometry. No invented search time guarantee.

**Persistent tracking:** Hold a configurable fixed-cap K of nonmaximum-suppressed hypotheses with stable IDs, separate map-to-odom transforms, SE(2) covariance/quality and source scan IDs. Propagate each using the OD-006 **relative** midpoint-to-midpoint odometry delta plus valid uncertainty (no stale absolute map prior). Validate subsequent eligible OD-005/004 scans for OD-008 LiDAR-only rank/overlap/robust residual, innovation, physically meaningful new geometric support, and consistent map-frame motion; reject stale/duplicated scans and correlated repeated views as independent evidence. Keep alternatives where symmetry cannot be broken, and record overflow when K-limiting would discard viable modes.

**Fail-closed acceptance:** Require sustained geometrically observable multi-scan evidence, temporal consistency, no contradictory motion, credible separation from alternative hypotheses and sufficient resolved map-search coverage. Symmetric rooms, repetitive corridors and incomplete LD19 sectors must default to **remain unlocalized** instead of confident wrong lock. Exact K, separation, scan/time windows, rank/margin/coverage thresholds and state hysteresis require B28/OD-022 calibration. A high Ceres score or OD-008 local covariance does not by itself establish *global* uniqueness.

**Mapping and frame contract:** While UNINITIALIZED, LOST or RELOCALIZING, suspend scan-to-map insertion, new active submaps and localization-derived graph constraints; continue continuous quality-rated local odom, and mark map pose unavailable/stale. Once verified, publish an **explicit version-checked map-to-odom correction** at the accepted scan midpoint without resetting odom-to-base or replaying untrusted scans into the map. OD-019 global graph publication must not bypass this trust gate; OD-022 still defines consumer-specific status and hysteresis.

**ROS-independent API/replay:** Native GlobalRelocalizer / HypothesisTrack / SearchCoverage / RecoveryDecision contain source scans, OD-001 epochs, map/graph/calibration/config revisions, OD-006 odom deltas, candidates with Ceres/OD-008 geometric evidence, coverage/frontier state, time/work budgets, rejected/missing regions, competing hypotheses and decision/correction reason codes. Preserve exact deterministic ordering and scheduling/evidence lineage for OD-024 replay. Keep ROS, Linux thread handles, solver objects and hardware adapters private. **B28** validates wrong-prior arbitrary relocation, repetitive corridors and symmetric rooms, partial LD19 sectors, incompletely searched maps, competing hypotheses, false locks, continuity, noninterference and deterministic decisions (not yet executed).

### 8.3 Mode C — Full online SLAM (third priority)

Reuse acquisition, high-rate prediction, deskewing, Ceres frontend, submaps, and graph definitions. Run graph optimization and loop closure asynchronously so sensor collection and local control are not blocked. Generate new global map versions while preserving a continuous odom trajectory.

## 9. Recording, API boundaries, and saved artifacts

### 9.1 Recording format

Use the **native ROS-independent MCAP C++ writer in the recorder** for initial raw recordings; ROS 2 rosbag2 belongs to optional adapters/converters. Define portable serializable event schemas (exact channel/encoding/evolution choices remain OD-023) for:

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
SensorSample -> TimestampAligner -> MotionPredictor -> TrajectoryProvider
LidarPacket -> LD19 Time Adapter -> observed TimedBeam + gap events
TimedBeam + gap events -> fixed-seam ScanAssembler -> AssembledScan + coverage + return status
AssembledScan + TrajectoryProvider + T_base_laser -> MidpointDeskewer + eligibility
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
- **Maps:** **Accepted OD-012** JSON-manifest directory + independently compressed portable binary authoritative occupancy/graph; OD-011 EDT/correlation caches optional and regenerable.
- **Platforms:** Linux ARM64 and x86-64 with portable core algorithms.
- **Testing:** Unit tests, synthetic-sensor tests, deterministic dataset replay, portable map/checkpoint B20 integrity/fault tests and local benchmark executable.

**Accepted OD-017 initial source/build layout** (eight libraries, three host daemons, and independently built Web Lab/Pico; tests/optional integrations excluded from target count):

~~~text
include/slam/
  core/{time.hpp,geometry.hpp,sensor_data.hpp,trajectory.hpp,interfaces/}
  mapping/{occupancy_grid.hpp,submap.hpp,map_snapshot.hpp}
  engine/{slam_engine.hpp,configuration.hpp,diagnostics.hpp}
  storage/{map_bundle.hpp,recording.hpp}
src/
  core/             # slam_core -- SDK
  mapping/          # slam_mapping -- SDK
  engine/           # slam_engine -- SDK facade/private runtime
  storage/          # slam_storage -- SDK
  frontend/         # slam_frontend -- internal
  match_ceres/       # slam_match_ceres -- internal
  graph_gtsam/       # slam_graph_gtsam -- internal
  platform_linux/   # slam_platform_linux -- internal
apps/
  slam_acquisitiond/  # native host daemon
  slam_recorderd/     # native host daemon
  slam_localizerd/    # native host daemon
  web_lab/            # separate application build, OD-016 fourth process
firmware/pico/        # separately cross-compiled only when enabled
tests/
cmake/
docs/
~~~

**Only four SDK libraries are installed/exported:** `slam_core` as `slam::core`, `slam_mapping` as `slam::mapping`, `slam_engine` as `slam::engine` and `slam_storage` as `slam::storage`. Remaining four libraries are implementation-private and not SDK targets. CMake uses ≥3.25, C++20 per-target features, isolated dependency/include visibility and an installed **versioned `FusionSLAMConfig.cmake` / `FusionSLAMConfigVersion.cmake` / `FusionSLAMTargets.cmake`**. The installed header/API namespace is `include/slam/{core,mapping,engine,storage}/`. Old provisional `include/fusion_slam`, `fusion_slam_core`, many-module layout and separate offline mapper host executable suggestions are superseded by this accepted initial target plan; future tools may be separately selectable without changing the fixed baseline.

**Dependency requirements:** `slam_core` has no ROS, Ceres, GTSAM, Linux, native IPC or driver dependencies; `slam_mapping` may link core only; `slam_storage` uses portable data types and private serializer codecs; `slam_engine`'s public API is portable but private runtime composition may use `slam_frontend`, `slam_match_ceres` and `slam_graph_gtsam`. `slam_platform_linux` is the outer acquisition/IPC adapter for host apps only, not portable SDK/algorithm libraries. Public installed headers cannot include solver types, `sys/socket.h`, ROS/Pico headers, IPC worker scheduling or private cache implementation; Ceres and GTSAM remain behind their own targets. Optional ROS2 package depends on installed SDK only.

**SDK install correctness:** Ensure actual `PUBLIC`/`PRIVATE`/`INTERFACE` CMake link closure is acyclic. A static engine `PRIVATE` dependency on internal static libraries can leak `$<LINK_ONLY:...>` into installed target exports; use a shared façade/pImpl or other demonstrably self-contained package model. Install/private-link required runtime solver libraries as needed without claiming binary runtime independence. Test a **clean out-of-tree consumer** with `find_package(FusionSLAM CONFIG REQUIRED)` and all four `slam::*` imports; require no private headers/targets or Ceres/GTSAM/ROS at consumer compile/configure. `BUILD_TESTING=OFF` excludes Catch2; cxxopts limited to three daemon CLIs; Web Lab and Pico built independently. **B25** measures install/consumer/test/runtime correctness on x86-64 and ARM64/Pi 5. Keep GitHub Actions manual-only if any are added; no Actions required.

### 10.1 Accepted OD-012 — Platform-independent native map bundle and checkpoint format

The initial persistent map representation is a **directory** containing a strictly parsed **JSON `manifest.json`** and **fixed-width, explicit-endian, independently Zstandard-compressed binary payloads**. Authoritative state includes OD-010 local log-odds occupancy **plus an explicit observed/unknown mask**, per-submap local frames (including yaw), resolution/dimensions, scan-insertion/lifecycle metadata, separately versioned graph poses, graph nodes/constraints/uncertainties and required calibration/time/reconstruction information. The OD-007 EDT and OD-009 score pyramid are **OD-011 optional cache files**, each keyed to an exact occupancy source/config hash, and may always be regenerated. Never serialize GTSAM/Eigen/C++ object memory directly.

**Directory generation model:**

~~~text
building_a.slam/
  CURRENT                           # checkpoint generation ID, atomically replaced
  generations/
    000000000001/
      manifest.json                 # schema version, map identity, indexed file hashes
      graph.bin.zst                 # versioned factor/node/pose/information records
      submaps/
        sm_000001.bin.zst           # log-odds + observed mask, independently loaded
      cache/
        sm_000001.edt.bin.zst       # optional, exact source/config revision
        sm_000001.corr.bin.zst      # optional
      checkpoint/state.bin.zst      # only for resumable live sessions
      preview.png                   # optional visualization
~~~

A one-off localization export MAY instead use a single self-contained directory containing the manifest and its files. **Repeated saves always write immutable numbered generations** with a small atomic `CURRENT` pointer, not in-place overwrites or rename-over-nonempty-directory assumptions.

**Manifest fields:** magic/type and major/minor version; required/optional features and compatible-reader requirements; map ID, generation/provenance, `bundle_kind`; REP-103/105 coordinate/SE(2) frame conventions, units, tangent/covariance ordering; submap IDs, local map origin/yaw, width/height/resolution and insertion IDs; OD-010 occupancy encoding/prior/hit/miss/unknown policy; per-file relative path, role, payload version, codec, **compressed and uncompressed sizes and SHA-256**; graph revision/factor IDs/references; extrinsics, gyro/wheel and clock-calibration identifiers; OD-001 timestamp domain/epoch/session mapping; and optional cache config/source hashes. Unknown safe optional JSON fields may be ignored; unknown **required features/incompatible major** must be rejected and reported.

**Binary encoding:** Define per-payload magic/version, bounded record counts and lengths, fixed-width integers, explicit **little-endian byte order**, specified IEEE-754 floats where needed, finite pose/covariance validation, cell-index ordering, packed observed bits and row-major log-odds values with documented quantization/precision. Do not encode `size_t`, compiler padding or native matrix layouts. Each frozen submap is **one independently decompressible zstd frame/file**; graph/checkpoint payloads may also be independently compressed. RFC 8878 is the transport-independent zstd format, but does **not** provide arbitrary random access *within* a compressed frame. Implement bounded streaming/decompression and reject malicious window sizes, size overflows and unsupported codec options before allocating.

**Publication and recovery:** Capture a consistent immutable occupancy **and pose-graph** snapshot first (live resume also needs estimator/active/clock/replay state). Write a **new same-filesystem staging directory** and all payloads, sizes/checksums and manifest; validate cross-file IDs/required features then **fsync files and directories as supported**. Rename staging to an **unused immutable generation name**, sync the parent directory, write/fsync a temporary `CURRENT` file, atomically replace `CURRENT`, and sync its directory. Keep a prior validated generation. Never overwrite the live bundle or expose half-built output; loaders pin one generation and **validate all required authoritative files** before publishing to runtime. On interruption recover the newest fully valid compatible generation (bounded scan), ignoring leftover staging; optional invalid cache files are discarded/rebuilt. Atomic rename alone does **not** ensure power-loss durability on every filesystem, so fail safely or report unsupported guarantees when flush/rename/directory sync semantics are insufficient.

**Strict loader limits:** Validate SHA-256 of stored bytes, relative path containment (no traversal, absolute paths, symlink escape), duplicate IDs, cross-references, feature/format versions, record size/count/decompression caps, finite transformations and covariance, matching observed-mask dimensions and source hashes. A partially valid required graph/submap **must not** reach the SLAM runtime. Optional cache/preview failure must not invalidate an otherwise valid map. Migration uses an explicit new generation, preserving the prior compatible source.

**Bundle kind matters:** `LOCALIZATION_MAP` contains a sufficient frozen map, graph and matching/calibration metadata for read-only localization. `RESUMABLE_CHECKPOINT` also contains active submap builders, OD-006 bias/covariance/trajectory, graph continuation state, insertion/replay cursor, exact input identity and **OD-001 clock-domain/session/epoch and conversion provenance**. Monotonic Pi timestamps **do not retain the same epoch after reboot**; initialize/validate a new mapping for live resume. Never claim resume from a map-only export or infer acquisition time from bundle creation wall time. OD-024 governs periodic replay checkpoint policy.

ROS/Nav2 YAML + PGM or OccupancyGrid adapters remain **separate and explicitly lossy** relative to native graph/submap/time state. Single-file packaging and database-backed persistence are deferred. The native loader/writer public interface MUST compile without ROS, Ceres, GTSAM serializers, Pico or sensor drivers; the GTSAM graph adapter converts native versioned factor records behind a separate boundary.

**Acceptance:** Docker Catch2 B20 tests must round-trip exact log-odds/observed flags, rotated origins, graph factors/uncertainties, schema/major-minor versions and clock sessions; cross-platform endian/golden vectors, individual zstd submap lazy reads, SHA corruption and optional-cache rebuild, malicious paths/oversized decompression, crash injection around staged writes/rename/pointer and bounded previous-generation recovery; map-only vs complete resumable checkpoint behavior. Profile Pi 5 flash writes, compression memory/time, lazy load and p95 save/load latency before choosing resource limits. See [OD-012](open-design-choices.md) and [Architecture Section 10.1](architecture-specification.md).

### 10.2 Accepted OD-016 — Four-process runtime and versioned native IPC

**Deployment:** Raspberry Pi 5 initially uses **four independently supervised Linux processes** and native `AF_UNIX/SOCK_SEQPACKET` IPC: (1) **acquisition** with exclusive LD19 UART, IMU/encoder/Pico FD ownership and OD-001/002/003 timing; (2) **recorder** with ordered raw packet/sync/calibration/sequence-gap MCAP capture and durable completion status; (3) **SLAM** with OD-005/004/006 scan/motion, OD-007/008 local matching, OD-009 recovery, OD-010/011 map caches, OD-013 GTSAM graph and OD-012 persistence; (4) **Web Lab** with only rate-limited telemetry and validated asynchronous commands. ROS is an optional outside adapter, **never** a native transport dependency.

| Data flow | Initial policy |
| --- | --- |
| Acquisition→recorder | Dedicated bounded ordered raw stream including source/epoch sequences and timing calibration; loss or disk failure explicitly flags `INCOMPLETE` or stops recording |
| Acquisition→SLAM | Independent bounded timestamped sensor stream; `EAGAIN`/full queue gives counted gap and OD-022 degraded state, not sensor-thread blocking |
| SLAM ingest→Ceres | Bounded short serial assembly/deskew and dedicated local-matching worker; quality/latency/OD-004 gates |
| SLAM localizer→graph/cache/save | Bounded lower-priority workers on pinned OD-011/013 immutable snapshots; coalesce superseded graph work |
| Graph→localizer | Version-checked atomic global map correction; discard stale solve, keep continuous OD-006 `odom→base_link` |
| SLAM→Web Lab | Rate-limited latest-state snapshot; obsolete UI frames dropped |
| Web Lab→SLAM | Authenticated/validated async commands with request IDs and explicit accepted/applied/config-generation acknowledgment |

**IPC semantics:** Unix `SOCK_SEQPACKET` preserves message boundaries and sends order **on a live connection**, not delivery across disconnection or durability. Native protocol header MUST include magic, **major/minor version and required features**, message/payload type and bounded length, producer/source/run ID, per-source ordered sequence, **OD-001 signed integer-nanosecond event time and clock-domain/epoch**, calibration/config revision, status/gap flags, and correlation ID if applicable. Detect truncation (`MSG_TRUNC`), nonblocking send failure (`EAGAIN`), closed peers, unsupported required versions and missing sequence IDs. IPC payloads use portable fixed-width fields and do not serialize C++ objects or depend on ROS/DDS/Ceres/GTSAM. Native IPC schema, OD-023 MCAP schema and OD-012 map bundle schema are distinct but preserve source IDs/clock provenance.

**Recording durability:** A socket send acknowledgment is not a disk durability acknowledgment. The recorder must track accepted, written and durably finalized data boundaries separately where implemented. On disk-full, recorder crash, unbounded service stalls or loss of a bounded sender buffer, mark recording **INCOMPLETE/LOSS_DETECTED** or stop a segment; **do not silently claim a complete raw dataset**. Maintain packet-gap, clock-fit, IMU/encoder and configuration records necessary for OD-014 re-deskew/replay. Acquisition never waits indefinitely for storage or SLAM; full losslessness is conditional on measured capacity and bounded fault conditions (OD-021 to set quantitative thresholds).

**SLAM internal scheduling:** Initial **fixed bounded workers**, no unbounded thread/task spawn: short IPC ingest/OD-006 odometry, ordered OD-005 assembly/OD-004 deskew, dedicated OD-007 Ceres local matching, lower-priority OD-013 graph optimization and OD-011 cache construction/OD-012 map saves, nonblocking immutable result publication. Graph solve snapshots and source IDs must match current acceptable versions before result publication. Use **normal Linux `SCHED_OTHER` first**, measure p95/p99 timing; `SCHED_FIFO` is optional only after validated high-priority acquisition starvation safety and privilege limits. Per-channel count/**byte** bounds, workers/max in-flight work, stale scan and job-revision policies, short enqueue timeouts and cancellation are mandatory. Accepted **OD-021** sets provisional queue capacities, per-subsystem loss/age policies, warnings and MCAP durability/rotation targets; validate them with **B29** on Pi storage before treating any deadline as proven.

**Restart and security:** Independently health-check/restart processes, negotiate IPC protocol on reconnect, verify socket filesystem ownership/peer credentials where available, and validate Web Lab commands rather than granting direct map/hardware write access. A restarted acquisition changes clock/run generation; recorder closes or marks incomplete segments and logs any gaps; SLAM reinitializes state or validates a compatible OD-012 checkpoint, never blindly reuses another process's old clock epoch/graph correction. Web Lab can restart without affecting sensor capture. Bound shutdown/drain and report incomplete recording if it cannot finish.

**Optional shared memory:** Linux `memfd_create`/`SCM_RIGHTS` may transfer **sealed immutable** large map snapshots as FD+length+hash+revision after profiling; ownership and pinned-memory/FD limits are mandatory. Default sensor/control IPC stays seqpacket; portable domain APIs cannot require memfd, UNIX socket types or Linux scheduler headers. Split graph into a fifth process **only if benchmark B24 shows GTSAM crashes or overload threaten the localizer**, not initially.

**Validation:** Docker Catch2 plus four-process integration B24 verifies protocol boundaries/version/truncation/sequence gap, independent acquisition→recorder/SLAM pressure, disk-full incomplete status, mismatched clock epochs and reconnection, stale graph correction, no UI backpressure, optional FD passing and normal scheduling under ~10 Hz LD19 and ~200 Hz IMU. Pi 5 p50/p95/p99 timestamp jitter, CPU/peak RSS, queue age/bytes, dropped-event counters, recorder durability and thermal throttling must be measured; no hard deadline or losslessness is claimed prior to tests. See [OD-016](open-design-choices.md) and [Architecture Section 11](architecture-specification.md).

### 10.3 Accepted OD-021 — Quantitative bounded queues, failure semantics and recorder integrity

**Initial deployment table (engineering defaults, NOT Pi-validated):**

| Queue | Initial bound | Operation at the limit |
| --- | --- | --- |
| LD19 acquisition ring | 512 packets **and** 128 KiB | Continue UART draining when possible; detect overflow by sequence and mark required raw gaps |
| Pico acquisition ring | 512 frames **and** 256 KiB | Detect epoch/sequence discontinuities; invalidate affected IMU/encoder motion intervals |
| Acquisition → recorder | 16 MiB **or** 5 s oldest backlog; finite measured message cap | No silent drops; mark session FAILED/INCOMPLETE if any required raw event cannot be preserved; never block acquisition indefinitely |
| Acquisition → SLAM motion | 1 s of source motion, finite byte/message cap | Explicit missing-motion intervals and invalid deskew/prediction support |
| Completed scan processing | 8 complete scans, finite byte/max-age caps | Discard oldest **unprocessed** stale complete scan, preserve processing-gap source/time IDs |
| Graph optimizer | 2 pending versioned requests + in-flight memory cap | Coalesce redundant solves, **not** accepted graph factors/loops |
| Map/cache | 1 running + 1 pending versioned build | Supersede stale pending work; reject stale completion and bound pinned bytes |
| Web Lab telemetry | 1 latest state per stream | Replace obsolete telemetry |
| Web Lab map updates | 2 pending versions | Skip superseded deltas; detect missed versions and send complete coherent snapshot |

All nine queues must have **hard count and byte limits** and applicable **host-steady oldest-event maximum ages**. An unspecified count/byte/age numerical bound is configuration derived from measured largest message and burst; not license for unlimited buffering. The **first** applicable bound to be exceeded activates explicit nonblocking overflow/backpressure semantics. Sequence/gap detection and run/clock epoch survive both seqpacket connection failures and process restarts. Count source events, accepted queue events, dropped scan IDs and graph/cache revisions independently; never use queue trimming to rewrite source clocks or accepted graph evidence.

**Recorder write/durability path:** One recorder writer thread; default **LZ4** compressed MCAP chunks, with Zstd tested as alternative. Finish a chunk when **1 MiB or 1 s of input** is reached (the C++ \`closeLastChunk()\` API can force time-based completion), **enable chunk CRC**, and aim for **≤1 s between *successful* OS sync calls**. Rotate segments at **60 s or 256 MiB**. Writer chunk closure and \`IWritable::flush()\` alone do not promise stable storage; an owned FD-backed sink must perform checked \`fdatasync\` or \`fsync\`, and file/parent-directory sync on rotation/atomic manifest updates as required by the selected filesystem. Separate acknowledgments/high-water sequence boundaries for **accepted, written, chunk-complete, OS-synced and fully finalized/verified**; acquisition→recorder transport receipt is **not** a durable ack. Publish exact sync error, worst-case stall and bytes-at-risk as diagnostics; do not claim a strict ≤1 s bound without fault-injected Pi evidence.

**Recording state and integrity:**
- \`RECORDING\`: all accepted raw events accounted for; \`DEGRADED\`: queue use **≥70%** or excessive queue age/disk lag; **≥90%** produces critical warning before loss. Track bytes, count and age separately; warnings may trigger below thresholds.
- \`FAILED\`: any required source sequence gap/overflow, irreversible write or sync failure, missing tail or inability to prove end-to-end raw completeness; **session completeness is sticky INCOMPLETE**, regardless of successful salvage.
- \`FINALIZING\`: bounded drain, complete MCAP chunk/footer/index, flush plus successful OS sync, rotate/close, durable atomic manifest; \`COMPLETE\`: finalized and integrity-verified with no missing required event. Shutdown that cannot meet these checks is INCOMPLETE, not silently COMPLETE.
- After recorder death, power loss, partial write or full disk, recover only complete reader-valid/CRC-valid records or chunks with defensible source/time continuity; quarantine broken tails, audit segment boundaries, and never invent MCAP summaries or full-session completion. A new session may start with a new ID while the old remains incomplete.

**SLAM / graph / UI safety:** OD-006 motion samples have their own valid coverage timeline. A missing IMU/wheel interval means no trustable interpolation/normal OD-004 deskew over the gap: degrade OD-022 tracking and reject the affected scan for submap insertion/factors. An overloaded matcher may drop *complete unprocessed LiDAR scans*, recording their scan IDs and processing-time gaps; maintain odom continuity only where supported. Graph worker requests contain graph version; accepted OD-013/019 factor records are stored in authoritative state/journal before coalescing, including verified loops. If the graph falls behind, suspend expensive new verification or degrade operation, not delete factors. Version-aware immutable map/cache publication rejects stale rebuilds. Web Lab's telemetry is latest-value-only, but lost map versions require fresh complete snapshot resynchronization; user commands retain explicit nonlossy validation/acknowledgment.

**Observability / required B29:** Export per-edge occupancy/max **item count/bytes/host-steady oldest age**, incoming/outgoing event rates, source/epoch/sequence gaps, scan drops and deskew-invalid windows, enqueue failure, deadline misses and coalescing, graph/cache version age, Web Lab snapshot resyncs, writer throughput, chunk CRC, synced-versus-written high-water marks, segment recovery, \`fsync\` p50/p95/p99 and failure codes, CPU/RSS and thermal budget. B29 injects disk-full/slow/unsynced writes, recorder kill, burst acquisition, Pico disconnect/reconnect, Ceres saturation, several-second GTSAM stalls, cache rebuild overload and disconnected Web Lab clients; verify bounded memory/no acquisition starvation and **no false complete raw session, incorrectly deskewed insertion or lost accepted graph factor**. Calibrate all thresholds on the target Raspberry Pi at measured worst-case LD19/Pico rates and selected disk. The documented numbers remain **initial targets, tests not executed**.

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

Measure end-to-end **capture-to-pose latency**, not only Ceres solve time. **Accepted OD-016** mandates four-process isolation and bounded queue diagnostics; **OD-021** still selects numeric caps and loss policy. Retain source raw timestamps and completeness/gap status. Report missed deadlines and thermal throttling on the Pi. Online SLAM is allowed to defer global optimization when compute is constrained.

## 12. Testing and acceptance strategy

### 12.1 Unit and integration tests

- LD19 frame parsing, CRC errors, truncated packets, 30-second timestamp rollover, angle/speed beam-time reconstruction and clock fit; OD-005 seam-straddling packets, variable speed, circular angular coverage, partial scans, missing sectors, invalid vs validated no-return map clearing, recovery-only watchdog and serial resynchronization.
- Clock offset/drift estimation and timing uncertainty; missing or late IMU/encoder samples.
- **OD-016 B24** four-process Unix seqpacket protocol/version/sequence, acquisition nonblocking pressure, recorder incomplete status on loss, worker staleness, independent restart and Web Lab isolation.
- SE(2) arcs/near-zero angular rate, OD-006 minimal wheel/gyro integration, verified/unconfigured kinematic adapter, stationary bias calibration and bounded updates, slip disagreement with covariance growth, asynchronous measurements, timestamp-addressable coverage and laser/IMU extrinsics.
- Synthetic moving-wall/apartment/warehouse scans proving **midpoint-referenced** rotational and translational deskew, full interpolation eligibility, 10 ms quality-gated extrapolation and rejection of 10.1 ms or uncertain/gapped scans.
- OD-007 Ceres EDT/bilinear/per-beam Huber/prior/boundary; OD-008 LiDAR-only rank/uncertainty; **OD-009** covariance-bounded correlative search, admissible BnB bounds, multiple hypotheses, cancellation and mode budgets; no posterior double counting.
- **OD-013** GTSAM keyframe+submap Pose2 gauge and Jacobians, one fused wheel/gyro interval constraint, one primary local LiDAR factor, source overlap/self-match/duplicate-loop rejection, correlation-conservative covariance, graph revision and B21 held-out accuracy.
- **OD-012 B20** map-bundle platform-independent binary/JSON round-trip, required-file SHA and size verification, strict version/clock-domain checks, malicious loader bounds, atomic generation fault recovery, lazy zstd submap loading and cache regeneration; replay determinism.

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
| MPU6050 gyro bias and temperature drift | Accepted OD-006 stationary startup calibration, conservative bias uncertainty and gated online correction; optional temperature characterization and graph bias refinement. |
| Acceleration and wheel slip | OD-006 compares independent wheel/gyro yaw only where observable, grows covariance, reports degraded quality and rejects false bias updates from slip. |
| Corridors / repetitive geometry | Degeneracy-aware scan matching, multiple pose hypotheses in recovery, properly scaled covariances. |
| Moving obstacles / map changes | Robust residuals, dynamic rejection, read-only localized map with versioned remapping. |
| Ceres posterior correlation with odom and IMU deskew | Accepted OD-007 explicitly regularizes frontend with a wheel/gyro odometry prior; separate LiDAR-only evidence from posterior pose before any independent GTSAM factor; conservatively handle deskew and map correlation (OD-008/013). |
| DeepRacer wheel telemetry availability | Verify actual hardware signals before specifying encoder factors; support Ackermann model. |
| Online graph growth | Asynchronous backend, keyframes, submap pruning, optional fixed-lag local estimation. |
| Offline/map-version mismatch | Version artifacts and reject mismatched calibration or coordinates. |

**Open decisions:** final robot drive type; availability of measured wheel speeds/steering angle on DeepRacer; Pico transport and clock-model tuning (the Pi-first/Pico-future approach is accepted under OD-002); exact LiDAR/IMU mounting; ROS 2 integration boundary; map binary serialization; and OD-020 B28-measured recovery threshold/K/resource tuning (the BnB + persistent top-K algorithm itself is Accepted; particle filtering is deferred).

## 14. Implementation milestones

1. **Foundation:** portable build, sensor schemas, clocks, raw recorder, deterministic replay, test harness.
2. **Motion and deskew:** accepted OD-006 minimal wheel/gyro integrator, stationary bias calibration, verified replaceable kinematics and uncertainty; OD-004 continuous SE(2) midpoint deskew.
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
