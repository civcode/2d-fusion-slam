# 2D Fusion SLAM — Architecture Specification

**Status:** Proposed architecture v0.1 · **Date:** 2026-10-30  
**Scope:** ROS-independent C++20 sensor fusion, offline mapping, live localization and online SLAM.  
**Related:** [Technical specification](technical-specification.md) · [Implementation contract](implementation-contract.md) · [Benchmarks](benchmarks-and-datasets.md) · [Open design choices](open-design-choices.md)

> This describes intended components, boundaries and proposed defaults, **not implemented functionality**. Decisions identified as **OD-xxx** are deliberately unresolved; consult the decision register before treating a proposal as final.

## 1. Goals and invariants

Primary aim: accurate **offline mapping** and lightweight **live known-map localization** using 2D LiDAR, yaw-rate IMU and wheel odometry; later enable **full onboard online SLAM**. Raspberry Pi 5 / ARM64 is the deployment target, x86-64 an offline and development target, with optional DeepRacer EVO experiments.

| Mode | Real-time path | Optimization | Map mutability |
| --- | --- | --- | --- |
| A — Record and offline SLAM | Acquire, timestamp, record, optional local odometry | Ceres scan matching, GTSAM batch + loop closure and iterative map rebuilding | Output new versioned maps offline |
| B — Frozen-map localization | Wheel/gyro prediction, beam deskew, Ceres scan-to-map, recovery | Optional EKF/fixed-lag enhancement; global graph not mandatory | **Read-only input map** |
| C — Online SLAM | Same frontend + active submaps | Async GTSAM iSAM2 and loop closure | Publish new immutable map revisions |

Architectural invariants:

1. **One ROS-free C++20 core** supplies geometry, sensors, estimator and map algorithms. Neither ROS messages, HTTP, Docker nor cxxopts enter the core algorithm APIs.
2. Use **REP-103/REP-105** units and frames natively. The optional ROS 2 adapter owns message conversion, TF, QoS, rosbag2 and lifecycle concerns.
3. **Measurement acquisition time** drives estimation; transport receipt time is retained separately, not treated as the same clock. **OD-001 is Accepted:** canonical time uses signed 64-bit nanoseconds and explicit clock-domain/epoch conversion. The simulator's ROS 2 `/clock` may pause or jump; it is not a continuous wall or steady clock.
4. **`odom -> base_link` stays continuous** through global optimization. Map corrections modify `map -> odom`, with one TF authority per edge.
5. Raw measurements are immutable; map, calibration, dataset, replay, solver and benchmark outputs carry provenance and version IDs.
6. Frontend prediction is **not an independent graph measurement** if derived from the same wheel/IMU streams already represented by factors.
7. Every global pose has time, frame, covariance/quality and tracking state. Lost/stale estimates may not be presented as valid fixes.
8. Long-running optimization, recording I/O, UI and ROS callbacks cannot block acquisition.
9. The synthetic apartment and warehouse fixtures support offline deterministic tests without external data downloads.
10. **Boost** components (including transitively introduced components) require case-by-case discussion and approval per the implementation contract.

## 2. Context, software layers and ownership

~~~text
LD19 UART      MPU6050        wheel encoders / steering
     \            |                     /
      \           |                    /
     acquisition adapters, timestamps, packet diagnostics
                          |
                   clock alignment
                          |
          ordered canonical native measurements
                   /                 \
             raw MCAP writer        wheel + gyro predictor
                   |                 |
          deterministic replay       +-- continuous T_odom_base(t)
                   |                 |
                   +----> scan assembler / timed beams
                                      |
                                full SE(2) deskew
                                      |
                           Ceres local scan matching
                              /                 \
                     map/submap update       fixed-map tracker
                              |                 |
                    GTSAM factor backend    relocalization
                    batch or async iSAM2        |
                              \                 /
                               pose and status
                                     |
                     +---------------+---------------+
                     |                               |
               ROS 2 adapter                 Web Lab / reports
              (optional)                    (optional)
~~~

Component ownership and forbidden dependencies:

| Module | Owned state/output | Boundary |
| --- | --- | --- |
| `core` | SE(2), units, timestamps, frame conventions, status, covariance types | Standard C++20 + Eigen; no Ceres/GTSAM/ROS |
| `sensors` | LD19 parser, Pi GPIO/I²C/UART input or optional Pico native framed input, health and raw samples | No estimator/map ownership; no micro-ROS |
| `time_sync` | Rollover, clock offset/drift, ordering and time quality | No map or solver dependency |
| `calibration` | Versioned sensor extrinsics, wheel scales, gyro bias priors and timing offsets | Invalidates derived artifacts on change |
| `motion` | Minimal wheel/gyro integrator, gyro bias, pose–bias uncertainty, replaceable KinematicModel, piecewise SE(2) trajectory, coverage and quality | No drivetrain assumption or global-map jumps; expose generic provider |
| `scan_assembly` | Fixed-seam generic timed-beam grouping, partial coverage, missing sectors and return validity | No LD19 packet or ROS dependency |
| `deskew` | Per-beam transformation into temporal midpoint laser frame, full-coverage validation and provenance | No device/ROS/optimizer dependence |
| `mapping` | OD-010 log-odds/two active submaps; OD-011 coherent pinned snapshots, full-rebuild baseline, tiled active EDT and reusable frozen EDT/score caches | Domain independent of Ceres/GTSAM/ROS; map revision and graph pose separate |
| `frontend_ceres` | OD-007 EDT/Ceres solve and **LiDAR-only** residual/Jacobian extraction with separate prior/boundary diagnostics | No ROS/device I/O; Ceres private behind native ScanMatcher |
| `match_quality` | OD-008 robust PSD information, normalized rank/eigenvectors, calibrated full covariance or observable subspaces and acceptance | Native C++20/Eigen; no Ceres, ROS or sensor SDKs |
| `coarse_search` | OD-009 covariance-aware local correlation, immutable multires score pyramids, async BnB and versioned candidate sets | No Ceres/ROS; bounded worker and native interface |
| `backend_gtsam` | Factor graph, loop factors, bias states, optimized graph revisions | No acquisition/UI ownership |
| `localization` | Frozen-map tracking state, confidence and recovery | Read-only map |
| `io` | MCAP/native datasets, versioned maps, conversion/report schemas | No ROS runtime requirement |
| `apps` | cxxopts CLI, lifecycle, deployment orchestration | Do not duplicate math |
| `web_lab` | Local HTTP JSON and Canvas experiment explorer | C++ backend is only estimator |
| `fusion_slam_ros` | ROS topics/messages/TF/bag conversions | Isolated ROS-specific build |

Component interfaces MUST remain usable in offline replay with no sensor hardware and, where relevant, with no map loaded. Process/thread decomposition is **OD-016**.

## 3. Geometry, frames and time

### 3.1 SE(2) convention

`T_A_B` maps coordinates **from frame B into frame A**; composition: `T_A_B * T_B_C = T_A_C`. Body X forward, Y left, Z up; yaw positive counterclockwise, radians/metres.

~~~text
map -> odom -> base_link -> laser
                        -> imu_link
~~~

`T_map_odom(t) = T_map_base(t) * inverse(T_odom_base(t))` using poses aligned to the same acquisition timestamp. Global corrections may jump; local odom may drift but must not jump. All sensor extrinsics are calibrated and versioned, including the LiDAR lever arm.

Native occupancy grid follows ROS semantics: cell index `j * width + i`, origin = lower-left corner of cell (0,0) with an optional nonzero yaw; cell-center position = `T_map_grid * ((i+0.5)r, (j+0.5)r)`. Convert top-down raster rows deliberately. Preserve unknown cells and trinary export thresholds.

### 3.2 Typed native contracts (illustrative, not source code)

**Accepted OD-001:** use a signed `std::int64_t` **nanosecond count**, with **explicit domain and epoch identity**. Production acquisition time is monotonic *within an epoch*; the ROS simulation/replay clock is controllable and may jump. Keep raw device ticks and host receive/steady timestamps separate from aligned acquisition time.

~~~cpp
using TimeNs = std::int64_t;  // nanoseconds; no implicit Unix epoch
struct CanonicalTime {
  TimeNs ns;
  ClockDomainId domain;         // normalized steady/run vs ROS simulation timeline
  TimeEpochId epoch;            // invalidates comparisons after discontinuities
};
struct Timestamp {
  CanonicalTime acquisition;    // mapped physical/simulation acquisition time
  SourceClockStamp original;    // unmodified sensor tick / bag / ROS stamp
  HostSteadyStamp receive;      // diagnostic / timeout clock, separate domain
  TimeQuality quality;          // exact/reconstructed/uncertain/invalid
  double sigma_seconds;         // or explicit unavailable
};
struct TimedBeam {
  double range_m, angle_rad, intensity;
  CanonicalTime acquisition;
  bool valid;
};
struct PoseEstimate {
  Pose2 T_frame_base;
  FrameId reference_frame;
  Timestamp stamp;
  Matrix3 covariance;          // x,y,yaw with documented tangent/frame
  TrackingState tracking;
  RevisionId calibration_id;
};
struct MatchResult {
  Pose2 T_submap_base;
  Timestamp scan_reference;
  SubmapId submap_id;
  RevisionId map_revision;
  MatchQuality quality;         // score, inliers, overlap, observability
  OptionalInformation3 information;
};
~~~

Use explicit physical units, coordinate-frame labels and data validity. Define pose-covariance tangent order and left/right perturbation convention. A raw device tick, host monotonic nanosecond and ROS time are **different clock domains** until related by an explicit, versioned mapping. Nanosecond arithmetic is checked for overflow; subtraction/comparison across unrelated domains or epoch IDs is an error. A native nanosecond counter has no automatic Unix epoch.

### 3.3 Clock alignment

~~~text
raw device tick + host receive time
            -> unwrapping/clock model
            -> estimated acquisition time (+ uncertainty)
            -> bounded out-of-order reorder buffer
            -> trajectory + scan assembly
~~~

Preserve raw timestamps, arrival times, device clock IDs, alignment version and error estimates. Validate 30-second LD19 clock rollover, dropped packets and anomalous gaps. Never silently extrapolate IMU/wheel motion through long gaps. **OD-001 is accepted** (signed 64-bit nanoseconds, explicit domains/epochs and ROS simulated-clock support). **OD-002 and OD-003 are accepted:** Pi-only timestamps first, optional Pico later; the **LD19 adapter** reconstructs per-beam time using angles/speed and aligns its millisecond source counter to Pi monotonic time. The physical packet-reference delay is still an experimental calibration task. **Accepted OD-004** fixes midpoint SE(2) interpolation and **accepted OD-006** fixes the minimal wheel/gyro estimator with stationary bias calibration, uncertainty and replaceable kinematics. Calibration parameters and actual drive telemetry remain unverified.

### 3.4 Clock source injection and ROS 2 `/clock` behavior

The core MUST accept an **injected, ROS-neutral time source** or equivalent typed `ClockReading` / `ClockEvent` interface, with an initialized/ready flag, active clock domain, epoch and nanosecond value. The optional `fusion_slam_ros` adapter owns subscription to `rosgraph_msgs/msg/Clock` on `/clock`, `use_sim_time` configuration, conversion to/from `builtin_interfaces/msg/Time`, and ROS time-jump notifications. No ROS headers or clock types enter the core.

~~~text
live sensors / host steady timestamp -- explicit conversion --> canonical acquisition ns
ROS 2 use_sim_time + /clock ---------> ROS adapter -------> native clock source/events
offline MCAP bag stamps ------------> replay adapter ----> canonical event-time source
                                                    |
                                          timestamped estimation core
                                                    |
                      separate host steady clock -> deadlines, timeouts, CPU metrics
~~~

- **Live mode:** canonical acquisition time must be monotonic inside one acquisition epoch. Host steady time handles hardware timeouts and benchmark latency. Do not use wall-clock adjustments to integrate motion.
- **Simulation/replay mode:** active `/clock` supplies the authoritative **simulation event time** when `use_sim_time` is enabled. A zero/uninitialized ROS clock is *not* a valid current time; gate time-dependent outputs until it is ready. Message acquisition stamps must be mapped to that same simulation timeline; never assume equality with hardware-host steady time.
- **Pause:** do not advance estimator integration or sleep/deadline budgets based on an unchanged simulation clock. Explicit replay steps may process distinct event timestamps. Rendering may still refresh using independent host steady time.
- **Backward jump, seek, clock-source switch or reset:** increment/change time epoch, broadcast typed discontinuity event, invalidate cross-epoch interpolation and clear/rebuild estimator time history, scan assembler, reorder buffers and relevant caches; the ROS adapter also resets invalid TF state. Restore from a compatible replay checkpoint only when explicitly supported.
- **Forward jump:** distinguish ordinary accelerated simulation from a true discontinuity using a documented event/gap policy; never integrate across an unsupported long gap. Exact thresholds and seek checkpoint strategy remain **OD-002/OD-024**.
- **Conversions:** ROS seconds/nanosecond stamps are converted **integer-exactly with bounds/normalization checks**; an out-of-range timestamp fails explicitly. Preserve original ROS stamps and mapping provenance; avoid converting via floating-point seconds.
- **Testing:** ROS-free Catch2 tests cover ns precision, overflow, mismatched epochs, clock readiness, pause, step, backward jump, forward gap and mode switch. A dedicated ROS 2 Docker test drives `/clock` and `use_sim_time` through the real adapter; the core builds and replays without ROS.

ROS 2 design basis: [Clock and Time](https://design.ros2.org/articles/clock_and_time.html).

### 3.5 Accepted OD-002: Pi-first acquisition and optional Pico hardware-near timestamps

**Stage A (required first):** The Raspberry Pi obtains raw LD19 UART frames, MPU6050 readings and available wheel encoder events directly. The driver captures a host-monotonic timestamp at the closest **known** acquisition boundary (IRQ, sample-ready signal, driver read start/end, packet reception), records which event it represents, and carries the source clock, timestamp quality/uncertainty and latency diagnostics. Device sample time and host receive time remain distinct. Timestamping packet completion does **not** imply that all LD19 beams were acquired at that instant.

**Stage B (planned, optional):** A Raspberry Pi Pico (initial RP2040 target) acquires wheel encoder and IMU events under **custom Pico SDK firmware**, sends source-timestamped measurements to the Pi, and participates in explicit clock synchronization. The Pico does **not** run micro-ROS, DDS, ROS message serialization, or a ROS client; the host native driver does not link to micro-ROS either.

~~~text
Stage A (Pi 5 first):
  LD19 UART / MPU6050 I2C / encoder GPIO
    -> Pi native drivers + near-event monotonic host stamps
    -> raw measurements + time quality + diagnostics
    -> time_sync canonical int64 nanoseconds -> shared estimation pipeline

Stage B (Pico optional):
  MPU6050 data ready / encoder transitions
    -> Pico timer + ISR / optional PIO-based capture
    -> Pico 64-bit microseconds-since-boot + boot ID
    -> framed binary USB CDC (candidate) / validated UART transport
    -> Pi Pico driver + t1/t2/t3/t4 synchronization exchanges
    -> affine clock mapping (offset + drift + uncertainty + epoch)
    -> same time_sync native int64 nanosecond measurement API

  LD19 remains on Pi UART unless rerouting via Pico is specifically benchmarked.
~~~

**Pico timer and PIO feasibility:** The RP2040 SDK exposes a boot-relative 64-bit hardware timer in **microseconds**, e.g. `time_us_64()`. Host conversion to signed nanoseconds is checked and retains the true microsecond measurement resolution. PIO may offer precise deterministic GPIO edge counting or UART handling, but a PIO program is **not inherently a 64-bit hardware timer capture unit**; translating PIO cycles/FIFO/IRQ events into the timer domain needs a designed and validated method. Compare ISR/timer against PIO, account for FIFO/DMA overflow, oscillator drift, GPIO pin allocation and CPU cost, and keep ISR as the fallback. **No assumed nanosecond accuracy**.

**Pico-host synchronization:** Use a bidirectional sequence with host transmit `t1`, Pico receive `t2`, Pico transmit `t3` and host receive `t4` in original clocks. Estimate `t_host_ns ≈ a*t_pico_us+b` from multiple exchanges; bound asymmetry and drift, reject high-RTT outliers, record fit version/uncertainty/validity. Sequence numbers, CRC, firmware/protocol version and Pico boot/session ID identify data integrity and resets. Reboot/reconnect/new session creates a new clock epoch and invalidates the old map until resynchronized. The exact transport, fit filters and synchronization cadence remain implementation tuning choices.

**Clock independence:** The Pico physical hardware clock must never pause/jump with ROS 2 `/clock`. The optional ROS adapter governs simulated event-time conversion to the OD-001 native time interface; hardware capture and clock synchronization are separate from simulation clock control. Live data remains available with no ROS installation.

**Acceptance:** Before approving Stage B as superior, measure timestamp jitter, synchronization residual and drift, packet/encoder/IMU loss, reconnect behavior and end-to-end latency on real hardware. Include Dockerized host protocol tests and isolated Pico SDK firmware builds, plus opt-in hardware-in-loop tests. The firmware may use SDK-supported C/C++ independently of the host's required C++20 standard; this does not relax the native core C++20 rule.

**Related:** OD-001 (accepted nanosecond time/domain/epoch), OD-002 (accepted acquisition staging), **OD-003 (accepted angle/speed timing and clock alignment, physical delay not yet calibrated)**, OD-021 (still open buffering), OD-023 (still open recording schema). [Pico timer](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_timer/include/hardware/timer.h) and [PIO API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) document primitives, not achieved accuracy.

## 4. LiDAR assembly, wheel/gyro odometry and deskew

### 4.1 LD19 acquisition and accepted OD-003 beam timing

**LD19-specific timing logic lives entirely in the acquisition adapter**, not in the generic SLAM estimator. The 47-byte CRC-validated UART packet contains 12 samples, start/end angles (0.01° units), reported motor angular speed (degrees/s), and a millisecond counter wrapping at about 30 seconds.

~~~text
LD19 UART (230400 baud) + Pi-monotonic receive observation
    -> frame sync, CRC/length and packet decoding
    -> angle unwrap / rotation direction / reported angular speed
    -> per-beam relative acquisition-time reconstruction
    -> device millisecond tick unwrap + epoch/reset detection
    -> robust device-to-Pi affine clock alignment (offset + drift)
    -> calibrated/bounded packet-reference delay + timing uncertainty
    -> ordered native TimedScan:
       canonical int64-ns beam stamps + clock-domain/epoch + quality
    -> generic deskew, recorder, mapping/localization and Web Lab
~~~

For `N` beam slots (normally 12), interpolate the unwrapped packet sweep angle then estimate time offset from a **physical reference angle/event** using the packet's signed motor speed:

~~~text
theta_i = theta_start + i/(N-1) * (theta_end - theta_start)
delta_t_i_anchor = (theta_i - theta_anchor) / omega_signed
t_device_beam_i ~= t_packet_device
                + delta_t_i_anchor + delta_physical_anchor
~~~

`theta_anchor` is a **documented mathematical reference angle** (for example the end angle) used solely to compute relative beam offsets. It does **not** imply the device timestamp represents that beam. The corresponding physical acquisition-to-packet timestamp offset `delta_physical_anchor` is **unknown initially** and must be calibrated or bounded. This avoids introducing two redundant free reference parameters; the manual does **not** prove the timestamp refers to the first/last beam or UART transmission completion. Use measured/bounded delay; quality flags and systematic timing uncertainty MUST remain available before calibration. Cross-check angle-derived duration against speed, adjacent packet timestamps, and scan continuity. Reject/flag near-zero speeds, implausible angles and invalid motor estimates.

**Unwrap and alignment:** Retain raw 16-bit millisecond ticks and original UART bytes. Handle nominal 30-second rollover, CRC corruption, reordering and sensor reset with separate source epochs. Gaps that may hide multiple wraps make an absolute counter count ambiguous: invalidate/relock rather than guess. Timestamp the documented Pi UART receive event (first byte, last byte, packet completion or read completion), retain its meaning, and fit `t_host_rx_ns ≈ a * t_device_ns + b + d_rx` from several valid observations. Track slope/drift, residuals, window, uncertainty and fit revision.

**One-way timing limitation:** `b` (clock epoch offset) and constant `d_rx` (sensor processing + UART/host receive delay) are **not separately identifiable** from one-way packet arrival samples. An arrival-time regression gives a combined offset. The physical packet-to-beam delay must be measured or bounded with controlled motion/reference experiments; neither a PIO capture nor a precise UART receive timestamp alone proves beam acquisition time. Explicitly model device 1 ms resolution, UART serialization, variable OS buffering, oscillator drift and shared per-packet bias.

**Output contract:** The adapter produces a generic ordered `TimedScan` with per-beam canonical nanosecond acquisition estimates, timestamp status (`estimated`/`calibrated`/`uncertain`/`invalid`), per-beam or shared uncertainty, source epoch, clock fit and calibration revision IDs. Preserve all raw data and allow offline re-timestamping/re-deskew. Never silently substitute nominal scan rate or mark unknown absolute delay as exact.

**Validation:** Synthetic variable-speed/0°-360°/direction tests; 29,999→0 and ambiguous multiple wraps; CRC/out-of-order and reconnect faults; controlled delay/clock drift/jitter injections; physical LD19 turn/translate tests estimating/bounding reference delay; deskew comparisons with fixed-spacing baseline and Pi p95/p99 timing. Tests are Dockerized; hardware measurements must be reported separately.

**Decision boundary:** OD-003 is **Accepted** (device timing method) and OD-004 is **Accepted** (midpoint frame and piecewise SE(2) trajectory); physical LD19 timing calibration remains experimental.

### 4.1.1 Accepted OD-005: seam-based partial-scan assembly

OD-003 LD19 adapter decodes CRC-validated packets and emits **generic time-ordered observed beam slots** with angle, intensity, range classification, clock quality and reconstructed beam time. The portable **ScanAssembler** groups them by **fixed configurable angular seam crossings** (0° as initial example), after normalizing the verified rotation direction and unwrapping beam angles. It does not segment normal scans by packet boundaries or elapsed wall time. If a packet straddles the seam, partition its **observed beam slots** between revolutions; do not invent, drop or duplicate samples.

~~~text
LD19 adapter (CRC + OD-003 beam timing) -> observed TimedBeam + gap events
   -> fixed-seam ScanAssembler -> AssembledScan {
        observed beams and time interval; partial/complete and seam status;
        circular angular coverage; missing sectors/largest gap;
        range/intensity status; timestamps and quality; termination reason }
   -> scan-matching eligibility -> OD-004 midpoint deskew -> Ceres
   -> map-insertion eligibility -> safe occupancy updates
~~~

Coverage is computed as **observed support on a circular 360° angular domain**, not just the span between min/max observed angles. Retain covered/missing angular intervals, largest missing sector, coverage fraction, complete/partial flags, beam counts, packet loss, valid-hit count, time gaps, seam confidence and configuration revision. Start/end mid-revolution or missing sectors are honest **partial scans**. Ambiguous direction changes, multiple potential unseen seam crossings, or device resets require resynchronization rather than fabricated revolutions. Coverage support granularity and quality thresholds are configurable, not universally preset.

| Return | Meaning | Mapping effect |
| --- | --- | --- |
| `VALID_HIT` | Trustworthy observed obstacle | Free cells **before** hit; occupied endpoint, after all gates |
| `VALID_NO_RETURN` | Sensor-documented trustworthy no-hit/max-range | **Optional** bounded free ray only when explicitly enabled |
| `INVALID_RETURN` | Zero/corrupt/saturated/unsupported value without trusted no-hit semantics | **No free or occupied update** |
| `MISSING` | No observation in lost packet or uncovered sector | **No ray or update**; only coverage metadata |

Observed beams with invalid ranges may still have valid **acquisition timestamps** and contribute to OD-004 scan time bounds, while contributing **no geometry or clearing**. Missing beams have **no invented timestamps**. Intensity and its validity remain recorded.

Apply separate, configurable **matching and insertion** quality gates: circular angular coverage, largest gap, spatial support/count of valid hits, source-clock quality, angle/time consistency, and pose reliability. A partial scan MAY pass a gate if its real support is sufficient. Normal map insertion additionally requires OD-004 **full interpolation across the actual observed scan interval**, including time-valid invalid-range slots. No extrapolated scan is inserted. Reject with reason and preserve raw data; match against a map that does not already contain that scan.

**Timeouts are recovery-only:** Host-steady watchdogs may mark a stalled/disconnected/shutdown scan partial and flush/abort it, then re-lock at the next trusted seam. A timeout is **not** a normal revolution segmentation mechanism.

This assembler belongs to portable C++20 code and consumes **generic timed beams and gap diagnostics**, not LD19-specific packet fields, Pico/IMU/encoder data or ROS types. ROS adapters must not falsely represent arbitrary partial/nonuniform beam times as a regular `LaserScan`. The Web Lab can display missing sectors, coverage and rejection reasons.

Test in Docker with Catch2: packet spanning seam, seam phase, both directions, missing gaps/multiple possible turns, partial first/last scans, invalid returns versus validated no-return, clearing safety, full-interval midpoint trajectory coverage, stalled-source timeout, and exact circular coverage on synthetic apartment/warehouse scans.

### 4.2 Accepted OD-004: continuous SE(2) trajectory

Build `T_odom_base(t)` with the **accepted OD-006 minimal measured-wheel-translation + bias-corrected gyro-yaw integrator**, maintaining planar pose, body twist, gyro bias, and uncertainty. **Accepted OD-004** fixes piecewise-constant body twist and the SE(2) exponential trajectory. Do **not** assume a differential-drive or Ackermann drivetrain until physical geometry and measured telemetry confirm it; select kinematics via a replaceable native adapter.

Represent motion over each valid event-time interval `[t_k,t_{k+1}]` as a **piecewise-constant body-frame twist** `xi_k=(v_x,v_y,omega)_k` (metres/s, metres/s, radians/s). For a nonholonomic platform `v_y=0` may be assumed by the appropriate estimator. Use **time differences in seconds** from the OD-001 signed-nanosecond clock, with explicit domain and epoch checks.

~~~text
T_odom_base(t) =
    T_odom_base(t_k) * Exp_SE2( ((t - t_k) seconds) * hat(xi_k) )
    for t_k <= t <= t_{k+1}

T_odom_base(t_{k+1}) =
    T_odom_base(t_k) * Exp_SE2( ((t_{k+1} - t_k) seconds) * hat(xi_k) )
~~~

`Exp_SE2` is the **SE(2) Lie-group exponential** with exact constant-twist/constant-curvature translation and a stable near-zero angular-rate limit. **Do not interpolate Cartesian x/y and yaw independently** as a substitute. Integrate segments from the actual timestamps of asynchronous wheel/gyro updates rather than inventing a fixed sample period. Track measurement quality, gyro bias, slip, covariance and invalid gaps; never interpolate across clock epoch boundaries.

**Only when independently driven left/right wheel observations are verified**, a differential adapter may compute `ds=(ds_R+ds_L)/2` and `dtheta_wheel=(ds_R-ds_L)/baseline`. An Ackermann adapter requires **measured** steering feedback and wheel speed, not just steering commands. Gyro yaw uses `dtheta_gyro=integral(omega_z-b_g)dt`. The integrator's local odom trajectory stays continuous under global map corrections.

#### 4.2.1 Portable trajectory-provider interface

The portable deskewer queries an **abstract timestamp-addressable SE(2) trajectory provider**. The accepted OD-006 motion estimator (or a future EKF/smoother) owns construction; the deskewer has **no direct dependencies on wheel encoders, MPU6050, Pico, LD19 packet data, ROS or TF types**. A future spline implementation MUST satisfy the same query/coverage/status contract.

Illustrative, not yet implemented:

~~~cpp
struct TrajectoryQuery {
  CanonicalTime time;                    // ns, clock domain, epoch
  bool allow_extrapolation = false;      // default: strict interpolation
  std::int64_t max_extrapolation_ns = 10'000'000;
};
enum class TrajectoryStatus { INTERPOLATED, EXTRAPOLATED, UNAVAILABLE };
struct TrajectorySample {
  Pose2 T_odom_base;
  Matrix3 covariance;                   // frame/tangent order documented
  TrajectoryStatus status;
  std::int64_t extrapolation_ns;        // zero if interpolated
  RevisionId trajectory_revision;
};
class TrajectoryProvider {
public:
  virtual ~TrajectoryProvider() = default;
  virtual CoverageResult coverage(CanonicalTime start, CanonicalTime end) const = 0;
  virtual TrajectoryResult evaluate(const TrajectoryQuery&) const = 0;
};
~~~

`coverage` must signal whether **all instants** in the requested interval are supported by valid interpolated segments and distinguish missing sensor data, gaps, epoch mismatches and not-yet-received samples. Query outputs include time/quality, covariance and trajectory revision. Implementation-specific error types will be agreed under OD-017.

#### 4.2.2 Accepted OD-006: state, kinematic adapters and gyroscope bias

**Initial state:** `T_odom_base`, `xi_body=(v_x,v_y,omega)`, gyro Z-rate bias `b_g`, a documented pose/bias uncertainty representation and time/quality metadata. A candidate 4-dimensional pose+bias error state `(δx,δy,δyaw,δb_g)` with **pose–bias cross covariance** is appropriate for the minimal integrator. Define frame, tangent ordering and noise units explicitly, and test propagation against synthetic truth rather than assuming diagonal errors.

~~~text
timestamped wheel measurements -> KinematicModel (selected by verified hardware)
                                        |
                            body translation + optional independent wheel yaw
                                        |
gyro Z rate + stationary bias calibration -> omega_z - b_g
                                        |
             minimal MotionEstimator + uncertainty/slip monitor
                                        |
                   piecewise body-twist Exp_SE2 trajectory
                                        |
             abstract TrajectoryProvider -> OD-004 midpoint deskew
~~~

- The **replaceable KinematicModel** maps *observed* encoder/wheel/steering inputs into body-frame motion and uncertainty, plus the wheel-heading observable where available. Differential kinematics require verified independent wheel drive/encoder data, wheel radii, polarity and baseline. Ackermann requires geometry and **actual measured steering angle**. Unknown drivetrain is **UNCONFIGURED/DEGRADED**, not automatically differential-drive. Previously fused odometry is labeled according to provenance and not treated as independent raw wheel ticks.
- Integrate wheel-derived translation and `omega_z-b_g` via OD-004 `Exp_SE2` at actual asynchronous sample times. Carry pose/bias correlation, gyro noise/bias random walk, wheel scale/slip, timing quality and increasing uncertainty through missing data. Do not invent zero yaw or wheel travel when a source is absent. Never bridge invalid clock epochs or gaps without an explicit degraded policy.
- **Stationary startup calibration** estimates gyro zero-rate bias only in a validated stationary window (wheel-no-motion, gyro stability, accelerometer if meaningful), with configured duration, sample count and outlier criteria. Store estimated bias, variance, sensor temperature if available, calibration revision and readiness. If motion/stationarity cannot be ruled out, mark bias **UNCERTAIN/UNINITIALIZED**, increase covariance, and gate operations needing a trusted motion model.
- **Online bias correction** is intentionally conservative: allow bounded adjustments in independently validated stationary windows, or with separately justified external observability and correct covariance handling. Never let wheel slip or gyro/wheel disagreement automatically train the gyro bias. An update alters **future** integration; replay/reintegration of past data must create a new revision rather than change previously published odom.
- **Wheel/gyro disagreement:** if the kinematics provide independent wheel-derived yaw, compare that yaw with integrated gyro yaw on the same interval and uncertainty model. Detect slip, encoder faults and calibration mismatch, grow uncertainty and report a degraded health state. Without independent wheel yaw, report the monitor unavailable.
- The local predictor's fused pose is **not an additional independent GTSAM factor** when the graph already contains factors from the same raw wheels and gyro. Preserve factor provenance and account conservatively for the correlation between gyro/wheel-conditioned deskew and LiDAR constraints.
- An **EKF, error-state filter or recent-state smoother** is a later drop-in estimator behind `MotionEstimator`/`TrajectoryProvider` only if benchmarks establish a benefit. No ROS, MPU6050, LD19, Pico or hardware-specific types may leak into these native interfaces.

**Test requirements:** synthetic translation/curves/rotation, asynchronous sensor rates, differential and Ackermann adapters with matching **verified-style** telemetry fixtures, unsupported drivetrain, stationary vs moving bias calibration, bounded online bias updates, slip disagreement with covariance inflation, missing data and bias drift, Monte Carlo covariance coverage, OD-004 no-extrapolation map insertion, and interchangeable mock estimator. Run Catch2/CTest in Docker; hardware bias thresholds and drivetrain facts remain to be measured.

### 4.3 Full per-beam SE(2) deskew into the scan midpoint

Define the scan's acquisition time bounds from **earliest/latest time-valid acquired beam slots**, including slots with invalid range values; only valid geometric returns contribute endpoints. For one canonical domain/epoch:

~~~text
t_start = min(timestamp of time-valid acquired beam slots)
t_end   = max(timestamp of time-valid acquired beam slots)
t_mid   = t_start + floor((t_end - t_start)/2)   // checked int64 nanoseconds
~~~

`t_mid` is the **temporal midpoint**, not a mean packet-arrival time, angular midpoint, or arbitrarily chosen beam timestamp. Every deskewed point `p_i` becomes:

~~~text
p_laser_mid =
    inverse(T_odom_base(t_mid) * T_base_laser)
  * (T_odom_base(t_i) * T_base_laser)
  * p_laser_i
~~~

Use the **continuous odom trajectory**, not a discontinuously corrected `map` pose. Include the full wheel-translation/gyro-rotation SE(2) path and calibrated LiDAR lever arm. Downstream Ceres scan matching and scan-derived pose estimates MUST use `t_mid` and the matching midpoint frame; raw source beam timestamps remain unchanged.

**Complete interpolation for normal mapping:** After the OD-005 matching/insertion quality gates, before scan insertion require a quality-valid interpolated trajectory covering `[t_start,t_end]` including every beam time and `t_mid`, in one clock epoch. **Never insert a scan requiring extrapolated poses** or a scan whose boundary timing is ambiguous, even if its numerical residual is small. Preserve partial/invalid scan diagnostics and map-eligibility reason.

**Bounded, optional prediction:** Prefer a bounded delayed-processing queue while waiting for real trajectory data through `t_end`. If no valid coverage arrives within a configurable host-steady wait budget, reject/defer with diagnostics rather than predict without bounds. Only explicit live prediction, visualization or localization policies MAY permit extrapolation; default is **disabled**, maximum initial horizon is **10 ms (10,000,000 ns)** beyond either valid trajectory boundary, and covariance/sensor-age/clock-quality/gyro-bias/slip gates may require a **shorter** horizon or reject it. An extrapolated result MUST be distinctly marked `EXTRAPOLATED`, include duration/uncertainty and MUST NOT be passed to normal map insertion or silently treated as high-confidence scan evidence. Never cross time epochs, invalid gaps, or stale calibration.

**Spline future extension:** The trajectory interface allows a continuous SE(2) spline alternative later, but the accepted first implementation is piecewise-constant twist and `Exp_SE2`; do not add spline dependencies or complexity speculatively. Offline re-deskew after improved bias/time calibration remains an OD-014 question.

**Validation:** Synthetic apartment/warehouse constant-curvature truth, scan temporal midpoint with nonuniform beam spacing, asynchronous IMU/encoder updates, off-center laser lever arm, missing samples, seek/time jumps, complete-coverage mapping gates, and quality-gated 9.9/10/10.1 ms extrapolation cases. Compare latency of bounded delay vs prediction on Pi.

## 5. Accepted OD-007: Ceres EDT scan matching and evidence accounting

### 5.1 Authoritative occupancy and derived distance field

The **authoritative spatial map** remains the native **ROS-compatible occupancy grid**, with unknown, free and occupied probabilities, resolution, origin (including rotation), and immutable revision ID. The matcher receives a **versioned truncated unsigned Euclidean distance transform (EDT)** computed from **confidently occupied** cells, not a second independent representation of the map. Threshold selection belongs to OD-010; derived-cache lifecycle, tiling and invalidation belong to OD-011.

- EDT values are **metres to nearest confidently occupied cell**, truncated at configurable `D_max`. Cache metadata references the **exact source occupancy revision/checksum**, confidence threshold, grid origin/resolution, truncation, unknown-region support mask and interpolation/version settings.
- **Unknown cells are neither occupied obstacles nor known free space**. Store a separate supported/observed validity mask; a numeric EDT value in an unknown/unsupported region does not make it a reliable scan-matching observation. Count/map-mask unsupported endpoints instead of treating unknown as zero distance.
- All scan matches pin a coherent immutable occupancy/EDT revision. A changed map creates a **new cache revision** and never modifies a matcher worker's field in place.
- Initial interpolation is **bilinear**, with documented grid-world transforms, masked-stencil validity, cell-boundary derivative behavior, and finite/valid interpolation domain. Consider **bicubic only after benchmark gains** outweigh overshoot/complexity and runtime costs.

### 5.2 Optimization pose, residuals and priors

The input to `ScanMatcher` is a **quality-accepted OD-005 AssembledScan** after **OD-004 deskew into the midpoint laser frame**, OD-006 predicted midpoint robot pose/covariance, laser extrinsic and a pinned immutable EDT. For a beam endpoint `p_i` in the midpoint laser frame, the solver varies `T = T_map_base_mid` (or the equivalent referenced submap pose), transforms `q_i = T_map_grid_or_submap^-1 * T * T_base_laser * p_i`, and samples the EDT at `q_i`.

~~~text
D(q) = clamp( distance_to_nearest_confident_occupied(q), 0, D_max )

r_i(T) = sqrt(w_i) / sigma_i * D(q_i(T))
e_prior(T) = Log( T_pred^-1 * T )
L^T L = Sigma_pred^-1

E(T) = 0.5 * sum_i Huber_delta( r_i(T)^2 )
     + 0.5 * ||L * e_prior(T)||^2
     + 0.5 * sum_i r_boundary_i(T)^2
~~~

`w_i` is validated nonnegative beam confidence; `sigma_i > 0` is effective endpoint/map uncertainty in metres. The Huber scale `delta > 0` is expressed in normalized residual units. The formula follows Ceres's convention of **one-half the robustified squared norm**. **Do not emit squared distances as residuals** and then allow Ceres to square them again.

- **One scalar Ceres residual block per valid beam**, each with its own `ceres::HuberLoss(delta)`. Grouping all beam residuals into one block under a single Huber robustifies the **combined** norm, not each outlier.
- Add the **3-vector, covariance-whitened SE(2) odometry prior** as a separate normally unrobustified residual block. Compute `T_pred` and `Sigma_pred` from OD-006 at the same **scan midpoint**, in the same frame and documented Lie tangent ordering; check positive-definiteness and proper covariance transformation before whitening.
- The odometry prior is a deliberate **local search regularizer**, not merely the initial pose. It must have separately recorded contribution, validity and provenance; absent/invalid covariance cannot silently become a zero-variance anchor.
- **Out-of-grid endpoints:** add explicit finite, outward-increasing smooth/hinge-style boundary residuals based on metric distance to the grid's supported boundary, with meaningful inward gradient. Do not clamp outside coordinates to an edge-value constant or conflate map **unknown** with out-of-grid. Make unknown-region validity policy explicit and testable. Avoid introducing unbounded costs at extreme positions.
- Saturated `D_max` distances often have zero gradient. Reject low-information scans unless enough known, nontruncated, spatially distributed endpoints remain; a strong prior cannot substitute for actual LiDAR geometry.

### 5.3 Evidence separation and acceptance

**Crucial OD-006/OD-013 compatibility:** The regularized optimized pose combines **LiDAR + a wheel/gyro odometry prior**. It is a **posterior pose**, not a statistically independent LiDAR observation. If GTSAM separately receives wheel and gyro factors from those same measurements, it MUST NOT receive that posterior pose as an independent LiDAR-only factor. Instead preserve separate **LiDAR-only geometric measurement/diagnostics**, prior provenance/covariance and the final regularized pose; **accepted OD-008** specifies LiDAR-only rank/uncertainty assessment; correlated graph factor extraction remains **OD-013**. Do not assert an exact independent LiDAR covariance merely by subtracting prior information from a posterior Hessian; deskew/map conditioning is correlated.

The native **`ScanMatcher`** returns pose, convergence and failure reason; **LiDAR-only score and geometric-only information/eigenvalues**; inliers/overlap and informative/nontruncated counts; raw/robust residual quantiles; prior pose/covariance and cost; boundary/unknown/outside counts and costs; solver iterations/runtime; source scan/calibration and pinned occupancy/EDT revision; and conservative pose uncertainty (or an explicitly unavailable status).

**Required quality gates before accepting a correction:**

1. OD-005 valid-hit geometry/coverage, OD-004 trajectory coverage where mapping, valid time/frame/calibration and covariance.
2. Sufficient known/observed supported-map **overlap** and spatial distribution of informative LiDAR endpoints; unknown/out-of-grid samples cannot count as valid inliers.
3. Finite geometric residual statistics, bounded outlier fraction and rejection of **boundary-dominated** or truncation-saturated solutions.
4. **LiDAR-only** observable SE(2) directions from geometry-only information/Hessian, with condition/eigenvalue diagnostics evaluated **without** the odometry prior and boundary term; a strong prior may stabilize a pose but does **not** make the scan informative.
5. Convergence status plus pose jump and Mahalanobis consistency against the OD-006 prediction, with **accepted OD-008** rank/quality gates and held-out-calibrated thresholds. A solver `SUCCESS` flag or low posterior objective is insufficient by itself.

**Graph information table:**

| Evidence | Frontend use | Backend use | Constraint |
| --- | --- | --- | --- |
| Wheel increments | Prediction, odometry prior and deskew | Raw wheel motion factors | The Ceres regularized posterior is **not another independent wheel measurement** |
| Gyro rate | Bias-corrected trajectory, odometry prior and deskew | Yaw/bias factors | Propagate shared conditioning and covariance conservatively |
| LiDAR hits | Separate geometric EDT residual | Geometric scan-to-submap / loop evidence | Separate from prior and boundary terms; avoid map self-matching |
| Map support | Interpolation/validity and boundary residual | Immutable reference revision | Boundary penalty is **not** observed wall geometry |

### 5.3.1 Accepted OD-008: independent quality/uncertainty evaluation

**Separation of responsibilities:** The private **Ceres adapter** optimizes the OD-007 prior-regularized pose and returns **only LiDAR residuals/Jacobians** plus solver, prior, map and acquisition diagnostics through native types. A separate native C++20/ Eigen **`MatchQualityEvaluator`** performs information analysis, covariance calibration and final correction gating **without linking to Ceres, ROS or device SDKs**. Solving a regularized objective and judging geometric reliability are separate operations.

For each valid, observed, map-supported, nontruncated beam, `r_i` is the normalized OD-007 scalar `sqrt(w_i^beam)/sigma_i * D(q_i(T))`, `J_i` its 1×3 SE(2) tangent Jacobian, and `alpha_i=rho_delta'(r_i²)` its **Huber influence**. `w_i^beam` is already inside `J_i`; do **not** apply it again.

~~~text
H_L = sum_i alpha_i * J_i^T * J_i     // LiDAR geometry only, PSD
// Exclude: OD-007 odometry prior, out-of-grid penalty, invalid/
// unsupported/unknown points, graph priors and map-update factors.

delta_xi = S*z     // physical tangent scaling; x/y metres, yaw radians
H_norm = S^T * H_L * S = V diag(lambda_1, lambda_2, lambda_3) V^T
rank_obs = count(valid information eigenvalues)
~~~

`S` is a **recorded normalization** with meaningful pose scales. Never compare unscaled mixed-unit Hessian eigenvalues as an invariant degeneracy test. A robust `rho''` curvature term is **excluded** from this PSD influence-weighted Gauss–Newton information; compare the implementation to Ceres manifold-aware Jacobian evaluations, rather than assuming Ceres's built-in covariance yields identical numbers.

**Rank and covariance:**

- For **rank 3 and acceptable condition**, `Sigma_raw≈H_L^-1`; calibrate `Sigma_match=K*Sigma_raw*K^T+Sigma_floor`, with validated finite inflation and symmetric PSD model-error floor. Track calibration ID, training/evaluation split, tangent frame and covariance status. Noise normalization, correlated LiDAR beams, conditioned deskew and uncertain occupancy invalidate any claim of exact independent covariance.
- For **rank 0–2**, return **observable-space information/directions and null-space directions explicitly**, with unavailable/unbounded variance in unsupported dimensions. **Never** reinterpret the pseudo-inverse or a covariance floor as a valid full-rank covariance. The evaluator may **project** the correction relative to OD-006 predicted pose into the normalized supported subspace, preserving unsupported predicted coordinates, and recheck quality; otherwise **reject**. The resulting pose carries `ACCEPT_PROJECTED` or a reasoned rejection, **not** a full-confidence global fix.
- Define separate **full covariance**, **observable subspace only**, and **unavailable** uncertainty states. The ROS adapter must not fill missing/degenerate directions with falsely small finite covariances; suppress or clearly degrade outputs when native uncertainty cannot be faithfully converted. GTSAM graph factor handling with rank deficiency and shared wheel/gyro evidence remains **OD-013**.

**Quality gates, including rejection reasons:** Require solver termination/finite Jacobians; OD-005 valid-hit angular distribution, support and OD-004 time coverage; known-map overlap and informative nontruncated beams; LiDAR-only robust raw/residual statistics and outlier rate; normalized eigen-rank and conditioning **excluding the motion prior/boundary**; predicted-pose innovation using conservative correlation-aware uncertainty; OD-003 beam-time accuracy, OD-006 slip/bias/source gaps, map/EDT revision and calibration validity. Strong odometry may stabilize the returned posterior but must **not** improve LiDAR-only observable rank. Publish diagnostics for accepted and rejected scans, including all prior/EDT IDs and reasons.

**Versioned calibration:** Fit `K`, `Sigma_floor` and rank/innovation/quality thresholds on held-out apartment/warehouse and genuinely independent trajectory-reference data. Include corridors, corners, open regions, partial scans, high speeds and degraded encoder/IMU/timestamps. Report covariance coverage, false accept/reject, overconfidence and runtime; distinguish synthetic exact from LiDAR-derived self-reference. An optional offline bootstrap can cross-check estimates but must acknowledge correlated beams.

~~~text
Ceres EDT solve -> native r_i/J_i + prior/boundary/solver diagnostics
                                |
                 MatchQualityEvaluator (no Ceres/ROS)
                                |
           H_L -> physically scaled rank/eigen-directions
                   /                             \
          rank=3, calibrated                 rank<3
          full covariance                 observable subspace
                   |                         /          \
             ACCEPT_FULL               PROJECT      REJECT
                   \                         |          /
                 native MatchResult + reason/provenance
                  -> localizer / Web Lab / guarded OD-013 graph
~~~

**Test obligations:** Docker Catch2 unit tests with hand-constructed rank 0/1/2/3 Jacobians, Huber influence vs original beam confidence, PSD and frame scaling, prior-independent rank, finite covariance calibration, projected null-space preservation, nonfinite/outlier rejection and missing data. Ceres-linked integration tests verify extracted Jacobians and optional `ceres::Covariance` comparison; held-out B16 benchmark calibrates error/coverage and false accepts. No dependency creep into public domain interfaces.

### 5.4 Private Ceres implementation and replaceable frontend

~~~text
native OccupancyGrid revision (source of truth)
     -> confidence occupied mask + supported/known mask
     -> immutable EDT cache (revision, truncation, metadata)
                                           |
OD-005 AssembledScan -> OD-004 midpoint deskew -> valid hit endpoints
OD-006 midpoint predicted pose + covariance ----------+
                                                      v
                            ROS/Ceres-free ScanMatcher interface
                                                      |
                          private Ceres EDT implementation:
                          bilinear distance + per-hit Huber,
                          whitened odom prior + boundary residual
                                                      |
                        native MatchResult (pose, separated evidence,
                                 quality, uncertainty, revision IDs)
                                                      |
                        localization / map gate / OD-013 backend
~~~

Core interfaces MUST NOT expose Ceres types, solver summaries, residual blocks or ROS types. The Ceres-based implementation links Ceres privately and emits generic native results. Alternate objectives (occupancy probability, signed distance, features), different interpolators (possibly Ceres's `BiCubicInterpolator`), and search initialization may evolve without changing OD-001–OD-006 timing/trajectory interfaces or the occupancy-grid schema.

**Validation:** Docker Catch2/Ceres synthetic fixtures for known Euclidean geometry, bilinear and derivative behavior at edges, masked unknowns, grid origin yaw, Ceres block layout/Huber scale, correct sqrt(weight)/sigma, SPD covariance whitening and Lie Log prior, boundary gradients, LiDAR-only observability despite a strong prior, degeneracy/false warehouse minima, residual quality/coverage gating, B15 performance and posterior/prior graph independence. OD-008, OD-010 and OD-011 still control covariance calibration, map threshold and cache design.

### 5.5 Accepted OD-009: multi-mode coarse search and asynchronous branch-and-bound

**Responsibilities:** OD-007 remains the **precise Ceres pose optimizer**; a separate **native `PoseCandidateSearch`** supplies coarse hypotheses when OD-008 tracking quality warrants it. The coarse score and compute budget are independent of OD-007 residual/loss/iteration settings. Both use a coherently pinned **authoritative occupancy-grid revision**, but the coarse search uses a **distinct immutable, occupancy-derived correlation-score pyramid**, while OD-007 uses its immutable EDT cache. Neither ROS messages nor Ceres types leak into the coarse-search API.

| Tracking/search mode | Accepted initial search extent | Pi 5 coarse compute target | Scheduling |
| --- | --- | --- | --- |
| `TRACKING` | Ceres at OD-006 predicted midpoint pose | **0 ms** | Direct local Ceres, no coarse worker |
| `DEGRADED` / `UNCERTAIN` | **±0.3 m X/Y, ±10°** | **10–20 ms** per invoked scan | Bounded local correlative search, then Ceres |
| `RECOVERY` | **±1.0 m X/Y, ±30°** | **30–50 ms** per invocation | Multi-resolution branch-and-bound |
| `GLOBAL_RELOCALIZATION` | Full eligible frozen submap(s), **360°** | **100–200 ms per processing slice** | Asynchronous, interruptible/resumable branch-and-bound |

These are **proposed—not benchmark-validated—starting targets** for ~10 Hz LD19 on Pi 5, not hard-real-time commitments or guaranteed coverage. Record host-steady elapsed CPU/work time, evaluated/pruned nodes, peak memory, queue lag and true p50/p95/p99 latency. Cap candidate count, node count, memory, queue depth and time independently. A global search can span multiple slices and must report incomplete coverage explicitly.

~~~text
OD-006 midpoint prediction + covariance, OD-005/004 eligible scan
     |
     +--> TRACKING ---------------------> OD-007 Ceres direct
     |
     +--> quality degrades -> covariance-clipped local correlative search
     |                                        |
     +--> RECOVERY/GLOBAL -> async multires BnB on frozen score pyramid
                                                  |
                                    distinct ranked pose hypotheses
                                                  |
                     OD-007 Ceres refinement (mode-appropriate prior)
                                                  |
                       OD-008 geometric rank / quality / innovation
                                                  |
                        OD-020/022 multi-scan relock and state gate
                                                  |
                       accepted global correction, or odom-only
~~~

**Search geometry:** Adapt local X/Y/yaw candidate limits to OD-006 covariance in the **correct SE(2) tangent frame**, clipping to mode maxima and reporting the clip. Invalid/stale covariance invokes degraded/global state instead of a fictitious narrow search. Choose angular sample spacing from **map-cell spatial resolution and informative beam range**, e.g. `delta_yaw <= spatial_step / representative_range` (small-angle guide). Preserve proper angle wrap; deduplicate high-scoring candidates by physical SE(2) separation, not just grid index.

**Multi-resolution correlation cache:** Generate versioned, immutable score pyramids from confidently supported occupancy information and a known/unknown mask; preserve map origin/yaw, resolution, score-policy, source occupancy revision/checksum and memory/build cost. Reuse across scans; publish a new cache only for a new authoritative map revision. Do **not** read a live mutated grid or assume that an EDT pyramid and a correlative score are interchangeable.

**Admissible bounds:** To claim true **branch-and-bound**, coarser levels must provide **upper bounds of the exact finer-level score** across the candidate translation region and fixed yaw sample, including map-support masks and score quantization. Build using a verifiable max-over-descendants or equivalent. Test against exhaustive fine-grid enumeration and nonzero rotated map origins. Heuristic coarse averages are **not** safe pruning bounds and must not be represented as complete search.

**Hypotheses and integrity:** A high coarse correlation is a **proposal**, never a validated map pose or covariance. Return `PoseCandidate` lists with scan/map/clock/calibration revisions, pose/score/support, distinctness, uncertainty/search window, progress, consumed budget and `COMPLETE`/`PARTIAL_BUDGET`/`CANCELLED`/`INVALID` status. Timeout/partial search does **not** prove no other candidates exist. A resumed search requires the identical pinned map, scan and configuration; revision changes or clock jumps cancel/restart it.

**Prior and quality semantics:** In large relocalization, an incorrect old OD-006 pose and tight covariance must **not pull OD-007 Ceres back to the wrong corridor**. Use explicit relaxed/uncertain or no-prior mode for recovery refinement, with mode-specific and documented consistency checks. OD-008 still evaluates LiDAR-only overlap, residual and observability; OD-020/022 validate stable hypotheses across scans before relock. No mapping while LOST/RELOCALIZING; continue acquisition and continuous local odometry. Multiple similar hypotheses must not be silently collapsed to one global fix.

**Concurrency:** Run large BnB in a separate bounded worker, yielding by **host steady clock**, independent of pausable OD-001 ROS simulation `/clock`. Do not block serial acquisition, recording, OD-006 trajectory updates, live scan matching or ROS callbacks. Support cancellation, shutdown, stale revisions, throttled Web Lab status and overload-based deferral. Fine-grained worker/process and scheduling policy remains OD-016/021.

**Boundary:** The core owns a replaceable `PoseCandidateSearch` abstraction plus mode-budget and result structs; the OD-007 matcher and OD-008 evaluator stay independently callable. Particle filtering is **not an initial dependency**; OD-020 may revisit persistent multi-hypothesis filters only if BnB + temporal relock fails in ambiguous or kidnapped-robot cases.

**Verification:** Docker Catch2 synthetic apartments/warehouse B17: covariance-window clipping, angular resolution, admissible upper bounds, exhaustive comparison on small grids, unknown-sector/partial scans, rotated grids, top-K distinct hypotheses, wrong-prior Ceres refinement, OD-008 rejection, deadline/yield/cancel behavior, immutable cache reuse and no acquisition blocking. Pi budgets require **measured** hardware data.

**Reference:** [Cartographer 2D fast correlative scan matcher](https://google-cartographer.github.io/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.html) documents precomputed maximum-score pyramids, BnB pruning and Ceres refinement.

## 6. Accepted OD-010: 5 cm log-odds occupancy and overlapping frozen submaps

### 6.1 Authoritative occupancy and 90/180 lifecycle

**The occupancy grid is the sole authoritative spatial map**: initial **0.05 m** resolution, **log-odds** per observed cell, **separate known/unknown state**, bounded dynamic allocation, local SE(2) frame including rotated origin, and immutable source revision after freezing. Normally maintain **two overlapping active submaps** (one during startup). The configured initial lifecycle uses **90 accepted scan insertions to create a successor** and **180 accepted insertions per submap to freeze its local raster**.

~~~text
OD-005 valid scan -> OD-004 interpolated midpoint deskew
      -> OD-007/008 pre-insertion pose OR explicit EMPTY-MAP bootstrap
      -> motion filter + extent/memory guard
      -> same accepted scan ID into EACH active map, once per map
          |
      A: IDs 1-90; create B AFTER #90
      A+B: IDs 91-180; freeze A AFTER #180; create C
      B+C: IDs 181-270; freeze B AFTER #270; ...
          |
        frozen authoritative occupancy + observed mask
             |                            |
     OD-007 immutable EDT          OD-009 score pyramid
             \                            /
            localization, matching, loop closure
                              |
         GTSAM optimizes global T_map_submap only
~~~

**Count only actual accepted insertions**, not LiDAR packets, attempted Ceres matches, filtered/rejected scans or summed insertions across maps. Apply scan quality, OD-004 interpolation, pose acceptance and **motion filtering** before any update. Skipped scans remain recorded and continue feeding odometry, but increment no submap count. Every accepted scan is transformed into **each active submap's own local coordinates**, inserted **once in each**, and assigned a stable scan ID. A receives #1–180 and B #91–270: overlap is **90 common IDs / 180 ≈50% by count**, **not** guaranteed physical area. Scan #90 creates B only *after* A receives it; #180 freezes A *after* A+B receive it, then creates C. Never match against the occupancy revision **after** inserting the same scan.

**Spatial/memory safeguard:** bounded maximum cells, physical extent/travel, allocation bytes, active submap count and bounded update work. A cap may **freeze early or stop map insertion safely**: record the *actual* scan count, overlap, stop reason and map validity; never claim 180 insertions after an early freeze or synthesize a wall when a ray exceeds bounds. If a new bounded map cannot be allocated, preserve recording/odom and report mapping degraded rather than grow RAM without limit. Exact cap values and motion-filter translation/yaw/time thresholds require Pi benchmarks; alternatives 0.025 m (~4× cells) and 0.10 m (~¼ cells) are optional.

**Empty-map bootstrap:** The initial submap has no occupied EDT support, so its first insertions cannot be Ceres-verified against that same empty map. Start from an explicit submap gauge and trusted initialized OD-006 motion trajectory (or externally documented pose), require OD-003–005 validity, and mark these insertions **BOOTSTRAP_UNVERIFIED_MAP_MATCH** with conservative uncertainty. Once enough geometry exists, transition to OD-007/008 accepted **pre-insertion** matching. Never use bootstrap to bypass tracking loss or claim an independently verified global fix. Subsequent overlapping submaps inherit the established continuous tracked frame and map state.

### 6.2 Inverse-sensor log odds and hit precedence

~~~text
L_prior = logit(0.50) = 0                  // initially UNKNOWN
Delta_hit  = logit(0.55)                   // positive
Delta_free = logit(0.49)                   // negative
L_new = clamp(L_old + Delta, logit(0.10), logit(0.90))
p_observed = sigmoid(L_new)               // separate observed flag retained
~~~

Values are **initial tuning parameters, not validated sensor probabilities**. A genuinely **observed** cell returning to `p≈0.50` after contradictory readings must **not** become unknown. All unused cells remain unknown, never implicitly free. OD-007 occupied-confidence threshold and OD-009 supported/unknown masks derive from observed occupancy, not bare numeric log odds alone.

For `VALID_HIT`, raycast trusted **free cells strictly before** its real endpoint and mark the endpoint occupied; for `VALID_NO_RETURN`, only a documented and explicitly enabled bounded ray may clear free cells. `INVALID_RETURN` and `MISSING` under OD-005 produce **no free or occupied updates**, even at a scan's missing angular sector. Use calibrated laser origin/extrinsics and midpoint deskew in the local submap frame. **Within one scan and one submap**, gather unique free and hit cell sets, subtract hits from free (hit priority), and apply **at most one update per cell**. Prevent duplicate beam updates from over-strengthening occupancy and prevent a ray passing through another beam's hit from clearing it. Multiple later accepted scans may update the same cell again.

Clip traversal to known allocated support without **inventing a hit** at the edge or clearing beyond an observed/validated bounded ray. Invalid/saturated/out-of-range measurements are not max-range clear rays unless independently verified by OD-005. No dynamic time decay of authoritative frozen maps; a future **separate transient map layer** may have its own aging and versioning.

### 6.3 Freezing, independently moving graph poses and derived caches

When frozen (normal count 180 or an explicit safety/shutdown reason), the submap's **local occupancy cells, observed mask, local origin and insertion history become immutable**. Its **global pose `T_map_submap` is stored separately** and may change under GTSAM graph optimization. A global pose update must not modify local cells, source hash, EDT or correlation-score pyramid; only global pose tables/composite spatial indexes may require updates. Freeze and graph revisions are independently identified.

OD-007 **unsigned truncated EDT** and OD-009 **multiresolution correlation-score pyramid** are **distinct, versioned derived caches** built from the occupancy grid and known mask, with the source content revision/hash, cell resolution, occupied threshold, origin, algorithm and score/truncation parameters. Cache reuse is valid for an unchanged frozen **local** raster even when its global graph pose moves. Readers use coherent immutable snapshots of active maps; no matcher reads a concurrently mutating grid. **Accepted OD-011** chooses coherent snapshot publication, incremental active EDT tiles as the final architecture, an initial bounded full-rebuild baseline and frozen cache reuse; actual refresh/staleness/memory budgets remain performance tuning.

### 6.3.1 Accepted OD-011 — Immutable matching snapshot service

The **OD-010 occupancy grid and observed/unknown mask are authoritative**. OD-007's **truncated unsigned Euclidean distance transform** and OD-009's **correlative-score pyramid** are **two different derived caches**, each keyed by the source local occupancy revision and its own configuration (thresholds/masks, grid resolution/origin, EDT truncation/interpolation or scoring/bounds). Neither is independently mutable map truth.

**Final design:** Active maps provide **dirty-region tracking and incremental 32×32-cell EDT tile rebuilding**; frozen maps retain **immutable precomputed EDT/pyramids** in a memory-bounded on-demand cache. **First implementation:** keep the full architecture/API but generate a complete exact EDT and pyramid on a **bounded background refresh cadence** as a correctness baseline; defer tiles until Pi 5 profiling and B19 equivalence tests warrant complexity. Do not run a synchronous full rebuild on every raw scan.

| Baseline parameter | Initial value | Meaning |
| --- | --- | --- |
| Grid resolution | **0.05 m** (OD-010) | Local occupancy and source masks |
| Unsigned EDT truncation | **0.30 m** (configurable) | Not Cartographer's *signed* TSDF truncation |
| EDT interior tile | **32×32 cells**, **1.6×1.6 m** | At 5 cm/cell |
| Source/invalidity halo | **≥0.30 m + interpolation support** | **≥6 cells + ≥1 stencil cell** at 5 cm, plus edge/rounding safety |
| Worker and cache budgets | Configurable CPU/time, queue, memory | Empirically measured, never unrestricted |
| Snapshot staleness | Both host-steady age **and accepted-insertion lag** | Configurable caps; do not invent initial numeric defaults |

**Correct dirty propagation:** After an accepted occupancy insertion, detect both **occupied-confidence threshold crossings (including removals)** and **known/unknown support-mask changes**. A changed EDT source affects output positions within truncated radius `R`; mark all 32×32 output tiles intersecting the **R-expanded** source-change region, including interpolation/edge support. Rebuild each output tile from the **current complete occupancy snapshot**, with sources **outside** the tile interior up to `R` plus interpolation halo. An obstacle *removal* requires finding the next-nearest remaining source; simply retaining prior tile minima is wrong. Compare tiled values/bilinear samples against a **full exact EDT** (Felzenszwalb–Huttenlocher is a viable full-field baseline). When extent, origin, confidence threshold, config or halo validity changes, or too many tiles are dirty, fall back to a **full rebuild**.

**Distinct pyramid invalidation:** A probability change can alter OD-009 scores **without crossing the EDT occupied threshold**. Track correlation-score dirty regions **separately**, then update the affected multi-resolution maxima/ancestors, including score decreases and unknown-mask changes. To claim branch-and-bound, all recomputed pyramid bounds must remain **admissible upper bounds of that snapshot's finer score**. An EDT dirty tile list is **not** proof a correlation pyramid is current.

**Atomic snapshot identity:**

~~~text
mutable OD-010 occupancy revision N (+ known-mask and dirty events)
    -> bounded worker freezes source N into private immutable/copy-on-write state
        -> build complete compatible EDT and correlative pyramid revisions
        -> validate source hash, config hash, local origin and ready flags
        -> publish ONE immutable MapSnapshot_N atomically
                   |
      shared immutable pinned handle / version metadata
              /                         \
      OD-007 Ceres match          OD-009 coarse BnB
        (EDT ready)               (score pyramid ready)
              \                         /
               OD-008 acceptance and mapping gate
                    -> NEXT OD-010 insertion (N+1)
~~~

A snapshot contains **one submap ID, monotonic occupancy revision, source content hash, immutable occupancy/known mask, compatible EDT/pyramid revisions and readiness, interpolation/score config hashes, insertion high-watermark, and a pinned graph-pose transform/version where needed**. An operation pins its complete snapshot **for its entire lifetime**. Never mix occupancy N+1 with EDT N, publish half-computed fields, or let GTSAM globally move the snapshot's local EDT. If a worker finishes an obsolete revision, discard/requeue without overwriting the latest complete snapshot.

**Staleness and memory:** A previous **complete coherent snapshot** MAY be used while a newer one builds, only within explicit **maximum steady-clock age and maximum accepted insertion lag**, logged in OD-008 quality. If exceeded, defer/reject the match or insertion (acquisition/recording/odom continue) rather than mix versions or pretend latest geometry. In particular, the current scan must match a **pre-insertion** revision that excludes that scan. Bounded worker queues, cancellation and copy-on-write/shared immutable references keep acquisition independent of cache work. Memory accounting MUST include **pinned old snapshots, scratch fields, both active maps and frozen caches**; when references prevent eviction, report backpressure and shed lower-priority jobs, not memory-bound violations.

**Freeze and cache reuse:** At OD-010 freeze, finalize the local occupancy/known-mask revision and precompute/verifiably persist or publish its local EDT and pyramid. A bounded cache evicts **unpinned cold derived fields** under pressure; a frozen miss or wrong config hash triggers on-demand **background load or reconstruction**, returning `NOT_READY`/deferred status until a coherent bundle is available. Changes to global `T_map_submap` update separately pinned transforms or global spatial indexes but **do not rebuild an unchanged local EDT/pyramid**. OD-012 defines persistent file representation, not snapshot coherence.

**Portable interface:** `MapSnapshotProvider` returns pinned `shared_ptr<const MapSnapshot>` (or equivalent) with operation-specific cache-readiness, stale/version/quality diagnostics. Cache-builder/worker implementations remain **ROS-, Ceres-, GTSAM- and device-independent** at domain boundaries; adapters call the shared native service rather than possessing writable occupancy pointers.

**Verification:** Docker Catch2 tests must compare full and tiled exact EDT under additions, removals, tile-edge sources and known-mask changes, validate interpolation halos and distinct pyramid score invalidation; test concurrency/version pinning, staleness limits, canceled builds, memory pressure, frozen eviction and reload, graph-pose-only changes and no self-matching. **B19** benchmarks tile vs baseline full rebuild and sustained Pi 5 CPU/RAM/capture latency; 40×40 m at 5 cm has **640,000 cells**, **2.56 MB decimal (~2.44 MiB)** per float32 EDT before other caches/copies.

### 6.4 Portable APIs, artifacts and validation

ROS-independent C++20 `OccupancyGrid`, `SubmapBuilder`, `FrozenSubmap` and `MapSnapshot` interfaces expose local geometry, unknown/observed flags, log-odds, scan IDs, insertion counts, memory limits, freeze cause, source hashes and independently revisioned global pose. Mapping-domain headers must not include Ceres, GTSAM, LD19/MPU6050/Pico or ROS types; adapters perform those conversions.

**Docker Catch2/CTest:** verify exactly A #1–180/B #91–270 and at most two active maps; seam-accepted but motion-filter-skipped scans do not increment; logit hit/miss/clamping and known-neutral cells; crossing rays hit-priority and per-scan unique cell updates; zero invalid/missing clearing; ray outside extent without fake endpoint; rotated submap origins and 5/2.5/10 cm grids; early cap/allocator failure with honest freeze counts; immutable local source hashes across graph pose change; EDT/pyramid cache coherence, no map self-match, deterministic replay and Pi CPU/RAM measurements.

**Proposed native map bundle** (exact serialization is still OD-012):

~~~text
map_name/
  manifest.json               # source/version/hash/frame/resolution
  calibration.yaml            # sensor extrinsics, time/encoder parameters
  occupancy.*                 # authoritative local log-odds and observed mask
  submaps.*                   # insertion IDs, lifecycle, local origin, global poses
  distance_fields.*           # OD-007 derived/rebuildable, versioned EDT
  correlation_pyramids.*      # OD-009 derived/rebuildable, versioned
  graph.*                     # loop factors and graph-pose revisions
  trajectory.*                # corrected pose/time history
  metrics.json                # truth class and benchmark evidence
~~~

PGM/YAML export may aid ROS consumers but alone does not preserve all probabilities, known-neutral status, graph topology or provenance. Offline remapping creates a **new versioned artifact**, not mutations of frozen submaps; live known-map localization only reads published snapshots.

## 7. GTSAM graph and loop closure

Proposed variables: keyframe poses `X_k ∈ SE(2)`, yaw bias `B_k ∈ R` (possibly coarser nodes), optionally global frozen-submap poses `S_j ∈ SE(2)`.

| Factor | Data | Quality constraint |
| --- | --- | --- |
| Gauge prior | Initial pose and optional bias prior | No unjustified strong absolute certainty |
| Wheel | Encoder-derived relative motion | Check provenance: processed fused odom may be correlated |
| Gyro | Yaw preintegration conditional on bias | Drift/random-walk and proper time covariance |
| LiDAR | Geometric `T_submap_base` observation from Ceres | Pinned submap, conservative measurement covariance |
| Loop closure | Verified relative submap/pose transform | Overlap, ambiguity, robust loss and consistency gate |
| Bias evolution | Temporal bias random walk | Calibrated noise; no hard constant-bias assumption |

For a scan-to-submap measurement `Z_jk` of body pose in submap j, the predicted pose is `inverse(S_j) * X_k`. A proposed error is `Log(inverse(Z_jk) * inverse(S_j) * X_k)`, with tested tangent-space and noise conventions. Do not create another pose prior from the same match.

For gyro bias reference `b_ref`:

~~~text
delta_yaw_ij = sum((omega_z - b_ref) * dt)
r_gyro = wrap(yaw_j - yaw_i - delta_yaw_ij
              + (b_i - b_ref) * (t_j - t_i))
~~~

**Offline:** create graph, verify closures, batch optimize, re-deskew/rebuild when corrections justify it, and export new map revision. **Online:** iSAM2 updates asynchronously with bounded work and revisioned snapshots. **Known-map localization:** global graph is not required; evaluate a simpler filter before adding fixed-lag smoothing.

Loop pipeline: descriptor/candidate retrieval -> coarse match -> Ceres refine -> overlap and degeneracy gates -> robust loop factor -> consistency monitoring and rejection. Graph parameterization/evidence **OD-013**, offline iteration policy **OD-014**, keyframe/bias spacing **OD-015**, loop search **OD-018**, online budget/fixed lag **OD-019**.

## 8. Known-map localization and recovery

Local tracking cycle:

1. Predict continuous `T_odom_base(t)` and covariance from wheel/gyro.
2. Assemble the scan and deskew every valid beam into its **temporal midpoint** frame through the portable, timestamp-addressable trajectory provider; require full interpolation for map insertion.
3. In TRACKING, match against nearby **frozen** map tiles using OD-007 Ceres directly. When confidence degrades, OD-009 supplies bounded local hypotheses; in recovery/global, async multi-resolution BnB supplies candidates. Every refined candidate still requires OD-008 quality and observability acceptance.
4. If accepted, produce a global pose at **scan midpoint time** and update `T_map_odom` using midpoint-aligned continuous local odometry.
5. Keep local odom publishing even if global correction is late; mark global pose freshness explicitly.

Tracking states:

| State | Meaning | Behavior |
| --- | --- | --- |
| `UNINITIALIZED` | No valid map/global pose | Do not publish a valid global fix |
| `TRACKING` | Fresh, observable consistent matches | Publish global correction and quality |
| `UNCERTAIN` | Degenerate environment, low overlap, lag or mismatch | Conservative uncertainty, widened bounded search |
| `LOST` | Repeated mismatch / invalid calibration or clocks | No valid global fix; start global recovery |
| `RELOCALIZING` | Alternative candidates undergoing validation | Require cross-scan consistency before adopting pose |

**Accepted OD-009** uses asynchronous hierarchical multi-resolution branch-and-bound and multiple candidate refinement for initial global recovery. A particle filter is **not initially required**; **OD-020** remains open on whether persistent multimodal hypotheses are needed after observing repeated-aisle/kidnapped-robot failures. Transition hysteresis and cross-scan publication semantics remain **OD-022**. Never mutate the frozen map or accept one high coarse correlation as a global fix.

## 9. Concurrency, queue discipline, and lifecycle

Proposed bounded execution path:

~~~text
acquisition threads ---+--> raw record queue --> MCAP writer
                       |
                       +--> time reorder -> trajectory -> scan matching
                                                          |
                                               accepted pose / scan
                                                          |
                                         map builder / graph async queue
                                                          |
                                        immutable optimizer/map revisions
                                                          |
                                         pose / ROS / Web Lab / report
~~~

- Acquisition threads never call optimizers, HTTP handlers or disk flush synchronously.
- Keep distinct recording and live-estimation queue overflow policies. Recording loss must be surfaced; live estimator may skip stale scans while marking gaps.
- Graph and loop closure work cannot block scan matching; snapshot writers publish whole coherent revisions.
- Stop flushes storage safely and reports incomplete/truncated recordings.
- Calibration hot changes require explicit reset or a transactional version handoff; scan processing cannot mix extrinsics mid-scan.
- Lifecycle: `CREATED -> CONFIGURED -> CALIBRATED -> RUNNING -> DEGRADED/STOPPING -> STOPPED/FAILED`.

Thread/process split **OD-016**, queue limits/drop and overload policy **OD-021**. UI and ROS are consumers, never timing authorities. Replay seek reinitializes time-dependent estimator state.

## 10. Persistence, reproducibility and interfaces

**Raw MCAP** should preserve LD19 bytes, raw IMU, real wheel measurements (or explicitly labeled derived wheel odom), sensor/host clocks and calibration. Native converters handle ROS 1/2 bags without making the core ROS-dependent.

- Each run gets dataset ID/hash, calibration revision, config digest, map revision, software commit, Docker image and event-time provenance.
- Derived scans and optimized trajectory never replace raw source measurements.
- Replay supports pause, step per completed scan, deterministic seek/reset and independent dataset evaluation.
- External benchmark downloads are opt-in and verified; no downloads during normal CMake/test setup.
- Web Lab uses local bounded HTTP/JSON, Canvas layers, immutable snapshots, scan/time playback and exported measurements from the *real* C++ pipeline.
- ROS 2 adapter maps native scan/IMU/odometry/map/pose to standard topics and owns TF/clock conversions; only one publisher per TF edge.
- No browser, ROS or Node/JS runtime is required for the headless core.

MCAP channel/schema **OD-023**, seek/checkpoint format **OD-024**, ROS distro/QoS defaults **OD-025**. **OD-001 signed-ns time, OD-002 Pi-first/Pico-future acquisition, and OD-003 LD19 beam timing/clock alignment are accepted**: use signed 64-bit nanoseconds with explicit clock-domain conversion and ROS-neutral simulated-clock events.

## 11. Resource budget and failure policy

Engineering targets to **measure**, not achieved numbers:

| Activity | Nominal rate / proposed target |
| --- | --- |
| Gyro / wheel ingest | ~200 Hz / ~100–200 Hz; acquisition cannot block |
| Continuous local odometry | >=100 Hz if measurements support |
| LD19 revolution | ~10 Hz, ~450 returns each |
| Full deskew | Aim <2 ms/scan on Raspberry Pi 5 |
| Ceres local scan match | Aim <20 ms typical on Pi 5; iteration deadline |
| Map insertion | Aim <5 ms/accepted scan |
| Online graph and loop closure | Async; can defer work but must report lag |
| Localizer | Target scan correction ~10 Hz with explicit tracking health |

Report p50/p95/p99 capture-to-pose latency, scan match and deskew time, queue backlog, missed deadlines, CPU/RAM and thermal throttling. On overload prioritize **sensor acquisition, continuous odometry and recording integrity**; defer graph updates, throttle global searches and UI, and explicitly degrade global localization state. Never invent a hard real-time guarantee before Pi measurements.

## 12. Testing contracts and milestones

All tests run **inside Docker**, with **Catch2 v3**, CTest, C++20 and cxxopts application CLIs; no GitHub Actions execution unless explicitly requested.

| Boundary | Required tests |
| --- | --- |
| Device/time | LD19 CRC/47-byte parsing, 30 s wraps, angle/speed per-beam timing, LD19↔Pi clock fit, injected jitter/offset/drift and physical delay bounds; signed-ns epochs and ROS /clock; Pi/Pico acquisition faults |
| Geometry | SE(2) composition, REP frames, rotated occupancy origin, Jacobians and extrinsics |
| Motion/deskew | OD-006 minimal wheel/gyro integrator, bias calibration and bounded correction, supported/unconfigured drivetrain, wheel/gyro slip with covariance growth, pose–bias uncertainty, OD-004 SE(2) arcs/midpoint/10 ms gate |
| Ceres and match quality | OD-007 EDT/bilinear/per-beam Huber and prior/boundary; **OD-008** robust LiDAR-only Hessian/rank, scale normalization, covariance coverage and observable-subspace rejection/projection; no double-counted GTSAM factors |
| Mapping/graph | No self-matching, immutable submaps, wheel/gyro factor independence, bias factor Jacobians, loop rejection |
| Localization/search | Map->odom correctness, OD-009 zero coarse work in TRACKING, local/recovery/global windows and slice budgets, BnB upper-bound correctness, multiple hypotheses, OD-008 gates, stale/lost and kidnapped recovery, immutable map integrity |
| IO/replay | MCAP and map schema roundtrip, version rejection, deterministic replay, interrupted writes |
| Adapters | ROS-free core build; optional ROS 2 frames/scan/maps/QoS; Web Lab real backend responses |
| Performance | Equivalent datasets on ARM64 and x86-64, capture-to-pose tail latency and resource use |

Implementation priority: **P0** toolchain/geometry/synthetic fixtures -> **P1** Pi-direct recorder/time sync (optional Pico firmware after measured need) -> **P2** motion/deskew -> **P3** Ceres/submaps -> **P4** GTSAM offline mapping -> **P5** frozen-map localization/recovery -> **P6** online SLAM. Web Lab can start as a synthetic replay viewer and expand without blocking estimation development.

## 13. Decision links and references

The companion [open design choices register](open-design-choices.md) contains **OD-001–OD-025** with priority, alternatives, evidence and decision gates. Proposed defaults here are **not approved decisions** until marked Accepted there.

External technical references:

- [ROS REP-103](https://reps.openrobotics.org/rep-0103/) · [ROS REP-105](https://reps.openrobotics.org/rep-0105/)
- [Ceres nonlinear least-squares residuals and robust losses](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html)
- [GTSAM iSAM2](https://borglab.github.io/gtsam/isam2/) · [MCAP](https://mcap.dev/)
