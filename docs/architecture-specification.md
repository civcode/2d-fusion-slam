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

### 2.1 Accepted OD-017 — Initial host CMake targets and ownership

**Build unit vs conceptual module:** The geometry diagram above and the subsequent conceptual modules (`sensors`, `motion`, `deskew`, `match_quality`, `offline_refinement`, `coarse_search`, etc.) are **responsibilities inside the eight accepted CMake libraries**, not a mandate to create a distinct library for each row. The initial host project creates exactly **eight libraries + three daemon executables**. Four libraries are **exported/installable SDK**; four are **private host implementation**.

| CMake library | Public SDK? | Architectural ownership |
| --- | --- | --- |
| `slam_core` | **Yes: `slam::core`** | OD-001 typed time/clock epochs, SE(2), units, sensors, trajectory/uncertainty/provenance and domain interfaces |
| `slam_mapping` | **Yes: `slam::mapping`** | OD-010 authoritative occupancy/known mask, local submaps, OD-011 immutable map snapshots and portable maps |
| `slam_engine` | **Yes: `slam::engine`** | High-level mapping/localization session/config/status API, private frontend/matching/graph orchestration |
| `slam_storage` | **Yes: `slam::storage`** | OD-012 versioned map bundle, recording and deterministic replay APIs (OD-023/024) |
| `slam_frontend` | **No** | OD-004/005/006 trajectory/scan/deskew, OD-015 keyframes, OD-018 spatial/signature retrieval, OD-009 BnB/quality and OD-014 offline orchestration |
| `slam_match_ceres` | **No** | OD-007 bilinear EDT Ceres optimizer and private OD-008 Jacobian extraction, native quality output |
| `slam_graph_gtsam` | **No** | OD-013 offline keyframe/submap batch, OD-018 verified loops and OD-019 sparse global submap iSAM2; conditional local fixed-lag |
| `slam_platform_linux` | **No** | Linux sensor/Pico acquisition adapters, OD-002/003 clock synchronization, OD-016 `SOCK_SEQPACKET` IPC, runtime process/scheduler helpers |

**The three native host executable targets** are `slam_acquisitiond` (platform/core), `slam_recorderd` (storage/platform), and `slam_localizerd` (engine/mapping/platform with private backend composition). They are thin `cxxopts` CLIs with independent startup/shutdown, versioned IPC and status reporting. **Web Lab remains the independent fourth OD-016 runtime process but is built separately**; Pico firmware is a separate RP2040/Pico SDK cross-compile. No default fourth/fifth host executable, forced Pico SDK, ROS 2, JavaScript/Web Lab build or external hardware requirement.

### 2.2 Public API and dependency DAG

~~~text
installed public include/slam/
  core/{time,geometry,sensor_data,trajectory}.hpp  + core/interfaces/
  mapping/{occupancy_grid,submap,map_snapshot}.hpp
  engine/{slam_engine,configuration,diagnostics}.hpp
  storage/{map_bundle,recording}.hpp

portable SDK dependency direction:
 slam_core  <--- slam_mapping
      ^               ^
      +--- slam_storage
      +---------------+--- slam_engine (public native facade)
                                 |
                          PRIVATE implementation:
             slam_frontend + slam_match_ceres + slam_graph_gtsam
                   (native interfaces, no cyclic links)

host executables ---> slam_platform_linux (outer-only native Linux adapters)
optional ROS 2 ----> public SDK (never the reverse)
~~~

**Hard architectural rules:**
- `slam_core` has **no** Ceres, GTSAM, ROS2, Linux/Unix socket, Pico SDK, device driver or IPC types/link dependency. `slam_mapping` depends only on core among host libraries. `slam_storage` depends only on portable core/mapping types and keeps MCAP/Zstd implementation private. `slam_engine`'s installed API is **only portable native types**; implementations may use private Ceres/GTSAM/frontend backends, not their classes in public headers.
- `slam_platform_linux` is the **outer adapter** linked at executable/deployment composition, never into SDK domain or solver/frontend algorithm libraries. Sensor/clock/IPC conversions terminate at the portable timestamped sensor interface; the ROS2 adapter separately consumes public SDK. Cross-module factor graph and cache models are portable; no Ceres/GTSAM types leak through the engine session contract.
- Avoid implementation cycles by separating interfaces from concrete backends and providing private factories/pImpl. Frontend composition must not introduce frontend↔Ceres↔GTSAM circular linkage. The package's four installed targets are `slam::core`, `slam::mapping`, `slam::engine`, `slam::storage`, not the internal targets.
- An installed **static SDK façade** that privately links internal static implementation libraries can export those dependencies with CMake `$<LINK_ONLY:...>` and break clean consumers. Use an appropriate shared-library/pImpl or equivalent audited packaging boundary for `slam_engine` and install private runtime dependencies separately; `PRIVATE` is not sufficient proof of an independent exported link interface. The package can avoid Ceres/GTSAM at **consumer configure time and installed public-header compilation** even though executable/runtime libraries may dynamically load private solver components. Inspect both exported CMake and runtime linker closure.
- Install only `include/slam/{core,mapping,engine,storage}` headers, `FusionSLAMConfig.cmake`/`FusionSLAMConfigVersion.cmake` and `FusionSLAMTargets.cmake` with a documented version/ABI policy. Use `GNUInstallDirs`, `install(TARGETS ... EXPORT ...)`, `EXPORT_NAME`, target-based `cxx_std_20`/`PUBLIC`/`PRIVATE`/`INTERFACE`, and `CMakePackageConfigHelpers`. Correctly declare any actually public Eigen dependency. Never add global compile flags/includes or unapproved Boost.
- Independent clean-prefix **`find_package(FusionSLAM CONFIG REQUIRED)` installed-consumer** integration must compile, link and run with all four installed imports and no private target/headers/implicit ROS/Ceres/GTSAM/Linux socket imports. `BUILD_TESTING=OFF` must not fetch Catch2; native CLI uses cxxopts only. Normal portable SDK build needs no hardware/Web Lab/Pico/ROS.

**B25 acceptance:** Inspect all eleven host targets, exported imported target names, compile/header and linker dependency DAG, consumer install-tree relocation, C++20/ABI version metadata, no leaked solver/IPC headers, static `LINK_ONLY` pitfalls, normal Docker x86/Pi ARM64 builds, three daemon `--help`/error paths and OD-016 three-daemon-plus-separately-built-Web-Lab four-process smoke test. Test Ceres/GTSAM individually in private implementation, Catch2 v3/CTest, and independent optional integrations. No real build/test execution is asserted yet.

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

The core MUST accept an **injected, ROS-neutral time source** or equivalent typed `ClockReading` / `ClockEvent` interface, with an initialized/ready flag, active clock domain, epoch and nanosecond value. The optional `slam2d_ros` adapter owns subscription to `rosgraph_msgs/msg/Clock` on `/clock`, `use_sim_time` configuration, conversion to/from `builtin_interfaces/msg/Time`, and ROS time-jump notifications. No ROS headers or clock types enter the core.

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
- **Forward jump:** distinguish ordinary accelerated simulation from a true discontinuity using a documented event/gap policy; never integrate across an unsupported long gap. Exact clock jump thresholds remain calibration work under OD-002; accepted OD-024 selects indexed semantic checkpoint seeking (B32 pending).
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

**Related:** OD-001 (accepted nanosecond time/domain/epoch), OD-002 (accepted acquisition staging), **OD-003 (accepted angle/speed timing and clock alignment, physical delay not yet calibrated)**, accepted OD-021 (initial bounded buffering; B29 Pi validation pending), accepted OD-023 (native Protobuf MCAP channel/schema, B31 pending). [Pico timer](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_timer/include/hardware/timer.h) and [PIO API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) document primitives, not achieved accuracy.

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

**Spline future extension:** The trajectory interface allows a continuous SE(2) spline alternative later, but the accepted first implementation is piecewise-constant twist and `Exp_SE2`; do not add spline dependencies or complexity speculatively. **Accepted OD-014** permits up to three offline, observability-gated selective re-deskew/remap passes after the initial full replay, each published as an immutable validated iteration.

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

**Crucial OD-006/accepted OD-013 compatibility:** The regularized optimized pose combines **LiDAR + a wheel/gyro odometry prior**. It is a **posterior pose**, not a statistically independent LiDAR observation. If GTSAM separately receives wheel and gyro factors from those same measurements, it MUST NOT receive that posterior pose as an independent LiDAR-only factor. Instead preserve separate **LiDAR-only geometric measurement/diagnostics**, prior provenance/covariance and the final regularized pose; **accepted OD-008** specifies LiDAR-only rank/uncertainty assessment; **accepted OD-013** requires separate geometry-based factor extraction, source-overlap/correlation admission and calibrated conservatism. Do not assert an exact independent LiDAR covariance merely by subtracting prior information from a posterior Hessian; deskew/map conditioning is correlated.

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
- Define separate **full covariance**, **observable subspace only**, and **unavailable** uncertainty states. The ROS adapter must not fill missing/degenerate directions with falsely small finite covariances; suppress or clearly degrade outputs when native uncertainty cannot be faithfully converted. **Accepted OD-013** rejects unsupported rank-deficient full-pose factors and requires provenance-aware, correlation-conservative factor admission.

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

**Freeze and cache reuse:** At OD-010 freeze, finalize the local occupancy/known-mask revision and precompute/verifiably persist or publish its local EDT and pyramid. A bounded cache evicts **unpinned cold derived fields** under pressure; a frozen miss or wrong config hash triggers on-demand **background load or reconstruction**, returning `NOT_READY`/deferred status until a coherent bundle is available. Changes to global `T_map_submap` update separately pinned transforms or global spatial indexes but **do not rebuild an unchanged local EDT/pyramid**. **Accepted OD-012** specifies persistent JSON manifest + independent zstd payload generations, while **OD-011** alone governs in-memory snapshot coherence.

**Portable interface:** `MapSnapshotProvider` returns pinned `shared_ptr<const MapSnapshot>` (or equivalent) with operation-specific cache-readiness, stale/version/quality diagnostics. Cache-builder/worker implementations remain **ROS-, Ceres-, GTSAM- and device-independent** at domain boundaries; adapters call the shared native service rather than possessing writable occupancy pointers.

**Verification:** Docker Catch2 tests must compare full and tiled exact EDT under additions, removals, tile-edge sources and known-mask changes, validate interpolation halos and distinct pyramid score invalidation; test concurrency/version pinning, staleness limits, canceled builds, memory pressure, frozen eviction and reload, graph-pose-only changes and no self-matching. **B19** benchmarks tile vs baseline full rebuild and sustained Pi 5 CPU/RAM/capture latency; 40×40 m at 5 cm has **640,000 cells**, **2.56 MB decimal (~2.44 MiB)** per float32 EDT before other caches/copies.

### 6.4 Portable APIs, artifacts and validation

ROS-independent C++20 `OccupancyGrid`, `SubmapBuilder`, `FrozenSubmap` and `MapSnapshot` interfaces expose local geometry, unknown/observed flags, log-odds, scan IDs, insertion counts, memory limits, freeze cause, source hashes and independently revisioned global pose. Mapping-domain headers must not include Ceres, GTSAM, LD19/MPU6050/Pico or ROS types; adapters perform those conversions.

**Docker Catch2/CTest:** verify exactly A #1–180/B #91–270 and at most two active maps; seam-accepted but motion-filter-skipped scans do not increment; logit hit/miss/clamping and known-neutral cells; crossing rays hit-priority and per-scan unique cell updates; zero invalid/missing clearing; ray outside extent without fake endpoint; rotated submap origins and 5/2.5/10 cm grids; early cap/allocator failure with honest freeze counts; immutable local source hashes across graph pose change; EDT/pyramid cache coherence, no map self-match, deterministic replay and Pi CPU/RAM measurements.

**Accepted OD-012 native bundle:** Use a JSON manifest and individually compressed portable binary occupancy/graph payloads, with optional derived EDT and correlation caches. The complete layout, atomic generation-pointer save/recovery policy, native clock/session fields and map-versus-checkpoint distinction are specified in **Section 10.1**. The OD-011 cache-service contract remains independent of physical serialization.

PGM/YAML export may aid ROS consumers but alone does not preserve all probabilities, known-neutral status, graph topology or provenance. Offline remapping creates a **new versioned artifact**, not mutations of frozen submaps; live known-map localization only reads published snapshots.

## 7. Accepted OD-013: GTSAM keyframe + submap Pose2 graph and evidence accounting

### 7.1 Variables, gauge, and factor topology

The **native portable graph model** has global keyframe `X_k = T_map_base(t_k) ∈ SE(2)` and global submap `S_j = T_map_submap_j ∈ SE(2)` variables. Each submap refers to one immutable OD-010 local log-odds grid with stable coordinate frame/ID. A keyframe timestamp is OD-004's **scan midpoint**; **accepted OD-015** selects motion keyframes (0.20 m, 5°, 2 s moving, 0.2 s minimum). This is a **pose-only graph initially**: OD-006 maintains wheel/gyro integration, gyro bias, covariance and continuous deskew trajectories **outside GTSAM**. Do **not** introduce duplicate GTSAM `B_k` bias states, raw wheel+yaw factors or a fixed-lag joint smoother before benchmarks justify them.

| Native constraint | Initial admission rule | GTSAM backend shape |
| --- | --- | --- |
| **Gauge/reference** | One documented map-frame gauge, **not** a spurious absolute sensor observation | Anchored frame or one appropriate prior |
| **Fused interval odometry** | **Exactly one** wheel/gyro-derived relative motion per keyframe interval, preserving raw sample provenance and uncertainty | `BetweenFactor<Pose2>` between `X_k` and `X_{k+1}` or a rank-aware equivalent if incomplete |
| **Primary local LiDAR** | At most **one per keyframe/source encounter**, independently evaluated geometry, OD-008 rank/uncertainty | Keyframe-to-submap relative `Pose2` factor (`X_k,S_j`) |
| **Inter-submap/keyframe loop** | Frozen target excludes query scan; geometry validated with OD-007/008 and duplicate/correlation guard | Robust relative pose factor between relevant keyframe/submap variables |
| **Submap anchor/membership** | Local frame definition, first accepted scan/anchor, both overlapping memberships | **Bookkeeping/initialization, not additional independent likelihood** |

For admitted LiDAR relative pose `Z_jk=T_submap_j_base(t_k)`:

~~~text
predicted Z_jk = inverse(S_j) * X_k
e_jk = Log( inverse(Z_jk) * inverse(S_j) * X_k )
e_odom = Log( inverse(Z_odom(k,k+1)) * inverse(X_k) * X_(k+1) )
~~~

Apply an explicitly tested SE(2) tangent convention, covariance frame transformation, source timestamp and Jacobian. **Fix the gauge exactly once.** Initialize each new `S_j` from its OD-010 anchor-frame relationship; do **not** add a high-confidence "anchor measurement" derived solely from the same odometry/local scan as an extra factor. An exact frame-definition relationship, if represented, must not redundantly constrain independent variables or prevent valid graph deformation. Test gauge rank/optimization freedom. Optimize `X_k` and `S_j` jointly; preserve immutable local occupancy while global `S_j` changes.

### 7.1.1 Accepted OD-015 — Motion-adaptive keyframes and optional sparse bias knots

**Independent state lifecycles:** OD-015 creates **keyframe SE(2) poses `X_k`** from timestamped OD-004 scan midpoint motion, while **OD-010 submap `S_j` creation/freeze continues at 90/180 accepted scan insertions**, not keyframe counts. Each submap has its own optimizable pose and an associated anchor keyframe, without a duplicated anchor measurement.

| Keyframe selector setting | Initial value | Meaning |
| --- | --- | --- |
| Translation | **0.20 m** | Relative motion from last keyframe |
| Rotation | **5° (pi/36 radians)** | Wrapped shortest yaw |
| Elapsed time | **2.0 s while meaningfully moving** | No stationary periodic keyframes |
| Minimum keyframe interval | **0.2 s** | Regular admission floor; tagged anchor/root exceptions |
| Prediction uncertainty | Bounded secondary request | Depends on available geometric information and node budget |
| Initial graph bias variables | **None** | OD-006 maintains local evolving gyro bias |
| Optional future offline knots | **5–10 s** | Scalar yaw bias, random-walk and observable only |

**Selection algorithm:** A native `KeyframeSelector` takes OD-005 valid scan IDs, OD-004 midpoint canonical nanosecond timestamps, OD-006 pose/twist/covariance and OD-008 match diagnostics; compare `T_last_keyframe^-1*T_candidate` translation magnitude and yaw (with correct wrap). Request a node on the **first** passed translation, yaw, meaningful-moving-time or evidence-backed uncertainty trigger; apply the 0.2 s minimum **after** the trigger. Log `DEFERRED_MIN_INTERVAL` and reconsider later scans rather than silently violating the minimum. Stationary periods do not create keyframes every two seconds; bias calibration may continue on raw IMU without graph nodes. Respect OD-001 domain/epoch resets, bad timestamp coverage and bounded node budgets.

**Quality versus node creation:** High uncertainty only triggers earlier sampling if useful geometry is likely; high uncertainty **with low OD-008 observability** must cause OD-009 recovery or OD-022 degraded tracking, not unbounded unconstrained keyframes. A keyframe required for odom continuity or submap anchor may have **no LiDAR factor** if OD-008 quality rejects the scan. Use **one fused wheel/gyro OD-006 relative factor per actual interval between adjacent admitted keyframes**, preserving original sample/clock/bias IDs even when nodes are deferred. At most one independently validated OD-013 local LiDAR constraint per encounter; no duplicate from two overlapping submap memberships.

**Anchor creation:** After OD-010's 90th accepted insertion creates a new submap, associate an existing suitable timestamped keyframe or explicitly create `SUBMAP_ANCHOR`, recording a justified 0.2 s exception. Split/recompute adjacent fused motion intervals to avoid reusing the same raw samples as independent factors. The anchor initializes `S_j` and records its local frame relationship; it is **not another high-confidence sensor factor** and must not pin `S_j` against loop-closure deformation. The OD-010 90/180 counters are **unchanged** regardless of graph node count.

**Optional OD-014 offline joint refinement, never initial Pose2 graph:** Keep one evolving `b_g(t)` in OD-006 initially. After independent bias-drift/temperature/observability validation, an offline version may introduce sparse scalar knots `b_j` every **5–10 s** (same OD-001 acquisition epoch) with:

~~~text
b_(j+1) = b_j + w_j
w_j ~ N(0, q_b * Delta_t_j)
Var(w_j) = q_b * Delta_t_j
// q_b: (rad/s)^2 per second; Delta_t_j in seconds
~~~

A random-walk factor alone does not couple bias to pose. The optional model must **replace** corresponding baseline fused odom factors with tested **planar wheel+gyro bias-aware relative-motion factors** that explicitly couple pose/yaw to interpolated knots and preserve correlations, instead of double-counting gyro/wheel measurements. GTSAM's 3D IMU preintegration is not directly interchangeable with this planar wheel/gyro model. Keep unobservable bias fixed/appropriately constrained (wheel slip is not bias evidence), selectively re-deskew/rebuild changed submaps via OD-014 and preserve raw intervals, calibration and factor IDs in OD-012. Enable knots only if held-out accuracy and uncertainty consistency improve.

**Portable boundary/test:** Native C++20 `KeyframeSelector`, `KeyframeDecision`, `KeyframePolicy`, `SubmapAnchorAssociation` and `BiasKnotPlan` expose only domain SE(2), time, source IDs and uncertainties; no ROS, Ceres, GTSAM or device types. Docker B23 checks 0.20 m/5°/2 s/0.2 s edges, no stationary flooding, OD-008 null-rank factor abstention, distinct anchor/pose variables, unchanged 90/180 counts, source interval nonduplication, optional 5/10 s random-walk variance/Jacobians and OD-014 replay.

### 7.2 Correlated evidence and admitted factors

**Single interval motion factor:** OD-006 fused encoder translation with bias-corrected gyro yaw becomes **one** relative SE(2) motion likelihood for each keyframe interval, with source encoder/gyro sample/time ranges, covariance/bias calibration and slip/timing quality. Do not simultaneously add independent wheel-only, gyro-only **and** already-fused odometry factors from those same samples. If a modality is absent, admit only constraints supported by that evidence; do not invent fully observed 3-DoF motion or small covariance.

**LiDAR conditioned on odometry:** OD-004 deskews each beam using OD-006 motion. That makes geometric residual errors share motion noise with the odometry factor. The joint uncertainty has off-diagonal cross-covariance:

~~~text
Sigma_joint = [ Sigma_odom    Sigma_cross   ]
              [ Sigma_cross^T Sigma_lidar   ]
~~~

Independent GTSAM noise models on two factors **do not encode `Sigma_cross`**. Preserve the exact motion sample intervals/bias revisions used to deskew each scan, and apply conservatively inflated **held-out-calibrated OD-008** LiDAR geometry uncertainty to reduce overconfidence. **Inflation is an approximation, not a proof of statistical independence**; if calibration cannot support it, reject/degrade and defer explicit joint correlated factors to a separately validated future implementation.

**Ceres posterior is not LiDAR measurement:** OD-007 optimizes a geometric EDT cost **plus OD-006 odometry prior and boundary penalty**. The resulting pose and full Hessian therefore contain motion evidence and nonmeasurement constraints. Do **not** add the posterior as independent LiDAR `Z_jk` or assume subtracting prior Hessian yields independent measurement covariance. For graph admission, perform a separate **LiDAR-only geometric refinement in OD-008 observable directions around a defensible candidate**, or use a separately justified explicitly conditioned geometry measurement. Carry geometry-only source Jacobians, rank, covariance calibration, map version and prior provenance; if reliable extraction fails **skip the LiDAR factor** rather than double-counting.

**OD-010 overlap and self-evidence:** Preserve the full scan-to-submap **insertion membership ledger**; scan IDs **91–180** belong to both A and B in a nominal lifecycle. Select **one primary local** keyframe-to-submap geometry factor for a given encounter, not two independent copies from overlapping A/B. OD-011 **pre-insertion** snapshots permit local tracking against geometry that does not yet include the current scan. A **loop** MUST NOT target any submap containing the query scan. A repeated match of the same scan to the same target map/source geometry is not new information. Multiple loops involving heavily overlapping maps or scan sets require source-intersection analysis, correlation group IDs, configurable per-encounter caps and conservative uncertainty/rejection. A shared submap-pose variable models *submap pose* dependence; it does **not** account for correlated occupancy raster errors from shared historical scans.

**Rank-aware factors:** OD-008's rank 0–2 result has **unbounded unknown nullspace**, not a complete 3×3 covariance. Reject such measurements by default until a properly tested **observable-subspace factor** is implemented, then constrain only supported directions; never set nullspace variance to zero, invert a pseudo-covariance as full rank, or let a strong motion prior manufacture LiDAR rank. Full-rank geometry uses OD-008 calibrated finite SPD noise with provenance. Loop robustification (e.g. GTSAM robust M-estimation) follows **geometric and correlation validation**, never substitutes for verification.

| Evidence source | Graph treatment | Audit proof |
| --- | --- | --- |
| OD-006 wheel/gyro | One fused interval constraint | Unique source interval, raw encoder/gyro IDs and bias revision |
| OD-007 Ceres prior | **No second graph observation** | Prior/source IDs retained, posterior identified as conditioned |
| OD-008 LiDAR geometry | Separately extracted, rank-checked and conservatively calibrated | Geometry-only score/information, rank, map revision |
| Shared A/B scan insertion | Membership but **one primary local LiDAR factor** | Membership + encounter dedup IDs |
| Loop candidate/target | Nonself, OD-007/008 verified; redundant overlaps capped | Query ID absent from target, source overlap/correlation group |
| Repeated frozen-map matching | Shared pose key and conservative geometry uncertainty | Snapshot/source hash, graph pose revision and factor provenance |

### 7.3 Native factor admission and optimization pipeline

~~~text
OD-006 fused wheel+gyro interval motion ───────┐
OD-004/005 LiDAR -> OD-007 Ceres -> OD-008 ──┐ |
OD-010 overlap membership -> OD-011 snapshots ├─> native GraphFactorAdmission
OD-018 spatial/signature candidates -> OD-009 BnB ------┘ |      |
                                                v      v
                                    native X_k, S_j, factors
                                    (IDs, sources, correlation groups)
                                                |
                                         private GTSAM batch solver
                                                |
                                versioned graph poses (X_k,S_j)
                             /                              \
                map->odom global correction       OD-012 graph bundle
                odom->base continuous             frozen local grid unchanged
~~~

The **native `PoseGraphModel` / `GraphFactorSpec` / `EvidenceProvenance` / `GraphFactorAdmission` / `GraphOptimizationResult`** types (illustrative) must use native SE(2), stable typed IDs, calibrated uncertainties or observable subspaces, scan membership, OD-011 map snapshot/source hash, raw motion sample interval, uncertainty calibration revision, correlation/dedup groups and accept/reject diagnostics. **No `gtsam::`/`ceres::`/ROS/vendor types in domain headers**. A private backend translates portable records to GTSAM `Pose2`/factor graph, and OD-012 serializes portable constraints—not native GTSAM objects. Keep backend construction out of acquisition and sensor adapters.

**Batch GTSAM first:** Validate graph error/linearizations, gauge, analytic versus numerical factor Jacobians, synthetic exact truth, robust loop rejection, and covariance consistency on independent held-out trajectories. Batch offline optimization runs on bounded asynchronous work outside capture/recording and local OD-006 prediction. **Accepted OD-019** adds an initial sparse global **submap-only** iSAM2 graph for online use; a second GTSAM local fixed-lag smoother remains conditional on measured benefit and preserves OD-013 original provenance. OD-014 offline re-deskew/remapping produces a **new versioned authoritative map**, not silent edits of frozen raster cells.

**Publication:** Publish immutable graph-pose revisions after numerical sanity/loop validation. Changing `S_j` changes global map/submap placement and `T_map_odom`, **not** a frozen OD-010 occupancy grid or its OD-011 local EDT/correlation cache. `T_odom_base(t)` remains continuous; global map correction may move and must carry time/freshness/quality (OD-022). Never let optimization block sensor ingestion or raw MCAP recording.

**Mandatory tests / B21:** Pose2 graph gauge and Jacobians, fused one-per-interval factor uniqueness, different submap anchors/gauge choices, overlap A/B 90 scan IDs and one primary local constraint, self-match and duplicate loop rejection, Ceres strong-prior posterior not reused as graph LiDAR, OD-008 rank-deficient admission, aggressive rotational deskew error correlation and held-out covariance inflation, wrong/repeated loops, graph pose revisions not touching local frozen hashes, Ceres/GTSAM/ROS-free domain compilation, and Pi CPU/RAM/backlog measurements. See [accepted OD-013](open-design-choices.md) and [B21](benchmarks-and-datasets.md).

### 7.3.1 Accepted OD-018 — Frozen-submap retrieval and verified loop closure

**Stage I (first implementation):** Use covariance-aware **spatial proximity retrieval against frozen OD-010 submaps**, OD-009 admissible multi-resolution **branch-and-bound (BnB)**, OD-007 **Ceres refinement**, and OD-008/013 factor acceptance. **Stage II:** Add **rotation-tolerant planar occupancy/known-mask signatures** for global retrieval among frozen submaps, once local loop closure is demonstrably reliable. **Individual keyframe scan descriptors are deferred.** This remains ROS independent and runs as a lower-priority, cancellable, bounded worker inside OD-016's SLAM process using OD-017's existing internal libraries, not a ninth CMake library or separate service.

~~~text
query scan/keyframe OR newly frozen submap
            |                         |
    pose + covariance              Stage II: rotation-tolerant
    spatial search                 2D frozen occupancy signatures
            \                         /
             merge, exclude near/overlapping/self,
              deduplicate top 3–5 distinct candidates
                             |
             pin OD-011 coherent frozen map,
                EDT and score pyramid revisions
                             |
              OD-009 admissible multires BnB
             (bounded / incomplete / top hypotheses)
                             |
                 OD-007 Ceres refinement
                  same immutable map source
                             |
             OD-008 LiDAR-only residual/rank/
                calibrated covariance validation
                             |
               OD-013 source/correlation/dedup
                 + drift-aware graph consistency
                      /             \
               accept factor       reject with diagnostics
                 into GTSAM         and no graph mutation
~~~

**Local spatial retrieval:** Query a versioned index of frozen submap bounds/poses using predicted query location and **covariance-aware search windows** bounded by OD-009 radius/angle and work limits. Enforce configurable time/trajectory separation to exclude trivial neighbors, refuse targets **containing the query scan**, and suppress adjacent overlapping submaps, repeated encounters and shared source-scan evidence. A graph-pose correction may require rebuilding the **global spatial index**, not the unchanged frozen local raster/EDT. Do not hard-reject a true loop solely because accumulated odometry drift makes the current global pose innovation large.

**Stage II descriptors:** For each immutable frozen local occupancy grid, build a **2D** rotation-tolerant polar/geometry signature with explicit **observed/unknown** support; compare circular-angle shifts and record coverage and orientation hypotheses. A 3D height-derived Scan Context implementation is not a valid unchanged LD19 descriptor. Index only known frozen revisions; key descriptors by OD-011 local source/hash and configuration, so missing/corrupt/stale entries rebuild asynchronously. Updates to only global submap pose do **not** invalidate local descriptors. Compare newly frozen submaps with historical ones and inspect **both** scan-membership sets before treating a match of overlapping raster content as new evidence.

**Candidate contract:** A portable `LoopHypothesis` contains stable query keyframe/scan or frozen submap ID, target frozen submap ID, encounter/correlation/source scan sets, retrieval mode and ranked score, OD-001 time/epoch, OD-011 pinned occupancy/EDT/pyramid and descriptor source revision/config hashes, graph/spatial-index revision, candidate covariance/window and budget state. Merge spatial and signature hits, deduplicate, and initially retain **top 3–5 distinct hypotheses**; do not create GTSAM evidence from candidate/descriptor scores alone. A worker result with stale map, membership or graph pose revision must be explicitly revalidated/rebased or rejected before graph publication.

**Verification and admission order:** (1) temporal and OD-010 source-scan/self-match exclusion; (2) OD-009 BnB against the pinned, coherent score pyramid, with admissible bounds, search completeness and explicit `INCOMPLETE`/`BUDGET_EXHAUSTED` outcomes; (3) refine each plausible distinct hypothesis using OD-007 truncated unsigned EDT, bilinear and **per-beam Huber** on a compatible immutable snapshot. An incorrect odometry prior may be used only as a seed/controlled broad regularizer for distant-loop search: **do not publish the odometry-conditioned Ceres posterior as independent LiDAR graph evidence**. Obtain separately evaluated LiDAR-only or correctly conditioned geometric constraints in OD-008 supported directions. (4) OD-008 robust LiDAR-only residual/overlap/observability/rank/calibrated covariance, timing-quality and multi-hypothesis ambiguity gates; (5) OD-013 shared scan/membership, motion-conditioned deskew, repeated/overlap-correlated factor and innovation checks; (6) **drift-aware, provisionally optimized graph consistency**. A true large-loop correction can disagree with raw odometry and still pass with strong independent geometry/other observations. An ambiguous loop or one that distorts the established map/held-out consistency must be rejected/rolled back; robust GTSAM losses are **not** an alternative to verification.

**Asynchronous work, status and provenance:** Execute against pinned immutable OD-011 map snapshots with bounded candidate count, memory, worker queue and **CPU time per invocation**, cancellation and lag/version checks. Initial **unmeasured Pi 5 targets**: spatial **1–5 ms**; descriptor lookup **5–20 ms** (Stage II); BnB **20–50 ms per candidate**; **top 3–5** candidates; **100–250 ms background batch** including refinement. Do not assume five 50 ms searches plus Ceres fit in 100 ms; enforce total budget, return incomplete status, or resume work later without blocking acquisition, recording, local odometry or local Ceres tracking. Accepted OD-021 provides initial queue capacities; B29 calibrates them using Pi evidence. Preserve retrieval/source map hashes, geometric scores, coarse completeness, Ceres convergence, OD-008 information rank/covariance, OD-013 factor group, drift innovation, acceptance or typed rejection reason, worker latency/CPU/memory and provisional graph result. When OD-014 remaps local geometry, invalidate all affected descriptors, candidate matches and loop factors using source-version dependencies; purely rigid global graph updates require only spatial-index/consistency refresh.

**Portable SDK/internal boundary:** Declare native `LoopCandidateRetriever`, `LoopVerifier`, `LoopHypothesis` and `LoopDecision` interfaces without Ceres/GTSAM/ROS/Linux IPC classes. Implement retrieval/2D signature index and orchestrator within internal `slam_frontend`, refinement in `slam_match_ceres`, and graph conversion in `slam_graph_gtsam`, respecting OD-017's eight-library target inventory.

**Required B26:** Docker/Catch2 synthetic apartment true revisits with drift, warehouse repetitive shelving false positives, OD-005 partial scans, OD-010 A/B overlapping/self evidence, OD-009 branch-bound admissibility/completeness and cancellation, OD-007 wrong-prior independent geometric refinement, OD-008 rank/covariance calibration and multiple hypotheses, false graph loop rejection/rollback, 2D signature rotation/mask/source consistency, concurrent OD-011/013 revision changes and Pi 5 background CPU/p95/p99/localization latency. No GitHub Actions.

### 7.3.2 Accepted OD-019 — Bounded online submap iSAM2 and optional correlation-preserving local smoother

**Selected online topology, staged for low implementation risk:** **Stage I** retains OD-006's independent high-rate wheel/gyro motion estimator and adds **ONE persistent GTSAM iSAM2 submap-only global Pose2 graph** inside the OD-016 SLAM process, managed by private OD-017 `slam_graph_gtsam`. The global graph stores every historical OD-010 submap pose `S_j` plus **admitted, source-traceable summarized inter-submap information and OD-018 validated loops**. It does **not** contain an unbounded stream of high-rate/keyframe pose variables and does **not** initially run a second GTSAM fixed-lag smoother or add gyro-bias knots. **Stage II (conditional)** adds a bounded local keyframe + active/recent submap GTSAM fixed-lag window only after held-out accuracy, uncertainty or node-complexity measurements justify it. The accepted OD-013 **full keyframe+submap batch graph** remains the offline reconstruction and evaluation authority, not an always-running second optimizer.

~~~text
OD-005 scan -> OD-006 continuous local motion (no GTSAM bias nodes)
     |                    |
 OD-007/008 frontend    OD-010 active/frozen submaps + OD-015 keyframes
     |                    |
 OD-013 source/correlation ledger and verified OD-018 loop evidence
                          |
                source-traceable submap summaries
                          |
       Stage I: persistent global submap Pose2 iSAM2
             historical S_j never arbitrarily pruned
                          |
         immutable graph revision -> map->odom correction
                          |
    OPTIONAL Stage II: separate bounded local fixed-lag GTSAM
      X_k + active/recent S_j, retained joint separators
         -> conditional nonduplicate summary transfer
                          |
       offline OD-013 full keyframe/submap batch rebuild
       from OD-012/014 original sources, not old summaries
~~~

**Stage I factor policy:** An anchor association or two overlapping OD-010 raster memberships is **not** independent submap-to-submap evidence. Compute a submap-level relative factor only from defensible **jointly condensed/independently verified** OD-006/007/008/013 source data, with timestamps, source scan/motion IDs, OD-011 occupancy version, OD-008 rank/calibrated noise and OD-013 correlation group. Do not synthesize several independently weighted edges by mixing fused odometry, deskewed scan matches and adjacent A/B map overlap. If correct summary correlations or calibrated uncertainty cannot be represented, **defer the factor** and preserve the raw source history rather than inventing certainty. Global iSAM2 never adds an original full keyframe factor **and** its summary as independent evidence.

**Stage II local window (not initial):** Initially propose **30–60 s** time horizon, about **150–200 keyframes**, two active submaps plus a recently frozen boundary. Real retention depends on the OD-010 90/180 insertion lifecycle and OD-015 motion keyframes. Maintain a valid local gauge, boundary references and sources; do not merely drop an old `X_k` with active factors or force the initial OD-006 bias estimator into an online GTSAM bias graph. Keyframe expiry requires numerically valid **joint marginalization** with both Schur information and vector terms:

~~~text
H = [ H_mm   H_mb ]
    [ H_bm   H_bb ]
H_sep = H_bb - H_bm * solve(H_mm, H_mb)
g_sep = g_b - H_bm * solve(H_mm, g_m)
~~~

Here `m` is eliminated local keyframe state and `b` all retained separator variables. Avoid explicitly forming `H_mm^-1`; use verified gauge/rank-aware factorization. The resulting joint separator factor may be **dense and correlated across multiple submap poses**. **Do not replace** it with artificially independent pairwise submap edges, floor its unknown nullspace into certainty or discard the RHS/linearization anchor. Mark source factor IDs as **superseded** when transferring a summary, and preserve a conditional message/ownership protocol so the local and global graphs **do not jointly count the same measurement again**. A nonleaf marginalization unsupported by the pinned GTSAM solver must be implemented through an appropriate tested elimination/fixed-lag path, not a blind `removeFactorIndices` operation. Nonlinear marginalization depends on linearization points and must be reproducible from raw factors after major loops.

**Graph growth:** OD-010 ~90 accepted scans/new submap means approximately **400 new submaps per hour at an assumed sustained 10 accepted scans/s**; this is a workload projection, not a verified dataset rate. Retain **all historic global submap variables** for OD-018 loops (even as the persistent global graph grows). Remove only duplicate or proven-invalid loops, or factors **properly superseded** by a validated summary; no arbitrary constraint pruning to meet a size limit. Emit `RESOURCE_LIMIT`/degraded status on exhausted memory instead of inventing a statistically invalid shortcut. Regional archiving/partitioning is a future measured option.

**Incremental scheduling:** OD-016 maintains a **lower-priority, bounded and coalescing graph job queue**, batch-admitting OD-013 provenance-checked submap/loop factors **on events**, not per beam. Targets: **global ordinary completed iSAM2 update p95 <50 ms**, **local smoother completed update p95 <20 ms only if Stage II exists**; a large loop may exceed these targets on a separate background job. Do not interrupt an active `ISAM2::update` at an arbitrary deadline, mutate solver state concurrently or publish a half-updated Bayes tree. Budget/timeout controls govern **admission before a job, batching, snapshots, worker queue and later scheduling**, with cancellation only between safe operations or on disposable candidate copies. Tune relinearization skip/threshold, ordering/re-elimination, factor-slot reuse and cleanup according to Pi 5 profiling. Graph update results carry exact source IDs, graph/map revision, lag, factor count, elapsed time/CPU/RSS and accept/reject status; stale outputs must be revalidated before OD-012/022 atomic publication. Local `T_odom_base(t)` stays continuous while global `T_map_odom`/`S_j` change.

**Offline cleanup / batch rebuild:** Persist **original accepted OD-013 keyframe-level factors, fused-motion samples, scans, memberships, loop provenance, calibration and source revisions**, not just compressed online edges. OD-012 map bundles contain authoritative maps/graph metadata, whereas **OD-014 full beam re-deskew still needs immutable raw MCAP acquisition**. Reconstruct a fresh full batch `X_k/S_j` graph offline from non-superseded source evidence (not old approximated summary factors), optionally re-deskew/remap selectively under OD-014, remove verified-invalid loops, optimize and validate held-out OD-008 covariance/graph consistency. Publish a **new coherent immutable OD-012 generation** only after verifying concurrent live-update revision and keeping prior accepted state for rollback. Global gauge and historical submap variable IDs must remain stable; no on-device acquisition or localizer wait for cleanup.

**Domain and test boundaries:** Native `GlobalSubmapGraph`, `LocalWindowPolicy`, `SeparatorInformation`, `SubmapSummaryFactor`, `GraphUpdateBudget` and `GraphReconstructionPlan` carry portable SE(2), factor provenance/linearization version, information cross terms, and update results without GTSAM, ROS, Ceres, Linux or IPC types. `slam_graph_gtsam` alone owns iSAM2 or optional fixed-lag implementation; no ninth OD-017 library or fifth OD-016 process. **B27**: Stage I sole global submap graph/continuous OD-006, Stage II optional 30/60 s/150–200 keyframe controlled Schur marginalization vs exact linear batch reference, true loop after large linearization changes, no duplicated information, factor removal vs marginalization, multi-hour submap growth, 20/50 ms completed-update p95 load tests, stale graph snapshots, offline batch replacement/rollback and Pi worker noninterference.

### 7.4 Accepted OD-014 — Bounded selective offline replay and reconstruction

**Offline coordinator:** Implement one deterministic full replay/map/graph pass `R0` first, then optionally **at most three additional outer candidate iterations** `R1–R3` (not three total including `R0`). A portable, ROS-independent `OfflineRefinementCoordinator` compares accepted and candidate versions. OD-006 continuous motion and optional gyro-bias updates, OD-002/003 clock corrections, OD-004 midpoint re-deskew, OD-010 submap construction, OD-011 caches, OD-007/008 geometric constraints, and OD-013 batch graph are **separately versioned dependencies**. The coordinator is outside the live sensor/odometry path and owns no Ceres/GTSAM/ROS types.

~~~text
immutable raw packets / IMU / wheel data / clock calibration + epochs
   -> R0 full deterministic replay, scan eligibility, deskew
   -> OD-010 submaps + OD-011 pinned caches
   -> OD-007/008 geometric matches + OD-013 graph batch optimization
   -> validate + OD-012 accepted immutable map generation R0
         |
 candidate Ri (i=1..3): observable local motion/bias/time updates
   -> geometry displacement test per scan and submap membership
   -> selectively re-deskew / change local scan pose / rebuild submaps
   -> rebuild affected derived EDT + correlation pyramids and factors
   -> OD-013 batch reoptimize -> held-out / consistency validation
          /                                  \
    accept: publish new Ri            reject: retain R(i-1)
~~~

| Change | Re-deskew scan? | Rebuild local occupancy? |
| --- | --- | --- |
| Global rigid loop correction / change to `T_map_submap` | **No** | **No**; update global pose/index revision |
| Common rigid change to all local poses preserving scan-to-submap geometry | **No** | **No** |
| Relative keyframe/scan midpoint placement changes in submap | Not necessarily | **Yes** when local insertion geometry moves materially |
| Intra-scan OD-004 twist/trajectory correction | **Yes** when material | **Yes** for each affected submap membership |
| Observable OD-006 gyro bias update changing beam trajectory | **Yes** when material | **Yes** if submap-local rays change |
| Observable OD-002/003 clock fit/LD19 beam timing correction | **Yes** when material | **Yes** if local beams move |
| No material local geometry change and all validity gates hold | Normally no | Normally no |

**Geometry trigger:** At OD-010's initial 5 cm grid, start with configurable **0.5 cell = 0.025 m maximum displacement** of **any valid beam endpoint between candidate and accepted geometries in the SAME local submap frame**. Calculate for all beams or use a safe analytic upper bound including yaw×range and laser lever arm; do not use global pose drift or mean displacement as a proxy. Classify **intra-scan relative motion** separately from **midpoint scan-to-submap placement**: only the first requires re-deskew, but both can require rebuilding local rays. Check both memberships for OD-010 overlapping scans. Invalid timing/clock epoch, trajectory coverage, OD-005 eligibility or serious OD-008 quality regression may invalidate the candidate **regardless** of 2.5 cm.

**Reconstruction dependency graph:** Affected submaps are **rebuilt from original timestamped, OD-005 validated rays and original insertion evidence**. Never warp or write into an accepted frozen raster, or re-apply log odds atop its already-integrated cells. Retain stable scan IDs, OD-010 hit-over-free precedence and observed/unknown flags; create new immutable occupancy source hashes. Invalidate and recompute OD-011 EDT and OD-009 score pyramids for changed occupancy, then **replace—not append—dependent OD-007/008 local and loop constraints** and OD-013 factors; preserve scan membership, correlation groups and self-match restrictions. Keep unchanged frozen local caches if geometry is rigid-only; only global indexes/revisions change.

**Observable calibration:** Bias correction must be supported by stationary or other independent measurement evidence; wheel/gyro discrepancy could be slip. Clock offset/drift must meet OD-002/003 calibration bounds and explicit clock-domain/epoch constraints. The LD19 timestamp's exact physical event is not proven by map-fit improvement; freeze unobservable time parameters and avoid absorbing map error into a clock. Changes affecting trajectory/time are versioned, bounded and uncertainty-gated.

**Iteration quality:** Relative graph objective improvement and pose-change tolerances are configurable. If the factor set/map changes, compare quality on **common measurements** and held-out scans/independent truth, not unmatched graph objectives. Gate on OD-008 residual/rank/covariance coverage, loop false acceptance, scan alignment, map consistency and timestamp calibration confidence. A lower objective on the **same measurements used to rebuild the map** alone is insufficient. Stop at 3 additional iterations, convergence/no geometry change, or first invalid/regressing candidate; preserve last accepted `R_(i-1)` and the reason.

**Publication and replay:** Each complete candidate retains immutable raw recording/MCAP hashes and source paths, OD-001 session+epoch, OD-002/003 fits, OD-004 deskew/trajectory revision, OD-006 bias, OD-005 scan eligibility, OD-010 membership/counters, OD-011 cache source hashes, OD-013 provenance/graph constraints, config/seeds and validation metrics. The **OD-012 map bundle is not itself the complete raw replay source**. Only successful candidates update the OD-012 atomic `CURRENT` generation pointer; rejected/staging-failed candidates never overwrite an accepted iteration. Accepted OD-024 defines indexed semantic checkpoint seeking (B32 tuning pending). No source GTSAM or Ceres types in `RefinementPlan`/`IterationSnapshot`/`RefinementDecision` APIs.

**B22 verification:** Docker Catch2 synthetic apartment/warehouse: exact `R0` plus ≤3 candidates; rigid-only shift triggers zero re-deskew/rebuild; relative pose-only change requires submap rebuild without unnecessary re-deskew; high-yaw and gyro/time perturbation move distant beams past 0.025 m; both overlapping submaps invalidated; no duplicated log-odds, cached fields or factors; unobservable clock/bias fixed; bad loop or held-out degradation rejects a lower-training-cost candidate and rolls back. Compare selective/full rebuild CPU/RAM, ATE/RPE, pose consistency and deterministic generation hashes on x86/Pi as available.

## 8. Known-map localization and recovery

Local tracking cycle:

1. Predict continuous `T_odom_base(t)` and covariance from wheel/gyro.
2. Assemble the scan and deskew every valid beam into its **temporal midpoint** frame through the portable, timestamp-addressable trajectory provider; require full interpolation for map insertion.
3. In TRACKING, match against nearby **frozen** map tiles using OD-007 Ceres directly. When confidence degrades, OD-009 supplies bounded local hypotheses; in recovery/global, async multi-resolution BnB supplies candidates. Every refined candidate still requires OD-008 quality and observability acceptance.
4. If accepted, produce a global pose at **scan midpoint time** and update `T_map_odom` using midpoint-aligned continuous local odometry.
5. Keep local odom publishing even if global correction is late; mark global pose freshness explicitly.

Tracking states (**accepted OD-022**, exactly four runtime states):

| State | Meaning / safety |
| --- | --- |
| `TRACKING` | Current independently validated global pose and calibrated covariance; mapping only if nominal quality gates hold |
| `UNCERTAIN` | Prediction with growing uncertainty; suspended mapping, restricted global pose |
| `LOST` | No authoritative current global pose; suspend mapping and initiate recovery when ready |
| `RELOCALIZING` | OD-020 bounded top-K recovery verification; no trusted global correction before acceptance |

**Initialization is a lifecycle condition, not a fifth tracking state.** Without a verified current map anchor start in LOST (`NO_TRUSTED_INITIAL_POSE`), not TRACKING. A new unmapped map has a separate bootstrapping mode that temporarily exempts existing-map endpoint-support thresholds while retaining time/motion/geometric integrity.

Accepted OD-020 continues to own global candidate generation and ambiguity checks; accepted OD-022 now owns four-state hysteresis and publication validity.

### 8.1 Accepted OD-020 — Global relocalization ownership and fail-closed flow

The **ROS-independent SLAM localizer coordinator** owns global recovery, not the Ceres matcher, the graph solver, the spatial retrieval index or ROS. OD-006 continuously produces local motion even when the map frame is unknown; the global coordinator pins a coherent OD-010/011 frozen-map and OD-013/019 graph-pose generation, then orchestrates:

~~~text
LOST (including no trusted startup anchor) / RELOCALIZING
          |
OD-018 eligible frozen-map enumeration + prioritized signature retrieval
          |     (spatial proximity is only a hint after kidnapping)
OD-009 async full-region, 360-degree BnB, coverage ledger and work slices
          |
spatially distinct top-K SE(2) global candidates
          |
OD-007 Ceres under recovery-mode prior + OD-008 LiDAR-only quality
          |
persistent hypothesis tracks <-- OD-006 relative odom at scan midpoints
          |                   <-- next eligible OD-005/004 LiDAR scans
geometry + observability + temporal + ambiguity/coverage gates
          |
    [unresolved?] -- yes --> remain unlocalized; repeat bounded verification
          |
          no
OD-022 version-checked map-to-odom correction; recovery-qualified TRACKING only after independent gates
~~~

**Coverage is part of trust, not a mere performance counter.** Index ranking may order jobs, but arbitrary relocation requires fallback over the **whole eligible frozen-map search domain** and 360° yaw, including regions outside historical OD-006 covariance. Track visited, admissibly pruned, pending, unavailable, omitted and failed-load submap/tile/yaw regions against exact revisions; no unvisited region is silently classified negative. A PARTIAL_BUDGET, RESOURCE_LIMIT or unavailable region that could contain a viable competitor prevents a trusted winner. Only a completed discrete search or a conservatively justified verified competitor-exclusion condition can permit commitment; tune any such early-exclusion policy in B28, never infer it from a good score alone.

Each **bounded top-K track** has stable ID, initial candidate and frozen-target IDs, map-to-odom hypothesis, relative-odom propagation and covariance, scan/time/evidence lineage, geometric rank/overlap/residual and temporal consistency history. Deduplicate by both translation and yaw; separate LiDAR-only OD-008 observability from the odometry-conditioned Ceres result; do not count repeated overlapping scans as independent proof. Reevaluate on new viewpoints when possible. Symmetric rooms, parallel corridors and partial LD19 sectors must leave plausible rivals alive and produce a truthful **unlocalized** status, not false confidence. If pruning the K cap would hide credible alternatives, mark ambiguity/overflow.

**Isolation, mapping and publication:** OD-016 schedules cancel/resume between **100–200 ms initial unmeasured** OD-009 global work slices on lower-priority bounded workers. Queue, CPU, map-pin and memory limits apply; revision changes invalidate saved frontiers and stale corrections. Keep OD-006 continuous local odom and source recording live. Suppress **all** new scan insertions, active-submap creation and localization-derived graph-factor admission until trusted map-frame recovery; never backfill scans collected while untrusted. Publish only an explicit versioned map-to-odom correction at the accepted scan midpoint after OD-022 state checks, without a jump in odom-to-base. A large correction does not rewrite immutable frozen submaps.

**Portable contracts and replay:** Native GlobalRelocalizer, RelocalizationRequest, PoseHypothesis, HypothesisTrack, SearchCoverage, RecoveryDecision and diagnostic interfaces carry map/pose/pyramid/calibration/clock revisions, source IDs, diverse candidates, quality/rank, scan lineage, visited/pruned/unsearched coverage, budget exhaustion, alternatives and decision reasons; implementation-private Ceres/BnB adapters compose them. Do not expose ROS, GTSAM, Ceres or Linux worker handles in OD-017 installed SDK headers. Stable evaluation ordering and tie-breaks, exact snapshot references and recorded scheduling/budget decisions make OD-024 deterministic replay practical.

**B28 acceptance (not yet tested):** Synthetic unknown/teleported SE(2) poses, repetitive corridors, rotated/reflective symmetries, partial LD19 scans, stale overconfident old pose, failed retrieval/partial coverage, odom drift/slip, multi-scan disagreement, map revision changes and worker starvation. Report **wrong global-lock rate** and refusal-to-localize separately from recovery rate/time, along with map suspension, continuous odom, CPU/RSS and pose-publication provenance. A high nominal success rate cannot excuse a false confident pose. Particle filtering is a later evidence-triggered comparison only.


### 8.2 Accepted OD-022 — Quality- and covariance-gated state and publication architecture

**State-owner:** The ROS-independent SLAM tracking coordinator owns only `TRACKING`, `UNCERTAIN`, `LOST`, `RELOCALIZING`. Initialization/map-bootstrap is separate state metadata. Health evaluation occurs on each scan result **and** an independent host-steady freshness watchdog; a stopped sensor must trigger aging without scans. Startup TRACKING requires an independently verified current global anchor.

| Trigger | Nominal TRACKING | UNCERTAIN soft trigger | LOST hard trigger |
| --- | --- | --- | --- |
| Since last accepted global LiDAR fix | ≤300 ms | >300 ms | >1,000 ms |
| Local rejected matches | 0–1 consecutive | ≥2 consecutive or 3/5 attempts | ≥8 consecutive |
| Calibrated global horizontal σ_xy | ≤0.15 m | >0.15 m | >0.50 m |
| Calibrated global yaw σ_θ | ≤5° | >5° | >15° |
| Acquired angular coverage | ≥90% | <90% | Persistently unusable scans/timeout |
| Largest missing acquired sector | ≤15° | >15° | Persistently unusable scans/timeout |
| Map-supported valid endpoints | ≥50% **and** ≥80 | Below either | Persistently insufficient/timeout |
| Independent LiDAR OD-008 geometry | Full rank/conditioned | Degenerate or weak | Invalid pose/uncertainty |
| Valid motion-data age | ≤100 ms | >100 ms | >250 ms |
| Clock/timestamp provenance | Valid | Degraded, bounded | Invalid mapping/reset/epoch discontinuity, **immediate** |

Compute `σ_xy=sqrt(λ_max(Σ_xy))` on the 2×2 global position-covariance block and `σ_θ=sqrt(Σ_θθ)` on its yaw variance. Preserve **full finite symmetric PSD 3×3** covariance with correlations, time/frame identity and calibration quality. Integrate OD-006 predicted uncertainty, OD-008 independent LiDAR information and OD-013/019 joint graph uncertainty without double counting shared sources. If joint correlation unavailable, mark conservative calibrated estimates or unavailable, never publish inverse Ceres Hessian alone as full global pose covariance. The motion-age limits must be revised for measured Pico/IMU/encoder sample rates.

**Precedence/hysteresis:** (1) critical clock/estimator/motion validity failure immediately → LOST; (2) any hard limit, ≥8 rejects or >1,000 ms accepted-fix age → LOST; (3) any soft quality/coverage/covariance limit, ≥2 consecutive rejects or ≥3 of last 5 → UNCERTAIN; (4) recovery only after hysteresis. UNCERTAIN→TRACKING requires **5 consecutive informative OD-008-accepted scans spanning ≥0.5 s**, `σ_xy<0.12 m`, `σ_θ<4°`, and valid sensors/timing. LOST→RELOCALIZING requires ready sensors and canonical timeline, frozen map and an initiated OD-020 search. RELOCALIZING→TRACKING requires **unique OD-020 hypothesis**, sufficient complete coverage/competitor exclusion, **5 independent informative scans over ≥0.5 s**, `σ_xy<0.20 m` and `σ_θ<7°`; abort/critical fault returns LOST. Repeated symmetric corridor observations alone never qualify; incomplete search retains ambiguous RELOCALIZING rather than false acceptance. Reset confirmation streaks on invalid gaps, clock/map/calibration changes and contradictory measurements. Hard failure wins immediately over favorable previous scans.

A relocalization fix in the `0.15–0.20 m` or `5–7°` band is **explicitly recovery-qualified/restricted TRACKING**, not permission to map or navigate: ordinary TRACKING covariance, full-rank geometry and freshness must first pass, and subsequent unresolved soft violations degrade to UNCERTAIN. Do not overwrite measured covariance to force the nominal threshold; never backfill scans from untrusted periods.

**Innovation:** For ordinary calibrated full-rank Gaussian local matches, SE(2) tangent innovation `e=Log(T_pred^-1*T_meas)` and correctly correlated `S` yield `d²=eᵀ S⁻¹ e`; `11.34` is a conditional 99% `χ²(3)` reject threshold, **not** a global BnB likelihood test, and never replaces OD-008 geometric/rank gates.

**Frame and publication contract:** Continuous `T_odom_base(t)` may publish at nominal **50 Hz** only while OD-006 motion validity permits, even in LOST/RELOCALIZING. `T_map_base(t)` receives **current valid** global updates on accepted LiDAR matches at nominal **≤10 Hz** in unrestricted TRACKING; UNCERTAIN can emit **explicitly uncertain/restricted**, covariance-inflated prediction but not a fresh global fix. LOST/RELOCALIZING emits **no authoritative current global pose** (retain last known pose as historical diagnostics with original time). Freeze last accepted `T_map_odom` in UNCERTAIN and mark it invalid for current localization in LOST/RELOCALIZING. Any discontinuous verified OD-013/019 loop correction or OD-020 relock increments `correction_generation` with coherent map/graph revisions, never alters `odom→base` continuous motion. Do not republish an old state with a new acquisition timestamp. Global warning at 300 ms and invalidation at 1,000 ms; optional live odometry extrapolation **≤10 ms**, OD-004 gates applied, never for normal insertion. Mapping and localization-derived factor admission suspend under UNCERTAIN/LOST/RELOCALIZING or recovery-qualified-but-not-nominal tracking; navigation defaults restricted/stop or disabled until a fully trusted pose.

**Portable status:** Expose `PoseEstimate` with separate acquisition/state and host publish timestamps, last-global-fix time, source clock domain/epoch, frame IDs, pose and **3×3 covariance with quality/validity**, `tracking_state`, `globally_valid`, `odom_valid`, freshness/age and failure reasons, map/graph IDs, correction generation and recovery-qualified restriction. Native `TrackingPolicy`, `TrackingCoordinator`, `GlobalCorrection` and `TrackingDiagnostics` must not depend on ROS, Ceres, GTSAM, MCAP or Linux IPC. ROS REP-105-style `map→odom` transforms alone cannot carry uncertainty and validity; OD-025 adapter must expose separate status and stop authoritative map TF publication when global localization is untrusted.

**Validation:** B30 boundary tests for 299/300/301 and 999/1,000/1,001 ms, reject 2/3-of-5/8, 0.12/0.15/0.20/0.50 m and 4/5/7/15°, coverage/endpoint/rank, covariance PSD/correlation, motion and clock jumps, OD-020 symmetric/kidnapped recovery, 5 scans spanning ≥0.5 s, map insertion restrictions, graph correction generations and stale-pose timestamp spoofing. Use deterministic Docker Catch2/CTest and held-out Pi recordings; **all thresholds are initial unmeasured values**, not deployment proof.

## 9. Accepted OD-021 — Bounded queues, recorder durability and overload isolation

The **four independent OD-016 processes** keep separate bounded acquisition→recorder and acquisition→SLAM routes; no acquisition UART/Pico callback waits indefinitely for disk, Ceres, GTSAM, map/cache work, ROS or Web Lab. Each queue has an enforced **item-count, byte-size and (where applicable) oldest-message age** maximum; host-steady age/deadlines are not ROS simulated time. Validate sequence, run/epoch and source IDs through every fan-out hop so that drops are explainable. Source event timestamps must never be rewritten to disguise late processing.

~~~text
LD19 / Pico / IMU / wheel source acquisition
           |
    per-source bounded rings (detect sequence/overflow)
           |
      nonblocking independent source/event fan-out
       /                                  \
raw sequence+epoch                     motion timeline
(16 MiB / 5 s backlog)                (1 s history cap)
       |                                  |
RECORDER worker                     scan assembler (8 scans)
MCAP chunks -> sink flush               |           |
OS fdatasync/fsync                        |       OD-006 continuous odom
segment rotation                         v
integrity/manifest                  OD-004/007/008 local matching
                                           |
                                 accepted graph factors / versions
                                           |
                               coalesced optimizer (2 pending jobs)
                               cache builds (1 running + 1 pending)
                                           |
                                coherent versioned state/snapshots
                                           |
                             latest telemetry / map snapshot resync
                                           |
                                         Web Lab
~~~

### 9.1 Initial queue allocations (unmeasured Raspberry Pi defaults)

| Owner / boundary | Initial cap | Full/stale behavior |
| --- | --- | --- |
| LD19 receive ring | **512 packets + 128 KiB** | Drain UART; detect/report lost packet IDs and invalidate raw completeness if required data lost |
| Pico receive ring | **512 frames + 256 KiB** | Detect boot/sequence discontinuity; mark motion intervals invalid |
| Raw acquisition→MCAP | **16 MiB or 5 s maximum age**, plus finite measured item cap | No silent raw drop; pressure alerts and sticky session failure on required loss, producer remains nonblocking |
| SLAM motion history | **1 s**, plus count/bytes | Maintain trajectory validity; missing interval invalidates affected deskew, not interpolated as truth |
| Completed scans | **8**, plus count/bytes/age | Drop oldest unprocessed complete stale scan with ID/time gap; never insert dropped scans |
| Graph optimization | **2 pending**, plus bytes/jobs | Coalesce optimization triggers by target version, **not** authoritative factors/loops |
| EDT/map cache | **1 running + 1 pending** | Supersede stale rebuild and check pinned snapshot/revision constraints |
| Web telemetry | **1 latest state per stream** | Replace obsolete delivery without upstream backpressure |
| Web map updates | **2 pending versions** | Abandon superseded deltas, deliver a complete versioned map snapshot on version gap |

For each route implement a finite message count and payload-byte ceiling even if an initial row emphasizes time or job count; determine omitted numerical ceilings from measured largest events and worst-case burst. A limit fires on the **first of** count, bytes or permissible age. Overflow and deadline results are **explicit, metered** and deterministically replayable. A 5-second raw backlog maximum is **not** a promise to absorb all 5-second stalls on 16 MiB; both bounds apply. These settings are not Pi-tested production guarantees.

### 9.2 Recording ownership and durability

Only the recorder owns the native **MCAP C++ writer** and session manifest; **accepted OD-023** fixes self-described, versioned native **Protobuf** channel schemas embedded in MCAP (B31 tests pending). Initial writer setup: **one writer thread; LZ4** (benchmark Zstd), **1 MiB or 1 s per chunk** (whichever comes first), **chunk CRC enabled**, **successful OS disk synchronization target ≤1 s**, **rotation at 60 s or 256 MiB** (whichever first). The recorder may call MCAP \`closeLastChunk()\` at a time deadline, then flush its writable sink. Neither that flush nor a successful Unix socket send proves persistence: an FD-backed sink must separately check \`fdatasync\`/\`fsync\` (plus directory synchronization for necessary renamed manifest/segments). A failed sync or uncertain tail cannot advance the **durable high-water mark**. Log independently received/accepted, written, chunk-completed, synced and finalized source IDs.

**State machine:** \`RECORDING\` → \`DEGRADED\` at **70%** queue use or latency/age warning, **critical alert at 90%**; \`FAILED\` on missing raw evidence, overflow, write/sync failure or unprovable continuity; healthy termination goes through \`FINALIZING\` to \`COMPLETE\` only after bounded drain, MCAP footer/index, successful synchronization, integrity checks and durable atomic session-manifest update. A FAILED session's incompleteness is irreversible, even if later segments are rescued or new sessions succeed. A forced crash, disk-full, interrupted chunk or unavailable device produces an explicit *recoverable-prefix* status, not an apparently complete bag. Recover only complete CRC-valid records/chunks, preserve sequence/clock discontinuities, quarantine invalid tails and do not fabricate a missing footer.

### 9.3 Processing, backend and UI consistency

OD-006 high-rate motion is consumed and validated independently from scan processing; loss of gyro/wheel intervals must invalidate OD-004 deskew coverage and OD-022 pose quality. Skip a complete stale scan if necessary, log processing gap and never use it for submap insertion, keyframe selection or factor admission. Long Ceres matching cannot freeze odometry. The graph owns admitted OD-013/019 factors in durable/in-process authoritative state or journal **before** a coalescible optimization request; never drop a loop closure to clear the work queue. Cancel/supersede redundant graph/cache *jobs* with version/revision checks, preserve immutable OD-011 snapshot pin limits, and degrade or pause expensive new verification when backend age grows.

Web Lab streams latest local/global state and coherent versioned map snapshots; missed map deltas are repaired by requesting a full snapshot, not by buffering an unbounded history. Command authorization, acknowledgment and applied-config generations follow OD-016; *commands are not lossy telemetry*. No HTTP/WebSocket or browser deadline may propagate to acquisition/SLAM.

### 9.4 Observability and fault-driven calibration

Expose per-queue current/high-water items, bytes and oldest age; per-source sequence/gaps, overflow and drop reasons; lateness/processing deadline misses; graph/cache version lag and coalescing; recording queue pressure, MCAP chunk/CRC/segment status, written versus successfully-synced ranges, throughput, fsync duration/failure, recovery outcome and sticky \`INCOMPLETE\` flag. Rate-limit exported diagnostics without losing event counts and preserve deterministic replay information. Benchmark **B29** injects disk stalls/full, interrupted writes/recorder kill, acquisition outrunning SLAM, Pico loss/reconnect, multi-second graph stalls and slow/disconnected Web Lab consumers. Calibrate all initial capacities and warnings using *actual* Pi worst-case LD19/Pico rates, CPU/thermal load, SD/SSD latency and p50/p95/p99 tails. Until B29 runs, numerical bounds are recommendations only.

Startup/shutdown still follows OD-016 process ownership and lifecycle (\`CREATED → CONFIGURED → CALIBRATED → RUNNING → DEGRADED/STOPPING → STOPPED/FAILED\`); recorder completeness is a **separate** session state machine. Calibration hot changes require an explicit reset or transactional version handoff. Raw MCAP remains independent of lossy SLAM/Web Lab processing; no ROS dependency is introduced.

## 10. Persistence, reproducibility and interfaces

**Raw MCAP** should preserve LD19 bytes, raw IMU, real wheel measurements (or explicitly labeled derived wheel odom), sensor/host clocks and calibration. Native converters handle ROS 1/2 bags without making the core ROS-dependent.

- Each run gets dataset ID/hash, calibration revision, config digest, map revision, software commit, Docker image and event-time provenance.
- Derived scans and optimized trajectory never replace raw source measurements.
- Replay supports pause, step per completed scan, deterministic seek/reset and independent dataset evaluation.
- External benchmark downloads are opt-in and verified; no downloads during normal CMake/test setup.
- Web Lab uses local bounded HTTP/JSON, Canvas layers, immutable snapshots, scan/time playback and exported measurements from the *real* C++ pipeline.
- ROS 2 adapter maps native scan/IMU/odometry/map/pose to standard topics and owns TF/clock conversions; only one publisher per TF edge.
- No browser, ROS or Node/JS runtime is required for the headless core.

Accepted OD-023 native Protobuf MCAP channels/descriptors, accepted OD-024 indexed seek/semantic checkpoint format (**B32** pending), accepted OD-025 Jazzy/Ubuntu 24.04 arm64 standalone ROS adapter and QoS defaults (B33 pending). **OD-001 signed-ns time, OD-002 Pi-first/Pico-future acquisition, and OD-003 LD19 beam timing/clock alignment are accepted**: use signed 64-bit nanoseconds with explicit clock-domain conversion and ROS-neutral simulated-clock events.

### 10.1 Accepted OD-012: versioned directory map bundles and generation publication

**Persistence format:** The project saves authoritative frozen local occupancy **log-odds AND observed/unknown masks**, OD-010 insertion provenance and local origins, graph node/factor/pose/uncertainty state (using accepted OD-013 topology and portable factor records), and all calibration/frame/clock metadata needed for reconstruction. The **directory-based bundle** uses a **strict JSON `manifest.json`** and **individually compressed Zstandard binary files**. OD-007 EDT and OD-009 correlation pyramids are optional, rebuildable OD-011 caches, not required authoritative contents. The native `MapBundleWriter`/`MapBundleReader` (illustrative API names) are **ROS-, Ceres-, GTSAM-native-object- and hardware-driver-independent**; graph-specific conversion remains an adapter. MCAP remains the **raw sensor recording** format and is not a replacement for the map bundle.

~~~text
building_a.slam/
  CURRENT                                  # checkpoint generations: atomically published pointer
  generations/
    000000000001/
      manifest.json                        # strict schema and file index
      graph.bin.zst                        # graph nodes/poses/factors/information/provenance
      submaps/
        sm_000001.bin.zst                  # independent log-odds + observed mask
        sm_000002.bin.zst
      cache/                               # OPTIONAL, hash/config-keyed derived files
        sm_000001.edt.bin.zst
        sm_000001.corr.bin.zst
      preview.png                          # OPTIONAL visual-only
      checkpoint/state.bin.zst             # ONLY when resumable (live mapping/clock state)
~~~

A single export may be a self-contained directory with `manifest.json` and required payloads. **Repeated checkpoints MUST publish new immutable generation directories and atomically replace a small `CURRENT` pointer**; readers pin one generation. Do **not** assume renaming over an existing nonempty directory works portably, and do not write the current live bundle in place. Flush and validate staged files first, then same-filesystem rename staging to a **nonexisting generation name**, sync its parent, atomically update/durably sync `CURRENT`, and retain the previous valid generation until the successor is verified. If an operation or filesystem cannot provide the necessary atomic/durability primitives, fail safely and report reduced guarantees; visibility atomicity and power-loss durability are **not equivalent**.

**Required JSON metadata:** Major/minor schema, required/optional feature flags, unique map/generation ID and provenance; coordinate/SE(2)/covariance frame and tangent conventions; explicit grid resolution/width/height and **rotated local origin**; submap IDs, frozen/active state, accepted insertion IDs and hashes; graph schema/node/factor references, constraint information, pose-table revision; calibration/clock-domain/session/epoch and time-conversion revision; record encoding/codec and a **complete index of safe relative paths, compressed/uncompressed byte sizes and SHA-256 for every required file**. Optional generated fields carry source-occupancy hash, threshold/known-mask and algorithm/config hashes; a missing, corrupt or stale optional cache is ignored and regenerated, never makes an otherwise valid authoritative map unloadable.

**Portable payloads:** Encode fixed-width integer and documented IEEE-754 floating fields in explicit byte order (initial proposal: **little-endian**) with versioned record headers, finite-value validation, documented row/cell/mask ordering, transform/covariance units and bounds. Do not save C++ object layouts, platform `size_t`, Eigen memory or native GTSAM classes. Each submap file is **independently zstd-compressed** and loadable on demand; graph/checkpoint files may be similarly compressed. Compression uses **Zstandard frames (RFC 8878)**, which can be decompressed independently **per file**, not randomly accessed inside a single frame. Bound file and decompressed sizes, zstd window/memory, record counts and total bundle bytes to avoid uncontrolled allocation.

**Reader integrity and publication:** Validate manifest/required features, path containment (no absolute paths, `..` traversal or symlink escapes), duplicate IDs, file hashes and size limits, graph reference integrity, map geometries, known masks and all **required** payloads before presenting a reconstructed map/pose graph to the runtime. Recovery scans bounded prior completed generations if `CURRENT` is missing/invalid; ignore incomplete staging. Unknown optional safe fields may be ignored; incompatible major versions or unsupported required features must fail explicitly. Migration is an **explicit new-generation write** with the old original retained. SHA-256 validates integrity, **not authenticity**.

**Map versus resumable checkpoint:**

| Type | Required state | Permitted operation |
| --- | --- | --- |
| `LOCALIZATION_MAP` | Frozen authoritative occupancy, frame/calibration identifiers, pose graph and supported constraints, matching configuration | Read-only known-map localization, graph inspection/export; **not** live state continuation |
| `RESUMABLE_CHECKPOINT` | All map/graph state **plus active submaps, OD-006 bias/trajectory and covariances, clock-domain/session origin + conversion mapping, scan insertion high-watermark, recorder/replay cursor/source identity and live graph state** | Explicit validated resume, after clock identity remapping/reinitialization |

A prior process-local monotonic nanosecond epoch is **not** safely reusable after reboot. Store OD-001 clock-domain/epoch/session metadata and necessary mapping provenance, then establish a valid new session mapping during resume; do not treat creation wall time as acquisition time. Accepted OD-024 defines initial 30 s/freeze-boundary semantic checkpoint cadence and indexed replay; B32 must calibrate storage and Pi latency.

**ROS adapter separation:** ROS 2 Nav2 YAML/image/OccupancyGrid import/export uses a dedicated adapter and must report the loss of native graph, submaps, log-odds nuance, clock metadata or unknown/observed neutral distinctions. The native binary bundle **must not** depend on ROS/Nav2 packages. A single-file distribution package and SQLite/database-backed tiles are **deferred**, not hidden baseline dependencies.

**Verification:** Docker Catch2 B20: binary golden vectors and x86-64/aarch64 roundtrip, rotated origins, known/unknown-neutral probabilities, graph cross-references/uncertainty, per-submap zstd lazy load and memory caps, schema/feature/migration tests, optional cache mismatch rebuild, required-file corruption/path traversal/decompression-bomb rejection, staging/rename/`CURRENT` crash recovery, map-only versus resumable checkpoint and OD-001 clock epoch integrity. Measure Pi 5 disk space, serialization/checksum/compression CPU, save/read latency and safe behavior under power loss/disk full. No GitHub Actions usage.

### 10.2 Accepted OD-023 — Native Protobuf MCAP recording and offline reconstruction

**Authoritative acquisition recording:** The only raw-device recording container is **MCAP**, with `slam.recording.v1` **Protobuf** messages and complete `FileDescriptorSet` schemas embedded in each independently playable segment. This is separate from OD-012 authoritative map/checkpoint bundles; map bundles are not redundant raw recording formats. The OD-016 **acquisition** process owns LD19/Pico/IMU/wheel device access, original capture timestamps and source sequences. The **recorder** process and `slam_recorderd` use the native C++ MCAP writer via private `slam_storage` adapters; storage owns corresponding MCAP/Protobuf **reader/replay** as well as writer. OD-021 still owns queue caps, durability, write/sync/rotation, `INCOMPLETE` integrity and recoverable-prefix semantics. No ROS/Protobuf/MCAP dependency in `slam_core`, `slam_mapping`, domain estimation APIs or public `slam::core` headers.

| Channel | Evidence / typed payload |
| --- | --- |
| `/raw/ld19/packets` | Exact original **47-byte** UART packet payload and Pi receive time, assigned capture sequence, CRC/framing status; retain CRC-bad packets and recoverable raw fragments where buffering makes capture possible |
| `/raw/pico/frames` | Original **unmodified received Pico wire frame**, separate device and host sequence/tick/reboot generation, CRC/framing status and Pi arrival time |
| `/time/pico_sync` | All four timestamp exchange observations including rejected/outlier samples, original source time units/clock domains, exchange ID and quality |
| `/time/clock_models` | Versioned offset/drift/covariance/uncertainty, clock generation/epoch, validity and source observation provenance; **derived** fit does not replace original exchanges |
| `/events/acquisition` | CRC/framing faults, inferred LD19 angular gaps, queue overflow/loss, Pico source gaps, resets/reconnects, changing clock epochs and recorder completeness |
| `/config/calibration` | Timestamped change events, effective-at IDs, source/extrinsic/clock/device/firmware/protocol/config revisions |
| `/derived/odometry` | Optional regenerable reconstructed OD-006 trajectory and covariance |
| `/derived/lidar_scans` | Optional regenerable OD-005/004 scan/deskew and partial coverage |
| `/derived/localization` | Optional regenerable OD-022 state/pose/full covariance and correction version, validity explicit |

**Raw versus derived:** Raw channels plus timing/config evidence are required for a complete raw session and OD-014 recalibration. Missing required raw events make the recording **FAILED/INCOMPLETE** under OD-021; missing optional derived streams do not. LD19 packets have **no on-device sequence field**: assigned capture sequence detects downstream loss after assignment only, not packets lost before receive/framing. Detect such early gaps conservatively with actual angle, timing and expected-revolution continuity evidence, flag estimated/inferred status separately from observed packet sequence gaps. Pico native sequence, when available, has different evidential meaning and must be kept alongside acquisition sequence.

**Embedded schema and compatibility:** Every channel has `message_encoding="protobuf"` and an MCAP schema with `encoding="protobuf"`, **fully qualified message name** and binary `google.protobuf.FileDescriptorSet` including imports. Channel/schema IDs are scoped to each MCAP file; each rotated segment is independently self-describing. Pin schema/hash, producer/version and channel contract metadata. Native Protobuf v1 messages use `proto3` with explicit optional/presence fields for device ticks and uncertainty, reserved removed tags, no reuse of numbers or unit reinterpretation. Separate `RawSensorFrame`, `PicoSyncExchange`, `ClockModelEstimate`, `AcquisitionEvent`, `CalibrationUpdate` and derived type schemas. Decode should work from a segment's embedded descriptors without installed source `.proto` when tooling supports dynamic parsing; incompatible changes require new versioned namespace and explicit refusal/migration.

**MCAP time semantics:** MCAP `log_time`/`publish_time` are unsigned 64-bit nanoseconds against **a user-defined, explicitly recorded epoch**, not automatically UTC. `log_time` captures host-monotonic Pi **reception** relative to the recording-session origin, checked to be nonnegative. `publish_time` uses real producer publication time if known in **the same epoch**; otherwise it equals log_time with explicit fallback reason. Neither means LD19 individual-beam physical acquisition time. The Protobuf payload preserves original **signed OD-001 host/source time**, raw device ticks/counter wrap, clock IDs/domains, boot and clock generations, uncertainty and all sync/calibration observations. Session and per-segment metadata preserve host-steady origin, clock/canonical epoch mapping, wall-time mapping (if available), rollover/reboot discontinuities, time units and reference calibration revisions. Reject invalid signed-to-unsigned conversion rather than clipping, and preserve different clock domains as distinct facts.

**Deterministic replay:** Implement in `slam_storage` on the native portable `RecordingReader`/`ReplayEvent` boundary: read authoritative raw and event channels, validate descriptor/segment/session/epoch/source-ID and sequence continuity, restore the original **ingest order index** and causal config/clock events, and feed existing OD-002/003/004/005/006/014 processing. MCAP log-time ordering alone may not recover cross-source ties/out-of-order acquisition; persist explicit ordering and deterministic tie-break information. Raw events and all sync exchanges permit replay with a **new clock fit**, per-beam timing model and calibration independent from stored optional derived data. Preserve original data during ROS import/export by optional adapters only; no core rosbag2/ROS dependency or second custom raw capture container. Accepted OD-024 specifies versioned semantic checkpoints and indexed seek layered over the already-correct start-to-finish replay; B32 validates both.

**Verification:** **B31** tests exact invalid/valid LD19 and Pico bytes, Pico four-time sync/outliers, raw-only offline trajectory reprocessing, LD19 30-s device-tick wrap, source restart/clock epoch and host-time conversion, independent dynamic Protobuf descriptor reads, backwards/forwards schema evolution, sequence loss vs merely inferred LD19 gaps, MCAP tie order, config-effective changes, OD-021 crash-recovered incomplete segments and optional ROS conversion. Verify public core headers have **zero** MCAP/Protobuf/ROS leakage; Docker x86-64/ARM64 and measured Pi overhead to be reported, **tests not yet executed**.

### 10.3 Accepted OD-024 — Indexed MCAP seek with semantic state restoration

**Ownership and trust:** `slam_storage` provides a ROS-/solver-independent `ReplayReader`, `ReplayCursor`, `SemanticCheckpointWriter/Reader`, `CheckpointCompatibility` and `ReplayResult` boundary. OD-023 **raw MCAP and original ingest-order IDs are authoritative**; separately versioned checkpoints are **disposable accelerators** referencing immutable OD-012 submap generations and authoritative portable OD-013/019 factor records, not replacement raw recordings. Native MCAP/Protobuf, GTSAM, Ceres, operating-system FDs and ROS serialization stay behind internal adapters. Missing/invalid checkpoints never prevent a correct replay from verified session-start raw MCAP.

**Initial, unmeasured policy:** Issue a semantic checkpoint **approximately every 30 seconds of recorded replay time** at the next coherent deterministic publication/commit frontier, and additionally after a completed OD-010 submap-freeze transaction. Bound total sidecar bytes/count, in-flight serialization, checkpoint age and pinned immutable map generations. With uninterrupted compatible intervals, target **≤30 s of event warm-up** but do **not** confuse that with a ≤30-second seek wall time; delayed/missing/incompatible checkpoints can require longer. Stage I implements sequential replay as baseline; Stage II adds indexed seek/checkpoint restoration; Stage III tunes interval, asynchronous capture and derived cache reuse from B32 Pi measurements.

~~~text
recording/                                     # distinct from OD-012 map bundle
  raw/
    segment_0001.mcap                           # authoritative OD-023 inputs
    segment_0002.mcap
  checkpoints/
    index.json                                  # atomically published compatible frontier index
    checkpoint_000001/
      manifest.json                             # schema/config/source/map hashes + cursor
      semantic_state.bin                        # portable versioned field records
  manifest.json                                 # OD-021 recording completeness
~~~

**Time locator versus logical processing order:** Configure MCAP chunking, **message indexing and summary/footer** in each finalized OD-021 segment (native `McapReader::readMessages` can narrow reads to a timestamp interval when indexes/summary exist). Message/chunk indexes are keyed to MCAP `log_time` (recording-session-relative host monotonic receive ns), not canonical sensor acquisition time or processing order. Read by indexed time window **only as a file locator**; explicit OD-023 **global ingest/order index**, exact segment/record identity, source sequences/epochs and logged scheduler commit IDs define the actual total event order. A cursor must name the **first not-consumed event**, including buffered-but-unprocessed events and which worker outputs were accepted but not yet committed. In case of timestamp ties, delayed source events, cross-segment boundaries or truncated files, choose a conservative earlier index window/full verified scan rather than skipping an event. Raw session completeness remains governed by OD-021 regardless of seek success.

**Portable checkpoint state matrix:**

| Semantic layer | Minimum required restoration |
| --- | --- |
| Identity/cursor | Raw manifest and source segment hashes, schema and algorithm revision, last consumed and next ingest global IDs, segment/index hints, per-source sequence/epoch, buffered unprocessed stream members and committed asynchronous frontier |
| Clocks / configuration (OD-001/002/003/023) | Clock fits and fitting history/observations, Pi/Pico generations, delayed sync exchanges, calibration/config and effective-at events, time reorder state |
| LD19 assembly (OD-003/005) | Seam and unwrap, partial revolution, accepted/rejected packet/beam IDs, pending beams and coverage state |
| Motion/deskew (OD-004/006) | Relative odom pose/twist, gyro bias/filter/integration state, covariance and supported trajectory knots, invalid motion gaps, interpolation/reorder queues |
| Localization (OD-009/020/022) | Tracking state, fix time/age, hysteresis counters/window, calibrated covariance and validity, correction generation, persistent top-K hypotheses, search coverage/frontiers/queued verification |
| Mapping/keyframes (OD-010/015) | Active mutable occupancy log odds/known masks, insertion counts, keyframe selection provenance and submap memberships; immutable frozen submap **verified content hashes/revisions** |
| Graph (OD-013/018/019) | Portable accepted factor/value/pose records, source correlations and versions, optimizer/relinearization settings, pending factors/loop decisions and deterministic update/commit ordering |
| Scheduler and randomness (OD-011/016/021) | Pending versioned worker requests, queue contents, staged map/graph results, logical commit events, deterministic tie breaks and PRNG state |
| Derived caches | OD-011 EDT and OD-009 pyramids **version identity only**; regenerate against pinned map revision when needed |

Do **not** persist native `ISAM2`/Ceres memory dumps, pointers or Boost object layout as the stable schema. Graph restore reconstructs native GTSAM state from portable factor/values and update history, checking compatibility and replay equivalence; relinearization or asynchronous execution affecting results must be captured as deterministic semantic commit events or force safe earlier-checkpoint/from-start fallback. Build a checkpoint only from a complete consistent frontier, not arbitrary in-flight mutable worker state. Preserve immutable referenced maps until no checkpoint needs them, with bounded garbage collection; if pin budget cannot be maintained, invalidate/prune older checkpoints without touching raw MCAP or OD-012 authoritative map snapshots.

**Restore procedure:** (1) Resolve request into **historical as-of-T** versus **offline OD-014 globally optimized reconstruction generation**, including recording/clock epoch and inclusive boundary for equal-time events. (2) Locate latest compatible checkpoint preceding the target *logical event frontier*, validating raw/session/segment hashes, state schema/feature and solver/algorithm versions, exact config/calibration and immutable map/graph IDs. (3) Restore clock, assembly, motion, localizer, active maps, graph/factors, pending queues and PRNG; rebuild derived cache revisions. (4) Locate MCAP segment/time via native indexes where usable, scan earlier as needed, then **resume at the first not-consumed global ingest event**, processing restored buffered items exactly once; replay asynchronous graph/map commit events in recorded deterministic logical order (never let worker completion on this host introduce future knowledge). (5) Publish state at the requested logical frontier with OD-022 actual original state timestamp, calibrated covariance, status, map/graph/correction revision and an explicit `HISTORICAL` or `OFFLINE_OPTIMIZED` mode flag.

**Historical commit evidence:** To make session-start fallback genuinely reproduce the online *as-of-time* graph, record control commands and nondeterministic accepted optimization/map **commit decisions** (with logical ingest frontier and graph/map revisions) as typed OD-023 **MCAP runtime diagnostic events**, or require a proven deterministic scheduler. The sidecar may index/snapshot these events but cannot be their only authoritative source. If original online race ordering was not recorded or reproducible, mark exact historical replay **unavailable** rather than silently substituting a rerun with different knowledge at T.

**Atomic sidecar persistence:** Follow OD-012 staging/checksum/size/path validation, file+directory synchronization, manifest and **atomic index/current pointer publication**; recover the last fully published checkpoint, reject partially written or corrupt/incompatible checkpoints and fall back to earlier compatible or session start. Segments, referenced immutable maps and checkpoint payloads are content-verified. Checkpoint sidecar failure must **not** mark an otherwise healthy raw MCAP session incomplete or block acquisition; OD-021 still controls recording completeness, bounded IO and queue isolation. A user-requested historical state must never include post-T loop closures; an offline globally optimized map may revise old poses but must be labeled a **different result**.

**B32 verification (not yet executed):** Synthetic exact-session baseline versus checkpoints with random, equal-timestamp, late/out-of-order and boundary seeks; 30 s/freeze cadence; Pico sync/clock epochs and LD19 partial scans; OD-006 bias, trajectory and covariance; active/frozen submaps/keyframe IDs; accepted/pending loops, OD-019 iSAM2 deterministic continuation; OD-020 competing recovery hypotheses; OD-022 trust/pose publication; corrupt/missing/incompatible sidecar and immutable map refs; segment rollovers, MCAP index disabled/truncated and B29 disk faults; and deliberately injected future loop closure. Compare **exact hashes** for discrete data/decisions and **declared numerical tolerances** for solver floating state; measure checkpoint size, per-seek wall time versus recorded warm-up length, index fallback cost, Pi CPU/peak RSS and disk contention. No runtime or measured Pi performance is claimed.

### 10.4 Accepted OD-025 — Optional ROS 2 Jazzy 24.04 arm64 IPC integration

**Placement:** Keep the four mandatory OD-016 processes (`acquisition`, `recorder`, `slam`, `web_lab`). An **optional, independently launched `slam2d_ros` ament_cmake/Jazzy integration process** connects **only through OD-016 versioned `SOCK_SEQPACKET` native IPC**, consuming existing source/pose/map/status streams and issuing only validated commands. It does not open LD19/Pico devices, own maps, run a separate SLAM frontend/graph or replace native Protobuf MCAP recording. Use an independently pinned **Ubuntu 24.04 Noble arm64 / ROS 2 Jazzy** adapter Docker/colcon image and installed ROS-neutral C++ SDK; the native CMake/Docker build, SDK, headless recording/replay, hardware and persistence must work without ROS, ament, DDS or rosbag2. `slam2d_ros` is one optional **integration package**, not an additional compulsory core target. Lyrical on Ubuntu 26.04 arm64 is a future separately tested adapter build, not source/API changes.

~~~text
LD19/Pico source -> native acquisition -> raw recorder (authoritative OD-023 MCAP)
                                |
                                +-> OD-016 native bounded IPC -> native SLAM/map
                                                                 |
                                         versioned telemetry/status/map IPC
                                                                 |
                                  optional slam2d_ros (Jazzy, isolated)
                                  |        |          |          |
                                 /scan   /odom   /slam/pose     /map
                                 /points  /diagnostics   /tf  /tf_static*
                                                       |
                                               Nav2 / RViz / optional rosbag2
* static edges only if adapter is explicitly appointed sole static authority
~~~

**Publishers and default QoS:**

| Topic | ROS 2 type | Reliability | Durability | Depth |
| --- | --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | best effort | volatile | 5 |
| `/points` | `sensor_msgs/msg/PointCloud2` | best effort | volatile | 5 |
| `/imu/data_raw` (optional) | `sensor_msgs/msg/Imu` | best effort | volatile | 5 |
| `/odom` | `nav_msgs/msg/Odometry` | reliable | volatile | 10 |
| `/slam/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | reliable | volatile | 5 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | reliable | transient local | 1 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable | volatile | 10 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | standard tf2_ros broadcaster | standard (static is late-joiner capable) | broadcaster-defined |

All topic names, frame IDs and supported QoS fields are adapter parameters/remappable with startup compatibility checks. Publishers and nonblocking subscriptions follow accepted OD-021 byte/count/age limits: slow ROS/TF/Nav2 subscribers never stall native acquisition, local odom, graph or recording. Transient-local `/map` publishes coherent versioned *full* snapshots for late joiners; do not replay an unbounded delta backlog. A normal `LaserScan` represents genuinely uniform angular/time samples with a **first-beam acquisition timestamp** and correct angle sign; OD-005 partial/nonuniform scans must instead be represented faithfully via `PointCloud2` with documented per-point timing, not a forged raw scan.

**REP-105 transform authority and OD-022 gating:** Exactly one designated broadcaster for each `map→odom`, `odom→base_link` and `base_link→{laser,imu_link}` edge. `slam2d_ros` may own `map→odom` when global authority is configured, never simultaneously with AMCL/another localizer; continuous `odom→base_link` comes from exactly one configured local odometry provider (disable adapter broadcast if external); extrinsics come from robot_state_publisher or one named static publisher. Compute `T_map_odom(t) = T_map_base(t) * inverse(T_odom_base(t))` using same valid acquisition time, matching source/clock/map/graph generation, and record OD-022 `correction_generation` on discontinuous relock or graph publication without jumping local odom. `TRACKING` publishes only currently authorized valid corrected poses/TF. `UNCERTAIN` issues restricted/uncertain status but **no current authoritative global TF by default**; `LOST`/`RELOCALIZING` issue neither current valid global pose nor new global TF. Static extrinsics and valid local odometry remain available as their own health permits. TF buffers may retain past transforms: consumers must check explicit `/diagnostics` tracking/freshness/validity, and stale TF cannot be refreshed with fake stamps.

**OD-001 ROS clock adapter:** The domain's signed int64 nanosecond timestamps/clock domains/epochs remain canonical. In live hardware mode monitor a measured **monotonic→system/ROS time mapping**, offset/drift/uncertainty/jumps and conversion overflow; stamp sensor ROS messages at *actual acquisition* time (or reconstructed first beam). `node->now()` on arrival is not a substitute. During `use_sim_time`, respect `/clock` readiness, pauses, resets/backward seeks and replay epochs: convert to the selected simulated timeline and suppress unconvertible/unsafe data. The bounded source→ROS transform must be validated, and clock discontinuities may invalidate TF and odom interpolation according to OD-001/022. Do not assign physical beam time to OD-023 MCAP `publish_time` or confuse ROS system time with session-relative MCAP log time.

**Recording interoperability:** `rosbag2_storage_mcap` writes **secondary** ROS message-typed recordings (`ros2 bag record -s mcap /scan /points /odom /slam/pose /map /tf /tf_static /diagnostics`). These MCAPs are not the authoritative raw `slam.recording.v1` Protobuf MCAP format, and a native raw file is **not automatically `ros2 bag play` compatible**. Keep optional exporter/converter in the adapter tooling: consume OD-023 `slam_storage` native raw events and OD-024 replay, transform to appropriate ROS serialization/time/frames, write ROS-compatible topics and document conversion losses. No rosbag2 in recorder, source drivers, `slam_core` or portable `slam_storage`.

**B33 tests (not yet executed):** ROS-free installed native SDK/CTest; standalone 24.04 arm64 Jazzy ament/colcon container and IPC version negotiation/reconnect/queue limits; message fields/covariances, map origin rotation/unknowns, true scan first beam/partial point cloud, TF authority duplicate prevention and OD-022 LOST stale transforms, time mapping wall vs `/clock` pause/seek/reset/epoch/uncertainty, QoS overrides/late map subscribers, Nav2/RViz frames, native versus ROS MCAP/play/export, and CPU/RSS/latency/queue faults on target Raspberry Pi. Keep OD-025 acceptance distinct from implementation and target hardware measurements.

## 11. Accepted OD-016: four-process isolation, versioned IPC and bounded workers

### 11.1 Process and ownership boundary

The **Pi 5 production layout** is exactly **four independently supervised processes**, with versioned native Unix IPC:

~~~text
hardware LD19 / MPU6050 / encoders / optional Pico
                  |
        [1] acquisition : own serial/I2C/GPIO/Pico FDs,
            source packet validity, OD-003 time and clock epochs
               /                              \
      RAW events, sequence                 SENSOR event stream
      with gaps/calibration                 bounded nonblocking
              |                                   |
        [2] recorder                           [3] slam
        raw OD-023 MCAP             ingest -> OD-006 odom/OD-004 deskew
        durability/INCOMPLETE         -> OD-007 Ceres/OD-008 localizer
                                             |           |
                                        map/cache     graph GTSAM
                                         worker        worker
                                             \           /
                                        immutable revision handoff
                                                  |
                                       rate-limited telemetry
                                                  |
                                            [4] web_lab
                                    validated async commands to SLAM
~~~

**Exclusive ownership:** Acquisition alone controls hardware descriptors, Pi→Pico time sync and source clock-generation. Recorder alone owns raw MCAP durability, segment closure, packet-gap/config/synchronization metadata and a lossless-*claim* eligibility decision. SLAM alone owns local state, Pose2/mapping and graph workers; graph remains **in-process**, not a fifth process. Web Lab only owns presentation/validated commands, **not** direct sensor or map mutation. Portable estimator, mapper and dataset libraries can still run within a standalone CLI/test executable **without starting IPC or Web Lab**. The optional ROS2 adapter remains outside these domain dependencies and must not introduce ROS into acquisition/SLAM math.

### 11.2 Versioned native IPC semantics

**Initial transport:** `AF_UNIX/SOCK_SEQPACKET` connected local sockets, ordered and message-boundary-preserving **per connection**. This does **not** imply delivery across peer death, buffer exhaustion or disk failures. Every bounded message envelope contains protocol major/minor and mandatory feature flags, message type/payload length, source/producer and run ID, acquisition **clock-domain/epoch/boot generation**, per-source monotonic sequence, OD-001 canonical integer-ns acquisition stamp and quality/timing metadata, calibration/config revision, and acknowledgment/correlation ID where relevant. Strictly validate length, unknown required features and `MSG_TRUNC`; avoid raw C++/Eigen/ROS/GTSAM object memory and process-specific pointers. Protocol and MCAP/bundle serialization versions remain independent.

| Edge | Buffer/reliability contract |
| --- | --- |
| Acquisition→recorder | Separate bounded ordered raw event route with gap/sequence detection, calibration and source clock observations; overflow makes recording **INCOMPLETE or stopped**, never silently lossless |
| Acquisition→SLAM | Separate bounded nonblocking timestamped sensor route; queue overflow/stale inputs trigger reported gaps and OD-022 degraded confidence, not invented scans/IMU samples |
| SLAM localizer→graph | Bounded in-process immutable graph jobs; coalesce stale requests, solve asynchronously and validate revisions before atomic map pose correction |
| Graph/map worker→localizer | Pin OD-011/013 snapshots and graph pose versions; atomically publish **only complete consistent** corrections; never jump OD-006 continuous odom |
| SLAM→Web Lab | Rate-limited/latest-value-only display frames; drop stale visualization without slowing localizer |
| Web Lab→SLAM | Validated asynchronous commands with request ID, authorization, applied-config generation and explicit success/error; no blocking UI command on acquisition |

A successful `send` or `recv` reports **transport**, not recorder `DURABLE` persistence. If acknowledgments are implemented distinguish accepted-to-queue, written, and fsync/segment-complete durability. A receiver that sees a sequence gap must record it with source/epoch/session context. After restart, reconnect using new peer/run identity, negotiate compatible schema and **do not reuse old clock epochs or graph/map generation**. Recorder crashes/disk-full conditions create explicit incomplete segments unless complete continuity is independently proved. Separate process privilege/socket permissions and verify local peer credentials where appropriate (`SO_PEERCRED` Linux). Web-accessible control requires explicit authentication/scope and strict command/schema limits.

**Optional measured optimization:** For large **immutable** map/graph snapshots only, `memfd_create` with suitable Linux read-only sealing and `SCM_RIGHTS` descriptor passing can avoid bulk copies. Transfer file length/hash/revision/owner lifetime and enforce pin/RAM/FD ceilings. Never expose mutable shared pages as OD-011 immutable data. Ordinary bounded socket messages are the baseline; shared memory is not required for raw sensor or control paths.

### 11.3 Fixed workers, backpressure, restart and scheduling

| Inside SLAM | Initial worker policy | On overload |
| --- | --- | --- |
| IPC ingest / OD-006 trajectory | Short high-priority bounded ingestion and time ordering | Flag gaps, stale motion and degraded output; never wait for graph |
| OD-005 assembly / OD-004 deskew | Bounded sequential pipeline with OD-004 coverage gates | Reject/record stale or unsupported scans |
| OD-007 local scan matcher | Dedicated bounded job executor | Deadline/revision rejection, OD-008 quality reasons; odom continues |
| OD-013 graph and optional iSAM2 | Lower-priority separate bounded worker | Coalesce/defer, report graph age; immutable consistent results only |
| OD-011 map/cache and OD-012 save | Budgeted low-priority background work | Fall back to last coherent snapshot under staleness policy; report disk/cache failure |
| Snapshot publication | Tiny atomic versioned handoff | Drop obsolete graph/cache outputs |

Use **`SCHED_OTHER` normal Linux scheduling first**, tune based on Pi 5 p95/p99 time, thermal and CPU contention. Do not assume `SCHED_FIFO` is safe; only measure bounded acquisition boost with starvation watchdog and privileges if justified. Enforce **fixed worker counts, per-edge item-and-byte queue caps, short nonblocking enqueue, bounded in-flight jobs, cancellation and backpressure**. Accepted **OD-021** provides provisional capacities, age/drop precedence, pressure thresholds and MCAP sync/rotation targets; **B29** must calibrate them on the actual Pi and storage before any timing/durability guarantee. Timeouts for execution use host-steady clock, not pausable `/clock`.

**Failure recovery:** Each process has independent health, restart limits, socket/FD cleanup and session-generation checks. Acquisition restart creates source clock epoch/run discontinuities. SLAM restart cannot silently continue an old graph as if the session were identical; it requires a compatible OD-012 checkpoint or a declared new localizer session. Recorder restart rolls a segment/gap and explicitly audits incomplete recordings. A crashed Web Lab has no hardware access and can reconnect to latest coherent telemetry. Graceful shutdown must drain/finalize the recorder within a bounded policy or mark incomplete. Avoid hidden reliability claims: finite buffers cannot guarantee raw losslessness under unbounded disk stalls.

**Benchmark B24** tests process and IPC isolation; **B29** adds quantitative OD-021 queue, MCAP durability, replay salvage and UI resynchronization testing. B24: test four-process lifecycle and graceful/crash restarts; ordered `SOCK_SEQPACKET` versioned envelopes, `MSG_TRUNC`, `EAGAIN`, sequence gaps, clock epochs and authorization; disk full/slow recorder, stopped SLAM matcher, long graph batch, stale immutable snapshots and long-running map saves while synthetic 10 Hz LD19 + ~200 Hz gyro/wheel ingestion continues. Monitor p50/p95/p99 capture→local pose, input queue age/bytes, gap/incomplete counts, CPU/peak RSS, FDs, thermal/graph lag, false durable acknowledgments, stable odom and stale correction rejection. Optional memfd vs copying benchmark after profiling. Docker Catch2/CTest multi-process tests; no GitHub Actions.

**References:** [Linux unix(7)](https://man7.org/linux/man-pages/man7/unix.7.html) and [memfd_create(2)](https://man7.org/linux/man-pages/man2/memfd_create.2.html).

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
| IO/replay/IPC | MCAP, OD-012 map checkpoints and **OD-016 four-process seqpacket/versioned schema, restarts, packet gaps, disk-failure INCOMPLETE semantics**, deterministic replay |
| Adapters | ROS-free core build; optional ROS 2 frames/scan/maps/QoS; Web Lab real backend responses |
| Performance | Equivalent datasets on ARM64 and x86-64, capture-to-pose tail latency and resource use |

Implementation priority: **P0** toolchain/geometry/synthetic fixtures -> **P1** Pi-direct recorder/time sync (optional Pico firmware after measured need) -> **P2** motion/deskew -> **P3** Ceres/submaps -> **P4** GTSAM offline mapping -> **P5** frozen-map localization/recovery -> **P6** online SLAM. Web Lab can start as a synthetic replay viewer and expand without blocking estimation development.

## 13. Decision links and references

The companion [open design choices register](open-design-choices.md) contains **OD-001–OD-025** with priority, alternatives, evidence and decision gates. All 25 choices are now **Accepted in the decision register**; numerical budgets, calibration and implementation performance still require the named benchmarks.

External technical references:

- [ROS REP-103](https://reps.openrobotics.org/rep-0103/) · [ROS REP-105](https://reps.openrobotics.org/rep-0105/)
- [Ceres nonlinear least-squares residuals and robust losses](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html)
- [GTSAM iSAM2](https://borglab.github.io/gtsam/isam2/) · [MCAP](https://mcap.dev/)
