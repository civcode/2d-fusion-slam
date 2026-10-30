# 2D Fusion SLAM — Open Design Choices

**Status:** Active design-decision register v0.1  
**Date:** 2026-10-30  
**Related:** [Architecture specification](architecture-specification.md) · [Technical specification](technical-specification.md) · [Implementation contract](implementation-contract.md) · [Benchmark and dataset plan](benchmarks-and-datasets.md)

> A suggested approach is **not** an accepted decision. The architecture deliberately leaves the choices below open until their decision gates are satisfied. Maintain this register on `main`; no new branch/PR is required by this document.

## 1. How decisions are made and recorded

- **State:** `Open` (not resolved), `Under evaluation`, `Accepted`, `Superseded` or `Deferred`.
- **Priority:** `P0` = needed before foundational code; `P1` = needed for first functional mapping; `P2` = needed for first live localization; `P3` = online/secondary integration.
- **Owner:** unassigned initially; assign a real accountable reviewer/contributor only when known. Do not invent owners or dates.
- **Approval:** a decision is accepted only after discussing at least two credible alternatives, recording measurements/constraints, and documenting why the chosen approach is sufficient for the current milestone.
- **Experiment:** tests/measurements run in Docker on synthetic apartment/warehouse fixtures first, then real datasets/hardware where relevant. Report CPU architecture and calibration; no unmeasured performance promises.
- **Change discipline:** record accepted outcome, alternatives rejected, evidence links, scope of approved change, migration/rollback effect and date. A later choice may supersede an earlier one; do not silently overwrite a decision.
- **No hidden dependency decisions:** Boost remains restricted by the implementation contract. Its use, including transitive adoption through ROS, HTTP or C++ libraries, requires separate explicit case-by-case approval. An OD entry is not blanket Boost authorization.

### Settled constraints (not open for routine implementation)

C++20; CMake; Catch2 v3 and cxxopts via pinned FetchContent; Docker for builds/tests/runtime; ROS-independent native core with REP-103/105 coordinate conventions; Ceres geometric frontend and GTSAM backend; raw-data preservation; **OD-001 signed 64-bit nanosecond time with explicit clock domains and ROS 2 /clock integration**; offline mapping -> fixed-map localization -> online SLAM priority; no automatic GitHub Actions or Actions execution without explicit permission; no silent Boost adoption.

## 2. Priority and dependency overview

| ID | Subject | Priority | Affects | State |
| --- | --- | --- | --- | --- |
| OD-001 | Canonical timestamp API and units, ROS /clock support | P0 | All streams, serialization, simulation | **Accepted** (implementation/tests pending) |
| OD-002 | Pi host timestamps now; Pico custom firmware + sync later | P1 | Live acquisition, replay | **Accepted** (implementation/tests pending) |
| OD-003 | LD19 angle/speed reconstruction and calibrated Pi clock alignment | P1 | Deskew fidelity | **Accepted** (calibration pending) |
| OD-004 | Midpoint scan reference and piecewise SE(2) exponential trajectory | P1 | Deskew, optimization | **Accepted** (tests pending) |
| OD-005 | Angular-seam scan assembly, partial coverage and invalid returns | P1 | Frontend acceptance | **Accepted** (tests/tuning pending) |
| OD-006 | Minimal wheel/gyro integrator and replaceable kinematics | P1 | Odometry, bias and slip | **Accepted** (calibration/tests pending) |
| OD-007 | Truncated unsigned EDT, bilinear, per-beam Huber and odom prior | P1 | Ceres matching | **Accepted** (implementation/benchmarks pending) |
| OD-008 | Robust LiDAR-only information, rank gates and covariance calibration | P1 | Graph, localizer quality | **Accepted** (tests/calibration pending) |
| OD-009 | Covariance-aware local correlation and asynchronous branch-and-bound recovery | P2 | Tracking recovery | **Accepted** (benchmark tuning pending) |
| OD-010 | 5 cm log-odds occupancy, 90/180 overlap and immutable freezing | P1 | Mapping and graph | **Accepted** (tests/tuning pending) |
| OD-011 | Coherent immutable snapshots, tiled active EDT and frozen-cache reuse | P1 | Map/matcher memory | **Accepted** (baseline first; optimization pending) |
| OD-012 | Versioned JSON manifest, zstd binary submaps/graph and atomic generations | P1 | Reuse across modes | **Accepted** (schema/tests pending) |
| OD-013 | Keyframe+submap Pose2 graph; fused odom and correlation-safe factors | P1 | Backend accuracy | **Accepted** (implementation/tests pending) |
| OD-014 | Bounded offline refinement, selective re-deskew/remap and rollback | P1 | Mapping quality | **Accepted** (tests/tuning pending) |
| OD-015 | 0.20 m/5°/2 s motion keyframes; independent submap anchors; optional sparse bias knots | P1 | Graph size/accuracy | **Accepted** (tuning/tests pending) |
| OD-016 | Process/thread layout | P1 | Pi resource isolation | Open |
| OD-017 | Native CMake target/API decomposition | P0 | Implementation scaffold | Open |
| OD-018 | Loop candidate retrieval and verification | P1 | Offline mapping | Open |
| OD-019 | Online iSAM2 work budget/fixed lag | P3 | Onboard full SLAM | Open |
| OD-020 | Global relocalization algorithm | P2 | Kidnapped robot | Open |
| OD-021 | Queue sizes, backpressure and recording loss | P1 | Fault handling | Open |
| OD-022 | Localization state and publication gating | P2 | Downstream safety | Open |
| OD-023 | MCAP channels, schema and bag import | P1 | Recording/replay | Open |
| OD-024 | Replay checkpoints, seek and reproducibility | P1 | Benchmarks/Web Lab | Open |
| OD-025 | ROS 2 distro, QoS and adapter interface | P3 | Nav2/RViz integration | Open |

Recommended **review order**: OD-017 + OD-001 (buildable core), then OD-002/003/005/023/024 (recording), OD-004/006 (motion), OD-007/008/010/011 (frontend), OD-012/013/014/015/018 (offline mapping), OD-009 accepted coarse recovery, OD-020/022 (global hypotheses and tracking state), then OD-016/019/025 (full online/deployment). Some concurrency questions in OD-016/021 must be settled earlier if they block recorder implementation.

## 3. Sensor, timestamp and odometry decisions

### OD-001 — Canonical time representation and ROS 2 simulated clock

**State:** **Accepted** (architectural decision; implementation and validation pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of option A  
**Priority:** P0

**Decision:** Use **signed 64-bit integer nanoseconds** for every canonical native timestamp and duration. Convert device ticks, host steady-clock readings, imported bag stamps, Unix/system time and ROS time **explicitly at system boundaries**, carrying the source clock domain and conversion provenance. No ROS 2 types or runtime dependencies may appear in the C++20 core.

**Alternatives evaluated:** (A) signed 64-bit nanoseconds — **chosen**, precise per-beam timing and compatible with ROS nanosecond resolution; (B) signed microseconds — simpler range but discards sub-microsecond precision unnecessarily; (C) generic chrono time points for every source — useful in implementation internals but does not by itself describe hardware clocks, simulated time jumps or serialized epoch identity.

**Required native contract:**

- `TimeNs = std::int64_t` for the **representation**, wrapped in typed `Timestamp` / `DurationNs` APIs with overflow-checked arithmetic. Do not encode missing timestamps as zero.
- Explicit `ClockDomainId` and `TimeEpochId` (or equivalent run/epoch identity). Time comparisons, subtraction and interpolation require a **common mapped domain and epoch**; cross-domain/epoch operations without conversion are invalid. An integer timestamp alone is insufficient to establish temporal comparability.
- Production sensor-time estimation uses a **monotonic local estimation timeline**; source device time, ROS timestamps and host receive time remain separately stored for auditing. Source-to-canonical mappings carry offset/drift, validity window and uncertainty when estimated; acquisition time must not be silently replaced by arrival time.
- Wall/system clocks and `steady_clock` remain available at boundary adapters for diagnostics, deadlines and sensor timeout scheduling. **Use steady/wall monotonic time for CPU performance and real-time budgets**, never a pausable simulator clock.
- The chosen integer type does **not** imply that *every* source clock is monotonic: a ROS 2 simulated timeline may stop, repeat, jump forward/backward or be uninitialized. The core MUST represent **time discontinuity events**, invalidate cross-epoch interpolation, and reset/restore time-dependent estimator state and buffers appropriately.
- The core exposes a **ROS-neutral clock interface** or `ClockEvent`/injected time source, accepting native nanosecond ticks and source/epoch changes. It supports live steady time, deterministic dataset replay, and an **external controllable simulated clock**. A pause does not advance integration; stepping uses authoritative simulated acquisition timestamps.
- The optional **ROS 2 adapter** honors `use_sim_time` and receives `rosgraph_msgs/msg/Clock` on `/clock` through ROS mechanisms. It converts ROS stamps to native int64 ns with range checks; supplies clock readiness, pauses/updates, backward and significant forward jumps, and clock-source switching to the native clock interface. It also converts native timestamps to `builtin_interfaces/msg/Time` for ROS publishers without changing their acquisition-time meaning. Never double-apply simulation time or fabricate a monotonic ROS clock.
- `/clock` is the authority for simulation/replay **event time** when `use_sim_time=true`; there is no assumption that raw hardware clocks are numerically identical. If the ROS clock has not initialized, withhold timed estimation/global pose outputs rather than interpret zero time as a valid measurement. Switching simulation/system-time modes starts a new epoch and resets/remaps dependent state.
- On a backward jump, replay seek, clock-source switch or other detected discontinuity: emit a typed clock-change event, clear/reinitialize timestamp ordering, trajectory interpolation, scan assembly, filter integration, and ROS TF caches (adapter-owned), or restore an explicitly compatible replay checkpoint. On large forward jumps apply a documented gap/reset policy; exact thresholds are left to **OD-002/OD-024**.
- ROS wire timestamp conversion must handle ROS's seconds/nanoseconds fields **without floating-point precision loss**, enforce field range and normalization, and handle negative/native out-of-range values explicitly. The internal nanosecond count has no implicit Unix epoch.
- Store clock mode/domain/epoch, source-to-canonical mapping, uncertainty and detected jumps in MCAP/replay/run metadata so that simulation and live diagnostics remain reproducible.

**Required interface behaviors (illustrative, not implemented):**

~~~cpp
using TimeNs = std::int64_t;
struct CanonicalTime { TimeNs ns; ClockDomainId domain; TimeEpochId epoch; };
struct ClockEvent { ClockEventKind kind; CanonicalTime current; };
struct ClockSource {
  virtual ~ClockSource() = default;
  virtual ClockReading now() const = 0; // includes readiness, domain, epoch
};
// ROS adapter maps /clock updates + ROS message stamps to native readings/events.
~~~

**Implementation acceptance tests (still outstanding):**

1. Nanosecond exactness, signed overflow/underflow, ROS `sec/nanosec` range normalization and nanosecond-perfect round-trip (including negative cases when representable).
2. Reject direct comparisons/differences for mismatched clock domains or epochs; validate explicit offset/drift conversion and uncertainty records.
3. No `/clock` yet / `use_sim_time` activated: clearly uninitialized, no valid global pose; first valid clock tick transitions to ready.
4. Pause/resume: no integration progress on a stopped simulation clock, no spurious CPU deadlines; step-by-scan updates exactly once.
5. Backward seek, large forward jump, source-mode switch, clock reset: emit event and flush/restore motion, deskew, queue and TF state; no negative integration intervals.
6. Mixed sensor stamps and imported ROS bag replay stay aligned to the authoritative acquisition timeline; hardware-only mode still operates without ROS.
7. Verify in a **dedicated ROS 2 Docker test image** with `/clock` publisher and `use_sim_time` consumer, plus **ROS-free Catch2 core tests** and deterministic synthetic replay. Document any supported distribution-specific jump API behavior.

**Consequences:** The accepted OD-001 time representation works with OD-002 sensor timestamping, OD-003 beam-time reconstruction and **accepted OD-004 midpoint SE(2) interpolation with gated 10 ms extrapolation**. Physical calibration and replay checkpoint policy (OD-024) remain pending. No implementation, measured timing results or passing tests are claimed.

**ROS 2 design reference:** [ROS 2 Clock and Time design](https://design.ros2.org/articles/clock_and_time.html).


### OD-002 — Sensor timestamping / clock synchronization

**State:** **Accepted** (architectural choice; implementation, calibration and measured results pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of initial approach A and future Pico approach B  
**Priority:** P1

**Decision:** Implement **A first**: timestamp LiDAR packet reception, MPU6050 sampling and wheel-encoder acquisition as close to the event as reasonably possible on the **Raspberry Pi host**, with an explicit host-monotonic clock domain, measured latency estimates, uncertainty and diagnostics. Architect **B as an optional future acquisition backend**: a **Raspberry Pi Pico (RP2040 initially; Pico model remains configurable)** running **custom Pico SDK firmware**, capturing IMU/encoder events locally and sending framed timestamped measurements to the Pi with a calibrated MCU-to-host clock mapping. **No micro-ROS or ROS client/middleware stack on the Pico, and no micro-ROS dependency in the host or core.**

**Alternatives:** (A) Raspberry Pi-only acquisition timestamping — **adopted first**, simplest working recorder and least firmware complexity; (B) Pico hardware-near capture with cross-device synchronization — **approved planned extension**, useful if measured host I/O jitter compromises quality; (C) hardware-triggered shared clock / PPS — not selected, possible later if hardware and accuracy requirements justify it.

**Stage A — Raspberry Pi-only initial implementation:**

- Capture host **monotonic** nanoseconds near an actual sampling boundary: GPIO encoder edge interrupt/counter read, IMU data-ready/FIFO or I²C read, and LD19 UART packet reception. Prefer an acquisition event timestamp when the driver provides a trustworthy source; otherwise bracket the read and identify which event was timestamped.
- Keep the distinction between **device event, sensor conversion, host read, serial arrival, packet completion and derived beam time**. A host timestamp at the end of a UART read is **not** automatically the instant a LiDAR beam was acquired.
- Preserve raw LD19 47-byte frames/device tick, raw IMU values, wheel ticks, host timestamps, sequence/drop/error counts and the timing-estimation method. Do not synthesize exact per-beam times from a coarse receive timestamp.
- Measure I²C scheduling jitter, encoder interrupt latency, serial buffering, receive timestamp dispersion and gaps on Raspberry Pi 5 before setting fixed offsets or claiming deskew accuracy.
- Record clock quality, uncertainty and provenance even if Stage A can only supply a conservative arrival-time proxy. Raw data recording MUST work without a Pico connected.

**Stage B — optional custom Pico acquisition backend:**

- Use the **Raspberry Pi Pico SDK** in a standalone, version-pinned firmware project (C/C++ with a compatible embedded toolchain). No ROS code, DDS, `micro-ROS`, `rclc`, ROS message headers, ROS transport, or `rclcpp` on the Pico. Firmware should remain usable independently of ROS and of a C++20-capable embedded standard library.
- Capture **encoder transitions/counts** and **MPU6050 sample/data-ready events** near their actual occurrence; retain both hardware/sample timestamps and value-read/completion timestamps where meaningful. Consider Pico sampling/reading the IMU over I²C and encoders via GPIO/interrupts; use the LiDAR through either Pi UART or Pico UART only if the wiring/throughput/timing benefit is measured.
- The RP2040 SDK exposes a **64-bit microsecond-since-boot hardware timer** (e.g., `time_us_64()`), not a native globally synchronized nanosecond clock. Preserve a **Pico boot-relative 64-bit microsecond tick**, convert to signed nanoseconds on the host with checked multiplication and source-domain metadata. **Nanosecond representation does not imply nanosecond measurement precision.**
- **PIO is conditional.** Evaluate programmable I/O for deterministic encoder edge counting/decoding or UART/event timing if better than a conventional IRQ/timer approach. A PIO state machine alone does **not** automatically stamp a GPIO edge with the RP2040's 64-bit hardware timer; a timestamped-edge design must explicitly specify how PIO cycle counts, FIFO/DMA or CPU IRQ timing are correlated with a clock, and measure actual accuracy, PIO state machine/FIFO overflow, CPU load and pin usage. Keep a simpler IRQ/timer fallback. PIO capture is **not** a mandatory initial feature or a claim of hardware-grade nanosecond precision.
- Pico messages MUST use a **small documented versioned binary/framed protocol** over a simple transport (initial candidate **USB CDC**, optionally UART after testing). Include protocol version, message type, payload length, sequence number, Pico boot/session ID, source timestamp/tick units, measurement fields, validity/status and an integrity check (CRC or equivalent). Reject corruption, unknown versions, invalid lengths and duplicated/out-of-order messages. **No ROS serialization or micro-ROS transport**. Final transport/framing/CRC selection requires bench validation.
- Design a **bidirectional time-sync exchange** initiated by the Pi: retain host send `t1`, Pico receipt `t2`, Pico transmit `t3` and Pi receive `t4` in their original domains; estimate affine clock mapping `t_host_ns ≈ a * t_pico_us + b`, drift, link delay/asymmetry and mapping uncertainty using repeated samples. Select low-RTT exchanges and reject outliers; do not assume USB/UART latency symmetry. Store mapping version and validity window. No external NTP/PTP/micro-ROS dependency is required.
- Detect **Pico reset/boot ID change**, host reboot, USB disconnect/reconnect, sequence gaps, timer anomalies, clock-map staleness and rollover. Such events require a **new source epoch** and recalibration/uncertainty propagation; never silently continue interpreting old Pico ticks on a new boot.
- Under **ROS `use_sim_time` / `/clock`**, the host remains responsible for converting Pico hardware timestamps to the active estimation timeline when appropriate. **Never slave the Pico's physical timer directly to a pausable/jumping ROS clock.** The native ROS-independent OD-001 clock interface handles simulated time jumps; the Pico sync model describes physical capture timing only.

**Dataflow and representation:**

~~~text
Stage A (initial):
  LD19 UART / MPU6050 I2C / encoder GPIO
           -> Pi driver near-event timestamps + raw values
           -> source-domain/time-quality tags -> native int64 ns -> recorder/estimator

Stage B (optional):
  encoder edge + MPU6050 sample -> Pico firmware hardware/PIO/IRQ capture
                                   -> Pico boot-relative microsecond ticks
                                   -> framed USB CDC or UART transport
  Pi host time-sync t1,t4 <----> Pico sync reply t2,t3
           -> offset + drift + uncertainty model
           -> clock-domain/epoch conversion to canonical int64 ns
           -> same recorder/estimator API as Stage A

  LD19 may stay on Pi UART; its accepted OD-003 timing model still requires physical reference-delay calibration.
~~~

**Acceptance criteria (not yet validated):**

1. **Stage A:** Dockerized ROS-free recording works from Pi-only sensors; synthetic/controlled tests quantify and log read/receive jitter, event/arrival differences, packet loss and timestamp quality.
2. **Common contract:** Switch between Pi and simulated Pico timestamp sources without changing motion/deskew APIs; cross-domain/epoch comparisons are rejected without explicit clock mapping (OD-001).
3. **Firmware/protocol:** Bench-test raw tick fidelity, CRC, version rejection, packet loss/out-of-order handling, safe reconnect/reboot, fixed-memory operation and timestamp provenance; preserve undecoded frames when diagnosing transport errors.
4. **Synchronization:** Inject known offsets/drift/jitter/asymmetry; verify mapping parameter recovery and realistic uncertainty. Test dropped sync replies, stale models, NTP-like round-trip outliers and host/Pico reboot.
5. **PIO feasibility:** Compare IRQ/timer vs PIO-assisted encoder capture under load, using an independent timing reference when possible; adopt PIO **only when accuracy, load and complexity justify it**.
6. **Hardware:** Validate MPU6050 sample timing and LD19 packet timing separately; **Pico synchronization is not a substitute for characterizing the LD19 device clock/beam model**.
7. **Dependency isolation:** Native core builds without Pico SDK/ROS; Pico firmware builds with its pinned SDK in a dedicated Docker toolchain; ROS `/clock` tests use the optional ROS adapter only; verify **no micro-ROS dependency**.
8. Report end-to-end acquisition-to-recording and acquisition-to-pose latency, clock offset/drift/uncertainty, encoder loss, gyro sample loss and link jitter on the actual Pi + Pico hardware before claiming Stage B superiority.

**Consequences:** Pi-only acquisition is the first deliverable. A future Pico firmware image and host driver must be modular and optional; firmware can evolve independently from the SLAM core. **Still open:** empirical LD19 physical reference-delay calibration under accepted **OD-003**, buffering and queue policy **OD-021**, MCAP schema **OD-023**, and exact Stage B transport/sync algorithm parameters as implementation-level choices requiring evidence (not permission to add micro-ROS).

**Hardware references:** [Raspberry Pi Pico SDK timer APIs](https://www.raspberrypi.com/documentation/pico-sdk/high_level.html), [hardware timer](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_timer/include/hardware/timer.h), and [PIO API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html). These establish available primitives, **not** verified measurement accuracy for the proposed application.

### OD-003 — LD19 angle/speed per-beam timing and clock alignment

**State:** **Accepted** (architecture selected; implementation and physical calibration pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of angle/speed reconstruction and calibrated Pi clock alignment  
**Priority:** P1

**Decision:** Reconstruct each LD19 beam's **relative acquisition time** using the packet's start/end angles and reported motor speed. Unwrap the LD19's nominal **30-second millisecond counter**; estimate source-to-Pi **monotonic clock offset and drift** from UART reception timestamps. Treat the actual physical event referenced by the packet timestamp as **unknown until measured or bounded experimentally**. Implement the entire timing reconstruction and clock alignment within the **native LD19 acquisition adapter**; return ordered beams with canonical signed 64-bit nanosecond timestamps and uncertainty to the portable SLAM core, **without ROS dependencies**.

**Alternatives:** (A) angle/speed per-beam reconstruction — **chosen**, retaining within-scan motion and speed variation; (B) constant 10 Hz / fixed beam spacing — not accepted as the primary model; (C) one packet/scan receive timestamp for all beams — rejected, conceals UART delay and motion distortion. Optional Pico capture under OD-002 is not needed initially and does not replace LD19 source-clock reconstruction.

### Packet geometry and relative time

LD19 v2.5 documentation specifies 12 samples per 47-byte UART packet, motor speed in **degrees per second**, start/end angles in **0.01 degrees**, a millisecond packet counter resetting at approximately **30,000 ms**, and a 230400-baud serial interface. Decode only CRC-valid packets. Unwrap angles consistently across 0/360 and validate the **device sweep direction independently** of the native CCW-positive Cartesian convention. For beam slot `i` in a packet of `N` points, derive:

~~~text
theta_i = theta_start_unwrapped
        + i/(N-1) * (theta_end_unwrapped - theta_start_unwrapped)

delta_t_i_anchor ≈ (theta_i - theta_anchor) / omega_signed   [seconds]

t_device_beam_i ≈ t_device_packet_unwrapped
                + delta_t_i_anchor + delta_physical_anchor
~~~

- `omega_signed` is the packet speed converted from degrees/s with verified rotation direction. Validate measured angle span against speed and neighboring packet times; handle small/invalid speed, inconsistent angle coverage, packet loss and motor-speed variation conservatively.
- `theta_anchor` is an **arbitrary documented computational reference**, e.g. the packet end angle. Choosing it **does not assert** the device timestamp marks that beam; it merely sets the origin for **relative** beam offsets. The **unknown physical acquisition-to-packet timestamp offset** is captured by a separate calibrated/bounded `delta_physical_anchor`. Use one explicit anchor and one offset, not two independently free reference parameters.
- A relative beam-time model can be computed before absolute packet-reference calibration; tag its **absolute** times as uncertain until a valid clock mapping and physical offset are available.
- The scan reference time used for deskew (`OD-004`) is separate from the per-beam acquisition-time reconstruction.

### Device clock and host alignment

- Retain the **raw 16-bit packet tick and bytes**, unwrap approximately **30-second rollover** into a wider epoch-local count, then convert milliseconds to the OD-001 signed nanosecond representation using overflow-checked arithmetic.
- Detect wrap using packet order and observed host elapsed time; distinguish rollover, packet corruption, reordering and sensor reset. If an outage may hide multiple 30-second wraps, treat the unwrapped count as **ambiguous**, invalidate the model and begin a new source epoch. Do not invent a wrap count.
- Capture a Pi host-monotonic timestamp as near as practical to **actual UART reception**, identifying whether it corresponds to first/last byte, read completion or packet completion; keep serial transmission time and host buffering separate from acquisition time.
- Fit a robust **affine** source-clock mapping over many packets, recording drift, alignment offset, residual jitter, uncertainty, validity window, version and relock state. For example:

~~~text
t_host_rx_ns ≈ a * (t_ld19_unwrapped_ms * 1_000_000) + b + d_uart_rx
t_beam_host_ns ≈ a * t_device_beam_ns + b + calibrated_delay_adjustment
~~~

- **Fundamental identifiability limit:** receiver-only **one-way UART** observations cannot separate a constant source-clock epoch offset from a constant sensor-processing/UART acquisition-to-reception delay. Thus `b + d_uart_rx` is initially a **combined offset**. Drift and variable delay can be estimated/characterized under suitable data conditions, while physical delay must be measured externally or bounded. Never claim physically exact acquisition timestamps solely from a receive-time regression.
- Account for the device counter's 1 ms granularity, UART serialization latency, driver/USB buffering, host receive jitter, oscillator drift, unknown physical acquisition delay and shared systematic error across packet beams. Record their individual estimates/upper bounds as available; correlation must not be represented as independent per-beam noise.

### Native adapter interface and persistence

~~~text
LD19 UART + Pi monotonic receive time
  -> packet sync / CRC / decode
  -> angle unwrap + speed-based per-beam relative times
  -> 30-second device tick unwrap and source epoch
  -> LD19-to-Pi affine mapping + physical delay calibration/uncertainty
  -> canonical int64 nanoseconds, domain/epoch, timing quality, sigma
  -> ordered native TimedScan -> portable deskew / MCAP / Web Lab
~~~

The acquisition adapter must preserve raw device tick, original packet bytes, start/end angles, speed, receive timestamps, model/calibration revision, and **per-beam relative and estimated canonical times**. Output must distinguish `estimated`, `calibrated`, `uncertain` and `invalid` timing status. If the model is not trustworthy, keep recording but allow downstream scan rejection/downweighting. The portable core consumes a generic ROS-free timed scan; **LD19 packet parsing and device-specific clock alignment stay behind the adapter boundary**. Reprocessing raw recordings with a revised clock fit must be possible.

### Validation and acceptance

1. **Synthetic timing:** N=12 interpolation, nominal/variable speed, angle seam, sensor rotation sign, known relative beam times and motor anomalies; compare against exact synthetic acquisition events.
2. **Counter faults:** 29,999→0 and device-specific rollover boundary, CRC failures, out-of-order packets, host pauses ≥30 s, multiple-wrap ambiguity, reboot/reconnect and source epoch changes.
3. **Clock fit:** injected offset, ppm drift, 1 ms quantization, realistic serial/USB scheduling jitter, delayed reads, outliers and stale model; verify parameter recovery and **combined-offset uncertainty**, not fictitious separation of clock offset and fixed transport delay.
4. **Physical reference calibration:** controlled stationary/rotating/translation experiments (optionally an independent timing or geometric reference) establish **or bound** the delay between actual beam acquisition and the reported packet timestamp. Test across rotation rates and buffering conditions.
5. **Deskew comparison:** no deskew vs fixed spacing vs **angle/speed timing**, on synthetic apartment/warehouse and recorded LD19; record endpoint error (where ground truth exists), residuals, quality flags, p50/p95 jitter, clock drift and Pi performance.
6. **Boundaries:** deterministic Docker tests of native adapter and replay; ROS-free core; no cross-epoch interpolation; malformed data must fail cleanly and retain raw debugging evidence.
7. No default assumption that the packet timestamp is the first beam, last beam or packet transmit-end time until supported by calibration evidence.

**Consequences:** Relative beam interpolation and calibrated host alignment architecture are **settled**. Exact physical timestamp reference, calibrated delay, clock-model fit parameters, quality thresholds and numerical uncertainty remain **empirical implementation/calibration work**. Accepted OD-004 sets the temporal midpoint deskew reference time, OD-002 the Pi/Pico acquisition transport and OD-023 the raw/derived timestamp storage schema.

**References:** [LD19 Development Manual v2.5](https://botland.de/img/art/inne/21991_Instrukcja%20rozbudowy.pdf), [technical protocol specification](technical-specification.md), [architecture specification](architecture-specification.md).

### OD-004 — Midpoint scan reference and SE(2) exponential trajectory

**State:** **Accepted** (architecture chosen; implementation/benchmarks pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of temporal midpoint, piecewise-constant body twist, and gated 10 ms horizon  
**Priority:** P1

**Decision:** Use a **time-indexed SE(2) trajectory** reconstructed from synchronized encoder and gyro/IMU readings. Model **piecewise-constant body-frame twist**, integrated and evaluated with the **SE(2) exponential map**, and deskew each LD19 point into the **laser frame at the scan's temporal midpoint**. Normal map insertion requires **fully interpolated trajectory coverage of the entire scan**. Any extrapolation is opt-in, at most **10 ms initially**, uncertainty/quality gated and explicitly labeled; prefer short delayed processing to unconstrained prediction. The trajectory interface is ROS-neutral and sensor-independent. A spline trajectory is a future interchangeable implementation only.

**Alternatives:** Midpoint/exponential (accepted) vs scan-end referencing (not selected), elementwise linear position/yaw interpolation (not selected as the motion model), or continuous SE(2) splines (deferred until benchmark evidence). A midpoint is **a time**, not the angular midpoint of a scan, a packet boundary or the time of HTTP delivery.

### Reference time and scan eligibility

- Define scan bounds from **minimum and maximum time-valid acquired beam-slot timestamps** after OD-003 reconstruction, including acquisition slots whose *range* is invalid/filtered. This prevents geometric range filtering from silently shrinking the acquisition window.
- Require the packet/scan assembler to report actual temporal coverage and full-scan/partial-scan status. Missing or ambiguous timing boundaries make a scan **ineligible for ordinary map insertion**.
- Compute `t_mid = t_start + floor((t_end - t_start)/2)` with checked **signed 64-bit nanosecond** arithmetic, in one **OD-001 clock domain and epoch**. Preserve the scan interval, midpoint, time-quality and calibration IDs in the deskewed result.
- The output pose and scan-matching reference transform MUST be timestamped at `t_mid`. Source beam timestamps are immutable and must not be shifted to pretend the beam happened at the midpoint.

### Continuous piecewise-constant body twist

Build velocity segments from actual timed wheel translation and **bias-corrected** IMU yaw rate, not a fixed-rate array. The body-frame twist `xi_k=(v_x,v_y,omega)_k` may initially set `v_y=0` for the nonholonomic drive model, with kinematics explicitly supplied by the motion estimator (OD-006).

For any valid segment `[t_k,t_{k+1}]`, interpolate the body-to-odom transform with the Lie-group exponential:

~~~text
T_odom_base(t) = T_odom_base(t_k) * Exp_SE2((t - t_k) * hat(xi_k))
T_odom_base(t_{k+1}) =
    T_odom_base(t_k) * Exp_SE2((t_{k+1} - t_k) * hat(xi_k))
~~~

Time in the exponential must be **seconds**, converted precisely from nanosecond differences. Implement the closed-form constant-curvature translation with a numerically stable small-angle limit. Do **not** interpolate x/y/yaw independently or interpolate across untrusted gyro/wheel gaps and epoch transitions.

Deskew every beam `p_i` into the **midpoint laser frame**:

~~~text
p_laser_mid =
    inverse(T_odom_base(t_mid) * T_base_laser)
  * (T_odom_base(t_i) * T_base_laser) * p_laser_i
~~~

Use the **continuous local odom frame**, not `map` (which may jump after loop correction). Account for the calibrated LiDAR lever arm and preserve interpolation covariance.

### Coverage, delay and optional extrapolation

- **Normal mapping** MUST verify that the timestamp-addressable trajectory covers `[t_start,t_end]` including **every timestamp-valid beam** and `t_mid` via valid interpolation. A scan containing any extrapolated pose, invalid gap or mismatched clock epoch is **not inserted** in an ordinary occupancy submap.
- **Prefer delayed processing:** Buffer scans briefly until sufficient IMU/wheel readings have arrived to interpolate to `t_end`, using a **bounded queue** with a configurable host-steady-clock wait/deadline. Report failure/defer/drop when no trustworthy continuation arrives; never block hardware acquisition.
- **Optional extrapolation** is allowed solely for explicitly configured diagnostic/live prediction or localization uses, **disabled for standard map insertion**. Initial horizon ceiling: **10,000,000 ns = 10 ms** measured outside a valid trajectory boundary (forward or backward). A smaller configured limit may apply. Gating additionally considers covariance, beam-time uncertainty, wheel/gyro freshness, gyro bias, slip, speed and valid calibration. A valid horizon **does not by itself authorize extrapolation**.
- Report distinct `INTERPOLATED` / `EXTRAPOLATED` / `UNAVAILABLE` results. Extrapolated estimates include direction/duration, uncertainty growth, quality reason, trajectory and clock revision; cannot silently be used as high-confidence scan/graph evidence. For a low-quality interval, `UNAVAILABLE` is the correct result even if it falls within 10 ms.
- Segment sample-gap thresholds, noise and covariance propagation, queue latency limits, and acceptance gates remain implementation tuning under OD-006/OD-008/OD-021; this decision does not set arbitrary precision targets.

### ROS- and sensor-independent trajectory provider

The portable SLAM core MUST query an interface supporting timestamp-addressable **pose/uncertainty**, **coverage intervals**, and **interpolation/extrapolation status**. Neither this interface nor the deskewer may depend on LD19 packet layouts, MPU6050, encoders, Pico, ROS/TF, or device drivers. Device-specific acquisition feeds a motion estimator that **implements** the trajectory contract; a future spline trajectory provider implements the **same** contract.

Illustrative, **not implemented**:

~~~cpp
struct TrajectoryQuery {
  CanonicalTime stamp;                    // nanoseconds, domain and epoch
  bool allow_extrapolation = false;
  std::int64_t max_extrapolation_ns = 10'000'000;
};
enum class TrajectoryStatus { INTERPOLATED, EXTRAPOLATED, UNAVAILABLE };
struct TrajectorySample {
  Pose2 T_odom_base;
  Matrix3 covariance;                   // with defined frame/tangent convention
  TrajectoryStatus status;
  std::int64_t extrapolation_ns;
  RevisionId trajectory_revision;
};
class TrajectoryProvider {
public:
  virtual ~TrajectoryProvider() = default;
  virtual CoverageResult coverage(CanonicalTime start, CanonicalTime end) const = 0;
  virtual TrajectoryResult evaluate(const TrajectoryQuery&) const = 0;
};
~~~

### Acceptance tests (not yet executed)

1. Pure translation, in-place rotation, constant-curvature arc, small-angle numerical stability, yaw wrapping and off-center LiDAR lever arm; compare SE(2) exponential against analytic truth.
2. Nonuniform LD19 beam times, scan duration with odd-nanosecond midpoint, invalid range values with valid acquisition slots, and partial/ambiguous scan windows.
3. Synthetic apartment/warehouse truth for every beam using gyro + wheel SE(2) with several asynchronous sensor sampling schedules.
4. Valid interpolation over entire scan admits mapping; incomplete coverage, uninitialized clock, missing gyro/wheel data, cross-epoch time, clock jumps and unsupported gaps defer/reject map insertion.
5. Extrapolation at **9.9 ms** and **10 ms** may succeed only if gates pass; **10.1 ms** MUST fail. Covariance grows, status is `EXTRAPOLATED`, and normal mapping never inserts an extrapolated scan.
6. Compare delay-until-covered with optional bounded extrapolation for p95/p99 latency/accuracy on Pi and deterministic replay; prove that a mock/spline trajectory backend can replace the motion estimator without altering deskew code.

**Consequences:** The temporal midpoint, constant body-twist SE(2) exponential, complete interpolation for map insertion, and quality-gated 10 ms maximum optional extrapolation are **settled**. Accepted OD-006 defines the minimal wheel/gyro integrator and bias/uncertainty policy; interpolation gap threshold, queue wait policy (OD-021), and offline re-deskew iteration (OD-014) remain to be evaluated.

### OD-005 — Angular-seam revolution assembly, partial scans and return validity

**State:** **Accepted** (architecture chosen; implementation, threshold tuning and tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of seam-based assembly, partial-scan support and validity gates  
**Priority:** P1

**Decision:** Assemble **LD19 revolutions by crossings of a fixed, configured angular seam** using **unwrapped, time-ordered beam angles** and the known sensor rotation direction. **Partial scans are supported** as honest observations carrying explicit **angular coverage, missing-sector, acquisition-time and return-quality metadata**; do **not** synthesize missing beams, ranges or timestamps. Reject malformed LD19 packets at the adapter boundary. Never convert invalid/unknown/missing measurements into free-space map updates. Apply configurable and independently recorded **matching and map-insertion quality gates**. **Timeouts are for stalled/disrupted acquisition recovery only**, not the primary segmentation rule. Keep device decoding in the LD19 adapter; the portable core handles generic timed beam/scan records without ROS dependencies.

**Alternatives:** (A) only complete 360° revolutions — rejected because packet loss would discard otherwise useful partial observations; (B) **fixed angular seam with partial scans and quality flags — accepted**; (C) fixed wall-time / arbitrary sample-count segmentation — rejected as normal scan segmentation because motor speed varies and it can change scan geometry/deskew reference unpredictably.

### Revolution boundaries and angle unwrapping

1. Decode and CRC-check LD19 packets **before** passing point observations to a scan assembler. Follow **accepted OD-003** for signed sweep direction, unwrapped packet/beam angles and canonical signed-64-bit-nanosecond acquisition timestamps. Preserve original packet/beam order; the assembler must not assume exactly 450 points or a fixed 100 ms duration.
2. Select a **fixed configurable seam angle**, with a sensible initial example of 0° in the normalized laser angular convention. Express the crossing using an unwrapped **rotation-phase coordinate** that increases in the verified rotation direction. A boundary is a crossing of `seam + k·2π` (or the equivalent 360° phase), **not** a UART packet boundary or a timer event.
3. On an observed, credible seam crossing, close the preceding revolution and start the next. A packet may straddle the seam: **partition its decoded beams at the crossing**, preserving each beam's actual angle, time, intensity and validity. Do not duplicate the seam beam or invent a synthetic point exactly on the seam.
4. A session may start or stop partway through a revolution. These are **partial scans** with their actual angular/time intervals and explicit start/end reasons; do not mark them complete. Handle reversals, resets, angle discontinuities, multiple potential seam crossings inside missing data, and epoch changes with resynchronization/partial-quality flags rather than silently inferring complete revolutions.
5. A large angle jump might span the chosen seam **without observable beam support**. Preserve the gap and a **seam-confidence** indicator; if the revolution assignment is ambiguous (e.g., unknown number of turns), terminate/discard the affected assembly with a clear reason and re-lock on subsequent trustworthy crossings. No fabricated revolutions.

### Beam representation and missing-sector metadata

The generic **ROS-free** assembled scan should provide:

- Canonical acquisition timestamps for each **observed beam slot**, original order, angle, optional range/intensity, return classification and timing quality; carry clock-domain/epoch and calibration/model IDs from OD-001/003.
- Explicit **scan acquisition interval** (`t_start`, `t_end`), angular seam/rotation direction, start/end boundary status, complete/partial status and assembly termination reason.
- **Coverage metadata** measured against the **full configured 360° angular domain**, not merely the min/max observed angle spread: observed angular support (possibly represented by configurable bins or intervals), **missing-sector intervals**, largest angular gap, total coverage fraction and sample/valid-return counts. Coverage resolution and the support model must be specified, tested and consistent across datasets.
- Separate counters for CRC-rejected packets, lost/gapped beam slots when detectable, angle/time discontinuities, invalid ranges, explicit valid no-return/max-range measurements and accepted obstacle hits. Do not represent unobserved sectors as beams with range=0 or range=max.

**Crucial distinction:** An **observed but invalid range** is not a missing beam. It may retain a valid *acquisition timestamp* needed for OD-004 temporal scan bounds even though it contributes no endpoint or free-space ray. A **missing packet/sector** has no trustworthy measurement/time for those nonexistent beams. Do not interpolate synthetic timestamps across unknown sectors for either matching or mapping.

### Return classifications and safe occupancy integration

Use a typed return classification, at minimum:

| Return | Meaning | Scan matching | Occupancy update |
| --- | --- | --- | --- |
| `VALID_HIT` | Finite, calibrated in-range obstacle measurement | Candidate beam if other quality gates pass | **Free cells strictly before observed hit; occupied cell at endpoint**, using validated ray model |
| `VALID_NO_RETURN` | Device explicitly signals a trusted no-hit / max-range ray with documented semantics | Usually no obstacle endpoint | **Optional** free-space update only through an explicitly validated bounded ray and configured max usable range |
| `INVALID_RETURN` | Corrupt/saturated/zero/out-of-range/unsupported value without trustworthy no-hit semantics | Exclude | **No free-space and no occupied update** |
| `MISSING` | No observed beam (packet lost/sector absent) | No sample | **No update**; metadata only |

The first implementation MUST **conservatively treat undocumented zero-range, out-of-range and missing points as invalid**, not reinterpret them as max-range clear readings. Intensity is retained (with explicit missing/invalid status) and MAY assist filtering, but must not silently change occupancy semantics. Never raytrace through missing sectors or assume that an invalid hit proves empty space.

A valid hit still requires correct calibration, pose quality and deskew coverage before occupancy insertion. A valid no-return may be used for freespace **only if sensor meaning is documented and configuration explicitly allows it**; otherwise do not update anything.

### Scan-quality gates and timeouts

- Use separate **scan-matching eligibility** and **map-insertion eligibility** decisions. Both must inspect **measured angular coverage**, largest uncovered sector, minimum number/distribution of `VALID_HIT` returns, valid timestamp fraction, packet/time/angle continuity, quality of clock alignment, and motion trajectory coverage. Expose typed rejection reasons and actual measurements.
- Coverage/quality thresholds MUST be configurable, versioned and included in diagnostics/benchmark reports. **Do not hardcode or claim universal numerical threshold values** until synthetic and real LD19 tests establish suitable bounds. A valid partial scan MAY be used for matching and mapping if it passes its respective gates.
- **Accepted OD-004 still applies to partial scans:** midpoint is the temporal midpoint of the **observed, time-valid scan slots**, and **normal map insertion requires valid interpolated trajectory coverage for that entire actual acquisition interval**. It does **not** require imaginary poses for unobserved beams; it MUST reject extrapolated poses, ambiguous timing boundaries and untrusted gaps in the actual interval.
- Timeouts (configurable, measured on **host steady time**) only detect stopped motor, failed/blocked UART stream, missing data or incomplete assembly at shutdown. They MAY flush/abort the current scan as a **partial, explicitly timed-out** diagnostic record and reset segmentation. They MUST NOT cut scans at a periodic wall-time interval during normal acquisition or synthesize a seam timestamp.
- Only insert an accepted scan **after** matching against a map that does not already contain it. No dropped/corrupt measurement may create a free-space map update by side effect.

### Proposed module split

~~~text
Pi UART + LD19 packet CRC/decoder + OD-003 angle/time adapter
     -> generic ordered TimedBeam records (+ packet-loss diagnostics)
     -> angular-seam ScanAssembler (no LD19 packet fields required)
     -> AssembledScan {beams, coverage, gaps, quality, time window}
             |                   |
             |             diagnostics / raw MCAP / Web Lab
             v
       ScanEligibility checks
             |
     OD-004 midpoint deskew with trajectory provider
             |
       post-deskew quality verification
           /             \
       Ceres match    accepted map insertion
~~~

The **LD19 acquisition adapter** owns frame parsing, CRC, reported motor speed and device-specific time reconstruction. The **portable scan assembler** consumes generic timed beam samples plus rotation/seam metadata and optional gap notifications. The estimator, deskew, occupancy and ROS 2 adapters MUST NOT parse LD19 packet layouts, import LD19-specific types or fabricate absent beams. A ROS `LaserScan` export must respect its regular-angle/time assumptions; if nonuniform partial scans cannot be faithfully represented, use a point cloud or explicit scan schema instead.

### Validation and acceptance (tests not yet run)

1. Variable RPM, clockwise/raw versus CCW/native angle conventions, 0° seam and nonzero configured seams, multiple revolutions, beam-on-seam, and **one packet straddling a boundary** with no duplicated/dropped valid beam.
2. Starting/stopping mid-revolution, packet loss crossing the seam, gaps with missing sectors, reordering, direction reversal, clock epoch reset, and gaps spanning **multiple** possible revolutions. No phantom points, scans or inferred timestamps.
3. Correct circular coverage computation (missing wedges vs min/max angular span), count consistency and return classification when all, none or only some beam ranges are valid.
4. CRC corruption, truncated 47-byte packet, invalid range/intensity, missing packet, and trustworthy versus untrustworthy max-range/no-return semantics; **zero invalid-return free-space updates**.
5. Matching/insertion gates for full and partial scans, low coverage, narrow field-of-view hits, poor timing, discontinuities and sparse valid-hit geometry. Reject with structured reasons and maintain raw-recording continuity.
6. Timeout under a stalled stream causes labeled partial flush/recovery; steady nominal revolutions are segmented **only by seam crossings**, never by the timeout interval.
7. Full OD-004 interpolation across the **actual observed** scan interval, including timestamp-valid slots with invalid ranges; missing time boundaries and extrapolated trajectory prevent ordinary insertion. Verify midpoint is not a mean beam time.
8. Deterministic synthetic apartment/warehouse replay, Docker Catch2/CTest tests, Web Lab overlay of observed/missing angular sectors and quality counters, and measured Pi CPU/latency.

**Consequences:** Angular-seam segmentation, honest partial scans, return validity and safe occupancy behavior, and recovery-only timeouts are **settled**. Exact coverage binning, thresholds, loss tolerance and timeout duration remain **configurable implementation/benchmark tuning**, not permission to invent missing samples. OD-003 owns per-beam time reconstruction; OD-004 owns midpoint trajectory deskew; OD-005 owns **observation grouping and scan eligibility**. OD-006 motion integration is accepted; OD-007 scan matching is accepted; OD-010 log-odds occupancy and freezing policy is accepted; OD-011 coherent snapshot and derived-cache architecture is accepted, with numeric performance tuning pending.

### OD-006 — Minimal wheel/gyro integrator and replaceable kinematics

**State:** **Accepted** (design selected; implementation, hardware verification and tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit choice  
**Priority:** P1

**Decision:** Initially use a **minimal wheel-odometry and bias-corrected Z-gyroscope integrator**, *not* an EKF or recent-state smoother. Maintain planar **SE(2) pose, body twist, gyro yaw-rate bias and uncertainty**, and provide the timestamp-addressable **piecewise-constant body-twist / SE(2) exponential trajectory** specified by accepted OD-004. Use a replaceable kinematic-model interface; **do not assume differential drive** until the first robot's wheel arrangement, encoder outputs and (where applicable) steering feedback have been established. Keep both the estimator and trajectory interfaces native C++20, independent of ROS and device-specific types.

**Alternatives:** (A) minimal direct integrator — **accepted** for simplicity, determinism, debugging and onboard cost; (B) EKF/error-state filter — deferred pending accuracy and consistency evidence; (C) fixed-lag/recent-state smoothing — deferred pending a demonstrated benefit and tolerable compute budget. A fallback with missing wheel or gyro information is explicitly **degraded**, not a fabricated complete sensor fusion result.

### Native motion state and interfaces

| State/output | Contract |
| --- | --- |
| `T_odom_base(t) ∈ SE(2)` | Continuous drift-prone local pose, no global map-correction jumps |
| `xi_body = (v_x,v_y,omega)` | Planar body twist in m/s, m/s, rad/s with measured-source validity |
| `b_g` | Gyro Z-rate bias in rad/s, calibrated with an uncertainty and revision |
| `P` | Pose/bias uncertainty and, if represented, twist uncertainty; explicit frame/tangent order and cross-correlations |
| `quality` | Sensor/time coverage, bias readiness, calibrated kinematics, slip/disagreement and degraded state |
| `trajectory` | OD-004 intervals, `Exp_SE2`, interpolation coverage, clock domain/epoch, covariance, revision |

A minimal error-state formulation with `(δx,δy,δyaw,δb_g)` and **pose–gyro-bias cross-covariance** is a suitable starting point; the precise covariance discretization must be verified, not assumed. Use OD-001 acquisition nanoseconds (converted to seconds for integration), OD-002/003 timing provenance and OD-004 trajectory queries. The trajectory provider consumes **generic wheel/motion and IMU measurements**, never direct MPU6050, encoder, Pico, LD19 or ROS types.

The estimator owns the mutable predictor state; deskew, Ceres, replay and ROS adapters use the stable `MotionEstimator` / `TrajectoryProvider` boundary. A later EKF or recent-state smoother MUST implement equivalent native output semantics, allowing replacement without modifying the SLAM frontend.

### Kinematic-model selection (hardware evidence required)

- Provide a native `KinematicModel` interface mapping **actual observed** wheel/steer telemetry into body translation, possible independent wheel yaw, quality/covariance and calibration identity.
- Differential drive may use `ds = (ds_R + ds_L)/2` and `dtheta_w = (ds_R - ds_L)/track_width` **only after independently driven wheel arrangement and measured left/right encoder information are confirmed**, with calibrated radii, signs and track width.
- Ackermann `omega_w = v*tan(delta)/wheelbase` is available **only** with valid wheel speed and **measured** steering angle/geometry. Commanded steering is not automatically a measurement.
- A distinct measured-forward-odometry source may be supported when raw ticks are unavailable, with its provenance and correlations recorded. An unsupported/unverified drivetrain MUST report unconfigured/degraded status, not silently select differential drive.
- Independent wheel-predicted yaw is optional; if unavailable, do not invent a disagreement signal.

### Integration, uncertainty and slip

Integrate body translation from wheel motion with yaw rate `omega_z - b_g` using **`Exp_SE2(xi_body * dt)`** over real timestamps. Use the exact constant-twist arc and stable small-angle limit. Propagate uncertainty from wheel measurement/scale, gyro noise, **gyro bias and its random walk**, timestamps, slip and kinematic mismatch. Preserve pose/bias covariance coupling (or demonstrate a conservative alternative), growing uncertainty through missing readings or unobservable heading.

When wheel-derived yaw is independently available, compare gyro and wheel relative yaw over the **same aligned interval**, using an uncertainty-aware consistency test. Persistent disagreement may indicate wheel slip, steering/scale errors or gyro problems; **raise covariance, mark degraded and log diagnostics**. It MUST NOT be treated automatically as a gyro-bias correction. If wheel/gyro evidence is missing, do not fabricate translation or zero yaw; report the unsupported trajectory interval and respect OD-004 full interpolation and gated 10 ms prediction rules.

### Stationary gyro-bias calibration and gated online corrections

- **Initialization:** estimate `b_g` and its uncertainty from a verified **stationary sample window**. Stationarity SHOULD be cross-checked using wheel-no-motion observations, low gyro variation and accelerometer/gravity information where available, not merely a single zero-rate reading. Record sample count, noise, duration, temperature if present and calibration revision.
- If the robot moves or the stationary condition is unproven, **do not claim a calibrated bias**. Expose uninitialized/uncertain state, increase yaw uncertainty and gate operations that require a trustworthy trajectory.
- **Online updates:** permit small, bounded, logged bias corrections only during independently validated stationary intervals or when a separately validated observable correction with proper uncertainty is available. Apply innovation/hysteresis checks; reject apparent bias caused by wheel slip or motion. Do not retroactively jump the published odom trajectory. Reintegrating raw history after offline recalibration produces a **new versioned trajectory** (OD-014).
- Temperature dependence and exact stationary thresholds remain empirical calibration/config choices, not hard-coded assumptions.

### Information accounting and validation

The fused predictor is used for **deskew and scan-matcher initialization**. If GTSAM also uses raw wheel and gyro factors, **do not insert this already-fused predicted pose as an additional independent prior**. Conditioning of LiDAR deskew on wheel/gyro creates correlation; carry conservative uncertainty and provenance under OD-008/013.

Tests required inside Docker using Catch2:

1. Exact SE(2) pure translation, rotation and arcs, asynchronous wheel/gyro samples, invalid clock epochs and OD-004 trajectory coverage.
2. Differential and Ackermann **synthetic fixtures with the corresponding configured measurements**; unsupported hardware and missing steering fail explicitly.
3. Stationary bias calibration with known drift/noise, moving startup, insufficient stationary evidence, recovery and bounded correction.
4. Slip/encoder-fault vs gyro disagreement on the same time window; confirm uncertainty inflation and **no spurious bias learning**.
5. Missing wheel/gyro data, growing yaw uncertainty, bias–pose covariance consistency with synthetic/Monte Carlo truth, and OD-004 interpolation/extrapolation gating.
6. Swap a mock alternative predictor behind the same provider interface; no ROS or device headers required; measure Pi 5 latency and memory.

**Consequences:** The **initial estimator architecture** is accepted; the actual drivetrain model and calibrated parameters are **still unverified hardware facts**, not silently chosen. Covariance tuning, stationary thresholds, slip gates, EKF/smoother feasibility and measured accuracy remain implementation/benchmark work.

## 4. Matching, mapping and graph decisions

### OD-007 — Truncated unsigned EDT scan matcher with independent Huber beams and odometry regularizer

**State:** **Accepted** (architecture chosen; implementation and quantitative validation pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of EDT, bilinear interpolation, per-beam Huber loss and uncertainty-weighted prior  
**Priority:** P1

**Decision:** Optimize the **midpoint-referenced planar SE(2) robot pose** against a **truncated unsigned Euclidean distance-to-nearest-confidently-occupied-cell field** computed from an **immutable occupancy-grid map revision**. Start with **bilinear interpolation**, independently robustify **each valid deskewed LiDAR endpoint** using Ceres Huber loss, apply an **uncertainty-weighted odometry-pose prior**, and **explicitly penalize endpoints beyond the supported map boundary**. Preserve unknown-space semantics and reject low-overlap, poor-residual, degenerate or otherwise untrustworthy solutions. **The occupancy grid is authoritative; EDT is a derived, versioned cache.** Ceres is private to the matcher implementation; public C++20 map/pose/match interfaces are independent of **Ceres and ROS**.

**Alternatives evaluated:** (A) truncated unsigned EDT + bilinear + per-beam Huber — **accepted**; (B) signed distance / occupancy-probability residuals — future interchangeable model; (C) feature/line/point objectives — later optional alternatives; (D) bicubic interpolation — defer until controlled convergence/overshoot/runtime measurements demonstrate an advantage. A common native `ScanMatcher` abstraction may select a different objective implementation later without changing timing, motion or occupancy APIs (OD-001–OD-006).

### Cost function and exact Ceres residual convention

For a valid **OD-004 midpoint-deskewed endpoint** `p_i` in the **midpoint laser frame**, candidate robot pose `T=T_map_base_mid`, calibrated laser extrinsic `T_base_laser`, and EDT in the **pinned submap/map frame**:

~~~text
q_i(T) = T_map_grid_or_submap^-1 * T * T_base_laser * p_i
D(q)   = clamp( EuclideanDistanceToConfidentOccupied(q), 0, D_max )

r_i(T) = sqrt(w_i) / sigma_i * D(q_i(T))     // ONE scalar per valid HIT

e_prior(T) = Log( T_pred^-1 * T )            // (dx,dy,dyaw); specified tangent

E_Ceres(T) = 1/2 * sum_i Huber_delta( r_i(T)^2 )
           + 1/2 * || L * e_prior(T) ||^2
           + 1/2 * sum_i r_boundary_i(T)^2

L^T L = Sigma_pred^-1
~~~

This is the user's proposed objective in **Ceres's conventional 1/2 robust-least-squares normalization** (the common factor does not change the minimizer). `w_i` is nonnegative beam confidence, `sigma_i` is strictly positive effective **metre-scale** endpoint/map uncertainty, and Huber `delta` is in the **dimensionless units of the normalized residual**. Use `rho_delta(w_i * D^2 / sigma_i^2)`, **not** a residual that already squares the distance. `T_pred` and `Sigma_pred` come from OD-006 motion prediction at **the same scan midpoint time**, transformed into the same reference frame with a documented SE(2) tangent convention and valid positive-definite covariance; if the prior is invalid, do **not** silently use a zero-variance or identity-covariance prior.

**Per-beam Huber implementation is mandatory:** Create **one scalar Ceres residual block per retained beam**, each with an independent `ceres::HuberLoss(delta)`. Packing all beams into one block under one Huber loss robustifies **one combined norm**, which is not equivalent. Keep the **3D whitened prior residual** as a separate **unrobustified** block unless a later decision explicitly changes prior robustness. Boundary penalties are separate residual blocks, with explicitly selected scaling (not silently multiplied by every beam's range confidence). Use a correct local parameterization for SE(2) and validate auto-differentiation or analytic Jacobians numerically.

**Motion-prior evidence and GTSAM double counting:** This prior is an explicit **regularizer for the frontend pose estimate**, unlike the former LiDAR-only formulation. The resulting **posterior/regularized pose is correlated with wheel and gyro measurements** and is **not itself a statistically independent LiDAR-only scan factor**. The frontend MUST return **separate LiDAR-only geometric diagnostics/information**, the applied prior (source IDs, covariance, pose, strength), boundary statistics and the regularized posterior pose. When the backend also uses wheel/gyro factors from the same samples, it MUST **not** add the regularized posterior pose as an independent scan-only GTSAM factor. Factor construction must instead use a separately justified LiDAR-only relative observation/information, an explicitly conditioned/correlated factor treatment, or an alternative audited graph topology; **accepted OD-008** governs LiDAR rank/covariance, while **accepted OD-013** fixes keyframe/submap graph topology and conservative factor admission. Do not simply subtract prior Hessians from the posterior and assert an exact independent LiDAR covariance without validating conditioning and shared map/deskew correlations.

### EDT cache, unknown-space semantics and boundaries

- Build EDT **only from confidently occupied occupancy cells** according to a documented configurable occupancy-probability/log-odds threshold (details OD-010). Unobserved/unknown cells are **not obstacles** and **not evidence of free space**. The authoritative occupancy grid keeps free/occupied/unknown probabilities and all map origins, rotations, resolutions and revision IDs.
- Use real **Euclidean metric distance** in metres and apply configurable truncation `D_max` for the interior field. Distance outside the confidently observed/supportable map region MUST NOT be treated as free or confidently matchable. **Unknown regions are masked/flagged** separately from EDT numeric values; beams in unknown/unsupported regions are not ordinary LiDAR match inliers and do not increase overlap.
- **Bilinear interpolation** samples the four neighboring valid grid values without overshooting their minimum/maximum, but its first derivatives may jump at grid-cell boundaries. Test continuity, finite derivatives within cells, grid origin including yaw, and near-cell-border stability. Do not automatically fill masked/unknown neighbors with zero distances: define and test the validity stencil/skip policy.
- **Out-of-map penalty is separate from interior truncation:** Compute a continuous, outward-increasing distance to the **supported grid/map boundary** in the map's local metric coordinates, and add a finite smooth or piecewise-smooth **hinge-style boundary residual** beyond it. Do not just clamp coordinates and sample a constant edge value, because this supplies no useful gradient back into the map. Distinguish out-of-grid, inside-grid-but-unknown, and inside-supported samples in metrics.
- All residuals MUST stay finite and bounded under extreme pose guesses; reject NaN, invalid resolution, invalid map size, degenerate fields with no confident occupied cells, and oversized parameter/search regions. Truncation saturation makes some beam residuals locally gradient-free; require enough informative (nontruncated/known) beams for a valid correction.
- An immutable EDT cache records source occupancy-grid revision/checksum, occupancy threshold, resolution/origin, unknown-mask policy, EDT algorithm/version, truncation and interpolation settings. Cache validity/rebuild/tiling is still **OD-011**; occupancy inverse-sensor updates remain **OD-010**. No matcher may read a concurrently mutated grid or mismatched cache.

### Acceptance, gating, and backend contracts

A Ceres optimizer reporting success is **not sufficient** to accept a scan correction:

- **Precheck:** OD-005 scan eligibility, **OD-004 full scan interpolation** where mapping is intended, valid calibration/clock epoch, predicted covariance, matching map/EDT revision, enough valid hit endpoints with angular/spatial distribution.
- **Postcheck:** report fraction and count of beams inside **supported observed map**, fraction of informative/nontruncated residuals, map unknown/out-of-bounds counts, robust **LiDAR-only** residual quality (also raw RMS/median/p95), pose jump and Mahalanobis consistency with predicted odometry, convergence/termination, iterations and solve time.
- **Observability:** Assess the **LiDAR-only** geometric Hessian/information **without the odometry prior and artificial boundary penalty** so a strong prior cannot disguise corridor degeneracy or missing map overlap. Inspect eigenvalues/condition, reject/inflate uncertainty in unobservable directions; numeric thresholds are configured and tuned under **OD-008**.
- Reject corrections with insufficient map overlap, excessive outliers/high geometric residuals, boundary-dominated solutions, failed convergence, non-finite covariance or underconstrained geometry. Return typed acceptance/rejection reason, provenance and conservative uncertainty; do not confuse a small posterior objective with reliable LiDAR alignment.
- Different modes may choose independently versioned thresholds and iteration/search budgets, but **the residual model and public C++20 interface are shared** for offline mapping and known-map localization.
- Publish a **LiDAR-only diagnostic score** as well as the **total regularized Ceres cost**, prior penalty and boundary penalty; do not rank experimental beam geometry quality solely by the posterior total.

### Interface and implementation boundary

~~~text
ROS-free OccupancyGrid revision (authoritative)
       + occupied confidence threshold / supported-known mask
        -> derived immutable truncated unsigned EDT cache
              + versioned metadata
                            |
generic AssembledScan -> OD-004 midpoint deskew -> valid beam endpoints
                            |                       |
OD-006 midpoint pose/covariance ---------------------+
                            |
            ScanMatcher interface (native C++20)
                            |
       Ceres EDT implementation (bilinear, per-beam Huber,
           distinct odom prior and out-of-bounds penalties)
                            |
            MatchResult {pose, quality, uncertainty,
             LiDAR-only evidence, prior provenance,
             map/EDT revisions, accepted/rejected reasons}
                            |
       map update gate / localization / OD-013 graph accounting
~~~

**No Ceres types, cost functions, solver summaries, ROS messages or vendor drivers in the public domain interfaces.** An optional future objective implementation (probability, signed distance, geometric features) plugs into the same scan/match/map contracts. Keep the private Ceres implementation and its dependency behind a separately linked target.

### Required tests (not yet executed)

1. Euclidean distance transform values on known walls/points at 0.05 m and rotated/translated map origin; threshold/truncation and a no-occupied-cell case.
2. Bilinear interpolation midpoint/edge samples, derivatives, discontinuities at cell boundaries, unknown-cell validity mask, known-vs-unknown overlap, no artificial free-space assumption.
3. **One residual block per beam**, Huber inlier/outlier boundary cases; objective and gradient match `sqrt(w)/sigma * D`, not double-squared distance or a single joint loss. Explicitly test Huber scale parameter units.
4. Predicted-odometry `SE(2) Log` and covariance whitening in the correct frame; prior weight changes result; invalid/singular covariance fails clearly. **Strong prior does not improve measured LiDAR-only observability.**
5. Finite, outward-increasing out-of-grid penalty with correct gradient for rotated map origin; boundary-dominated solutions rejected; unknown inside grid handled separately.
6. Synthetic apartment/warehouse ground-truth scan matching with OD-003–006 midpoint beams, prior-covariance/pose perturbations, outliers, dynamic clutter and symmetric aisle ambiguities.
7. Compare EDT+bilerp+per-beam Huber to LiDAR-only and no-prior ablations, plus later optional bicubic/occupancy/signed-distance variants. Log convergence basin, acceptance/rejection, false-lock, accuracy, p50/p95/p99 matching latency and Pi CPU.
8. Graph integration proves a **regularized pose is not exported as an independent LiDAR-only factor** beside wheel/gyro factors; raw source evidence and map/EDT cache revisions remain auditable.
9. Docker Catch2/CTest with Ceres-linked matcher target and **ROS-free**, **Ceres-free public core headers**. Never add Boost without the separately required per-case discussion and approval.

**Consequences:** The **objective family, beam loss grouping, initial interpolation, odometry regularization, boundary policy, versioned cache and dependency boundaries** are accepted. Numerical Huber scale, `D_max`, occupancy confidence threshold, beam weights, effective measurement sigma, gate thresholds, map-boundary residual scale, observation covariance calibration, solver budget and exact backend factor choice remain **configurable/tunable or separately open under OD-008/010/011/013**. No implementation or measured SLAM accuracy is claimed.

**Technical references:** [Ceres residual/loss conventions](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html), [Ceres loss function API](https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/loss_function.h), [Ceres model FAQs](https://ceres-solver.readthedocs.io/latest/modeling_faqs.html).

### OD-008 — Robust LiDAR-only information, rank-aware gating and calibrated covariance

**State:** **Accepted** (architecture selected; implementation/calibration/tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection  
**Priority:** P1

**Decision:** Derive **LiDAR-only robust Gauss–Newton information** from the individual normalized beam residuals of accepted OD-007. **Exclude the OD-007 odometry prior and artificial map-boundary residuals**; assess physically normalized SE(2) eigenstructure, observability rank and degeneracy. For fully observable matches, estimate covariance from information and apply **held-out-data-calibrated inflation and positive-semidefinite floors**. For rank-deficient matches, preserve only genuinely observable subspaces; **project/suppress unsupported corrections or reject the match**. A distinct **ROS-/Ceres-independent quality/uncertainty component** judges convergence, overlap, residuals, observability, innovation and acquisition uncertainty. A future bootstrap is optional for **offline validation**, not an initial online dependency.

**Alternatives considered:** (A) plain Hessian inverse — rejected as overconfident with outliers, unknown map error and sensor correlation; (B) robust LiDAR-only weighted information plus eigen-rank gating and calibration — **accepted**; (C) Ceres covariance directly — diagnostic cross-check only, not a calibrated SLAM covariance; (D) scan-resampling/bootstrapping — deferred optional offline check.

#### LiDAR-only information (distinct beam confidence versus robust influence)

Use the **OD-007 normalized per-beam scalar residual** `r_i(T)=sqrt(w_i^beam)/sigma_i*D(q_i(T))` at the converged pose, with native 1×3 SE(2) tangent Jacobian `J_i=∂r_i/∂delta_xi` and **independent per-beam Huber** derivative `alpha_i=rho_delta'(r_i²)`.

~~~text
H_L = sum_i alpha_i * J_i^T * J_i
alpha_i = rho_delta'(r_i^2),  0 <= alpha_i <= 1
H_L = H_L^T >= 0   // symmetric positive semidefinite
~~~

`w_i^beam` already appears **inside** `r_i` and `J_i`; **do not multiply it again** as `alpha_i`. Only include valid-hit, map-supported, finite, useful LiDAR geometric Jacobians; count/exclude unknown, out-of-grid, invalid/missing returns, saturated zero-gradient EDT samples and invalid Jacobians. **Never include** odom-prior residual blocks, boundary penalties or graph priors. This PSD *robust-influence Gauss–Newton approximation* deliberately omits `rho''` robust curvature: it is **not necessarily identical** to the solver's robustified effective Hessian. Evaluate and test the actual nonrobustified LiDAR Jacobians in Ceres's SE(2) tangent convention, applying `rho'` explicitly and checking Ceres manifold conversions/`Problem::Evaluate` loss settings.

#### Physically normalized observability and rank

Translation metres and yaw radians require a **recorded, configurable physical normalization**. Define `delta_xi=S*z`, e.g. `S=diag(s_x [m],s_y [m],s_yaw [rad])`. Then:

~~~text
H_norm = S^T * H_L * S = V * diag(lambda_1,lambda_2,lambda_3) * V^T
d_j = S * V[:,j]      // observable/weak directions in physical tangent
rank = count(lambda_j passing absolute AND relative information gates)
P_obs = V_obs * V_obs^T
P_null = I - P_obs
~~~

Report `S`, tangent frame/perturbation convention, eigenvalues, condition number, rank (0–3) and observable/null bases. **Unscaled mixed-unit eigenvalue ratios are not sufficient**. A strongly weighted motion prior MUST NOT improve the **LiDAR-only** rank. Account for near-threshold eigenvalue uncertainty, negative numerical eigenvalues and non-finite inputs; true material negative eigenvalues indicate bad information calculation.

#### Covariance and rank deficiency

For **rank 3** with credible normalized noise and acceptable condition number, initialize:

~~~text
Sigma_raw ≈ H_L^-1
Sigma_match = K * Sigma_raw * K^T + Sigma_floor
~~~

`K` is a **documented empirically calibrated inflation transform**; `Sigma_floor` is an **empirically calibrated PSD model-error floor**. Symmetrize and check finite PSD/SPD behavior, frame transformation and covariance calibration revision. This is an **approximation**, not exact independence: correlated beams, gyro/wheel-conditioned deskew, occupancy/EDT map uncertainty, variable timing and shared geometric structure may cause optimistic precision.

For **rank < 3**, **DO NOT** publish a pseudo-inverse as a valid full 3×3 pose covariance: its null-direction zero variances would falsely imply perfect certainty. Provide **observable-subspace information/covariance**, an explicit null-space basis, and `FULL_COVARIANCE` / `OBSERVABLE_SUBSPACE` / `UNAVAILABLE` status. Null-direction uncertainty is **unavailable/unbounded**, never made confidently finite by `Sigma_floor`. Either (1) **project the candidate correction** relative to OD-006 predicted pose into the **normalized observable subspace**, leaving unsupported components unchanged, then recheck acceptance; or (2) reject and keep odometry-only tracking marked uncertain/lost as appropriate. Projection is **not required** where nonlinear coupling or poor evidence makes rejection safer. The OD-013 backend may accept a rank-aware constraint only with an appropriate factor type; never feed this as an ordinary full-rank LiDAR measurement.

#### Separated acceptance and empirical calibration

A native **MatchQualityEvaluator** takes geometric/Jacobian diagnostics **from** the private Ceres adapter, but does not depend on Ceres, ROS, LD19, MPU6050 or Pico. It checks all of:

1. Successful solver termination, finite parameters/Jacobians, coherent map/EDT/calibration and clock revisions.
2. OD-005 hit count, angular distribution, valid **observed-map overlap**, informative/truncation-free residuals, unknown/boundary exclusions.
3. LiDAR-only normalized raw and Huber-robust residual quality, outlier fraction and boundary-dominated solution.
4. **LiDAR-only** normalized eigenvalues/rank/conditioning, independent of the prior.
5. OD-006 prediction-to-match **SE(2) innovation** and uncertainty-aware consistency, conservatively accounting for **shared wheel/gyro-conditioned deskew** (do not automatically assume independent covariance sums).
6. OD-003 beam-time uncertainty, gyro bias/slip, missing wheel/IMU samples, OD-004 trajectory coverage, stale clock/model and computational budget.

Return `ACCEPT_FULL`, `ACCEPT_PROJECTED`, `REJECT` or `UNAVAILABLE` with typed reasons; the exact public enumerators are chosen under OD-017. An accepted match exposes corrected pose, information rank, **observable directions and null directions**, full calibrated covariance **only if rank 3**, quality and provenance. A rejected match still exposes diagnostics. A strongly constrained posterior pose **does not justify** a LiDAR geometry constraint when rank is low.

Calibrate `K`, `Sigma_floor`, rank/condition and acceptance/innovation thresholds on **held-out** synthetic truth and datasets with **independent reference** after confirming map-building/training/evaluation splits. Scenarios MUST cover corridors, corners, open rooms, partial scans, repeated warehouse aisles, varying speed, degraded encoders/IMU, clock uncertainty and wrong odometry prior. Quantify consistency/coverage, NEES when a defensible full covariance and independent truth exist, false accept/reject, rank classification and runtime. **Never** claim absolute-error calibration from a LiDAR-derived self-reference. Bootstrap/resampling may be an **offline sanity check** accounting for correlated beams; do not assume they are i.i.d.

#### Architecture and checks

~~~text
OD-007 private Ceres ScanMatcher
   -> converged pose + separate LiDAR-only r_i/J_i/Huber influences
   -> prior/boundary/solver/overlap and data-quality diagnostics
   -> native MatchQualityEvaluator (Ceres/ROS independent)
   -> PSD H_L -> physical normalization -> rank / directions
   -> calibrated full-rank Sigma OR observable-only subspace / reject
   -> MatchResult {pose, outcome, rank, bases, uncertainty status,
                    LiDAR score, prior provenance, calibration/revisions}
   -> localization / Web Lab / guarded OD-013 graph adapter
~~~

Docker Catch2/CTest MUST verify analytical PSD Hessians with beam weights counted **once**, Huber outliers and `rho''` distinction, SE(2) tangent Jacobians vs numeric derivatives, normalization/rank under frame changes, rank-1/rank-2/rank-3 cases, a corridor with strong odometry prior still LiDAR-degenerate, no nullspace false covariance, projected corrections and reject fallback, covariance PSD/floor/inflation, clock/slip/partial-scan gating, source/prior/EDT provenance, held-out calibration accuracy and optional Ceres covariance comparison. Quality evaluator unit tests build **without Ceres and ROS**; numerical Ceres adapter tests link Ceres privately.

**Consequences:** This adopts **robust LiDAR-only PSD information, normalized observability, subspace-aware degeneracy, empirical covariance calibration and a separate quality module**. Exact normalization scales, thresholds, inflation/floor parameters, projection acceptance and fit quality are **not fixed or calibrated yet**; accepted OD-013 provides the initial correlation-aware factor admission policy; exact joint cross-covariance modeling remains a future validated extension. ROS adapter outputs must never translate unknown null-direction uncertainty into fake small finite pose covariance.

**References:** [Ceres covariance documentation](https://ceres-solver.readthedocs.io/latest/nnls_covariance.html) and [Ceres robust-loss conventions](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html).

### OD-009 — Covariance-aware multi-resolution correlative search and branch-and-bound recovery

**State:** **Accepted** (architecture chosen; implementation, calibration and Raspberry Pi measurements pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of mode-dependent correlative search and branch-and-bound  
**Priority:** P2

**Decision:** During normal tracking, initialize **OD-007 Ceres directly from the OD-006 motion-predicted midpoint pose**, with **no coarse search**. When confidence falls, use bounded **local correlative search** to propose Ceres seeds. For recovery or global relocalization, use **multi-resolution correlative branch-and-bound** over **immutable, versioned map/submap snapshots**. Search windows derive from pose covariance and tracking state, subject to configured mode maxima and **separate real compute budgets**. Preserve several spatially distinct hypotheses where necessary, refine candidate poses using OD-007, and **accept corrections only after OD-008 LiDAR-only quality, observability and state-appropriate innovation checks**. Run large searches **asynchronously**, never blocking acquisition, odometry, recording or local tracking. The coarse-search interface remains independent of ROS and Ceres.

**Alternatives:** (A) Ceres-only from odometry — retained for **normal** tracking, insufficient after large pose error; (B) small bounded local exhaustive/correlative search — selected for degraded tracking; (C) **precomputed multi-resolution branch-and-bound — selected for recovery/global**, with provable score bounds if exact pruning is claimed; (D) persistent particle filter — **not required initially**, potentially revisited under OD-020 if multimodal kidnapping/repeated-aisle behavior needs ongoing hypothesis tracking. Search is a *candidate generator*, not a reliable pose measurement by itself.

#### Accepted operating modes and initial budget targets

| Mode | Search window | Initial CPU execution budget | Strategy |
| --- | --- | --- | --- |
| `TRACKING` (normal) | Ceres seed at odometry prediction | **0 ms coarse** per scan | OD-007 direct Ceres |
| `DEGRADED` / `UNCERTAIN` | **±0.3 m X/Y, ±10° yaw** | **10–20 ms** per scan when invoked | Bounded local correlative scan search; top candidate(s) -> Ceres |
| `RECOVERY` | **±1.0 m X/Y, ±30° yaw** | **30–50 ms** per invocation | Multi-resolution branch-and-bound, candidate refinement |
| `GLOBAL_RELOCALIZATION` | Full eligible frozen candidate submap(s), **360° yaw** | **100–200 ms per processing slice** | Asynchronous multi-resolution branch-and-bound with checkpointed/cancellable slices |

These are **initial proposed budgets for Raspberry Pi 5 at roughly 10 Hz LD19 input**, **not measured or guaranteed runtimes, schedulability proofs or hard real-time deadlines**. The budget measures **host steady monotonic execution time**, not pausable ROS `/clock` event time, and MUST cap evaluated candidate nodes, memory and queued work independently of elapsed time. Full global search may require **multiple slices** and MUST NOT be falsely advertised as exhaustively completed within one slice. Record actual p50/p95/p99 latencies, scheduling interference, queue lag and thermal effects; tune on real Pi hardware.

#### Covariance-aware windows, angular resolution and hypotheses

- Derive a **bounded covariance-aligned SE(2) search region** around the OD-006 predicted pose for local modes, with explicit tangent-frame/unit conversion and a configured confidence multiplier. Clip the final X/Y/yaw extents to the **mode-specific maxima** above, report truncation/clipping and avoid implying complete recovery outside the cap. Handle invalid/strongly correlated/degenerate prediction covariance by degrading mode or moving to global recovery, not fabricating a tiny window.
- Choose angular sampling **from occupancy-grid spatial resolution and useful beam range** so rotation between samples does not displace informative endpoints by more than a configured spatial sampling tolerance (approximate small-angle relation `delta_yaw <= spatial_step / representative_range`); handle near-zero/large ranges and cap sample count.
- Preserve **multiple distinct high-scoring hypotheses** using nonmaximum suppression or equivalent SE(2) separation (translation/yaw with proper wrap), source map/scan revisions, geometric support, score, remaining budget and rank. Avoid returning duplicates of one local peak as independent hypotheses.
- Candidate score is a **coarse ranking**, not a covariance, likelihood calibration or OD-008 acceptance. Correlation ambiguity in repeating warehouse aisles requires cross-scan temporal consistency before a global relock; this persistent-confirmation policy is coordinated with **OD-020/OD-022**.

#### Immutable correlation maps and valid branch-and-bound

- Maintain an **occupancy-derived, versioned, immutable correlative score grid and precomputed multi-resolution search pyramid** for each frozen map/submap snapshot. The occupancy grid remains the source of truth; the OD-007 EDT is a **distinct derived cache** and need not share an objective with the coarse matcher. Store source occupancy revision/hash, known/unknown mask, score policy, resolution/origin/yaw, pyramid construction parameters, cache version and memory size. Invalidate/rebuild **on map revision change**, not every scan; caching/eviction remains OD-011.
- **Unknown, out-of-map, invalid and OD-005 missing returns must not be scored as known free or occupied evidence.** Only OD-004 midpoint-deskewed valid-hit endpoints enter the coarse candidate score (or a separately documented sensor-supported valid-no-return policy for an explicitly tested objective). Track number/fraction of supported beams, occupancy coverage and map support across levels.
- If pruning is called **branch-and-bound**, coarse-level cells MUST provide **admissible upper bounds for the same finer-resolution score** under all translations, yaw samples and supported-region/unknown-mask handling. A maximum-over-descendants/precomputation approach is a candidate; prove or exhaustively test bounds. A coarse **average, EDT minimum or heuristic** is not automatically an upper bound. If admissibility cannot be shown, label the algorithm **heuristic best-first multiresolution** and do not claim optimality or exhaustive exclusion.
- Use deterministic ordering/tie breaking, controlled score quantization and proper occupancy-map rotated origin transforms. Cache precomputation is separate from scan-time CPU budget, but **must** be measured and bounded in RAM/build time. Reuse map pyramids across scans; validate cache lifetime across online map revision publication.

#### Quality, refinement, cancellation and failure

1. **Direct normal path:** OD-005 scan + OD-004 midpoint deskew and OD-006 predicted pose -> OD-007 Ceres -> **OD-008** quality evaluator. No coarse pyramid work on the critical path unless explicitly enabled by tracking health.
2. **Degraded/local:** on deteriorating OD-008 score/overlap/rank/innovation, perform a **bounded** local correlative search then Ceres refinement on top candidate(s). Refined candidates must pass OD-008; high coarse correlation alone is never sufficient.
3. **Recovery/global:** schedule branch-and-bound on a **background bounded worker**, holding immutable pinned map/scan/calibration/clock revisions. Search can yield **PARTIAL_BUDGET** or **CANCELLED** without implying no valid poses exist. If a worker resumes, its search frontier/checkpoint must match the original snapshot and configuration; stale/replaced snapshots require invalidation/restart.
4. **Refinement vs priors:** A distant correct recovery hypothesis may violate a **tight OD-007 motion prior**. Use a mode-appropriate covariance/prior policy (weakened, validity-gated or explicitly configured no-prior Ceres path) rather than forcing all global hypotheses back toward a stale odometry seed. Still apply **OD-008** LiDAR observability and **state-aware** innovation/relock gates; do not turn a large kidnapped-robot jump into an unconditional high-confidence correction.
5. **Acceptance:** Separate search score, local refined pose, OD-008 geometric acceptance, and multi-scan/global relock confirmation. **Normal occupancy map insertion remains disabled when global pose is uncertain/lost**, even if a partial search or one coarse hypothesis looks plausible. OD-006 local odometry continues, while recovery attempts are logged.
6. **Resource isolation:** enforce host-steady CPU slice budgets **plus** node-count, hypothesis-count, memory, queue-depth and cancellation limits. Do not call a long search synchronously from sensor acquisition, packet decoding, recording, odometry, live frontend deadline or ROS callback. A processing slice must yield to higher-priority tracking. Detect starvation, map revisions, new scans, source clock jumps and shutdown; cancel or explicitly restart.
7. **Native boundary:** Define an abstract `CoarseSearch` or `PoseCandidateSearch` taking an immutable map-score snapshot, OD-005/004 prepared scan, covariance/pose hint if available, search mode/budget and cancellation token; return versioned ranked `PoseCandidate` list, progress/coverage, `COMPLETE`/`PARTIAL_BUDGET`/`CANCELLED`/`INVALID` and detailed diagnostics. A separate coordinator invokes **OD-007 ScanMatcher** and **OD-008 MatchQualityEvaluator** for each selected candidate. No Ceres, ROS, sensor hardware or micro-ROS types in coarse-search public headers. Coarse score objective and budget configuration are **independent** of Ceres solver cost/budget; shared authoritative map identity is enforced.

#### Test and benchmark requirements (not yet executed)

1. Synthetic apartment/corridor/warehouse with exact pose perturbations (small, ±0.3 m/10°, ±1.0 m/30°, large global 360°); confirm correct mode selection, bound clipping and odom-only normal coarse-work **0**.
2. Rotation sampling vs map resolution/beam distance, SE(2) yaw wrap, rotated/translated occupancy origin, partial/missing OD-005 beams and unknown-region support.
3. Compare branch-and-bound versus exhaustive fine-grid enumeration on small fixtures: upper-bound admissibility, deterministic pruning, candidate scores/ranks and no lost true optimum under **completed** full-budget search.
4. Repetitive shelving and symmetric apartment layouts: deduplicated distinct top-K candidates, ambiguity flag, Ceres refinement/OD-008 true-vs-false lock decisions and independent multi-scan relock stability.
5. Candidate covariance-window scaling/clipping under growing OD-006 wheel slip/bias uncertainty; stale predictions and mixed map/scan revisions; no false certainty from an invalid prior.
6. Quantified **10–20 / 30–50 / 100–200 ms** host-steady execution slices with hard node/memory limits, explicit incomplete/cancelled result, resumable frontier and no blocking of simulated 10 Hz acquisition.
7. Global kidnapped-robot recovery using a correctly relaxed/no-prior OD-007 refinement policy and OD-008 state-aware innovation, with no map insertion while LOST/RELOCALIZING.
8. Immutable occupancy-derived pyramid cache reuse, invalidation on revision change, independent EDT vs correlative score objective, map/unknown masks, bounded build time and RAM.
9. Docker Catch2 and end-to-end benchmarks measure success/false-lock, branch pruning efficiency, number of candidates and Ceres runs, final pose accuracy with independent truth, p50/p95/p99 latency, Pi CPU/memory and acquisition queue delay. No GitHub Actions execution.

**Consequences:** Multi-mode search, initial covariance-aware windows, proposed resource targets, branch-and-bound for recovery/global, asynchronous execution and **OD-008-only correction trust** are accepted. Exact score function, pyramid resolution/depth, admissibility construction, candidates K, normalized scoring, thresholding, memory/latency caps and state transitions remain tunable or related to OD-011/020/021/022. Particle filtering is **not initially required**; it remains an alternative if multi-modal ambiguity demands persistent beliefs. **No measured performance or completed search implementation is asserted.**

**External reference:** [Cartographer 2D fast correlative scan matcher](https://google-cartographer.github.io/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.html) (precomputed maximum-score grid pyramid with branch-and-bound, later Ceres refinement).

### OD-010 — Bounded 5 cm log-odds occupancy and two overlapping submaps

**State:** **Accepted** (architecture chosen; configuration calibration and implementation/tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit choice of 5 cm probabilistic grids, 90/180 insertion lifecycle and immutable freezing  
**Priority:** P1

**Decision:** Use **0.05 m** native planar **log-odds occupancy grids** as the **authoritative map**, with normally **two overlapping active submaps**. After **90 accepted scan insertions into the youngest submap**, create a successor; after **180 accepted insertions into the older submap**, permanently **freeze its local occupancy raster**. Thus each complete submap normally contains 180 accepted scan insertions, and adjacent submaps share about **90 accepted scan IDs**, i.e. **50% overlap by insertion count** (not a promise of 50% spatial overlap). Apply configurable **motion filtering and bounded spatial/memory safeguards**. Preserve OD-005 invalid/unknown semantics; updates are probabilistic per-hit/per-free-ray, with **hits taking priority and no repeated cell update within one scan/submap insertion**. Frozen submap **global SE(2) pose may change** through GTSAM while local grid contents never change. EDT (OD-007) and correlative pyramids (OD-009) are **versioned derived caches**, not authoritative maps. No time-based decay of frozen maps; any future dynamic aging belongs in a separate transient layer. All mapping/domain interfaces stay **ROS-independent**.

**Options:** (A) 0.05 m — **accepted initial default**; (B) 0.025 m — optional for accuracy if approximately **4×** grid-cell RAM/update cost is justified for a fixed physical area; (C) 0.10 m — optional for large/memory-constrained maps, about one-quarter the 0.05 m cell count; (D) global mutable grid or time-aged frozen submaps — not chosen. Thresholds are configurable and require Pi 5 measurements.

#### Initial configuration (not validated defaults)

| Parameter | Initial setting | Status |
| --- | --- | --- |
| Grid resolution | **0.05 m** (5 cm) | Accepted default; configurable |
| Active submaps | **2 normally**, with 1 during startup/terminal stages | Accepted |
| Spawn successor | Youngest reaches **90 accepted insertions** | Accepted initial threshold |
| Freeze predecessor | Oldest reaches **180 accepted insertions** | Accepted initial threshold |
| Overlap | 90 common inserted scan IDs out of 180 per full submap | Approximately **50% by insertion count** |
| Occupancy prior | **0.50** | Unknown until first valid observation; retain separate observed flag |
| Hit update probability | **0.55** | Calibration starting value |
| Miss update probability | **0.49** | Calibration starting value |
| Probability limits | **0.10–0.90** for observed cells | Calibration starting bounds |
| Motion filtering | Minimum translation, rotation and/or elapsed acquisition time | Configurable; no unverified numeric defaults |
| Extent/memory/number of cells | **Hard bounded** dynamic allocation; safe early freeze/degrade | Numeric limits need Pi measurements |
| Dynamic occupancy decay | **Off for frozen submaps** | Accepted |
| Future transient dynamic layer | Optional, versioned separately | Deferred |

The 90-insertion / 0.05 m / 0.55 / 0.49 start points align with published **Cartographer 2D example configuration**, while the 0.10–0.90 clamp is a documented probability range; these values have **not** been optimized for this robot/LD19. An LD19 at ~10 Hz and roughly 450 points/revolution provides many rays, but does **not** justify unbounded per-scan memory or a claim that every scan is useful.

**Bootstrap exception for an empty map:** A newly created mapping session has no occupied cells, so **OD-007 Ceres cannot establish a scan-to-map match for its first insertions**. Initialize the first submap with a declared local gauge pose and a **trusted initialized OD-006 trajectory** and valid OD-003/004/005 timing/scan evidence (or a documented external initial pose). Mark these insertions **BOOTSTRAP_UNVERIFIED_MAP_MATCH**, with conservative uncertainty/provenance, until sufficient occupied geometry exists for OD-007/008 validation. Do not publish a bootstrapped pose as a LiDAR-verified global fix. Switch to ordinary OD-007/008 gated insertions after the map becomes geometrically supportable. A new overlapping submap may use the already tracked pose and prior established map as its seed; it does not independently require a fresh global bootstrap. A LOST/UNCERTAIN pose must not bypass gates using this initial-empty-map exception.

#### Exact lifecycle and insertion-count semantics

- Count **successful logical scan insertions**, **not** raw LD19 packets, beam count, received revolutions, Ceres matching attempts, wall seconds or the number of insertions **summed across both active maps**. A scan must pass OD-005/004 quality and OD-008 localization gates, then the configured **motion filter**, before entering mapping. A skipped scan is retained in the recorder but **does not change any submap counter or occupancy cells**.
- Every newly accepted scan is transformed into **each currently active submap's own local frame**, inserted **once per submap**, and assigned a stable scan/insertion ID in each. Each map keeps its own accepted insertion count and IDs. If two active submaps exist, the scan is normally inserted into **both**, not alternating between them.
- **Startup/steady timeline for threshold 90:** insert accepted IDs **1–90** into submap A, create B **after** insertion 90; insert IDs **91–180** into **both** A and B; at ID 180, **freeze A** with 180 insertions while B has 90, then create C for subsequent IDs. Continue: B receives IDs 91–270 and freezes at 180, while C begins at ID 181. A and B share exactly IDs 91–180 in the nominal case.
- Define boundary event ordering explicitly so scan ID 90 is **not accidentally duplicated into the new B** and ID 180 is **not inserted twice** during the rollover. There are **at most two writable active submaps** in nominal operation. A matching scan **must not match against a map already containing its current insertion**; freeze/snapshot revision read before insertion.
- **Safeguards override target counts**: spatial extent, occupied allocation count, peak memory, trajectory travel, max walltime only as a safety trigger, or pathological thin geometry may force **early freeze/new-map rotation, skip or safe stop**. Mark the **actual** insertion count, end reason (`COUNT`/`SPATIAL_CAP`/`MEMORY_CAP`/`SHUTDOWN`/`ERROR`) and observed overlap; never claim a shortened map contains 180 scans or guaranteed 50% overlap. If unable to allocate a bounded next map, **stop insertion safely, retain raw scans and diagnostics**, and never silently truncate occupancy/rays. When only one map remains, report reduced overlap.
- A frozen submap has an immutable **local occupancy raster and local coordinate frame**; its **global graph pose `T_map_submap`** is a **separate mutable/versioned transform**. A pose-graph update does not rewrite frozen cells or invalidate a cache of local-grid geometry; a global spatial index/pose-dependent composite view may need revision updates.

#### Log-odds inverse sensor and unknown semantics

Define per-cell `L = log(p/(1-p))` and an explicit `observed` flag (or equivalent sentinel). Unknown must be distinguishable from an **observed cell with p≈0.50**, including after contradictory updates.

~~~text
L0 = logit(0.50) = 0                      // prior, unobserved
Delta_hit  = logit(0.55)                  // > 0
Delta_free = logit(0.49)                  // < 0
L_min = logit(0.10),  L_max = logit(0.90)

L_new(c) = clamp(L_old(c) + Delta(c), L_min, L_max)
p(c) = 1 / (1 + exp(-L(c)))               // only an observed probability
~~~

- The **first valid update** makes a cell `observed=true`; an untouched prior cell remains **unknown**, not known free. Occupancy confidence and known/unknown support masks feed OD-007 EDT and OD-009 score pyramids. Do not interpret `L=0` alone as "unknown".
- Use validated `VALID_HIT` endpoints only: cast free-space cells **strictly before** the hit and apply a hit update **at** the endpoint. A device-documented `VALID_NO_RETURN` may clear **only a bounded range** if OD-005's explicit no-return configuration permits it; otherwise it makes no ray update. `INVALID_RETURN` and `MISSING` **never** update either free or occupied cells. Range/intensity validation, calibrated max usable range and OD-004 motion coverage remain mandatory.
- **Per scan, per submap:** collect unique **hit cells** and unique **free cells** from all valid rays; remove any free cell also marked hit (hit priority), then apply **at most one update per cell** in that insertion. A hit endpoint encountered on another ray's free traversal **must stay hit**. Do not repeatedly strengthen a wall by multiple rays from the same scan; later accepted scans can update it anew.
- Clip ray traversal to a **bounded allocated grid region** only without inventing an endpoint hit or free evidence outside actual measured, supported rays. If rays/grids cannot be represented safely within memory/extent limits, reject/degrade insertion with an explicit reason rather than fabricate map cells.
- Motion filter decisions use the accepted **map-frame/local-reference pose, timestamp and calibrated uncertainty**, with tunable minimum translation, yaw and max elapsed accepted-insertion interval. Pure long stationary runs should not saturate maps with thousands of nearly identical scans; real motion/timeout policies are tested rather than guessed. **Do not apply the filter to raw recording, time synchronization or OD-006 odometry.**
- Dynamic clutter is initially handled by OD-005 validity, OD-007/008 robust matching and repeatable probabilistic evidence; do not implement aging/decay in frozen maps or silently treat transient unknown as free. A future **separate transient overlay** can define time-limited occupancy with independent revision/serialization, not mutate frozen submap source data.

#### Frozen maps, derived caches and modular API

~~~text
OD-005 quality scan + OD-004 interpolated midpoint deskew
     -> OD-007/008 localization acceptance against prior map snapshot
     -> motion filter + spatial/memory budget guard
     -> active log-odds submap A [and overlapping B] (local frames)
           |              |
       accepted IDs      hit/free/unknown masks
           |              |
     at 90: spawn new  -> at 180: freeze oldest (per-submap counts)
                              |
                      immutable occupancy grid snapshot
                        |                    |
               OD-007 versioned EDT   OD-009 versioned score pyramid
                        \                    /
                    loop closure / localizer / saved map
                              |
             graph optimizes T_map_submap only, not occupancy
~~~

Native C++20 `OccupancyGrid` / `SubmapBuilder` / `FrozenSubmap` / `MapSnapshot` APIs MUST keep log-odds and known masks independent from derived EDT/pyramids, with explicit local SE(2) frames, resolution, origin including yaw, scan/insert IDs, state, memory/cell usage, immutable occupancy revision and separately versioned global pose. **No Ceres, GTSAM, ROS, LD19, Pico or MPU6050 types** in the generic mapping interface. Accepted OD-011 specifies active incremental tile architecture, the initial full-rebuild baseline, atomic snapshot publication and bounded frozen cache eviction/reload; Accepted OD-012 defines JSON + portable zstd binary generations; detailed version-1 schemas and tests remain pending.

#### Required synthetic/hardware tests (not yet executed)

1. Exact insertion-count regression: A IDs 1–180, B IDs 91–270, two active maximum; stable overlap 90; scan 90/180 rollover; skipped motion-filter scans do not alter counters; early safety freeze reports true counts/overlap.
2. `0.05`/`0.025`/`0.10` m geometry and cell count/RAM, transformed submap origin including yaw, sensor extrinsic and laser ray endpoints. Compare detail/coverage with exact apartment/warehouse geometry.
3. Log-odds hit/miss increment/clamps and exact prior unknown-vs-known-neutral discrimination, including contradictory observations and saturation; deterministic serialization/replay equivalence.
4. A scan/submap's duplicate rays update each cell **at most once**, hit cells always take precedence over traversing free rays; no illegal update from invalid/missing/unverified no-return; no endpoint hit at clipped/outside-grid ray.
5. Motion filter: nearly repeated stationary scans, meaningful translation/yaw, max elapsed input, skipped scans still recorded and odometry advances; quality/rejected match cannot enter map or increase counters.
6. Spatial/memory cap, dynamic allocation failures, out-of-grid rays, early freeze and map-creation failure; **no unbounded allocation or silent false occupancy**. Profile x86/Pi 5 memory and insertion CPU, including two active maps.
7. Freeze immutability across concurrent Ceres/OD-007 EDT and OD-009 correlative matcher readers; pose-graph moves only global `T_map_submap`, local hashes and versioned derived fields remain unchanged; global index may be updated.
8. Deterministic replay of active-to-frozen transitions, map revisions, forced scan acceptance/rejection, per-submap scan ID provenance and no map self-matching. Verify offline map artifact round trip, ROS-free Catch2/CTest Docker tests and eventual hardware recordings.

**Consequences:** OD-010 settles **authoritative log-odds occupancy, initial 5 cm/90/180 defaults, two overlapping active maps, per-scan hit precedence, safe unknown handling, bounded local grids and immutable freezing**. The exact motion filter thresholds, memory/spatial caps, hit/miss calibration and alternative resolutions remain benchmark-tuned. **Accepted OD-011** fixes coherent cache updates/eviction/publishing architecture (with tuning pending); **Accepted OD-013** fixes graph topology and factor admission; **OD-012** fixes map serialization and **Accepted OD-014** specifies bounded selective offline remapping; implementation and calibration are pending.

**References:** [Cartographer 2D example configuration](https://github.com/cartographer-project/cartographer/blob/master/configuration_files/trajectory_builder_2d.lua), [Cartographer 2D submap lifecycle](https://google-cartographer.readthedocs.io/en/latest/configuration.html), [Cartographer 2D grid inserter hit precedence](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc).

### OD-011 — Coherent immutable map snapshots, active EDT tiles and reusable frozen caches

**State:** **Accepted** (target architecture selected; simpler baseline first, tiled implementation and performance validation pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of immutable snapshots, incremental active tiles and precomputed frozen fields  
**Priority:** P1

**Decision:** The OD-010 **authoritative 5 cm occupancy/observed grid** remains the only mutable map source. Publish **atomic, immutable, versioned matching snapshots** containing coherent occupancy/known-space support and compatible **OD-007 truncated unsigned EDT** and **OD-009 multiresolution correlative pyramid** representations. For active submaps, **dirty-region tracking and bounded incremental tile regeneration** are the target architecture; a **bounded-cadence complete EDT/pyramid rebuild** is the initial correctness/performance baseline and legitimate fallback. Frozen submaps have immutable precomputed derived fields, reused through a **memory-bounded pinned cache** with on-demand load/reconstruction. All rebuilding and publication are background/budgeted; never block sensor acquisition, odometry or recording. Native domain and cache interfaces have **no ROS or Ceres dependencies**.

**Options:** full rebuild every received scan — rejected as default (can be wasteful); rebuild only at freeze — insufficient for matching against active maps; **tiled active EDT and reusable frozen fields — accepted target**; **full-field rebuild on bounded refresh cadence — accepted initial implementation and correctness oracle**. An optimization shall not be assumed faster without Pi 5 profiling.

#### Initial configuration (unmeasured starting values)

| Parameter | Initial value | Meaning |
| --- | --- | --- |
| Authoritative map resolution | **0.05 m/cell** (OD-010) | Per-submap local coordinates |
| Unsigned EDT truncation | **0.30 m** | Maximum nearest-occupied distance used by OD-007, **not Cartographer TSDF** |
| EDT tile interior | **32 × 32 cells** | At 5 cm, **1.6 × 1.6 m** |
| EDT source halo | **≥0.30 m plus bilinear interpolation support** | At 5 cm, **at least 6 cells radius plus ≥1 cell for interpolation**; account for border alignment/rounding |
| Builder execution | **Background, bounded work units and time/memory caps** | No sensor-thread builds |
| Initial baseline refresh | **Configurable cadence / work cap**, full field | Measure before adding tiled optimization |
| Frozen fields | **Precomputed immutable; on-demand reload/recompute** | LRU-like eviction, pinned reader lifetime |
| Staleness and cache RAM limits | **Explicit configurable caps** | Numerical values require Pi benchmarks |

A **40 × 40 m** submap at 5 cm has **800 × 800 = 640,000 cells**. One float32 EDT is **2,560,000 bytes (~2.56 MB decimal / 2.44 MiB)**, excluding known mask, occupancy, tiling, interpolation padding, score pyramids, object metadata and concurrent snapshot copies. A 32-cell tile has a **32 × 32** output interior; its source computation window is larger.

**Important distinction:** The **0.30 m value here is an *unsigned occupancy-to-obstacle EDT truncation parameter*** for OD-007; Cartographer's similarly sized **TSDF truncation parameter** is a different representation and does **not** establish a validated LD19 EDT radius.

#### Snapshot identity, immutability and ownership

A `MapSnapshot` is a **coherent immutable bundle** for a single local occupancy revision, not unrelated `occupancy_latest`, `edt_latest` and `pyramid_latest` pointers:

~~~text
MapSnapshot {
  submap_id; occupancy_revision; content_hash; map_grid_origin/resolution;
  observed_mask_revision; occupied_threshold/config_hash;
  immutable_occupancy_and_known_mask;
  immutable_EDT { source_occupancy_revision, truncation, interpolation, status };
  immutable_correlation_pyramid { source_revision, score_config, bounds, status };
  graph_pose_revision_or_pinned_transform; // separate from local cache identity
  scan_insertion_id_high_watermark; creation/publication_stamp;
  build_mode { FULL_REBUILD, TILED_INCREMENTAL, FROZEN_LOAD };
}
~~~

- The map builder writes to private mutable state; a worker captures a **read-only occupancy revision** (via copy-on-write/shared immutable tiles or bounded equivalent), builds derived fields and publishes a **single completed coherent snapshot atomically**. No reader sees partially updated tiles, half-built pyramids, mismatched known masks or an occupancy/EDT version mismatch.
- Both derived caches in a snapshot MUST reference that snapshot's **exact occupancy source revision and their own configuration hashes**. When a required cache is not built/valid, the bundle is **not ready for that operation**: OD-007 matching needs a coherent EDT; OD-009 coarse search needs a coherent pyramid. Never attach a new occupancy grid to old EDT/pyramid. A snapshot may expose operation-specific readiness; only **complete coherent versions** are handed to readers.
- Pin snapshot lifetime through a native shared immutable handle (e.g. `shared_ptr<const MapSnapshot>` or equivalent). A Ceres match, BnB search, loop candidate or replay view uses the **same pinned snapshot for its entire operation** and records its map and graph-pose revisions. Global `T_map_submap` updates are independent of frozen local field identity; pin/consistently convert any global transform needed for the match.
- An out-of-date worker result (base revision no longer valid) must be **discarded, versioned separately or restarted**, never published as a matching "latest" revision. If builders lag, a matcher MAY use a **previous complete coherent snapshot** only within separately configured **maximum age and number of accepted insertions behind**; record lag and mark quality accordingly. Once limits are exceeded, defer/reject matching or map insertion instead of mixing revisions or bypassing OD-008 gates.
- Maintain **OD-010 match-before-insert**: candidate scan `n` matches a pinned map revision containing **no observation from scan n**. Successful OD-008 gate and OD-010 motion filter then insert scan n into the authoritative builder, emitting a *new* occupancy revision and dirty events. The presence of older not-yet-published insertions must be accounted for in the staleness/quality policy.
- Bound **peak live memory including old pinned snapshots**, build scratch, tiles, EDT and score pyramids, in-flight jobs and frozen-cache entries. Never evict an in-use pinned object; if pinning prevents reclamation, throttle/cancel low-priority matching or cache builds, report `MEMORY_PRESSURE` and keep acquisition/recording alive.

#### Correct active dirty-region EDT and pyramid updates

After each **accepted** OD-010 occupancy insertion, detect cell changes and publish **dirty events** for:

- Cells crossing the configurable **confidently occupied** threshold in either direction (**addition or removal** of nearest-occupied sources).
- Changes to the **observed/unknown support mask** even without an occupied-threshold transition, because bilinear validity and score support can change.
- Grid growth/extent/origin/resolution change, source calibration or cache-configuration change: **force full rebuild** when local dirty logic cannot prove equivalence.

**Influence bound for a truncated Euclidean EDT:** For an occupancy source transition at `c`, only EDT output sample positions within its configured `R=0.30 m` metric radius can change *as a result of that source* (with appropriate rasterization boundary handling). Mark **all output tiles intersecting the changed source's R-expanded region**, plus the bilinear interpolation stencil/padding. This includes **removing** an occupied cell: any location whose former nearest source was removed must be recomputed against **all remaining nearby occupied sources**, not left with a stale minimum.

For every dirty tile, calculate the output interior from the **current authoritative occupancy snapshot** and source region extended **at least R plus interpolation/padding** in all directions; include occupied sources **outside the tile interior**, invalid/unknown boundaries and map edges. Results must equal a **full exact EDT followed by metric truncation** for every supported output cell, within declared numeric tolerance. A `6-cell` halo is merely the raw radius at 5 cm; allow extra alignment/rounding and stencil support as needed. **Do not treat a small local flood update as automatically exact**, especially after source removal or a changed known-space mask.

- If dirty coverage, topology, support transitions or scratch memory exceed configured bounds, **fall back to a complete field rebuild**.
- The **OD-009 correlative score pyramid is distinct from EDT**; update/invalidate affected descendants and all influenced ancestors with score-bound correctness (including unknown masks). A changed occupancy probability can change correlation scores **even without crossing the occupied/known threshold**. Implement distinct dirty criteria and rebuild logic; do not derive pyramid validity solely from EDT dirty cells.
- A pyramid used for branch-and-bound MUST preserve **admissible upper bounds** against its exact fine score under the pinned occupancy and score configuration. Recompute any affected aggregate maxima/halos, including decreases; test against exhaustive enumeration.
- The *initial implementation* may always compute the **full exact EDT** (e.g. Felzenszwalb–Huttenlocher distance transform) and the **full score pyramid** on a bounded refresh cadence in a background worker. This is the correctness oracle for later tile optimization, not permission to rebuild on every incoming scan or expose incompatible maps.

#### Frozen snapshots, memory cache and graph pose

At OD-010 freeze, finalize the immutable local occupancy/observed mask, construct and verify final EDT and OD-009 pyramid with matching configuration hashes, and publish the **frozen complete snapshot**. A frozen local field never changes when its separate **global graph pose** is optimized. Frozen matching data is retained under a **bounded cache**: keep frequently used pinned entries, evict **only unpinned** cold derived fields, and reload/reconstruct on demand from the unchanged authoritative frozen grid if hashes/config differ or fields are missing. Cache-miss rebuilds occur on background/budgeted workers; a user operation defers or reports `NOT_READY` instead of blocking acquisition.

The **cache-key contract** includes submap ID, authoritative content hash and revision, grid geometry/origin, observed/known mask/occupancy-confidence threshold, EDT truncation/interpolation/code version, and independent correlative score/pyramid configuration. Global pose revision is **not part of local-field cache identity**, but any global spatial index must track pose revisions. OD-012 still decides the physical on-disk native format and atomic persistence; cache source/hashes do not decide file layout.

#### Minimal ROS-free cache-service contract and tests

~~~text
OD-010 accepted insertion -> authoritative OccupancyRevision_N + dirty events
                  |
       bounded background SnapshotBuilder
              /                     \
    full-field initial baseline   incremental tiles later
              \                     /
      verify occupancy, known-mask, EDT, pyramid source revisions
                  |
          atomic publish MapSnapshot_N (ready flags)
                  |
     native MapSnapshotProvider -> pinned immutable handle
            /                 |                  \
      OD-007 Ceres       OD-009 coarse BnB    frozen localizer
       (EDT ready)        (pyramid ready)     / loop closure
                  |
       OD-008 matching quality and map-update gate
~~~

Core-domain `MapSnapshotProvider`, `CacheConfig` and `SnapshotHandle` APIs depend only on native C++20 map, memory and version types; **no Ceres, GTSAM, ROS, LD19, Pico or vendor structures**. Build/eviction scheduling is separate from sensor acquisition and map mutation. Worker queues must be bounded and jobs cancellable; record time, bytes, dirty tiles, queue age, snapshot lag, cache hits/misses, read pins, rebuild reason and publication outcomes. Benchmark thresholds and OS-level thread priorities remain tuning/OD-016/021 questions.

**Required Docker/Catch2 and Pi benchmarks (not yet executed):**

1. Compare exact full-grid EDT with tiled dirty updates for **addition, removal, repeated threshold crossings, obstacle motion, support-mask transitions, near edges/corners and tile seams**, including bilinear sampling/gradients within tolerance.
2. Distinguish EDT occupied-threshold dirty tracking from **pyramid occupancy-probability changes**; verify OD-009 admissible upper bounds even when score **decreases** and when a cell becomes unknown/known.
3. Check tile size 32×32, 0.30 m radius, **≥6-cell + interpolation halo**, rotated map origin, resolutions 0.025/0.05/0.10 m, source cells just outside tile boundary and map growth/recenter full rebuild.
4. Concurrent insertion/read stress: optimizer pins one coherent revision while new maps and caches publish; no mixed versions, torn masks, use-after-free, stale cache attachment or self-matched scan.
5. Refresh fallback and bounded **staleness in time *and* accepted insertion count**; reject/defer when too stale, no silent map catchup/late publications and deterministic replay.
6. Frozen LRU-like eviction, pinned reader preservation, wrong config hash and on-demand reload/reconstruction, map global pose changes without local-cache rebuild.
7. Resource caps: small map, **40×40 m / 640,000-cell** example, concurrent two active submaps, old pinned versions and several frozen entries; measure peak RAM/RSS, background CPU, p50/p95/p99 refresh time, Ceres/BnB latency, cache hit and thread-queue interference at ~10 Hz on **actual Pi 5**.
8. Native interfaces compile without Ceres/ROS and match/insert ordering remains OD-010 compliant; forced worker cancellation/shutdown leaves the latest published snapshot valid.

**Consequences:** **Atomic coherent snapshots, initial bounded full-field builds, eventual exact incremental active tiles, immutable frozen precomputation, bounded eviction/reload and separate versioned EDT/score caches** are settled. Exact refresh cadence, stale age/insertion caps, memory budget, cache eviction policy, tile fallback threshold and numerical tolerance are **performance-tuned parameters**. No incremental implementation or Pi runtime results are yet claimed.

**References:** [Felzenszwalb–Huttenlocher, *Distance Transforms of Sampled Functions*](https://toc.cs.uchicago.edu/articles/v008a019/), [Cartographer fast correlative matching precomputation](https://google-cartographer.github.io/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.html), [accepted OD-010](open-design-choices.md).

### OD-012 — Versioned directory map bundle: JSON manifest + portable compressed binary

**State:** **Accepted** (format architecture chosen; exact binary schema/loader and crash tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of Option A  
**Priority:** P1

**Decision:** Implement a **versioned directory-based native map bundle** with a strictly validated **JSON manifest**, platform-independent, fixed-width **binary authoritative submaps and graph payloads**, and **per-file Zstandard (zstd) compression**. Persist authoritative OD-010 occupancy log-odds **and separate observed/unknown mask**, submap-local transforms, global poses and the graph's nodes, constraints, uncertainty and reconstruction/calibration metadata. Support **lazy, independently bounded submap loads**. OD-007 EDT and OD-009 correlation pyramids are **optional, versioned and regenerable OD-011 caches**, not required to open a valid map. Require **major/minor schema compatibility**, SHA-256 integrity/size checks, strict bounded decoding and **atomic generation publication with crash recovery**. Keep ROS Nav2/PGM/YAML import-export and future single-file packaging separate from core persistence.

**Alternatives:** (A) **directory + JSON + binary selected** for transparent inspection, individual submap loading, clean versioning and independent compression; (B) YAML as canonical metadata — rejected; (C) single-file container — deferred **optional export/transport**; (D) SQLite/other database tile store — deferred until much larger maps or incremental transaction requirements justify the complexity. No initial database or custom monolithic container dependency.

#### Canonical bundle layout and intent

~~~text
building_a.slam/                         # root directory, not an in-place mutable map
  CURRENT                                 # atomic generation pointer (if checkpointed)
  generations/
    000000000001/
      manifest.json                       # strict schema and authoritative file index
      graph.bin.zst                       # nodes, submap poses, factors, uncertainty
      submaps/
        sm_000001.bin.zst                 # independent occupancy + observed mask
        sm_000002.bin.zst
      cache/
        sm_000001.edt.bin.zst             # OPTIONAL OD-007, source/config-keyed
        sm_000001.corr.bin.zst            # OPTIONAL OD-009, source/config-keyed
      preview.png                         # OPTIONAL non-authoritative visualization
      checkpoint/                         # only for resumable sessions
        state.bin.zst                     # mutable estimator/active-map/replay state
    000000000002/                         # next complete immutable generation
~~~

For a **one-shot export** a self-contained directory containing `manifest.json` and the referenced payloads MAY be used directly. For **repeated save/checkpoint** use uniquely named immutable generation directories plus a small atomic `CURRENT` pointer; **do not attempt an unsafe rename over an existing non-empty directory**. A reader resolves `CURRENT` once, then pins that generation and its manifest, avoiding mixed-generation reads. The final file layout and names are illustrative but the generation semantics are mandatory.

**Two bundle purposes** are explicitly distinguished by manifest `bundle_kind`:
- `LOCALIZATION_MAP`: frozen/local authoritative occupancy plus graph poses and sufficient calibration/frames to load a known map; no live continuation is implied. It may include loop data if needed to audit or reoptimize the saved graph, but cannot claim to reconstruct unrecorded sensor history.
- `RESUMABLE_CHECKPOINT`: includes all authoritative map/graph data **and** versioned **active submap builders, insertion IDs/counters, estimator bias/covariance/trajectory state, calibration/clock-domain/session mapping, replay cursor/input identity and consistent graph-optimization state** required to continue. A `LOCALIZATION_MAP` MUST NOT be silently accepted as resumable; an incomplete checkpoint MUST fail resume with a descriptive error.

#### Manifest schema and authoritative contents

The strict JSON manifest MUST contain:

| Category | Required semantics |
| --- | --- |
| Format | Magic/type, **major/minor version**, required/optional features and minimum compatible reader |
| Identity | Unique map ID, generation ID, deterministic content/source identifiers, creation provenance (wall-clock informational only) |
| Frames | REP-103/105-aligned map/odom/base/laser frame definitions, SE(2) transform conventions, grid-local origin **including yaw**, units, coordinate and covariance tangent ordering |
| Submaps | Stable ID, state (frozen/active where checkpoint), resolution, width/height/extent, local transform, insertion count, scan IDs/range/overlap, content hash, per-submap file |
| Graph | Graph schema revision, node and submap IDs/poses, factors, measurement types, robust/uncertainty model, correspondence/loop evidence, graph-pose revision and file |
| Calibration | Extrinsics, OD-001 canonical timestamp representation/clock domain/epoch identity, OD-002/003 clock-fit/calibration revision, OD-006 wheel/gyro model and bias settings, OD-004 trajectory convention, OD-010 log-odds and occupied/known thresholds |
| Index | Every required/optional file's normalized safe **relative path**, role, encoding/compression, **compressed size, uncompressed size, SHA-256 of stored bytes**, payload schema/version and optional uncompressed hash |
| Compatibility | Feature flags, codec requirements, decoder limits and migration requirements |
| Checkpoint only | Consistent acquired/replayed input offset/identity, active map and estimator/graph/clock state required for resume |

**Preserve authoritative information**: packed log-odds (encoding defined explicitly) and a **distinct observed-space bit mask**; a cell with observed `p≈0.5` is **not UNKNOWN**. Preserve submap local frames, exact grid origin rotation and width/height, immutable source hashes, per-submap accepted insertion IDs and freeze reason. Graph serialization must preserve not just best poses but **nodes, constraints, covariance/information, factor provenance, submap identifiers and revision** sufficient to reconstruct the supported graph semantics. The exact portable factor serialization is a separately versioned payload schema, not native GTSAM object memory. Persist the pose-table version consistently with occupancy and graph snapshot.

**Binary portability:** Define **field-by-field fixed-width** unsigned/signed integer sizes, floats (IEEE 754 only where specified), explicit little-endian (or documented explicit) byte order, units, count/length bounds, binary magic and payload version, index/cell order, transform and covariance ordering and NaN/Inf rejection. **Never write C++ object layouts, compiler padding, host pointers, platform `size_t`, raw Eigen memory or native GTSAM classes.** Use deterministic encoding/order for identical logical content where practical. A writer and reader must work across x86-64 and aarch64 and reject unsupported architectures/features rather than guess.

**Compression:** Use an **independently decompressible zstd frame per submap** (and separately per graph/cache/checkpoint file where compression is used). RFC 8878 documents portable lossless frames; it does not itself provide arbitrary random access *inside* one compressed stream, hence the independent file/frame layout. Manifest sizes/hashes protect the full stored files independently of optional zstd frame checksums. Decoder MUST apply **pre-open and streaming decompressed-size, memory/window and file-count limits**, not trust untrusted file sizes or claim that compression alone prevents decompression bombs.

**Derived cache policy:** Cache entries carry their own algorithm version, source authoritative occupancy hash/revision, known-mask/occupied-threshold configuration, EDT truncation/interpolation or correlative score/pyramid configuration. Cache missing, corrupt, stale or incompatible => **ignore/delete/rebuild safely** under OD-011's bounded background worker; **do not fail loading the authoritative map solely because an optional cache is bad**. A pose graph changing only `T_map_submap` does not invalidate local-frame occupancy or locally keyed caches; it may invalidate global spatial indexes.

#### Transactional save, crash safety and recovery

1. Capture a **consistent, immutable point-in-time map+graph snapshot**: all referenced submaps and graph poses/constraints/covariances must agree; a checkpoint additionally captures compatible estimator, active submaps, clock/session and replay progress. Pause/quiesce or coordinate a *snapshot barrier* where necessary, but **do not block acquisition** for slow compression/disk writes.
2. Create a **new unique staging directory on the target filesystem**. Write each payload independently; compute and check size/hash; write and schema-validate `manifest.json` **after** the complete authoritative file index is known. Bound bytes, count, time, memory and scratch disk space; report disk-full/io errors.
3. **Durability:** flush payloads and the manifest to stable storage (file `fsync`/`fdatasync` where appropriate), then sync their containing directories and the staging root as supported. Validate cross-file references, IDs, feature compatibility, bounded sizes/checksums and checkpoint completeness **before publication**.
4. **Publish an immutable generation** by renaming staging to a **previously nonexistent generation name on the same filesystem**, and durably sync its parent directory. For repeated saves, write a small temporary `CURRENT` file containing the new generation ID (and optional checksum), flush it, **atomically replace the `CURRENT` pointer**, and sync its directory. A one-shot named bundle is likewise published only at a **previously nonexistent destination**, or under an explicit generation scheme.
5. **Retention:** Keep at least one previously validated generation until the new pointer is durably published and validated. Garbage-collect old generations only after retention and reader pin policies permit. Do not overwrite/delete the only known-good checkpoint.
6. **Load/recovery:** Resolve `CURRENT` once; validate its generation's manifest and all **required authoritative** file indexes, sizes, hashes and references before exposing any graph/map to SLAM. If the current pointer/required data are invalid after an interrupted save, **scan only bounded allowed prior generations** and select the newest fully validated compatible one, reporting recovery provenance. Ignore incomplete staging directories. Never expose half-published or mixed-generation maps.

**Important filesystem limitation:** A rename is atomic **only under the target filesystem's supported semantics**, and atomic visibility is **not the same as durable recovery**. Durability relies on correct flush/fsync and filesystem/hardware behavior, including directory sync support. Cross-filesystem renames and some network/removable media have different guarantees; **detect unsupported operations and fail safely or document a weaker guarantee**, never promise universal power-loss atomicity. Replacing nonempty directories is not a portable atomic overwrite; the immutable-generation + pointer protocol avoids that ambiguity.

**Untrusted-file loading:** Reject absolute/parent-traversal paths, unexpected symlinks/hardlinks/links out of the bundle root, duplicate paths/IDs, path case-normalization collisions as applicable, directory recursion, malformed lengths/overflow, unexpected graph endpoints, corrupt checksums, unsupported codec/version/required features, nonfinite poses/covariances, mismatched grids/masks and resource-limit violations. Read/verify with bounded memory and **do not publish partially reconstructed runtime state**. Optional preview/cache corruption is nonfatal. SHA-256 provides integrity, **not source authenticity**; signed origin/authentication remains out of initial scope.

#### Version evolution, clock identity and ROS interop

- Use explicit **major/minor** versions and required feature flags. A reader must reject unknown **required** features or incompatible major versions, ignore safe unknown optional manifest keys and apply **version-specific readers or explicit tested migrations** for old versions. Migration writes a **new validated generation** and preserves the original.
- **OD-001 time identity:** Preserve signed nanosecond values **with clock domain, epoch/session origin and calibration mapping provenance**. A Pi process's monotonic timestamp number alone is **not portable across reboot/replay**. On resume, establish a new live-clock mapping or replay equivalent; do not assume old monotonic epoch equals the new process's. Distinguish captured acquisition event time from non-authoritative wall-clock bundle creation time.
- Maintain standalone **ROS 2 Nav2 YAML/PGM/OccupancyGrid export/import adapters** (and ROS 1/converters where useful) with explicit **loss reports**: flattened probability/unknown image alone cannot preserve active submaps, source scans, graph constraints, calibrated clock IDs or checkpoints. Keep core bundle APIs ROS-independent.
- Defer a single-file package as optional export/import (e.g. for distribution) and database-backed storage until measured needs arise, **without weakening the authoritative directory/manifest contract**.

#### Required Docker tests and benchmark B20 (not yet executed)

1. Round-trip multi-submap log odds and observed mask (including known neutral), rotated origin, signed nanosecond/clock epochs, accepted scan IDs, OD-010 freeze reasons and graph nodes/factors/uncertainties. Validate exact lossless authoritative payload identity.
2. x86-64/aarch64 endian/portable binary golden vectors, deterministic canonical bytes, explicit size/count bounds and independently compressed zstd submaps.
3. Required-file tamper/truncation/duplicate/missing references, SHA-256 mismatch, incompatible versions, unsupported required feature, malformed graph factor, path traversal/symlink escape, huge counts, decompression bombs and nonfinite floating data all fail closed.
4. Optional EDT/pyramid corrupt/stale/missing/config-mismatched => authoritative map loads and OD-011 cache rebuild is requested without data loss.
5. Power-cut/fault injection before/after file sync, generation rename, pointer sync and GC; reader never sees mixed/partial generation and can fall back to verified prior generation. Test distinct mount/unsupported filesystem handling honestly.
6. Lazy loading of several individually compressed frozen submaps under bounded memory; graph pose updates do not rebuild unchanged local EDT; source/config hashes enforce cache identity.
7. Distinguish `LOCALIZATION_MAP` from complete `RESUMABLE_CHECKPOINT`. Resume restores active 90/180 counters, bias/trajectory clock identity, recorder/replay cursor, graph revisions; map-only bundle MUST reject resume and restart monotonic clock alignment properly.
8. ROS/Nav2 import/export loss report and isolated adapters (no ROS dependencies in native core). Test on Docker x86 and Pi 5 where possible; benchmark save/read p50/p95/p99, peak RSS, compression ratio, lazy first-submap access and realistic disk-full handling.

**Consequences:** The initial persistent map representation, binary portability, compression, provenance, generation publication, recovery and map/checkpoint distinction are **settled**. Exact factor encoding fields, stable schema version assignments, cache selection and performance caps must be implemented/tested as a **documented version 1 schema**, not invented by automatic raw-object serialization. **OD-024** still governs periodic replay checkpoint/seek strategy; **accepted OD-013** governs graph topology and factor provenance; **OD-025** governs ROS distribution/QoS packaging. No runtime code or saved maps are yet implemented.

**References:** [Zstandard format RFC 8878](https://www.rfc-editor.org/rfc/rfc8878.html); [Nav2 Map Server](https://docs.nav2.org/configuration/packages/map_server/configuring-map-server.html); [SQLite WAL (deferred alternative)](https://www.sqlite.org/wal.html).

### OD-013 — Keyframe + submap SE(2) graph with conservative correlated-evidence admission

**State:** **Accepted** (topology and admission policy selected; C++ implementation, factor calibration and tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of a keyframe/submap graph and provenance-aware constraints  
**Priority:** P1

**Decision:** Optimize **keyframe global SE(2) poses `X_k`** and **submap global SE(2) poses `S_j`** in **GTSAM**. Initially add **one fused wheel/gyro relative-motion constraint per keyframe interval**, **one primary geometrically justified local keyframe-to-submap constraint**, and carefully validated **inter-submap/keyframe loop closures**. Maintain raw wheel/gyro-based continuous trajectory, stationary/online gyro-bias correction and OD-004 beam deskew **outside the graph initially** under OD-006; **do not create separate graph bias, wheel and gyro factors** for that same fused evidence. Preserve every factor's source IDs, timing, scan-insertion membership, shared motion intervals, OD-011 pinned map revision, OD-008 observability/covariance calibration and explicit correlation policy. **Do not treat measurements as independent merely because they were created by separate processing stages.** Keep graph-domain interfaces **ROS- and GTSAM-independent**; GTSAM only in the backend.

**Alternatives:** (A) pose-only wheel/scan graph — simpler baseline/ablation, less explicit submap connectivity; (B) **keyframe + submap Pose2 graph — selected** as a natural fit for OD-010 overlapping immutable submaps; (C) full joint pose/submap/gyro bias (or continuous trajectory) graph — **deferred** pending verified consistency/accuracy gains versus Pi/desktop cost. **Accepted OD-015** selects keyframe motion thresholds/anchors; loop retrieval remains OD-018, online iSAM2 scheduling OD-019.

#### Variables, gauge and graph topology

| Variable or record | Initial meaning |
| --- | --- |
| `X_k = T_map_base(t_k) ∈ SE(2)` | Global pose of keyframe at **OD-004 scan midpoint** |
| `S_j = T_map_submap_j ∈ SE(2)` | Global pose of **immutable OD-010 local** occupancy submap |
| `Z_odom(k,k+1)` | **One** relative SE(2) fused wheel/gyro motion observation over keyframe interval |
| `Z_lidar(j,k)` | LiDAR-only geometry-derived relative keyframe-to-submap pose, with rank/quality |
| `Z_loop` | Verified cross-submap/encounter relative constraint with evidence provenance |
| `Membership(k,j)` | Which exact accepted scan IDs entered which of the **two overlapping** submaps; **not** an automatic factor |
| `Anchor(j,k_anchor)` | Deterministic submap-local frame definition/initialization relationship; **not** another statistically independent measurement |

Use `T_A_B` mapping B-frame points into A, `X_k = T_map_base` and `S_j = T_map_submap`. For an admitted LiDAR-only local measurement `Z_jk = T_submap_base`, the predicted graph transform is `S_j⁻¹ X_k` and a compatible local Lie residual is `Log(Z_jk⁻¹ S_j⁻¹ X_k)` with specified tangent ordering and tested Jacobians.

**Gauge fixing:** Choose a consistent map gauge (e.g. set one initial keyframe or submap frame as the reference) with a **single documented fixed-reference convention**. An initial `S_j` may be **initialized from an anchor keyframe**, and its *local coordinate frame is definitionally tied to the builder frame*, but **anchor bookkeeping must not be added as an extra independent sensor likelihood**. If a deterministic exact relationship is explicitly represented, avoid redundantly constraining its derived variables and preventing legitimate global submap/keyframe adjustment. Test gauge observability and allow global loop optimization to change `S_j` relative to subsequently corrected `X_k`.

**Submap association:** OD-010's accepted scans 91–180 contribute to **both A and B**; maintain both insert memberships. To avoid duplicated local LiDAR evidence, choose at most **one primary local scan-to-submap factor for each keyframe and underlying source scan encounter**, even when that scan was inserted into both active submaps. The choice of primary submap can follow snapshot support/quality, with an explicit **pre-insertion** reference revision. Additional relationships do not automatically become independent constraints.

#### Factor-admission and correlation policy

| Shared evidence | Initial admission rule |
| --- | --- |
| Wheel encoders + gyro | **One fused relative-motion factor** per keyframe interval from OD-006; no simultaneous raw wheel, yaw and fused odom factors for those same samples |
| OD-006 fusion / OD-007 odom prior | The **prior-conditioned Ceres posterior pose is not a LiDAR-only measurement**; no posterior-as-independent-factor |
| Wheel/gyro-conditioned LiDAR deskew | Save motion-source interval IDs; apply **held-out-calibrated conservative LiDAR uncertainty inflation** and quality gating, acknowledging remaining cross-correlation |
| One scan in two OD-010 active submaps | Preserve both `Membership` records; **only one primary local scan constraint** per keyframe/encounter |
| Local matching to map already containing the query scan | **Never admit a new independent LiDAR/loop factor** from self-matched geometry; use OD-011 pinned pre-insertion revisions |
| Multiple loops against overlapping submaps | Suppress near-duplicate encounter/geometry factors; track shared source scans and overlap, conservatively inflate or reject |
| Repeated frozen-map use | Shared `S_j` variable models pose coupling, **not all correlations of repeated raster/beam evidence**; retain conservative uncertainty, provenance and admission caps |
| Degenerate scan match (OD-008) | Only a justified **observable-subspace factor** with explicit nullspace support, or **reject**; never artificially invert/floor null directions |

**Motion factor:** Integrate the OD-006 calibrated wheel translation plus gyro yaw over **the precise keyframe time interval**, with explicit source IDs, bias-calibration revision, covariance/information (with correct SE(2) tangent frame), and slip/timing quality. **Exactly one fused relative-motion likelihood per source interval** is the starting point. Its covariance captures the fused state uncertainty/correlation as conservatively calibrated; a delta computed from the same wheel and gyro data must not be included again under another factor type. If wheel data or gyro are absent, mark degraded and admit only evidence the available modalities actually support; do not pretend a full 3-DoF motion observation exists.

**Deskew/odom correlations:** Let `e_o` be the wheel/gyro relative-motion error and `e_l` a motion-conditioned LiDAR geometric error. Their actual joint covariance has the form:

~~~text
Sigma_joint = [ Sigma_odom    Sigma_cross   ]
              [ Sigma_cross^T Sigma_lidar   ]
~~~

Ordinary independent GTSAM factors do **not** capture nonzero `Sigma_cross` merely because `Sigma_odom` and `Sigma_lidar` are separate covariances. **Initial approximation:** preserve exact wheel/gyro sample intervals that influenced every deskewed scan; separately calibrate OD-008 LiDAR-only covariance (including time/deskew/model quality) and **inflate conservatively** for unmodelled cross-correlation. **Inflation reduces overconfidence but is NOT an exact joint covariance model**. Observe empirical consistency/NEES/loop errors on held-out truth; defer an explicit correlated/joint factor or full smoother until evidence shows the approximation insufficient.

**Ceres output into graph:** OD-007 minimizes **LiDAR + an odometry pose prior + boundary penalties**. Thus its optimized pose is **not independent LiDAR evidence**, and **subtracting the prior Hessian from its posterior Hessian does not recover an independent LiDAR covariance** (the optimum, map geometry and deskew are conditioned). For any admitted LiDAR graph factor, obtain **separately evaluated geometric evidence**: ideally a **LiDAR-only refinement restricted to OD-008 observable directions** around a valid candidate, or an explicitly justified conditioned measurement model. Validate map support, quality, innovation and **rank-aware covariance** before factor construction. If a valid independent-enough LiDAR geometry observation cannot be extracted, **skip** that factor; a regularized posterior may still be used for frontend tracking/initialization. Do not infer an independent factor merely from a residual score or inverted full regularized Hessian.

**Scan membership and loops:** A scan can be evaluated against an OD-011 **pre-insertion** map snapshot for local alignment, then inserted into OD-010 A/B; source scan IDs and submap membership are retained. A **new loop closure** must target a frozen/reference submap that **does not contain the query scan**, with source-scan intersection/overlap analysis (particularly scans shared by adjacent submaps). A locally pre-insertion scan can be useful for primary tracking without authorizing later **self-match as independent loop evidence**. Robust losses are a final safeguard **after geometric verification**, not permission to insert questionable loops. Suppress duplicate pair/encounter constraints and control repeated shared-raster evidence.

**Graph factor keys:** Use stable native graph IDs (trajectory, keyframe, submap, factor and encounter), source-sample/scan ranges, OD-003 time/OD-006 bias revision, OD-010 insertion memberships, OD-011 occupancy/EDT snapshot/content hash, OD-007 geometric model and OD-008 uncertainty/observable-rank revision. Store factor type, measurement frame, noise model/robust loss, source provenance, **correlation group ID** and deduplication/accept/reject reason. The **OD-012 JSON/binary bundle** persists these *portable* graph records, **never native GTSAM factor objects**.

#### Optimization, robustness and publication

- **Batch first:** run GTSAM batch **Pose2 graph** optimization offline with verified analytic/numeric Jacobians, correct gauge, SPD full-rank motion noise and appropriate rank-aware LiDAR factors. An **iSAM2 asynchronous incremental backend** is a future mode (OD-019); factor topology/provenance and results stay identical at the native boundary.
- **Loop closure:** OD-018 candidate retrieval -> OD-009 multi-resolution candidate search -> OD-007 geometric refinement (recovery-mode prior policy) -> OD-008 observability/inlier/residual and uncertainty validation -> OD-013 correlation/scan-membership admission -> GTSAM robust factor. Reject ambiguous, rank-insufficient, temporally inconsistent or highly redundant loops; robust M-estimation **does not make a wrong loop safe**.
- **Graph outputs:** Publish a **new immutable graph-pose revision** (`X_k, S_j`, source/factor/config IDs and diagnostics) only after optimization validity/convergence and loop sanity checks. **Never mutate OD-010 frozen occupancy** or in-place OD-011 local EDT/correlation cache; only `T_map_submap` and map-to-odom global correction change. High-rate `T_odom_base` remains continuous (OD-006); global `T_map_odom` may change, with freshness/quality handled by OD-022. Graph optimization must not block acquisition/recording/OD-006 prediction.
- **Out-of-graph motion/bias:** OD-006 remains the live/fine-timescale wheel/gyro trajectory and bias estimator. Do not introduce duplicate bias states or raw yaw factors inside the initial Pose2 graph. OD-014 may offline re-deskew/re-map **from raw measurements** and create a **new versioned map**, not silently rewrite the old frozen submap raster.
- **Validation of graph uncertainty:** OD-008 full-rank covariance, observable directions, inflation/calibration revision and degeneracy admission apply to LiDAR constraints. Monitor graph residuals and **held-out relative/absolute errors with genuinely independent reference**. A pose-graph Hessian confidence report under assumed independent factors is **not** proof of calibrated SLAM uncertainty.

#### Implementation boundary and tests (not yet executed)

~~~text
OD-006 local wheel+gyro estimator -> fused relative keyframe odom (1/interval)
OD-004/005 beams -> OD-007 frontend -> OD-008 geometric rank/quality
OD-010 overlapping submaps -> immutable OD-011 snapshots + membership ledger
                               |
                native GraphFactorAdmission:
               source/provenance intersection, rank/covariance,
                 dedup, self-match and correlation rules
                               |
                   native PoseGraph records:
               X_k, S_j, admitted factors and source IDs
                               |
                private GTSAM batch backend (initial)
                               |
                graph pose revision + OD-012 persistence
                        |                       |
            global map->odom correction    local map poses only
               (odom continuous)           (frozen raster unchanged)
~~~

**GTSAM- and ROS-independent native interfaces:** `PoseGraphModel`, `GraphFactorSpec`, `EvidenceProvenance`, `GraphAdmissionResult` and `GraphOptimizationResult` carry stable IDs/SE(2)/calibrated covariance/subspace info and graph revisions, never `gtsam::` types, Ceres residual pointers, ROS messages or sensor SDK classes. Isolate GTSAM translation into a backend target with no dependency leakage into core or OD-012 binary format.

**Required Docker Catch2/CTest and benchmark B21:**

1. Known synthetic `X_k/S_j` graph, gauge, Lie residuals and Jacobians; compare GTSAM batch solution with analytic truth and simpler pose-only baseline, across distorted local keyframe and submap initializations.
2. **One fused wheel/gyro factor per keyframe interval**; duplicate source IDs or independent raw gyro/wheel plus fused odom rejected. Bias changes trigger versioned derived motion, not double factor addition.
3. Repeated optimization or matching **same scan** never accumulates new independent information. Direct self-matches/loop factors where query scan is a member of target submap rejected.
4. OD-010 two active submaps with scans 91–180 shared; both memberships recorded but **one primary local LiDAR factor per keyframe**. Adjacent-overlap duplicate loops suppressed and recorded.
5. OD-007 strong odom prior: **posterior pose and Hessian subtraction are NOT graph-factor extraction**; test LiDAR-only/observable-subspace refinement, abstention on inadequate geometry, correct OD-008 rank/covariance and correlation source tags.
6. Synthetic aggressive in-place rotation/translation and injected gyro bias/slip/timing shifts: cross-correlation affects covariance coverage. Inflation calibrated on held-out independent truth, with explicit admission rejection when overconfident.
7. Corridor/warehouse rank 1/2 LiDAR, ambiguous loop, repeated wrongly correlated loop candidates and outliers: no false full-rank factor, no silent wrong map correction, robust loss cannot bypass prevalidation.
8. Loop closure optimization changes global `S_j/X_k` and `T_map_odom` without modifying frozen local occupancy or continuous `T_odom_base`; snapshot revisions atomic and graph worker off acquisition path.
9. Portable OD-012 graph roundtrip of node/submap IDs, admitted/rejected factors, source membership/scan IDs, OD-008 calibration, correlation groups, and no raw GTSAM object serialization.
10. Report ATE/RPE, yaw drift, loop acceptance/false loop rate, held-out NEES/coverage where valid, graph node/factor count, optimization p50/p95/p99, RAM/CPU, mapping throughput and graph-pose revisions on x86-64 and Pi 5 if available. Test no Actions dispatch.

**Consequences:** **Keyframe/submap Pose2 topology, fused interval odometry, single primary local LiDAR evidence, explicit motion/deskew/overlap correlations, conservative factors, batch-first GTSAM and native interface isolation** are accepted. **Accepted OD-015** sets initial keyframe thresholds (unvalidated); loop retrieval (OD-018), online iSAM2 budget (OD-019), correlated joint models and calibrated uncertainty remain benchmarks/extensions. This selection is a specification, **not** evidence that graph code exists or its uncertainty is calibrated.

**References:** [GTSAM BetweenFactor Pose2](https://borglab.github.io/gtsam/betweenfactor/); [Cartographer PoseGraph2D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/2d/pose_graph_2d.h); [Cartographer graph constraint tags](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/proto/pose_graph.proto).

### OD-014 — Bounded offline refinement with selective re-deskew and submap rebuild

**State:** **Accepted** (design chosen; baseline and iterative implementation/tests pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit selection of selective, quality-gated outer refinement  
**Priority:** P1

**Decision:** Implement **one complete deterministic offline replay, map construction and OD-013 batch pose-graph optimization first**. Add an optional **bounded outer refinement loop with at most three ADDITIONAL candidate iterations after the initial pass** (initial baseline result `R_0`, candidates `R_1` through `R_3`; maximum four full graph-solve passes if each candidate reaches a solve). Re-estimate **local OD-006 trajectories and, only when independently observable, gyro bias and OD-002/003 clock offsets/drift**. Selectively **re-deskew** OD-004 scan beams if their within-scan geometry changes materially and **rebuild only submaps whose local scan geometry or relative scan placements change**. Purely rigid global or local submap-frame pose corrections change **OD-013 graph pose variables only**, with **no unnecessary re-deskew or occupancy rebuild**. Publish only complete, validated **immutable iteration generations**; reject/regress/rollback candidate results without mutating accepted raw or map state. Orchestration and domain APIs are ROS-independent; Ceres and GTSAM remain in private implementations.

**Alternatives:** (A) single-pass pipeline — required deterministic **baseline**, not the only final refinement option; (B) **bounded selective iterative refinement — accepted**; (C) unconditional full remap after every graph solve — rejected as wasteful/self-reinforcing; (D) full real-time coupled bias/clock/submap estimator — deferred pending controlled evidence.

#### What triggers processing?

| Change between accepted iteration and proposed candidate | Re-deskew? | Rebuild occupancy? | Required operation |
| --- | --- | --- | --- |
| Pure global OD-013 pose-graph rigid transform `T_map_submap` | **No** | **No** | Publish new graph/global index revision only |
| Uniform rigid transform of all poses within one local submap | **No** | **No**, if every scan-to-submap relative transform and local ray geometry is unchanged | Update reference/global transforms |
| Relative keyframe/scan midpoint poses inside one submap change | Only if intra-scan trajectory also changes | **Yes if material**, since local insertion rays move | Recompute affected local scan-to-submap transforms, rebuild those submaps |
| Beam-relative trajectory changes (translation/yaw/twist, gyro bias) | **Yes if material** | **Yes for affected member submaps if ray geometry changes** | Recompute beam endpoints from raw time-stamped measurements |
| OD-002/003 clock offset/drift or beam-time model changes | **Yes if observable and material** | **Yes if local ray geometry changes** | New time-calibration version; retimestamp, reconstruct OD-004 motion/deskew |
| Corrections below geometric threshold, with no validity/quality change | Usually **No** | Usually **No** | Retain prior geometry; still evaluate graph convergence/quality |
| Invalid time coverage, changed scan eligibility, OD-005 return classification | Reprocess affected scans as needed | Rebuild affected submaps if source/geometry/membership changes | Preserve previous accepted generation until candidate is validated |

**Initial trigger:** Measure the **maximum Euclidean displacement of any valid hit endpoint in the *same submap-local reference frame*** when using candidate versus accepted scan-relative geometry, including laser extrinsics and time correction. Start with **0.5 × 0.05 m = 0.025 m (2.5 cm)**, **unvalidated and configurable**. A correction of global pose alone does **not** count if all beam/scan points are unchanged in the local grid. Angle alone is not sufficient: yaw change produces endpoint displacement proportional to range. Evaluate all valid beam geometries or a **conservative provable upper bound**, not an unrepresentative average or sparse sample. For a scan reused in two OD-010 submaps, mark **each affected membership**; effects must be measured in its own local frame.

Do **not** use the 2.5 cm threshold to silently ignore **clock epoch changes, incomplete trajectory coverage, invalid gyro calibration, OD-008 severe quality regressions, or changes in scan eligibility**. These are correctness gates that can force invalidation/rejection even if a geometric displacement estimate is small. Distinguish **changed scan midpoint pose** from **changed intra-scan relative motion**: re-deskew is needed for the latter, while a change to the relative midpoint-to-submap insertion transform may require only remapping.

#### Outer iteration, provenance and convergence

~~~text
Raw immutable LD19 packets/beam times, IMU, encoders, calibration & epochs
          |
   R_0: deterministic full replay -> OD-003/004/006 motion/deskew
      -> OD-005/007/008 scan selection and OD-010 overlapping submaps
      -> OD-011 derived caches -> OD-013 factor admission & batch graph
      -> validation -> ACCEPTED baseline generation R_0
          |
 R_(i-1) accepted graph/trajectory + candidate observable updates
          |
   Candidate R_i (i=1..3):
      recalibrate bias/time ONLY with independent observability evidence
      recompute local trajectory/beam times and per-beam displacement
      re-deskew affected scans; recompute local insertion rays/scan transforms
      rebuild affected OD-010 submaps as NEW local immutable revisions
      rebuild dependent OD-011 EDT and OD-009 score pyramids
      invalidate/recompute OD-007/008 matches and OD-013 factors affected
      enforce OD-013 scan membership, correlation and dedup from raw sources
      batch optimize; validate convergence and held-out/independent checks
            |
         improved? ---- yes --> publish immutable R_i and continue
             |
            no --> discard candidate, keep R_(i-1), stop or retry
~~~

- **State preservation:** `R_0` and every accepted `R_i` include exact raw data/source checksum references, OD-001 clock-domain/session identity, OD-002/003 time-fit versions, OD-004 trajectory/beam-time convention, OD-006 bias/calibration revision, OD-005 scan eligibility and stable scan IDs, OD-010 submap membership/insertion order and freeze reasons, OD-011 occupancy/cache revisions, OD-013 graph factor/source groups, software/config hashes and benchmark metrics. **OD-012** publishes complete new map generations transactionally; **the map bundle alone is insufficient** for re-deskew: raw recording/MCAP sensor streams and original acquisition/synchronization metadata must remain addressable and immutable.
- **Determinism:** Hold selected scan IDs, replay order, seeds and configuration fixed **where validity permits**, with changes to eligibility/membership explicitly recorded and justified. Use stable source IDs and versioned dependency/invalidations. A rebuilt submap must be re-created from **raw scan insertion evidence**, never by transforming/copying its old occupancy probabilities, because repeated log-odds additions would double-count observations and lossy raster warping would alter occupancy/unknown semantics. Do not mutate frozen submaps already referenced by an accepted map bundle.
- **Selective invalidation:** Pose-graph-only changes update graph poses/indexes, **not** immutable OD-010 local grids or OD-011 local EDT/pyramids. Geometry changes rebuild affected submaps **from scratch** with original ray hit/miss/unknown update semantics and stable insert IDs, then rebuild matching caches and **all** dependent local and loop-closure candidate factors that referenced old geometry. Invalidated factor IDs are **replaced, not appended**; no stale EDT/score pyramid, old membership, original factor or rejected loop survives as a supposedly independent new factor. Frozen-source references and source-scan intersections must be revalidated (OD-013).
- **Calibration observability:** OD-006 gyro bias correction only if stationary/independent evidence and uncertainty justify it; **wheel/gyro disagreement alone cannot identify bias vs slip**. OD-002/003 clock offset/drift changes require constrained multi-sensor timing evidence and accepted physical timestamp-delay model; the **LD19 30-second counter's beam-acquisition event remains experimentally uncertain** until bounded. Never optimize time using only the same LiDAR map fit and then claim independent calibration certainty. Refit only valid **same-epoch** mappings, preserve timing covariance, cap parameter changes and reject unobservable/time-inconsistent candidates.
- **Convergence and acceptance:** Stop on first non-improving/invalid candidate, when no local geometric update exceeds the trigger and graph metrics have converged, or after three additional outer iterations. Define configurable **relative graph objective improvement and maximum local pose-change tolerances**, but **do not compare raw total graph objectives from different maps/factor sets as if directly comparable**. Evaluate matched common measurements, held-out scans/routes and independent truth where available, loop residual/inlier/false-closure quality, OD-008 observability/covariance consistency, acquisition uncertainty and map quality/coverage. Require positive or non-regressing independent quality evidence **and** no safety/uncertainty regressions before accepting; a lower same-data residual on a rebuilt map is not sufficient proof.
- **Transactional iteration:** Build and test each `R_i` in a separate staging workspace, pin source revisions, validate cache/source consistency and graph factor provenance, and publish **only via OD-012 atomic immutable generation** when successful. Rejected, failed, non-finite, timing-invalid or budget-exceeded candidates leave `R_(i-1)` authoritative. Log stop/rejection reason and dependency graph for replay and Web Lab. The outer iteration runs offline under explicit work/memory budgets; it does **not** make real-time acquisition wait.
- **Isolation:** Orchestration interfaces such as `OfflineRefinementCoordinator`, `RefinementPlan`, `IterationSnapshot` and `RefinementDecision` are native C++20 with typed revisions/SE(2) and no ROS/Ceres/GTSAM/vendor objects. Ceres matching and GTSAM batch solve remain behind OD-007/013 adapter interfaces; offline work may use Docker on a workstation, while Pi5 background CPU/thermal impact is measured before any optional onboard execution.

#### Required tests / benchmark B22 (not yet executed)

1. One-pass `R_0` baseline and at most **3 additional** iterations, exact source/config seed/replay order, deterministic accepted hashes and no unbounded remap loop.
2. Pure loop-closure/global rigid submap shift, and uniform local rigid transform: **zero re-deskew and zero local-grid/caches rebuild**; only poses/global indexes revise.
3. Changed relative scan-to-submap midpoint poses but unchanged per-beam relative trajectory: remap affected submap **without unnecessary re-deskew**.
4. Inject true within-scan translational/yaw/gyro-bias/time offset/drift changes: calculate **worst-case submap-frame endpoint displacement** and trigger when `>0.025 m` initial threshold, including distant beams, laser lever arm, scan midpoint and two overlapping submap memberships.
5. Raw/OD-005 packet and invalid-return evidence restored exactly; no map insertion from missing rays or double-strengthening occupancy; rebuilt log odds/known masks reproduce full replay and fresh OD-011 caches.
6. Unknown/unobservable gyro-bias or LD19 clock-delay parameters held fixed, rejected erroneous updates, bounded epoch/time uncertainty and deliberately bad calibration candidate rollback.
7. OD-013 factor IDs/provenance replaced, not duplicated; rebuilt scan is never matched to itself or admitted as independent overlapping/loop evidence. Check candidate factor/graph revision coherence.
8. Wrong loop, lower posterior cost but worse independent held-out truth/coverage, covariance overconfidence or nonfinite graph: reject candidate and retain previous accepted OD-012 generation intact.
9. Compare selective vs unconditional full rebuild and one-pass baseline on exact synthetic apartment/warehouse; report ATE/RPE, beam alignment, map-cell disagreement, cache rebuild count, false closures, objective on common observations, CPU/RAM and outer-loop convergence/stop reasons.
10. Verify no ROS dependency in coordinator, frozen-map hashes unchanged under rigid changes, deterministic resume/replay source references, staging failure/abort recovery and no GitHub Actions run.

**Consequences:** The one-pass first milestone, **three additional-iteration cap**, 2.5 cm geometric candidate trigger, selective re-deskew/remap, observability-gated bias/time corrections and transactional validation/rollback are accepted. Thresholds, graph convergence parameters, calibration identifiability, workload limits and measured gains remain benchmarking tasks. **OD-024** still controls replay checkpoints/seeks, **Accepted OD-015** governs motion keyframes; **OD-018** loop candidate retrieval remains open.

### OD-015 — Motion-threshold keyframes, quality adaptation and optional sparse gyro-bias knots

**State:** **Accepted** (policy selected; implementation, numerical tuning and offline knot validation pending)  
**Decision date:** 2026-10-30  
**Decision authority:** User's explicit motion/quality and sparse optional knot selection  
**Priority:** P1

**Decision:** Create timestamped OD-013 **keyframe Pose2 nodes** using deterministic SE(2) **translation/rotation/elapsed-moving-time eligibility**, with an **uncertainty-aware secondary request** and a quality/information/observability gate for associated **LiDAR factor admission**. Initial parameters: **0.20 m** translation, **5° yaw** (`pi/36 rad`), **2.0 seconds during meaningful motion**, **0.2-second minimum keyframe interval**; do **not** make periodic keyframes while stationary without a specific observable constraint. Always maintain independent `X_k=T_map_base(t_k)` and `S_j=T_map_submap_j` variables, and create/associate a **submap anchor keyframe** when OD-010 starts a new submap, without double-counting its source evidence. **OD-010 submaps still advance at 90/180 accepted map insertions**, irrespective of graph keyframe count. Keep the evolving OD-006 gyro bias **outside the initial GTSAM graph** (zero graph bias knots). A **future optional OD-014 offline joint-refinement variant** may add scalar, time-indexed gyro-bias knots with random-walk evolution, initially every **5–10 s**, only after independent observability and held-out accuracy/consistency evidence justifies its complexity. Native selection and state APIs remain ROS-/GTSAM-independent.

**Alternatives:** (A) every scan or time-only keyframe — rejected as wasteful during stationary/poor-geometry periods; (B) **translation/rotation/meaningful-moving-time with uncertainty adaptation — accepted**; (C) information-gain-only online selection — deferred until information quality and cost are validated; (D) one gyro-bias variable per scan — rejected; (E) **no graph bias knots initially, optional sparse offline random-walk knots — accepted** subject to measurements; (F) joint bias/pose/continuous scan model in online graph — deferred.

#### Initial keyframe and bias policy

| Item | Initial selection | Interpretation |
| --- | --- | --- |
| Translation trigger | **0.20 m** | Body/planar SE(2) displacement since last keyframe; configurable |
| Rotation trigger | **5° = pi/36 radians** | Absolute shortest relative yaw, wrap-safe |
| Moving-time fallback | **2.0 s** | Only if meaningful motion persists; no idle heartbeat keyframes |
| Minimum interval | **0.2 s** | Regular keyframe admission floor; submap-anchor exception explicitly recorded |
| Stationary keyframes | **Suppressed** | Unless special constraint/observable experiment explicitly requires |
| Uncertainty | **Secondary trigger**, bounded | High uncertainty requests consideration, not unconditional node growth |
| Information gain | Optional future trigger | No unvalidated formula or fabricated information metric |
| Submap creation | **OD-010 90/180 map insertions** | Independent of graph keyframe selection |
| New submap anchor | **Associate/create one anchor keyframe** | Explicit exceptional reason if inside min interval; no duplicate evidence |
| Initial graph gyro-bias knots | **0** | OD-006 estimator owns bias and trajectory |
| Future offline bias-knot spacing | **5–10 s initially** | Only when observable, justified by drift dynamics, not every scan |
| Future bias dynamics | Scalar yaw-axis random walk | `Var(b_(j+1)-b_j) = q_b * Delta_t_j` |

All numeric thresholds are **starting values, not measured Pi 5 or robot defaults**. Time comparisons are typed OD-001 **canonical integer-nanosecond** differences within a consistent domain/epoch. Use **scan midpoint timestamp** (OD-004) for a keyframe, deterministic ordering and source IDs (OD-005), and avoid crossing paused/reset simulated-clock epochs. A wall-time/system uptime tick is not an acquisition-time substitute for the 2 s rule.

#### Deterministic eligibility versus factor admission

1. For each eligible OD-005 scan with valid OD-004 midpoint pose, calculate `Delta=T_last_keyframe^-1*T_candidate` with the accepted SE(2) tangent convention; measure Euclidean translation and wrapped absolute yaw. Request a keyframe when **any** of distance `>=0.20 m`, yaw `>=5°`, **elapsed >=2 s with meaningful motion**, or independently justified rising prediction uncertainty triggers. Require **elapsed >=0.2 s** since the last keyframe for normal admission. If the floor blocks a threshold request, log **DEFERRED_MIN_INTERVAL** and re-evaluate subsequent eligible scans rather than silently violating it. Initial/root keyframe and OD-010 anchor events are explicit exceptions.
2. **Meaningful motion** for the elapsed-time trigger must be established by verified translation/rotation or the OD-006 predictor over the period, with configurable floor and source quality. Remaining stationary for hours **does not** create a new keyframe at every 2 s. If bias calibration benefits from stationary data, OD-006 may use **raw IMU** without adding pose graph nodes.
3. Evaluate OD-006 prediction covariance/innovation and OD-008 LiDAR geometry support/overlap/rank/quality. Elevated uncertainty may request **earlier bounded pose sampling**, **but cannot turn a rank-deficient scan into reliable LiDAR evidence**. If uncertainty is high and new geometry is uninformative, keep tracking/odometry continuity and trigger OD-009 recovery/OD-022 degraded state instead of making many empty/unconstrained keyframes. Resource-admission limits and deduplication prevent runaway graph growth.
4. **Keyframe node admission and LiDAR factor admission are distinct**: a keyframe may be needed to preserve a continuous fused OD-006 relative-motion chain or submap anchor even when its OD-008 LiDAR match fails; mark pose/quality status and admit **no independent LiDAR factor** until geometry, rank and shared-evidence policy pass. Conversely, accepting a scan for **OD-010 map insertion** does **not** necessarily create a keyframe. Never treat a keyframe-selection threshold as proof of measurement quality.
5. Build **one fused OD-006 wheel/gyro relative-motion factor per successive keyframe interval** using exactly the original timestamped source samples spanning that interval, with calibrated bias/uncertainty, freshness and overlapping-source provenance; no duplicate factors when an earlier candidate was deferred. Preserve OD-013's one-primary-LiDAR-factor-per-encounter, self-match prohibition, and shared A/B submap membership.
6. At each OD-010 new-submap event **after its own 90 accepted insertions**, find a stable keyframe at or near the boundary (create a special **ANCHOR** at the valid event timestamp if necessary), initialize `S_j` from the local builder's anchor frame and retain association. An anchor keyframe is **not** a second local LiDAR measurement; split/refactor motion intervals without duplicating raw wheel/gyro likelihood if a forced anchor falls between existing pose nodes. Report any minimum-interval override. **Submap spawn/freeze counters remain completely independent** of graph node count; a keyframe's scan may belong to A and B, but overlap is not evidence for two independent local factors.
7. Apply the same deterministic, documented selection policy during OD-014 raw replay, or record a **new versioned keyframe selection** when trajectory/quality changes alter thresholds. Rebuild affected relative odom intervals, scan/anchor/submap associations and OD-013 factors using original source IDs; never append duplicates from the previous generation.

#### Sparse optional offline yaw-bias knots (future extension, NOT initial graph)

Keep `b_g(t)` in OD-006's local wheel/gyro integrator initially; GTSAM graph variables are only `X_k, S_j`. The optional future OD-014 offline model introduces **time-indexed scalar yaw gyro-bias knots** `b_j` on one acquisition clock epoch, initially separated **5–10 s** and adapted only to **measured drift, temperature, and identifiable information**. The assumed bias random walk is:

~~~text
b_(j+1) = b_j + w_j
w_j ~ N(0, q_b * Delta_t_j)
Var(w_j) = q_b * Delta_t_j
q_b >= 0, units (rad/s)^2 / s; Delta_t_j measured in seconds
~~~

- Bias-knot spacing is **independent of the approximately 10 Hz LiDAR scan rate** and graph keyframe density. The 5–10 s window is a tunable first experiment, not a validated bias drift timescale. If sensors are stationary, OD-006 may calibrate bias without any keyframe or knot. Add knots only for identifiable drift/temperature changes, not to chase map-fit noise.
- Any optional knot must be correctly **coupled to the yaw-motion measurement prediction and its covariance/cross-correlation**, rather than adding a random-walk prior on unused independent `b_j` variables. Develop a **planar wheel+gyro bias-aware relative-motion factor** (or equivalent joint model) with tested Jacobians and interval integration, source provenance and bias interpolation/hold policy. A GTSAM 3D IMU-preintegration API is **not automatically an appropriate drop-in** for this planar encoder+gyro model.
- **No double-counting:** If a knot-conditioned joint odometry factor replaces the initial bias-corrected fused odometry constraint, remove/supersede the original factor for the corresponding interval. Re-deskew and selectively rebuild affected OD-010 local maps through OD-014 if the changed bias modifies beam geometry, and use OD-008 calibration/OD-013 correlated-factor gates. Do **not** add separate yaw-bias corrections next to an already bias-corrected fused motion factor and treat them as independent.
- An unobservable bias (e.g. wheel slip, straight corridor with poor yaw geometry, clock uncertainty or absence of sufficiently independent stationary/revisit evidence) must remain **fixed or regularized to genuinely measured prior uncertainty**; do not introduce unconstrained knots or claim low posterior covariance. Validate against **held-out independent yaw/motion truth**, not only reduced training scan-to-map residual.
- Persist source time/epoch, calibrated bias/uncertainty, knot spacing and timestamps, random-walk `q_b`, affected raw gyro sample intervals, factor revisions and model-version IDs in OD-012 future-checkpoint/schema extension. OD-001 clock jumps/epochs never share a single unqualified `Delta_t`.

#### Architecture, tests and consequences

~~~text
OD-005 scans + OD-004 midpoint / OD-006 prediction
       -> native KeyframeSelector: motion or meaningful moving-time threshold
       -> min-interval + uncertainty/quality/node-budget gates
           |                          |
      X_k admitted              no keyframe; keep odom continuity
           |
    OD-013 one fused relative odom factor since previous keyframe
           |
     OD-008 LiDAR factor?    [YES geometry / NO unsupported]
           |
    OD-010 independent 90/180 active-submap boundary
           -> ensure anchor association; S_j independent Pose2
           |
     batch OD-013 graph initially: X_k and S_j only
           |
    optional OD-014 offline experiment: sparse b_j (5–10 s)
           -> bias-aware interval factors + selective re-deskew/remap
~~~

Native C++20 `KeyframeSelector`, `KeyframePolicy`, `KeyframeDecision`, `SubmapAnchorAssociation` and optional `BiasKnotPlan` inputs/outputs depend only on timestamped SE(2), covariance/OD-008 diagnostics, source/scan IDs and revisions, **not on ROS/GTSAM/Ceres or individual sensor driver APIs**. The backend privately creates GTSAM Pose2 variables and factor objects; clock and raw motion uncertainty stay explicit.

**Required Docker Catch2/CTest and benchmark B23:**

1. Exact comparisons at **0.20 m**, **5°**, **2.0 s while moving**, **0.2 s min interval** including below/equal/above, yaw wrap, acquisition timestamp discontinuity and deterministic ties.
2. Stationary/repetitive observations cause **no 2 s keyframe flood**, while small but meaningful motion gets a bounded elapsed-time keyframe. Bias calibration can continue without graph pose nodes.
3. High uncertainty with no observable LiDAR info triggers OD-009 degradation/recovery **not a chain of artificial LiDAR factors or unbounded nodes**. High uncertainty plus informative geometry may request keyframes within resource caps; source and reason logged.
4. Compare OD-010 submap insertion 90/180 counts **unchanged** when keyframe thresholds vary. At new submap boundary a valid anchor is associated/created; if within 0.2 s mark explicit exception and no duplicated fused factor. Both A/B memberships retained, one primary local LiDAR factor.
5. One fused wheel/gyro motion factor across actual successive keyframe intervals, even with deferred/missed scans, correct covariance/source ID nonoverlap; rejected LiDAR geometry does not enter graph.
6. Optional `b_j` 5/10 s random walk: `Var(w)=q_b*Delta_t` scaling, bias interpolation, random-walk factor Jacobians, temperature/drift cases, unobservable bias held fixed. After replacing initial odom with bias-aware factors no duplicate interval evidence.
7. OD-014 reselected keyframes and changed bias timing rebuild affected factor/scan provenance with deterministic replay and selective map updates; OD-012 localization map/optional future checkpoint serialization preserves model revision.
8. Metrics: graph node/factor density, motion/loop/ATE/RPE and covariance truth, held-out calibration/observability, CPU/RAM and batch/iSAM2 prospective budget across synthetic apartment/warehouse and Pi 5; no GitHub Actions.

**Consequences:** OD-015 fixes **motion-based keyframe defaults and independent submap anchors**, uncertainty-gated factor admission and **zero initial graph bias knots**; sparse 5–10 s bias knots and random-walk coupling are an **optional offline future extension**, not an instruction to add new initial GTSAM variables. Numerical thresholds/bias dynamics and benefits remain unmeasured; OD-016/017/018/019 still control execution layout, target APIs, loop retrieval and incremental compute budgets. **Cartographer's motion filter is a related design reference, not a claim that its scan-insertion filter is our graph keyframe policy.**

**Reference:** [Cartographer 2D motion-filter configuration](https://github.com/cartographer-project/cartographer/blob/master/configuration_files/trajectory_builder_2d.lua), [Cartographer motion-filter source](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/motion_filter.cc).

### OD-018 — Loop closure retrieval, verification and rejection

**Question:** Keyframe scan descriptor, submap occupancy signature, spatial proximity or branch-and-bound candidate retrieval?

**Provisional preference:** candidate retrieval over frozen submaps, geometric coarse alignment, Ceres refinement and robust graph gates; exclude local neighbors from loop candidates.

**Decision gate:** synthetic repeated shelving false positives, apartment true loop recall, deliberate dynamic clutter, graph consistency after wrong candidate rejection. Log candidate scores, revisions, residuals and rejection reasons.

## 5. Runtime, integration and localization decisions

### OD-016 — Processes and threads

**Question:** One process with bounded worker threads, or separate acquisition, recorder, localizer, graph and Web Lab processes connected through native IPC?

**Provisional preference:** small native process with bounded distinct workers for first recorder/replay; keep interfaces usable across processes when required by robustness. Web Lab should be opt-in and unable to stall acquisition.

**Decision gate:** Raspberry Pi 5 CPU scheduling, crash/isolation behavior, memory copies, queue backlog, serial device permissions and Docker Compose complexity. Do not pretend C++ thread safety is guaranteed by Docker isolation.

### OD-017 — CMake libraries, APIs and build boundaries

**Question:** How many initial libraries/targets and which modules are public/installable?

**Provisional preference:** `fusion_slam_core`, sensors/time/motion, mapping/frontend/backend adapters, apps, test fixtures; public native headers behind installable CMake targets with optional ROS target. Catch2 v3 and cxxopts pinned via FetchContent.

**Decision gate:** C++20 Docker image builds without ROS, Catch2 tests discoverable with CTest, cxxopts help/smoke executable, no global compiler flags, no unapproved Boost. Avoid 12 empty targets before real code exists.

### OD-019 — Online iSAM2 compute and graph growth

**Question:** Full incremental graph, bounded/local fixed-lag smoother, submap-level graph, factor pruning or periodic batch cleanup?

**Provisional preference:** async iSAM2 graph with explicit keyframe cap/optimization budget and submap-level summaries; defer full fixed-lag complexity until Pi measurements.

**Decision gate:** sustained 10 Hz LiDAR inputs on Pi, p95/p99 mapping/odometry latency, RSS growth, graph update lag, loop closure impact and safe degradation when compute is exhausted.

### OD-020 — Global localization / kidnapped robot recovery

**Question:** Does accepted OD-009 branch-and-bound with several hypotheses suffice for kidnapped-robot/global recovery, or is persistent particle filtering required?

**Provisional preference:** use accepted OD-009 asynchronous branch-and-bound + OD-007 refinement + OD-008/022 multi-scan validation first; only benchmark a persistent AMCL/Beluga-style particle baseline if global repeated-aisle ambiguity or kidnapping shows a material unsolved failure mode.

**Decision gate:** unknown initial pose, robot teleportation, repeated aisles, dynamic obstacles, success rate, false-lock rate, CPU/memory, time to stable lock and multi-scan consistency. Never call single-match convergence a global relocalization.

### OD-021 — Bounded queues, drop and recorder durability

**Question:** Queue capacities and overload semantics for acquisition, MCAP writer, scan processing, backend and UI?

**Provisional preference:** separate bounded channels with explicit policy: acquisition cannot block on optimization; recorder raises integrity failure if unable to preserve raw data; live matcher may drop stale scans while recording the drop; Web Lab samples aggressively.

**Decision gate:** sustained Pi load, disk slowdown/full, serial burst, optimization stall, shutdown and interrupted writes. Show observable counters and no silent data loss. Test replay reproducibility from deliberately truncated bags.

### OD-022 — Tracking-state thresholds and pose publication

**Question:** Exact TRACKING/UNCERTAIN/LOST/RELOCALIZING transition thresholds, hysteresis, covariance and stale-pose handling?

**Provisional preference:** multi-scan hysteresis with overlap, residual, degeneracy, time-gap and predictor-disagreement gates. Publish continuous local odom separately from quality-rated global pose.

**Decision gate:** held-out map localization, featureless corridors, kidnapped robot, wheel slip and stale IMU tests; tune for low false-positive global fixes and measured recovery latency.

### OD-023 — MCAP native channel and schema

**Question:** Raw packet blobs + simple native schemas, ROS-compatible serialized messages inside MCAP, or custom binary alongside MCAP?

**Provisional preference:** native schema-defined channels storing raw LD19 packets, IMU/wheel samples, clock metadata, calibration and diagnostics; optional ROS bag import/export in adapter. Raw packet retention is non-negotiable.

**Decision gate:** independent converter interoperability, byte-level round-trip, time-order preservation, schema evolution, file size/read speed and ROS-free replay on ARM64/x86. Never describe processed `/odom` as raw encoder data.

### OD-024 — Deterministic replay seek and checkpointing

**Question:** Replay from start on every seek, periodic full estimator checkpoints, or hybrid indexing plus state rebuild?

**Provisional preference:** first implement deterministic start-to-finish replay and step; seek initially rebuilds state from start. Add checkpoints when profiling proves necessary for the Web Lab.

**Decision gate:** identical output hashes/metrics after pause/seek/step compared with straight replay, including gyro bias, active maps, graph states and random seeds. Document floating-point tolerance and thread-order constraints.

### OD-025 — ROS 2 distributions and adapter deployment

**Question:** Initial ROS 2 distribution, ament packaging strategy, QoS defaults, TF authority policy and support for rosbag2?

**Provisional preference:** one clearly declared supported ROS 2 distro in a separate Docker image, exported native CMake core, standard LaserScan/Imu/Odometry/OccupancyGrid topics, remappable frames, correct `use_sim_time`, single transform publisher per edge.

**Decision gate:** ROS-free native build, map PGM/YAML/OccupancyGrid round-trip with rotated origins, TF tree with Nav2/RViz, bag replay time jumps, late-subscriber map QoS and adapter CPU cost on Pi 5. Transitive Boost dependencies require their own review.

## 6. Cross-cutting trade-offs that must stay visible

**Data duplication and overconfidence:** A wheel/gyro trajectory is needed for deskew and search initialization; graph wheel and yaw factors use that raw evidence again. This conditioning does not mean frontend LiDAR residuals and graph factors are perfectly independent. Require conservative uncertainty, explicit factor provenance and controlled synthetic evaluations before assigning precise covariance.

**Map and pose revision coupling:** A scan matched against a frozen submap revision must reference that revision when the graph changes. Do not mix an old distance field with a newly edited occupancy map, and never update the same map the localizer is using.

**Latency versus optimality:** Offline can perform multi-pass global refinements; onboard must prioritize acquisition and local tracking, even if loop closure is delayed. No silently blocking graph solve or UI request in the critical path.

**Hardware parity:** Public datasets with ideal ROS LaserScan timing are not equivalent to LD19 packet timing. Synthetic timing can verify math, but a recorded LD19 + MPU6050 + real wheel dataset is needed before declaring the actual deskew and fusion pipeline validated.

**Dependency discipline:** A library choice is not a free dependency permission. Catch2 and cxxopts are mandatory with pinned FetchContent; heavy Ceres/GTSAM dependencies are provisioned via Docker; any proposed Boost component (even transitive) requires explicit approval, and performance/runtime/size impact must be recorded.

**Truth classes:** Synthetic exact, independent motion capture, LiDAR-derived reference and absent truth are distinct benchmark classes. Do not call low scan residual, internal map agreement or derived trajectory reference independent absolute trajectory error.

## 7. Decision record template

Copy this into this register when accepting one of the entries; retain history of superseded outcomes.

~~~markdown
### OD-XXX — Decision title
- **State:** Accepted / Superseded
- **Decision date:** YYYY-MM-DD
- **Owner / reviewers:** actual participants
- **Chosen approach and scope:** precise implementation boundary
- **Alternatives evaluated:** A, B, ... with pros/cons
- **Evidence:** Docker commands, dataset IDs/hashes, benchmark files and hardware
- **Rationale:** why it meets this milestone and what it does not solve
- **Consequences:** new dependencies (including Boost disclosure), migration, maintenance
- **Revisit trigger:** measurable condition or new requirement
- **References:** committed code/tests/reports
~~~

## 8. Immediate design workshop order

**OD-001 through OD-015 are Accepted** (implementation/tests pending). Before the first implementation, decide **OD-017** so C++20 native module and Docker targets are stable. Implement signed-nanosecond clock interfaces and Pi host-side timestamping first; design source metadata and transport so a custom Raspberry Pi Pico firmware source can be added later without changing estimator APIs. Next implement the accepted acquisition/beam timing contracts (OD-002, OD-003) and resolve the remaining recording schema decision (OD-023), then produce the synthetic fixtures and deterministic replay (**OD-024**). **Accepted OD-007** establishes the initial Ceres EDT objective; implement accepted OD-008 robust LiDAR-only rank/uncertainty assessment and implement accepted OD-013 keyframe/submap graph and conservative factor admission, with cross-factor covariance model refinement deferred until measured evidence warrants it. Remaining submap/graph decisions are evidence driven.

**Suggested first concrete experiment:** Verify accepted OD-004 midpoint reference, piecewise SE(2) interpolation and 10 ms extrapolation gate using a 15 m × 12 m apartment and 30 m × 20 m warehouse with exact beam times, 200 Hz yaw rate, wheel ticks and known extrinsics. Compare uncorrected, gyro-only and full wheel+gyro deskew, then measure Ceres convergence under initial-pose perturbations and warehouse symmetry. This single fixture suite verifies accepted **OD-004/OD-006/OD-007** and informs **OD-008 and OD-009** without relying on a large public dataset.
