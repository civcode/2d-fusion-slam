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
| OD-004 | Trajectory interpolation and scan reference time | P1 | Deskew, optimization | Open |
| OD-005 | Partial scans, filtering and assembly policy | P1 | Frontend acceptance | Open |
| OD-006 | Wheel/gyro predictor and kinematic models | P1 | Odometry, bias and slip | Open |
| OD-007 | Distance-field residual / loss design | P1 | Ceres matching | Open |
| OD-008 | Match uncertainty and degeneracy policy | P1 | Graph, localizer quality | Open |
| OD-009 | Coarse correlative search design | P2 | Tracking recovery | Open |
| OD-010 | Occupancy submap construction and thresholds | P1 | Mapping and graph | Open |
| OD-011 | Distance-field cache / immutable map snapshots | P1 | Map/matcher memory | Open |
| OD-012 | Native map bundle and persistence format | P1 | Reuse across modes | Open |
| OD-013 | GTSAM topology and information accounting | P1 | Backend accuracy | Open |
| OD-014 | Offline re-deskew and remapping iteration | P1 | Mapping quality | Open |
| OD-015 | Keyframe, submap and gyro-bias knot policy | P1 | Graph size/accuracy | Open |
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

Recommended **review order**: OD-017 + OD-001 (buildable core), then OD-002/003/005/023/024 (recording), OD-004/006 (motion), OD-007/008/010/011 (frontend), OD-012/013/014/015/018 (offline mapping), OD-009/020/022 (live localization), then OD-016/019/025 (full online/deployment). Some concurrency questions in OD-016/021 must be settled earlier if they block recorder implementation.

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

**Consequences:** This settles the native time **representation and simulation integration requirement**, **not** the sensor clock-sync algorithm (OD-002), LD19 beam timestamp model (OD-003), interpolation/extrapolation thresholds (OD-004), or replay checkpoint strategy (OD-024). No implementation, measured timing results or passing tests are claimed.

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

**Consequences:** Relative beam interpolation and calibrated host alignment architecture are **settled**. Exact physical timestamp reference, calibrated delay, clock-model fit parameters, quality thresholds and numerical uncertainty remain **empirical implementation/calibration work**. OD-004 still governs the deskew reference time, OD-002 the Pi/Pico acquisition transport and OD-023 the raw/derived timestamp storage schema.

**References:** [LD19 Development Manual v2.5](https://botland.de/img/art/inne/21991_Instrukcja%20rozbudowy.pdf), [technical protocol specification](technical-specification.md), [architecture specification](architecture-specification.md).

### OD-004 — Scan reference time and continuous trajectory

**Question:** End-of-scan vs midpoint reference, piecewise constant twist vs SE(2) exponential vs spline interpolation, and how far may state be extrapolated?

**Provisional preference:** midpoint reference for offline matching, with a documented alternative end-reference for reduced online latency; piecewise SE(2) integration from gyro/wheels and no unmarked extrapolation. Final choice must be **one consistent contract**, not mode-dependent silent behavior.

**Decision gate:** exact synthetic moving-beam tests and measured tracking latency across fast translation/rotation, data gaps and an off-center laser mounting. Verify that changing reference time yields equivalent world geometry when transforms are applied correctly.

### OD-005 — Scan assembly and invalid returns

**Question:** When is an LD19 revolution complete, how are partial scans, angle seams, intensity, invalid range and missing packets handled?

**Options:** strict full revolution, partial scan with coverage metadata, or fixed angle/time windows.

**Provisional preference:** angle-wrap revolution assembly with a partial-scan quality field; matching may reject low coverage, recording never drops valid raw source packets.

**Decision gate:** unit fixtures for 0/360 wrap, variable motor speed, duplicate beams, 12-beam packet truncation, CRC failure, inverse angular direction, sparse coverage; measure lost update rate and residual quality.

### OD-006 — Motion state, kinematics and estimator type

**Question:** Minimal wheel + bias-corrected gyro integrator, EKF/error-state filter, or recent-state smoother? Which drive model does the first robot actually expose?

**Provisional preference:** minimal continuous SE(2) predictor with wheel translation and gyro yaw, explicit gyro bias and covariance; separate differential-drive and Ackermann adapters. **Verify the primary robot drive type and encoder/steering telemetry before implementing drive-specific factors.**

**Decision gate:** synthetic wheel slip/gyro drift, actual stationary gyro calibration, encoder scale and yaw cross-check, numerical propagation tests and Pi runtime. Only add EKF/fixed lag if it demonstrably improves consistency or tracking.

## 4. Matching, mapping and graph decisions

### OD-007 — Ceres scan-to-map objective

**Question:** Euclidean signed/unsigned distance field vs occupancy probability vs NDT/line features; bilinear vs higher-order interpolation; robust loss and outside-grid policy?

**Provisional preference:** occupancy-derived Euclidean distance field and smooth interpolation with LiDAR-only residual `D(Tp)/sigma`, Ceres robust loss, finite range/unknown masks and bounded live iterations.

**Decision gate:** synthetic corridor/apartment/warehouse convergence and Jacobian tests, outliers, dynamic obstacles, boundary conditions, speed at 5 cm resolution and quality under biased pose initialization. Avoid squaring residual distance twice.

### OD-008 — Scan-match covariance, quality and degeneracy

**Question:** How are information/covariance, observable directions and rejection thresholds derived from scan matching?

**Options:** raw Hessian approximation, inflated robustified Hessian with condition gating, bootstrapped/resampled local fits, or empirical covariance calibration on held-out data.

**Provisional preference:** report overlap/inliers/residual plus Hessian spectral diagnostics; conservative covariance inflation, mark underconstrained cases invalid. Do not equate convergence or a tiny Hessian inverse with true independent certainty.

**Decision gate:** warehouse corridor degeneracy, yaw ambiguity, outlier-heavy scenes, synthetic true-error-vs-reported-covariance consistency and effect of conditioned gyro deskew. Explicitly evaluate dependence on the reference submap.

### OD-009 — Coarse search for poor initial pose

**Question:** Resolution pyramid correlative matching, branch-and-bound, grid search or particle hypotheses? Search budget per mode?

**Provisional preference:** bounded multi-resolution correlative scan search to recover small-to-moderate pose errors before Ceres; reserve full-map search for the relocalization module.

**Decision gate:** perturbation sweeps in metres and degrees, false match rate in repetitive aisles, p95 latency on Pi and candidate quality under 100 ms scan cadence. Keep search implementation separate from local nonlinear refinement.

### OD-010 — Occupancy submap size and freezing policy

**Question:** 5 cm vs finer/coarser grid; overlap size; scans per active submap; occupancy inverse-sensor model and dynamic aging?

**Provisional preference:** 0.05 m cell resolution; 1–2 overlapping active submaps and 30–100 scans/submap **as initial tuning ranges only**. Frozen submaps use immutable local coordinates.

**Decision gate:** map detail/accuracy and memory on synthetic apartment + warehouse and one real bag; verify hit/miss clamping, unknown handling, no scan matched against itself, consistent overlap/freeze semantics.

### OD-011 — Immutable map snapshots and distance-field cache

**Question:** Rebuild whole distance field every scan, rebuild at freeze boundaries, incremental distance-transform tiles, or precomputed frozen fields loaded on demand?

**Provisional preference:** immutable frozen fields for matching and offline precomputation for localization. Use versioned active-map snapshots where a live active submap is being matched; avoid concurrent read/write races.

**Decision gate:** peak memory, matching p95 latency, cache invalidation correctness, map revisions/tiling and object lifetime under concurrent graph updates. Validate no stale map interpolation.

### OD-012 — Persisted map bundle format

**Question:** Directory with JSON/YAML metadata plus compact binary grids/graphs, single binary container, or database-backed tiles?

**Provisional preference:** versioned manifest + binary blobs, explicit checksums and atomic publication. Offer Nav2 PGM/YAML import/export separately, accepting information loss on export.

**Decision gate:** schema evolution, round-trip rotated map origins, unknown occupancy and calibration identity; partial write recovery, large map load time, target-endianness and portability. One frozen map must support both batch output and onboard localizer.

### OD-013 — GTSAM graph topology and correlated evidence

**Question:** Pose-only scan/odom graph, keyframe+submap pose graph, or full joint pose/submap/bias graph? How are reused scan/map and wheel/gyro-conditioned deskew correlations represented?

**Provisional preference:** keyframe Pose2 + sparse gyro bias state, independent wheel and yaw factors, and scan-to-**frozen**-submap geometric factors when justified; test a simpler pose-only graph first. Avoid independent redundant LiDAR or fused-pose factors.

**Decision gate:** factor Jacobians, gauge fixing, spectral covariance checks, known synthetic truth, loop reconstruction, compare simpler vs richer topology for accuracy and graph size. Document provenance for every factor and any deliberately conservative noise inflation.

### OD-014 — Offline optimization and re-deskew iterations

**Question:** One pass vs repeated pose/bias/time updates, re-deskew, submap rebuild and re-optimization?

**Provisional preference:** one-pass pipeline first, then bounded re-deskew/re-map when bias/trajectory changes exceed meaningful thresholds. Keep original raw scans and every derived map version.

**Decision gate:** moving-scan synthetic error reduction, final loop closure quality, convergence and diminishing improvement per pass, repeatability and workstation runtime. Do not rebuild forever or hide self-feedback.

### OD-015 — Keyframes, submap poses and bias knot spacing

**Question:** Distance/angle threshold, uncertainty or information-gain keyframes; how many bias states per interval?

**Provisional preference:** begin with roughly **0.3–0.5 m** or **10–15 degrees** keyframe movement as *unverified tuning seeds*; allow time-based fallback when stationary bias matters. Bias knots may be less frequent than pose nodes.

**Decision gate:** graph RAM/solve time, path reconstruction quality, low-motion bias observability, long straight corridors and loop deformation; measure on identical recordings with varied thresholds.

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

**Question:** Adaptive particle filtering, hierarchical correlative pose search, or hybrid particle + Ceres refinement?

**Provisional preference:** keep a clean multi-hypothesis recovery interface. Benchmark a correlative baseline and AMCL/Beluga-style adaptive particle localization on the same frozen map before selecting.

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

**OD-001, OD-002 and OD-003 are Accepted** (implementation/tests pending). Before the first implementation, decide **OD-017** so C++20 native module and Docker targets are stable. Implement signed-nanosecond clock interfaces and Pi host-side timestamping first; design source metadata and transport so a custom Raspberry Pi Pico firmware source can be added later without changing estimator APIs. Next implement the accepted acquisition/beam timing contracts (OD-002, OD-003) and resolve remaining recording choices (OD-005, OD-023), then produce the synthetic fixtures and deterministic replay (**OD-024**). Only after collecting correctness evidence should we lock the Ceres residual/submap/graph decisions, rather than selecting complex algorithms merely for their sophistication.

**Suggested first concrete experiment:** Simulate a 15 m × 12 m apartment and 30 m × 20 m warehouse with exact SE(2) beam times, 200 Hz yaw rate, wheel ticks, and known extrinsics. Compare uncorrected, gyro-only and full wheel+gyro deskew, then measure Ceres convergence under initial-pose perturbations and warehouse symmetry. This single fixture suite informs **OD-004, OD-006, OD-007, OD-008 and OD-009** without relying on a large public dataset.
