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

**Consequences:** Angular-seam segmentation, honest partial scans, return validity and safe occupancy behavior, and recovery-only timeouts are **settled**. Exact coverage binning, thresholds, loss tolerance and timeout duration remain **configurable implementation/benchmark tuning**, not permission to invent missing samples. OD-003 owns per-beam time reconstruction; OD-004 owns midpoint trajectory deskew; OD-005 owns **observation grouping and scan eligibility**. OD-006 motion integration is accepted; OD-007 scan matcher scoring and OD-010 occupancy inverse-sensor model remain open.

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

**OD-001, OD-002, OD-003, OD-004, OD-005 and OD-006 are Accepted** (implementation/tests pending). Before the first implementation, decide **OD-017** so C++20 native module and Docker targets are stable. Implement signed-nanosecond clock interfaces and Pi host-side timestamping first; design source metadata and transport so a custom Raspberry Pi Pico firmware source can be added later without changing estimator APIs. Next implement the accepted acquisition/beam timing contracts (OD-002, OD-003) and resolve the remaining recording schema decision (OD-023), then produce the synthetic fixtures and deterministic replay (**OD-024**). Only after collecting correctness evidence should we lock the Ceres residual/submap/graph decisions, rather than selecting complex algorithms merely for their sophistication.

**Suggested first concrete experiment:** Verify accepted OD-004 midpoint reference, piecewise SE(2) interpolation and 10 ms extrapolation gate using a 15 m × 12 m apartment and 30 m × 20 m warehouse with exact beam times, 200 Hz yaw rate, wheel ticks and known extrinsics. Compare uncorrected, gyro-only and full wheel+gyro deskew, then measure Ceres convergence under initial-pose perturbations and warehouse symmetry. This single fixture suite verifies accepted **OD-004 and OD-006** and informs **OD-007, OD-008 and OD-009** without relying on a large public dataset.
