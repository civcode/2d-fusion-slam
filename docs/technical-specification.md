# 2D Fusion SLAM — Technical Specification

**Status:** Revised technical baseline v0.2; time/SE(2), synthetic fixtures, scan assembler, occupancy, LD19 packet parser and sampled deskew library implemented; full SLAM/hardware behavior not benchmark-validated  
**Date:** 2026-10-31  
**Related:** [Architecture](architecture-specification.md) · [Implementation contract](implementation-contract.md) · [Benchmarks](benchmarks-and-datasets.md) · [Design decisions](open-design-choices.md)

## 1. Purpose, scope and modes

Provide a portable planar LiDAR/gyro SLAM system. The **first implementation and algorithm tests use deterministic synthetic fixtures and validated, opt-in public benchmark datasets**, on a workstation or suitable development computer. A **parallel** Raspberry Pi 5 / LD19 / MPU6050 acquisition and raw-recording activity prepares eventual real-world evidence; it is not a gate on the first algorithm milestones.

- **A — Offline mapping:** workstation, replayable data, no hard real-time limit, occupancy mapping plus GTSAM batch optimization and carefully verified loop closure.
- **B — Known-map localization:** Raspberry Pi 5, read-only prebuilt map, bounded scan matching and conservative global-pose validity; no initial GTSAM requirement.
- **C — Online SLAM:** later Pi 5 configuration after A/B measurements; selected active maps and asynchronous incremental optimization with explicit memory/session limits.

Not first-scope: 3D inertial SLAM, visual SLAM, autonomous control/navigation, GPU kernels, mandatory ROS, continuous precise planar displacement from MPU6050 acceleration, or full offline optimization on Pi.

## 2. Functional requirements and evidence

| ID | Requirement | Initial verification |
| --- | --- | --- |
| FR-01 | Parse/adapt measured sources with explicit timing, quality, frame and provenance; preserve raw packets | Generated packet fixtures; imported public records; later real LD19 |
| FR-02 | Canonical checked nanosecond clock with domains/epochs, no false continuity across gaps or jumps | Synthetic rollover/jitter/replay/ROS-neutral clock fixtures |
| FR-03 | Calibrated laser/IMU extrinsics, gyro axis/bias and configuration identity | Synthetic transforms; controlled calibration data later |
| FR-04 | Support `LIDAR_GYRO` without a wheel input; optionally `LIDAR_GYRO_WHEEL` only when measurements exist | Sensor-availability matrix, dropout and covariance tests |
| FR-05 | Assemble faithful full/partial scans, invalid/no-return classification and optional safe deskew | Generated timed-beam fixtures, missing-sector tests |
| FR-06 | Ceres scan matching with robust residual, known mask, diagnostics, overlap/rank and prior separation | Synthetic objective/Jacobian/outlier/degenerate tests |
| FR-07 | Clamped probabilistic occupancy with known/unknown state and safe insertions | Synthetic ray/scan/submap tests |
| FR-08 | First offline pose-only GTSAM batch reference with correct gauge and nonduplicated evidence | Exact synthetic graph and wrong-loop tests |
| FR-09 | Verified immutable portable map export/import and deterministic replay | Round-trip/checksum/crash-fault and same-run hash/tolerance |
| FR-10 | Read-only known-map localizer with truthful pose validity and bounded normal tracking | Held-out synthetic/public traversals, then Pi |
| FR-11 | Separable bounded multi-hypothesis recovery interface with explicit ambiguous/unavailable states | Repeated warehouse/kidnap fixtures; comparative Pi trial later |
| FR-12 | Optional staged Pi incremental online SLAM only after profile B is measured | Offline/online parity; Pi CPU/RSS/session-duration test |
| FR-13 | Benchmark results distinguish exact synthetic, independent measured, derived reference and no truth | Report validator; split leakage detection |
| FR-14 | Support native Linux ARM64/x86-64 and optional ROS adapter without ROS in core | Build/consumer/adapter tests when corresponding targets exist |

## 3. Hardware and capability assumptions

| Item | Baseline | Confidence and consequence |
| --- | --- | --- |
| LiDAR | LDROBOT LD19, native UART; nominal ~10 Hz revolutions, ~4,500 samples/s; 12 points/47-byte packet; ~30 s timestamp wrap | Packet-reference beam time and buffering delays **unverified** until hardware tests; packet parser testable synthetically |
| IMU | MPU6050; initial candidate 200 Hz gyroscope acquisition | Gyro yaw rate, sample timing, bias, mounting and temperature drift must be calibrated; no trusted inertial XY trajectory |
| Optional wheels | Actual wheel encoder or measured steering/speed telemetry, **not assumed present** | No wheel-specific factor or full translation deskew until physically verified |
| Workstation | x86-64/ARM64 Linux development system | Offline batch compute/memory targets must be documented by benchmark |
| Onboard | Raspberry Pi 5 / ARM64 Linux; actual RAM/cooling/storage unspecified | All latency/memory targets provisional pending on-device measurement |

Public ROS bag scans are preprocessed input, not proof of LD19 packet acquisition timing. Public datasets may lack wheel ticks or true external trajectory ground truth; experiments must label those limitations.

## 4. Frames, time, calibration

### 4.1 Frame convention

Native `Pose2` / `T_target_source` uses metres, radians, `x` forward, `y` left, yaw positive CCW. Define fixed `T_base_laser` and `T_base_imu`, with provenance and calibration revision. `T_map_base(t)=T_map_odom(t) * T_odom_base(t)`; evaluation uses the **same event time** for both factors. Do not jump odometry when loop closure/global relocalization changes `map -> odom`. If no valid local XY predictor exists, say that the local pose is sparse/uncertain or unavailable; do not synthesize a continuous precise transform.

### 4.2 Time model

Use signed `int64` nanoseconds wrapped as typed `Timestamp`/`Duration` with explicit domain and epoch; checked addition/difference/conversion. Keep raw device ticks, Pi receive stamps, OS monotonic readings, mapped acquisition estimate, conversion fit and uncertainty independent. Arrival time is not beam acquisition time. Simulation time (`/clock`, via optional ROS adapter) may pause/jump; epoch changes reset or explicitly restore time-dependent trajectories, assembly and estimator. Deadline and queue age measurements use **host steady time**. Preserve original ordering and deterministic equal-time tie-breaks.

LD19: validate packet framing/CRC, start/end angle unwrap, physical rotation direction, motor speed, raw 16-bit millisecond counter rollover, dropout and reset; estimate **relative** beam times using angle/motor speed subject to reference delay uncertainty. Never assert that packet counter names a first/last physical beam until measured. A one-way host-arrival fit cannot separately resolve fixed transport delay and source clock offset. Validate serial event semantics and clock uncertainty later with actual packets.

### 4.3 Required calibration and health

- LiDAR translation/yaw and measured tilt; IMU orientation/gyro-axis sign.
- Gyro stationary bias with variance and validity, sample rate/filter delay and optional temperature.
- Actual wheel radius/baseline or steering geometry **only when wheel source verified**.
- Clock offset/drift/uncertainty and acquisition/receipt event definition, with epoch and calibration IDs.
- Source quality, dropouts, stationary detection, slip (when independently observable) and rate limits.

A calibration change invalidates incompatible derived scans, cached geometry, pose/graph estimates and map generations. Raw source bytes never change.

## 5. Motion and deskew

### 5.1 Estimator capability contract

A ROS/device-independent `TrajectoryProvider` returns a time-addressed `T_odom_base(t)` candidate, coverage/quality, covariance in documented tangent/frame, interpolation or extrapolation status, source IDs and revision. Use the simplest initial integrator: calibrated gyro-rate yaw with bias uncertainty, optionally observed wheel translation. A scan-derived displacement from prior accepted scan matches may seed **interscan** motion but does not, by itself, prove intrarevolution translation.

For each valid piecewise-constant body twist segment,
`T(t)=T(t_k) Exp_SE2((t-t_k) xi_k)`, with `xi=(v_x,v_y,omega)` in body coordinates. Use exact SE(2) exponential and a stable zero-`omega` limit; do not interpolate independently in Cartesian coordinates as an alleged constant-twist path.

In `LIDAR_GYRO` mode: support gyro-only rotational correction, conservative no-translation assumption **only when explicitly identified as a modeling approximation**, and mark XY uncertainty appropriately. A zero wheel input means **unavailable**, not measured stopped. In `LIDAR_GYRO_WHEEL` mode: translation source must be observed, calibrated and time aligned; differential/Ackermann motion is selected from actual drivetrain telemetry, never from actuator commands alone.

### 5.2 Scan midpoint and per-beam transform

`t_start` and `t_end` span **actual acquired time-valid beam slots**, including invalid range slots; no phantom timestamps in missing sectors. Use `t_mid=t_start+floor((t_end-t_start)/2)` with checked integer math.

~~~text
p_laser_mid = inverse(T_odom_base(t_mid) * T_base_laser)
              * (T_odom_base(t_i) * T_base_laser) * p_laser_i
~~~

For a normal map insertion, require a supported, calibrated trajectory over the entire actual scan interval. An unsupported translation model is not silently upgraded into full SE(2) compensation: record `NONE`, `GYRO_ONLY` or `FULL_SE2` correction mode, quality and limitations. Compare all supported variants on **identical synthetic/public prepared observations**; full SE(2) is valid only with credible translation. For live prediction, any extrapolation must carry separate status, measured duration and conservative bounded quality; do not insert extrapolated scans into a normal map.

## 6. LiDAR scan model and occupancy

Use fixed angular-seam assembly of CRC-validated observed beams after verified angle unwrap and direction. Packets may straddle a seam; divide actual beams without duplication. Record partial/full status, circular coverage, missing sectors, largest gap, actual timestamp bounds, packet gaps and return classes:

| Class | Match evidence | Map action |
| --- | --- | --- |
| `VALID_HIT` | Endpoint if calibration/time/geometry quality passes | Clear trusted cells **before** hit, then occupy hit cell |
| `VALID_NO_RETURN` | No endpoint | Optional bounded clear ray only with documented sensor meaning and explicit config |
| `INVALID_RETURN` | None | No clearing or hit |
| `MISSING` | None | No invented ray or time |

Separate eligibility gates for matching and map insertion. A partial scan may pass when actual angular/spatial support is adequate. On insertion collect unique hit/free sets per scan/submap, hit priority over free and bounded ray traversal. Preserve observed/unknown mask independently from log-odds value: observed cells returning to 0.5 probability do **not** become unknown.

A **candidate 5 cm** grid and **two overlapping submaps with 90/180 accepted insertions** make a reproducible starting experiment, not a locked deployed policy. Compare count/extent/resolution on identical data; document actual overlap membership and early freeze/size caps. Local frozen occupancy/known mask/origin is immutable; global submap pose is separately versioned.

## 7. Ceres geometric frontend

Use a pinned coherent map source (occupancy, known/unknown support, EDT and config hashes). The initial residual for each **valid supported hit endpoint** is `r_i=sqrt(w_i)*D(q_i)/sigma_i`, where `D` is truncated unsigned Euclidean distance to confident occupied cells in metres. Use **one scalar Ceres residual block and one Huber loss per beam**; do not square distance inside the residual. Bilinear interpolation must implement a documented support/edge stencil. Unknown cells and out-of-bounds do not produce false zero residual/inliers. A separate finite boundary penalty may lead an optimizer toward supported map, but is not LiDAR evidence.

Compare geometry-only matching (prediction as initial seed) with optional covariance-weighted prior matching:
`E=0.5 sum_i rho(r_i^2)+0.5||L Log(T_pred^-1*T)||^2+0.5 sum boundary^2`, `L^T L=Sigma_pred^-1`. A prior is used **only if valid**, with source ID/covariance and same time/frame. The posterior pose must never become a graph LiDAR factor merely because Ceres converged.

`MatchResult` must include solver status, pose, map/source hash, used prediction, LiDAR-only residual/overlap/observability information, boundary/unknown counts, input timing/deskew quality, and an optional **separately justified** geometry constraint. Initial quality gate: finite result, sufficient inliers and overlap, nontruncated informative beams, plausible innovation with correlated prior acknowledged, LiDAR-only Hessian rank/condition, and conservative accept/reject/unavailable outcome. Do **not** claim that an inverse Hessian is a calibrated independent pose covariance. Rank 1–2 geometry does not automatically justify an SPD full-rank graph factor; projected constraints are an optional later tested feature.

## 8. Offline GTSAM and loop closure

Initial offline baseline: one **pose-only** batch graph with explicit gauge, a source-provenance ledger and verified relative constraints. A measured fused motion factor can be used only when its underlying sources actually exist. A gyro yaw constraint cannot be silently converted into precise XY. Do not insert simultaneously an already-fused wheel+gyro motion factor and separately weighted duplicates of the same wheel/gyro samples. Scan matches involving the same source beams, overlapping occupancy, previous scans or an odom-conditioned posterior require correlation guard or omission.

Stage I loop closure: candidates from frozen-submap spatial index with time/source exclusion, coarse geometric search if needed, independent Ceres/quality verification and provisional graph-consistency checks. A matching score or GTSAM robust loss alone is insufficient. An incorrect loop must be rejected/rolled back, not baked into occupancy. Optional signatures and keyframe+submap topology are post-baseline comparisons. Save original evidence/relative constraints for later reconstruction.

Initial `R0` processing: full deterministic replay, map build and batch solve, versioned immutable output. Only when held-out or independent measurements show benefit, allow bounded candidate `R1...` recalibration/re-deskew/rebuild and reoptimization with rollback. Never judge iterations solely by different training graph costs.

## 9. Known-map localization and trust

The Pi known-map profile loads a **read-only** verified map bundle; Ceres tracking and motion prediction operate on immutable snapshots, without online GTSAM. `TRACKING`, `UNCERTAIN`, `LOST` and `RELOCALIZING` have typed transition reasons and explicit current-global-pose validity. Initialize without a trusted map anchor as LOST. A host-steady watchdog degrades freshness even when scans cease.

An accepted map correction uses `T_map_odom(t_mid)=T_map_base(t_mid)*inverse(T_odom_base(t_mid))` at the same valid time. Preserve odom continuity. No authoritative current global pose under LOST/RELOCALIZING; uncertainty/invalidity must not be hidden behind a stale TF stamp. If wheel-independent local odom is not available, publish only the motion components and frequency supported by the actual estimator.

Recovery is a **separate bounded and asynchronous** interface with distinct `PARTIAL`, `AMBIGUOUS`, `VERIFIED`, `UNAVAILABLE` outcomes, top-K spatially distinct temporal candidates and known map revisions. Local correlation, hierarchical/global BnB and AMCL/Beluga-style particle search are comparative alternatives. A partial/budget-exhausted search does not prove uniqueness; a well-scoring scan in repeated aisles does not justify a global lock. Suspend map insertion until ordinary trusted tracking.

Numerical thresholds for covariance, freshness, overlap, reject streaks and search budget are **experiment configurations**, not deployable constants. Use synthetic and public data to set initial candidates, then validate with actual Pi and device recordings before claiming reliability.

## 10. Performance, resources and failure behavior

### 10.1 Offline workstation

Report accuracy, rejected/invalid cases, optimizer convergence, map quality, memory and wall time; do not impose Pi latency constraints on batch mapping. Reconstruct and compare algorithm variants from identical raw or prepared datasets.

### 10.2 Pi 5 known-map localizer

The LD19 nominal ~10 Hz scan stream is a **workload**, not proof of a 20 ms matcher deadline. Measure **capture-to-valid-pose p50/p95/p99**, scan/queue lag, rejected/stale scans, missed update cycles, CPU/RSS, cooling/thermal throttle, power and storage behavior. Bounded queue/CPU behavior and safety under stalled scan processing matter more than maximizing nominal Ceres throughput. Drop **stale derived scans** with diagnostics; do not silently discard required authoritative recording data.

### 10.3 Pi 5 online SLAM

Only after localization is stable and benchmarked: measure incremental graph growth over a stated supported session duration, loop delays, optimizer CPU/RAM and map revision lag. Define resource exhaustion as explicit degraded operation or limited session, not as a claim that an unbounded graph is bounded.

## 11. Persistence, replay and optional integration

Native raw Protobuf MCAP preserves original packet/sample evidence, clock/calibration/config changes and explicit loss or corruption. Raw recording is independent of optional derived pose streams. Recorder states distinguish accepted, written, synced and complete; a crash or full disk must never produce a false `COMPLETE` session. Start with deterministic **linear replay**, step and restart; add indexed seeking/semantic state checkpoints only if user workflow and profiling warrant the cost.

The native map bundle uses a versioned JSON manifest and portable compressed binary occupancy/known mask and graph/pose payloads as needed, content hashes and atomic version publication. Caches are regenerable. Check dataset/calibration/map compatibility and reject corrupt/oversized/untrusted input.

ROS 2 is an **optional separate adapter**: acquisition-time stamps, `use_sim_time` handling, single TF edge authority, faithful partial/nonuniform scan representation, status-aware global pose publication and QoS. The accepted integration target is Jazzy on Ubuntu 24.04 when that adapter is built and verified; no ROS/rosbag2 dependency in the core. Web Lab, Pico and advanced cross-process deployment are optional measured additions.

## 12. Validation sequence and milestone gates

| Stage | Required dataset / environment | Exit evidence |
| --- | --- | --- |
| V0 — contracts + synthetic fixture | Local/Docker workstation; apartment/warehouse exact truth | Time/SE(2), sources, packet simulation, scan/ray math, deterministic test pass |
| V1 — prepared public inputs | Explicitly selected/verifiable public bag subset; no normal-test downloads | Converter and sensor-availability tests, no unsupported wheel assumption, provenance/truth labels |
| V2 — first offline mapping | Same synthetic/public inputs and held-out segments | One-pass map + Ceres + simple GTSAM graph, false-loop rejection, map round-trip |
| V3 — offline known-map localization | Independent held-out synthetic/public traversal | Accuracy/rejection/false-lock/pose-validity baseline, no runtime graph |
| V4 — Pi localization trial | Prepared identical datasets on Pi, then actual connected sensors | Latency/RSS/thermal and safe degradation, real sensor calibration/capture behavior separated |
| V5 — optional Pi online SLAM | Same datasets + timed extended sessions | Measurable improvement over simpler baseline within explicit RAM/CPU/session bounds |

**Parallel hardware path:** Build a minimal LD19+MPU6050 acquisition recorder and later controlled real recordings independently of V0–V3; do not claim real-hardware accuracy from synthetic/public input. Details of datasets, modes, scoring and ablations live in [Benchmarks](benchmarks-and-datasets.md).

## 13. Verification references

[Architecture](architecture-specification.md) defines ownership and mode composition; [Implementation contract](implementation-contract.md) defines build/repo/CI/release rules; [Design choices](open-design-choices.md) records selected, staged and to-be-compared options. These specifications are **proposals until code and evidence exist**, not a record of passed tests.
