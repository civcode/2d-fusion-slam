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

C++20; CMake; Catch2 v3 and cxxopts via pinned FetchContent; Docker for builds/tests/runtime; ROS-independent native core with REP-103/105 coordinate conventions; Ceres geometric frontend and GTSAM backend; raw-data preservation; offline mapping -> fixed-map localization -> online SLAM priority; no automatic GitHub Actions or Actions execution without explicit permission; no silent Boost adoption.

## 2. Priority and dependency overview

| ID | Subject | Priority | Affects | State |
| --- | --- | --- | --- | --- |
| OD-001 | Canonical timestamp API and units | P0 | All streams, serialization | Open |
| OD-002 | Clock synchronization/acquisition architecture | P1 | Live acquisition, replay | Open |
| OD-003 | LD19 packet timestamp-to-beam model | P1 | Deskew fidelity | Open |
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

### OD-001 — Canonical time representation

**Question:** Which typed timestamp and clock-domain model shall the portable core expose?

**Options:** (A) signed 64-bit nanoseconds in one canonical monotonic time domain with explicit source clock IDs and uncertainty; (B) signed 64-bit microseconds; (C) `std::chrono` point types parametrized by clock domain.

**Provisional preference:** A, wrapped in a strong native type. Device ticks and ROS/Unix timestamps remain distinct types until calibrated. Do not assume acquisition time equals receive time.

**Decision gate:** unit tests for overflow, ROS bag times, backward seeks, per-beam interpolation, clock-domain mismatch rejection and round-trip serialization. Benchmark conversions on Pi 5; record the exact canonical clock epoch semantics.

### OD-002 — Sensor timestamping / clock synchronization

**Question:** How are MPU6050, encoders and LD19 placed on a common acquisition timeline?

**Options:** (A) Raspberry Pi acquisition timestamps near each device read/interrupt plus latency model; (B) microcontroller acquisition timestamps with measured host offset/drift; (C) hardware synchronized capture if available.

**Provisional preference:** Start with A to unblock recording, architect B as an upgrade when a suitable MCU is available. Neither approach implies precise LD19 per-beam hardware timestamps.

**Decision gate:** stationary and moving recordings quantifying jitter, UART buffering, device clock rollover, clock offset/drift, missed readings and timestamp uncertainty. Choose calibration and reset policy before claiming reproducible deskew.

### OD-003 — LD19 per-beam time reconstruction

**Question:** What instant does the LD19 packet millisecond timestamp refer to, and how should each beam be interpolated?

**Options:** packet start/end beam, packet-generation time, or motor angle/speed derived estimates with independently fitted offset. Treat the hardware meaning as **unknown until tested**, not chosen by convention.

**Provisional approach:** preserve 47-byte raw frames and both host receive and device timestamps; unwrap the nominal 30,000 ms counter, interpolate angles and derive tentative beam times with explicit quality flags.

**Decision gate:** controlled turn-rate experiments, packet-loss and rollover tests, cross-check gyro yaw vs angular scan distortion, estimate bias/jitter and identify whether packet time refers to first/last beam. Save evidence in a calibration report.

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

Before the first implementation, decide **OD-001** and **OD-017** so C++20 native types and Docker targets are stable. Next agree the acquisition/recording contract (**OD-002, OD-003, OD-005, OD-023**), then produce the synthetic fixtures and deterministic replay (**OD-024**). Only after collecting correctness evidence should we lock the Ceres residual/submap/graph decisions, rather than selecting complex algorithms merely for their sophistication.

**Suggested first concrete experiment:** Simulate a 15 m × 12 m apartment and 30 m × 20 m warehouse with exact SE(2) beam times, 200 Hz yaw rate, wheel ticks, and known extrinsics. Compare uncorrected, gyro-only and full wheel+gyro deskew, then measure Ceres convergence under initial-pose perturbations and warehouse symmetry. This single fixture suite informs **OD-004, OD-006, OD-007, OD-008 and OD-009** without relying on a large public dataset.
