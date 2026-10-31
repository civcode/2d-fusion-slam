# 2D Fusion SLAM — Design Decisions and Evaluation Register

**Status:** Revised v0.2 — directions selected, implementation/evidence stages pending  
**Date:** 2026-10-31  
**Related:** [Architecture](architecture-specification.md) · [Technical specification](technical-specification.md) · [Implementation contract](implementation-contract.md) · [Benchmarks](benchmarks-and-datasets.md)

> **Change in decision policy:** The previous v0.1 register labeled OD-001–OD-025 “Accepted” with detailed architecture-specific algorithms, fixed module/process counts, proposed thresholds and unmeasured Pi budgets. Those labels **never meant implemented or validated**. This revision keeps the correctness contracts, stages optional functionality and reopens choices that should follow benchmark evidence. It supersedes the former initial implementation mandates, not the historical fact that those options were considered. See the [implementation contract](implementation-contract.md) for staged deliverables.

## 1. Evidence-first decision process

A design direction can be selected **without** mandating a sophisticated implementation. Separately record these independent evidence states for each implemented feature:

1. **Design selected / candidate / reopened / deferred.** This is the architectural plan, not a performance claim.
2. **Implemented.** Versioned source and runnable entry point exist.
3. **Synthetic validated.** Deterministic synthetic tests with exact geometric truth run and pass.
4. **Public dataset evaluated.** The pinned source/crop/converter and measured output exist; modality/truth class disclosed.
5. **Real sensor evaluated.** Actual LD19/MPU6050 capture, clock and calibration evidence exists.
6. **Pi validated.** Actual Raspberry Pi 5 load/CPU/RAM/thermal/fault tests pass for the stated runtime profile.

A feature can be synthetic/public tested while hardware capture is still in progress. **The initial implementation and benchmarks begin with synthetic and opt-in verified public data**; hardware acquisition proceeds in parallel. Do not claim real-sensor or Pi validation from a public ROS bag or x86-only Docker test. Every benchmark assertion cites dataset hash/truth class, configuration/calibration/clock version, software revision, environment and failure outcomes. Do not invent owners, successful tests or performance values.

Priorities: **P0** necessary for first runnable core; **P1** first workstation offline mapping and deterministic replay; **P2** known-map localization; **P3** later Pi online SLAM or optional integrations. Numerical thresholds below are **candidate experiment inputs**, unless an explicitly tested invariant.

## 2. Overview — current decisions

| ID | Topic | Priority | Current design status | First implementation / decision |
| --- | --- | --- | --- | --- |
| OD-001 | Canonical time and epochs | P0 | **Selected invariant** | Typed checked int64 ns and clock-domain/epoch identity |
| OD-002 | Device capture and clock sync | P1 | **Selected, staged** | Pi near-event stamps + quality; optional Pico later |
| OD-003 | LD19 beam time | P1 | **Selected relative model, calibration pending** | Raw packet+counter+angles; uncertainty carried |
| OD-004 | Continuous trajectory/deskew | P1 | **Selected geometry, staged capabilities** | Midpoint SE(2) contract; gyro-only when no translation |
| OD-005 | Scan assembly/return safety | P1 | **Selected invariant** | Angular-seam partial scans, honest invalid/no-return |
| OD-006 | Motion estimation | P1 | **Revised, staged** | LIDAR_GYRO baseline, wheel only if observed |
| OD-007 | Ceres objective | P1 | **Compare two candidate tracking objectives** | Geometry-only seeded baseline vs optional prediction prior |
| OD-008 | Quality and degeneracy | P1 | **Selected invariant, advanced models staged** | LiDAR overlap/rank/reject; no false covariance |
| OD-009 | Coarse candidate search | P2 | **Reopened algorithm/budget** | Bounded local correlation first; global option separately |
| OD-010 | Occupancy and submaps | P1 | **Selected representation; parameters to tune** | 5 cm, log-odds/known mask; 90/180 counts as experiment |
| OD-011 | Map snapshots/EDT | P1 | **Selected coherence; optimization deferred** | Complete immutable EDT snapshots first |
| OD-012 | Native map persistence | P1 | **Selected** | Manifest, checked portable binary, immutable generations |
| OD-013 | Graph and source accounting | P1 | **Selected evidence rule; topology staged** | Simple pose-only GTSAM graph first |
| OD-014 | Offline refinement | P1 | **Selected staged** | Deterministic one-pass R0; optional selective iterations |
| OD-015 | Keyframe policy | P1 | **Reopened thresholds** | Compare sparse/dense motion thresholds on same data |
| OD-016 | Process/worker topology | P1/P2 | **Reopened/staged** | Logical owners + bounded queues, not four mandatory processes |
| OD-017 | CMake/API decomposition | P0 | **Reopened/staged** | Build useful actual targets; package only implemented APIs |
| OD-018 | Loop retrieval | P1 | **Selected staged** | Spatial frozen-map retrieval and conservative verification |
| OD-019 | Pi online graph | P3 | **Reopened, deferred** | Compare incremental options after Pi localization baseline |
| OD-020 | Global recovery | P2 | **Reopened algorithm choice** | Top-K safe interface; compare BnB, particle, hybrids |
| OD-021 | Queues/raw integrity | P1 | **Selected invariant; capacities to measure** | Bounded fan-out, sticky INCOMPLETE on required raw loss |
| OD-022 | Pose trust/state | P2 | **Selected states; thresholds to measure** | TRACKING/UNCERTAIN/LOST/RELOCALIZING, truthful validity |
| OD-023 | MCAP raw channels | P1 | **Selected, staged** | Protobuf raw/device/clock/calibration, linear replay |
| OD-024 | Indexed seek/checkpoint | P1/P3 | **Reopened/deferred** | Deterministic start-to-finish replay first |
| OD-025 | Optional ROS integration | P3 | **Selected isolation; optional build** | Standalone Jazzy adapter only after native integration need |

**Implementation evidence update (2026-10-31):** Checked time/SE(2), synthetic scenes, scan assembly, known-mask occupancy, native LD19 47-byte/CRC parser with nominal counter unwrap, and interpolation-only motion/deskew code are implemented. Four local CTest suites pass under strict warnings, UBSan and ASan+UBSan. They partially exercise B01–B06, **not** calibrated physical packet timing, gyro-bias/wheel motion estimation, hardware or public benchmarks. The full SLAM pipeline, submaps, Pi profiling and real calibration remain unverified; see [implementation status](implementation-status.md). See [implementation status](implementation-status.md). No candidate numerical performance guarantee is accepted.

## 3. Source time, motion and scan decisions

### OD-001 — Canonical time, clock domain and epoch

**Selected invariant.** Signed `int64` nanoseconds with typed timestamp and duration arithmetic; explicit domain and epoch, checked conversion and source/receipt stamp preservation. ROS simulated `/clock` pauses/jumps are adapter-managed clock events, not wall-clock integration. Host-steady deadlines are independent. **Do not reopen solely for an alternative scalar time type.**

**First evidence:** overflow and mixed-epoch rejection, exact synthetic integer conversion, pause/seek/step. **Revisit:** only if an actual time-domain limitation is documented.

### OD-002 — Acquisition and synchronization

**Selected staged design.** Pi-side near-event host-monotonic capture with documented reception event semantics and error flags first; an optional Pico firmware source can later provide hardware-near sensor timing and explicit offset/drift estimates. Acquisition hardware is developed in parallel while core tests work on prepared data. Do not require Pico, custom PIO, four-time exchanges or real sensor access for first algorithm milestones.

**Compare later:** measured Pi interrupt/read timing against Pico capture if physical drift/jitter is material; reject false nanosecond accuracy. **Evidence:** future actual Pi captures, epochs/reboots, queue and device jitter.

### OD-003 — LD19 packet-to-beam timing

**Selected relative model, not a validated physical timestamp reference.** Retain exact 47-byte UART packets, CRC failures when observed, unwrapped millisecond counter, angles and motor speed. Reconstruct **relative** per-beam timing with signed angle/speed and documented uncertainty; preserve clock source/fit/calibration versions. One-way packet arrival cannot separate constant transport delay from source epoch offset; do not presume first- or last-beam physical timestamp.

**First tests:** synthetic wrap/drop/variable RPM. **Later evidence:** controlled LD19/MPU6050 recordings determining or bounding absolute beam reference delay. Synthetic/public LASER scans do not resolve it.

### OD-004 — Midpoint scan frame and continuous SE(2)

**Selected geometric contract, capability staged.** Use the midpoint of **actual time-valid acquired** scan slots and `T(t)=T(t_k) Exp_SE2((t-t_k)xi_k)` with checked event time, transform lever arm and interpolation coverage. No normal map insertion requiring unsupported trajectory intervals or extrapolated poses. With no wheel translation source, gyro-only rotational deskew is an explicitly limited mode, not full SE(2) measured translational deskew.

**First comparison:** no deskew vs yaw-only; full SE(2) only with supported simulated/measured translation. **Revisit:** spline/other trajectory after hold-out benefit.

### OD-005 — Angular seam, partial scans and return validity

**Selected invariant.** Assemble beams across a fixed angular seam; do not infer complete revolutions from wall time or packet boundaries. Preserve partial coverage, time bounds, missing sector/gap metadata and return status. Distinguish valid hit, explicitly trusted no-return, invalid return and missing measurement; never clear map from invalid/missing beams.

**First evidence:** synthetic seam-straddling packets, motor reversal/gaps, no phantom timestamps or rays; separate matching/insertion gates.

### OD-006 — Sensor-capability-aware motion estimator

**Revised from mandatory wheel+gyro first state.** Initial `LIDAR_GYRO` supports bias-corrected yaw trajectory and conservative scan-derived interscan displacement without inventing continuous metric XY odometry. Enable `LIDAR_GYRO_WHEEL` only with verified measured wheel/speed/steering telemetry and calibrated kinematics. MPU6050 acceleration cannot substitute for wheel displacement by naive double integration.

**First implementation:** minimal gyro bias-aware integrator and portable `TrajectoryProvider`; stationary-bias criteria and uncertainty status. **Compare later:** wheel fusion, EKF or smoother against same prepared data. **Evidence:** synthetic capability/dropout tests first; actual hardware source verification later.

## 4. Matching, mapping and batch decisions

### OD-007 — Ceres scan objective and prior

**Reopened objective comparison, solver choice retained.** Start with a geometry-only, prediction-seeded truncated unsigned EDT residual using bilinear interpolation, one scalar Huber loss per valid hit, explicit observed/unknown support mask and finite boundary handling. Compare optional odom-regularized frontend tracking using a separately recorded prediction prior. Do not treat unknown as a free inlier or assign constant zero distance outside valid support. Avoid squaring the EDT distance inside Ceres's already squared residual.

**Invariant:** The prior-conditioned tracking posterior is not an independent LiDAR graph observation. Geometry-only matching may still share deskew/map correlations. **Evidence:** B08 synthetic convergence, false-locks, Jacobians, outliers, rank, timing; public hold-out before selecting default.

### OD-008 — Observability and covariance

**Selected correctness boundary; advanced math staged.** Report LiDAR-only supported-hit overlap, residual distribution, information rank and degeneracy **without** motion prior or artificial boundary residuals. Reject or mark unavailable when geometry cannot support a reliable correction. Never publish pseudo-inverse nullspace as precise full covariance; do not equate Ceres Hessian inverse with externally calibrated independent uncertainty.

**Start:** full-rank-or-reject and conservative status. **Later:** rank-projected corrections and empirical covariance inflation only with held-out independent truth or defensible consistency tests.

### OD-009 — Local coarse search

**Reopened; restore older minimal direction.** Normal tracking can use direct Ceres with odom/last-match initialization; uncertain tracking may run a bounded multiresolution/local correlative initialization. Expensive global BnB belongs to OD-020 candidate recovery, not an unconditional installed first frontend.

**Evidence:** perturbation sweeps, repeated warehouse false locks and Pi time/candidate budget before choosing pyramid/cache/BnB implementation.

### OD-010 — Occupancy and frozen submap lifecycle

**Selected representation, candidate parameters.** Occupancy log odds with independent observed mask, bounded 5 cm initial resolution, checked ray traversal, per-scan hit precedence and immutable frozen local maps. Two active overlapping submaps with 90 accepted scans to create successor and 180 to freeze is **one deterministic test configuration**, not a locked product policy. Never insert a scan then match it against itself; source/membership IDs remain stable.

**Compare:** count/extent/resolution/overlap on same synthetic/public inputs. **Revisit:** data show resolution, dynamic scenes or memory effects.

### OD-011 — EDT and immutable snapshots

**Selected coherent revision invariant; tiled optimization deferred.** The first implementation builds a **full-field EDT** and coherent immutable known-mask/occupancy map snapshot, pinned during matching. Derivative/caches must share source/config hash; separate EDT versus any correlation pyramid. No concurrent mutable read or mixed occupancy/EDT revisions.

**Do not implement initial fixed 32×32 dirty tiles** just to satisfy an architectural proposal. Add incremental update/eviction only after validated Pi profiling and equality against full EDT.

### OD-012 — Native map bundle

**Selected.** Portable versioned JSON manifest plus binary occupancy/known mask and optional graph/pose information, checksums, bounds/version validation and atomic generation publication. Graph-pose revisions may move frozen submaps without mutating local raster or EDT source hashes. Optional ROS PGM/YAML export is lossy interchange.

**Evidence:** round-trip on synthetic maps, rotated origin, corruption and crash-recovery tests before Pi loading.

### OD-013 — GTSAM topology and nonduplicated evidence

**Selected evidence accounting; first topology simplified.** Begin with a **simple pose-only workstation GTSAM batch graph** and correct gauge; do not pre-build a full keyframe+submap+bias graph. A measured fused wheel+gyro interval factor must not be duplicated by independent wheel/yaw factors from the same samples. In a gyro-only case, yaw does not create a fully observed XY motion likelihood. Never add a Ceres prediction-conditioned posterior as an independent LiDAR factor. Deskew and occupancy-source correlations still require conservative admission/provenance or skipped factors.

**Compare later:** joint keyframe/submap poses, offline bias knots, full correlation model only if held-out gains justify complexity. **Evidence:** B09 graph/false loop/duplicate source tests.

### OD-014 — Offline outer refinement

**Selected staged.** First deliver one reproducible complete mapping/graph pass **R0** and immutable map bundle. Select bounded selective re-deskew and re-map passes only if pose/clock/bias changes are observable and held-out/common-sample metrics improve. Rebuild affected submaps from original rays; never mutate an accepted frozen generation or blindly append duplicate factors.

**Former proposal:** R1–R3 maximum is a possible experiment configuration, **not** a requirement for the initial map pipeline.

### OD-015 — Keyframe and bias node policy

**Reopened threshold experiment.** Compare current draft **0.20 m / 5°** with older provisional **0.3–0.5 m / 10–15°** and time/uncertainty triggers on *identical* runs. A stationary robot does not need endless graph nodes. Anchor and submap lifetimes are independent; do not equate scan insertion count with graph node count.

**Bias knots:** optional future observable gyro-drift correction, not required in first pose-only graph. Evidence includes graph growth, accuracy/loops and CPU/RAM.

### OD-018 — Loop retrieval and verification

**Selected staged.** Start offline with frozen-submap spatial candidate retrieval, temporal/source intersection rejection, geometric coarse alignment, Ceres quality, correlated-factor vetting and graph-consistency rollback. A robust loss or descriptor score is never alone sufficient to admit a loop. Reject exact query-in-target self evidence and repeated heavily overlapping scan memberships.

**Deferred:** rotation-tolerant global signatures, many candidate indexes and Pi online loop closure until false-positive/recall evidence exists.

## 5. Runtime, relocalization, recording and integration

### OD-016 — Processes and workers

**Reopened; previous exact four-process mandate superseded.** Offline workstation CLI can run in one process. Pi acquisition/raw recording/localizer have separate *logical ownership* and bounded nonblocking paths; split them into distinct services where fault injection or operational need warrants it. Web Lab is optional and cannot own sensor devices or block acquisition. Separate graph process is not a default.

**Compare later:** fault/restart and Pi CPU/memory results for bounded workers versus separate recorder/acquisition/localizer and `SOCK_SEQPACKET` IPC. A process count is not itself correctness evidence.

### OD-017 — Native CMake targets and SDK exports

**Reopened; previous fixed eight-library/four-export/three-daemon initial inventory superseded.** Build actual `slam_core`, mapping, storage, matcher, batch graph and apps **as functionality arrives**. Keep portable API names/ABI disciplined and solver types private; exported targets are chosen when a real consumer exists. Do not create empty libraries or require Web Lab/Pico/ROS for headless synthetic tests.

**Evidence:** staged CMake/CTest builds, dependency DAG and installed clean-prefix consumer when packaging is implemented. Future eight-lib deployment profile is optional.

### OD-019 — Pi online graph topology

**Deferred/reopened.** Known-map localization does **not** need GTSAM. Workstation batch R0 is the reference. Compare modest keyframe incremental iSAM2 with source-correct submap-only summary designs using the same prepared datasets and a stated Pi session duration. Arbitrarily retained historical submap variables grow unbounded; budgeted `update()` alone does not solve memory. Do not invent independent summaries from A/B overlapping map membership.

**Gate:** actual Pi RAM/CPU/lag over extended sessions, held-out mapping parity and valid cross-source information. Choose graph only after B profile works.

### OD-020 — Global relocalization

**Reopened implementation strategy; selected safe interface.** Maintain bounded async recovery, spatially distinct top-K hypotheses, multi-scan consistency, explicit partial/unsearched/ambiguous coverage, and no trusted map correction on a sole low-residual candidate. Compare hierarchical BnB, AMCL/Beluga-style particle localization and hybrids on repeated warehouse/kidnap cases and Pi limits. Do not demand exhaustive 360° map search **as an unconditional first implementation guarantee**.

**Critical safety:** incomplete search is not unique localization, and unresolved rival hypotheses retain LOST/RELOCALIZING validity. Track false confident locks and abstention separately from success.

### OD-021 — Backpressure and raw durability

**Selected invariant; all numeric caps unmeasured.** Acquisition must not block indefinitely on solver, graph, disk or browser. Use independent bounded queue item/byte limits, source gaps/ages, explicit SLAM stale-scan drop and raw recorder sticky `INCOMPLETE` on required loss. Distinguish queue/write from `fsync` durable high-water and verified finished MCAP.

**First tests:** generated worst-case events, delayed/full recorder and lost data; **Pi-specific capacity** waits for measured storage/thermal behavior. No claimed losslessness under arbitrary disk stall.

### OD-022 — Localization tracking state and publication

**Selected four states; numerical thresholds candidate only.** `TRACKING`, `UNCERTAIN`, `LOST`, `RELOCALIZING` with typed cause/age/covariance status, current global-pose validity and host-steady freshness checks. Start without a valid map anchor in LOST, not TRACKING. No authoritative current pose in LOST/RELOCALIZING; odom stays continuous only when the estimator supports it. Mapping is suspended while untrusted.

**Tune later:** covariance/age/coverage/hysteresis/rejection thresholds from held-out synthetic/public data and later real Pi recordings. The former 300/1000 ms or 0.15 m/5° gates are **not validated defaults**.

### OD-023 — Raw Protobuf MCAP

**Selected staged.** Authoritative recording uses native versioned Protobuf MCAP, original LD19 packets/CRC when captured, raw MPU6050 and actual wheel observations, source and host stamps, calibration/clock model/events, schema metadata and gap/completeness status. Protobuf schemas and codecs stay private to storage; no ROS-typed recording requirement for raw replay.

**First implementation:** linear writer/reader and deterministic processing order; no semantic checkpoint dependence. Actual sensor capture is parallel to synthetic packet/replay testing.

### OD-024 — Deterministic seek/checkpoint

**Reopened; return to older provisional simplicity.** Deliver deterministic sequential start-to-finish replay, reset and step on synthetic/public prepared data first. MCAP indexes, near-30-second semantic checkpoints, full pending-worker-state snapshots and exact live historical as-of-time replay are **not first milestones**.

**Gate:** demonstrate needed interactive seek speed/huge dataset size using measured baseline; ensure indexed optimization preserves original logical event order. Never claim historical exactness without recorded asynchronous commit choices.

### OD-025 — ROS 2 adapter

**Selected isolation, optional implementation.** If ROS integration is needed, start with independently built **Jazzy on Ubuntu 24.04** and native IPC/SDK integration after the headless pipeline works. Preserve single REP-105 TF authority, true acquisition timestamps, `use_sim_time` jumps, proper scan/point timing, late `/map` readers and status-gated global pose. Native Protobuf MCAP and ROS-typed rosbag2 MCAP are different formats, with conversion explicitly labeled.

**Gate:** ROS-free native installed core; adapter/TF/QoS/time integration only when implemented and tested. Any transitive Boost component requires separate review.

## 6. Implementation/evidence order

1. **C0/V0:** Buildable portable core, time/SE(2), synthetic apartment/warehouse and no-network tests.
2. **C1/V1:** Public dataset manifest/import; gyro/no-wheel source behavior, scan assembly, reproducible mapping primitives.
3. **C2/C3/V2:** Implement Ceres matching first, then one-pass offline GTSAM mapping, verified loops and a native map export on synthetic/prepared public data. The [implementation contract](implementation-contract.md) lists C0–C5 work items; [Benchmarks](benchmarks-and-datasets.md) defines V0–V5 evidence stages independently.
4. **C4/V3/V4:** Evaluate held-out known-map localization on the workstation first, then replay the same prepared datasets on Pi before source-connected tests. Measure latency, thermal/memory and failure behavior before adding caching/graph complexity.
5. **C5/V5:** Optional online Pi graph after comparison with offline reference, bounded real session duration and a measured resource policy.

**Parallel H0–H3:** Pi LD19/MPU6050 hardware acquisition, native raw MCAP and controlled calibration recordings. It is encouraged early but **not a prerequisite for C0–C3 synthetic/public benchmark acceptance**. Real device data subsequently validates hardware clock, drift, vibration and real trajectory behavior.

## 7. How to accept a more complex design

A proposed optimization or alternate graph/recovery algorithm should carry a short decision record:

~~~markdown
### OD-XXX — Change proposal
- Status: candidate / under evaluation / selected / superseded
- Problem seen in baseline: measured failure, profile or feature need
- Alternatives on the same dataset: simplest reference and candidate
- Evidence: dataset hash, truth class, sensor capability, software/config/map/calibration
- Results: errors/false accepts/abstention plus latency, CPU/RSS and failure counts
- What changes: public interface, map/recording compatibility, rollback
- Revisit trigger: observed regression or changed hardware/requirements
- Review/approval: actual contributor(s), date and rationale
~~~

If a candidate offers no material improvement or cannot be evaluated honestly, **keep the simpler implementation** without weakening the rigorous data, time, map, evidence, and pose-validity contracts.
