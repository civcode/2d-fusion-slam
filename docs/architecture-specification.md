# 2D Fusion SLAM — Architecture Specification

**Status:** Revised design baseline v0.2; implementation and measurements pending  
**Date:** 2026-10-31  
**Scope:** ROS-independent C++20 2D LiDAR/gyro SLAM, workstation offline mapping, Raspberry Pi 5 known-map localization and optional later onboard SLAM.  
**Related:** [Technical specification](technical-specification.md) · [Implementation contract](implementation-contract.md) · [Benchmarks and datasets](benchmarks-and-datasets.md) · [Design decisions](open-design-choices.md)

> This is an intended architecture, **not a completed SLAM implementation**. The checked timestamp/SE(2) core, synthetic fixtures, timestamp-aware scan assembler and basic occupancy grid are implemented (without device drivers, deskew or full mapping pipeline); see [implementation status](implementation-status.md). A selected interface or algorithm is not a tested performance claim. The original detailed proposals are historical design input, not mandatory first-implementation requirements.

## 1. Architectural strategy and deployment profiles

Build **one portable core and three compositions**, not one maximum-feature implementation copied onto every host. The first algorithmic implementation, tests and comparisons run on **deterministic synthetic fixtures and opt-in, verified public benchmark datasets**. Independently prepare an LD19/MPU6050 Raspberry Pi acquisition and raw-recording path so real-hardware measurements can inform later revisions. Real-device data is **not a prerequisite** for completing the first synthetic/public-data milestones; conversely, synthetic data does **not** validate hardware timestamps, drivers, thermal load or real accuracy.

| Profile | Main environment | Required functionality | Explicitly not required initially |
| --- | --- | --- | --- |
| **A — Offline mapping** | Powerful workstation, offline recorded/replayed data | Calibrated input, motion/deskew when supported, occupancy submaps, Ceres local alignment, safe GTSAM batch graph, validated loops, immutable map export | ROS, hardware connection, Web Lab, multi-process deployment, online iSAM2, checkpointed seek |
| **B — Known-map localization** | Raspberry Pi 5 with an existing verified map | Read-only map, prediction appropriate to measured sensors, bounded Ceres scan-to-map, match quality, pose validity, recovery interface, bounded queues | GTSAM, map insertion, active submap graph, globally exhaustive recovery at every scan |
| **C — Online SLAM** | Raspberry Pi 5 (later measured capability) | Proven A/B frontend, bounded active maps, selected keyframes, asynchronous incremental graph if justified, graceful degradation | Workstation batch-equivalent global optimization, unconditional historical graph growth, guaranteed global recovery within arbitrary budgets |

**Implementation order:** A starts with a synthetic/public-data vertical slice; B reuses its map and matching contracts and is benchmarked on prepared datasets first, then on Pi; C follows measured A/B evidence. The hardware recorder develops in parallel, without blocking that order.

## 2. Invariants that must hold from the first implementation

1. **ROS-independent core.** Native geometry, estimator, map and storage interfaces use portable C++20 types. Ceres and GTSAM remain private solver implementations; Linux acquisition, transport and optional ROS 2 belong at outer adapters.
2. **Measurement-time integrity.** Canonical time is checked signed 64-bit nanoseconds with clock domain and epoch. Retain original device timestamp, host reception timestamp, timing model, uncertainty and raw bytes separately. Use host-steady time, not replay or ROS `/clock`, for CPU deadlines.
3. **Explicit SE(2) geometry.** REP-103/105 conventions: X forward, Y left, CCW-positive yaw, transforms expressed as `T_target_source`. `odom -> base_link` remains continuous when available; global changes are represented by `map -> odom`.
4. **No fabricated sensor observations.** Missing beams, invalid returns, unmeasured wheel velocity and unsupported trajectory intervals must never become zero motion, free space, precise covariance or graph factors.
5. **Immutable evidence and map versions.** Source recordings, calibration, map generations and graph factors retain stable IDs. Occupancy/known mask is authoritative; EDT, search pyramids and descriptors are derived, disposable caches.
6. **No double-counted measurements.** A Ceres match conditioned on wheel/gyro prediction is not an independent LiDAR-only graph factor. Deskewed geometry and motion factors can remain correlated even if the explicit prior is removed.
7. **Fail-closed tracking.** A stale, ambiguous, degenerate or unsupported global pose is not a current valid localization. Mapping must not insert untrusted scans.
8. **Isolation by behavior, not predetermined process count.** Acquisition and recording cannot be indefinitely blocked by matching, optimization or UI. Bounded queues, loss diagnostics, and recorder incompleteness are essential; exact threads, processes and IPC are deployment choices.
9. **Portable test inputs before hardware assumptions.** Synthetic apartment/warehouse and validated public recordings are the initial acceptance basis. Pi/hardware claims require later direct measurement.

## 3. Core pipeline and composition

~~~text
synthetic fixtures / validated public datasets / recorded native sensor events
                                  |
                      typed time + source conversion
                                  |
                   measured-sensor capability assessment
                                  |
              gyro/optional-wheel motion + trajectory provider
                                  |
               scan assembly -> optional safe deskew
                                  |
           immutable occupancy/snapshot + Ceres scan matcher
                                  |
                    match-quality / pose-validity
                       /          |             \
         offline submap map     Pi frozen-map    later online maps
         + batch GTSAM         localization     + measured backend
              |                    |                |
     immutable map bundle    bounded pose output   immutable revisions
              \____________________|________________/
                                   |
                     optional ROS 2 / Web Lab adapters

In parallel on actual Pi: LD19 + MPU6050 (+ verified wheel source if present)
   -> adapter / original bytes / acquisition-receipt clocks
   -> bounded authoritative MCAP recording -> identical replay interfaces
~~~

A workstation command must load a dataset and run without a hardware driver or an IPC supervisor. A known-map localizer must run without a graph optimizer. The acquisition/recorder path need not run an estimator.

### 3.1 Minimal stable native boundaries

- **`SensorEvent` / `ClockModel`:** raw source, original timestamp(s), canonical-time conversion, calibration/clock revision and validity.
- **`TrajectoryProvider`:** timestamp-addressable `SE(2)`, supported interval, covariance/quality, interpolation/extrapolation status, source IDs and capabilities; it must be usable without wheels.
- **`ScanMatcher`:** deskewed observations, pinned immutable map revision, optional prediction; returns tracking pose, geometry-only diagnostics and *optional separately validated* graph-eligible constraint.
- **`MapSnapshot` / `MapBundle`:** occupancy/known mask and local frame, calibration, source hash and graph/map revision; EDT cache never masquerades as authority.
- **`Session` / `PoseEstimate`:** mode, source time, frame, covariance/quality or unavailable status, tracking state, validity, map generation and correction generation.

Do not create speculative plugin factories or empty CMake libraries solely to implement this conceptual decomposition.

## 4. Sensors, time, motion and scan assembly

### 4.1 First-class motion capabilities

Initial physical assumptions are **LD19 LiDAR + MPU6050 IMU + Raspberry Pi 5**. Wheel encoders, wheel velocity and measured steering feedback are **optional, unverified capabilities**; they are not presumed by the core.

| Capability | Supported motion evidence | Permitted estimate |
| --- | --- | --- |
| `LIDAR_GYRO` | Measured gyro rate, calibrated bias and time; accepted prior scan matches when available | Short-term yaw trajectory, conservative scan-derived interscan displacement and growing uncertainty; no claimed high-rate metric XY odom or exact translational intrarevolution deskew |
| `LIDAR_GYRO_WHEEL` | Above plus **observed** wheel displacement / speed and, where required, measured steering | Configured differential/Ackermann body twist, full translation+yaw SE(2) trajectory with validated uncertainty |
| Unknown/unhealthy source | Missing calibration, invalid epoch or gap | Explicit unavailable/degraded estimate; never fabricated zero motion |

Use a minimal bias-aware gyro integrator first; stationary-window bias calibration is gated by real evidence of stationarity. An EKF, spline or fixed-lag smoother is optional only after measured benefit. The MPU6050 accelerometer is not a dependable source of long-term planar translation by naive double integration.

### 4.2 LD19 and clock alignment

LD19 native packet parsing and physical reference interpretation stay in an acquisition adapter. Preserve original **47-byte packets with 12 sample slots**, raw millisecond counter, CRC status, per-packet angles and motor rate, reception event and frame ordering. The roughly 30-second counter wrap and packet loss are handled with explicit clock epochs/gaps.

Per-beam relative time may initially be reconstructed from unwrapped packet angles and signed motor speed, but **the physical event represented by the packet timestamp is unverified**. A source-to-host fit from one-way arrivals cannot independently identify a constant transport delay and clock offset. Therefore preserve timing uncertainty and permit offline recalibration without modifying raw measurements. Imported public datasets with preconstructed `LaserScan` are **not** evidence that the physical LD19 timestamp model is correct.

### 4.3 Partial scans and safe occupancy

Assemble observed beams by a fixed angular seam with verified direction; report partial scans, missing angular sectors, actual timestamp bounds, invalid ranges and trustworthy no-return classifications. A missing sample is not an observed maximum-range ray. Invalid or missing points **never clear or occupy** a cell. Valid hit rays clear only cells *before* the endpoint, then update the hit endpoint; trustworthy no-return may clear only when sensor semantics and configuration explicitly permit it. Scan-matching and map-insertion eligibility are separate quality decisions.

### 4.4 Midpoint trajectory and deskew

For a valid piecewise-constant body twist `xi_k`, define
`T_odom_base(t) = T_odom_base(t_k) Exp_SE2((t-t_k) xi_k)` within its supported interval; source times are converted to seconds safely. For scan timestamp bounds from actual time-valid observed slots, `t_mid=t_start+floor((t_end-t_start)/2)`. For beam `p_i` at time `t_i` in the laser frame:

~~~text
p_laser_mid =
  inverse(T_odom_base(t_mid) * T_base_laser)
  * (T_odom_base(t_i) * T_base_laser) * p_i
~~~

This is the **reference geometric contract**. In `LIDAR_GYRO` mode, do not claim accurate full XY deskew unless translation is independently supported. Compare no deskew and gyro-only yaw correction first; apply full SE(2) when a credible translational trajectory is available. An insertion requiring unsupported time coverage, crossed clock epochs or unjustified extrapolation is rejected. Extrapolation for limited live prediction must be explicit, bounded and quality marked.

## 5. Matching, maps and offline graph

### 5.1 Ceres scan matching

Start with one robust, measurable scan-to-occupancy-derived **unsigned truncated Euclidean distance field** objective, using bilinear sampling and one independently robustified scalar hit residual per beam. The authoritative occupancy grid has a separate **known/unknown support mask**; an unknown or out-of-map sample is not a valid obstacle match, is not silently zero residual and contributes no false overlap. Define finite boundary behavior and reject field-free/low-support/degenerate results.

Initially compare: (a) geometry-only objective seeded from prediction and (b) geometry objective with a covariance-weighted prediction regularizer. Keep prediction/prior and boundary penalty provenance separately from LiDAR-only residuals and Jacobians. A Ceres success flag is not evidence of a correct global pose. Start with robust overlap, residual/inlier and rank-3-or-reject gating; add projected rank-deficient corrections and fine covariance calibration only with independent evidence.

### 5.2 Occupancy submaps and snapshots

Start at a **candidate** 5 cm grid resolution with clamped log-odds and an observed/unknown mask, individual scan IDs, checked ray traversal and hit-over-free precedence. Two overlapping active submaps with **90/180 accepted insertion counts** are a reproducible **experiment configuration**, not a permanent system invariant; benchmark alternative sizes and count policies before locking them. Frozen submap local occupancy and origin are immutable; graph optimization changes only the global submap pose.

Use **complete coherent immutable EDT snapshots first**, including a pinned occupancy/known-mask source revision, grid origin/resolution and matching configuration. Avoid synchronous rebuild per beam; bound asynchronous work and staleness. Incremental 32×32-cell tiles, cache eviction and pyramids are potential later optimizations if Pi measurements justify them. Do not mix map and distance-field revisions or match an observation against itself.

### 5.3 Offline graph authority

The first workstation implementation should build a **simple pose-only GTSAM reference graph** with explicitly fixed gauge, measured-source provenance, validated relative motion where available and separately justified LiDAR/loop observations. Do not add wheel factors in a no-wheel configuration. A gyro-only yaw observation does not constitute a fully constrained SE(2) motion factor.

A frontend pose incorporating an odometry regularizer is a **conditioned posterior**, not independent geometry. Even a geometry-only objective can share motion and map uncertainty through deskew and raster construction. Graph admission requires a defensible LiDAR-only or explicitly conditioned observation; if unavailable, skip the factor and retain the diagnostic. Do not combine fused wheel+gyro motion with independent wheel/gyro factors representing those same samples. Reject self-matching, shared-source duplicate constraints and unverified loops.

A joint keyframe+submap batch graph, sparse bias knots, rank-projected LiDAR factors and correlated cross-factor models are **future controlled comparisons**, not prerequisites for a basic offline map. Preserve original source factors so later graph experiments do not depend on approximated summaries.

### 5.4 Loop closure and offline refinement

Stage I candidate loop retrieval uses frozen-map spatial proximity and time/source exclusions, followed by geometric coarse alignment, Ceres refinement, match-quality checks and provisional graph consistency. Do not treat candidate score or a robust loss as verification. A signature-based global index is a later alternative.

Run one complete deterministic replay/optimization pass `R0` first. Only after held-out data shows improvement, permit bounded selective re-deskew/remap/graph iterations. Publish new immutable generations, never mutate accepted frozen submaps; compare candidate maps on **common or held-out evidence**, not just their own changed training objective.

## 6. Pi known-map localization and later online SLAM

### 6.1 Frozen-map localizer

Load verified map generations in **read-only mode**. Normal tracking uses Ceres locally with bounded input queues and computation; no GTSAM backend is required. Report `TRACKING`, `UNCERTAIN`, `LOST`, `RELOCALIZING`, explicit confidence/covariance status and acquisition time. No globally valid pose is published from an old fix with a newly fabricated timestamp.

The recovery API owns bounded candidate searches, spatially distinct temporal hypotheses, ambiguous/partial coverage statuses and a fail-closed acceptance policy. **Do not commit to exhaustive full-map BnB or particle filtering before comparing them** on held-out repetitive scenes and Pi budgets. A partial search is not a proof of unique localization. Suspend mapping and globally authoritative corrections while untrusted.

### 6.2 Pi online SLAM, conditional

Enable active mapping, sparse keyframes and a low-priority graph only after the Pi localizer is shown stable on identical prepared recordings and actual hardware measurements. Compare a modest incremental keyframe graph with a submap summary graph against the workstation batch reference. Summaries must preserve justified information and source correlation; overlapping submap memberships alone are not independent factors. Define and measure a **supported session duration and memory ceiling**; all-history iSAM2 growth cannot be called bounded merely because each update is scheduled in a worker.

On overload retain raw input integrity and local tracking as far as supported; defer or coalesce **jobs**, never silently erase admitted measurements. Offline workstation remapping can later produce a better map generation from the raw evidence.

## 7. Runtime, persistence and integrations

Authoritative raw capture is native **schema-versioned Protobuf MCAP**, with original LD19 bytes, MPU6050 samples, any truly measured wheel events, time/clock/calibration changes, source continuity, recording failures and run metadata. Begin with linear start-to-finish deterministic replay. MCAP indexing, exact historical asynchronous commit restoration and semantic checkpoints are optional later steps with separate tests.

The recorder distinguishes queued/written from durable/synced data. A finite nonblocking acquisition queue cannot guarantee losslessness under arbitrary disk stalls: mark recording `INCOMPLETE` and preserve recoverable evidence. Workstation offline replay and synthetic tests require **neither** a four-process supervisor nor actual hardware.

Process count and CMake target count are **not correctness properties**. Start with actual useful targets, then split acquisition/recorder/localizer where fault, scheduling and operational evidence justifies isolation. Web Lab, optional Pico and ROS adapter do not gate headless core builds. The optional ROS 2 adapter uses REP-105 TF authority, correct acquisition-time stamps and explicit pose validity; it must never require ROS libraries in public core headers.

Portable map bundles comprise a versioned manifest with source/calibration/clock identity, typed binary occupancy/known masks and graph metadata as available, source/checksums and crash-safe generation publication. JSON/PGM/YAML map interchange is lossy and is secondary to the native bundle.

## 8. Acceptance stages and design change control

Each decision separately records: **selected direction**, **implemented**, **synthetic tested**, **public-dataset evaluated**, **real-hardware evaluated**, **Pi validated** (as applicable). Never infer later stages from a design approval. Initial synthetic and public-data verification leads; hardware capture is prepared in parallel and later closes timing/accuracy/deployment gaps.

Change an algorithm or optimize a cache/graph/runtime only after a reproducible baseline identifies the weakness. Record dataset hash/truth class, build revision, config/calibration/map IDs, p50/p95/p99 latency where measured, false accept/reject and failures, memory/CPU, alternative results and rationale. For exact test matrix and milestones, see [Benchmarks](benchmarks-and-datasets.md); for currently staged/open choices, see [Design decisions](open-design-choices.md).
