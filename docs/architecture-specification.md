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
3. **Measurement acquisition time** drives estimation; transport receipt time is retained separately, not treated as the same clock.
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
| `sensors` | LD19 parser, IMU/encoder ingest, health and raw samples | No estimator or map ownership |
| `time_sync` | Rollover, clock offset/drift, ordering and time quality | No map or solver dependency |
| `calibration` | Versioned sensor extrinsics, wheel scales, gyro bias priors and timing offsets | Invalidates derived artifacts on change |
| `motion` | Wheel/gyro state, local pose history, interpolation and covariance | No global-map jumps |
| `deskew` | Per-beam SE(2) correction and provenance | No optimizer dependence |
| `mapping` | Occupancy log-odds, submap builders, frozen snapshots and fields | No direct dependence on GTSAM |
| `frontend_ceres` | Geometric alignment, scores, degeneracy and uncertainty | No raw device or ROS I/O |
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

~~~cpp
using TimeNs = std::int64_t;  // canonical monotonic acquisition domain
struct Timestamp {
  TimeNs acquisition_ns;
  TimeNs receive_ns;
  ClockDomainId source_clock;
  TimeQuality quality;          // exact/reconstructed/uncertain/invalid
  double sigma_seconds;         // or explicit unavailable
};
struct TimedBeam {
  double range_m, angle_rad, intensity;
  TimeNs acquisition_ns;
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

Use explicit physical units, coordinate-frame labels and data validity. Define pose-covariance tangent order and left/right perturbation convention. A raw device tick, host monotonic nanosecond and ROS time are **different clock domains** until related by a calibrated mapping. The canonical domain has no automatic Unix epoch.

### 3.3 Clock alignment

~~~text
raw device tick + host receive time
            -> unwrapping/clock model
            -> estimated acquisition time (+ uncertainty)
            -> bounded out-of-order reorder buffer
            -> trajectory + scan assembly
~~~

Preserve raw timestamps, arrival times, device clock IDs, alignment version and error estimates. Validate 30-second LD19 clock rollover, dropped packets and anomalous gaps. Never silently extrapolate IMU/wheel motion through long gaps. Replay seek/backward time jump requires state reset/checkpoint reconstruction. **OD-001/002/003/004** cover timestamp unit/API details, synchronization, LD19 beam timing and interpolation tolerances.

## 4. LiDAR assembly, wheel/gyro odometry and deskew

### 4.1 LD19 acquisition

Nominal UART speed 230400 baud; nominal 47-byte packet with 12 beams and CRC. Pipeline: frame search -> CRC/length -> range/intensity and unwrapped angles -> device/host timing -> per-beam timestamp estimate -> revolution assembly.

Do not assume packets arrive at uniform intervals, the reported packet timestamp labels a known first/last beam, or every revolution contains exactly 450 beams. Store full source packets and separate beam-time **quality**. Scan seam, missing packets, partial revolutions and range filtering are **OD-003/005**.

### 4.2 Continuous local motion

Wheel travel gives local translation; gyro integration (with estimated bias) gives continuous yaw. For a differential robot:

~~~text
ds = (ds_R + ds_L)/2
dtheta_wheel = (ds_R - ds_L)/baseline
dtheta_gyro = sum((omega_z - bias) * dt)
x_next = x + ds * sinc(dtheta_gyro/2) * cos(yaw + dtheta_gyro/2)
y_next = y + ds * sinc(dtheta_gyro/2) * sin(yaw + dtheta_gyro/2)
yaw_next = wrap(yaw + dtheta_gyro)
~~~

The exact arc formula is for a constant-curvature interval. Support an Ackermann model when actual steering measurements are available; steering commands alone are not measurements. Monitor wheel/gyro disagreement and slip. Gyro bias and wheel calibration uncertainty MUST appear in prediction uncertainty. The MPU6050 does **not** provide absolute yaw. Simple predictor versus EKF is **OD-006**.

### 4.3 Full per-beam SE(2) deskew

For a beam in laser coordinates at time `t_i`, deskew into the laser frame at reference time `t_ref`:

~~~text
p_laser_ref =
    inverse(T_odom_base(t_ref) * T_base_laser)
  * (T_odom_base(t_i) * T_base_laser) * p_laser_i
~~~

Use a **continuous odometry trajectory** within the scan, not a graph-corrected map transform that may jump. Translation, yaw and extrinsic lever arm all matter. Reference time (midpoint vs end), interpolation/extrapolation policy and offline re-deskew rules are **OD-004/014**. Raw beam data stays immutable.

## 5. Ceres scan matcher and front-end measurement accounting

Against a frozen submap-local Euclidean distance-to-obstacle field `D_j`, a proposed LiDAR-only residual is:

~~~text
r_i(x) = D_j(T_submap_base(x) * T_base_laser * p_i) / sigma
cost   = 0.5 * sum_i robust_loss( r_i(x)^2 )
~~~

**Ceres receives the unsquared distance residual**; the optimizer squares it internally. The gyro/wheel predicted pose is the **initialization and search window**, not an automatically added independent prior. Ceres uses bounded iterations/robust losses, interpolated fields and explicit outside-map/unknown-cell handling.

The matcher outputs: measurement-only score, convergence state, inlier count, overlap, reference submap/version, candidate transform, eigenvalue/condition diagnostics, conservative uncertainty and failure reason. A good residual is not a guarantee of correct global pose in a repetitive warehouse. Provide a coarse correlative fallback and use temporal/graph consistency gates.

| Evidence | Prediction / deskew | Graph constraint | Information-accounting rule |
| --- | --- | --- | --- |
| Wheel increments | Translation, predicted pose | Relative wheel factor | Do **not** additionally insert the same fused predicted pose as a separate factor |
| Gyro rate | Yaw trajectory and deskew | Bias-aware integrated yaw factor | LiDAR deskew conditioned on gyro creates correlation; do not claim full independence |
| LiDAR | Local geometric residual vs frozen map | Scan-to-submap constraint | Avoid self-matching and redundant insertion of the same scan |
| Revisit scan | Candidate retrieval/verification | Robust loop factor | Verify and gate false matches and repeated evidence |

Raw Hessian inverse is not automatically an independent measurement covariance; map uncertainty, shared gyro conditioning, degeneracy and outliers matter. Conservative inflation or unavailable covariance should be explicit. Field/interpolation **OD-007**, covariance/gating **OD-008**, global search **OD-009**.

## 6. Occupancy grids and immutable submaps

Submap lifecycle:

~~~text
accepted scan -> active occupancy builder -> frozen immutable snapshot
                       |                       |
                    next scan             distance field /
                    insertion            matching / loop closure
                       |
              when complete, create new active submap
~~~

- A scan MUST match a map that has not yet included it.
- Document occupied/free updates, unknown handling, range truncation, log-odds clamping and dynamic rejection.
- Each match pins one immutable map/submap revision. Optimizing global submap poses does not rewrite the submap's local raster.
- Offline re-deskew or re-mapping creates a **new artifact**, not a silent mutation of previous outputs.
- Live localization only reads published frozen map tiles; no map mutation or experimental Web Lab edits on the production bundle.
- Resolution, overlap/freeze trigger **OD-010**; distance-field storage and snapshot/eviction **OD-011**.

Proposed map bundle:

~~~text
map_name/
  manifest.json          # format version, source dataset, hashes, frames, resolutions
  calibration.yaml       # extrinsics, timing model, wheel parameters
  occupancy.*            # ROS-compatible grid origin incl. yaw
  submaps.*              # immutable local textures + poses
  distance_fields.*      # versioned derived cache
  graph.*                # factors/vertices and provenance
  trajectory.*           # timestamped corrected poses
  metrics.json           # optional, truth class explicit
~~~

Physical serialization, checksums, schema migration, atomic export and tile indexing are **OD-012**. PGM/YAML export is supported but does not preserve graph structure or all probabilistic occupancy information.

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
2. Assemble/deskew current scan using time-indexed local trajectory.
3. Match against nearby **frozen** map tiles with Ceres; verify observability and quality.
4. If accepted, produce a global pose at scan reference time and update `T_map_odom` using time-aligned local odometry.
5. Keep local odom publishing even if global correction is late; mark global pose freshness explicitly.

Tracking states:

| State | Meaning | Behavior |
| --- | --- | --- |
| `UNINITIALIZED` | No valid map/global pose | Do not publish a valid global fix |
| `TRACKING` | Fresh, observable consistent matches | Publish global correction and quality |
| `UNCERTAIN` | Degenerate environment, low overlap, lag or mismatch | Conservative uncertainty, widened bounded search |
| `LOST` | Repeated mismatch / invalid calibration or clocks | No valid global fix; start global recovery |
| `RELOCALIZING` | Alternative candidates undergoing validation | Require cross-scan consistency before adopting pose |

Global recovery may use adaptive particles (AMCL/Beluga approach) or hierarchical correlative search; choose from evidence **OD-020**. Transition hysteresis and publication semantics **OD-022**. Never mutate the frozen map during relocalization or mistake a single local minimum for a verified global hypothesis.

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

MCAP channel/schema **OD-023**, seek/checkpoint format **OD-024**, ROS distro/QoS defaults **OD-025**.

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
| Device/time | LD19 CRC/47-byte parsing, 30 s tick rollover, time offsets, missing/late samples and uncertainty |
| Geometry | SE(2) composition, REP frames, rotated occupancy origin, Jacobians and extrinsics |
| Motion/deskew | Gyro bias, wheel slip, full SE(2) ray endpoints, lever arm, gap/extrapolation behavior |
| Ceres | Distance-field interpolation, convergence, false matches, corridor degeneracy, robust residuals |
| Mapping/graph | No self-matching, immutable submaps, wheel/gyro factor independence, bias factor Jacobians, loop rejection |
| Localization | Map->odom correctness, stale/lost states, kidnapped recovery and frozen map integrity |
| IO/replay | MCAP and map schema roundtrip, version rejection, deterministic replay, interrupted writes |
| Adapters | ROS-free core build; optional ROS 2 frames/scan/maps/QoS; Web Lab real backend responses |
| Performance | Equivalent datasets on ARM64 and x86-64, capture-to-pose tail latency and resource use |

Implementation priority: **P0** toolchain/geometry/synthetic fixtures -> **P1** recorder/time sync -> **P2** motion/deskew -> **P3** Ceres/submaps -> **P4** GTSAM offline mapping -> **P5** frozen-map localization/recovery -> **P6** online SLAM. Web Lab can start as a synthetic replay viewer and expand without blocking estimation development.

## 13. Decision links and references

The companion [open design choices register](open-design-choices.md) contains **OD-001–OD-025** with priority, alternatives, evidence and decision gates. Proposed defaults here are **not approved decisions** until marked Accepted there.

External technical references:

- [ROS REP-103](https://reps.openrobotics.org/rep-0103/) · [ROS REP-105](https://reps.openrobotics.org/rep-0105/)
- [Ceres nonlinear least-squares residuals and robust losses](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html)
- [GTSAM iSAM2](https://borglab.github.io/gtsam/isam2/) · [MCAP](https://mcap.dev/)
