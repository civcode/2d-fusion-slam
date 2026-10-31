# 2D Fusion SLAM — Benchmarks and Datasets

**Status:** Revised benchmark plan v0.2; C0, scan assembly, occupancy, LD19 packets, trajectory/deskew, exact EDT and geometry-only synthetic matching tests run locally; full benchmark campaigns not yet run  
**Date:** 2026-10-31  
**Related:** [Architecture](architecture-specification.md) · [Technical specification](technical-specification.md) · [Implementation contract](implementation-contract.md) · [Design decisions](open-design-choices.md)

## 1. Primary validation strategy

**The first implementation, tests and algorithm benchmarks MUST use synthetic and validated, prepared public benchmark data.** This enables repeatable correctness, controlled ablations and workstation offline SLAM before Raspberry Pi hardware is ready. Actual **LD19 + MPU6050** acquisition/recording is a **parallel engineering track**, not an initial test dependency. Controlled real recordings are later indispensable to validate device-clock behavior, temperature, vibration, timing delays, and Pi workload; no synthetic/public success may be described as such validation.

The project has three separate quality questions:

1. **Workstation offline SLAM:** Does a replayable algorithm produce a consistent map and global trajectory, with conservative factor accounting and independently meaningful error metrics?
2. **Known-map localization on Pi 5:** Does a map built offline support safe, bounded local tracking with truthful unknown/lost states? Begin with workstation replay of held-out data, then run the **same prepared inputs** on Pi, then real hardware.
3. **Later Pi online SLAM:** Does adding online mapping and graph optimization improve utility within a measured supported session duration, RAM/CPU envelope and safe-overload policy, compared with the workstation reference?

A baseline that works on public data but not real LD19 timing is **algorithmically validated**, not **hardware validated**. Use separate decision/evidence statuses; do not turn the absence of independent truth into invented accuracy.

## 2. Dataset classes and initial use

| ID / source | Modalities | Truth / validation status | Initial role |
| --- | --- | --- | --- |
| `synthetic-apartment-v1` | Generated LiDAR, gyro, optionally **realistic simulated wheel measurements** | Exact simulated pose and map; fixture **to implement** | **Mandatory network-free** basic math/deskew/mapping/localization |
| `synthetic-warehouse-v1` | Same, with repeated aisles, symmetry, dynamic clutter and missing sectors | Exact simulated pose and map; fixture **to implement** | **Mandatory network-free** ambiguity, loops, false locks |
| `rosbot-indoor-full` | Published 2D scan, IMU, wheel-derived `/odom` subject to inspection | Candidate; independent truth **not verified** | First opt-in public multi-sensor experiment **after manifest validation**; derived odom is not automatically raw ticks |
| `mit-stata-selected` | Candidate 2D LiDAR, IMU and wheel/odometry bag subset | Candidate; session/crop/hash and truth quality **not pinned** | Alternative real-world three-sensor cross-check once verified |
| `cartographer-museum-selected` | Published 2D LiDAR + IMU candidate | Candidate; independent ground-truth trajectory **not verified** | Compact public **LiDAR + gyro** smoke test; not a wheel-fusion test |
| `iilabs-independent-truth` | Candidate with LiDAR, IMU, wheel and externally measured trajectory | Deferred; exact licensed sequence/size/hash to be chosen | Later independent accuracy/covariance validation |
| `ld19-mpu6050-pi-recording` | Raw LD19 packet, MPU6050, optional **verified** measured wheel source | **Future parallel hardware track**, external reference optional | Real clock/IMU/driver/deployment validation after initial baseline |

Dataset names are **internal aliases**, not claims that a compatible file has been downloaded, licensed, converted, or measured. The former documents' indicative durations and sizes are not pinned dataset contracts; do not propagate them as verified metadata. External source pointers: [ROSBOT candidate](https://github.com/MBarah10/indoor-outdoor-multisensor-robot-dataset), [MIT Stata resource](https://projects.csail.mit.edu/stata/), [Cartographer resources](https://google-cartographer.readthedocs.io/). Link correctness and usable topic/schema content must be checked in the dataset manifest before promotion. No network use is permitted during default tests.

### 2.1 Public dataset promotion gate

Before a candidate becomes a default benchmark, record:

- Official source URL, usage/license/attribution, actual downloaded bytes and SHA-256, archived source where permitted.
- Exactly chosen session/file/crop, timestamp domain and unit, converter revision, topic inventory, frames, message frequencies and measured gaps.
- Sensor capability: `LIDAR_GYRO` vs `LIDAR_GYRO_WHEEL`; explicitly mark raw wheel ticks, wheel-derived processed `/odom`, or **no** wheel source.
- Known extrinsics, calibration quality, first/last event, sample order, simulated vs real timestamp semantics.
- Reference truth class, coordinate alignment method and whether mapping/evaluation paths are independent.
- A conversion/report smoke test and deterministic prepared-data hash.

No benchmark may silently synthesize missing wheel telemetry from command velocities or integrate MPU6050 XY acceleration as exact displacement.

## 3. Synthetic fixture specification

### 3.1 Apartment and warehouse

**Apartment:** candidate ~15 m × 12 m, rooms/doorways, a loop, turns, off-center laser mounting, varied translation/yaw and stop/rotate phases. **Warehouse:** candidate ~30 m × 20 m, parallel walls/shelves, symmetric corridors, repeated loop views, occlusion/dynamic clutter and deliberately plausible false alignments. Final geometry, motion trajectory, rates, noise configuration and random seed must be versioned alongside the generator.

Each fixture generates **time-indexed exact `SE(2)` truth**, exact ideal ray intersections, timestamped beam angles/ranges, optional intensity/no-hit, gyro source rate, optional wheel observations, and the map/truth ground frame. Generate **both** mapping traversal and **separate held-out localization traversal**; do not report localization on a map built from the same evaluation scans as independent.

### 3.2 Required variants and ablations

| Variant | Evidence | Intended comparison |
| --- | --- | --- |
| `lidar_only` | LiDAR geometry, no gyro/wheel motion contribution | Geometry baseline |
| `lidar_gyro` | LiDAR + calibrated Z gyro with bias/uncertainty, **no measured wheel translation** | Yaw deskew and sparse interscan pose vs LiDAR-only; honest XY limitation |
| `lidar_gyro_wheel` | LiDAR + gyro + **simulated or verified actually measured** wheel translation/steering | Full `SE(2)` prediction/deskew and fusion benefit |
| `lidar_gyro_wheel_degraded` | Above with slip, dropout, drift, wrong calibration | Robust failure/rejection and covariance sensitivity |

Also compare `NONE`, `GYRO_ONLY` and `FULL_SE2` deskew **only when corresponding trajectory information is defensible**. At minimum one noiseless case supports exact analytical checks; noisy cases vary bias, gyro temperature surrogate, wheel scale/slip, LiDAR outliers, partial scan gaps, timestamp jitter and variable motor RPM. Vary a clock model independently from motion truth.

Do not pretend a generated no-wheel vehicle has measured wheel odometry. A `LIDAR_GYRO` scenario may use truth internally to score full-SE2 error as an oracle comparison, but must never give that truth trajectory to the estimator while labeling it a deployable no-wheel result.

### 3.3 Map and recovery tests

- Map 5 cm vs 10 cm initially; compare alternative submap counts, including candidate 90/180 accepted insertions; verify all actual source scan memberships and no self-match.
- Corridors and repeating shelving produce deliberate rank deficiency, multiple top hypotheses and false-but-low-residual Ceres solutions.
- Kidnapped robot: unknown initial pose and teleport; report false **confident** global lock separately from correct recovery, refusal and unresolved search coverage.
- Dynamic obstacles and partially changed maps test resilience without corrupting authoritative immutable source maps.
- Reuse identical scan sequences for tracker variants and identical training/held-out splits for accuracy comparisons.

## 4. Truth and scoring integrity

| Truth class | Meaning | Permitted result |
| --- | --- | --- |
| `simulated_exact` | Generator provides exact independent pose/map | Absolute/relative errors against **synthetic** truth |
| `independent_measured` | External MoCap/survey with validated time/frame alignment | ATE/RPE and uncertainty coverage against **independent** truth |
| `derived_reference` | Trajectory derived from LiDAR, wheel, floor plan or evaluated sensor family | *Discrepancy from reference*, not independent absolute position accuracy |
| `none` | No defensible external trajectory | Relative/map consistency, residuals and runtime; **no** ground-truth ATE |

For pose metrics, state alignment (initial rigid `SE(2)`, optimized rigid `SE(2)` or none), timestamp interpolation bounds, missing-sample coverage and unit conventions. Do not fit scale for metric 2D LiDAR by default. Do not evaluate map localization against training scans without explicitly labeling self-evaluation.

Required metrics, when applicable: ATE translation RMSE/median/p95, RPE distance/yaw at specified intervals, map occupancy agreement against independent truth where possible, deskew endpoint error, Ceres convergence and residual/inlier/overlap/rank, false graph loop accepts, recovery success/false-global-lock/abstention/time, covariance consistency (only where meaningful truth exists), rejected/missing observations, dataset coverage, and p50/p95/p99 CPU/wall stage latency plus memory/thermal on target platforms.

Every rejected scan, LOST interval, unavailable metric and recovery abstention must count in the denominator/report. Missing metrics are **null with a reason**, never zero.

## 5. Initial benchmark IDs and acceptance intent

B01–B08 are **partially exercised** by local synthetic/unit tests: time/SE(2), LD19 frames/CRC/nominal counter wrap, assembly, interpolated deskew, occupancy, immutable map revision snapshots, exact full-field EDT and geometry-only robust scan-to-map reference optimization. **B07** still lacks cache concurrency/staleness and large-grid profiling. **B08** has unverified Ceres compilation/runtime, no independent real benchmark, no odom-prior ablation, no calibrated covariance, and incomplete outlier/degeneracy campaigns. B02 physical calibration, B04 gyro estimator, B06 submaps, B09–B18, public dataset evaluation and full performance campaigns are **unexecuted**. IDs describe test objectives, not necessarily existing tools. The previous B01–B33 enumerations were design proposals, **not completed evidence**.

| ID | Test / benchmark | Initial input | Required assertion or output |
| --- | --- | --- | --- |
| **B01** | Checked time/frame/SE(2) | Synthetic exact | Overflow/epoch/frame guards, Lie compose/inverse/exp, yaw wrap |
| **B02** | LD19 packet and clock model | Generated 47-byte packets, wraps, CRC faults | Exact raw bytes, timing uncertainty, gaps, no invented beam timestamps |
| **B03** | Scan assembly and return validity | Synthetic partial/missing/invalid beam sets | Accurate sectors, no phantom beams, zero invalid/missing ray clearing |
| **B04** | Motion capability/bias | LiDAR+gyro and gyro+verified-style wheel | No fake XY in no-wheel mode; bias/gap/slip diagnostics |
| **B05** | Deskew | Exact moving beams, nonuniform times | None/gyro/full comparison, eligibility and no unsupported extrapolated insertion |
| **B06** | Occupancy/submaps | Synthetic map and rays | Known-neutral distinction, hit priority, bounded allocations, versions, overlap ledger |
| **B07** | Map snapshot/EDT | Immutable occupancy and known masks | Full-field EDT geometry, unknown stencil and source hashes, no mixed revisions |
| **B08** | Ceres matcher/quality | Apartment/corridor/warehouse | Robust per-beam Huber, geometry/prior separation, Jacobians, false-match/rank rejection |
| **B09** | Offline graph/loops | Synthetic exact paths and false loops | Gauge, unique motion evidence, no conditioned-posterior factor, safe wrong-loop veto |
| **B10** | One-pass mapping / bundle | Synthetic mapping traversal | Deterministic R0, map content hash, safe round-trip/read-only import |
| **B11** | Held-out localizer | Separate synthetic traversal | Accuracy/coverage, false confidence, four-state and stale-pose gating |
| **B12** | Initial/global recovery comparisons | Symmetric warehouse, kidnapping | Correlation/BnB/particle candidates evaluated with unique top-K, false lock/abstention |
| **B13** | Public dataset importer | Selected validated public sample | Modality and truth labels, strict hashes, deterministic event order |
| **B14** | Offline public dataset ablation | Verified source and splits | LiDAR-only vs gyro vs wheel when present; no false GT claims |
| **B15** | Pi prepared-data localization | Same prepared sequences on Pi 5 | CPU/RSS/thermal/latency/backlog, no requirement for runtime GTSAM |
| **B16** | Recorder data integrity | Generated source/queue/disk faults; later Pi | INCOMPLETE on loss/sync fault, recoverable prefix, no false durable ACK |
| **B17** | Optional offline refinement | Held-out data, perturbations | New generation only if common/held-out quality improves; deterministic rollback |
| **B18** | Optional online Pi SLAM | Prepared extended sessions; later Pi | Offline reference parity, bounded session memory/update lag, safe overload |

Extensions (semantic seek, incremental EDT tiles, advanced signatures/graph condensation, ROS/Web Lab and Pico clock sync) receive **separate benchmark cases only if selected for implementation**. Their former numeric budgets do not silently become requirements.

## 6. Benchmark stages aligned with implementation

| Stage | Default execution environment | Minimum evidence |
| --- | --- | --- |
| **V0** | Workstation/Docker, network-free synthetic | B01–B08 geometry, time, motion capability and matcher correctness |
| **V1** | Workstation/Docker, opt-in verified public datasets | B13 import validation plus applicable B05/B08/ablation comparisons |
| **V2** | Workstation/Docker, synthetic and prepared public | B09–B10 offline graph/map, false loops and immutable map |
| **V3** | Workstation/Docker, held-out prepared data | B11 localizer and B12 recovery comparison |
| **V4** | Raspberry Pi 5, **same prepared datasets first**, real sensor feed later | B15 latency/thermal and B16 fault behavior, separate real sensor calibration results |
| **V5 (optional)** | Pi 5, prepared extended sessions and later hardware | B18 online graph comparative benefit, memory/session cap and recovery |

Synthetic and opt-in public tests are **the initial product acceptance basis**; the hardware path can make progress simultaneously but must not block V0–V3. Conversely no Pi performance or LD19 capture-time claim is proved by Docker tests on x86-64 or ideal synthetic scans.

## 7. Hardware track, parallel and explicitly later for algorithm validation

Prepare in parallel:
- **H0:** record Pi model/RAM/cooling/storage, LD19/MPU6050 electrical connections, mounts/extrinsics, IMU sample/FIFO settings and any *actually present* wheel/steering signals.
- **H1:** native timestamped Pi capture/MCAP recording of original LD19 packets, MPU6050 gyro/temperature, clock/time/event diagnostics and wheel raw data **only if measured**. Tests can use fake data before device is attached.
- **H2:** controlled stationary/rotation/translation recordings, LD19 30-s timestamp wrap, actual serial buffering and motor variability, IMU bias/temperature/vibration, LiDAR/gyro clock delay, reboots/gaps, optional independent surveyed checkpoints.
- **H3:** a separate map-building traversal and held-out localization traversal on actual robot; full reference claims require an external measured trajectory.

When hardware recording becomes available, add a **small, versioned, privacy-reviewed** subset to the fixture/manifest catalog or store it outside Git by checksum reference. Do not commit large raw recordings, private environments or logs by default. Re-run the same Ceres/deskew/graph ablations. Track differences from public benchmark sensor models and revise calibration/threshold assumptions accordingly.

## 8. Reproducible experiment protocol

Each run MUST declare source revision/commit, prepared dataset SHA-256, manifest/schema version, input modalities/raw-versus-derived distinction, truth class, map-building and held-out traversal IDs, configuration/calibration/solver version/hash, clock origin/epoch, generator seed (where synthetic), actual hardware architecture/CPU/RAM/cooling/storage and Docker image digest when applicable.

Repeat stochastic runs with pinned seeds and include sample count/dispersion. Configure timeout/candidate/memory limits per profile and **report actual observed values**, not an invented Pi guarantee. Track input event ordering and raw source timestamps separately from processing/publish times.

Suggested report (fields initially `null` until measured):

~~~json
{
  "schema_version": 2,
  "run_id": "example-not-executed",
  "dataset_id": "synthetic-apartment-v1",
  "dataset_sha256": null,
  "ground_truth_class": "simulated_exact",
  "sensor_capability": "LIDAR_GYRO",
  "mapping_split_id": "map-traversal",
  "evaluation_split_id": "held-out-traversal",
  "software_revision": null,
  "config_sha256": null,
  "calibration_revision": null,
  "hardware_profile": "workstation",
  "results": {
    "ate_translation_rmse_m": null,
    "false_confident_global_locks": null,
    "scan_match_p95_ms": null,
    "capture_to_valid_pose_p95_ms": null
  },
  "unavailable_reasons": {
    "capture_to_valid_pose_p95_ms": "no live capture in offline dataset"
  },
  "status": "example_not_executed"
}
~~~

The example is a **report template**, not evidence of any result. Failure, skipped/rejected and unsupported data must appear explicitly.

## 9. Selection criteria before adding complexity

Require a reproducible improvement relative to the **simplest adequate baseline** on the same prepared data, without regressing false locks, uncertainty honesty or memory/latency envelope.

| Candidate enhancement | Compare against | Adopt only when |
| --- | --- | --- |
| Predictive gyro/translation models | Basic gyro yaw, recorded source availability | Robust held-out motion/deskew improvement |
| Odometry-regularized Ceres | Geometry-only seeded matcher | Better convergence/false-lock/latency without graph evidence contamination |
| Keyframe+submap / gyro-bias graph | Simple offline pose-only graph | Better held-out accuracy/loops with justified source covariance and cost |
| Incremental EDT tiles/cache | Full coherent EDT snapshots | Measured Pi CPU/p99/memory bottleneck and exact equivalence |
| Hierarchical global BnB | Simple local search and particle/AMCL-style baseline | Better recovery/false-lock/abstention for Pi budget |
| Submap-only iSAM2 summary | Modest incremental keyframe graph and workstation batch | Valid correlation summaries, bounded memory/session and quantitative accuracy benefit |
| Exact semantic seek/checkpoints | Linear deterministic replay | Measured interactive workflow value that outweighs complexity |
| Four-process IPC | Simplest safe bounded-worker runtime | Fault-isolation/latency gain with acceptable operational burden |

## 10. Data hygiene and release gates

Default tests remain **offline and network-free**; real dataset downloads are explicit. Avoid publishing real platform performance without a real Pi, and do not invent timestamps, frame origins, source independence or hardware wheel signals. A benchmark release includes dataset manifests/hashes, test command and result logs, environment versions, truth/split integrity and failures. For hardware-specific acceptance, state the actual LD19/MPU6050 firmware/config and measured clock calibration quality.

No Github Actions are necessary; local/Docker test execution is preferred, and no workflow is run without explicit permission.
