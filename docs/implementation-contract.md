# 2D Fusion SLAM — Implementation Contract

**Status:** Revised implementation policy v0.2; applies to future code and documentation  
**Date:** 2026-10-31  
**Related:** [Architecture](architecture-specification.md) · [Technical specification](technical-specification.md) · [Benchmarks](benchmarks-and-datasets.md) · [Design decisions](open-design-choices.md)

> The repository now has **C0 foundations and early C1 sensor/motion/mapping primitives** (time/SE(2), synthetic scenes, scan assembly, known-mask occupancy, LD19 packet/CRC/rollover parser, sampled trajectory/deskew and four local CTest suites). Most application targets and benchmark features below remain staged plans rather than existing software; see [implementation status](implementation-status.md). The first implemented algorithms and benchmarks use **synthetic and opt-in verified public datasets**. The real LD19/MPU6050 recorder is a **parallel development path**, not a gate for the initial synthetic/public-data vertical slice.

## 1. Policy and precedence

- **MUST / MUST NOT** is a correctness, interface or contribution requirement when the feature exists. **SHOULD** is a recommended default with an explained exception. **MAY** is optional.
- The [Architecture specification](architecture-specification.md) governs three runtime profiles, module ownership and invariants; the [Technical specification](technical-specification.md) governs sensor/math behavior and acceptance; [Benchmarks](benchmarks-and-datasets.md) governs evidence/truth/experiment reporting; [Design decisions](open-design-choices.md) governs decision stage and implementation deferral.
- An unimplemented optional feature is **not** a violation because its historical design had a target count, worker policy or benchmark name. A feature that is implemented MUST meet applicable correctness and failure contracts.
- Resolve conflicting requirements by revising the documents and recording the rationale. Do not silently choose one contract.
- No experiment, compile, hardware validation or numerical performance can be called successful without a reproducible result.

## 2. Fixed tools and dependency boundaries

| Topic | Contract |
| --- | --- |
| Language | C++20 for native host code/tests |
| Build | CMake ≥3.25, target-based configuration, Ninja where available |
| Tests | Catch2 v3 and CTest, Catch2 fetched via pinned FetchContent **only when testing enabled** |
| CLI | cxxopts via pinned FetchContent for native application command-line parsing |
| Geometry/math | Eigen, document actual version and transitive public header use |
| Local scan optimizer | Ceres private to matcher implementation; optional to a geometry-only/recording target |
| Batch graph | GTSAM private to offline graph implementation; **not** a known-map Pi localizer prerequisite |
| Primary platforms | Linux ARM64 Raspberry Pi 5 and Linux x86-64 workstation |
| Reproducibility | Docker images/toolchains pinned; datasets/mapping artifacts mounted separately |
| ROS | Optional independently built adapter; **no** ROS/DDS/rosbag2 types in portable core/API |
| Pico / hardware | Pi-side LD19/MPU6050 drivers in separate platform adapter; Pico optional after evidence |
| Boost | Any new direct or transitive Boost use requires explicit component-level dependency review/approval |
| GitHub Actions | No automatic workflows or workflow execution unless explicitly requested |

Do not fetch heavy dependencies at runtime. Pin Catch2/cxxopts to immutable SHAs, and pin compatible Eigen/Ceres/GTSAM in the toolchain image or a controlled provisioning manifest. Check licenses and compiled dependency closure. Do not silently add Boost via HTTP/ROS/storage wrappers; existing unavoidable transitive use needs disclosure and a reviewed exception.

## 3. Staged CMake targets — build only real code

The previously prescribed **exact eight native libraries, four installed SDK targets and three daemon executables are superseded as an initial acceptance requirement**. Their names may be reused as a **future deployment/package profile**, but are neither mandatory from C0 nor permission to create placeholder libraries.

### 3.1 Recommended staged inventory

| Implementation step | Useful targets, when they contain real functionality | Dependency rule |
| --- | --- | --- |
| Core + fixtures | `slam_core`, core/unit test executable, synthetic fixture utility | C++20/Eigen only; no Ceres/GTSAM/device/ROS |
| Mapping primitives | `slam_mapping`, map/scan test tools | `slam_core`; derived cache remains replaceable |
| Record/read/replay | `slam_storage` and a recorder/replay CLI as needed | Native source schemas and MCAP/Protobuf private; no ROS in public API |
| Matcher | `slam_match_ceres` (private library) and offline matching executable/tests | Ceres internal; portable matcher interface |
| Offline mapping | graph implementation (`slam_graph_gtsam`) and offline mapping executable | GTSAM internal; no hardware/IPC requirement |
| Known-map localization | `slam_engine` facade/session and Pi localizer executable when appropriate | Ceres may be runtime dependency; GTSAM optional/not linked for minimal localization |
| Pi deployment | `slam_platform_linux`, acquisition/recorder/localizer service boundaries when justified | Linux driver/IPC/scheduling restricted to outer layer |
| Later integrations | Web Lab, optional ROS 2, Pico SDK build, incremental online graph | Independently selected/built and tested |

Target names are candidate stable names, not a mandate to expose an API before an implementation exists. A single offline CLI or local benchmark test may compose concrete objects directly. Split libraries around **real reuse and tested boundaries**, not each conceptual algorithm. Keep `slam::` namespace for portable interfaces where useful.

### 3.2 Public SDK and packaging

When there is an actual external consumer, install the **implemented** portable libraries and headers under an audited `FusionSLAMConfig.cmake` CMake package with versioned target names. Exported target inventory is driven by implemented stable libraries, **not automatically four**. A future full SDK may include `slam::core`, `slam::mapping`, `slam::engine`, `slam::storage`, subject to consumer tests. Never export internal Ceres/GTSAM/Linux IPC types.

Beware **static-link target leakage**: `PRIVATE` internal static libraries can appear in installed target `$<LINK_ONLY:...>` closures. Prefer a proven shared façade/pImpl or other audited configuration when private dependencies must be hidden from consumer configuration; validate public headers, imported targets, relocation, loader closure and actual runtime dependencies. Do not claim that a solver-free public header makes its runtime library solver-free.

### 3.3 Minimal CMake patterns

~~~cmake
cmake_minimum_required(VERSION 3.25)
project(fusion_slam VERSION 0.2.0 LANGUAGES CXX)
include(CTest)
include(FetchContent)

add_library(slam_core ...)
target_compile_features(slam_core PUBLIC cxx_std_20)
set_target_properties(slam_core PROPERTIES CXX_EXTENSIONS OFF)

if(BUILD_TESTING)
  # FetchContent_Declare(Catch2 GIT_TAG <PINNED_IMMUTABLE_SHA> ...)
  # FetchContent_MakeAvailable(Catch2)
  # target_link_libraries(slam_core_tests PRIVATE slam_core Catch2::Catch2WithMain)
endif()
~~~

The snippet is illustrative and is **not runnable as written**; ellipses/placeholders need real files, versions and targets. No global compiler flags, global include paths or automatic Catch2 fetch when `BUILD_TESTING=OFF`. All owned tools must provide `--help` and meaningful nonzero failure status, and never auto-download public benchmark datasets.

## 4. Runtime topology and isolation policy

Design logical owners and bounded queues before committing to a fixed process topology:

- **Offline workstation:** single-process CLI is permitted; replay, mapping, matching and batch graph run without a supervisor, hardware or Web Lab.
- **Hardware acquisition:** Pi owns LD19 UART and MPU6050 I2C inputs plus near-event reception/acquisition stamps, device diagnostic/clock metadata and optional true wheel source. Never block sensor capture indefinitely on map optimization or UI.
- **Raw recorder:** MCAP write/sync/error and session completeness are explicit owners. Acquisition-to-recorder is a bounded independent channel, whether initially a worker or a separate process. A disk stall must be observable; inability to preserve raw data marks `INCOMPLETE`.
- **Known-map localizer:** a separate Pi service/process is appropriate once deployed or when isolation measurements justify it, but not necessary for synthetic/public-data library tests.
- **Optional Web Lab, ROS adapter, Pico and online graph:** none is required for the headless first implementation.

A four-process acquisition/recorder/SLAM/Web Lab topology and Unix `SOCK_SEQPACKET` protocol are **candidate later deployment options**, not initial MUSTs. If chosen, validate envelope version/endian/length/identity/epochs, `MSG_TRUNC`, peer restarts, gap reporting and authorization. A socket send is not a durable MCAP acknowledgment. Avoid inventing a fifth mandatory graph process.

Workers/queues MUST be bounded in item count and bytes (and age where appropriate), have explicit overload/restart decisions and preserve source sequences. Older raw-queue capacities, fixed worker counts, p95 targets and map-cache tiling sizes are **experiment inputs only** until Pi tests measure them. Coalesce redundant compute **requests**, never silently discard authoritative sensor evidence or already admitted factors.

## 5. Sensor capability and algorithm constraints

**Physical baseline:** LD19 and MPU6050. A measured wheel/steering input is optional. The source adapter reports `LIDAR_GYRO` or `LIDAR_GYRO_WHEEL` (or a documented equivalent) and its actual provenance. No wheel-only sensor data, fully observed 3-DoF motion factor or precise high-rate XY odom may be fabricated in a no-wheel build. MPU6050 accelerometer integration alone is not a supported metric XY dead-reckoner.

Require source/epoch conversion and calibration, actual beam timing bounds, MIDPOINT geometry and trajectory quality before normal map insertion. One native Ceres matcher implementation should expose **LiDAR-only** residual/Jacobian quality and any applied prior separately. Its odometry-conditioned posterior must not become an independent graph factor. An initial simple rank-3/reject geometric gate is acceptable; no rank-deficient covariance padded into false certainty. Use immutable map/EDT coherence, correct unknown-space semantics and no self-match.

First offline GTSAM graph should be a simple pose-only reference. Verified wheel+gyro can become **one fused motion factor per source interval**, rather than duplicated raw wheel/yaw factors, when supported. In gyro-only mode model yaw information only and avoid invented translational precision. A larger keyframe+submap or bias-state graph requires a separate benchmark gate. The first Pi frozen-map localizer requires **no GTSAM**.

## 6. Recording, replay and map persistence

### 6.1 Authoritative raw capture

Use schema-versioned **Protobuf MCAP** with original LD19 bytes (including CRC failures when capture permits), raw MPU6050 data, raw measured wheel samples when available, source IDs/sequence, device/reception/canonical timestamps, original clock domains and epochs, calibration/clock fit/config events, buffer/drop/errors and segment/session completeness.

Separate `accepted`, `written`, `chunk-closed`, `synced` and `COMPLETE` state. OS `fdatasync`/`fsync` and synchronized manifest publication matter; a `send()` or write success is not proof of durability. Recover only validated complete chunks/prefixes after a crash. A source-gap or unrecoverable required raw loss cannot be rebranded as a complete session. Any queue/backpressure promise applies to a **bounded stall**, never arbitrary storage failure.

### 6.2 Replay first, indexed seek later

First deliver sequential raw replay, stable total ingest ordering, deterministic tie-breaks, reset and step. Distinguish original event-time behavior from CPU scheduling; initial reproduction is **logical algorithmic determinism on the same input**. A demand for exact historical state of asynchronous published graph revisions requires logged logical commit events or a deterministic scheduler; it is **not** implied by base raw MCAP.

MCAP time indexing, 30-second semantic checkpoints, persisted in-flight worker queues and historical exact-seek restoration are **deferred** until benchmarked usefulness. A missing optional sidecar must not make a valid linear raw session unreadable.

### 6.3 Maps

Persist native versioned map generations with manifest, checksums, portable binary occupancy and known mask, local frames, calibration and source identity, derived graph/pose records where applicable, and atomic generation publication. Reject unsafe sizes, unsupported versions, missing required payloads and mismatched clock/config identities. A cache is rebuildable, not map authority. Nav2 PGM/YAML and ROS occupancy export are optional lossy interchange, not the only native representation.

## 7. Docker, testing and data policy

Development, native builds and repeatable tests SHOULD run in pinned Docker images without requiring host Ceres/GTSAM. Normal synthetic tests must run without downloads, hardware, ROS, Web Lab or Pico. A dataset provisioning tool may download **only after explicit opt-in**, validating license, source, actual bytes and hashes. Imported public records must disclose missing modalities and ground-truth quality.

Container services are introduced per working use case, rather than as an exhaustive catalog of fictional runnable targets:
- `dev` / `test` for CMake/Catch2/CTest with synthetic fixtures.
- `benchmark` / `offline-mapper` for validated prepared datasets and report artifacts.
- `recorder` / `localizer` for measured Pi deployment after basic pipeline exists.
- Optional `ros2-adapter`, `web-lab` and Pico cross-build if/when implemented.

Use non-root runtime where practical; map only required devices and explicitly writable output volumes, prefer read-only input datasets/maps, avoid blanket `--privileged` and network dependencies during run. Pin images and record toolchain/compiler/SDK versions; report actual CPU/ARM64 rather than treating x86 runs as Pi performance.

**Required test categories, as applicable:** geometry/time/clock epochs; packet/scan validity; synthetic truth under gyro-only and verified-wheel configurations; per-beam deskew; Ceres robust objective and unknown masks; map insertion/snapshot versions; GTSAM factor provenance/false loops; replay/hash and crash recovery; installed consumer tests once exported; resource/fault tests when a Pi runtime exists. The [Benchmark plan](benchmarks-and-datasets.md) is the single owner of dataset matrix and report fields.

## 8. Development milestones and evidence gates

| Stage | Deliverable | Evidence required |
| --- | --- | --- |
| **C0** | C++20 core, synthetic apartment/warehouse, time/SE(2) and test harness | Local/Docker native build, deterministic fixture checks |
| **C1** | Prepared public dataset import, scan assembly, gyro-only/no-wheel motion capability, basic occupancy | Verified metadata/truth status, unit/integration tests; no real hardware required |
| **C2** | Ceres scan matcher and quality gates against known synthetic/reference maps | Geometry-only/prior ablations, Jacobians, rank/degeneracy and false-match rejection |
| **C3** | Workstation one-pass offline mapping with GTSAM batch, verified loops and native map bundle | Graph/factor/gauge tests, false-loop rollback and map round-trip |
| **C4** | Held-out workstation known-map localization, then identical prepared-data tests on Pi and later live sensor input | Accuracy/false-lock/status first; then measured Pi latency/CPU/RSS/thermal, failure behavior |
| **C5** | Optional Pi online SLAM if measured need | Incremental-vs-offline parity, graph growth/session limit and safe degradation |

**Parallel hardware track H0–H2:** H0 confirm device electrical wiring, drivers and actual wheel telemetry; H1 minimal Pi native LD19+MPU6050 raw capture/MCAP; H2 controlled sensor timing, bias and calibration recordings. Its readiness does **not** block C0–C3; later real-data checks refine models and thresholds.

A feature is done when its implemented tests and applicable evidence artifacts exist. Documentation-only revisions need consistency/link/schema checks and an explicit statement that runtime tests are not applicable.

## 9. Git, CI and review

Prefer small descriptive commits on the working feature/review branch, push checkpoints, and never force-push or merge without explicit instruction. Do not run GitHub Actions unless explicitly requested; local tools, CMake, scripts and Docker are primary. Any workflow newly created by default must be manually triggered:

~~~yaml
on:
  workflow_dispatch:
~~~

Do not add automatic triggers, dispatch jobs or spend GitHub Actions minutes without specific authorization. Check secrets, generated maps, logs and caches before committing. Review revised docs for obsolete references to required exact target/process counts, nonexistent runnable commands and unvalidated performance guarantees.

## 10. Optional ROS 2 / Web Lab interoperability

Keep the optional ROS 2 Jazzy/Ubuntu 24.04 adapter outside native SDK/core. Use acquisition-time stamps with checked ROS clock conversion, real `use_sim_time` behavior, a single TF authority for each REP-105 edge, valid pose/status semantics and accurate `LaserScan` only when beams are sufficiently uniform; otherwise use point cloud/timed observations. `/map` should support late subscribers if implemented. Native Protobuf MCAP is **not** automatically a rosbag2-playable ROS-typed recording.

A Web Lab, if later built, consumes coherent native states and versioned maps; it may not duplicate the estimator in JavaScript or block acquisition. HTTP commands require authentication/validation and versioned applied acknowledgments. The absence of either integration does not block offline mapping or localizer algorithm tests.
