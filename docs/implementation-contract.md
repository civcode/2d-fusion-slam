# 2D Fusion SLAM — Project Implementation Contract

**Status:** Proposed — v0.1  
**Date:** 2026-10-30  
**Applies to:** [2d-fusion-slam](https://github.com/civcode/2d-fusion-slam)  
**Related document:** [Technical Specification](technical-specification.md)

> This contract defines how the project is implemented, built, tested, run, reviewed, and delivered. It is a set of engineering requirements, **not a statement that the listed build files or executables already exist**. Exact dependency revisions, images, and CLI flags are selected and committed during initial scaffolding.

## 1. Contract language, scope, and precedence

- **MUST / MUST NOT**: mandatory for new work.
- **SHOULD / SHOULD NOT**: strong defaults; deviations require a documented reason.
- **MAY**: optional implementation choice.
- This document governs code conventions, dependency management, container execution, validation, and contributions.
- `docs/technical-specification.md` governs algorithmic scope, hardware assumptions, coordinate frames, sensor processing, and evaluation requirements.
- Conflicting requirements MUST be resolved explicitly in a documented review; neither document is silently overridden.
- Deviations MUST be recorded with their rationale, affected targets, and path back to compliance.
- The project is initially a research/engineering framework and must not be described as safety-certified navigation software.

## 2. Fixed technical decisions

| Area | Contract |
| --- | --- |
| Language | **C++20**, required; do not rely on compiler-specific language extensions. |
| Build system | **CMake** with target-based configuration; minimum CMake version **3.25**. |
| Testing framework | **Catch2 v3**, acquired with **CMake FetchContent**. |
| CLI parsing | **cxxopts**, acquired with **CMake FetchContent**. |
| Math | Eigen (explicitly versioned and supplied by the Docker toolchain). |
| Local nonlinear optimization | Ceres Solver, isolated behind a frontend interface. |
| Graph optimization | GTSAM, isolated behind a backend interface. |
| Primary OS | Linux, **ARM64** (Raspberry Pi 5 or comparable) and **x86-64**. |
| Development builds | Docker containers. |
| Tests and validation | **Executed inside Docker containers**. |
| Application and replay runtime | **Executed inside Docker containers**. |
| Dataset / map artifacts | Versioned, reproducible, separately mounted from code and images. |
| ROS conventions | **REP-103 and REP-105** plus ROS-compatible map/scan semantics; adopted by the native core. |
| ROS 2 integration | Optional separately built adapter; **no ROS 2 headers, types or linked dependencies in the core**. |
| Debugging / visualization | **Web Lab**: framework-free HTML/CSS/JavaScript and Canvas; small local C++ HTTP/JSON adapter; no algorithms duplicated in JavaScript. |
| Boost libraries | **No implicit adoption**; every proposed Boost component and use case requires prior discussion, comparison against alternatives, and explicit approval. |
| Pico acquisition | **Optional future** bare-metal Raspberry Pi Pico firmware, custom Pico SDK; no micro-ROS, DDS or ROS types; Pi-only drivers are the required starting point. |
| GitHub Actions | No workflows required by default; **never run Actions** without explicit authorization. |

All new **native host** production code and test code MUST use C++20. A 3rd-party dependency MAY use an older language standard internally, provided it does not force a lower standard or extensions on project targets. **Future Pico firmware** is a separate embedded build using the pinned Pico SDK and its compatible C/C++ compiler; seek C++20 for project-owned firmware where supported, and document a specific embedded-toolchain exception rather than changing the native C++20 requirement.

## 3. Accepted OD-017 project targets and public API boundaries

The first host CMake implementation has **eight production libraries** (four installed SDK and four internal implementation) and **three native daemon executables**. Do not create empty CMake libraries for every conceptual module. **Web Lab is separately built as OD-016's fourth runtime process**, and Pico firmware is an independent optional cross-compile.

~~~text
2d-fusion-slam/
  CMakeLists.txt
  CMakePresets.json
  cmake/                              # FusionSLAM package configuration
  docker/
  include/slam/
    core/
      time.hpp geometry.hpp sensor_data.hpp trajectory.hpp
      interfaces/
    mapping/
      occupancy_grid.hpp submap.hpp map_snapshot.hpp
    engine/
      slam_engine.hpp configuration.hpp diagnostics.hpp
    storage/
      map_bundle.hpp recording.hpp
  src/
    core/             # slam_core (public)
    mapping/          # slam_mapping (public)
    engine/           # slam_engine (public facade, private internals)
    storage/          # slam_storage (public)
    frontend/         # slam_frontend (private)
    match_ceres/      # slam_match_ceres (private)
    graph_gtsam/      # slam_graph_gtsam (private)
    platform_linux/   # slam_platform_linux (private)
  apps/
    slam_acquisitiond/
    slam_recorderd/
    slam_localizerd/
    web_lab/          # independent application build, not host target
  firmware/pico/      # independent optional Pico SDK CMake cross-compile
  ros2/fusion_slam_ros/ # independent optional ROS adapter
  tests/
  configs/
  docs/
~~~

| Raw library | Public CMake imported name | Responsibility |
| --- | --- | --- |
| `slam_core` | **`slam::core`** | Time/SE(2), sensor/covariance/trajectory/portable interfaces; no ROS/Ceres/GTSAM/Linux/IPC |
| `slam_mapping` | **`slam::mapping`** | Occupancy/submaps/immutable snapshots, depends on core |
| `slam_engine` | **`slam::engine`** | Portable high-level engine facade and diagnostics |
| `slam_storage` | **`slam::storage`** | Map bundles/recording/replay portable APIs |
| `slam_frontend` | **Not installed/exported** | Motion/deskew/scan assembly/keyframes, native quality/search, submap orchestration |
| `slam_match_ceres` | **Not installed/exported** | Private Ceres matcher/OD-008 evaluation |
| `slam_graph_gtsam` | **Not installed/exported** | Private GTSAM backend/factors |
| `slam_platform_linux` | **Not installed/exported** | Linux acquisition/Pico-clock/OD-016 seqpacket IPC |

**Target dependency DAG:** `slam_core` has zero project libs. `slam_mapping` depends on `slam_core`, `slam_storage` on portable core/mapping if required, `slam_engine` exposes portable core/mapping/storage APIs and composes internal frontend/Ceres/GTSAM privately. `slam_frontend` uses only portable types and no Linux/device/solver dependencies; `slam_match_ceres` and `slam_graph_gtsam` depend on portable native interfaces and their own private solver libraries, **not on each other**. `slam_platform_linux` is an outer adapter for daemon executables and is NEVER pulled into the SDK or algorithm targets. Native public headers contain no `rclcpp`, `tf2`, `sensor_msgs`, `ceres::`, `gtsam::`, `sys/socket.h`, Pico SDK or private implementation data. A later ROS adapter only depends on public SDK. Domain library tests work with no sensor hardware, Web Lab, IPC daemon or ROS installation.

**Three thin native host executables:** `slam_acquisitiond` links platform/core; `slam_recorderd` links storage/platform; `slam_localizerd` links engine/mapping/platform with internal backend composition as needed. Each CLI uses cxxopts and implements `--help`/invalid-input exit codes, health/checkpoint and clean shutdown. Offline operations use approved SDK/CLI modes in the baseline target count; optional tooling is independently selected/approved. The source tree is **a target layout**, not a mandate to create empty placeholder directories or library stubs.

## 4. CMake and C++20 requirements

The top-level project MUST specify:

~~~cmake
cmake_minimum_required(VERSION 3.25)
project(fusion_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(FetchContent)
~~~

Production library targets SHOULD additionally expose the appropriate `cxx_std_20` compile feature via `target_compile_features`. Compile include paths, libraries, compile definitions, and warnings MUST be declared per target, not via global directory-wide flags.

- **Accepted OD-017** fixes target names `slam_core`, `slam_mapping`, `slam_engine`, `slam_storage`, `slam_frontend`, `slam_match_ceres`, `slam_graph_gtsam`, `slam_platform_linux`, with installed aliases `slam::core`, `slam::mapping`, `slam::engine`, `slam::storage` only.
- Use `PUBLIC`, `PRIVATE`, and `INTERFACE` visibility intentionally.
- Export only headers that form a supported public API.
- Use out-of-source build directories and Ninja where available.
- Provide CMake presets for debug/development, release, sanitizers, and test builds.
- Tests MUST be optional with `BUILD_TESTING=OFF` without pulling in Catch2.
- Normal library/application builds MUST NOT require a network connection after the dependency cache/toolchain has been provisioned.
- Applications MUST accept standard `--help` and nonzero exit codes for invalid arguments.
- Source files MUST be UTF-8; formatting rules SHOULD be enforced with a committed `.clang-format`.

### 4.1 Catch2 v3 via FetchContent

The project MUST use **Catch2 v3**, not Catch2 v2's legacy single-header interface. CMake MUST obtain Catch2 through `FetchContent_Declare` / `FetchContent_MakeAvailable` and link ordinary test binaries to `Catch2::Catch2WithMain` (or `Catch2::Catch2` when defining a custom main).

The following snippet is **illustrative**; `<PINNED_CATCH2_COMMIT>` must be replaced with a reviewed, complete upstream commit SHA before it is committed as a real configuration:

~~~cmake
if(BUILD_TESTING)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG <PINNED_CATCH2_COMMIT>
  )
  FetchContent_MakeAvailable(Catch2)
  add_executable(slam_core_tests tests/unit/core_test.cpp)
  target_link_libraries(slam_core_tests
    PRIVATE slam_core Catch2::Catch2WithMain)
  add_test(NAME slam_core_tests COMMAND slam_core_tests)
endif()
~~~

`catch_discover_tests` SHOULD be preferred once individual test registration is useful. When FetchContent is used, add the downloaded Catch2 `extras` directory to `CMAKE_MODULE_PATH` before including `Catch`; alternatively register the test executable with `add_test` until that is set up. CTest is the standard test entry point.

### 4.2 cxxopts via FetchContent

All application CLI targets MUST use **cxxopts**, fetched with CMake's `FetchContent` mechanism, and link `cxxopts::cxxopts`. The core library MUST NOT depend on cxxopts.

Illustrative configuration (replace the placeholder with a complete, reviewed commit SHA):

~~~cmake
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG <PINNED_CXXOPTS_COMMIT>
)
FetchContent_MakeAvailable(cxxopts)

target_link_libraries(slam_recorderd PRIVATE cxxopts::cxxopts)
~~~

Application arguments SHOULD include named input/output paths, configuration path, logging level, and a well-defined `--help`. Flags for recording, replay, localization, and online mapping SHOULD be consistent where their meanings overlap. No interactive prompts in batch/replay applications.

### 4.3 Other dependencies

- **Eigen, Ceres, and GTSAM** MUST be pinned to compatible known versions in the **Docker image/toolchain**, with the selected versions documented in the image build definition. Use target-based `find_package` / imported targets where practicable.
- It is **not** mandatory to fetch heavy Ceres/GTSAM dependency trees with FetchContent; provisioning them in a reproducible Docker image is the default.
- Any other newly introduced C++ dependency requires a stated purpose, license check, supported platform matrix, and pinned version.
- **Boost is exception-gated, not an implicit foundation:** before adding a direct Boost include, target, `find_package(Boost ...)`, source use, or new Boost-dependent convenience wrapper, the specific Boost component and its use case MUST be discussed and **explicitly approved case by case**. A decision for one component or module does not approve other Boost components or usages. Proposals MUST assess C++20 standard-library and existing-dependency alternatives, necessity, licensing, version, portability, image/compile-size impact, transitive linkage, and long-term maintenance.
- An otherwise approved dependency or ROS 2 adapter that brings in Boost **transitively** MUST have that dependency and its impact disclosed and evaluated; do not quietly introduce Boost through FetchContent, packages, Docker images, or adapter headers. Incidental upstream transitive Boost dependencies are not automatically banned, but they MUST be visible and reviewed before adoption. If Boost is not essential, prefer a Boost-free solution.
- For Web Lab networking, **Boost.Asio or Boost.Beast MUST NOT be added by default**. Their use requires the same explicit per-case evaluation. Do not add Boost speculatively for future networking needs.
- The optional Raspberry Pi Pico firmware MUST use a **version-pinned Raspberry Pi Pico SDK** and a separately provisioned cross-compiler; the SDK is not a dependency of host core libraries. **Do not include micro-ROS, ROS transports, ROS 2 message serialization, DDS, rclc, or rclcpp in Pico firmware or Pico host driver.** The Pi↔Pico transport MUST use a documented custom framed protocol, independently testable with a fake source. Pico SDK license, SDK transitive dependencies and any proposed Boost use still need dependency review.
- Prefer an independently reusable library over adding a new external dependency for trivial utility code.
- Third-party licenses and notices MUST be preserved.

### 4.4 Accepted OD-017 installed CMake SDK package

The host build MUST produce **exactly eight library targets and three native executables** before tests/independent integrations. Public/installable raw libraries are `slam_core`, `slam_mapping`, `slam_engine` and `slam_storage`, with **only four exported namespaced targets** `slam::core`, `slam::mapping`, `slam::engine` and `slam::storage`. Internal `slam_frontend`, `slam_match_ceres`, `slam_graph_gtsam` and `slam_platform_linux` MUST NOT appear as public exports. Install public headers only under `include/slam/{core,mapping,engine,storage}` and package files `FusionSLAMConfig.cmake`, `FusionSLAMConfigVersion.cmake`, `FusionSLAMTargets.cmake`. Their `EXPORT_NAME`, build aliases and imports must agree.

**CMake implementation policy:** CMake ≥3.25, native C++20 per-target `target_compile_features(... PUBLIC cxx_std_20)`, `target_include_directories` with `BUILD_INTERFACE`/`INSTALL_INTERFACE`, precise `PUBLIC`/`PRIVATE`/`INTERFACE` transitive links, `GNUInstallDirs` and `CMakePackageConfigHelpers` (`configure_package_config_file`, `write_basic_package_version_file`), `install(TARGETS ... EXPORT ...)` and `install(EXPORT ... NAMESPACE slam:: ...)`. Export only genuine public dependencies (`find_dependency` where necessary), not Catch2/cxxopts or platform/ROS/Ceres/GTSAM. Define a supported package version and explicit binary ABI/compatibility policy; compiling against a matching CMake version alone is not proof of C++ ABI compatibility.

**Private static-link pitfall:** An exported installed **static** `slam_engine` privately linking unexported internal static libraries can still place `$<LINK_ONLY:slam_frontend>` etc. in its imported link closure. Merely marking `PRIVATE` does not prevent this. Prefer a suitable **shared public façade with pImpl and private implementation/runtime shared dependencies** or another demonstrated packaging strategy that enables a clean consumer to configure/link/run without internal target names. Installation must deploy runtime solver dependencies as needed; a solver-free public header/package interface does **not** imply that the runtime shared library has no Ceres/GTSAM dependencies. Verify both build export and install export plus loader paths, relocation and transitive headers. Keep Linux platform/IPC code outside portable algorithm and SDK targets.

**Exact three CLIs:** `slam_acquisitiond` (platform/core), `slam_recorderd` (storage/platform), `slam_localizerd` (engine/mapping/platform). Web Lab builds independently as the fourth OD-016 process; firmware Pico SDK/C++ cross-compile separately. Tests, optional ROS2 adapter and tools are independent selectable switches. `BUILD_TESTING=OFF` must skip Catch2 FetchContent; `cxxopts` links only CLIs. Heavy solver/dependency versions pinned within Docker, no unapproved Boost, no global flags.

**Installed consumer acceptance:** After `cmake --install` to a **clean relocation prefix**, a separate source tree MUST configure `find_package(FusionSLAM CONFIG REQUIRED)`, include representative four public headers, link all four `slam::*` targets and execute a smoke binary. In that consumer **do not provide** internal/private headers, ROS, sensor SDKs, Linux socket headers or Ceres/GTSAM CMake packages for configure/header compilation. Verify imported target resolution, `BUILD_TESTING=OFF`, test-enabled Catch2/CTest, C++20 features, runtime private solver deployment, no dependency cycles or `LINK_ONLY` leaks, ARM64/x86-64 portability and host target count. Benchmark **B25**; no GitHub Actions dispatch.

## 5. Dependency and reproducibility policy

1. `FetchContent` dependencies MUST use immutable complete Git commit SHAs or an equivalently content-verified archive, not `main`, `master`, `latest`, or mutable floating tags.
2. Docker base images MUST be pinned to reviewed versions and SHOULD be locked by digest for release-quality reproducibility.
3. Apt/system packages in images MUST be installed at image-build time; document package/repository revisions or package snapshots where strict rebuild reproducibility is required.
4. Record CMake, compiler, libc, Eigen, Ceres, GTSAM, Catch2, and cxxopts versions in build metadata.
5. Reuse Docker build layers, BuildKit cache mounts, and a controlled FetchContent cache to avoid repeated downloads.
6. Production runtime images MUST contain neither a compiler nor test-only dependencies unless explicitly justified.
7. Never fetch dependencies at application startup.
8. Dependency upgrades MUST include successful containerized builds and tests on the applicable target architectures.

Avoid contradictory build behavior in which a container provides one version of a library and FetchContent silently selects another.

## 6. Docker-first development, testing, and runtime

Docker is the **supported execution boundary** for builds, test runs, offline processing, and live applications. Developers do not need Ceres/GTSAM installed on the host.

### 6.1 Required container targets / services

The initial container implementation MUST provide:

| Target or service | Role |
| --- | --- |
| `dev` | C++20 toolchain, Ninja, CMake, debugging/formatting tools, development dependencies, editable source mount. |
| `test` | Build and execute Catch2/CTest tests and regression checks; fail on test failures. |
| `runtime` | Minimal non-root image containing installed runnable applications and runtime libraries only. |
| `recorder` | Run sensor acquisition/recording on robot with explicit hardware access. |
| `offline-mapper` | Process mounted datasets and write versioned maps. |
| `localizer` | Load a read-only map and run live scan matching/fusion. |
| `online-slam` | Full mapping mode, created when the corresponding application exists. |
| `web-lab` | Opt-in local visualization and replay/diagnostic API served by a C++ adapter, with explicitly mapped loopback-only host port. |
| `ros2-adapter` / `ros2-test` | Opt-in ROS 2 integration and tests with a dedicated Docker image; the default core images MUST stay ROS-free. |
| `pico-firmware-build` / `pico-protocol-test` | **Future opt-in** Dockerized Pico SDK cross-build, protocol and time-sync host simulation; firmware executes bare-metal **on the Pico**, not inside Docker. |

A multi-stage `docker/Dockerfile` with `dev` / `test` / `runtime` stages, and a `docker/compose.yaml` or root-level `compose.yaml`, SHOULD be the standard. Services may share an image when it avoids redundant dependencies. A top-level Makefile or helper script MAY wrap Docker commands, but MUST NOT bypass Docker.

### 6.2 Required standard commands

The scaffolding milestone MUST make commands equivalent to these work (names and file paths may be refined once the compose file exists):

~~~sh
# Development shell, from repository root
docker compose -f docker/compose.yaml run --rm dev

# Complete unit and integration test suite
docker compose -f docker/compose.yaml run --rm test

# Offline dataset replay and mapping
docker compose -f docker/compose.yaml run --rm \
  offline-mapper --input /data/recordings/session.mcap --output /data/maps/session

# Live localization (requires mapped devices and an existing map)
docker compose -f docker/compose.yaml run --rm \
  localizer --map /maps/site-a --config /config/robot.yaml
~~~

**These commands are contractual examples, not executable instructions in the repository today.** When implemented, the README MUST state the actual commands, required volumes, device paths, and output locations.

### 6.3 Host mounts and runtime safety

- Dataset and map inputs SHOULD be mounted read-only; recording and offline output destinations MUST be separate explicitly writable volumes.
- Containers MUST run as a non-root UID by default, with documented file ownership for generated artifacts.
- Live sensor access MUST map only necessary device nodes (e.g. LD19 UART serial and MCU/IMU interfaces) and required Unix groups; avoid `--privileged` and broad host filesystem mounts.
- Hardware acquisition may need host udev rules, I2C enablement, device access permissions, and platform-specific adapters. These MUST be documented.
- Container processes use the host kernel and clocks; **Docker does not automatically synchronize clocks of independent microcontrollers or the LD19**.
- The live localizer MUST run without internet access after its image and maps have been provisioned.
- Use explicit stop/shutdown behavior so recordings and map writes are flushed safely.
- Document optional CPU affinity / scheduler priority only when benchmarked and truly necessary.
- Resource use on Raspberry Pi 5 MUST be measured under sustained operation, including thermal throttling and device I/O.

### 6.4 ARM64 and x86-64

- Both architectures MUST build from the same source tree and CMake project.
- A container test on x86-64 MUST NOT be presented as proof of Pi 5 ARM64 performance.
- CI-like local validation SHOULD build native target-architecture images on available hardware; cross-builds can check compilation but not prove device compatibility.
- Release images MUST identify their target architecture and dependency/toolchain versions.
- If the DeepRacer lacks true encoder observations, its sensor adapter MUST make this limitation explicit rather than fabricating odometry.

### 6.5 Pico acquisition firmware and host protocol (optional)

**OD-002 is accepted:** implement Pi-host timestamping for LD19, MPU6050 and encoders **first**. The core MUST remain fully functional without a Pico. A later Pico source is a swappable timestamped-measurement acquisition adapter, not a replacement SLAM implementation.

- Firmware MUST be **custom Pico SDK firmware**, with no micro-ROS, DDS, ROS 2 message types or ROS middleware. The Pi native driver similarly MUST NOT require micro-ROS.
- Use a simple framed binary protocol, versioned and length-bounded, containing at least sequence number, source timestamp and units, Pico boot/session ID, event type, sensor values, status/validity and integrity check. **USB CDC is the initial transport candidate, not an already approved physical interface.** UART is an alternative; reject malformed, missing, duplicate and out-of-order frames.
- Use the RP2040's boot-relative microsecond hardware timer for source timestamping and convert into canonical 64-bit nanoseconds with overflow checks on the host. This **does not** imply sub-microsecond accuracy.
- Implement Pi↔Pico two-way time-sync messaging with host send/receive and Pico receive/transmit timestamps; estimate offset, drift, round-trip delay/asymmetry and uncertainty, and invalidate the mapping across Pico resets or stale sync. The exact estimator/cadence is subject to real-device calibration.
- **PIO is optional and evidence-driven.** Use it for edge counting or deterministic capture only after demonstrating a clock-anchored timestamping scheme and a benefit over IRQ/timer capture. PIO does not automatically read the hardware timer.
- Cross-build firmware **inside a Docker toolchain**, and test the protocol, sync math, restart/reconnect and mock acquisition inside Docker. The firmware itself necessarily executes on the **physical microcontroller** (bare metal); that is the one runtime exception to the Docker-on-Linux rule. Opt-in hardware-in-loop tests use Docker on the Pi/host to communicate with actual firmware.
- Do not make the native host image, ordinary Catch2 suite or offline replay depend on Pico SDK, connected hardware, Pico firmware flashing, or micro-ROS. Firmware flashing and MCU operation require explicit user action and are not performed by GitHub Actions.

See [Architecture `3.5](architecture-specification.md) and [Accepted OD-002](open-design-choices.md) for clock-domain and test semantics.

### 6.6 LD19 timing adapter contract (accepted OD-003)

The **native LD19 acquisition adapter** MUST own all protocol-specific beam-time reconstruction and clock alignment, while the C++20 core consumes only a **generic timestamped scan**. Do not add LD19 packet fields, UART drivers, vendor SDK dependencies, ROS types, or serial clock-fitting logic to public estimator APIs.

- Decode CRC-validated 47-byte packets, start/end angles (0.01°), reported degrees/s speed, 12 sample slots and the 30-second millisecond timestamp counter.
- Derive **relative per-beam acquisition times from unwrapped angles and reported speed**, validating rotation direction, speed, angular coverage and missing packets. Never silently assign one receive time to every beam.
- Unwrap device timestamps with explicit source epoch management, including ambiguous wraps, dropped frames and sensor resets. Convert to OD-001 canonical signed nanoseconds with checked arithmetic.
- Timestamp the actual observed **Pi UART receive event** and fit **device-to-host monotonic offset and drift** using many valid packet observations. Record model parameters, version, validity, packet jitter and uncertainty.
- **One-way UART arrival timestamps cannot identify clock epoch offset separately from constant physical transmission/processing delay.** Record a combined offset until an independently measured or bounded **packet reference/acquisition delay** is available. Do not mark such timings exact before calibration.
- Carry timestamp status, per-beam or shared systematic uncertainty, packet clock drift/jitter, raw device tick and receive time, map/calibration/clock revision provenance and all original packet bytes for offline reprocessing.
- Keep these algorithms **ROS-independent**; the optional ROS 2 adapter only converts already aligned native per-beam timestamps to standard messages when the format can represent them faithfully.
- Dockerized Catch2/CTest tests MUST cover angle/speed timing, 0/360 and 30-second wraps, CRC, drop/reordering/reset, bad speed, injected offset/drift/jitter, ambiguous one-way delay, repeatability, and offline re-timestamping. Hardware experiments must establish or bound actual packet-to-beam acquisition delay.
- Any later Pico-based UART capture remains optional under OD-002; it is not a reason to postpone Pi-only recording or to introduce micro-ROS.

References: [accepted OD-003](open-design-choices.md), [architecture §4.1](architecture-specification.md), and the [LD19 development manual](https://botland.de/img/art/inne/21991_Instrukcja%20rozbudowy.pdf).

### 6.7 Accepted OD-005 — Scan assembly and invalid returns

The implementation MUST separate **LD19-specific acquisition/packet handling** (device decoder + accepted OD-003 timing) from a **portable, ROS-free ScanAssembler** that groups generic timestamped beam slots into fixed-angular-seam revolutions and emits `AssembledScan` with explicit coverage and validity.

- Segment revolutions at a configurable fixed angular seam using unwrapped beam angles and verified rotation direction, **not** packet boundaries, elapsed time, or expected points per revolution. Partition a packet straddling the seam without losing or duplicating observations.
- Support **partial scans** with actual time bounds, angular intervals, **360° circular coverage**, uncovered-sector intervals, maximum angular gap, quality counters, seam confidence, packet-loss markers, intensity, raw source/provenance and completeness/termination reason.
- An observed beam may have a **valid acquisition timestamp but invalid range**. It contributes to OD-004 scan start/end/midpoint timing, but never to geometric matching or clearing. Missing sectors have **no synthetic beams or timestamps**.
- Use typed `VALID_HIT`, `VALID_NO_RETURN`, `INVALID_RETURN` and `MISSING` states. **INVALID_RETURN and MISSING MUST NOT generate free-space updates or occupied endpoints.** A documented valid no-return MAY produce only a bounded free ray under explicitly enabled calibration/config. An undocumented zero or out-of-range return MUST NOT be silently reclassified as clear space.
- Use **separate configurable and versioned matching/insertion gates** for actual angular support, largest uncovered sector, count/distribution of valid hits, sensor/time reliability and pose/interpolation status. A well-supported partial scan may be accepted; report failures with structured reasons.
- Normal map insertion requires both the OD-005 insertion gate and **OD-004 complete interpolation over the observed scan interval**; scans requiring any extrapolated motion are not inserted.
- **Use timeouts only for stalled/disrupted acquisition recovery**: an explicit timeout/abort reason on the partial record is mandatory. Ordinary scan segmentation MUST remain seam-based, not periodic wall-time-driven.
- Implement the scan assembler against native timed-beam and gap-event inputs and unit-test it with **mock beams**, without an LD19 device, Ceres/GTSAM, Pico SDK or ROS. Web Lab diagnostics SHOULD display observed/missing angular wedges, partial flags, invalid returns and eligibility outcomes. ROS adapters must not fabricate uniform `LaserScan` geometry for arbitrary partial sweeps.

**Docker Catch2/CTest acceptance tests** MUST cover seam-crossing packets, nonzero seam, either rotation direction, repeated revolutions and first/last partial scan, dropped packets/ambiguous multiple turns, known angular coverage vs min/max span, zero valid ranges, invalid-return/no-hit occupancy behavior, confidence/threshold gates, watchdog-only recovery, and OD-004 full interpolated midpoint coverage. These tests should use the deterministic apartment/warehouse fixtures, not external dataset downloads.

See [accepted OD-005](open-design-choices.md) and [architecture `4.1.1](architecture-specification.md). This is a specification, not a claim that the scan assembler already exists.

## 7. Testing contract

Tests MUST use **Catch2** and be registered with **CTest**. All test execution, including sanitizer runs and offline replay regression suites, MUST happen within Docker.

### 7.1 Test categories

| Category | Must cover | Dependencies |
| --- | --- | --- |
| Unit | SE(2)/bias/kinematics and LD19 timing/OD-005 validity, **OD-010 log odds, 90/180 submap IDs, hit priority, known/unknown, motion filter, memory caps and freeze immutability** | Synthetic/no hardware |
| Integration | OD-007/008/009 matching/search, OD-011 snapshots, OD-012 persistence, OD-013 private graph, OD-014 replay and **OD-016 four-process versioned Unix IPC/fault isolation** | Docker B21/B22/B24 with crash/queue tests, Pi profile; domain APIs remain ROS-/GTSAM-/Ceres-/IPC-free |
| Optional ROS 2 integration | ROS ↔ native message conversions, TF authority and frame conventions, map-file interoperability, ROS time and QoS, Nav2/RViz smoke tests | Dedicated ROS 2 Docker test image |
| Regression | Deterministic offline replay, stable outputs, previously fixed bugs | Versioned datasets |
| Performance | Per-scan latency, CPU, memory, cache allocations, Pi thermal behavior | Measured on target hardware |
| Hardware-in-loop | UART acquisition, timing offsets, loss/reconnection, wheel/IMU integration | Opt-in device-access container |
| Optional Pico firmware/protocol | Pico cross-build, framed-protocol CRC/versioning, boot-ID changes, clock-sync offset/drift uncertainty, PIO vs IRQ feasibility | Docker cross-build and mock-host tests; opt-in physical Pico test |

Tests MUST be deterministic unless they are explicitly marked stochastic, and random seeds MUST be configurable and logged. Hardware-in-loop tests MUST NOT block the normal hardware-free test suite.

### 7.2 Required test behaviors

- Correct behavior at zero/negative/overflow edge cases and wraparound boundaries.
- Input validation, no unchecked indexing, no division-by-zero on short sensor intervals.
- Frame and angle convention checks; test °/rad and sensor rotation sign.
- Analytical Jacobians checked against numerical/finite-difference derivatives.
- Graceful handling of corrupt LD19 packets, missing IMU/encoder samples, stale maps, and ill-conditioned scan matching.
- A failed test MUST cause the container command to exit nonzero.
- Benchmarks MUST be separated from functional unit tests and should not make normal tests unstable.
- Critical regressions SHOULD retain minimized datasets or small synthetic fixtures.

### 7.3 Quality gates

A change is not complete until the relevant gates pass **inside containers**:

1. CMake configure and build with warnings enabled; no new warnings in modified targets.
2. Catch2 / CTest unit and integration tests.
3. `clang-format` check for changed C++ files.
4. Dependency review: flag any added direct or transitive Boost usage; verify the per-case discussion and explicit approval before acceptance.
5. Static analysis (`clang-tidy` when configured) and AddressSanitizer/UndefinedBehaviorSanitizer for supported test configurations.
6. Deterministic replay/integration checks when the change touches timestamps, sensor models, optimization, or serialization.
7. ARM64 smoke build/test for release-affecting changes, with a clearly stated exception if hardware is unavailable.
8. Updated docs, configuration examples, and test expectations for externally visible behavior changes.

Tests that cannot run in the current environment MUST be identified explicitly as **not run**, including the reason. They MUST NOT be reported as passing.

## 8. Runtime and operational behavior

- Applications MUST have machine-readable configuration (YAML or JSON, selected during scaffolding) with documented defaults and validation.
- CLI flags parsed by cxxopts MAY override configuration files with an explicit precedence rule.
- Application startup MUST log the mode, build/version, map/calibration identity, key solver settings, and input data sources.
- Every published pose MUST carry a timestamp, frame ID, quality/state indicator, and covariance or documented uncertainty representation.
- Logging MUST be structured enough to diagnose packet loss, time-sync faults, scan-matching degeneracy, loop closure, and lost localization.
- Runtime tasks MUST have bounded queues and controlled backpressure; acquisition threads MUST NOT block behind global graph optimization.
- A lost/stale localization state MUST NOT be published as a valid current pose without an explicit status.
- Recorders MUST flush/close datasets gracefully after shutdown signals.
- Run-time configuration and calibration MUST be tracked with map/dataset artifacts.

## 9. Data, replay, and maps

- Recording MUST retain raw LD19 frames, IMU samples, available encoder measurements, original acquisition/receive times, and synchronization metadata.
- **MCAP** is the preferred recording format, directly or through ROS 2 adapters.
- Offline mapping MUST run against mounted datasets without physical sensors.
- Map bundles MUST use accepted OD-012 **JSON manifest + platform-independent compressed binary occupancy/graph**, integrity/size indexes and atomic generations; preserve OD-010 known masks, calibrated frames/clock sessions and map-vs-resume kinds.
- Runtime localization MUST treat input map bundles as read-only.
- Replaying a known dataset with the same configuration and version MUST produce deterministic results within documented floating-point tolerances.
- Data conversion and schema changes MUST be tested for backward compatibility or explicitly use versioned migrations.
- Large datasets and generated binaries MUST NOT be committed directly to Git without a separately approved storage strategy.

## 10. C++ style and engineering conventions

- Use RAII, deterministic ownership, typed units or explicit unit suffixes, const-correctness, and clear error propagation.
- Prefer value types, `std::span` / `std::optional` where appropriate, and explicit lifetime boundaries for sensor buffers.
- Avoid unbounded allocation in the high-rate acquisition and deskew paths when a bounded buffer suffices.
- Use `std::chrono` for times/durations; document how integer monotonic timestamps map to host/sensor clocks.
- No exceptions may cross C ABI, hardware interrupt, or process boundaries; application-level error handling MUST translate failures into actionable diagnostics.
- Threading and locking rules must be documented for state estimators, data queues, map access, and background optimization.
- Solvers MUST have explicit iteration/time budgets in online modes; offline mode MAY use more expensive convergence criteria.
- Mathematical assumptions and units MUST be documented near nontrivial algorithms; references belong in source comments or docs.
- Functions that perform I/O or alter global state MUST be separated from deterministic math where practical.

### 10.1 Accepted OD-004: portable SE(2) trajectory and deskew boundary

The native C++20 core MUST expose a **timestamp-addressable trajectory-provider abstraction** returning `T_odom_base(t)` with covariance, trajectory revision, interpolation coverage, and explicit status. The first implementation MUST use **piecewise-constant body-frame twist integrated with `Exp_SE2`** from timestamped wheel and bias-corrected gyro measurements. Implement a numerically stable small-angular-rate branch; do not replace the Lie-group exponential with independent linear x/y/yaw interpolation. A future SE(2) spline MAY implement the same interface only when justified by benchmarks.

- The **motion estimator** owns synchronized sensor fusion, body-twist segments and trajectory history. **Deskew, Ceres, maps, replay, Web Lab and ROS adapters** depend on a generic trajectory/query contract, **never** on MPU6050, wheel encoder, Pico, LD19 packet structures or ROS types.
- The **LD19 adapter** (accepted OD-003) owns per-beam reconstructed timestamps; the **deskewer** (accepted OD-004) queries poses at these timestamps and transforms every geometric beam into the laser frame at the **temporal midpoint of the scan's acquisition window**.
- A complete scan's midpoint is `t_start + floor((t_end - t_start)/2)`, with checked signed-nanosecond arithmetic and an explicit clock domain and epoch. Use timestamp-valid acquired beam slots to determine bounds, even if their range values are rejected as geometry; do not use packet arrival midpoint or scan angle midpoint.
- **Normal mapping requires full quality-valid interpolation over the complete scan interval.** A normal map-insertion call MUST reject/hold scans with missing motion, ambiguous acquisition boundaries, unresolved clock epoch changes or any extrapolated beam. Prefer bounded delayed scan processing until wheel/gyro data covers the scan end.
- **Extrapolation is opt-in and disabled for ordinary map insertion**. Its initial configurable maximum is **10,000,000 ns (10 ms)** outside the valid trajectory boundary; reduce/reject based on propagated uncertainty, wheel/IMU freshness, bias/slip and timestamp calibration. Return distinct `INTERPOLATED`, `EXTRAPOLATED` and `UNAVAILABLE` status, actual horizon and covariance. Never treat a 10 ms cap as a guaranteed prediction quality threshold.
- Invalid sensor intervals, trajectory clock mismatch, stale calibration and backwards `/clock` transitions MUST invalidate trajectory queries; deterministic replay seek resets state or uses a compatible checkpoint.
- A raw ROS 2 `sensor_msgs/LaserScan` still stamps its **first beam** according to message semantics; the midpoint is a **derived deskew reference**. The optional ROS adapter MUST NOT falsely publish a midpoint-stamped deskewed cloud as if it were a raw uniform-angle `LaserScan`. Use a properly stamped diagnostic point cloud or explicit deskewed-scan schema when appropriate.
- External data recording and test fixtures MUST retain beam acquisition times, scan midpoint, coverage/quality indicators and the reference trajectory/calibration revisions; never rewrite raw source timestamps during map processing.

**Containerized verification:** Catch2 tests MUST cover pure translation, pure yaw, exact constant-curvature motion, near-zero omega, asynchronous gyro/encoder events, t_mid from nonuniform beams, full-scan coverage, off-center LiDAR, clock resets and invalid gaps. Test configured bounds at **9.9 ms, 10 ms and 10.1 ms**, including covariances and eligibility for map insertion. A mock trajectory provider MUST allow standalone deskew tests without device drivers or ROS installed; run equivalent replay/latency measurements in Docker on Pi where available. A spline implementation remains deferred.

See [Architecture Sections 4.2–4.3](architecture-specification.md) and [accepted OD-004](open-design-choices.md). This is a specification, **not a claim that the trajectory API already exists**.

### 10.2 Accepted OD-006 — Minimal wheel/gyro estimator implementation

The **first native C++20 motion estimator** MUST integrate **measured wheel translation and bias-corrected gyro Z yaw rate** using accepted OD-004 piecewise-constant body twist/SE(2) exponential. It MUST NOT require an EKF, fixed-lag smoother or any ROS types to produce local odometry.

- **State and interface:** Provide SE(2) odometry pose, body twist (with units and source validity), gyro bias with uncertainty, pose/bias covariance (including cross-correlation or a tested conservative equivalent), time domain/epoch, calibration revision and tracking/quality diagnostics. Implement the **sensor-neutral MotionEstimator/TrajectoryProvider** APIs; no MPU6050, encoder, Pico or LD19 types in core headers.
- **Kinematic models:** Select a replaceable, explicitly configured native wheel/steer model. Differential drive is NOT the default until actual left/right drive geometry and encoder readings are verified; Ackermann requires **measured** steering and wheel telemetry. A model MUST reject incompatible/missing sensor inputs rather than silently assume a wheel arrangement or use steering commands as measurements.
- **Bias initialization:** A stationary sample interval, validated through available wheel motion evidence and gyro variability (optionally accelerometer), is required to calibrate gyro Z bias and its variance. Store quality, calibration version, sample count, duration and optional temperature. Startup motion or insufficient stationary evidence MUST produce an **uncalibrated/degraded** status, not a trusted zero bias.
- **Online bias:** Permit only bounded, logged, covariance-aware corrections within verified stationary windows or independently justified observable updates. Disagreement between wheel yaw and gyro yaw is **not itself** gyro-bias ground truth. Never silently modify earlier published local odom; offline reruns create new trajectory revisions.
- **Slip and uncertainty:** When independently measured wheel yaw is available, compare it with gyro integrated yaw over matching timestamps with uncertainty/innovation gates. Suspicious disagreement inflates covariance and lowers quality. If not available, the check reports `UNAVAILABLE`. Propagate wheel noise/scale, gyro noise/bias random walk, acquisition uncertainty and lost samples; no fabricated zero motion on missing inputs.
- **Graph independence:** Do not add fused wheel/gyro predicted odometry as a second independent GTSAM factor beside wheel and gyro factors derived from the same data. Record the measurement provenance.
- **Future variants:** The EKF, error-state model and recent-state smoother remain optional later implementations of the **same native trajectory interface** and must not require rewriting deskew, mapping, ROS adapters, recorder or Web Lab.

**Docker/Catch2 acceptance tests:** Known-bias SE(2) curves and asynchronous samples; differential/Ackermann models with valid hardware-style measurements; unsupported drivetrain and missing steering; stationary calibration vs moving startup; accepted/rejected bounded bias corrections; injected slip, wheel scale errors, missing independent wheel yaw and covariance growth; Monte Carlo calibration of pose/bias uncertainty; clock gap/epoch reset; no OD-004 map insertion with extrapolated/incomplete trajectory; mock alternative trajectory implementation and Pi 5 CPU/latency measurements.

**Implementation checkpoint:** Add these requirements to C2 motion/deskew work; hardware drive model, bias thresholds and process noise are **not fixed** by this contract. See [OD-006](open-design-choices.md) and [architecture `4.2.2](architecture-specification.md).

### 10.3 Accepted OD-007 — EDT Ceres frontend, objective and isolation

The **first scan-matching implementation** MUST optimize a planar **OD-004 midpoint-referenced SE(2) pose** using a **truncated unsigned Euclidean distance field** computed from confidently occupied authoritative occupancy-grid cells. Ceres MUST be confined to the private frontend implementation and NOT appear in native core headers, public `ScanMatcher` / `MatchResult` interfaces, mapping/serialization types or ROS adapters.

- **Derived map cache:** The occupancy grid is authoritative and keeps unknown/free/occupied semantics, grid origin including yaw, resolution and revision. A derived **immutable versioned EDT** stores nearest-confident-occupied distance in metres, truncated to configurable `D_max`, with source grid checksum/revision, occupancy confidence threshold, known/support mask, interpolation config and cache version. Unknown must **not** be treated as occupied or confidently free, and changing an occupancy grid invalidates its derived EDT revision. **Accepted OD-010** determines log-odds occupancy and immutable 90/180 submap lifecycle; **OD-011** still selects derived-cache update and lifetime policies.
- **Inputs:** Only OD-005 eligible **VALID_HIT** observations, deskewed via OD-004 into the **scan midpoint laser frame**, with calibrated `T_base_laser`; OD-006 midpoint predicted robot pose `T_pred` and SPD covariance `Sigma_pred` in the same map/submap frame and documented tangent coordinates; matching map/EDT revision. No invalid, missing or undocumented no-return rays generate artificial obstacle constraints.
- **Per-beam residual:** `r_i(T) = sqrt(w_i) / sigma_i * D(T_map_grid_or_submap^-1 * T * T_base_laser * p_i)`, with `w_i >= 0`, `sigma_i > 0` in metres. Ceres MUST receive the **unsquared scalar residual**; Huber applies to its squared norm, not to an already squared distance.
- **Independent Huber losses:** For each retained beam allocate **one Ceres residual block returning exactly one scalar** with its own `ceres::HuberLoss(delta)`. Do not put all beams into one residual block under one Huber (which would robustify the **combined** squared norm). Huber `delta` has the units of the normalized residual; keep this explicit and record the configured value.
- **Covariance-weighted prior:** Use an explicit, separate normally unrobustified **three-dimensional** residual whitening `Log(T_pred^-1 * T)` by `L` satisfying `L^T L = Sigma_pred^-1` (with a documented SE(2) tangent/frame convention and checked finite positive-definite covariance). **The odometry prior is part of the accepted frontend objective, not only an initializer.** If prediction/covariance is invalid or unavailable, return explicit failed/degraded status or a documented no-prior configuration; never invent certainty or an identity covariance.
- **Boundary penalty:** Out-of-map endpoints receive a separately scaled, finite, outward-increasing distance/hinge penalty in **metric map-local coordinates**, with a useful inward derivative. Merely clamping to the nearest EDT border or assigning a constant outside value is prohibited. Unknown **inside** the grid is tracked separately and MUST NOT count as valid overlap or an occupied match. Define masked interpolation stencils/skip rules explicitly.
- **Interpolation:** Start with **bilinear**, which is bounded between sampled values and continuous within/at valid grid boundaries but has derivative changes at cell seams. Derivative/rotated-origin tests required. Do not add bicubic until benchmarks show a benefit without unacceptable overshoot/cost; the future interface supports alternate occupancy-probability, signed-distance and feature-based objectives.
- **Quality gates:** Require enough **supported-map overlap**, informative/nontruncated and spatially distributed hit beams, acceptable **LiDAR-only geometric residual quality**, finite/consistent posterior pose, positive/observable LiDAR-only geometry information, and bounded odometry innovation. Compute LiDAR observability **without the prior or artificial boundary Hessian** so a strong prior cannot make a featureless corridor appear well constrained. Reject boundary-dominated, unknown-dominated, failed/degenerate or poor-quality solutions with typed reasons; OD-008 configures numerical thresholds and conservative covariance.
- **Factor provenance:** Return the **prior-regularized posterior** and **separately reported LiDAR-only geometric evidence**, OD-006 prior pose/covariance/source IDs, prior/boundary costs and map/EDT revision IDs. If GTSAM uses wheel and gyro factors from the same measurements, the posterior pose **MUST NOT** be added as an **independent LiDAR-only factor**. The statistically justified graph factor form remains OD-008/013; explicitly audit correlation and do not subtract Hessians and claim exact independence.
- **Replaceability:** Expose native `ScanMatcher` / `MatchResult` contracts with no Ceres classes in public APIs. Alternate objectives or solvers MUST plug into those contracts. Ceres/Ceres dependencies remain private to the implementation target; no ROS, sensor drivers or new Boost usage without separately discussed approval.

**Ceres objective convention** (illustrative; frontend code not implemented):

~~~text
r_i = sqrt(w_i) / sigma_i * D(T * T_base_laser * p_i)
e_prior = Log(T_pred^-1 * T)
L^T L = Sigma_pred^-1

E_Ceres(T) = 1/2 sum_i Huber_delta( r_i^2 )
           + 1/2 || L * e_prior ||^2
           + E_boundary(T)
~~~

The matching-frame transform into EDT coordinates is required but omitted in this shorthand. `E_boundary` is the sum of independent squared boundary residuals in the chosen units. Do not silently multiply the Huber cost by additional beam weight after the weight has been absorbed in `r_i`.

**Docker Catch2 + Ceres integration tests** MUST exercise: known occupied-geometry Euclidean distances and metre truncation; rotated world-to-grid transforms; confident occupancy threshold and separate unknown support mask; bilinear interpolation at cell corners/interiors and derivative transitions; **one block per beam** Huber behavior and `sqrt(w_i)/sigma_i`; valid/invalid SPD prediction and SE(2) prior whitening; out-of-grid penalty finite inward gradients; unknown-vs-outside and zero-occupied-map cases; no OD-005 invalid-return beams; enough overlap, LiDAR-only Hessian degeneracy despite strong odom prior, repetitive-warehouse false matches; separated posterior/prior/LiDAR diagnostics, and **no double-counted downstream GTSAM factors**. Performance and error benchmarks must run on the synthetic fixtures and optionally real bags, with Pi 5 p50/p95/p99 timing. No GitHub Actions required or run by default.

See [accepted OD-007](open-design-choices.md) and [architecture Section 5](architecture-specification.md). The numerical `D_max`, beam σ/weights, confidence threshold, Huber δ, boundary scale, overlap/conditioning thresholds and solve limits require tuning under OD-008/010/011; none is a claimed validated default.

### 10.4 Accepted OD-008 — LiDAR-only observability and uncertainty quality boundary

**Numerical optimization and quality approval MUST be separate.** The private OD-007 Ceres adapter supplies the regularized optimized midpoint SE(2) pose and **unregularized LiDAR-only per-hit normalized residuals/Jacobians**, plus OD-006 prior, artificial boundary and solver diagnostics. A native **`MatchQualityEvaluator`** (or equivalent) performs robust PSD information analysis, physical-axis normalization, information rank, covariance calibration, and acceptance/projection/rejection. Its target and public interfaces MUST remain **independent of Ceres, ROS and hardware drivers**.

- **Robust LiDAR-only information:** For normalized OD-007 beam residual `r_i=sqrt(w_i^beam)/sigma_i*D(q_i(T))` and tangent-space 1×3 Jacobian `J_i`, form `H_L=sum_i rho_delta'(r_i²)*J_i^T*J_i`. This is a PSD **robust-influence Gauss–Newton approximation**, deliberately excluding `rho''` terms. The beam confidence `w_i^beam` is **already** inside the Jacobian: no second multiplication by it. Select only valid supported LiDAR geometry; prior and boundary residual blocks MUST be excluded. Document the Ceres Jacobian/manifold and robust-loss evaluation settings and validate with analytic fixtures.
- **Observability:** Convert mixed translation/metres and yaw/radians tangent coordinates with a **recorded physical scaling `delta_xi=S*z`**; classify rank and observable/null directions from eigenvalues of `S^T H_L S` with calibrated absolute/relative information gates. Reject nonfinite or materially non-PSD results. An OD-007 **strong odometry prior cannot raise LiDAR-only rank**.
- **Full rank:** Only when rank=3 with valid noise normalizations and credible conditioning, estimate `Sigma_raw≈H_L^-1` and calibrated `Sigma_match=K*Sigma_raw*K^T+Sigma_floor`. `K` and PSD `Sigma_floor` must have revisioned, held-out calibration evidence; verify symmetry, finite PSD/SPD status, tangent frame, source map/EDT calibration and cross-correlated scan/gyro/model errors. If calibration is unavailable, mark it **uncalibrated** rather than claim accurate uncertainty.
- **Rank deficient:** Return **rank 0–2, observable bases and null-space bases** with an `OBSERVABLE_SUBSPACE` or `UNAVAILABLE` uncertainty type; unknown/null variance is **not zero**. Never export `pinv(H_L)` as a finite confident 3×3 covariance or add floors to fabricate missing information. Either **project** the OD-006 prediction-to-fit correction into the *normalized supported subspace* and re-gate it, or **reject** and use degraded odometry-only localization. Do not send an ordinary full-rank GTSAM factor for this observation; **accepted OD-013** initially rejects unsupported rank-deficient full-pose factors until a tested observable-subspace factor is added.
- **Independent decision:** Gate on solver convergence and finite inputs, OD-005 scan support/hit distribution, observed-map overlap, informative/nontruncated beam count, LiDAR-only raw/Huber residual quality/outliers, normalized eigen-rank, OD-006 innovation consistency (with conservative correlated-odom/deskew uncertainty), OD-003 time uncertainty, wheel/gyro bias/slip/gaps, correct OD-004 midpoint coverage and immutable occupancy/EDT revision.
- **API:** Native `MatchResult` must expose rank (0–3), physical normalization, observable/null directions, robust LiDAR-only information, `FULL_COVARIANCE` / `OBSERVABLE_SUBSPACE` / `UNAVAILABLE`, calibrated covariance only when rank=3, `ACCEPT_FULL` / `ACCEPT_PROJECTED` / `REJECT` / `UNAVAILABLE` outcome and typed reasons, source/prior/cache calibration IDs and separate geometric/prior/boundary quality metrics. Do not expose Ceres types. ROS conversion must not publish false finite confidence in null directions.
- **Calibration:** Fit `K`, `Sigma_floor`, rank/overlap/residual/innovation gates **only on training/calibration splits**, then evaluate on held-out exact synthetic or independently referenced real traversals. Include corners, straight corridors, open scenes, repetitive warehouse aisles, valid partial scans, high speeds, degraded IMU/encoders and misaligned LD19 timing. Record uncertainty coverage, NEES where statistically defensible, false accepts/rejects, rank confusion and compute time. Avoid map-building/holdout leakage and invented absolute truth. Bootstrap is **optional offline validation**, not part of the onboard runtime.
- **Integration:** Distinguish **Ceres adapter tests** (Jacobian extraction, robust loss, optional `ceres::Covariance` cross-check) from **Ceres-free quality tests** (PSD/eigenvalue/rank, normalization, covariance inflation, nullspace and decision gates). Execute both through Docker Catch2/CTest, plus synthetic benchmark **B16** and Pi 5 timing when present. No GitHub Actions execution.

**Required edge-case tests:** Construct known rank-0/1/2/3 LiDAR Jacobians; Huber outlier treatment and no double beam weighting; verify `rho'` vs omitted `rho''`; corridor unobservable longitudinal pose with a tightly weighted odom prior; arbitrary yaw/translation physical normalization; projected correction preserving predicted null direction; reject rank deficiency without false covariance; calibrated PSD floors/full-rank bounds; finite/invalid Jacobians; low overlap, poor residual, map boundary/unknown and timing/slip gaps. A mock quality evaluator consumer must work with no Ceres or ROS installed.

See [OD-008](open-design-choices.md) and [Architecture Section 5.3.1](architecture-specification.md). Numerical values are **not fixed/calibrated by acceptance of the architectural choice**; accepted OD-013 governs the initial correlation-aware graph admission policy while exact joint covariance remains deferred.

### 10.5 Accepted OD-009 — Coarse candidate search, mode budgets and async recovery

The project MUST expose a **replaceable C++20 native coarse candidate-search interface**, independent of Ceres/ROS/hardware drivers, supplying ranked SE(2) hypotheses to the accepted OD-007 precise Ceres matcher. The OD-008 evaluator remains the **only** authority for geometric quality/observability acceptance; coarse correlation cannot directly publish map correction, information or trusted covariance.

| Operation | Search bounds | Initial Pi 5 coarse execution budget | Scheduling |
| --- | --- | --- | --- |
| Normal TRACKING | Direct OD-007 Ceres from OD-006 motion prediction | **0 ms** coarse search per scan | No coarse worker call |
| DEGRADED/UNCERTAIN | **±0.3 m X/Y, ±10°** around prediction | **10–20 ms** when invoked | Local bounded correlation |
| RECOVERY | **±1.0 m X/Y, ±30°** around usable hint | **30–50 ms** per invocation | Multiresolution branch-and-bound |
| GLOBAL_RELOCALIZATION | Eligible entire immutable candidate submap(s), **360°** | **100–200 ms per slice** | Async/resumable multi-resolution branch-and-bound |

These are **unmeasured implementation-tuning targets** for Pi 5 near 10 Hz scans. Measure and enforce host-steady work-time (separate from `/clock` simulation time), node/candidate counts, cache memory, worker queue capacity and scheduling overhead. A budgeted **incomplete** slice must be labeled `PARTIAL_BUDGET`, not `COMPLETE` or "no match".

- **Covariance-aware window:** Derive X/Y/yaw limits using the OD-006 pose covariance in a documented SE(2) tangent frame and configured multiplier, clipping to the mode's maximum extents and logging clipping. Invalid covariance must not silently become a narrow search. Select yaw sample spacing using map resolution/valid beam range so rotations do not skip significant map features; preserve yaw wrapping.
- **Coarse score and immutable cache:** Derive a **separate occupancy-based correlative score** and **multiresolution pyramid** from immutable versioned authoritative occupancy snapshots. The **OD-007 EDT cache stays distinct**. The correlation pyramid must record source occupancy/map revisions, known/unknown masks, grid origin including yaw, score/pyramid parameters, memory/build cost and invalidation. Unknown/outside/OD-005 invalid or missing measurements cannot count as observed hits or free-space evidence. Do not regenerate a frozen pyramid for every scan.
- **Admissible BnB:** Precomputed coarse scores must be demonstrable **upper bounds on the same fine score** for each candidate translation region and sampled angle, including masks/quantization. A heuristic coarse score must not be marketed as globally optimal branch-and-bound. Cross-check completed search against exhaustive enumeration on compact maps, with deterministic tie-breaking and no omitted true optimum.
- **Candidate diversity:** Retain configurable **spatially distinct** top-K candidates in ambiguous warehouses using SE(2) nonmaximum suppression. Return coarse score, supported beam count/overlap, candidate pose, scan/covariance/map/pyramid/clock/calibration revisions, search window, budget consumed and progress. Coarse scores are **not** OD-008 calibrated pose likelihood/covariance.
- **Refine and accept:** Apply OD-007 per candidate, then **OD-008** LiDAR-only rank, geometric overlap, residual, acquisition-uncertainty and innovation checks. In recovery/global, OD-007's **tracking odometry prior may be stale/tight**; use a documented mode-appropriate broadened, validity-gated or no-prior refinement policy rather than dragging valid distant hypotheses back to the original seed. OD-008 innovation/relocking must be state-aware; OD-020/022 still govern multi-scan hypothesis persistence and global pose publication. **No scan map insertion** while localization remains LOST/RELOCALIZING or quality untrusted.
- **Asynchronous safety:** Large BnB searches MUST run on a separate **bounded worker**, yield between slices, support cancellation and shutdown, and never block UART/IMU/encoder acquisition, raw recording, continuous local odometry, normal scan matching or ROS callbacks. Search may resume only when map/scan/time/config snapshots and frontier match; new revisions, calibration changes or time jumps invalidate/cancel stale work. Expose `COMPLETE`, `PARTIAL_BUDGET`, `CANCELLED`, `INVALID` and reasons.
- **API modularity:** Native `PoseCandidateSearch` / `SearchRequest` / `SearchResult` abstractions accept a generic OD-004 midpoint-deskewed, OD-005 eligible scan, an immutable correlation-map snapshot and optional OD-006 predicted pose/covariance, mode-specific caps, cancellation and budgets. The coordinator invokes Ceres and `MatchQualityEvaluator` through their existing public interfaces. Keep objectives/budgets independent and avoid new Boost usage without explicit case-by-case discussion.
- **Particle baseline:** Do **not** require a particle filter or its dependencies in the first implementation. OD-020 remains open if repeated global ambiguity/kidnapping proves the need for persistent multi-hypothesis filtering. This does not prohibit optional future AMCL/Beluga comparisons.

**Docker Catch2/CTest acceptance:** No coarse call in normal tracking; ±0.3/±10 and ±1.0/±30 clipping; full 360° sampling/yaw wrap; angle step vs grid resolution/range; admissible upper-bound tree vs exhaustive small-grid search; empty/unknown/rotated maps and partial scans; immutable pyramid cache re-use/invalidate; repeatable top-K candidates in symmetric warehouse; wrong odom prior with relaxed recovery refinement; OD-008 reject despite high coarse score; timeout/partial-budget/cancel/resume/map-revision semantics; multi-scan relock before publication; **ongoing simulated 10 Hz sensor capture and odometry while global BnB runs**. Check resource budgets on Pi 5 separately; Docker on a workstation does not prove Pi deadlines.

**Required diagnostics and Web Lab:** Search mode, configured/actual window, pyramid/map revision, coarse scores and distinct hypotheses, search node count/prune percentage, time per slice, in-progress/incomplete/cancel state, candidate Ceres/OD-008 outcome, stable multi-scan relock status and queue/CPU load. Heavy search is opt-in and must not be triggered on every scan during healthy tracking.

See [accepted OD-009](open-design-choices.md), [architecture `5.5](architecture-specification.md) and [B17 benchmark](benchmarks-and-datasets.md). No implementation or measured Pi performance is asserted.

### 10.6 Accepted OD-010 — Probabilistic occupancy, submap lifecycle and freeze guarantees

The **native C++20 map builder** MUST implement **authoritative 0.05 m log-odds occupancy** with a separate known/unknown flag, **two normally overlapping active submaps**, **90 accepted-insertion successor trigger** and **180 per-submap accepted-insertion freeze**, using bounded dynamic allocation. All values are **initial configurable benchmarks**, not prevalidated Pi 5 sizing or calibrated LD19 probabilities.

- **Lifecycle and insertion ID correctness:** First accepted scans 1–90 enter A, then create B after #90; accepted 91–180 enter **both** A/B, freeze A **after #180** and create C; B reaches 180 accepted insertions at global ID 270. Max two mutable active maps; overlap of adjacent complete maps is 90 **shared scan IDs**, ~50% **by insertion count**, not a spatial coverage guarantee. Track per-map accepted scan IDs/actual counts and transition/end reasons. A scan skipped by OD-005/004/008 eligibility or the **motion filter** updates no map and no insertion counter, but remains recorded and does not stop odometry. Insert a qualifying scan **once into each active map**, in local submap coordinates. Never match it against a map revision already incorporating it.
- **Empty-map bootstrap:** Ordinary OD-007/008 matching against a pre-insertion map is impossible before the initial map has occupied geometry. Allow only an **explicit initial bootstrap** with declared local pose gauge and validated OD-006 trajectory/OD-003–005 scan timing and quality (or an external documented initial pose), recorded as **BOOTSTRAP_UNVERIFIED_MAP_MATCH**. No falsely verified global fix. Transition to pre-insertion OD-007/008 gating as soon as supported map geometry exists. Do not use bootstrap to permit insertion during LOST/UNCERTAIN later in a session; successor overlapping submaps inherit established map/odom alignment.
- **Log-odds semantics:** Preserve `observed=false` for genuinely unknown cells even though their prior is `p=0.5`; an observed-known-neutral cell is distinct. Implement `L_next=clamp(L+Delta, logit(0.10), logit(0.90))` with `Delta_hit=logit(0.55)` and `Delta_free=logit(0.49)`. Per-cell prior, hit/miss and probability bounds MUST be configurable/versioned; calibrate on B18.
- **Ray correctness and hit precedence:** For trustworthy `VALID_HIT` raytrace free **before** its endpoint and occupied **at** endpoint. For `VALID_NO_RETURN` clear an explicitly bounded ray **only** if OD-005 sensor semantics/config permit; `INVALID_RETURN` and `MISSING` produce **no occupied or free updates**. Accumulate per-scan/per-submap unique hit and free cell sets, make **hit win over free**, and update **each cell at most once** in one scan/submap insertion. No invented beams/clear sectors; duplicate points cannot increase log odds repeatedly within one insertion. Reject/out-of-bounds ray endpoints without producing false occupied boundary cells.
- **Motion filter and caps:** Implement configurable translation/yaw and optional elapsed-acquisition-time motion filtering with pose/time uncertainty gating. The filter only controls map insertions, not timestamps, raw acquisition, OD-006 integration or localization outputs. Cap spatial radius/grid extent, allocated cells, RAM and update work; on cap/allocator error record early freeze/reduced actual overlap or stop mapping safely. Do **not** grow allocation unbounded, claim 180 insertions after an early stop, or invent geometry from clipped ranges. Keep raw data and diagnostics available.
- **Immutable freeze and graph separation:** Freeze the **local raster, known mask, local origin and scan insertion provenance** irreversibly, with content hash/revision; optimization may change only the independently versioned global `T_map_submap`. A graph correction does **not** invalidate local-grid source hashes or identical local EDT/correlation score caches; update global spatial index/pose-version as needed. Coherent immutable map revisions MUST be used by OD-007/009 readers; do not expose an active mutable grid directly to concurrent matching.
- **Derived caches:** OD-007 unsigned EDT and OD-009 multires correlative score pyramid are **separate rebuildable, versioned caches** referring to exactly one authoritative occupancy content revision, occupied confidence threshold, known/unknown mask, grid origin, resolution and their algorithm/truncation/score revisions. **Accepted OD-011** selects coherent snapshots, a full-field initial baseline, eventual active tiled EDT, frozen cache eviction/reload; OD-012 still defines persistence. Do not treat a derived cache as source truth or silently mix versions.
- **Dynamic scene policy:** Frozen maps have **no time-based decay**. Initial transient-object handling uses OD-005 validity and probabilistic contradictions/OD-007 robust outliers; any future time-aged occupancy belongs in a **separate transient overlay**, not a mutation of frozen map data.
- **Portability:** `OccupancyGrid` / `SubmapBuilder` / `FrozenSubmap` / `MapSnapshot` public headers must be ROS-independent and contain no Ceres, GTSAM or sensor-device types. Optional ROS OccupancyGrid/PGM/YAML exports explicitly preserve known/unknown semantics where representable; full versioned map serialization remains OD-012.

**Docker Catch2/CTest acceptance criteria:**

1. 90/180 exact insertion sequence A 1–180/B 91–270, overlap 90, no duplicate boundary scan and at most two writable maps.
2. Motion filter/insertion-quality rejection does not affect counters; match uses prior revision; two-map scans update one time per map.
3. `L=logit(p)` increments and clamps, contradictory observations, known-neutral `p≈0.5` vs genuinely unknown; invalid/missing/no-return safe update behavior.
4. Multi-ray crossing and duplicated endpoints: each cell updated **once per scan per submap** with **hit precedence**; no fictitious free clearing/hit at unsupported range or grid boundary.
5. Grid resolution 0.05/0.025/0.10 m, rotated origins, laser extrinsics, dynamic extent and memory failure; failure-safe early freeze and true insertion count.
6. Frozen occupancy byte/content hash unchanged when `T_map_submap` changes; correct derived EDT/pyramid source references, coherent active snapshots, GTSAM graph poses isolated from grid and deterministic replay.
7. B18 benchmark accuracy/resource reports on synthetic apartment/warehouse and Pi 5 (when available), with no automatic dataset download or GitHub Actions use.

**UI diagnostics:** Web Lab MUST expose each submap's state, inserted ID span/count, overlap, known/free/occupied statistics, resolution, physical size, allocated cells/peak bytes, motion-filter skips, freeze reason, immutable local source revision, graph pose revision, EDT/pyramid revision and update/rebuild latency. No Web Lab interaction mutates frozen production occupancy.

Reference: [accepted OD-010](open-design-choices.md), [architecture Section 6](architecture-specification.md) and [B18 benchmark](benchmarks-and-datasets.md). The numeric hit/miss/90/180 choices require calibration, not just implementation.

### 10.7 Accepted OD-011 — Atomic map snapshots, bounded build workers and frozen-cache lifetime

Implement the accepted **OD-011 immutable snapshot architecture** behind native C++20 domain interfaces, with **authoritative OD-010 occupancy/observed mask**, derived **OD-007 truncated unsigned EDT**, and independent **OD-009 correlation-score pyramid**. The snapshot service MUST supply **coherent versions**, immutable reader lifetimes, correct source/config hashes, bounded staleness and resource isolation without importing ROS, Ceres or hardware types into public headers.

**Staged delivery, without premature optimization:**

| Stage | Required behavior | Acceptance |
| --- | --- | --- |
| **I: Simple correct baseline** | A bounded background worker generates complete exact EDT + complete correlative pyramid from an immutable occupancy revision, periodically or on demand; publishes one coherent snapshot | No every-scan synchronous rebuild, no mixed revisions; full-field baseline passes correctness and load tests |
| **II: Target active-map optimization** | Track occupied-threshold and known-mask dirty regions; update **32×32 EDT output tiles** with **≥0.30 m source halo + bilinear support**, exact under source **addition/removal**; independent pyramid-score invalidation | Tiled output equals full exact field within tolerance for all supported cells, including removal/edge/unknown cases and admissible OD-009 upper bounds |
| **Both** | Precompute frozen submap fields, memory-bounded pinned-cache reuse/eviction and background on-demand loading | Frozen local fields do not rebuild merely because global graph pose changes; unpinned memory can be evicted/reconstructed |

**Initial numeric configuration:** OD-010 resolution **0.05 m**; unsigned EDT truncation **0.30 m** (different from a signed TSDF); EDT tile **32×32 cells** = **1.6 m × 1.6 m**; source halo at least **six 5 cm cells plus interpolation-cell support and boundary rounding**. This is a starting point, not a measured Pi 5 optimum. A **40×40 m** grid has 640,000 cells; float32 EDT alone occupies **2,560,000 bytes (~2.56 decimal MB / 2.44 MiB)** before source mask, two active submaps, pyramid, scratch and pinned versions.

- **Snapshot identity and atomic publication:** A `MapSnapshot` binds **submap ID**, monotonic local occupancy revision, content hash, immutable local grid/observed mask, compatible EDT and correlation-pyramid source revisions plus their **own config hashes**, resolution/origin, supported readiness and insertion high-watermark. Publish only a **complete operation-ready coherent version** atomically; an EDT-only ready snapshot cannot masquerade as pyramid-ready. No occupancy N+1 paired with EDT/pyramid N. Work from a pinned immutable source; discard/defer obsolete builder results and never publish partial tiles.
- **Read pins:** OD-007 Ceres and OD-009 asynchronous BnB each acquire a native `shared_ptr<const MapSnapshot>` (or equivalent) and retain it **throughout the entire operation**, including its stable map-to-local transform/graph-pose version where needed. Neither optimizer can observe concurrent map mutation or cache eviction during evaluation.
- **Exact dirty updates:** Occupied-confidence **threshold transitions in either direction**, observed-space mask changes, source growth/origin/config changes must be observed. At EDT truncation radius `R`, dirty all output tiles intersecting an `R`-expanded changed-source neighborhood plus interpolation stencil. Recompute output against all sources in a `R`-expanded halo, including obstacles outside tile and **after removals**; compare to exact full EDT. Full rebuild is required when tile equivalence cannot be proven or dirty work is too broad. Propagate numeric truncation and interpolation/mask validity identically.
- **Separate OD-009 score pyramid dirty logic:** A change in a cell's occupancy probability (not just a threshold transition) can alter the coarse score and its ancestor maximum bounds. Rebuild/publish the pyramid coherently for the same occupancy revision and **verify bound admissibility** including score decrease, unknown support change and rotated map coordinates. EDT dirty tiles are not a sufficient invalidation signal for the score pyramid.
- **Staleness:** An older **complete** snapshot can be used during a background rebuild **only within configured maximum host-steady age AND maximum accepted-scan-insertion lag**. Report both to OD-008 and the trace; exceed either -> defer/reject matching/insertion, never mix inconsistent versions. OD-010 match-before-insert must exclude the current scan from its own map. No automatic removal of previously accepted scans to synchronize old snapshots.
- **Frozen submaps:** Freeze the OD-010 local raster/known mask permanently, finalize derived fields asynchronously, then mark the frozen matching bundle ready. Cache hot frozen EDT/pyramid entries, evict **only unpinned** cold derived data under fixed memory budgets; reconstruct/reload on a background worker if missing or config hash changes. A global GTSAM `T_map_submap` update changes global pose revision/index, **not the local-cache source identity**.
- **Bounded concurrency:** Map build/refresh, on-demand frozen loads and OD-009 global search execute on separate/bounded, cancellable work with queue limits, memory accounting for retained old `shared_ptr` snapshots and scratch; **must not block LD19/IMU/encoder acquisition, raw recording, gyro/odom updates or main scan transport**. Under pressure throttle, skip or cancel lower-priority work and surface `NOT_READY` / `STALE` / `MEMORY_PRESSURE`, not unsafe mixed data or unlimited allocations.
- **ROS-/Ceres-free API:** Domain `OccupancyRevision`, `MapSnapshotProvider`, `SnapshotHandle`, `CacheConfig`, `CacheDiagnostics` types expose version/ready/stale/resource state without importing `ceres::`, ROS messages, `gtsam::` or specific sensor formats. Freeze/persistence APIs may use OD-012 versioned storage adapters but cannot require a fixed file layout at OD-011 stage.

**Docker Catch2/CTest mandatory tests:**

1. Exact full EDT vs incremental tiled distances and bilinear values/gradients for **source additions, removals, simultaneous hit/free changes, near-tile-boundary sources, unknown/known transitions, grid growth, map edges and changed threshold/configuration**.
2. Score pyramid changes on **probability-only updates** without EDT occupancy threshold change; coarser upper bounds remain admissible after decreases/known-mask changes vs exhaustive OD-009 score evaluation.
3. One Ceres scan and a long asynchronous BnB both pin immutable version N while N+1 builds/publishes; no N+1 occupancy with N caches, no use-after-free, no stale worker overwrite, no mixed global pose frame.
4. Max accepted-insertion lag and host-steady staleness gate; deferred/rejected matching after limit; no self-match, deterministic versioned replay and explicit `NOT_READY` status.
5. Frozen precomputation, config-hash invalidation, unpinned eviction/reload and retained pinned reader; GTSAM pose-only change does not rebuild the local field; bounded memory when snapshots outlive current map.
6. Memory/CPU/queue pressure, cancellation/shutdown and worker fairness at simulated ~10 Hz acquisition; no blocking sensor recording, odom or normal local tracking.
7. Profile **B19** on x86-64 and actual Pi 5 for complete vs tiled EDT, 32-cell tile/halo, cache hits, p50/p95/p99 matching/build latency, peak RSS and scheduling interference, separately reporting tile correctness and physical performance.

**Milestone alignment:** Stage I is part of C3 mapping/frontend infrastructure and C4 map infrastructure; Stage II is a **profiling-justified optimization**, not a gate for initial correctness. Frozen cache readiness supports C5 known-map localization. Web Lab SHOULD report source revision, cache config hash, EDT/pyramid readiness, dirty tile count, rebuild mode, snapshot age/insertion lag, pin count, cache hits/evictions, worker load and canceled/stale builds.

Reference: [accepted OD-011](open-design-choices.md), [architecture `6.3.1](architecture-specification.md), and [B19](benchmarks-and-datasets.md). Parameter tuning and measured target performance are **not yet validated**.

### 10.8 Accepted OD-012 — Portable directory map bundles and crash-safe checkpointing

**Initial native persistence contract:** A strict **JSON manifest + individual, compact fixed-width binary payloads** compressed with **Zstandard** under an immutable directory-based generation. Persist the **authoritative OD-010 occupancy log-odds and observed/unknown masks**, local origins/resolutions/scan ID histories, independently versioned submap global poses and graph nodes/factors/covariance or information, reconstruction/config/calibration and OD-001 clock/session metadata. **OD-007 EDT and OD-009 score pyramids are optional regenerable OD-011 derived caches**. No GTSAM raw-object dumps, native C++/Eigen object bytes, host endian or raw `size_t` serialization.

~~~text
building_a.slam/
  CURRENT                            # for mutable checkpoint series only
  generations/
    000000000001/
      manifest.json                  # major/minor, features, provenance, file hash index
      graph.bin.zst                  # versioned graph/factors/uncertainty
      submaps/sm_000001.bin.zst      # separately compressed local raster + known mask
      cache/*.bin.zst                # OPTIONAL, source/config-validated
      checkpoint/state.bin.zst       # ONLY RESUMABLE_CHECKPOINT
      preview.png                    # OPTIONAL
~~~

A one-shot immutable export MAY be a single self-contained bundle directory. For repeated saves, use **fresh immutable generations plus an atomically updated `CURRENT` pointer**, not in-place replacement of a nonempty map directory. Keep a known-good prior generation; loaders resolve and pin one generation through load/validation.

**Mandatory manifest/payload metadata:** Unique map/generation ID, `bundle_kind`, explicit **major/minor schema**, required/optional features and minimum reader; coordinate frames with SE(2) axis/rotation conventions; **rotated submap origin**, width/height/resolution, grid index order, log-odds encoding and observed-bit semantics, insertion IDs/counters/freeze reason; graph pose/node/factor IDs, factor provenance and information tangent ordering, calibration and known/map/clock revisions; index of **safe relative paths, roles, codec, compressed + uncompressed sizes, SHA-256 per stored file**, and cache source/config hashes. Payload schemas must have documented magic/header/version, fixed integer and IEEE-754 formats, byte order (initial proposed **little-endian**), bounded record lengths/counts and NaN/Inf rejection. A future major/minor reader migrates explicitly to a new validated generation; unknown required features fail.

**Zstd and lazy load:** Compress each submap **independently** as its own zstd frame/file so the `MapBundleReader` can load only selected frozen submaps. Graph/checkpoint files may also be separately compressed. Limit zstd decoder window, compressed/uncompressed sizes, total file count, graph factor count, memory and disk usage before and during decompression. **Missing/stale/corrupt optional caches must trigger bounded regeneration**, not a failed authoritative map load, while required graph/occupancy corruption fails closed. Source graph pose revisions do not invalidate unmodified local cache sources.

**Save transaction and crash recovery:**

1. Capture a **consistent immutable graph and referenced submap snapshot** from OD-010/011; checkpoint generation captures all live active map/estimator/clock/replay state without making acquisition wait for compression.
2. Write a **unique staging directory on the same filesystem**, all authoritative payloads, then JSON manifest with verified cross-references/size/hash. Ensure required data and schema are complete, enforce disk/resource budgets.
3. Flush data to durable storage with file `fsync`/`fdatasync` and directory sync as applicable. Publish by renaming staging to a **previously nonexistent immutable generation**; sync parent.
4. Write/sync a temporary `CURRENT` file; **atomically rename it over `CURRENT`**, then sync parent directory. Retain at least one last validated generation until replacement has been durably confirmed. Never depend on atomic replacement of nonempty directories.
5. Load **one pinned generation**, validate **all required authoritative files** before publishing a map or graph object; on invalid/missing latest pointer/generation perform **bounded verified previous-generation fallback**, ignoring partial staging. Log recovery and never combine generations. Unsupported filesystem sync/rename semantics must produce a clear error/weaker guarantee, **not an unsupported claim of universal power-loss durability**.

**Security and compatibility:** Reject path traversal/absolute paths, symlinks/escapes, duplicate IDs, malformed lengths/overflows, huge decoder frames, file count/size violations, invalid graph references, nonfinite SE(2)/covariance fields, unsupported codecs/required versions and inconsistent masks/grid shapes. SHA-256 checks integrity, not author authenticity. Unknown optional JSON keys are accepted only when safe; incompatible major versions or mandatory feature flags fail with typed error. ROS 2 Nav2 map image/YAML and ROS OccupancyGrid import/export are **isolated, explicitly lossy adapters**; the native persistence library never links ROS.

**Map-only vs resumable:** `LOCALIZATION_MAP` permits read-only frozen-map localization and graph inspection. `RESUMABLE_CHECKPOINT` MUST additionally persist active submap occupancy/builders/90–180 counters, OD-006 bias/trajectory/covariance, calibration, OD-001 **clock domain+epoch+session identity/fit mapping**, graph continuation revision, accepted insertion high-watermark and replay/recorder input cursor/identity. Resume must **reinitialize or validate the new monotonic clock-session mapping** after reboot; old nanosecond values alone cannot identify the new live epoch. A localization-only map or incomplete checkpoint MUST reject resume; OD-024 still decides replay checkpoint scheduling.

**Public interfaces:** Native C++20 `MapBundleWriter`/`MapBundleReader`/`MapBundleManifest`/`MapBundleLoadOptions`/`MapGeneration` (illustrative) use portable schema/map structures, bounded streaming and typed errors **without Ceres, GTSAM-native-object, ROS, Pico, LD19 or Boost dependencies** unless a separately approved case is made. GTSAM and ROS conversion lives in their own adapters. New Zstandard dependency is authorized by this selected option; pin/document its version and Docker package, test static/dynamic linkage on Pi/x86, and do not silently add unrelated dependencies.

**Docker Catch2/CTest acceptance (B20):**

1. Exact roundtrip of authoritative OD-010 log-odds **and known-neutral mask**, 2 active/frozen submap IDs, rotated origins/units, graph nodes/factors/covariances and clock domain/session/calibration IDs.
2. Cross-platform/endianness golden binary fixtures, schema major/minor migration and required-feature refusal; no raw C++ memory serialization.
3. Independent zstd submap file/lazy read, bounded streaming decompression, optional-cache stale/missing/corrupt rebuild, wrong source/config hash handling.
4. Required-file checksum/corruption/truncation, missing graph endpoint/duplicate IDs, unsafe path/symlink and oversized compressed payload injection must **fail closed**, with no partially published runtime map.
5. Fault injection at staging write, file sync, generation rename, `CURRENT` replacement, dir sync and old-generation GC: use last valid completed generation or explicit error, not partially updated graph/occupancy.
6. Complete `RESUMABLE_CHECKPOINT` save/load restores active insertion IDs, pose/covariance, session-aware time fit and replay cursor; map-only `LOCALIZATION_MAP` rejects resume and recognizes rebooted clock epochs.
7. ROS/Nav2 map conversion report documents loss, adapter-only ROS dependency. Compare Pi 5 and x86-64 CPU/RAM, save/load latency, compressed size, SSD/SD-card write load and disk-full handling. No GitHub Actions run.

**Milestone:** B20 and an independently loadable `LOCALIZATION_MAP` belong to C4 offline-mapper output; `RESUMABLE_CHECKPOINT` may follow after OD-024 replay/live-state coverage, but the format must distinguish it from day one. No application implementation or measured persistence throughput is implied. See [OD-012](open-design-choices.md), [architecture Section 10.1](architecture-specification.md) and [benchmark B20](benchmarks-and-datasets.md).

### 10.9 Accepted OD-013 — Keyframe/submap Pose2 graph and correlation-aware factor admission

The initial GTSAM backend MUST optimize **global keyframe Pose2 `X_k`** and **OD-010 submap global Pose2 `S_j`** variables, while high-rate OD-006 wheel/gyro motion estimation, stationary/online gyro bias and OD-004 continuous trajectory deskew **remain in ROS/GTSAM-independent core code**. Do not add a full joint pose/bias smoother initially. **GTSAM's factor graph implementation must be private** to a separately linked backend; portable graph-domain headers and OD-012 map bundles store no `gtsam::` classes or C++ factor object dumps.

**Initial factor types and exact evidence policy:**

| Factor / record | Contract |
| --- | --- |
| Global gauge | One clearly defined reference frame/gauge; no duplicated arbitrary high-confidence anchoring |
| Interval relative-motion | **One fused wheel/gyro OD-006 SE(2) factor per keyframe interval**, with unique source sample/time IDs and calibrated uncertainty |
| Primary keyframe↔submap | **At most one primary local LiDAR-only geometry constraint per keyframe/scan encounter**, OD-007/008 verified and snapshot-referenced |
| Loop closure | Against a target that **does not contain the query scan**, with overlap/duplicate checks, OD-008 rank/quality and robust loss after validation |
| Submap anchor | Frame-definition/initialization and scan membership bookkeeping **not an additional independent sensor factor** |
| Bias/trajectory | OD-006 native estimator only; graph bias states/raw wheel/gyro duplication deferred |

For relative LiDAR pose `Z_jk = T_submap_j_base`, predicted `Z = S_j^-1 X_k` with `r=Log(Z_jk^-1 S_j^-1 X_k)`. Verify Lie tangent ordering, transformations and local/global frame conventions, gauge and factor Jacobians through independent analytic/synthetic/finite-difference tests. An explicit anchor relation must not redundantly constrain the global submap pose and prevent loop deformation. **Accepted OD-015** defines the initial 0.20 m/5°/2 s motion keyframes with 0.2 s minimum interval and independent submap anchors.

**Double counting prohibitions:**

- **Motion:** Do not add fused OD-006 relative-odometry, its raw encoder wheel factor and gyro yaw factor over the same interval as three independent factors. One fused motion factor is the initial rule. Missing sensor modalities cannot yield a fabricated fully constrained factor.
- **OD-007 prior:** Ceres's result already includes an odometry prior and boundary penalty. Its **prior-conditioned posterior pose or posterior Hessian MUST NOT be exported as an independent LiDAR scan factor**, even if the regularizer Hessian is subtracted later. Require independently evaluated **LiDAR-only geometric relative pose/information** through a separate prior-free/observable-subspace refinement or a defensible conditioned model. If not possible, omit the LiDAR factor and report reason.
- **Deskew:** OD-004 wheel/gyro-conditioned LiDAR endpoints share motion noise with the fused odometry factor. Record contributing wheel/gyro sample IDs and calibration/time intervals. Under OD-008, use empirically calibrated **conservative LiDAR covariance inflation**, with an explicit approximation/correlation note. Inflation **does not mathematically remove cross covariance**; test held-out consistency and flag overconfidence.
- **Overlapping OD-010 submaps:** Persist **both scan insertion memberships** (nominal A/B shared accepted scan IDs 91–180), but admit only **one primary local LiDAR constraint per keyframe/encounter**. Do not automatically create a second independent factor by matching against the other overlapping submap.
- **Self-matching and repeated loops:** OD-011 provides pre-insertion pinned map revisions for local matching; a **new loop factor whose target contains the query scan is prohibited**. Deduplicate repeated scan/target/encounter source groups; limit heavily overlapping loops and treat shared raster evidence conservatively even with common submap variables.
- **Degenerate geometry:** OD-008 rank 0–2 LiDAR evidence MUST **not** become an ordinary invertible full 3×3 graph factor. **Reject initially** until a tested observable-subspace factor type is implemented and explicitly selected. A robust noise model is applied only after geometric/innovation/correlation validation, not as the sole defense against a bad loop.

**Portable provenance and interfaces:** Native `PoseGraphModel`, `GraphFactorSpec`, `EvidenceProvenance`, `GraphFactorAdmission` and `GraphOptimizationResult` (names illustrative) carry keys/frames, measurement/residual frame, noise model and OD-008 calibration/rank, raw wheel/gyro sample/time ranges, OD-004 deskew inputs, query scan IDs, OD-010 memberships, OD-011 pinned occupancy/EDT source hash and graph pose revision, OD-007 prior/geometric evidence, candidate/loop encounter and **correlation-group/duplicate IDs**, acceptance/rejection reasons. GTSAM conversion is private, ROS topic serialization separate, portable graph schema OD-012 versioned and independent of GTSAM binaries.

**Batch-first execution:** Initial solver is **GTSAM batch Pose2 LM/Gauss–Newton** offline. Build factors and optimize asynchronously from acquisition/recording/OD-006 motion so large graphs cannot block capture. Publish only a numerically valid **immutable new graph-pose revision** after loop quality and gauge checks. OD-010 frozen raster/OD-011 local EDT/pyramids never change when the global `S_j`/`X_k` optimize. Continuous `T_odom_base` is never jumped; correction goes through `T_map_odom` and OD-022 global-pose quality. Online iSAM2 remains optional under OD-019, future full joint bias/trajectory factors require a new validated design, and OD-014 re-deskew/remapping creates **new versioned maps**.

**Docker Catch2/CTest and B21 mandatory regression:**

1. Keyframe/submap Pose2 graph exact-synthetic truth, gauge once, factor Jacobians/whitened tangent noise; compare simple pose-only baseline to keyframe/submap topology.
2. Fused wheel/gyro interval factor uniqueness; reject factor sets that also contain raw encoder and yaw factors for the same sensor samples; missing modality/source provenance.
3. Nominal OD-010 overlap of 90 scan IDs for A/B and **two memberships but one primary local LiDAR factor**, with no unwarranted anchor information and freely optimizable global submap poses.
4. Repeated matching/optimizing same scan produces no artificial information growth, self-match target rejected, overlapping and duplicated loop candidate groups rejected or conservatively handled.
5. Strong OD-007 odometry prior and a divergent LiDAR measurement: no prior-conditioned posterior-as-independent-factor; LiDAR-only refinement/geometry rank or abstain; OD-008 full/degenerate covariance correctly represented.
6. High-yaw-rate/translation and wheel slip/gyro bias errors with shared deskew intervals: show conservative inflation/consistency against independent synthetic truth and no falsely independent odom/scan factors.
7. Ambiguous repeated-warehouse loop and injected wrong closure: require verified geometry/rank and robust post-gating; no irreversible false map flip or unjustified factor.
8. GTSAM graph optimizes `X_k/S_j` and changes versioned `T_map_odom` while OD-006 odom remains continuous, frozen local raster and cache hashes remain unchanged.
9. Native graph records round-trip through OD-012 `graph.bin.zst` with source IDs/factor uncertainty, no Ceres/GTSAM dependencies in portable headers, no raw object serialization.
10. Graph node/edge density, held-out ATE/RPE and covariance coverage where independent reference, loop false-positive metrics and p50/p95/p99 batch CPU/RAM/latency on x86/Pi where available; no acquisition or recording blocking. **Do not dispatch GitHub Actions.**

**Milestone alignment:** C4 offline mapper must include accepted OD-013 **batch graph topology and safe factor admission** together with OD-010/011/012 persistence; C6's future iSAM2 path must preserve the same native graph records and needs OD-019 timing validation. Web Lab SHOULD expose `X_k/S_j` nodes and factor types, both insertion memberships, admitted/rejected loop edges, correlation groups, OD-008 rank/calibration, pose revision and continuity of odom vs map correction. Reference [OD-013](open-design-choices.md), [Architecture Section 7](architecture-specification.md) and [benchmark B21](benchmarks-and-datasets.md). Graph implementation and measurements are **not yet complete**.

### 10.10 Accepted OD-014 — Deterministic selective offline refinement and rollback

**Implementation sequence:** Deliver **one full deterministic offline map/graph processing pass `R0`** first, then a separate, opt-in `OfflineRefinementCoordinator` supporting **up to three additional outer candidate iterations `R1`–`R3`**. This is not a requirement for onboard live SLAM; it runs on offline workers without slowing raw sensor acquisition or OD-006 real-time trajectory updates. Do not add coupled GTSAM gyro-bias/clock states to the initial accepted OD-013 Pose2 graph.

**API/ownership:** Native C++20 `OfflineRefinementCoordinator`, `RefinementPlan`, `IterationSnapshot` and `RefinementDecision` (illustrative) accept and return **portable** scan IDs, local SE(2) trajectories, bias/clock revisions, pose-graph snapshots, occupancy/derived-cache source versions, geometry-delta decisions, validation metrics and generation IDs. They MUST have **no Ceres, GTSAM or ROS dependency in public headers**. Private OD-007 Ceres and OD-013 GTSAM adapt through their existing graph/matcher contracts, mapping through OD-010/011 and atomic saves through OD-012.

| Input change between candidate and accepted result | Re-deskew | Submap rebuilding |
| --- | --- | --- |
| Pure global rigid graph correction `T_map_submap` | **No** | **No**, graph pose/global spatial indexes only |
| Common rigid local-frame change leaving all `T_submap_scan` geometry intact | **No** | **No** |
| Relative scan midpoint pose changed in a submap | Only if intra-scan motion changes | **Yes** for material local ray shift |
| OD-004 within-scan wheel/gyro trajectory correction | **Yes if material** | **Yes** for affected inserted scans/submaps |
| Observable OD-006 gyro bias / OD-002/003 clock offset and drift change | **Yes if material** | **Yes if beams move materially** |
| All local endpoint changes below gate, no quality/validity regression | Generally no | Generally no |

- **Half-cell geometry gate:** Start at **0.5 × 0.05 m = 0.025 m maximum** valid beam-endpoint movement in a **consistent OD-010 submap-local frame**, comparing the candidate to last accepted geometry. Include OD-003 reconstructed beam times, OD-004 SE(2) trajectory, laser extrinsics and rotational lever arms, and check every valid beam (or conservative upper bound) across **both overlapping submap memberships**. Do not make the decision from global graph-pose displacement, mean displacement or yaw alone. The 2.5 cm initial trigger is configurable **and unvalidated**; invalid clock/trajectory/scan-eligibility evidence overrides the threshold.
- **Calibration observability:** Optional OD-006 gyro bias refinement requires independent stationary/observable motion evidence and bounded uncertainty; wheel/gyro disagreement may be slip. Optional OD-002/003 clock offset/drift updates require independent synchronization/physical timestamp-event calibration, same OD-001 domain/session and bounded uncertainty. **Do not optimize LD19 time delay or bias merely to reduce current-map LiDAR cost** and then treat it as independently observable. Unobservable parameters stay fixed and their provenance is recorded.
- **Recompute from original evidence:** Affected occupancy must be **rebuilt from scratch from immutable raw packets/IMU/encoder streams and original accepted beam/ray evidence**, not in-place warped, incremented into frozen log odds, or superficially shifted in a rendered map. Preserve OD-005 invalid/no-return semantics, hit-before-free precedence, single cell update per insertion and OD-010 scan IDs/90–180 overlaps as appropriate. A validity-driven membership/scan selection change MUST be versioned and explained.
- **Dependency invalidation:** On material scan geometry change, create new local OD-010 occupancy hashes, rebuild affected OD-011 EDT and OD-009 correlation pyramids, and recompute **all affected OD-007/008 geometry constraints, OD-018 loop candidates where needed, and OD-013 factors** from the new pinned immutable snapshots. **Replace** invalidated factor records; never append another independent factor from the same scan or reuse a self-match/overlapping submap loop. Keep factor/correlation/source interval IDs fully auditable and conserve accepted unmodified snapshots where changes are only rigid.
- **Consistent candidate generation:** Each `Ri` captures configuration/algorithm version and deterministic seeds, immutable raw source checksums and file identity, clock epoch/offset/drift, scan IDs/time/eligibility, bias and trajectory revision, changed geometry decisions, map/cache source hash, OD-013 graph factor source/correlation groups, metrics and OD-012 generation ID. OD-012 `LOCALIZATION_MAP` alone **is not enough for replay**; retain raw MCAP/sensor streams and original acquisition/sync metadata. Publish only after graph/map/scan-source references reconcile and all required artifacts validate.
- **Acceptance/rollback:** Gate on finite graph optimization, **relative graph improvement only over comparable common factors**, scan-map quality/OD-008 observable rank and covariance, map consistency, credible timing/bias uncertainty, loop support, held-out scans or independent truth when available and no safety regression. A reduced cost after fitting to the **same rebuilt map** does not suffice. Stop early on convergence or lack of material geometric change, reject/revert to the **last accepted OD-012 immutable generation** on quality regression, invalid/false loops or an incomplete candidate; cap at **3 additional iterations** regardless of repeated improvements. Do not publish partially built cache/factor state or overwrite accepted raw/maps.
- **Offline resource limits:** Bound candidate count (three additional), rebuild area/submap count, snapshot scratch/RAM, worker queue, disk output and solve time. A slow run is not a live sensor-thread dependency. Expose cancellation and `NOT_READY`/`REJECTED`/`ACCEPTED`/`CONVERGED` states plus detailed stop reason; no hidden unbounded replay loop.
- **Web Lab/reports:** Show R0 vs each candidate `Ri`, calibration and raw source revisions, per-scan maximum displacement, re-deskew/rebuild counts, unchanged rigid submap counts, invalidated factor/loop/cache IDs, graph cost **only on comparable measurements**, held-out quality, OD-008 covariance consistency and accepted/rejected generation/rollback reasons.

**Mandatory Docker Catch2/CTest and benchmark B22:**

1. R0 one-pass baseline, no more than three **additional** iterations; deterministic ordering/seeds, cancellation, accepted generations stable across identical replays.
2. Rigid loop closure/global map correction leaves every immutable submap grid/EDT/pyramid hash unchanged, **zero re-deskew** and no cache rebuild; pose/index revision only.
3. Relative keyframe/scan-midpoint geometry changes trigger correct submap remapping **without gratuitous beam re-deskew**; altered intra-scan trajectory requires re-deskew, including high-yaw/long-beam cases over the 0.025 m bound.
4. Gyro bias, invalid gyro-bias-from-slip and clock offset/drift perturbations: only independently observable/calibrated corrections accepted; source epoch and LD19 physical delay uncertainty respected.
5. Raw hit/free/known/unknown and OD-005 invalid returns identical after remap, overlapping submap membership honored; no double log-odds writes, missing-rays as free or warped frozen cells.
6. Affected OD-007/008 matches, OD-011 versioned caches, OD-013 factor IDs and loop candidates replaced atomically; no appended duplicates, no self-match or outdated correlation groups.
7. True synthetic quality improvement vs false improvement from same-data overfitting, held-out error/covariance/loop-consistency regressions; **bad candidate rejected and previous accepted generation restored**.
8. Simulated interrupted OD-012 candidate save and wrong source-map/factor versions: old accepted generation persists; inputs/provenance remain accessible; no ROS/Ceres/GTSAM native objects leak through coordinator domain.
9. Profile selective vs unconditional full-map rebuild, original one-pass R0, p50/p95/p99 offline time, peak RSS, number of rebuilt scans/submaps, false loop rate, held-out ATE/RPE and map-cell consistency on exact apartment/warehouse and Pi when available. **Do not dispatch GitHub Actions.**

**Milestones:** C4 first delivers R0 + B18–B21 offline mapping and persistence. **OD-014 iterative coordinator and B22** are a later C4 enhancement; initial one-pass acceptance must not be blocked on tiled optimization or optional bias/time refinement. OD-024 replay seek/checkpoint strategy and OD-018 loop candidate retrieval remain open; **accepted OD-015** now fixes initial keyframe selection and optional future bias-knot criteria. See [OD-014](open-design-choices.md), [architecture Section 7.4](architecture-specification.md) and [B22](benchmarks-and-datasets.md).

### 10.11 Accepted OD-015 — Keyframe selector, anchors and optional offline bias knots

**Initial C++20 implementation:** Implement a **ROS-/GTSAM-/Ceres-independent `KeyframeSelector`** producing typed `KeyframeDecision` and reason/provenance records from timestamp-valid OD-004 midpoint scans, OD-006 predicted SE(2) motion/covariance and OD-008 quality diagnostics. Initial selection thresholds are **0.20 m translation**, **5° yaw**, **2.0 seconds elapsed during meaningful motion**, **0.2-second normal minimum interval**; all values are configurable/benchmark-tuned. The initial GTSAM Pose2 graph has independent `X_k` keyframe and `S_j` submap nodes and **zero graph gyro-bias knots**; OD-006 retains evolving bias correction.

- **Deterministic selection:** Compute `Delta=T_last_keyframe^-1*T_candidate`, Euclidean translation, shortest wrap-safe absolute yaw, elapsed acquisition nanoseconds and optional covariance/information. Trigger on **any** translation, rotation, moving-time or justified uncertainty criterion. Apply the **0.2 s floor** to ordinary selection and report `DEFERRED_MIN_INTERVAL`; do not silently insert a high-rate burst. Initial reference/root and OD-010 `SUBMAP_ANCHOR` events can override this minimum only with a typed exception. Treat clock-domain/epoch reset, invalid temporal ordering, incomplete scan midpoint and nonfinite predictor as rejected/degraded input; do not use host-wall time in place of OD-001 acquisition time.
- **Stationarity and meaningful time:** Suppress time-only keyframes while stationary, including multi-minute gyro-bias calibration. A moving-time trigger requires a configurable significant-motion/noise test, not mere system uptime. New information gain is an optional future trigger rather than an unsupported assertion. Use stable scan/source IDs, deterministic tie-break ordering and fixed threshold comparison conventions at equality.
- **Quality adaptation:** High OD-006 covariance can propose a keyframe **only within bounded node density/CPU** and with plausible additional geometry. OD-008 residual/overlap/rank/innovation independently decides LiDAR **factor** admission; a keyframe may exist to carry OD-006 relative odometry or submap-anchor bookkeeping even when `NO_LIDAR_FACTOR` is reported. High uncertainty with rank-deficient geometry should trigger OD-009/022 recovery/degradation, not a flood of graph nodes or pretend full-rank LiDAR constraints.
- **Fused motion factors:** Create **exactly one** OD-013 fused wheel/gyro relative-motion factor between **successive actual keyframes**, integrating the source samples and bias correction across that entire time interval. If a threshold crossing was deferred, do not add an extra factor over part of the same raw interval. Record raw sample ranges, timestamps, OD-006 bias, uncertainty and correlation group. If an anchor is inserted between planned nodes, recompute/split intervals consistently and avoid duplicated measurements/evidence.
- **Independent submaps:** OD-010 creates a new submap after **90 accepted scan insertions** and freezes after **180 per submap**, with overlap count independent of keyframe selection. Associate an anchor keyframe with each `S_j` and permit a documented 0.2 s exception if needed at birth; its initialization transform is **not an independent observation**. One keyframe's scan can have both A/B insertion memberships, with at most one primary local scan constraint under OD-013. Modifying keyframe thresholds cannot alter 90/180 insertion counts merely due to node scheduling.
- **Initial bias policy:** Bias knots in graph: **0**. OD-006 continues the bias-corrected wheel/gyro trajectory and covariance and feeds OD-004 deskew; no bias state, raw yaw and bias-corrected fused odometry are inserted independently into GTSAM.
- **Conditional future offline joint model:** OD-014 may optionally introduce **scalar yaw-gyro bias knots** `b_j` at timestamped **5–10 s starting intervals** only after validation. Their random-walk evolution is `b_(j+1)=b_j+w_j`, `w_j∼N(0,q_b*Delta_t_j)`, `Var(w_j)=q_b*Delta_t_j`, with `q_b` in **(rad/s)^2/s** and `Delta_t_j` in seconds within a valid OD-001 clock epoch. Include independent bias/drift/temperature observability gates. Knot interpolation and bias-aware **planar fused wheel/gyro relative-motion factors must couple bias to yaw/pose**. Do **not** add unused random-walk bias variables or naively substitute GTSAM 3D IMU preintegration for the planar model. Replaced interval motion factors must supersede original OD-006 fused factors (no duplicate evidence); OD-014 selectively re-deskews/remaps affected scans/submaps and replaces impacted graph factors. A biased gyroscope under wheel slip must not invent small bias covariance or erroneous calibration certainty.
- **Native interface and persistence:** `KeyframePolicy`/`KeyframeSelector`/`KeyframeDecision`/`SubmapAnchorAssociation`/`BiasKnotPlan` use only native time, SE(2), uncertainty, stable scan/interval IDs, OD-008 geometry diagnostics and source/bias provenance. GTSAM factor conversion stays private. OD-012 factor/checkpoint schema records each admitted keyframe/anchor, trigger/source interval, optional knot spacing/`q_b`/clock epoch and superseded factor versions. OD-014 deterministic replay may reselect nodes from changed trajectories but must rebuild graph factors and source IDs without duplicating raw intervals.
- **Tuning and reporting:** Record keyframe density per metre/minute, reason distribution, deferred/anchor overrides, no-LiDAR-factor rate, observability, duplicate-evidence prevention, pose/graph quality and CPU/peak RAM. Values are **initial tuning targets, not measured quality guarantees**; do not assert that 5–10 seconds matches actual sensor bias drift before hardware calibration.

**Docker Catch2/CTest and benchmark B23:**

1. Threshold below/equal/above **0.20 m**, **5°**, **2.0 s during motion**, **0.2 s minimum interval**, yaw 359°↔0°, translation with pure rotation, irregular scan timestamps/clock epochs and deterministic decisions.
2. Stationary periods spanning 2 s through minutes produce **no repeated keyframes**; bias calibration continues from raw OD-006 IMU samples without graph growth.
3. High covariance + little new LiDAR geometry does not create floods or LiDAR factors; uncertain but geometrically informative scans may request a bounded keyframe; OD-008 rank-deficient geometry never gets a fake full Pose2 measurement.
4. OD-010 90/180 insertion transitions and two overlapping memberships remain unchanged when changing keyframe settings; anchor created/associated at new-submap boundary and minimum-interval exception logged, graph `S_j` remains independently adjustable.
5. Fused wheel/gyro source IDs integrated exactly once over every adjacent actual-keyframe time interval; deferred candidates and forced anchors do not duplicate raw motion factors.
6. Optional 5/10 s scalar knot random-walk variance scales with `q_b*Delta_t`; drift/temperature and held-out bias/yaw truth, observability/clock discontinuity, factor Jacobian and replacement of initial fused intervals; no unconstrained knots or double-counted raw gyro evidence.
7. OD-014 replay after trajectory/bias updates reproduces deterministic decisions/typed reasons, regenerates dependent scan membership and factor versions, and conditionally re-deskews/remaps; OD-012 serialization preserves provenance.
8. Benchmark across synthetic apartment/warehouse and genuine held-out reference where available: keyframes per metre/time, graph node/factor count, false LiDAR factor rate, ATE/RPE, rank and uncertainty consistency, latency/peak memory, Pi 5 resource budget and bias-knot benefit vs cost. No GitHub Actions.

**Milestone:** Initial C4 maps use OD-015 fixed-seed motion selector and **zero graph bias nodes**, with B23 keyframe/anchor tests alongside OD-013 B21. Experimental bias knots are **offline opt-in** after OD-014 baseline and B23's independent improvement tests, not required for initial C4 map completion. See [accepted OD-015](open-design-choices.md), [Architecture Section 7.1.1](architecture-specification.md), [benchmark B23](benchmarks-and-datasets.md).

### 10.12 Accepted OD-016 — Four-process supervisor, versioned seqpacket IPC and worker isolation

**Initial Raspberry Pi deployment MUST run four independent OS processes**: `acquisition`, `recorder`, `slam` (localizer + map/graph workers) and `web_lab`. Linux `AF_UNIX/SOCK_SEQPACKET` is the initial **native, ROS-free** IPC transport. Device-specific acquisition adapters, recorder, IPC and Linux scheduling belong to private app/infrastructure layers, not portable geometry, trajectory, matcher, graph or map interfaces. Standalone CLI tests and Docker offline replay MUST work without any IPC service or Web Lab.

| Process / channel | Implemented responsibility and mandatory failure policy |
| --- | --- |
| `acquisition` | Exclusive LD19 UART/MPU6050 I²C/wheel/Pico source ownership, packet validation, OD-003 timestamp normalization, clock sync, **independent bounded fan-out** to raw recording and SLAM; **no unbounded blocking send** |
| `recorder` | Ordered raw OD-023 MCAP, calibration/clock epoch/packet-gap/config records, segment/commit/`INCOMPLETE` status; disk full or input loss never silently reported as complete |
| `slam` | OD-005/004/006 input/trajectory, dedicated OD-007 Ceres match worker, OD-009 recovery, OD-011 cache worker, OD-013 graph worker and OD-012 map save worker; immutable versions and atomic graph correction |
| `web_lab` | Separate low-priority browser/UI process; rate-limited coalesced visualization and **validated asynchronous** commands only, no sensor devices or direct graph/map writes |

**IPC envelope and negotiation:** Define stable, bounded fixed-width native wire types with magic, major/minor protocol version, required feature flags, message kind/payload length, source ID, source/process run/session generation, per-source monotonic sequence number, OD-001 canonical **signed-64-bit nanosecond timestamp with clock domain/epoch**, original device/receive stamps as appropriate, calibration and config revision, gap/quality flags, optional command/ack ID. Include explicit byte order, length/record caps and checksum where required by a documented threat model (SOCK_SEQPACKET framing is not end-to-end disk integrity). Detect `MSG_TRUNC`, `EAGAIN`, peer close, malformed payload, unsupported required version and sequence gaps; never deserialize native C++ object memory, ROS/DDS/Ceres/GTSAM classes or raw platform pointers. Pin a schema compatibility matrix and test it on x86-64/ARM64 Docker.

**Transport is not durability:** `send`/`recv` boundaries preserve order only **per live connection**. Recorder enqueue/write acknowledgment must not be labeled durable until documented fsync/segment validation. Overflow, gap, recorder restart, disk pressure or corruption produces `LOSS_DETECTED` / `INCOMPLETE` or a controlled stopped segment. Preserve raw sensor and sync/calibration history necessary for OD-014 replay, including source ID/epoch and sequence gap locations. The producer cannot guarantee lossless data under unbounded disk stalls; **acquisition keeps timestamping**, logs dropped counts and never blocks indefinitely waiting for recorder, SLAM or UI.

**Worker and queue boundaries:** Use a **fixed configured number of workers** and per-edge queue **items+bytes** caps, max in-flight work, short bounded enqueue timeouts, host-steady deadlines and cancellation. Inside `slam` prioritize OD-006 odometry/input, sequential scan assembly/deskew and Ceres local matching; GTSAM graph updates, map cache builds, persistence and Web Lab telemetry are lower-priority bounded jobs. Coalesce outdated graph/search/build work; do not let a long GTSAM optimization, OD-011 EDT full rebuild or OD-012 disk serialization hold the acquisition/localizer work locks. All consumers pin immutable OD-011/013 snapshots and verify revision/config/clock epoch before publishing results. Graph changes only `map→odom` and submap poses, never jumps `odom→base_link`. No unbounded dynamic thread/task launch.

**Scheduling:** Start with ordinary Linux `SCHED_OTHER` and measured latency/thermal load. Only trial bounded `SCHED_FIFO` acquisition scheduling after reproducible Pi starvation tests and privileged/watchdog safeguards; never assume containers guarantee correct scheduler timing. If high-priority tasks monopolize CPU, lower priorities and recorder durability may fail; require explicit perf evidence. A sub-100 ms LiDAR scan period is a **load target**, not guaranteed real-time response.

**Restart and security:** Provide independent supervisor health checks, bounded restart policies, clean FD/socket ownership, signal shutdown, explicit reconnect/version negotiation, and source/clock generation changes. Acquisition restart begins a **new epoch/session**; recording logs discontinuity or starts a new incomplete/validated segment; SLAM restart starts a clearly identified new estimator session or loads **validated OD-012 compatible checkpoint**, not an old monotonic clock mapped blindly. Web Lab may be restarted independently. Secure Unix socket file permissions and credential/role validation; validate/authenticate Web Lab commands, do not expose a remotely reachable unprotected control interface. Ceres/GTSAM/ROS-independent domain types and typed error handling are required.

**Optional large immutable transfer:** Defer shared memory until measured overhead supports it. `memfd_create` plus `SCM_RIGHTS` fd passing can transfer **sealed read-only** occupancy/EDT/graph payloads across process boundaries, with bounded size, content hash/revision, lifetime/pin accounting, FD leak/size validation, and a socket control descriptor; keep this Linux-only adapter completely separate from public map APIs. Default control/sensor traffic stays bounded seqpacket.

**Required diagnostics:** Per-process health, run ID, peer connection/reconnect, source clock epoch, per-channel last sequence/received/dropped/truncated/retried bytes, queue max capacity/occupancy/lag, outstanding worker count and budget overshoot, raw recording **completeness and durable-through sequence**, last coherent map/graph revision, stale-result rejects, loss/stop/recovery reason, Pi CPU/peak RSS/FDs and thermal/p50/p95/p99 acquisition-to-pose latency. Send rate-limited aggregates to Web Lab and record provenance in raw MCAP/report metadata.

**Docker Catch2/CTest + four-process B24 validation:**

1. Protocol endian/golden bytes/version negotiation, ordered `SOCK_SEQPACKET` framing, exact boundaries, oversize/`MSG_TRUNC` and `EAGAIN`, peer crash, injected packet omissions and explicit per-source gaps.
2. Simulated ~10 Hz LD19, ~200 Hz IMU and encoder acquisition with recorder slow/disk full, SLAM Ceres stall, graph CPU spike, cache/persistence IO spike; producer input timing never waits indefinitely and loss is detected, not silently declared complete.
3. Recorder segment durable versus enqueue/write acknowledgment distinction, independent session restart, complete vs `INCOMPLETE` manifest flags, captured OD-002/003 sync/time and config changes, valid OD-014 replay input identity.
4. Graph/cache worker uses pinned immutable revisions while concurrently receiving new scans; stale graph optimization result is rejected; continuous odom preserved while map global pose changes.
5. Web Lab disconnected/slow subscribers and malformed/unauthorized control requests cannot hold sensor/SLAM workers, provoke unbounded memory or change calibration without validated acknowledgment.
6. Kill/restart each process in isolation; epoch/run ID checks, reconnect and full prior-session gap flags, bounded shutdown/drain. Reject untrusted old checkpoint/clock ID after process restart.
7. Optional sealed memfd/`SCM_RIGHTS` test for immutable revision and FD/byte lifetime if enabled; compare copying vs shared-memory CPU/RAM, never assume a benefit.
8. Profile ARM64 Pi 5 under CPU starvation/thermal/IO faults for p50/p95/p99 queue/latency, dropped events, storage durability, graph lag/quality and restart times. Execute **locally/in Docker, not GitHub Actions**.

**Milestone alignment:** Four-process native app scaffolding and versioned seqpacket IPC join **C1 acquisition/recorder and C1-W Web Lab** early; in-process bounded Ceres/graph/cache workers are added with C3/C4/C5/C6 functionality without changing the four top-level processes. **OD-017** still determines exact native CMake targets and executables; **OD-021** still sets numeric queue/backpressure/durability thresholds; **OD-023** MCAP channel schema; **OD-025** ROS adapter selection. Stage II performance optimization may add memfd snapshot transfer; optional **fifth graph process only on measured fault/latency need**. Reference [OD-016](open-design-choices.md), [Architecture Section 11](architecture-specification.md), [B24](benchmarks-and-datasets.md).

## 11. Change workflow and GitHub Actions policy

- Use small logical commits. After a meaningful implementation step or before long/risky validation, commit and push to the working feature branch.
- Do not force-push, rewrite shared history, or merge without explicit instruction.
- Changes SHOULD be proposed on a feature branch and reviewed through a pull request rather than committed directly to protected/default branches.
- Before committing, check for secrets, logs, generated maps, build outputs, caches, and machine-specific paths.
- GitHub Actions MUST NOT be used for execution unless expressly requested; prefer local Docker builds and tests.
- Any new GitHub Actions workflow MUST be **manual only**, with a `workflow_dispatch` trigger, unless automatic execution is explicitly requested.
- If a modified existing workflow has automatic triggers, do not add new automatic triggers or silently change existing ones; surface them for review.
- No Actions minutes should be consumed as a fallback for missing local hardware or tools.

## 12. Definition of done

A feature or milestone is ready for review only if it includes:

- Focused implementation with defined interfaces and ownership.
- Tests appropriate to the functionality, runnable inside the standard Docker test environment.
- Documentation of configuration, expected inputs/outputs, and failure modes.
- Repeatable build instructions and known dependency revisions.
- Evidence of relevant test results, with platform and architecture noted.
- No accidental host-only dependencies, undisclosed external downloads, or unbounded live-path CPU/memory use.
- A clear statement of remaining gaps, unvalidated hardware behaviors, and any deviations from this contract.

A **documentation-only change** need not create fake code or run unavailable application tests. It MUST pass structural/documentation checks and explicitly state that runtime validation is not applicable.

## 13. Initial implementation checkpoints

| Checkpoint | Deliverables | Acceptance |
| --- | --- | --- |
| C0 — Toolchain scaffold | CMake C++20, FetchContent Catch2/cxxopts, Docker stages and Compose, minimal executable + tests | Fresh-container build/test and runtime smoke check work without host C++ dependencies. |
| C1 — Acquisition/recorder/replay | **OD-016 separate acquisition and recorder processes**, versioned bounded seqpacket raw/SLAM fan-out; LD19 OD-003 timing, OD-005 assembly in SLAM, OD-023 MCAP writer/reader, cxxopts CLIs | Docker B24 verifies ordering/gaps, nonblocking acquisition and recorder incomplete-session semantics; synthetic parser/replay tests, no Pico required. |
| C1-P — Optional Pico hardware-near acquisition | Custom Pico SDK firmware, native framed Pi driver, time sync, measured IRQ/PIO alternatives | Firmware cross-built in Docker, protocol/clock tests pass in Docker, hardware measurements justify deployment, no micro-ROS. |
| C1-W — Web Lab foundation | **OD-016 independently supervised Web Lab process** with rate-limited native telemetry/validated commands; browser Canvas and C++ HTTP/JSON API, map/scan view and dataset timeline | Docker B24 UI slow/disconnect/command authorization never blocks acquisition or SLAM; headless and geometry tests pass. |
| C1-R — ROS-compatible native geometry | REP-103/105 framing, OccupancyGrid-style indexing/origin, rotated map import/export fixtures | No ROS install required for core geometry/PGM-YAML round-trip Catch2 tests. |
| C2 — Motion/deskew | **OD-006 minimal wheel/gyro integrator**, stationary gyro bias, replaceable verified kinematics and uncertainty; OD-004 SE(2) trajectory, midpoint deskew | Docker Catch2 proves startup stationary bias, rejected online bias under slip, covariance growth, unsupported kinematics, full-interval deskew and 10 ms gating. |
| C3 — Ceres frontend | OD-007 EDT/bilinear/per-beam Huber/prior/boundary, OD-008 LiDAR-only rank/covariance, **OD-011 atomic snapshot full-field baseline** | Docker tests include B16 rank gates and B19 snapshot/version/build correctness before Pi profiling. |
| C4 — GTSAM offline mapper | OD-010/011 submaps, OD-012 bundles, OD-013 Pose2 graph; **OD-015 motion/quality keyframes and independent anchors**; optional OD-014 iterative passes | Docker B18–B21 and **B23 keyframe/anchor tests** for baseline; B22 for later selective remap; zero graph bias knots initially. |
| C5 — Live localizer | **OD-016 in-process bounded ingest/Ceres workers** within SLAM; frozen occupancy/EDT, OD-007/008 tracking and OD-009 bounded async recovery | Docker B17/B24 verify no scanner/recorder starvation, stale correction rejection and loss/relock gating; Pi 5 CPU budgets measured. |
| C5-R — ROS 2 integration | Optional `fusion_slam_ros` adapter, standard topics/TF, Nav2/RViz launch, ROS bag and time support | Dedicated ROS 2 Docker tests prove interoperability; no ROS dependency added to core. |
| C6 — Online SLAM | OD-013 keyframe/submap graph within **OD-016 SLAM-process lower-priority bounded worker**, OD-019 future iSAM2, OD-010 submaps/loops | Docker B24 validates worker/version isolation; Pi 5 graph lag, latency and RAM measured, no separate fifth process by default. |

Do not create all scaffolding in advance merely to satisfy the directory diagram; implement each checkpoint end-to-end. OD-010 log-odds/submap correctness belongs to C4 mapping foundation, not a requirement to implement the GTSAM graph before mapping can be independently tested.


## 14. Web Lab: visualization, testing, and debugging contract

### 14.1 Motivation and reference architecture

The project MUST offer an **opt-in browser-based Web Lab** for inspecting, replaying, debugging, and comparing the real C++ SLAM implementation. It takes inspiration from the user's [Hybrid A* Planning Lab](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/README.md), as inspected at source commit [2832244](https://github.com/civcode/hybrid-astar-cpp/commit/2832244407c92c7ff3e393adc69fa75376814208).

The inspected reference deliberately uses **static HTML/CSS/JavaScript without a SPA framework or Node.js application server**, **Canvas for visualization**, and a small **native C++ executable exposing HTTP/JSON endpoints**. Its browser does not implement Hybrid A*; the C++ library computes the result. Its UI presents configuration, visual overlays, animation/scrubbing, and numerical diagnostics. Relevant source boundaries:

- [Web assets](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/web/index.html), [rendering and requests](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/web/app.js), [coordinate transforms](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/web/map_geometry.js), and [brush editing](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/web/map_editor.js).
- [HTTP server and CLI](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/main.cpp) (loopback default, explicit fixed assets/routes, request-size limits, structured error responses) and [planner orchestration](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo/demo.cpp).
- [Browser/HTTP integration tests](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/tests/demo_http_test.py) and [Docker Compose](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/compose.yaml).

**Transfer the architecture, not the planner-specific UI or its exact HTTP protocol.** The SLAM Web Lab requires timeline-based asynchronous data, multiple synchronized sensors, localization state, and long-running replay; those are new responsibilities rather than properties of the original demo. No code from the other repository is copied by this contract.

### 14.2 Architectural boundaries

The C++ algorithm libraries are **the sole source of truth** for timestamps, scan deskew, coordinate transforms, maps, pose estimation, solver results, and evaluation metrics. JavaScript MUST NOT implement a second SLAM pipeline or silently alter estimation results.

~~~text
                    Browser (static HTML / CSS / JS)
                controls + Canvas layers + timeline + metrics
                                 |
                          local HTTP / JSON
                                 |
                  C++ Web Lab adapter / controller
                 request validation + lifecycle + DTOs
                         /              \
                        /                \
       Dataset replay / experiment       Live read-only snapshots
              runner                           from localizer
                 |                                  |
     Shared sensor, motion, deskew, Ceres, GTSAM, mapping core
                 |
       Raw MCAP, configs, maps, reports, metrics
~~~

- A thin `web_lab` executable/service MUST serve fixed static assets and a **versioned API**. It MUST use the same core interfaces as headless CLI applications.
- The adapter MAY live in the same process as the offline experiment runner, or communicate with a separate local worker, but long-running work MUST NOT block HTTP responsiveness or sensor acquisition.
- Headless CLI and dataset replay MUST remain fully functional when Web Lab is disabled or unavailable.
- The browser is a **debugging/experimentation client**, not a control-loop dependency, safety interface, or public production endpoint.
- The Web Lab MUST NOT write to or mutate live robot maps, SLAM state, hardware configuration, or sensor drivers by default. Explicit editing applies only to a copied, isolated experimental state.

### 14.3 First-class user workflows

The Web Lab MUST support the following workflows incrementally:

| Workflow | Visible data and controls | Intended phase |
| --- | --- | --- |
| **Offline replay** | Dataset/session selection, play/pause, step by scan, seek by timestamp, playback rate, scan index, deterministic rerun | First Web Lab release |
| **Scan debugging** | Raw vs midpoint-deskewed LiDAR beams, **partial/complete flag, angular coverage and missing-sector overlay**, valid/invalid/no-return counts, eligibility reasons, midpoint/scan acquisition interval and trajectory status | First release |
| **Pose and geometry** | Wheel/gyro prediction, scan-match pose, optimized trajectory, map-to-odom correction, optional uncertainty ellipses | First release / C3 |
| **Mapping inspection** | OD-010 submaps/known cells, OD-011 snapshot/cache revisions, **OD-012 bundle generation/size/format/required checksum, load/cache readiness, recovered prior generation and map/checkpoint type** | C3–C4 |
| **Localization inspection** | OD-009 mode/window, search progress/budget/pruned nodes, distinct coarse hypotheses and OD-007/008 refinement outcomes; TRACKING/UNCERTAIN/LOST, map relock, covariance and last valid global fix | C5 |
| **Solver diagnostics** | OD-007 geometric/prior/boundary costs, **OD-008 information rank, normalized eigenvalues/eigen-directions, full/subspace covariance status, projected/rejected reason and calibration ID**, EDT version and Ceres iterations; gyro bias/slip and graph factors | C2–C5 |
| **Experiments** | Algorithm toggles (e.g., deskew variants), seed, solver budget, scenario parameters; compare saved runs | C2–C5 |
| **Live monitor** | Timestamped pose/scan snapshots, packet loss, clock offset, FIFO/queue age, CPU and latency | C5–C6 |

The UI SHOULD have a clear left-side experiment/control panel, a prominent map/scan Canvas, timeline/playback controls, synchronized diagnostic charts or readouts, and an explicit connection/replay/quality status. Interactive layers MUST be independently toggleable to avoid obscuring the scan.

**Critical distinction:** The replay timeline uses **sensor acquisition time**, not when HTTP responses arrive. Seeking MUST restore the correct deterministic pipeline state using replay/reset and optional checkpoints. It MUST NOT pretend that independently interpolated poses are verified solver outputs.

### 14.4 Map and coordinate visualization

- Render a 2D Canvas, using device-pixel-ratio-aware scaling and resize handling.
- Explicitly convert **world ↔ map-local ↔ grid cell ↔ canvas pixel** coordinates. Map origin translation, yaw, resolution, row orientation, and unknown occupancy MUST be honored. Preserve the native ROS-compatible `OccupancyGrid` indexing and origin convention specified in Section 15; reuse tested geometry functions shared with the core.
- Provide zoom, pan, fit-to-map, grid/scale legend, pointer readout in metres, and optional heading arrows. Inputs to the estimator use documented SI units and radians; if UI controls display degrees, convert at the adapter boundary and label visibly.
- Provide separately color-coded/toggleable views for raw beams, deskewed beams, scan matching, sensor prediction, global optimized path, live pose, submaps, loop constraints, and candidate relocalization hypotheses.
- Large maps MUST NOT be returned as full dense JSON on every pose update. Send immutable map metadata and occupancy representation (tiles, run-length encoding, or compact binary payload) once per map revision, then small timestamped dynamic overlays.
- Rendering MUST remain responsive when the backend is computing; throttle/coalesce visual updates and cap history buffer sizes. A requested higher visualization rate must never drive the core estimator cadence.
- Avoid implying visual accuracy beyond the actual timestamp, covariance, and scan-matching quality. Highlight missing/stale data instead of extrapolating invisibly.

### 14.5 Experimental isolation and revision consistency

The source Planning Lab's most important transferable reliability pattern is **immutable map snapshots plus request-local changes**. Its browser invalidates stale output after map edits and checks a revision number before accepting a result; SLAM MUST extend this to all relevant inputs.

Each run MUST have a unique **run ID**, and every result MUST identify relevant immutable inputs:

- Dataset ID/hash, acquisition timestamp or processed scan index.
- Map ID/revision, calibration revision, configuration/parameter hash, and algorithm/build version.
- Frame ID, units, request ID, status/quality, and solver timing where applicable.

User-side edits (experimental obstacles, sensor masks, parameter changes, artificial clock offsets) MUST create an isolated **derived run**, never modify the source MCAP or the production map. A display must NOT combine scan points, pose estimates, and maps from incompatible revisions without an explicit compare mode.

The browser MUST cancel superseded requests and/or reject responses whose request generation, run ID, or revision no longer matches the selected experiment. It MUST reset invalidated results when map or config changes; completion order must not decide what is shown. Exported experiment reports MUST include the effective parameters and provenance required to reproduce the run.

### 14.6 Proposed API and transport

Start with a minimal **HTTP/JSON contract** to keep build and runtime dependencies low. The following paths are **proposed, not implemented**; the initial scaffold MUST document its real routes and return schemas. All APIs live under `/api/v1/`.

| Method / path | Responsibility |
| --- | --- |
| `GET /api/v1/capabilities` | Build features, modes, available sensors, data formats, supported visualization layers. |
| `GET /api/v1/status` | Health, current run ID, state, backend progress, latest sensor timestamp, dropped packets. |
| `GET /api/v1/datasets` | Enumerate **allowlisted / mounted** datasets; no arbitrary server-side path access. |
| `POST /api/v1/replay` | Start/reset replay from a selected dataset ID and validated experiment config; return run ID. |
| `POST /api/v1/replay/control` | Play/pause, bounded speed, step by scan, or seek; must include expected run ID. |
| `GET /api/v1/snapshot?run_id=...&at=...` | Consistent map/scan/pose/diagnostic snapshot at acquisition time; may return not-ready. |
| `GET /api/v1/maps/{map_id}/metadata` | Occupancy origin, resolution, dimensions, map revision, available tile/field layers. |
| `GET /api/v1/maps/{map_id}/tiles/...` | Bounded static tiles or compact occupancy data keyed by revision. |
| `GET /api/v1/runs/{run_id}/metrics` | Estimator accuracy (where ground truth exists), latency, convergence and diagnostics. |
| `GET /api/v1/runs/{run_id}/export` | Download reproducible experiment summary with inputs, parameters and metrics. |

Do not implement every endpoint in the first milestone; define schemas and version them before clients depend on them. The contract MUST specify typed error responses, limits, timestamp units, frame semantics, status enumerations, empty/missing data behavior, and incompatible-version handling.

- **Replay/experiment computation:** asynchronous background worker with bounded queue and cancellable job IDs; polling for snapshots/status is sufficient initially.
- **Live telemetry:** start with rate-limited snapshot polling; if measurements justify it, add **Server-Sent Events or WebSocket** for one-way streaming, with bounded buffers, reconnection and backpressure. Do not assume the original Planning Lab supports streaming.
- Never use request arrival order as the ordering of sensor measurements or results.
- Implement JSON serialization and validation server-side; do not send opaque internal Eigen/Ceres/GTSAM object layouts directly to JS.
- Preserve a CLI / headless output with equivalent metrics; browser-only behavior is not acceptable.

### 14.7 Security and Docker networking

The original demo serves allowlisted files, defaults to IPv4 loopback, and uses bounded requests. The SLAM Web Lab MUST adopt equally constrained defaults:

1. Default bind: **127.0.0.1** in native mode; when binding to `0.0.0.0` inside a Docker container is required, Compose MUST publish only `127.0.0.1:HOST_PORT:CONTAINER_PORT` unless explicitly approved.
2. No public internet exposure, production authentication assumptions, generic filesystem browser, arbitrary dataset path URL, command execution endpoint, or unrestricted map upload.
3. Only fixed static assets and documented API routes may be served. Validate methods, MIME types, duplicated/unknown parameters, numeric bounds, payload length, and effective solver/scan budgets.
4. Set restrictive response headers including `X-Content-Type-Options: nosniff` and an appropriate Content Security Policy. Do not permit wildcard CORS by default.
5. Use read-only mounts for source datasets and immutable map bundles, and a distinct restricted writable directory for derived experiment artifacts.
6. Containers MUST run with least privilege; browser routes must not expose raw sensor devices. Hardware access belongs only to acquisition services.
7. Limits MUST bound concurrent jobs, CPU/memory, result history, HTTP requests, search ranges, and export size. Long runs must expose progress and cancellation without blocking the HTTP service.
8. A web debugging server MUST NOT be positioned as an authenticated remote robot management service. Remote access requires a separately reviewed design (e.g., SSH port forwarding).

The Compose scaffold SHOULD provide an opt-in `web-lab` service and documented loopback port (e.g., 8765). The source project uses this port, but the final SLAM port is an implementation detail. All backend execution and tests MUST remain containerized.

### 14.8 Testing and acceptance of the Web Lab

Web Lab tests MUST execute within the Docker `test` environment and be registered with CTest (including non-C++ checks as necessary). Keep these test layers:

- **Pure geometry tests:** map/world/canvas conversion (including rotated origins), zoom/pan, angle and unit conversion; browser JavaScript helpers MUST have deterministic tests. A headless JS runtime MAY be included in the **test image only**, not required in the production server.
- **Core/adapter tests with Catch2:** snapshot serialization, schema versions, timestamp ordering, run/revision consistency, parameter validation, and safety limits.
- **Real HTTP integration tests:** launch the actual local C++ server and verify assets, capabilities, read-only data access, replay controls, invalid methods/parameters, malformed/oversized requests, and typed errors.
- **Replay-based tests:** deterministic pause, one-scan stepping, seek/reset, raw-vs-deskew alignment, map provenance, and refusal to present stale/wrong-revision results.
- **Browser smoke tests:** UI boots, backend unavailable message is visible, canvas shows a known fixture, playback changes the displayed acquisition time, export includes provenance; headless-browser tests are encouraged when affordable.
- **Performance tests:** benchmark web-on versus web-off latency/CPU, payload sizes, large maps, and slow clients; enable a limited publish/snapshot rate so the Pi 5 real-time path is not starved.

Initial Web Lab acceptance requires a **real C++-backed** scan/map viewer, **deterministic offline replay** with pause/step/seek, correct origin/coordinate visualization, exported diagnostic JSON, reproducible run IDs and revisions, and passing HTTP/geometry tests in Docker. Later milestones add full solver and live diagnostics.

The Web Lab MUST remain optional at compile-time and run-time; a headless SLAM build MUST NOT require a browser, JS/Node runtime, or the local HTTP debug server.


## 15. ROS-compatible conventions and optional ROS 2 integration

### 15.1 Contract and architectural boundary

**ROS 2 conventions are normative for the portable core, but ROS 2 libraries are not.** Follow [REP-103](https://reps.openrobotics.org/rep-0103/) (SI units and axes), [REP-105](https://reps.openrobotics.org/rep-0105/) (mobile robot frame semantics), and ROS-compatible occupancy-map representations wherever applicable. A deliberate deviation MUST be recorded and documented with its mapping to ROS 2.

Native types (e.g., `Pose2`, `Transform2`, `TimedScan`, `ImuSample`, `WheelDelta`, `OccupancyGrid`, `PoseEstimate`) MUST belong to the standalone C++20 libraries and use Eigen/standard C++ representations. They MUST NOT expose or inherit from ROS message types, include ROS headers, or require `ament`, `rclcpp`, `tf2`, `rosidl`, DDS, or ROS environment variables.

Provide an **optional `fusion_slam_ros` adapter package**, built and tested independently using `ament_cmake` / `colcon` against an exported/installable CMake core. The adapter owns publishers, subscriptions, ROS messages, TF, clock conversion, parameters, QoS, and launch descriptions; it MUST call the real native estimator and MUST NOT reimplement scan matching or sensor fusion.

~~~text
 Native sensors / MCAP replay                 ROS 2 topics / rosbag2
            |                                         |
            v                                         v
   ROS-independent C++20 core  <--- converters ---  fusion_slam_ros
  time sync, motion, deskew,                          |
  Ceres, GTSAM, mapping                               +--> /tf, /tf_static
            |                                         +--> /odom, /map, /scan
  MapBundle, pose + diagnostics                       +--> RViz / Nav2
            |
    Headless CLI / Web Lab (no ROS required)
~~~

A ROS adapter MUST NOT inject fused odometry back as an **independent measurement** alongside the same source encoder/IMU data. Explicitly select which node owns each sensor source, published odometry, and TF link. A native/headless installation MUST continue to work with **zero ROS dependencies**.

### 15.2 Coordinate frame and transform ownership

- **REP-103:** Right-handed coordinates; robot **+X forward, +Y left, +Z up**, yaw positive CCW about +Z. Length in metres, angles in radians, angular rates in rad/s, linear velocity in m/s, and acceleration in m/s². Device-native LD19 clockwise angle conventions must be corrected in its driver/adapter. If the Web Lab displays degrees, conversion and labels MUST be explicit.
- **REP-105:** The standard chain is **`map -> odom -> base_link -> {laser, imu_link}`**. Frame names MAY be remapped/namespaced, but their meanings MUST remain consistent. Use calibrated `base_link` to sensor frames.
- **`odom -> base_link`** must be continuously updated by **one** local odometry authority. It should not jump after a scan-map correction or global graph optimization.
- **`map -> odom`** must be updated by **one** mapping/localization authority when enabled. If an external AMCL/other localizer owns that transform, this project's publication of `map -> odom` MUST be disabled. Do not publish a competing direct `map -> base_link` TF.
- Static sensor extrinsics come from URDF/`robot_state_publisher` or a single `static_transform_publisher` authority, not both simultaneously.
- To compute global correction use `T_map_odom = T_map_base * inverse(T_odom_base)` at consistent timestamps. The adapter must handle TF timeout, stale data, frame lookup errors, and extrapolation limits; it must not present invalid TF as a good localization fix.
- Coordinate transforms MUST document source/target frame multiplication order; covariance MUST be expressed in a stated frame and converted consistently. Preserve the same spatial result in native geometry, Web Lab and ROS TF.

### 15.3 Occupancy grid, map origin, and Nav2 map files

The native occupancy grid MUST convert to [`nav_msgs/msg/OccupancyGrid`](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg) without axis flips or changing its world geometry.

| Field / convention | Requirement |
| --- | --- |
| Indexing | **Row-major starting at (x=0,y=0)**, `index = y * width + x`. |
| Axis meaning | Grid columns grow in +X; rows grow in +Y in the **grid's local frame**. |
| Cell value | ROS export: signed `int8`, **-1 unknown**, 0 free, 100 occupied; probabilities 1–99 allowed where supported. |
| Resolution | Positive finite metres/cell. |
| Origin | `MapMetaData.origin` is the world pose of the **lower-left corner of cell (0,0)**, not its centre. Include yaw as a normalized ROS quaternion. |
| World coordinates | Centre of cell (i,j) is `T_map_grid * [(i+0.5)*resolution, (j+0.5)*resolution]`. |
| Image rows | PGM / PNG top-down raster rows MUST be converted explicitly; do not interpret image row 0 as native grid y=0. |
| Metadata | Width, height, resolution, origin, frame ID, map revision, and map threshold/unknown policy. |

- Support **Nav2 map-server-style PGM/YAML** alongside the richer native map bundle. Standard fields: `image`, `resolution`, `origin: [x,y,yaw]` (yaw in radians), `negate`, `occupied_thresh`, `free_thresh`, `mode` (initially `trinary`). Unsupported formats or modes MUST fail explicitly.
- Map loading and saving MUST preserve the origin including nonzero yaw, correct occupancy polarity, unknown cells, dimensions, and metric scale. Trinary export loses intermediate occupancy probability by design; document thresholds and loss of precision.
- The map-adapter and Web Lab MUST use the **same native world/grid/canvas transformations** and pass round-trip tests for shifted and rotated origins.
- The map message's `header.frame_id` defaults to `map`. Its timestamp and `map_load_time` policies must be explicit and tested. Localization maps remain read-only.

### 15.4 Conventional ROS 2 messages and topics

Prefer standard message definitions and **remappable** names; implement custom messages only for information the standard messages cannot faithfully represent.

| Native quantity | ROS 2 message and suggested topic | Adapter requirements |
| --- | --- | --- |
| Planar scan | `sensor_msgs/msg/LaserScan` on `/scan` | Only when samples support uniform angular and time steps. Header stamp is **first beam acquisition time**, +Z CCW, zero angle along +X; radians, metres, `time_increment` and `scan_time` in seconds. |
| Raw LD19 or nonuniform beam timing | `sensor_msgs/msg/PointCloud2` with documented per-point time, or explicit raw packet interface | Retain original timing/order; do not fabricate uniform timing when not valid. Raw packet storage is still required for deterministic offline replay. |
| IMU | `sensor_msgs/msg/Imu` on `/imu/data_raw` | Angular velocity rad/s, acceleration m/s², correct `frame_id`/covariances; if orientation is unavailable, set `orientation_covariance[0] = -1`. Gyro-integrated yaw is not an absolute orientation measurement. |
| Local odometry | `nav_msgs/msg/Odometry` on `/odom` | `header.frame_id = odom`, `child_frame_id = base_link`, pose/twist covariances meaningful, continuous estimate. |
| Global localization | `geometry_msgs/msg/PoseWithCovarianceStamped` on `/localization/pose` | `header.frame_id = map`; source timestamp, quality/state published separately. |
| Map | `nav_msgs/msg/OccupancyGrid` on `/map` | Correct metadata, origin quaternion, occupancy indexing and unknown semantics. |
| Pose history (optional) | `nav_msgs/msg/Path` on `/trajectory` | Bounded, frame-consistent history; do not publish unbounded growth at IMU rate. |
| TF | `tf2_msgs/msg/TFMessage` via TF broadcasters | Follow Section 15.2: one broadcaster per dynamic edge, static extrinsics on `/tf_static`. |

A **deskewed** scan may be published separately for diagnostics; its timestamp/frame MUST describe the reference pose to which points were transformed. Do not silently label deskewed beam points as raw `LaserScan` rays when it violates uniform-ray assumptions.

### 15.5 Time, QoS, lifecycle and integration behavior

- The native estimator MUST use monotonic acquisition timestamps for integration and preserve each clock's origin and synchronization uncertainty. ROS stamps are **adapter-side conversions** using a documented and observable time mapping.
- Honor ROS `use_sim_time` and `/clock` during bag replay/simulation. On backward time jumps, recording seeks, or clock resets, reset/restore state and TF buffers consistently; do not continue integrating across a discontinuity.
- Record whether a message timestamp is a hardware acquisition instant, reconstructed beam time, ROS receive time, or conversion estimate. Reject unsynchronized data rather than silently pretending precise alignment.
- Start with conventional ROS QoS profiles: sensor-data-compatible best-effort for sensors where appropriate, configurable odometry QoS, and reliable **transient-local** map delivery for late subscribers. Expose QoS via ROS parameters and test common Nav2/RViz subscription compatibility.
- ROS-facing applications SHOULD provide remappable topics, frame names, params, launch files, and useful diagnostics. Consider managed lifecycle behavior for Nav2 interoperability where appropriate; it is not part of the core API.
- Support rosbag2 / MCAP input and output in the adapter. Headless native replay remains available without ROS 2.
- Only enable ROS-specific dependencies in the dedicated Docker image / Compose profile. Record the supported ROS 2 distribution explicitly when implementing the adapter. Avoid globally changing core compiler flags or introducing ROS transitive dependencies.
- Any direct or transitive **Boost** component introduced by ROS integration still falls under the **per-case Boost review in Section 4.3**; using an existing ROS package is not blanket approval to add Boost to the native core.

### 15.6 ROS interoperability and no-dependency acceptance

The ROS 2 integration is not considered complete until containerized tests verify:

1. **ROS-free core build** succeeds when ROS packages are absent; its exported CMake targets expose no ROS includes or linked dependencies.
2. **Independent adapter build** succeeds in a declared ROS 2 Docker environment using `ament_cmake` / `colcon` and the separately installed core.
3. **Map round trips:** Native occupancy grid -> ROS `OccupancyGrid` -> native and native -> PGM/YAML -> native preserve frame, resolution, occupied/free/unknown, translated and **rotated** map origins, and cell indexing.
4. **Message conversion:** `LaserScan` timestamps/angles/sign/units, IMU orientation-unavailable semantics, odometry frame IDs, covariances, and yaw/quaternion conversion round-trip against known fixtures.
5. **TF correctness:** One publisher per edge; static extrinsics, continuous odom, and global corrections produce the correct transform chain. Exercise missing, stale, and conflicting TF authority cases.
6. **Time semantics:** Test bag replay with `use_sim_time`, delayed sensors, timestamp offsets, clock reset/backward seek, and correct snapshot time alignment.
7. **Nav2/RViz compatibility:** A documented Docker launch/fixture demonstrates map and scan visualization and a consuming navigation system can resolve `map -> odom -> base_link -> laser` without manual data conversion.
8. **Resource isolation:** ROS adapter/Web Lab may add overhead but must not starve onboard sensor acquisition or the Ceres tracking deadlines.

ROS 2 adapter tests are **optional to the core Docker test profile**, but mandatory when changes touch the ROS adapter; all such tests MUST run inside the dedicated Docker service. No GitHub Actions runs are required or permitted by default.

## 16. Reference documentation

- [CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html)
- [Catch2 CMake integration](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md)
- [cxxopts upstream project](https://github.com/jarro2783/cxxopts)
- [Docker multi-stage builds](https://docs.docker.com/build/building/multi-stage/)
- [Docker Compose](https://docs.docker.com/compose/)
- [Ceres Solver](https://ceres-solver.readthedocs.io/)
- [GTSAM](https://gtsam.org/)
- [MCAP](https://mcap.dev/)
- [ROS REP-103: Units and coordinate conventions](https://reps.openrobotics.org/rep-0103/)
- [ROS REP-105: Coordinate frames for mobile platforms](https://reps.openrobotics.org/rep-0105/)
- [ROS 2 OccupancyGrid definition](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)
- [ROS 2 LaserScan definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)
- [Nav2 transform configuration guidance](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [Hybrid A* Planning Lab source, inspected branch](https://github.com/civcode/hybrid-astar-cpp/tree/feature/nav2-full-warehouse-margin-pipeline-benchmark/apps/demo)
- [Hybrid A* Web Lab HTTP integration test reference](https://github.com/civcode/hybrid-astar-cpp/blob/feature/nav2-full-warehouse-margin-pipeline-benchmark/tests/demo_http_test.py)
