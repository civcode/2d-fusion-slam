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

## 3. Project structure and module boundaries

The implementation SHOULD evolve toward:

~~~text
2d-fusion-slam/
  CMakeLists.txt
  CMakePresets.json
  cmake/
  docker/
    Dockerfile
    compose.yaml
  include/fusion_slam/
  src/
    core/
    sensors/
    time_sync/
    calibration/
    motion/
    deskew/
    mapping/
    frontend_ceres/
    backend_gtsam/
    localization/
    io/
  ros2/
    fusion_slam_ros/  # optional ROS 2 adapter package
  firmware/
    pico/             # optional RP2040 custom acquisition firmware, when implemented
  apps/
    web_lab/
      web/          # static HTML/CSS/JavaScript
    recorder/
    offline_mapper/
    live_localizer/
    online_slam/
    replay_evaluator/
  tests/
    unit/
    integration/
    regression/
    performance/
  configs/
  docs/
    technical-specification.md
    implementation-contract.md
~~~

This is a target layout, not a requirement to create empty placeholder directories.

1. `core` owns geometry, timestamps, frame conventions, data types, and math helpers.
2. Hardware drivers MUST expose timestamped measurements and diagnostics through small interfaces; they MUST NOT depend on Ceres or GTSAM.
3. Estimation logic MUST NOT depend directly on ROS messages, device files, CLI parsing, or Docker. In particular, `rclcpp`, `tf2`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, and `rosidl` types and includes are prohibited from the core public API and core link interface; ROS packages belong in the optional adapter.
4. Ceres-specific types MUST remain inside scan-matching/optimization adapters; GTSAM-specific types MUST remain inside graph adapters.
5. Frontend inputs and outputs MUST carry explicit timestamps, coordinate frames, and meaningful validity/quality information.
6. Applications MUST be thin orchestration layers; their CLI parsing uses cxxopts.
7. Libraries MUST support use in offline replay without connected physical hardware.
8. Prefer small, independently testable targets over one large executable or singleton state container.

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

- Prefer conventional library target names such as `fusion_slam_core`; use a namespaced alias such as `fusion_slam::core` for consumers.
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
  add_executable(fusion_slam_unit_tests tests/unit/core_test.cpp)
  target_link_libraries(fusion_slam_unit_tests
    PRIVATE fusion_slam_core Catch2::Catch2WithMain)
  add_test(NAME fusion_slam_unit_tests COMMAND fusion_slam_unit_tests)
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

target_link_libraries(fusion_slam_recorder PRIVATE cxxopts::cxxopts)
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
| Unit | SE(2) exponential/midpoint/coverage, 10 ms extrapolation, LD19 CRC/timing, OD-005 seam-based partial scans, missing angular wedges, safe invalid/no-return rays, gyro/kinematics and CLI validation | Synthetic/no hardware |
| Integration | Ceres scan matching, GTSAM factor Jacobians, map serialization, sensor-stream synchronization | Docker image libs |
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
- Map bundles MUST be versioned and include occupancy/submap data, frame conventions, calibration identifier, and generation metadata.
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
| C1 — Recorder/replay | LD19 parser, OD-003 beam timing, **OD-005 fixed-seam partial scan assembler**, timestamp schema, MCAP writer/reader, cxxopts CLIs; Pi-only capture | Docker synthetic packets/replay verify circular coverage and zero free-space from invalid/missing beams; no Pico required. |
| C1-P — Optional Pico hardware-near acquisition | Custom Pico SDK firmware, native framed Pi driver, time sync, measured IRQ/PIO alternatives | Firmware cross-built in Docker, protocol/clock tests pass in Docker, hardware measurements justify deployment, no micro-ROS. |
| C1-W — Web Lab foundation | Browser Canvas view, local C++ HTTP/JSON API, map/scan view, dataset timeline, status/metrics, Compose service | Real backend data is rendered; headless/API and geometry tests pass in containers; no JavaScript SLAM implementation. |
| C1-R — ROS-compatible native geometry | REP-103/105 framing, OccupancyGrid-style indexing/origin, rotated map import/export fixtures | No ROS install required for core geometry/PGM-YAML round-trip Catch2 tests. |
| C2 — Motion/deskew | Wheel/gyro estimator, yaw bias, piecewise-constant body-twist SE(2) exponential, timestamped trajectory provider, midpoint deskew | Complete-interpolation mapping gate, 10 ms bounded/gated extrapolation and synthetic arc/lever-arm tests pass in Docker. |
| C3 — Ceres frontend | Grid/submap representation, distance field, scan matcher | Synthetic alignment and degeneracy tests pass; time budget benchmarked on Pi 5. |
| C4 — GTSAM offline mapper | Bias/wheel/LiDAR graph constraints, loops, batch optimization, map bundle | Offline replay produces evaluated, versioned map. |
| C5 — Live localizer | Read-only map loader, Ceres tracking, localization state machine/recovery | Containerized onboard tracking and loss recovery demonstrated. |
| C5-R — ROS 2 integration | Optional `fusion_slam_ros` adapter, standard topics/TF, Nav2/RViz launch, ROS bag and time support | Dedicated ROS 2 Docker tests prove interoperability; no ROS dependency added to core. |
| C6 — Online SLAM | Asynchronous incremental backend, submap lifecycle, loop closure | Real-time latency and resource limits profiled on Pi 5. |

Do not create all scaffolding in advance merely to satisfy the directory diagram; implement each checkpoint end-to-end.


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
| **Mapping inspection** | Occupancy/unknown/free cells, active/frozen submaps, scan-to-submap associations, loop-closure links | C3–C4 |
| **Localization inspection** | TRACKING/UNCERTAIN/LOST, match score, covariance/degeneracy, recovery candidates, age of last good fix | C5 |
| **Solver diagnostics** | Ceres iteration counts, residual distributions, inlier/overlap ratio; GTSAM factors, bias estimates, graph errors | C3–C5 |
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
