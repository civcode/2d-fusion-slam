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
| ROS | Optional adapter; no ROS dependency in the core algorithms. |
| GitHub Actions | No workflows required by default; **never run Actions** without explicit authorization. |

All new production code and test code MUST use C++20. A 3rd-party dependency MAY use an older language standard internally, provided it does not force a lower standard or extensions on project targets.

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
  apps/
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
3. Estimation logic MUST NOT depend directly on ROS messages, device files, CLI parsing, or Docker.
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

## 7. Testing contract

Tests MUST use **Catch2** and be registered with **CTest**. All test execution, including sanitizer runs and offline replay regression suites, MUST happen within Docker.

### 7.1 Test categories

| Category | Must cover | Dependencies |
| --- | --- | --- |
| Unit | SE(2), math, timestamps, LD19 CRC/frames, gyro integration, kinematics, CLI validation | Synthetic/no hardware |
| Integration | Ceres scan matching, GTSAM factor Jacobians, map serialization, sensor-stream synchronization | Docker image libs |
| Regression | Deterministic offline replay, stable outputs, previously fixed bugs | Versioned datasets |
| Performance | Per-scan latency, CPU, memory, cache allocations, Pi thermal behavior | Measured on target hardware |
| Hardware-in-loop | UART acquisition, timing offsets, loss/reconnection, wheel/IMU integration | Opt-in device-access container |

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
4. Static analysis (`clang-tidy` when configured) and AddressSanitizer/UndefinedBehaviorSanitizer for supported test configurations.
5. Deterministic replay/integration checks when the change touches timestamps, sensor models, optimization, or serialization.
6. ARM64 smoke build/test for release-affecting changes, with a clearly stated exception if hardware is unavailable.
7. Updated docs, configuration examples, and test expectations for externally visible behavior changes.

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
| C1 — Recorder/replay | LD19 parser, timestamp schema, MCAP writer/reader, cxxopts CLIs | Synthetic packet suite and deterministic replay pass in test container. |
| C2 — Motion/deskew | Wheel/gyro estimator, yaw bias, continuous SE(2) interpolation, full scan deskew | Numerical and synthetic moving-scan tests pass. |
| C3 — Ceres frontend | Grid/submap representation, distance field, scan matcher | Synthetic alignment and degeneracy tests pass; time budget benchmarked on Pi 5. |
| C4 — GTSAM offline mapper | Bias/wheel/LiDAR graph constraints, loops, batch optimization, map bundle | Offline replay produces evaluated, versioned map. |
| C5 — Live localizer | Read-only map loader, Ceres tracking, localization state machine/recovery | Containerized onboard tracking and loss recovery demonstrated. |
| C6 — Online SLAM | Asynchronous incremental backend, submap lifecycle, loop closure | Real-time latency and resource limits profiled on Pi 5. |

Do not create all scaffolding in advance merely to satisfy the directory diagram; implement each checkpoint end-to-end.

## 14. Reference documentation

- [CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html)
- [Catch2 CMake integration](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md)
- [cxxopts upstream project](https://github.com/jarro2783/cxxopts)
- [Docker multi-stage builds](https://docs.docker.com/build/building/multi-stage/)
- [Docker Compose](https://docs.docker.com/compose/)
- [Ceres Solver](https://ceres-solver.readthedocs.io/)
- [GTSAM](https://gtsam.org/)
- [MCAP](https://mcap.dev/)
