# Docker development with Ubuntu APT dependencies

**Goal:** Build and test the C++ SLAM project without installing Ceres,
GTSAM, Catch2, Eigen3, CMake or a compiler on the host. Docker is the only
required host tool. **GitHub Actions stays manual-only**.

## Why use APT instead of the dependency Release?

APT is the better default *inside* an Ubuntu Docker image. It installs
the libraries and their transitive system dependencies together, uses
standard CMake package locations, and avoids loading the separately
compiled Debian 13/GCC 14 development bundle into Ubuntu 24.04.

The Ubuntu 24.04 (Noble) package versions are:

| Dependency | Ubuntu 24.04 APT version | Existing development baseline |
| --- | --- | --- |
| Ceres | 2.2.x (`libceres-dev`) | 2.2.0 |
| GTSAM | 4.2.x (`libgtsam-dev`) | 4.3.0 |
| Catch2 | 3.4.x (`catch2`) | 3.16.1 |
| Eigen3 | Ubuntu's `libeigen3-dev` | 3.4.0 |

Package references:
[Ubuntu GTSAM](https://packages.ubuntu.com/noble/libgtsam-dev),
[Ubuntu Ceres](https://packages.ubuntu.com/noble/libceres-dev),
[Ubuntu Catch2](https://packages.ubuntu.com/noble/catch2).

**The Docker/apt build is a compatibility matrix, not a replacement for
the original-version regression.** The native default still expects
GTSAM 4.3 and Catch2 3.16. The older distribution versions may expose
differences; the complete Docker CTest run will detect failures and must
pass before claiming compatibility.

## Build/test commands

Run these from a clone of `civcode/2d-fusion-slam`. On a Linux host
(or inside WSL2 with Docker Desktop/Engine):

~~~sh
# First image build: installs packages only, not the project source
bash scripts/docker-build-test.sh image

# Configure and build the complete project inside the container
bash scripts/docker-build-test.sh build

# Configure, build and run the complete CTest suite
bash scripts/docker-build-test.sh test

# Optional interactive shell with repo read-only and build outputs mounted
bash scripts/docker-build-test.sh shell
~~~

With no argument the script defaults to **test**:

~~~sh
bash scripts/docker-build-test.sh
~~~

The runner automatically builds the image if it does not exist. It uses
`docker/Dockerfile.apt` and tags the image
`2d-fusion-slam:ubuntu24.04-apt`. The repository is mounted **read-only**
under `/workspace`. CMake/Ninja outputs are written to the host's
Git-ignored `.build/docker-ubuntu24.04-apt/` directory via `/build`,
so reruns reuse compiled project files. Docker runs with the host user's
UID/GID on Linux to avoid creating root-owned build files.

Adjust parallelism or image name without editing source:

~~~sh
SLAM_DOCKER_JOBS=4 bash scripts/docker-build-test.sh test
SLAM_DOCKER_IMAGE=slam-local:noble bash scripts/docker-build-test.sh image
~~~

A separate build directory is important: do not reuse the native
Debian 13/GCC 14 build cache inside the Ubuntu container.

## Explicit CMake compatibility profile

The runner selects these CMake options:

~~~sh
cmake -S /workspace -B /build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DSLAM_APT_DEPENDENCIES=ON \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON \
  -DSLAM_ENABLE_CATCH2=ON -DSLAM_REQUIRE_SYSTEM_CATCH2=ON \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build /build --parallel 2
ctest --test-dir /build --output-on-failure --parallel 2 --timeout 120
~~~

`SLAM_APT_DEPENDENCIES=ON` changes only the minimum **GTSAM to 4.2**
and **Catch2 to 3.4** for APT compatibility. The existing default
versions remain 4.3 and 3.16. With both `SLAM_REQUIRE_GTSAM=ON` and
`SLAM_REQUIRE_CERES=ON`, configuration fails if a solver is absent.
`SLAM_REQUIRE_SYSTEM_CATCH2=ON` prevents a fallback Catch2 source
download; `FETCHCONTENT_FULLY_DISCONNECTED=ON` adds protection.

Before executing CTest, the runner also asserts that the expected
`match_ceres`, `core_catch2`, `pose_graph`,
`offline_map_promotion` and `map_bundle` CTests are registered. It
will not print a misleading green test run after silently skipping a
solver or the detailed tests.

The Docker image has no GitHub source checkout or GTSAM/Ceres/Catch2
build from source; all dependencies are standard Ubuntu APT packages.
The `.dockerignore` excludes maps, large datasets and local build
directories from the image's build context.

## Compatibility, reproducibility and limits

- The image is Ubuntu 24.04, **not** the Debian 13 release-cache ABI.
  Do not copy the Debian 13 compiled archive into this image.
- Docker image rebuilds can pick up newer patched APT packages in
  Ubuntu's repositories. The Dockerfile **checks** the expected
  major/minor versions, but is not a bit-for-bit pinned APT snapshot or
  base-image digest. For immutable reproducibility, record the built
  image digest or use a reviewed apt snapshot/locked base.
- The default `test` command runs synthetic/unit CTests. It does **not**
  silently download FR079 archives or make claims of independent
  real-dataset localization accuracy.
- This development workspace did **not** expose a working Docker engine,
  so the exact Ubuntu container build/runtime suite remains **unverified
  here**. The Dockerfile and shell script were syntax/structure checked,
  the script's Docker invocation was tested with a CLI stub, and CMake
  discovered the relaxed version constraints using available locally
  installed libraries. That is **not** a substitute for running the
  actual Ubuntu 24.04 image.
- No GitHub Actions workflow was created, dispatched, or changed.

For a new chat, the repository supplies this Docker configuration;
the [binary dependency Release](dependency-cache.md) remains an
independent fallback for its exact Debian 13/GCC 14 target environment.
