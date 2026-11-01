# Docker development with Ubuntu APT dependencies

Build and test without installing a host compiler or SLAM dependencies.
Docker is the only required host tool; GitHub Actions remains manual-only.

## Build and test

From the repository root, use 28 parallel build/test jobs:

```sh
SLAM_DOCKER_JOBS=28 docker compose run --build --rm test

# Or without Compose:
docker build --build-arg SLAM_DOCKER_JOBS=28 --target test -t fusion-slam:test .
docker run --rm --init fusion-slam:test
```

The build argument controls CMake build parallelism and sets the test image's
`CTEST_PARALLEL_LEVEL`. Override test parallelism at runtime with
`docker run --rm -e CTEST_PARALLEL_LEVEL=28 fusion-slam:test`.
Without an override, builds default to two jobs.

For incremental development, the wrapper mounts the checkout read-only and
writes build outputs to Git-ignored `.build/docker-ubuntu24.04-apt/`, using
the host UID/GID:

```sh
bash scripts/docker-build-test.sh image
SLAM_DOCKER_JOBS=28 bash scripts/docker-build-test.sh test
# Other actions: build, shell
```

Rebuild the dependency image with `image` after Dockerfile dependency changes;
the wrapper otherwise reuses an existing image. Override its name with
`SLAM_DOCKER_IMAGE`. Keep container build directories separate from host builds.

If Docker's default configuration directory is read-only, set `DOCKER_CONFIG`
to a writable directory. Custom contexts/registry credentials may need to be
configured there as well. If the checkout has an unreadable `.env`, use
`docker compose --env-file /dev/null ...` to bypass implicit loading without
modifying that file.

## Native browser demo

```sh
SLAM_DOCKER_JOBS=28 docker compose up --build demo
# Open http://127.0.0.1:8765
```

Compose publishes loopback only and mounts `apps/demo/maps` read-only. With
no maps, the viewer serves a labeled synthetic example without writing to the
mount. Do not expose this unauthenticated viewer publicly.

Dockerfile targets:

- `dependencies`: Ubuntu 24.04 development libraries and Python test driver.
- `build`: complete native build, requiring Ceres, GTSAM, Catch2 and the viewer.
- `test`: runs CTest on invocation, not during image construction.
- `demo-builder`: independent C++ viewer build without solvers or Catch2.
- `demo` (default): non-root native viewer runtime; no Python runtime required.

The `test` Compose service is opt-in and does not start with the demo.
The `.dockerignore` excludes local build outputs, maps and recorded sessions.

## Ubuntu compatibility profile

| Dependency | Ubuntu 24.04 package baseline | Native default minimum |
| --- | --- | --- |
| Ceres | 2.2.x | Toolchain-provided |
| GTSAM | 4.2.x | 4.3 |
| Catch2 | 3.4.x | 3.16 |
| Eigen3 | 3.4.x | 3.3 |

`SLAM_APT_DEPENDENCIES=ON` selects the distro GTSAM/Catch2 minimums. The
Dockerfile checks installed package major/minor versions. Required solver and
Catch2 options prevent silently skipped coverage; FetchContent is disconnected.
This is a compatibility matrix, not validation of the newer native baseline.
Do not mix in the [Debian 13 binary dependency bundle](dependency-cache.md).

Noble's `libgtsam-dev` package has incomplete development dependencies and
exports a `CppUnitLite` target whose archive is absent. The dependency stage
explicitly installs the required Boost, TBB and METIS development packages,
and removes only the unused CppUnitLite export from the installed CMake files.
It preserves GTSAM's actual library exports and existence checks; no solver
is replaced or built from source. References:
[Ubuntu package](https://packages.ubuntu.com/noble/libgtsam-dev),
[package file list](https://packages.ubuntu.com/noble/amd64/libgtsam-dev/filelist),
[upstream dependency discovery](https://github.com/borglab/gtsam/blob/4.2/cmake/Config.cmake.in).

## Local validation

Verified on the `local-build` branch with Docker Engine 28.3.3, Linux amd64,
Ubuntu 24.04/GCC 13.3, Ceres 2.2, GTSAM 4.2 and Catch2 3.4:

- Release `test` image and Compose test service: **31/31 CTests passed**, 28 jobs.
- Incremental RelWithDebInfo wrapper: **31/31 CTests passed**, 28 jobs.
- `scripts/verify-native-demo.sh` inside Docker: **4/4 CTests passed**.
- Slim `demo` image: non-root startup with a read-only map mount; HTTP checks
  passed for health, map listing/metadata/PNG, both pages, JavaScript and CSS.
- Headless Chromium in a separate Playwright container decoded the sample map
  and replay overlay. Both servers explicitly allow `blob:` in CSP `img-src`
  for fetched PNG object URLs; scripts remain restricted to `'self'`.

These are synthetic/unit/integration regressions. External FR079 dataset
CTest targets remain opt-in and were not run; no hardware validation is claimed.
No GitHub Actions workflow was dispatched or changed.

Ubuntu tags and APT repositories are not immutable snapshots: record image
IDs/digests for exact reproduction, or pin a reviewed base and package snapshot.
