# Offline Catch2 3.16.1 and Ceres 2.2.0

This repository does not vendor third-party projects and never runs
GitHub Actions. The supplied archives are complete source releases,
not runtime dependencies bundled into this Git repository.

## Catch2 3.16.1

Extract `Catch2-3.16.1(1).zip` somewhere outside the checkout.
CMake now accepts a source-root override:

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/Catch2-3.16.1
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Without an override, CMake pins Catch2 upstream **v3.16.1 commit**
`08092139210881f01a34e60c5f2fd6afa0ea4024`.
No Catch2 download occurs when `BUILD_TESTING=OFF` or
`SLAM_ENABLE_CATCH2=OFF`. The supplied archive SHA-256 is
`7509086e230246c393e82a4065b0c2b3b934c3f9e0572b7ab4927a74cf50efc0`.

## Ceres 2.2.0

Ceres requires Eigen3 3.3+ and a functioning C++17+ toolchain. This
development environment had Eigen 3.4 headers but no system-installed
Eigen CMake package, so a **local Eigen3 package configuration** was
provided for offline compilation; this is an environment workaround,
not a project source file. Ceres was built from the supplied archive:

~~~sh
cmake -S /path/to/ceres-solver-2.2.0 -B /path/to/ceres-build -G Ninja \
  -DEigen3_DIR=/path/to/Eigen3Config-directory \
  -DCMAKE_BUILD_TYPE=Release \
  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF \
  -DCXSPARSE=OFF -DLAPACK=OFF -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DSCHUR_SPECIALIZATIONS=OFF
cmake --build /path/to/ceres-build
cmake --install /path/to/ceres-build --prefix /path/to/ceres-prefix
~~~

For another machine with a conventional Eigen installation, omit the
`Eigen3_DIR` override or point it to the real package.
The Ceres source ZIP SHA-256 is
`1fc28e22ce190ce4c1db04d0c2ddfdf8dc836609866d09372ea8533c8b6d490b`.

To demand a Ceres matcher and fail clearly if it cannot be linked:

~~~sh
cmake -S . -B build-ceres -G Ninja \
  -DCeres_DIR=/path/to/ceres-prefix/lib/cmake/Ceres \
  -DEigen3_DIR=/path/to/Eigen3Config-directory \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/Catch2-3.16.1
cmake --build build-ceres
ctest --test-dir build-ceres --output-on-failure
~~~

**Verification completed here:** Ceres 2.2.0 static library built; a
separate Ceres consumer solved a least-squares system; the repository's
byte-identical `src/match_ceres.cpp` compiled using real Ceres 2.2.0
headers/library in an isolated adapter build. **A complete checkout
Ceres-linked CTest run remains unverified** and must not be inferred
from these checks. When using `MINIGLOG`, review compatibility before
linking another logging implementation.
