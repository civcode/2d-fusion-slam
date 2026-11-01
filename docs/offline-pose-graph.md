# Offline Pose2 factor-graph reference (GTSAM 4.3.0)

**Status:** A workstation-only *batch reference optimizer*, not a complete
SLAM frontend, loop detector, relocalizer, pose covariance estimator, or
Raspberry Pi executable. Its synthetic fixtures provide exact independent
relative measurements. FR079 logged poses are NOT independent evidence.

## Uploaded GTSAM source

The user-supplied source archive `gtsam-4.3.0.zip` has SHA-256:

```text
44a5a8da6e553f03c229ebc75973c80a6474b7f3f485a7ef230ad64bce3a5f76
```

Do not vendor the 48 MB archive or generated build products into Git. The
repository supports a **locally installed 4.3 package** with `GTSAM_DIR`, or
building from an explicitly supplied source root with
`SLAM_GTSAM_SOURCE_DIR`. No GTSAM download is performed automatically.

After extracting the ZIP to `/path/to/gtsam-4.3.0`, one reproducible offline
workstation build is:

```sh
cmake -S /path/to/gtsam-4.3.0 -B /path/to/gtsam-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=OFF \
  -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
  -DGTSAM_BUILD_WITH_PRECOMPILED_HEADERS=OFF \
  -DGTSAM_COMPILE_OPTIONS_PRIVATE_RELEASE=-O0 \
  -DCMAKE_CXX_FLAGS_RELEASE='-O0 -DNDEBUG' \
  -DCMAKE_INSTALL_RPATH='$ORIGIN'
cmake --build /path/to/gtsam-build --target gtsam CppUnitLite --parallel 1
cmake --install /path/to/gtsam-build --prefix /path/to/gtsam-prefix
```

The low-optimization single-job settings are intended for small developer
containers; use standard Release settings on a workstation with more RAM.
The relative `$ORIGIN` runpath is for a locally installed shared build:
it lets `libgtsam.so` locate the bundled Metis/Cephes libraries installed
beside it, without requiring a global loader configuration. `CppUnitLite`
is included because GTSAM 4.3's source install manifest references it.

Then build this project:

```sh
cmake -S . -B build-gtsam -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON \
  -DGTSAM_DIR=/path/to/gtsam-prefix/lib/cmake/GTSAM
cmake --build build-gtsam --parallel 2
ctest --test-dir build-gtsam --output-on-failure
./build-gtsam/slam_pose_graph_demo
```

`SLAM_REQUIRE_GTSAM=ON` guarantees a missing source/package cannot silently
skip the graph tests. You can also set
`SLAM_GTSAM_SOURCE_DIR=/path/to/gtsam-4.3.0` to build GTSAM as a CMake
subproject if no installed GTSAM 4.3 is found. The default standalone
build does **not** require GTSAM and does not compile it for Pi targets.

## Factor provenance and safety

- One Pose2 variable per ordered keyframe; exactly one odometry factor
  connecting each consecutive keyframe; no invented wheel/IMU translation.
- Every odometry factor must identify its source and positively assert a
  verified source clock/frame and independence from the initial estimate.
  Synthetic-exact factors are permitted **only in synthetic fixtures**.
- The first Pose2 is anchored by a constrained GTSAM prior to fix the gauge.
- Candidate loop constraints are excluded unless they have independent
  measured/synthetic evidence, sufficient supported scan hits, overlap and
  full-rank diagnostics, and pass a conservative initial-path consistency
  gate. A *robust Huber* loss reduces (but cannot eliminate) the influence
  of an accepted outlier. Rejected candidates never affect the graph.
- A real loop factor sharing a source identifier with odometry is treated
  as correlated and rejected. Geometry-only Ceres information eigenvalues
  do **not** certify a calibrated, independent covariance.
- This is a **batch offline graph**: it has no automatic loop search,
  incremental optimization, map-generation mutation, statistical
  independence proof, or independent real-data pose ground truth.
- Graph costs measure internal factor consistency and are **not** ATE/RPE.
  A batch graph result should drive a *new* map generation only after
  additional held-out evaluation and provenance checks.

The manual GitHub Actions workflow remains manual-only and does not install
or test the optional GTSAM target by default. No workflow was run to
implement this milestone.
