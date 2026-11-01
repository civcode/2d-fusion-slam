#!/usr/bin/env bash
# Manual-only, local/offline C++ fusion build and deterministic benchmark checks.
# Does not invoke GitHub Actions, fetch network dependencies or overwrite maps.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

build="${SLAM_FUSION_BUILD_DIR:-build-fusion}"
if [[ "$build" = /* ]]; then
  echo "Use a relative SLAM_FUSION_BUILD_DIR for the repository checkout." >&2
  exit 2
fi
jobs="${SLAM_FUSION_JOBS:-2}"
if ! [[ "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "SLAM_FUSION_JOBS must be a positive integer" >&2
  exit 2
fi

# Full Ceres + GTSAM path by default. Export SLAM_FUSION_REQUIRE_SOLVERS=OFF
# for a geometry-only fallback smoke test on a minimal Eigen toolchain.
solver="${SLAM_FUSION_REQUIRE_SOLVERS:-ON}"
if [[ "$solver" != ON && "$solver" != OFF ]]; then
  echo "SLAM_FUSION_REQUIRE_SOLVERS must be ON or OFF" >&2
  exit 2
fi
cmake -S . -B "$build" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_BUILD_NATIVE_DEMO=ON -DSLAM_REQUIRE_NATIVE_DEMO=ON \
  -DSLAM_ENABLE_CERES="$solver" -DSLAM_REQUIRE_CERES="$solver" \
  -DSLAM_ENABLE_GTSAM="$solver" -DSLAM_REQUIRE_GTSAM="$solver" \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build "$build" --parallel "$jobs" --target \
  slam_gyro_wheel_test slam_offline_fusion slam_map_accuracy_cli fusion_slam_demo
ctest --test-dir "$build" --output-on-failure \
  --timeout 240 -R '^(gyro_wheel_fusion|offline_fusion|fusion_replay_http|native_web_api)

echo "Fusion and native replay HTTP CTests completed. To retain benchmark results:"
echo "  ./$build/slam_offline_fusion apartment /tmp/fusion-apartment-unique"
echo "  ./$build/slam_offline_fusion warehouse /tmp/fusion-warehouse-unique"
echo "Neither command will overwrite an existing output directory."


echo "Fusion native CTest completed. To retain benchmark results:"
echo "  ./$build/slam_offline_fusion apartment /tmp/fusion-apartment-unique"
echo "  ./$build/slam_offline_fusion warehouse /tmp/fusion-warehouse-unique"
echo "Neither command will overwrite an existing output directory."
