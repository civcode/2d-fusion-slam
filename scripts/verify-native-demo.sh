#!/usr/bin/env bash
# Manual-only local verification for the native C++ SLAM browser demo.
# Never invokes GitHub Actions, network downloads, or a remote runner.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if ! command -v cmake >/dev/null || ! command -v ninja >/dev/null; then
  echo "Need cmake >=3.25 and ninja installed locally" >&2
  exit 2
fi
if ! command -v python3 >/dev/null; then
  echo "Need Python 3.10+ as a test driver (not as the viewer runtime)" >&2
  exit 2
fi

build_dir="${SLAM_NATIVE_DEMO_BUILD_DIR:-build-native-demo}"
if [[ "$build_dir" == /* ]]; then
  echo "Build directory must be relative to the repository" >&2
  exit 2
fi
jobs="${SLAM_NATIVE_DEMO_JOBS:-2}"
if ! [[ "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "SLAM_NATIVE_DEMO_JOBS must be a positive integer" >&2
  exit 2
fi

cmake -S . -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_ENABLE_GTSAM=OFF \
  -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_BUILD_NATIVE_DEMO=ON \
  -DSLAM_REQUIRE_NATIVE_DEMO=ON \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON

cmake --build "$build_dir" --parallel "$jobs" --target \
  fusion_slam_demo slam_map_inspect slam_native_viewer_json_test

# Execute the native API end-to-end regression, cross-language binary-map
# parity fixture, and all existing offline Python viewer tests.
ctest --test-dir "$build_dir" \
  --output-on-failure --timeout 90 \
  -R '^(native_web_api|native_viewer_json|demo_fixture_native|web_ui_unit)$'

echo "Native viewer local verification passed."
echo "Run the compiled viewer:"
echo "  ./$build_dir/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty"

# Docker remains an explicit, separate opt-in validation:
# docker build --target demo -t fusion-slam-native-demo .
# docker run --rm -p 127.0.0.1:8765:8765 \
#   -v "$PWD/apps/demo/maps:/maps:ro" fusion-slam-native-demo
