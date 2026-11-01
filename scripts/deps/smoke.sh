#!/usr/bin/env bash
set -euo pipefail
# Build and execute one CTest binary linking all four installed packages.
if [[ $# -ne 1 ]]; then echo "Usage: $0 /path/to/extracted/bundle" >&2; exit 2; fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
BUNDLE="$(cd -- "$1" && pwd -P)"
python3 "$ROOT/scripts/deps/verify.py" "$BUNDLE"
BUILDDIR="$(mktemp -d)"
trap 'rm -rf -- "$BUILDDIR"' EXIT
cmake -S "$ROOT/tests/deps" -B "$BUILDDIR" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF \
  -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF \
  -DCMAKE_PREFIX_PATH="$BUNDLE/install/eigen3;$BUNDLE/install/catch2;$BUNDLE/install/ceres;$BUNDLE/install/gtsam"
cmake --build "$BUILDDIR" --parallel 2
ctest --test-dir "$BUILDDIR" --output-on-failure
if [[ "${SLAM_SUPPRESS_USAGE:-0}" != 1 ]]; then
  printf '\nTo use this dependency bundle with 2d-fusion-slam:\n'
  printf '  cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="%s" -DSLAM_REQUIRE_CERES=ON -DSLAM_REQUIRE_GTSAM=ON\n' "$BUNDLE/install/eigen3;$BUNDLE/install/catch2;$BUNDLE/install/ceres;$BUNDLE/install/gtsam"

fi
