#!/usr/bin/env bash
set -euo pipefail
# This rebuild path is for a fresh/incompatible environment, not the normal
# restore path. It may take substantial CPU and RAM; it never runs Actions.
if [[ $# -ne 5 ]]; then
  echo "Usage: $0 <workdir> <Catch2-source-dir> <Ceres-source-dir> <GTSAM-source-dir> <Eigen3-header-root>" >&2
  exit 2
fi
WORK="$(mkdir -p -- "$1" && cd -- "$1" && pwd -P)"
CATCH="$(cd -- "$2" && pwd -P)"; CERES="$(cd -- "$3" && pwd -P)"
GTSAM="$(cd -- "$4" && pwd -P)"; EIGEN="$(cd -- "$5" && pwd -P)"
[[ -f "$CATCH/CMakeLists.txt" && -f "$CERES/CMakeLists.txt" && -f "$GTSAM/CMakeLists.txt" && -f "$EIGEN/Eigen/Core" ]] || {
  echo 'Complete extracted source trees and an Eigen 3.4 header root are required' >&2; exit 1;
}
mkdir -p "$WORK/install/eigen3/include/eigen3" "$WORK/install/eigen3/lib/cmake/Eigen3"
cp -a -- "$EIGEN/Eigen" "$EIGEN/unsupported" "$WORK/install/eigen3/include/eigen3/"
cat > "$WORK/install/eigen3/lib/cmake/Eigen3/Eigen3Config.cmake" <<'CMAKE'
get_filename_component(_SLAM_EIGEN_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(Eigen3_VERSION 3.4.0)
set(Eigen3_FOUND TRUE)
set(EIGEN3_INCLUDE_DIR "${_SLAM_EIGEN_PREFIX}/include/eigen3")
set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
unset(_SLAM_EIGEN_PREFIX)
CMAKE
cat > "$WORK/install/eigen3/lib/cmake/Eigen3/Eigen3ConfigVersion.cmake" <<'CMAKE'
set(PACKAGE_VERSION "3.4.0")
if(PACKAGE_FIND_VERSION VERSION_LESS_EQUAL PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
CMAKE
JOBS="${SLAM_DEPS_JOBS:-2}"
cmake -S "$CATCH" -B "$WORK/build-catch2" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$WORK/install/catch2" \
  -DCATCH_BUILD_TESTING=OFF -DCATCH_BUILD_EXAMPLES=OFF -DCATCH_BUILD_EXTRA_TESTS=OFF
cmake --build "$WORK/build-catch2" --parallel "$JOBS"
cmake --install "$WORK/build-catch2"
cmake -S "$CERES" -B "$WORK/build-ceres" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$WORK/install/ceres" \
  -DEigen3_DIR="$WORK/install/eigen3/lib/cmake/Eigen3" \
  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF \
  -DLAPACK=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF -DSCHUR_SPECIALIZATIONS=OFF
cmake --build "$WORK/build-ceres" --parallel "$JOBS"
cmake --install "$WORK/build-ceres"
# Conservative -O0/single-job GTSAM is for constrained development environments.
# The installed shared library uses relative RUNPATH for Metis/Cephes.
cmake -S "$GTSAM" -B "$WORK/build-gtsam" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O0 -DNDEBUG' \
  -DCMAKE_INSTALL_PREFIX="$WORK/install/gtsam" '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=OFF \
  -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
  -DGTSAM_BUILD_WITH_PRECOMPILED_HEADERS=OFF
cmake --build "$WORK/build-gtsam" --target gtsam CppUnitLite --parallel 1
cmake --install "$WORK/build-gtsam"
echo "Rebuilt dependency prefixes under $WORK/install"
echo 'Run package.py with the pinned source archives, source trees and installed prefixes.'
