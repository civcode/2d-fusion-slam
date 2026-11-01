#!/usr/bin/env bash
# Build and test the native C++ project inside Ubuntu 24.04 using APT packages.
# No host C++ dependencies or GitHub Actions required.
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
image="${SLAM_DOCKER_IMAGE:-2d-fusion-slam:ubuntu24.04-apt}"
build_dir="$repo/.build/docker-ubuntu24.04-apt"
action="${1:-test}"
if [[ $# -gt 1 ]]; then
  echo 'Usage: scripts/docker-build-test.sh [image|build|test|shell]' >&2
  exit 2
fi
case "$action" in image|build|test|shell) ;; *)
  echo 'Usage: scripts/docker-build-test.sh [image|build|test|shell]' >&2; exit 2;;
esac
if ! command -v docker >/dev/null 2>&1; then
  echo 'Docker CLI is required, but no host Ceres/Catch2/GTSAM packages are needed.' >&2
  exit 1
fi

if [[ "$action" == image ]] || ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build --file "$repo/Dockerfile" --target dependencies --tag "$image" "$repo"
fi
[[ "$action" == image ]] && exit 0
mkdir -p -- "$build_dir"

args=(run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp
  --mount "type=bind,source=$repo,target=/workspace,readonly"
  --mount "type=bind,source=$build_dir,target=/build"
  --workdir /workspace)
if [[ "$action" == shell ]]; then
  exec docker "${args[@]}" --interactive --tty "$image" bash
fi

# Keep package requirements explicit: no fallback to FetchContent, GTSAM
# source builds, or geometry-only tests if an APT package is unavailable.
script='set -euo pipefail
cmake -S /workspace -B /build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON \
  -DSLAM_APT_DEPENDENCIES=ON \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON \
  -DSLAM_ENABLE_CATCH2=ON -DSLAM_REQUIRE_SYSTEM_CATCH2=ON \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build /build --parallel "${SLAM_DOCKER_JOBS:-2}"
'
if [[ "$action" == test ]]; then
  script+='for test in match_ceres core_catch2 pose_graph offline_map_promotion map_registry map_bundle web_ui_unit demo_fixture_native; do
  if ! ctest --test-dir /build -N -R "^${test}$" | grep -Fq ": ${test}"; then
    echo "Required CTest target missing: ${test}" >&2
    exit 1
  fi
done
ctest --test-dir /build --output-on-failure --parallel "${SLAM_DOCKER_JOBS:-2}" --timeout 120
'
fi
exec docker "${args[@]}" --env "SLAM_DOCKER_JOBS=${SLAM_DOCKER_JOBS:-2}" "$image" bash -c "$script"
