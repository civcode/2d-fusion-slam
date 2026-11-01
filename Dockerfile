# syntax=docker/dockerfile:1
# Root-level Docker targets follow the hybrid-astar-cpp demo/test pattern.
# Native SLAM builds use Ubuntu 24.04 APT, never the Debian 13 binary bundle.
FROM ubuntu:24.04 AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN set -eux; \
    sed -i '/^Components:/ { /universe/! s/$/ universe/; }' /etc/apt/sources.list.d/ubuntu.sources; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      ca-certificates build-essential cmake ninja-build pkg-config python3 \
      libeigen3-dev zlib1g-dev libceres-dev libgtsam-dev catch2; \
    for spec in 'libceres-dev:2.2.' 'libgtsam-dev:4.2.' 'catch2:3.4.'; do \
      name="${spec%%:*}"; version="${spec#*:}"; \
      actual="$(dpkg-query -W -f='${Version}' "$name")"; \
      case "$actual" in "$version"*) echo "Verified $name $actual";; \
        *) echo "Unsupported $name $actual (expected $version*)" >&2; exit 1;; \
      esac; \
    done; \
    rm -rf /var/lib/apt/lists/*

FROM dependencies AS build
WORKDIR /workspace
COPY . /workspace
RUN cmake -S . -B build-docker -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
      -DSLAM_APT_DEPENDENCIES=ON \
      -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
      -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON \
      -DSLAM_ENABLE_CATCH2=ON -DSLAM_REQUIRE_SYSTEM_CATCH2=ON \
      -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  && cmake --build build-docker --parallel 2

# Tests are run only when the user invokes this target.
FROM build AS test
ENV CTEST_OUTPUT_ON_FAILURE=1
CMD ["ctest", "--test-dir", "build-docker", "--output-on-failure", "--parallel", "2", "--timeout", "120"]

# The viewer reads completed .slammap files; it does not require native SLAM
# libraries or a compiled project to run. Keep the default demo image small.
FROM ubuntu:24.04 AS demo
ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1
RUN apt-get update && apt-get install -y --no-install-recommends python3 ca-certificates \
  && rm -rf /var/lib/apt/lists/* \
  && useradd --system --uid 10001 --home-dir /workspace --no-create-home viewer \
  && mkdir -p /maps
WORKDIR /workspace
COPY --chown=viewer:viewer apps/demo /workspace/apps/demo
USER viewer
EXPOSE 8765
CMD ["python3", "apps/demo/main.py", "--data-dir", "/maps", "--host", "0.0.0.0", "--allow-network"]
