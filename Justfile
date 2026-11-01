
# Justfile

set shell := ["bash", "-euo", "pipefail", "-c"]

# Configuration
export BUILD_JOBS := env_var_or_default("BUILD_JOBS", "28")
export TEST_JOBS := env_var_or_default("TEST_JOBS", "16")
export SLAM_DOCKER_JOBS := BUILD_JOBS

# Show available commands
default:
    @just --list

# Build test and demo images
build:
    docker compose build test demo

# Run test container
test: build
    docker compose run --rm test

# Run selected CTest suites
ctest: build
    docker compose run --rm --entrypoint ctest test \
        --test-dir build-docker \
        --parallel "$BUILD_JOBS" \
        -R 'offline_fusion|fusion_replay_http|native_web_api' \
        --output-on-failure

# Generate all fusion replays
fusion: fusion-apartment fusion-warehouse

# Generate apartment fusion replay
fusion-apartment: build
    mkdir -p apps/demo/fusion
    docker compose run --rm \
        -v "$PWD/apps/demo/fusion:/fusion" \
        --entrypoint ./build-docker/slam_offline_fusion \
        test apartment /fusion/apartment-001

# Generate warehouse fusion replay
fusion-warehouse: build
    mkdir -p apps/demo/fusion
    docker compose run --rm \
        -v "$PWD/apps/demo/fusion:/fusion" \
        --entrypoint ./build-docker/slam_offline_fusion \
        test warehouse /fusion/warehouse-001

# Start demo in background
demo: build
    docker compose up -d demo

# Follow demo logs
demo-logs:
    docker compose logs -f demo

# Stop and remove demo containers
demo-down:
    docker compose down

# Run complete workflow
all: test ctest fusion demo


