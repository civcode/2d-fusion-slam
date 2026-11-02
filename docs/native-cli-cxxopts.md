# Native C++ command-line interfaces

Every native executable that previously parsed argv directly now uses
[cxxopts](https://github.com/jarro2783/cxxopts). Positional-only tools
share the helper `include/slam/cli/parse.hpp`, which itself delegates
tokenization, option handling, and `--help` to cxxopts. Tools with
flags declare typed cxxopts options directly.

## Build dependency

CMake first attempts `find_package(cxxopts 3.1 CONFIG)`. If no
compatible installed target exists, it uses `FetchContent` to obtain
the pinned `v3.3.1` release; it is not vendored into the repository.
For disconnected builds, supply a system package or
`-DFETCHCONTENT_SOURCE_DIR_CXXOPTS=/path/to/cxxopts`.

Ubuntu 24.04 Docker build images install `libcxxopts-dev` (3.1.1)
and keep `-DFETCHCONTENT_FULLY_DISCONNECTED=ON` as before, so building
the test/demo images does not fetch dependencies from GitHub.
CMake links `cxxopts::cxxopts` **only to CLI executables**, not core
SLAM libraries. No GitHub Actions changes were made.

## Migrated executables

| Executable | Arguments retained |
|---|---|
| `slam_fr079_offline` | `log prefix`, `--reference`, `--support-aware`, `--match-every`, `--field-every`, `--map-stride`, `--match-stride`, `--distance-truncation`, `--capture`, `--capture-every`, `--capture-max-frames` |
| `slam_fr079_train_only_benchmark` | `log [--reference]` |
| `slam_ceres_replay_export` | `map log scan-stride`, `--beam-stride`, `--max-scans`, `--verified-fr079-holdout` |
| `slam_fr079_alignment_inspect` | `log map` |
| `slam_fr079_map_export` | `log output-prefix` |
| `slam_carmen_inspect` | `log [expected_scan_count]` |
| `slam_carmen_map_inspect` | `map [width height]` |
| `slam_map_inspect` | `map` |
| `slam_map_accuracy_cli` | `apartment|warehouse generated-map` |
| `slam_synthetic_eval_fixture` | `apartment|warehouse output-map` |
| `slam_synthetic_mapping_benchmark` | `apartment|warehouse output-dir` |
| `slam_offline_fusion` | `apartment|warehouse output-dir [--synthetic-independent-loops]` |
| `fusion_slam_demo` | `--data-dir`, `--replay-dir`, `--fusion-dir`, `--port`, `--host`, `--allow-network`, `--example-if-empty` |

`--help` is now available consistently. Unknown flags are rejected.
Existing range validation, no-overwrite rules, and explicit
`--allow-network` requirement for non-loopback viewer binding remain.

Python tools continue to use Python argument parsers; cxxopts is a
C++ header-only library, not a replacement for Python argparse.

## Local validation

```sh
docker compose build test demo
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'cli_cxxopts_|fr079_offline_slam|fr079_support_aware|offline_fusion|native_web_api'
```

Each `cli_cxxopts_*` CTest runs its executable with `--help`
(expect success) and an unknown flag (expect failure), without
opening datasets or writing output artifacts. Existing CTest
functional tests exercise successful data-processing paths.
