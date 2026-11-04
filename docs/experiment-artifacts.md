# Experiment output and review-artifact convention

**Status:** version 2 (foldered FR079 runs are opt-in; legacy flat exports remain readable; no results are moved, deleted, overwritten, or re-scored automatically). This applies to persistent, user-invoked
SLAM benchmarks; temporary CTest fixtures are deliberately not retained.

## Where files go (host filesystem)

| Family | Output convention | Read-only Web UI |
| --- | --- | --- |
| Individual Freiburg FR079 native runs | `apps/demo/maps/<run-id>/<run-id>.slammap` with `.report.json`, `.trajectory.json`, `.matches.json`, `.slamreplay.json`, `.review.json` in the same run folder | Map Explorer and Recorded LiDAR Replay |
| Native synthetic fusion/mapping runs | `apps/demo/fusion/<run-id>/fusion_report.json`, `trajectories.json`, `fusion_replay.json`, `review-manifest.json` and map variants | Fusion Replay and Map Explorer |
| Multi-run experiment campaigns | `benchmarks/<campaign-id>/` containing immutable per-run outputs, `scorecard.csv`, `manifest.json`, `review-manifest.json` | Explicitly mount a supported folder if appropriate |
| Derived full/region/ranking CSVs | `reports/<prefix>-full.csv`, `<prefix>-regions.csv`, `<prefix>-ranking.csv`; companion `<prefix>-summary.review.json` | Upload the CSV files for analysis |

Map Explorer and Recorded LiDAR Replay now inspect **one level of named run
directories** below `apps/demo/maps/`, alongside existing root-level files.
They do not follow symlinks or recurse into deeper paths. The HTTP API gives
foldered map/replay IDs the syntax `<run-id>@<run-id>.slammap` and
`<run-id>@<run-id>.slamreplay.json` respectively. Legacy flat IDs remain
unchanged. `apps/demo/fusion` stays a run-directory collection. The multi-run study directory may instead be
under `benchmarks/` and mounted explicitly. Keep raw FR079 logs outside
Git; do not upload full recordings merely to inspect a benchmark report.
Use a **new run ID** when rerunning; do not silently overwrite previous data.

## Manifest and console output

New persistent native FR079 and synthetic-fusion experiments write a
`SLAM_REVIEW1` JSON inventory. The Python sweep drivers also write a
review inventory for aggregated CSVs and/or per-scene report files.
The inventory lists each generated file's `role`, `path`,
`host_path` (nullable), and `upload` flag.

Every supported generator prints a terminal block called
**Review artifacts (upload to ChatGPT)** with paths for the report,
trajectory/poses, matching audit, replay, and/or aggregated CSV.
Large `.slammap` maps and lossless binary logs remain accessible but are
**not marked as default uploads**. Upload them only when visual inspection
or map-file debugging is needed.

The path `/output` in Docker is **not** the host's `/output`: it is the
host directory specified to `-v`. A process inside Docker cannot infer
the bind mount's host path. Pass the host mapping explicitly:

```bash
# Run from repository root; stdout now prints the exact host paths.
export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
mkdir -p apps/demo/maps
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-continuous-noloop-002 \
  --run-directory --continuous-cost --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5
```

This creates `apps/demo/maps/fr079-continuous-noloop-002/` and its nested `fr079-continuous-noloop-002.review.json`
and prints absolute host paths for the four main JSON review files.
For a different Docker mount, set `SLAM_HOST_OUTPUT_DIR` to that mount's
actual host directory. `SLAM_CONTAINER_OUTPUT_DIR` defaults to
`/output` and may be overridden when another in-container mount is used.
Without a host mapping, the manifest uses `host_path: null` for files
under `/output` instead of incorrectly claiming those paths are on the
host. For a native host run, paths are already absolute host paths.

For synthetic fusion runs mounted as `/output`, use
`-e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/fusion"` alongside
`-v "$PWD/apps/demo/fusion:/output"`. The native generator prints the
per-scene review list and the seven-scene Python sweep can print the
aggregate paths when `summarize` completes.

### Organize an existing flat FR079 run

The following commands operate on **your existing run files** without rerunning
SLAM. First preview every move; only `--apply` actually moves data:

```bash
python3 tools/review_artifacts.py organize-fr079 \
  apps/demo/maps/fr079-continuous-noloop-001

python3 tools/review_artifacts.py organize-fr079 \
  apps/demo/maps/fr079-continuous-noloop-001 --apply
```

The tool requires all seven original FR079 artifacts, refuses symlinks and
existing destination directories, and writes a fresh review inventory within
the dedicated run folder. Earlier review manifests outside the folder
(including ones under `reports/`) are not modified and may point to the
old locations. After the move, upload the new manifest.

### Index old results without rerunning

The following command **does not run SLAM, copy maps, or alter result data**.
It generates the same review inventory for an existing Freiburg run:

```bash
mkdir -p reports
python3 tools/review_artifacts.py fr079 \
  apps/demo/maps/fr079-continuous-noloop-001 \
  --manifest reports/fr079-continuous-noloop-001.review.json
```

It prints absolute host paths for
`fr079-continuous-noloop-001.report.json`,
`.trajectory.json`, `.matches.json`, and `.slamreplay.json`,
and a review-manifest path in `reports/`. A previous inventory is
never rewritten. For a foldered FR079 run, the `fr079` indexing command
also accepts the folder path directly. To index a synthetic run, use
`python3 tools/review_artifacts.py fusion apps/demo/fusion/<run-id>`.

**For ChatGPT review**, upload the `*.review.json` inventory and the
files marked `upload: true`. The inventory contains paths only; it does
not include the contents of the reports, so it is not a substitute for
those JSON/CSV uploads.

## Local regression tests

```bash
python3 tests/review_artifacts_tools_test.py
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(review_artifacts_tools|review_artifacts_cpp|fr079_offline_slam|fr079_offline_http|native_web_api|fusion_mapping_sweep_tools)$'
```

No GitHub Actions are created, run or dispatched.
