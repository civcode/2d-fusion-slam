# FR079 — Ceres vs frontend-preserving GTSAM loop benchmark

**Experimental, reference-relative comparison only.** This benchmark asks whether
offline LiDAR loop factors improve the estimated trajectory and LiDAR hit-map
agreement relative to the published **FR079 scan-matched estimate**.
The scan-matched recording is *not surveyed ground truth*. Neither trajectory
ATE nor raster scores establish absolute physical accuracy.

The official dataset entry lists three separate artifacts:
[MIT/Radish fr079](https://dspace.mit.edu/entities/publication/2fe88c52-8693-4343-add4-b09f07257f94)
(`fr079.log.gz`, `fr079-sm.log.gz`, and `fr079-map.tgz`).
Use the **raw** recording for estimation and the **scan-matched** recording
for evaluation only. The published `fr079-sm.log.gz` compressed-file MD5 is
`2f1d5b62f229d3e86c43460255f761bb`; the benchmark verifies the
archive bytes and refuses a different reference log by default. Synthetic
test fixtures opt out of this checksum through the Python API only.

## Experimental arms

1. **A — existing Ceres frontend:** the already computed, immutable
   `fr079-continuous-noloop-001` run (3,118 scans,
   Ceres continuous cost, logged odometry prediction, 472/623 accepted).
2. **B — identical frontend + GTSAM loop factors:** a **new** run
   `fr079-continuous-ceresgraph-loops-001` using the same frontend
   settings plus the new flags
   `--lidar-loop-closure --frontend-sequential-graph`.

Unlike the older `fr079-continuous-loop-001`, arm B does **not**
replace Ceres sequential motion with raw logged odometry during GTSAM
optimization. It uses consecutive relative poses from the frontend's
corrected trajectory, explicitly tagged as **correlated frontend-derived
factors**. The GTSAM backend validates this provenance separately from
independently measured factors. These are not independent wheel
measurements and their nominal noise is an experimental modeling
assumption, not measured covariance.

The no-loop graph **must** reconstruct the frontend poses to within
`1e-4 m` and `1e-4 rad` at each scan or the native run fails.
The loop-enabled graph adds only the detected reciprocal LiDAR loop
constraints, uses the same sequential factors, and rebuilds a third map.
Zero accepted loops is a valid result and must not be reported as an
improvement. The legacy **3 m / 0.9 rad** post-optimization graph-shift
warning still exists; it is not an optimization rejection gate.

## 1. Local build and tests

Run from the repository root. GitHub Actions are neither enabled nor invoked.

```bash
git pull --ff-only
docker compose build test demo
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(fr079_offline_slam|fr079_loop_experiment|fr079_reference_benchmark_tools|fr079_loop_compare_tools)$'
python3 tests/fr079_reference_benchmark_tools_test.py
```

The native integration test exercises both historical odometry-only and
**new frontend-sequential** graph modes. The Python fixture tests reference
pairing, SE(2) alignment, error metrics, map-proxy calculations, source
provenance and rejection of mismatched Ceres or laser scans.

## 2. Reuse A; generate only the new B

The old no-loop run must stay unchanged. Obtain
`fr079-sm.log.gz` separately from the official FR079 dataset download.
Never pass it as the SLAM source.

```bash
export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
export FR079_SM_LOG="$PWD/.build/datasets/fr079-sm.log.gz"
test -f "$FR079_LOG" && test -f "$FR079_SM_LOG" || {
  echo "Missing raw or published scan-matched FR079 log" >&2
  exit 1
}

mkdir -p apps/demo/maps reports
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-continuous-ceresgraph-loops-001 \
  --run-directory --continuous-cost \
  --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph
```

The new `*.report.json` and `*.loops.json` both contain
`sequential_factor_source=frontend_scanmatched_relative_correlated_v1`.
For this new mode, the JSON array is named `graph_control_poses`, not
the historical `odometry_graph_poses` used by the old raw-odometry experiment.
The former also carries
`loop_mode=fr079_frontend_relative_gtsam_loops_v2`.
The generated maps are:

- `*.slammap` — unchanged incremental Ceres map.
- `*.graph_control.slammap` — frontend-preserving GTSAM no-loop control.
- `*.graph_optimized.slammap` — frontend-relative factors plus LiDAR loops.

The historical `fr079-continuous-loop-001` remains a
separate **logged-odometry graph** experiment, not this benchmark's arm B.

## 3. Compare both against the scan-matched FR079 estimate

```bash
python3 tools/fr079_reference_benchmark.py \
  --raw-log "$FR079_LOG" \
  --reference-sm-log "$FR079_SM_LOG" \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --optimized apps/demo/maps/fr079-continuous-ceresgraph-loops-001 \
  --out reports/fr079-reference-benchmark-001
```

Outputs (exclusive; choose a new prefix for a rerun):

- `reports/fr079-reference-benchmark-001.json`
- `reports/fr079-reference-benchmark-001.csv`
- `reports/fr079-reference-benchmark-001.review.json`

**Fail-closed checks:** identical raw log SHA-256; same Ceres matcher,
configuration, poses, per-scan matching decisions and frontend map
fingerprint; correct GTSAM frontend-sequential provenance; graph control
equal to frontend; exact scan-index pairing with timestamp difference at
most 20 ms; *all* LiDAR beam ranges identical in the raw and published
scan-matched logs within 0.0001 m. The reference is read by this offline
evaluator only, never by Ceres/GTSAM.

### Trajectory statistics

For **each** arm against the **same** published reference, report:
ATE translation mean/RMSE/median/p95/max; absolute yaw error; and SE(2)
relative-pose translation/yaw errors over 1, 10 and 50 scan intervals.
Show two distinct coordinate alignments:

- **First-pose anchored:** align only the first pose, retains subsequent
  drift and global orientation difference.
- **Rigid XY (SE(2)) best-fit:** estimate one global rotation and
  translation from all positions; no scaling and no reflection.
  Reports the favorable alignment separately, never as if
  first-pose anchored error.

The report records both before and after values and the percent
reduction in reference-relative ATE RMSE (and 10-scan RPE RMSE).

### Map statistic: a reference-derived *proxy*, not physical map truth

For a reproducible shared frame, the evaluator projects **identical raw
LiDAR hit endpoints** at the published scan-matched reference poses, and
separately at the Ceres and loop-optimized poses. It uses 0.10 m raster
cells and every fourth beam by default, retaining occupied cells with
at least two hits. It reports strict occupied-endpoint IoU and 1-cell
tolerant occupied-endpoint precision/recall, for both coordinate
alignments and both algorithms. Configure with
`--map-resolution 0.1 --map-beam-stride 4`.

These are **reference-trajectory-conditioned occupied endpoint maps**,
not full occupancy grids with equivalent unknown/free-state semantics.
The official `fr079-map.tgz` map is **not silently compared**
because its frame registration, origin and comparable occupancy
semantics have not yet been verified. No number from this tool should
be labeled independent physical wall/map accuracy, map IoU against
surveyed truth, or externally validated ATE.

## Interpretation

A positive percentage means **more agreement with the published,
correlated scan-matched reference**, not proven improvement in
physical localization accuracy. The reference may share scan matching
biases with our frontend. A failed scan-pairing or provenance check is
an invalid benchmark, not a reason to weaken the checks.

To assess physical map error against `fr079-map.tgz`,
first establish a documented frame transform, cell conventions,
resolution and independently verifiable registration, then add that
as a distinct evaluation.
