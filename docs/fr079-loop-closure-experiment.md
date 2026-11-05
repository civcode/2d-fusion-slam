# FR079 — controlled real-data LiDAR loop-closure experiment

**Status:** opt-in **experimental** post-hoc GTSAM graph comparison.
This is a *new real-data experiment*, not a production SLAM setting.
The FR079 recording has logged scan-time odometry and LiDAR but no
independently surveyed pose ground truth, calibrated laser extrinsics,
or per-beam acquisition timestamps. Never describe Ceres or GTSAM graph
objective changes as measured real-world accuracy.

## What stays fixed

The previously completed, unmodified `fr079-continuous-noloop-001` is
our reference. The loop run uses the same **3,118** source scans, same
front-end matching decisions, same occupancy evidence settings, and
same scan-time embedded odometry. Only an **offline graph stage** is added
after the incremental front-end has finished.

The experiment has *three* maps and pose histories:

1. `<run>.slammap` — original causal Ceres incremental map (must have
   the **same fingerprint and trajectory** as the old no-loop run).
2. `<run>.graph_control.slammap` — GTSAM optimized with the logged
   odometry factors **only**, no closures. This is the fair graph control
   against which loop factors must be compared; using only the old Ceres
   map as a control would conflate graph optimization with loop closure.
3. `<run>.graph_optimized.slammap` — identical GTSAM odometry factors
   plus conservative LiDAR scan-to-scan loop factors.

Loop proposals are drawn from spatially and temporally separated
front-end estimates; estimated poses only **shortlist** scan pairs.
Rigid registration from independently read **raw scan hits** creates
the relative-pose measurement (the same LiDAR evidence is not
independent of the frontend). Each candidate must pass reciprocal ICP,
overlap/inlier, rank, innovation, and loop-factor redundancy checks.
The GTSAM graph uses robust, conservatively weighted loop factors.
A run with **zero** accepted loops remains a valid *negative result*.

No external FR079 maps, logged laser poses, simulator truth or future
scans are used as estimating measurements. There is no automatic
parameter promotion.

## 1. Validate local native build

From the repository root:

```bash
git pull --ff-only
docker compose build test demo

docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(fr079_loop_compare_tools|fr079_loop_experiment|fr079_offline_slam|fr079_offline_http|native_web_api)$'
```

The real-data loop flag is **fail-closed** without GTSAM: the binary
prints a missing-backend error instead of silently running no-loop.

The native integration test uses a small deterministic **CARMEN-format
synthetic fixture** to check graph exports, source hashes and graph
controls. It is not an FR079 accuracy validation.

## 2. Generate only the new loop-enabled run

These exact matcher settings mirror the existing
`fr079-continuous-noloop-001` invocation: continuous-cost Ceres,
`match-every=5`, `field-every=15`, `map-stride=1`,
`match-stride=4` and EDT truncation 0.5 m. An incompatible source or
option is rejected by the comparison tool, not silently analyzed.

```bash
export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
test -f "$FR079_LOG" || { echo "Missing FR079_LOG" >&2; exit 1; }
mkdir -p apps/demo/maps reports

docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-continuous-loop-001 \
  --run-directory --continuous-cost \
  --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure
```

Do **not** rerun or delete `fr079-continuous-noloop-001`.
The new folder `apps/demo/maps/fr079-continuous-loop-001/` contains
the original frontend artifacts and three new files:

- `fr079-continuous-loop-001.loops.json` — hypotheses, rejections,
  source class, GTSAM control/loop poses, graph cost and shift diagnostics.
- `fr079-continuous-loop-001.graph_control.slammap` — odometry-only graph.
- `fr079-continuous-loop-001.graph_optimized.slammap` — loop graph.

The new `.report.json` includes the loop counts and graph map
fingerprints; its original `map` field remains the causal Ceres map.
The `.review.json` and console output list **absolute host paths**
for report, trajectory, scan-match decisions and loop decisions for
upload to ChatGPT. Binary maps are optional larger uploads.

## 3. Validate pairing and produce the comparison

```bash
python3 tools/fr079_loop_compare.py \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --loop apps/demo/maps/fr079-continuous-loop-001 \
  --out reports/fr079-continuous-loop-001-comparison
```

Both flat and foldered historical run IDs are supported.
The tool *refuses* mismatched source SHA-256, Ceres settings,
scan/matcher acceptance counts, incremental pose histories, and frontend
map fingerprints. This ensures the **only intentional change** to the
frontend run was the post-hoc graph stage.

Outputs:

- `reports/fr079-continuous-loop-001-comparison.json`
- `reports/fr079-continuous-loop-001-comparison.csv`
- `reports/fr079-continuous-loop-001-comparison.review.json`

The updated scorecard explicitly reports **three paired trajectory differences**,
all in the shared first-scan gauge, each with mean, median, p95, maximum,
and index of largest displacement for both position (metres) and yaw (degrees):

1. **Original Ceres frontend → LiDAR-loop GTSAM:** how much the mapped
   solution differs from the original no-loop Ceres reconstruction.
2. **Odometry-only GTSAM → LiDAR-loop GTSAM:** how much loop factors
   correct the purely logged-odometry graph. This is the reference for
   the **legacy post-optimization safety warning** (3 m / 0.9 rad).
3. **Original Ceres frontend → odometry-only GTSAM:** exposes divergence
   between the two pre-loop estimation strategies.

These are **paired differences between estimates**, not independent
trajectory error. They do not require rerunning SLAM: both FR079 run
folders already hold the trajectories and graph pose arrays.
The JSON groups the results under `trajectory_comparisons`; CSV flattens
the statistics into explicitly prefixed columns. The old
`maximum_loop_vs_control_shift_m` remains for compatibility but continues
to refer **only** to GTSAM odometry versus GTSAM loops.

The scorecard also reports nearby pair counts, registration attempts,
geometry-verified/accepted loop counts, graph costs, and map fingerprints.
**It does not invent occupied IoU, ATE, RPE or map precision**. For real map-quality scoring, an
independently surveyed reference would be necessary. The FR079
scan-matched archive is at most a correlated agreement proxy.

If the comparison refuses an input, do not weaken its pairing checks;
verify the baseline command/options and exact source data first.

### Re-evaluate the already completed run (no SLAM rerun)

If `reports/fr079-continuous-loop-001-comparison.{json,csv}` already
exist, use a **new** output prefix to avoid overwriting previous results:

```bash
git pull --ff-only
python3 tests/fr079_loop_compare_tools_test.py

python3 tools/fr079_loop_compare.py \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --loop apps/demo/maps/fr079-continuous-loop-001 \
  --out reports/fr079-continuous-loop-001-comparison-v2
```

This reuses the original `*.trajectory.json`, `*.matches.json`,
`*.report.json` and `*.loops.json` artifacts. It creates a fresh
`-v2.json`, `-v2.csv` and `-v2.review.json` and prints the new
host paths for review. **No Ceres, ICP or GTSAM jobs run** during this
comparison.

## 4. Inspect the actual maps

```bash
docker compose up -d --build --force-recreate demo
```

At http://localhost:8765/ in Map Explorer, choose among the original
frontend map, the **odometry-only GTSAM control**, and **LiDAR-loop
GTSAM** map within `fr079-continuous-loop-001/`. The previously generated
`fr079-continuous-noloop-001` map remains browsable. The recorded
LiDAR replay remains the original causal frontend trajectory; the
graph-optimized poses are exported in `.loops.json`, not inaccurately
presented as scan-level measured odometry.

For ChatGPT review, upload `fr079-continuous-loop-001.report.json`,
`.loops.json`, the comparison JSON/CSV, and their review manifests.
Upload maps/PGM only for further visual inspection.

No GitHub Actions are run or enabled.
