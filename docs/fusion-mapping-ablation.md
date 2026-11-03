# Fixed-graph occupancy mapping and re-deskew ablation

**Status:** opt-in, synthetic-only, offline diagnostic. This experiment does not
change the default SLAM estimator, accepted loop factors, its original four
non-graph maps or the standard `graph_optimized.slammap` result.

## Purpose and controlled variables

In the asymmetric warehouse, the two-loop detector lowered final-segment
pose error, but exact 5 cm occupied IoU fell from ~0.06625 to ~0.06391.
The same run had lower yaw RMSE and lower mean wall-distance error.
This experiment separates two hypotheses:

1. **Evidence integration:** hit/free log-odds settings change the occupied/
   free classifications without changing any scan, pose, grid dimensions,
   reference raster or beam selection.
2. **Deskew motion model:** the existing scan was deskewed at beam acquisition
   times using original gyro/wheel odometry. After GTSAM has produced corrected
   scan-midpoint poses, a *smooth correction field* can be interpolated between
   optimized scan nodes and applied to the original high-rate fused trajectory.
   Re-running the existing verified deskew operation then changes intra-scan
   beam positions while keeping **the same optimized scan-midpoint poses**.

The correction model is **not an independent motion measurement**, is not a
second graph optimizer, and does not consult synthetic truth. Its corrections
are clamped to the first/last optimized scan outside the scan-midpoint interval
while intra-scan motion continues to come from the high-rate gyro/wheel
trajectory. The native experiment checks that corrected trajectory samples
still agree with the GTSAM pose at every scan midpoint and refuses missing or
untrusted beam timing.

All 24 reconstructions use exactly the same graph poses. No mapping variant
can update the pose graph. The reference truth is accessed only **after**
each map is fully constructed, by the scoring API.

## Run

Build a GTSAM-enabled test image. Use the same sensor, Ceres and LiDAR loop
configuration as the previous two-loop run, but use a new directory:

```sh
git pull --ff-only
docker compose build test
mkdir -p apps/demo/fusion
docker compose run --rm \
  -v "$PWD/apps/demo/fusion:/output" \
  --entrypoint ./build-docker/slam_offline_fusion \
  test warehouse_asymmetric /output/warehouse_asymmetric-map-ablation-001 \
  --continuous-cost \
  --match-every 1 --match-stride 1 \
  --field-every 15 --map-stride 1 \
  --lidar-only-bootstrap --lidar-only-predict-first \
  --lidar-loop-closure \
  --mapping-ablation
```

**A new native run is necessary:** the existing browser replay saves
*downsampled* laser endpoints, not the complete timed 240-beam scan and
high-rate gyro/wheel pose history needed for re-deskew. The new run must
have the same flags as the existing baseline for a controlled comparison.

Every run with `--mapping-ablation` produces:

- `fusion_report.json`, `fusion_replay.json`, `trajectories.json`, and
  the original five `.slammap` maps, unchanged by the ablation mode.
- `mapping_ablation.json` — `FUSMAPABL1` report with each variant's
  reference identity, frame window, evidence settings, map fingerprint,
  original full-scene exact-cell `FUSMAPACC1` metrics, and 10/25/50 cm
  wall-distance precision/recall/F1, **plus spatially region-masked**
  IoU/precision/recall/wall-distance and tolerance scores using a
  common original-ray observation footprint.
- `mapping_ablation/*.slammap` — 24 additional independent FUSMAP2D
  maps (24 files; substantial extra disk and build time). These are
  **not** included in the regular Map Explorer's method allowlist.

```sh
# With the project virtual environment activated (see the root README):
mkdir -p reports
python tools/fusion_mapping_ablation.py \
  --run apps/demo/fusion/warehouse_asymmetric-map-ablation-001 \
  --csv reports/warehouse_asymmetric_mapping_ablation_summary.csv
```

The Docker container may create run directories owned by root on the host.
Use the run directory only as input and write CSV exports into a
host-user-writable directory (such as `reports/`), avoiding permission
errors. The report uses only Python's standard library; no pip dependencies
are needed for this analysis.

The reader verifies the original graph map fingerprint equals the full-run
original-deskew, baseline-odds reconstruction. Its local checks reject
missing/duplicate variant IDs, inconsistent scan windows, wrong scene,
missing maps and invalid/nonmonotonic wall-tolerance scores.

### Controlled variants (2 × 3 × 4)

| Dimension | Levels |
| --- | --- |
| Deskew | original gyro/wheel deskew; graph-corrected high-rate motion model |
| Contribution window | all frames; early/middle first 88%; late final 12% |
| Occupancy evidence | baseline hit +0.85 / free −0.40; stronger hit +1.20 / free −0.40; gentler free hit +0.85 / free −0.20; combined hit +1.20 / free −0.20 |

All evidence values are *configured log-odds updates*, not calibrated sensor
probabilities. The grid remains **5 cm**. No map threshold, scan decimation,
graph uncertainty or matching rule is tuned inside the experiment.

### Accuracy definitions and interpretation

**Exact occupied IoU** stays the original `FUSMAPACC1` metric:
`TP/(TP+FP+observed-free truth-wall cells+unknown truth-wall cells)`.
It does not use tolerance or morphological dilation.

**Distance-tolerant wall precision** is the fraction of generated occupied
cell centres within the requested Euclidean radius of *any exact reference*
wall cell centre. **Distance-tolerant wall recall** is the fraction of
reference wall cell centres within that radius of *any generated occupied*
cell centre. Their harmonic mean is F1. The requested radii are 0.10 m,
0.25 m and 0.50 m. They complement but do not replace occupied-cell IoU.

For comparability with earlier reports, each variant **retains its
full-scene metrics**. Subset-window maps remain penalized for unvisited
scene geometry in those original fields. An additional `region_map`
score evaluates only the cells of a **spatial observation footprint**,
created *once per window* from the original gyro/wheel-deskewed baseline
map's ray-observed cells, with an exact Euclidean **25 cm dilation**.
Its region truth walls are independently rasterized and clipped to that
footprint only at post-hoc scoring time.

The footprint contains the observed ray corridors **and** a fixed margin
so small pose errors do not automatically remove a missed wall from
the denominator. Within each window, every evidence level and both deskew
variants share precisely the same mask. It is based on estimated poses
and observed scan rays, never oracle truth or occupancy classification
outcomes. The region can still miss reference walls beyond the padded
ray coverage; this is a measured local *observability footprint*, not
a physical visibility oracle or a complete coverage claim.

Region IoU, occupied precision/recall and mean/P95 wall distance treat
outside-mask cells as excluded from both prediction and reference.
Unknown truth-wall cells **inside** the footprint continue to count as
misses. Region known coverage divides by footprint cells, not full-grid
cells. Region tolerance F1 at 10/25/50 cm is likewise computed on
the same masked truth and prediction, not globally. The `region_footprint`
object in every variant records the mask definition, radius, number
of eligible cells and deterministic decimal FNV-1a mask fingerprint
(for repeatability checks, not cryptographic authentication);
`region_map.counts.truth_wall` records the relevant
reference-wall denominator.

Because the robot may revisit the same space, early and late
footprints can overlap. Do not add their scores or interpret
early/late recall as percentages of disjoint physical walls.
Compare evidence choices *within a given region*, and report both
full-scene and region-masked metrics.

The report gives observed post-hoc measurements, not a policy to choose
weights during SLAM. A higher F1@25 cm or better yaw RMSE must not be
described as ground-truth-calibrated sensor noise, and a larger number
of occupied cells is not automatically better reconstruction.

## Seven-scene generalization sweep (same detector, fixed poses per scene)

The experimental evidence settings remain **opt-in**; the standard
hit +0.85/free −0.40 remains the default. Use the same LiDAR-loop
detector, Ceres options, scan sampling and 5 cm map grid for all
seven synthetic scenes. Each scene solves its graph **once** and
evaluates the four evidence settings against the identical graph
poses and LiDAR scans. The number of accepted loops may legitimately
differ per scene; never invent or force exactly two loops.

Build the GTSAM-enabled Docker test image, then explicitly run all
seven scenes under a fresh run prefix. This is local execution,
not GitHub Actions.

```sh
git pull --ff-only
docker compose build test
mkdir -p apps/demo/fusion reports

docker compose run --rm \
  -v "$PWD/apps/demo/fusion:/output" \
  --entrypoint python3 test \
  tools/fusion_mapping_sweep.py run \
  --binary ./build-docker/slam_offline_fusion \
  --runs-dir /output \
  --prefix mapping-generalization-001
```

Every run is named
`apps/demo/fusion/mapping-generalization-001-<scene>/`
and contains the normal fusion artifacts and 24 mapping-ablation
maps. **The sweep refuses to overwrite any existing output directory.**
Use a new prefix for a repeat or experiment revision.

Activate the [repository-local Python venv](../README.md#python-virtual-environment-and-local-analysis)
on the **host** and aggregate the finished runs without rerunning SLAM:

```sh
source .venv/bin/activate
python tools/fusion_mapping_sweep.py summarize \
  --runs-dir apps/demo/fusion \
  --prefix mapping-generalization-001 \
  --csv reports/mapping-generalization-001-full.csv \
  --region-csv reports/mapping-generalization-001-regions.csv
```

The full-scene CSV has **28 rows** (7 scenes × 4 evidence
settings, original deskew, full scan window). The region CSV has
**84 rows** (7 × 3 windows × 4 evidence levels, original deskew).
Both include accepted loop counts. The tool fails closed on
missing/duplicated variants, missing region metrics, wrong scene,
changed graph/estimator settings or map fingerprint mismatch.
It reports unweighted macro averages and counts of scenes showing
occupied IoU regression, relative precision decreases beyond 10%,
or worsening mean wall distance. These are **diagnostic guardrails**,
not automatic acceptance thresholds or evidence of calibrated
sensor probabilities. We cannot conclude generalization from
the asymmetric warehouse alone; inspect all seven result rows and
both precision and recall before considering any default change.

The sweep uses only Python's standard library and writes CSVs
to the host-user-owned `reports/` directory, not into Docker-created
root-owned run directories.

### Local checks

The lightweight checks run without the heavy 24-map integration:

```sh
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(map_accuracy|fusion_mapping_ablation_tools|fusion_mapping_sweep_tools)
```

The separate native test builds and scores all 24 maps:

```sh
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^fusion_mapping_ablation$'
```

No GitHub Actions are configured or dispatched by this experiment.
Recorded FR079 inputs are outside the provenance scope of these scores.

```

The separate native test builds and scores all 24 maps:

```sh
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^fusion_mapping_ablation$'
```

No GitHub Actions are configured or dispatched by this experiment.
Recorded FR079 inputs are outside the provenance scope of these scores.
