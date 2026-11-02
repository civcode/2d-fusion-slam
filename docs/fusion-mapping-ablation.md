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
  original exact-cell `FUSMAPACC1` metrics, and 10/25/50 cm
  wall-distance precision/recall/F1.
- `mapping_ablation/*.slammap` — 24 additional independent FUSMAP2D
  maps (24 files; substantial extra disk and build time). These are
  **not** included in the regular Map Explorer's method allowlist.

```sh
python tools/fusion_mapping_ablation.py \
  --run apps/demo/fusion/warehouse_asymmetric-map-ablation-001 \
  --csv apps/demo/fusion/warehouse_asymmetric-map-ablation-001/mapping_ablation_summary.csv
```

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

The subset-window maps are built only from the corresponding scans but are
scored against the **entire exact reference scene**. Their recall, IoU and
coverage necessarily penalize unvisited walls: do not compare an early-only
score directly with a late-only score as a region-local accuracy claim.
Compare different hit/free evidence or deskew choices *within the same*
frame window. To distinguish regional error causes, examine the window's
wall-distance, predicted-wall precision, known coverage and raw counts,
then use a region-masked evaluator in any follow-up study.

The report gives observed post-hoc measurements, not a policy to choose
weights during SLAM. A higher F1@25 cm or better yaw RMSE must not be
described as ground-truth-calibrated sensor noise, and a larger number
of occupied cells is not automatically better reconstruction.

### Local checks

```sh
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(map_accuracy|fusion_mapping_ablation_tools)
```

No GitHub Actions are configured or dispatched by this experiment.
Recorded FR079 inputs are outside the provenance scope of these scores.


# Separate, longer native experiment (24 maps, possibly several minutes):
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^fusion_mapping_ablation
```

No GitHub Actions are configured or dispatched by this experiment.
Recorded FR079 inputs are outside the provenance scope of these scores.

```

No GitHub Actions are configured or dispatched by this experiment.
Recorded FR079 inputs are outside the provenance scope of these scores.
