# Ground-truth 2D map comparison (FUSMAPACC1)

**Status:** deterministic, read-only C++ evaluation for the repository's two
**exact synthetic** reference scenes. The output is map reconstruction evidence,
not a localization ATE/RPE score, a real FR079 accuracy result, or proof that a
robot surveyed the physical environment.

## Metrics and policy

`slam_map_accuracy_cli` reads an authoritative, checksummed `FUSMAP2D`
`.slammap` bundle and rasterizes the exact scene segments from
`slam::fixtures::apartment()` or `slam::fixtures::warehouse()`.
Map grid width/height/resolution and **rotated** `map_from_grid` origin are
respected. A cell is reference-wall-occupied if its **centre** is within
half of a cell diagonal of a wall segment. This explicitly measures wall
*surface occupancy*, **not** unknown obstacle interiors or full traversable
floor area; raster radius and resolution affect IoU.

Generated map occupancy is classified using the authoritative observed mask
and log odds: `> 0` is predicted wall, `< 0` predicted non-wall, and
unobserved or exactly zero is **unknown** (not free). No dilation or
post-hoc registration is applied to improve apparent agreement.

The JSON report includes:

- `occupied_iou = TP/(TP+FP+FN_observed_free+unknown_wall)`: all
  missed reference wall cells, **including unknown ones**, reduce the score.
- `precision = TP/(TP+FP)`, `recall = TP/reference_wall_cells`,
  `f1` and `known_coverage = classified_cells/all_cells`.
  `reference_wall_coverage` measures only how many true-wall cells are
  classified (correctly **or** incorrectly); it is not accuracy.
- Counts for TP/FP/FN/TN and **unknown_wall/unknown_free** separately.
- Exact Euclidean nearest-wall distance from each predicted wall cell to
  the rasterized true walls, and vice versa, using a deterministic 2-pass
  squared EDT. `symmetric_mean_wall_distance_m` is the average of the
  two directed means. `symmetric_p95_wall_distance_m` averages their
  directed 95th percentiles. These distances are between wall cell centres,
  not a surveyed continuous-surface Hausdorff distance.
- Map revision/fingerprint, reference fixture ID, resolution/dimensions,
  frame-alignment assumption and a versioned `FUSMAPACC1` report identifier.

Undefined denominators and missing wall-distance evidence serialize as JSON
`null`, not fake zero or non-finite numbers. An entirely unknown map
therefore has zero wall IoU and zero coverage, but null precision and distance.

## Reproduce without Ceres, GTSAM, ROS or GitHub Actions

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_ENABLE_GTSAM=OFF
cmake --build build --target \
  slam_map_accuracy_test slam_synthetic_eval_fixture slam_map_accuracy_cli
ctest --test-dir build --output-on-failure -R '^map_accuracy'

# Reproducible END-TO-END plumbing check, not a SLAM accuracy result.
./build/slam_synthetic_eval_fixture apartment /tmp/apartment-oracle.slammap
./build/slam_map_accuracy_cli apartment /tmp/apartment-oracle.slammap \
  > /tmp/apartment-accuracy.json
# Also supported: 'warehouse' with the corresponding generated fixture.
~~~

The fixture reconstructs a map by inserting deterministic simulated stationary
LiDAR hits with **exact oracle robot poses**. This intentionally demonstrates
a real occupancy-building and binary import path, but **does not execute SLAM
pose estimation** and must not be published as SLAM accuracy. To score an
actual synthetic SLAM run, export *its own generated* FUSMAP2D with the
same reference frame and provenance and run the identical CLI.

CTest provides deterministic ideal/perturbed maps, rotated origins, warehouse
geometry, unknown/unclassified regions, exact binary roundtrip and negative
source/frame/crop/budget checks; the optional Python CLI smoke uses the
end-to-end fixture and validates JSON and refused mismatches.

## Fail-closed reference identity

The evaluator refuses a map unless **all** of the following match:

1. `provenance.truth_class` is `synthetic_exact` for oracle-produced
   fixtures or `synthetic_estimated` for estimated-pose synthetic maps;
2. `provenance.metric_origin_verified == true`;
3. `provenance.frame_id == "map"`;
4. `provenance.source_id == scene.id` (e.g. `synthetic-apartment-v1`);
5. The map covers **every reference wall endpoint**, including its boundary,
   so no metric can silently omit harder portions of the scene.

The `synthetic_estimated` map class means the *reference geometry* is exact
synthetic truth; it **does not** assert that SLAM reconstructed the map
correctly. See [estimated-pose synthetic mapping](synthetic-mapping-benchmark.md)
for the measured odometry-baseline versus scan-matched comparison. The
oracle-pose demonstration remains only a file-format plumbing check.

These metadata fields are **declarations by the producer**, not authenticated
physical calibration. Benchmark producers must demonstrate that they used the
fixed exact synthetic frame and did not use reference truth to tune/evaluate
the same candidate (the oracle-fixture is explicitly excluded from SLAM
performance claims). Map bundles use FNV for corruption detection, not
adversarial authentication.

No map rescaling, reflection, ICP, rotation optimization, or best-fit cropping
is performed. A same-frame metric comparison is only meaningful after
independently establishing registration. FR079 reference and logged laser
frame alignment remain **unverified**: this tool deliberately refuses
unverified real-data maps and cannot report their ground-truth IoU. For real
datasets, first obtain externally surveyed maps/frame registration and a
separate truth-class contract.

The evaluation memory/work is capped at four million cells by default; the
map itself and the registry are never modified. The output is JSON on stdout,
so report storage and comparison remain explicit caller actions.
