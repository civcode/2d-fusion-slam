# Synthetic estimated-map accuracy benchmark (FUSSYNTHBENCH1)

**Status:** deterministic C++ scan-to-map *front-end* benchmark wired into the
repository's existing simulator, occupancy integrator, immutable EDT, reference
scan matcher and FUSMAPACC1 truth evaluator. This is the first end-to-end
**estimated-pose** synthetic mapping benchmark, not a full wheel/gyro-fused
GTSAM offline SLAM pipeline or a Raspberry Pi measurement.

## Reproduce (no GitHub Actions, Ceres, GTSAM or network)

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=OFF -DSLAM_ENABLE_CERES=OFF \
  -DSLAM_ENABLE_GTSAM=OFF
cmake --build build --target \
  slam_synthetic_mapping_benchmark slam_map_accuracy_cli \
  slam_synthetic_eval_fixture slam_map_accuracy_test --parallel 2
ctest --test-dir build -R 'map_accuracy|synthetic_mapping_benchmark' \
  --output-on-failure

# Outputs must NOT already exist; no automatic clobbering of previous reports.
./build/slam_synthetic_mapping_benchmark apartment /tmp/fusion-apartment-run1
./build/slam_synthetic_mapping_benchmark warehouse /tmp/fusion-warehouse-run1

cat /tmp/fusion-apartment-run1/benchmark.json
./build/slam_map_accuracy_cli apartment \
  /tmp/fusion-apartment-run1/estimated.slammap
~~~

Replace output directories on subsequent runs, or explicitly archive and
remove old results. All three reports and maps are deterministic for a
given binary/configuration/compiler environment; tests perform two identical
runs and compare report bytes.

## What is actually measured

1. The exact **synthetic apartment** (~15 x 12 m) or **warehouse**
   (~30 x 20 m) fixture raycasts 360 angular beams per stationary scan
   along a fixed loop-like route. Exact pose is only used by the simulator
   and by the final *after-the-fact* pose error evaluation.
2. The first exact pose supplies **only the global gauge/initial anchor**.
   Subsequent estimated poses use deterministically biased *relative*
   simulated odometry (1.8% forward-scale distortion, lateral and yaw
   perturbations), **not** exact simulated poses or a truth-correcting
   trajectory.
3. The odometry trajectory creates an initial authoritative occupancy map
   from actual simulated laser hit rays, clearing traversed cells through
   the existing occupancy insertion policy. Invalid/no-return observations
   are **not** silently treated as hits or trusted free-space rays.
4. A **frozen training map** is built from **even** scan indices using
   those same odometry estimates. Odd-index scans are excluded entirely
   from construction of this matcher reference map. Only odd scans are
   matched via the real \`slam::matching::match_reference\` over the
   original immutable EDT. No Ceres adapter or graph optimization is
   secretly substituted.
5. Conservative geometry/overlap/rank/fingerprint and innovation gates
   decide whether each held-out local scan correction is accepted;
   otherwise the predicted pose is retained. The corrected trajectory
   then builds a **new map** using all recorded scan rays.
6. Both generated FUSMAP2D maps are independently evaluated against the
   exact original synthetic wall geometry using \`slam_map_accuracy\`.
   Translation RMSE/p95 and yaw RMSE are also calculated against exact
   synthetic poses **after** the map generation.

The split prevents **exact same-scan** occupancy reuse in the frozen
matching reference; adjacent scans and the shared odometry estimates
remain spatially and statistically correlated. The benchmark therefore
measures a bounded two-pass local geometric front-end, **not** independent
real-world SLAM validation and not a cross-dataset generalization test.

### Files per scene

| File | Meaning |
| --- | --- |
| \`baseline.slammap\` | Noisy-odometry map (not truth) |
| \`estimated.slammap\` | Scan-matched estimated-pose map (not truth) |
| \`benchmark.json\` | FUSSYNTHBENCH1: scan counts, outcomes, pose errors, both fully nested FUSMAPACC1 accuracy reports |

Both map bundles use \`truth_class=synthetic_estimated\`, even though
the **reference scene** has \`synthetic_exact\` truth. The evaluator accepts
this *specific* generated-map label only when the independent reference
source ID and metric map frame also match. The JSON reference class still
says \`synthetic_exact\`, reflecting the reference geometry, not an oracle
generated map.

## Interpretation

Use the **difference** between baseline and candidate IoU, recall,
symmetric mean/p95 nearest-wall distance, and absolute trajectory RMSE.
A Ceres/EDT matcher accepting a scan does **not** prove better real geometry.
A candidate may have worse IoU or ATE, in which case the report must preserve
the regression rather than claim an improved SLAM algorithm.

FUSMAPACC1 distinguishes known and unknown cells. Unknown reference wall
cells reduce IoU/recall and are **not** mislabeled free or ignored; missing
distance measurements remain JSON \`null\`. The comparison uses fixed map
frame, resolution, extent, and no best-fit map alignment/rotation. There is
no automatic map generation promotion: both maps and metrics are reported,
and an operator/experiment can assess the regression before publishing.

The CTest integration checks that **some held-out matches actually accept**,
that the two exports contain authoritative FUSMAP2D maps which the separate
CLI scores identically, that two identical runs match byte-for-byte, and
that a pre-existing output directory cannot be overwritten. Both apartment
and warehouse fixtures are exercised. Results from an ideal/oracle-pose
fixture are **not** used for the candidate mapping experiment.

## Limitations and next milestone

- Each scan is **stationary**, so this does not validate LD19 packet
  timing or moving-scan deskew.
- Relative odometry is synthetic, not a measured wheel sensor or calibrated
  gyro; only odd scans are corrected. No recovery search, GTSAM loop closure,
  posterior covariance, real IMU bias or live streaming is implemented.
- The baseline frozen map is made from the same traversal as the candidate,
  so correlation and drift can persist. No claim of real FR079 accuracy,
  real map ground truth, or pose error against surveyed hardware is made.
- Full native CTest execution must be recorded before publishing numerical
  results. The first milestone is *implementing the reproducible run and
  regression tests*; measured benchmark numbers should come from the
  actual native executable, never from an oracle fixture or an unexecuted
  theoretical estimate.

**Next:** run on compatible local C++20/Eigen toolchain, inspect actual
\`benchmark.json\` in both scenarios, compare a Ceres solver variant on an
identical frozen train/query split, then add GTSAM loop estimates with
independent factor provenance as a separate experiment.
