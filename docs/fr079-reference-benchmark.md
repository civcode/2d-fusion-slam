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
- `*.pgm` and `*.yaml` — new loop runs visualize the optimized graph map;
  no-loop runs retain the frontend preview. Previously completed previews
  are immutable and are not silently rewritten.

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


## Opt-in experiment: full-heading scan-to-scan loop matching (before submaps)

The original 11-loop benchmark above used the **legacy** scan-to-scan
candidate/matcher path. Preserve those maps and scores as the baseline. The
heading correction is a **separate selectable experiment**, not a retroactive
change to those archived results.

The new loop-registration API is
`register_scans_from_pose_prior(previous,current,initial_previous_from_current,...)`.
The initial transformation is the complete Ceres-estimated **SE(2)** relative
pose. It may contain **any initial heading, including ~180 degrees**. The
matcher bounds the **innovation from that supplied prior**, not the absolute
relative yaw: maximum 0.55 m translational and 0.17 rad angular correction,
and maximum 3 m candidate translation. The consecutive-scan registration
API retains its original small-angle envelope.

New opt-in mode:
`--loop-registration pose-prior-scan`. Its default remains
`legacy-scan`. This flag requires `--lidar-loop-closure`.
In pose-prior mode:

- Candidate discovery uses the Ceres-predicted **spatial** neighborhood
  (2.8 m), **without a relative-heading cutoff**.
- The first three of six registration slots can be assigned to
  near-heading candidates and three to candidates differing by at least
  90 degrees; unused slots fill from the nearest remaining candidates.
  This avoids starving reverse corridor traversals.
- Scan registration searches small corrections **around the full prior**
  rather than treating the absolute relative rotation as a motion limit.
- Existing strict reciprocal inlier, overlap, rank, loop innovation and
  graph-consistency checks are unchanged. A valid large-heading prior
  **does not guarantee** overlapping 180-degree LiDAR views.
- The run's `*.loops.json` records `registration_mode`, the number of
  large-heading (>=90 deg) nearby pairs, registrations, geometrically
  verified candidates and accepted loops. Every attempted loop decision
  carries its `predicted_relative_yaw_rad`. The report includes
  `loop_registration_mode`.

From the same `FR079_LOG` used for the existing Ceres baseline, generate
**a new run ID** without modifying the 11-loop run:

```bash
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-ceresgraph-poseprior-001 \
  --run-directory --continuous-cost \
  --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph \
  --loop-registration pose-prior-scan
```

Run exactly the same reference evaluation as before, with a **new** prefix:

```bash
python3 tools/fr079_reference_benchmark.py \
  --raw-log "$FR079_LOG" \
  --reference-sm-log "$FR079_SM_LOG" \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --optimized apps/demo/maps/fr079-ceresgraph-poseprior-001 \
  --out reports/fr079-reference-poseprior-001
```

Compare its loop diagnostics and errors with the existing
`fr079-reference-benchmark-001` results. Inspect `*.graph_optimized.slammap` in Map Explorer. New loop runs export
the ordinary `*.pgm` from that optimized map; old runs keep their original
frontend-derived `*.pgm`.

### Future submap matcher, deliberately not implemented in this step

The scan-to-scan matcher's bounded **pose-prior correction** is a reusable
geometric registration primitive. A subsequent Cartographer-style
scan-to-submap registration mode should build historical frozen local
submaps, search pose hypotheses against them (including large-drift
alternatives), then add validated factors to **the same GTSAM backend**.

Keep the legacy scan-to-scan mode, the new pose-prior scan-to-scan mode and
future scan-to-submap mode separately selectable for controlled ablation.
Neither the new spatial gate nor a single-scan 82% reciprocal overlap
threshold solves scan-to-submap place recognition or corridor ambiguity.
No improvement in accepted loop count, trajectory agreement, or map
quality is claimed before a real FR079 rerun.


## Third experimental arm: frozen 2D scan-to-submap constraints

The option `--loop-registration scan-to-submap` is an **opt-in** third mode.
The default `legacy-scan` mode and the prior
`pose-prior-scan` results (13 accepted loops in the recorded
FR079 comparison) remain unchanged and available for ablation.

This is an initial **Cartographer-inspired local** scan-to-submap constraint
builder, not a reproduction of Cartographer's branch-and-bound or global place
recognition. It does the following:

1. For each considered historical keyframe, create and cache an **immutable
   0.10 m occupied-endpoint likelihood raster**. Fuse hits from up to
   approximately 22 historical 180-degree scans spanning 32 scan indices
   before/after the anchor, expressed in that anchor's Ceres frame. All of
   these observations must precede the candidate by at least 120 scans.
   Hits need two endpoint observations per raster cell before being used,
   and a compact spatial likelihood kernel allows discretization error.
2. Discover keyframe pairs **without filtering by relative heading**.
   Retain the old Ceres proximity gate of 2.8 m, exclusion window, at most
   six registrations per current keyframe and at most 24 proposed loops.
   Reserve candidate slots for large heading differences, as in
   `pose-prior-scan`.
3. Run a deterministic **coarse-to-fine correlation search** around the
   **full** Ceres SE(2) relative pose, including its arbitrary initial
   heading. The positional correction is limited to 0.54 m and angular
   correction to 0.18 rad; unlike direct scan-to-scan registration, there
   is no absolute rotation cap of 40 degrees. Score on a sparse beam set
   during search and recheck all current LiDAR hits at the final pose.
4. Accept a geometric match only if it has at least 80 supported current
   LiDAR hits, at least 58% current-scan support from the frozen historical
   map, a normalized correlation score >=0.48, and a raster-distance
   proxy RMS <=0.22 m. In addition, use the **smallest eigenvalue and
   eigenvalue ratio of the translation correlation-score curvature** and
   four-sided 0.40 m score-peak checks to reject flat, ambiguous corridor
   ridges. These are deterministic observability **heuristics**, not a
   calibrated covariance or a proof of correct place recognition.
5. Feed verified submap-to-current relative pose measurements into the
   existing **GTSAM pose graph**, represented in the anchor scan's frame.
   They carry the explicit correlated
   `lidar_scan_to_submap_registration` evidence type and nominal
   0.40 m / 0.40 m / 0.24 rad factor sigmas; the graph backend applies
   separate gates, robust loss and the same first-pose anchor. The
   **Ceres-derived sequential factors and frontend maps do not change**.

Unlike the previous 82% *reciprocal scan-to-scan* overlap, the new 58%
criterion is **support of a current scan in an accumulated historical
submap**, not a symmetric two-scan overlap value. It must never be
misrepresented as an equivalent relaxed scan-to-scan gate.

### Reproduce with Docker

From the repository root, after obtaining the unchanged FR079 raw and
published scan-matched archives:

```bash
git pull --ff-only
docker compose build test demo
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(scan_to_submap_correlative|scan_to_scan_bootstrap|fr079_loop_experiment|fr079_reference_benchmark_tools)$'

export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
export FR079_SM_LOG="$PWD/.build/datasets/fr079-sm.log.gz"
mkdir -p apps/demo/maps reports

docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-ceresgraph-submap-001 \
  --run-directory --continuous-cost \
  --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph \
  --loop-registration scan-to-submap

python3 tools/fr079_reference_benchmark.py \
  --raw-log "$FR079_LOG" \
  --reference-sm-log "$FR079_SM_LOG" \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --optimized apps/demo/maps/fr079-ceresgraph-submap-001 \
  --out reports/fr079-reference-submap-001
```

Inspect
`fr079-ceresgraph-submap-001.graph_optimized.slammap`
when judging the optimized map (new `*.pgm` previews use this geometry).

### Diagnosis and reproducibility

The `*.loops.json` artifact includes the selected mode,
numbers of built historical submaps, submap registration attempts,
geometrically verified candidates, ambiguous-geometry rejections and
accepted large-heading loops. Per-attempt records include the number of
historical sweeps, support, correlation, approximate distance RMS and
translation-curvature eigenvalue diagnostics. `submap_regression_probes`
evaluates the following **previously rejected, unlabeled** FR079 pairs,
provided the recording contains exactly 3,118 scans:

- 1040 -> 1288
- 2704 -> 2936
- 536 -> 872

Those probes are **diagnostic-only**; even a passing result is not inserted
into the graph and does **not** establish a true loop closure. The usual
candidate finder and quality gates independently decide graph constraints.
Unit tests use a synthetic reverse-direction revisit with distinctive
transverse structure and a featureless corridor that must remain rejected.

**Known limitations:** The new submaps are frozen **endpoint likelihood
rasters**, not full probabilistic free/occupied Cartographer submaps. Search
is deterministic coarse-to-fine but **not branch-and-bound**. Initial
place recognition is still Ceres-position-gated, so substantial drift may
miss true revisits. Correlation scores are sensor- and scan-dependent;
curvature is not probabilistic uncertainty and repeating corridor geometry
may still be deceptive. The frontend's pose errors can distort the
historical submaps, and all loop evidence is correlated with Ceres LiDAR.
Visual and independent accuracy validation remain important.

The previously published FR079 heading-aware benchmark remains the
baseline to beat: **13 loops, 1.173 m first-pose ATE RMSE and 0.846 m rigid
ATE RMSE relative to the published scan-matched estimate** (which is not
surveyed ground truth). No new FR079 score is claimed without running
this new mode on the real dataset.


## Fourth and fifth experimental arms: full-trajectory selection and global retrieval

These opt-in additions deliberately **preserve** the previous
`--loop-registration scan-to-submap` implementation and its
24-loop FR079 benchmark. They do not alter the Ceres frontend or
frontend-relative sequential factors. The published `fr079-sm`
poses remain evaluation-only and are never used to recognize or validate
loop candidates.

### A: Distributed scan-to-submap closure

Select `--loop-registration scan-to-submap-distributed`.

The old scan-to-submap experiment quit candidate discovery as soon as
24 loops had been proposed (on FR079, after scan 1768). This mode
**processes all keyframes** through the end of the log, collects the
best geometrically verified candidate per current keyframe, then
chooses **at most 24 globally distributed factors**:

- Partition the full scan-index range into **24 narrow time slots**
  (eight bins with three slots each), so high-scoring early visits cannot
  use the entire 24-factor budget.
- Allocate up to one factor per slot by current-scan support followed by
  registration error.
- Fill any unused slots from other verified candidates. Two constraints
  are considered redundant **only when both their historical anchors
  and current indices are within 24 scans**. This preserves separate
  later traversals returning to the same historical anchor.
- Keep the historical 2.8m Ceres-relative position gate, 0.54m/0.18rad
  local search and all original submap observability checks unchanged.

This makes the effects of **full-pass temporal coverage** independently
measurable, without conflating them with place recognition.

### B: Global descriptor-retrieved scan-to-submap closure

Select `--loop-registration scan-to-submap-global`.

In addition to the distributed local search, an **appearance-based**
candidate generator compares shape signatures built from frozen local
LiDAR submaps at every 16th current scan:

- A normalized 16-bin histogram of pairwise distances between
  geometry-driven, farthest-point-sampled occupied endpoints provides a
  compact **translation- and rotation-invariant (up to rasterization)**
  geometric descriptor. It does not consume Ceres
  global XY or heading to rank candidate places.
- Keep only descriptor similarity >=0.82; retrieve up to two
  temporally separated historical clusters, with a nearby second anchor
  where available. This small budget limits expensive global matching.
- Each retrieved hypothesis can begin **outside the 2.8m gate**. A
  coarse-to-fine occupancy correlation searches up to **4.5m** and
  **0.55rad** around the full Ceres relative pose; the local matcher
  remains unchanged.
- A retrieved match needs >=70% current-scan submap support, correlation
  >=0.60, the same 2D score-curvature and four-sided score-peak
  observability tests, and **agreement within 0.55m / 0.12rad between
  two neighboring historical anchor matches** before it can become a
  loop proposal. These neighboring scans are correlated, not independent
  truth; agreement is a conservative heuristic.
- GTSAM receives global matches under a **distinct correlated evidence
  type** and stronger 0.60m/0.60m/0.30rad factor sigmas. Only this
  provenance can tolerate up to 4.6m / 0.60rad discrepancy from the
  frontend trajectory. The original local graph gates remain unchanged.
  Robust loss and the first-pose anchor remain active.

**Limitations:** The pair-distance histogram is inexpensive but can
confuse similarly shaped corridors; it is not semantic or learned place
recognition, nor Cartographer's exhaustive branch-and-bound matcher.
Spatial retrieval and larger search windows do not prove that a loop is
correct. Adjacent historical submaps share most of their observations,
so cross-anchor agreement is not independent validation. Large-drift
events outside 4.5m/0.55rad can still be missed; factor budgets may
still exclude meaningful matches. **Do not** interpret an increase in
loop count as a guaranteed improvement in physical map accuracy.

### Reproducible evaluation

Reuse the *same* FR079 source logs and all frontend flags for both arms.
Do not overwrite the old `fr079-ceresgraph-submap-001` or
`fr079-ceresgraph-poseprior-001` runs.

```bash
git switch experiment/fr079-ceres-ablation-beam-sweep
git pull --ff-only
docker compose build test demo
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R '^(scan_to_submap_correlative|scan_to_scan_bootstrap|pose_graph|fr079_loop_experiment|fr079_reference_benchmark_tools)$'

export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
export FR079_SM_LOG="$PWD/.build/datasets/fr079-sm.log.gz"
mkdir -p apps/demo/maps reports

# A: Full trajectory discovery, original pose-based candidate search
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-ceresgraph-distributed-002 \
  --run-directory --continuous-cost --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph \
  --loop-registration scan-to-submap-distributed

python3 tools/fr079_reference_benchmark.py \
  --raw-log "$FR079_LOG" --reference-sm-log "$FR079_SM_LOG" \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --optimized apps/demo/maps/fr079-ceresgraph-distributed-002 \
  --out reports/fr079-reference-distributed-002

# B: Full-trajectory selection + appearance-retrieved global proposals
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-ceresgraph-global-002 \
  --run-directory --continuous-cost --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph \
  --loop-registration scan-to-submap-global

python3 tools/fr079_reference_benchmark.py \
  --raw-log "$FR079_LOG" --reference-sm-log "$FR079_SM_LOG" \
  --baseline apps/demo/maps/fr079-continuous-noloop-001 \
  --optimized apps/demo/maps/fr079-ceresgraph-global-002 \
  --out reports/fr079-reference-global-002
```

The `*.loops.json` records `last_keyframe_searched`,
`verified_candidate_pool`, `late_verified_candidates`,
`late_selected_loops` (current scan index >=1769), global
appearance retrieval/registration/corroboration/acceptance counts,
per-decision descriptor similarity and corroboration results, and
the original submap curvature diagnostics. A mode with zero late
accepted loops is not considered a success just because it processed
later scans: inspect per-phase candidates and map alignment.

Compare each new run against the *existing* results of
`fr079-reference-submap-001` (24 loops, 0.7832m first-pose
ATE RMSE, 0.6261m rigid XY RMSE; 0.1792 first-pose occupied
endpoint IoU and 0.1747 rigid XY IoU) and the heading-aware
13-loop baseline (1.1727m/0.8462m ATE RMSE; 0.2100/0.1325 IoU).
These metrics compare to the published scan-matched reference, **not**
surveyed ground truth. Local 10- and 50-scan RPE, large graph-shift
warnings, actual loop locations and the visual map remain necessary
checks for false or overly strong constraints.


## Scoring real map geometry, not just trajectory endpoints

`fr079_reference_benchmark.py` evaluates trajectory-derived **occupied
endpoint sets**, not the exported FUSMAP2D cells. For reference-relative
geometric map diagnostics, use `tools/fr079_map_compare.py` on the
`*.graph_optimized.slammap` files. Its 5 cm Freiburg CARMEN map archive
input is **fr079-maps.tgz**, not the pose log `fr079-sm.log.gz`.

```bash
export FR079_MAPS_ARCHIVE="$PWD/.build/datasets/fr079-maps.tgz"
# This optional map-alignment tool needs numpy, scipy and matplotlib.
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy scipy matplotlib
mkdir -p reports
# Align the common, unchanged Ceres frontend just once to the supplied map.
# Do NOT fit a separate transform to each optimized graph.
frontend=fr079-ceresgraph-poseprior-001
python3 tools/fr079_map_compare.py \
  --repo "$PWD" \
  --slammap "apps/demo/maps/$frontend/$frontend.slammap" \
  --reference-archive "$FR079_MAPS_ARCHIVE" --reference-cm 5 \
  --out reports/fr079-common-frontend-map-alignment

for arm in poseprior-001 distributed-002 global-002; do
  name="fr079-ceresgraph-$arm"
  python3 tools/fr079_map_compare.py \
    --repo "$PWD" \
    --slammap "apps/demo/maps/$name/$name.graph_optimized.slammap" \
    --reference-archive "$FR079_MAPS_ARCHIVE" --reference-cm 5 \
    --fixed-alignment-json reports/fr079-common-frontend-map-alignment.json \
    --out "reports/$name-map-geometry"
done
```

The fixed transform from the *common Ceres frontend* (not any preferred
optimized graph) is reused for all three output maps. This avoids separate
candidate-specific map fitting that could hide global shape distortion.

Compare `wall_metrics.wall_f1_within_0p15m` and
`wall_f1_within_0p30m` (higher is better),
`wall_metrics.symmetric_mean_m` and `symmetric_p95_m` (lower is
better), occupied IoU and known-region coverage. Examine all three
overlay PNGs at **the same metric scale**, in particular around loops and
parallel corridors.

**Interpretation constraints:** The shared alignment is fitted to the
unchanged frontend using a map derived from the same log and then applied
without refitting to the three graph maps. This remains *reference
agreement*, not independent physical wall accuracy. The Freiburg map frame has no independently established
transform into our map frame. Wall doubling, thickening, and multi-pass
consistency need dedicated evaluations against independent geometry or
reserved scan observations; they are not separately scored here. Neither
new metrics nor existing metrics should be tuned to force a favored run
to win.

Historical `*.pgm` previews were generated from the Ceres frontend.
To generate a **new** preview for an already completed optimized graph
map without rerunning SLAM and without overwriting previous files:

```bash
name=fr079-ceresgraph-poseprior-001
python3 tools/slammap_to_pgm.py \
  --slammap "apps/demo/maps/$name/$name.graph_optimized.slammap" \
  --out-prefix "apps/demo/maps/$name/$name.graph_optimized"
```

This emits `*.graph_optimized.pgm` and `*.graph_optimized.yaml` from
the verified authoritative graph map. The export is a lossy visualization
and does not modify `*.slammap`.
