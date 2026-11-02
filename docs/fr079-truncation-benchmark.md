# FR079 distance-field truncation and capture-range benchmarks

This experiment isolates three limits: Euclidean distance-field (EDT) truncation, observed-cell support, and Ceres search bounds. Default truncation remains **0.7 m**. The SLAM estimator still starts with an empty map and never uses logged laser poses or the supplied reference occupancy map to locate itself.

**Evaluation provenance:** FR079 supplied maps are correlated scan-matched reconstructions, not independent ground truth. Controlled perturbations use accepted poses of the same run as *proxy anchors*, not independently surveyed poses. Proxy recovery cannot establish true ATE/RPE or loop-closure quality.

## Build and test locally

Run from repo root, on the latest main. No GitHub Actions are needed.

~~~sh
git pull --ff-only
export BUILD_JOBS=28 SLAM_DOCKER_JOBS=28
docker compose build test

docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'distance_field|match_ceres|fr079_offline_slam|fr079_capture_benchmark|fr079_benchmark_tools'
~~~

The capture CTest uses a deterministic synthetic CARMEN fixture and the explicit reference matcher; the full real-data sweep requires your recording and the Ceres-enabled Docker binary.

## Full recording: one-variable-at-a-time sweep

~~~sh
source .venv-fr079-eval/bin/activate
export FR079_LOG=/home/chris/tmp/fr079.log.gz
export FR079_MAPS=/home/chris/tmp/fr079-maps.tgz

python tools/fr079_truncation_sweep.py \
  --repo "$PWD" \
  --log "$FR079_LOG" \
  --out-dir benchmarks/fr079-edt-sweep-001 \
  --reference-archive "$FR079_MAPS" \
  --reference-cm 5 \
  --docker \
  --capture \
  --capture-every 120 \
  --capture-max-frames 12
~~~

Six maps are reconstructed at 0.35, 0.50, 0.70, 1.00, 1.50 and 2.00 m EDT truncation while holding **match every scan, rebuild field every 15 scans, map stride 3, match stride 4** fixed. Each run produces a map, detailed report, matching decisions, replay, trajectory, and an optional FR079-reference map-agreement JSON/PNG. The output directory must not exist. Use --dry-run to preview exact Docker commands without creating files. Supply --binary /path/to/slam_fr079_offline instead of --docker for a native build.

The benchmark writes sweep.csv and manifest.json. The scorecard includes wall F1 within 15 cm, symmetric mean wall distance, p95 wall error, occupied IoU, accepted/rejected scan matches, per-trial Ceres invalid residual counts and their causes, p95 frame latency and offline FPS. Agreement with FR079's supplied maps is **not independent accuracy**. This suite never dispatches GitHub Actions.

## Controlled perturbations on a fixed map

With --capture, *only* the 0.70 m baseline mapping run does a frozen-snapshot capture study. Every selected accepted frame is evaluated against the **same previously built, immutable map snapshot** at all six EDT cutoffs, preventing different mapping trajectories from confounding capture-range comparisons.

| Parameter | Experiment values |
| --- | --- |
| EDT truncation | 0.35, 0.50, 0.70, 1.00, 1.50, 2.00 m |
| Translation error | 0, 0.10, 0.20, 0.40, 0.70, 1.00 m |
| Angular error | 0, 1, 2, 3, 5, 8 degrees |
| Offset direction | positive global X/positive yaw, negative global Y/negative yaw |
| Tight Ceres bounds | ±0.12 m per coordinate; ±0.08 rad heading |
| Wide Ceres bounds | ±0.50 m per coordinate; ±0.25 rad heading |
| Recovery success | estimate ends within 0.10 m XY and 1 degree of original accepted pose |

There are **864 trials per sampled scan**. Default capture selects at most 12 accepted frames at indices divisible by 120. Capture may generate many legitimate Ceres trust-region warnings because it deliberately starts far from the anchor. These warnings are counted rather than suppressed.

The capture CSV for the example is at benchmarks/fr079-edt-sweep-001/fr079-trunc-0700mm.capture.csv. The experiment does NOT alter the SLAM pose sequence or occupancy map. Reported capture_benchmark_ms and slam_wall_including_capture_ms account for diagnostic overhead separately, so slam_compute_ms and its effective FPS exclude capture trials.

Plot and summarize results:

~~~sh
python tools/fr079_capture_summary.py \
  --capture-csv benchmarks/fr079-edt-sweep-001/fr079-trunc-0700mm.capture.csv \
  --out benchmarks/fr079-edt-sweep-001/capture-summary
~~~

This creates capture-summary.csv, capture-summary.json and capture-summary.png. Add --no-plot to avoid the optional matplotlib dependency.

Each perturbation trial records initial supported-hit fraction and per-range support in 0–3, 3–7, 7–12 and 12–18 m bins; unknown/truncated/out-of-grid initial residual counts; Ceres solver termination, invalid residual evaluations by cause, successful and unsuccessful steps; and final translation/yaw error *relative to the correlated proxy pose*.

The support classification prioritizes unknown interpolation cells, then truncation, then other failures. A sample may physically fail more than one condition; the class records the first failing prerequisite. Unknown cells and truncation cells remain unsupported: no artificial zero-cost evidence is introduced.

## Interpreting outcomes

- Better initial support but no recovery under tight bounds suggests **optimizer search bounds**, not just truncation, limit capture range.
- Decreasing Ceres invalid_truncated evaluations as the cutoff increases points toward **distance-field truncation**.
- Dominant invalid_unknown_cells counts indicate that increasing the cutoff will not fix **unknown-cell rejection**.
- Increased accepted_but_far_from_proxy with wider bounds warns of **ambiguous local minima**; it is not a measured real-world false-positive rate.
- Compare the *full-run* wall F1, mean wall distance, IoU, and p95 wall error separately. Frozen-map proxy recovery cannot guarantee a better whole-run map.

This is still an offline benchmark with assumed identity laser extrinsics and no independent global trajectory ground truth. It does not introduce loop closure, submaps, coarse-to-fine matching or real-time paced input. A future benchmark should use more perturbation directions, independently known poses if available, and held-out map regions.


## Opt-in support-aware Ceres matcher (experimental)

The current legacy matcher remains the default so previous FR079 scores remain
comparable. The new **--support-aware** flag enables a staged Ceres matcher that:

1. Examines the pinned, strictly prior-scan EDT and conservatively certifies
   every bilinear interpolation stencil reachable by a small bounded pose
   step. It never interprets unknown cells as empty or occupied evidence.
2. Selects only beams that are supported throughout that pose box, adapting
   the size of the box when support is sparse.
3. Runs a bounded Ceres refinement using only those certified real EDT
   measurements. Re-selects on the next stage rather than permanently
   discarding observations.
4. Validates the proposed pose on **all original matching beams**: rank,
   overlap, supported-hit count, total robust geometric cost, and preservation
   of every previously supported endpoint. It also retains the frontend's
   existing innovation and map-fingerprint gates.

It does *not* fabricate zero residuals, force unknown occupancy cells to be
known, change map insertion, or introduce loop closure. Conservative support
certification can reduce match acceptance in sparse regions; **fewer Ceres
warnings alone is not success**. The new path must also preserve map quality
and recovery.

To A/B test against existing mapping, use separate output prefixes:

~~~sh
# Legacy baseline: match every scan, 90-beam matching, map all valid beams
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-map-all-match90-legacy \
  --match-every 1 --field-every 15 --map-stride 1 --match-stride 4 \
  --distance-truncation 0.7

# Experimental matcher: identical input, map settings and EDT cutoff
docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-map-all-match90-support-aware \
  --match-every 1 --field-every 15 --map-stride 1 --match-stride 4 \
  --distance-truncation 0.7 --support-aware
~~~

Use tools/fr079_map_compare.py on **both** outputs and compare wall F1 at
15 cm, mean/p95 wall distance, occupied IoU, full-run accepted matches,
processing FPS, and invalid EDT evaluations. The report field matcher is
either "ceres" or "ceres_support_aware". Per-frame .matches.json now
includes support_aware_stages, support_aware_accepted_stages, and cumulative
selected/skipped beam counts across internal solver stages.

The existing sweep script also accepts --support-aware and --map-stride 1
for controlled one-variable-at-a-time runs. New runs must use a unique
--out-dir and the same dataset/reference files. Optional --capture applies
the selected matcher to frozen-map perturbations too. Keep in mind the
capture study is a same-run pose proxy, not independent ground truth.

Local tests after rebuilding the Docker image:

~~~sh
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'distance_field|match_ceres|fr079_support_aware|fr079_offline_slam|fr079_capture_benchmark'
~~~

This first step is support-aware local refinement only. Coarse-to-fine
relocalization, local submaps and loop closure remain separate future work.
