# FR079 Ceres residual/gate ablation and matching-beam benchmark

**Status:** Experimental, opt-in. Legacy Ceres remains the default; ordinary
`--continuous-cost` remains unchanged. No GitHub Actions are used.

## Why this experiment exists

On the same FR079 input (3,118 scans), legacy Ceres at 0.5 m EDT cutoff
accepted 2,962/3,115 matching attempts but reached only 0.679 wall F1
against the correlated FR079 reference. Continuous-cost accepted
1,925/3,115 with wall F1 0.961. Legacy reported 227,765 invalid
residual evaluations (219,676 unknown-cell, 8,089 truncated); continuous
reported zero. An accepted correction is not necessarily an accurate
one. Do **not** use accepted-match count as a substitute for map quality.

Older experiments also found that increasing the number of matching beams
could worsen map quality. More measurements generally help constrain an
estimator only under suitable model, independence, weighting, initialization
and outlier assumptions. Here returns share nearby surfaces and errors,
the cost is robust and nonlinear, the map is continually reconstructed,
and a denser/shifted beam subset changes both the objective and candidate
acceptance. **Monotonic map quality with beam count is not guaranteed.**
This benchmark measures the effect rather than assuming monotonicity.

## Controlled factors

The following matcher paths are all available from the *same* Ceres
executable and use the same iteration limits, bounded pose search, frozen
prior-geometry snapshots, Huber settings, and final frontend innovation
checks. New modes require explicit `--continuous-cost --ablation MODE`:

| Mode | Ceres residual | Solver beam cohort | Improvement gate |
| --- | --- | --- | --- |
| `legacy` | strict, may fail outside observed EDT | seed-supported only | strict geometric cost |
| `finite-seed-legacy` | finite | seed-supported only | strict geometric cost |
| `finite-all-legacy` | finite | all finite hits | strict geometric cost |
| `finite-seed-continuous` | finite | seed-supported only | fixed-cohort finite Huber cost |
| `continuous` | finite | all finite hits | fixed-cohort finite Huber cost |

Each matcher additionally retains the seed's strict support/rank/overlap
gate and a candidate strict-support/rank/overlap gate. The continuous
improvement gate is evaluated on **all input finite hits at both poses**
even in the seed-only experimental solver mode: it is explicitly a
fixed-cohort *scoring* gate, not a different solver beam cohort.

The pair `legacy` vs `finite-seed-legacy` changes the residual
evaluation while preserving the seed-supported beam selection and
legacy improvement gate. The pair `finite-seed-legacy` vs
`finite-all-legacy` changes only the residual cohort. The pair
`finite-seed-legacy` vs `finite-seed-continuous` changes only
the improvement gate. `continuous` is the existing production-optional
finite objective with all eligible matching beams.

These are **closed-loop** ablations. As soon as a run differs in its accepted
correction, its future pose predictions, occupancy map and cached EDT can
also differ. Consult `first-divergence.csv` before attributing *later*
differences to a single per-frame objective change. A fully frozen common
map/pose corpus would be an additional study.

## Beam-count isolation

`--map-stride 1` is held constant for **all** experiments. Only
`--match-stride` is varied: it selects original CARMEN beam indices,
*before* range validation, by `phase, phase+stride, ...`. Thus with
360 raw front-laser bins, nominal beam ceilings are:

| Match stride | Nominal input beams at phase 0 |
| --- | ---: |
| 1 | 360 |
| 2 | 180 |
| 4 | 90 |
| 8 | 45 |
| 12 | 30 |

Actual finite, range-valid matching hits can be fewer; the audit records
`match_input_beams` and `solver_residual_blocks` per attempted frame.
The legacy seed-supported cohort is often substantially smaller than
the input count. `--match-phase` changes the starting raw beam index
while **not changing map insertion**; test phases to reveal decimation
aliasing. Beam-count experiments do not change `--field-every`,
`--match-every`, map resolution, truncation, reference archive or input.

## Build and test locally

From the repository root, with Docker installed:

~~~bash
git pull --ff-only
export SLAM_DOCKER_JOBS=28
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'distance_field|match_ceres|fr079_support_aware|fr079_matcher_ablation|fr079_offline_slam'
~~~

The deterministic 55-frame fixture test checks all five modes at three
beam strides, evidence accounting and invalid combinations. It does not
replace a real FR079 sweep or a hardware test.

## FR079 factorial + beam sweep

Activate the existing local FR079 analysis virtual environment (for
`numpy`, `scipy` and `matplotlib` used by the optional reference comparison):

~~~bash
source .venv-fr079-eval/bin/activate
export FR079_LOG=/absolute/path/to/fr079.log.gz
export FR079_MAPS=/absolute/path/to/fr079-maps.tgz

python tools/fr079_matcher_beam_benchmark.py \
  --repo "$PWD" --log "$FR079_LOG" \
  --out-dir benchmarks/fr079-matcher-beams-001 \
  --reference-archive "$FR079_MAPS" \
  --reference-cm 5 --docker \
  --distance-truncation 0.5 --map-stride 1 \
  --modes legacy finite-seed-legacy finite-all-legacy finite-seed-continuous continuous \
  --match-strides 1 2 4 8 --phases 0
~~~

This is 20 complete, fresh reconstructions. The output directory must
not exist before the run. Add `--dry-run` to inspect every Docker
invocation first; use `--binary /absolute/path/to/slam_fr079_offline`
instead of `--docker` for an equivalent native run.

A smaller first pass to examine beam-count trends without all ablations:

~~~bash
python tools/fr079_matcher_beam_benchmark.py \
  --repo "$PWD" --log "$FR079_LOG" \
  --out-dir benchmarks/fr079-continuous-beams-001 \
  --reference-archive "$FR079_MAPS" --docker \
  --modes continuous --match-strides 1 2 3 4 6 8 12 --phases 0
~~~

A second **phase-sensitivity** experiment (vary the raw starting bin):

~~~bash
python tools/fr079_matcher_beam_benchmark.py \
  --repo "$PWD" --log "$FR079_LOG" \
  --out-dir benchmarks/fr079-continuous-phases-001 \
  --reference-archive "$FR079_MAPS" --docker \
  --modes continuous --match-strides 2 4 8 --phases 0 1 2 3
~~~

Phase values not smaller than the stride are skipped. Every generated
prefix is unique; existing results are never overwritten.

## Outputs and interpretation

- `scorecard.csv`: requested stride and phase, nominal/actual beam counts,
  solver residual blocks, accept rates, invalid/finite unsupported evaluations,
  accepted regressions in the two cost definitions, p95 frame and matching
  time, FPS, and optional reference-wall F1/IoU/distance/coverage proxies.
- `frame-audit.csv`: per-frame seed/candidate strict and finite costs,
  support, correction magnitudes, Ceres evaluations, acceptance and timing.
  Rejected candidate metrics remain present when Ceres produced a candidate;
  a solver failure or early support rejection is flagged separately.
- `first-divergence.csv`: first frame of differing correction for adjacent
  matcher modes at the same stride and phase, allowing fair interpretation
  of subsequent closed-loop divergence.
- `manifest.json`: source identity, fixed parameters, exact commands,
  run names and limitations.
- Per-run `.report.json`, `.matches.json`, `.trajectory.json`,
  `.slammap` and the map/reference comparison JSON/PNG if requested.

Read `accepted_strict_cost_regressions` and
`accepted_finite_cost_regressions` carefully: the two objectives differ.
Neither is independent ground truth. The supplied FR079 reference map
comes from the same recording and alignment is fitted post hoc, so wall
F1/IoU remain *correlated geometric agreement proxies*. A decrease with
additional beams is an actionable diagnostic, but not a violation of an
estimator correctness guarantee. Inspect first-divergence frames and
beam-selection phases before choosing any new default.

Do not silently change production matcher defaults or infer Pi timing
from offline workstation FPS.
