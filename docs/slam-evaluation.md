# Local synthetic pipeline evaluation

## Scope and evidence

Executed the existing `slam_offline_fusion` C++ pipeline in Ubuntu 24.04
Docker on `local-build`, with Ceres 2.2 and GTSAM 4.2 required. Builds and
CTest used 28 parallel jobs; experiment processes ran concurrently and map
re-scoring used up to 28 workers. Individual Ceres solves remain single-threaded.

**These are apartment/warehouse experiments, not the web UI workshop replay.**
The workshop's 64-beam scan-level replay is not an input to this CLI. These
experiments generate 240-beam moving scans plus biased wheel/gyro observations.
Truth is withheld from estimation except the initial gauge anchor, and used
for simulation and fixed-frame evaluation. No fitted trajectory alignment is used.

Final local artifacts:
`.build/slam-evaluation-validated-gTbm2K/`

- Four experiment directories: `apartment`, `warehouse`, and their
  `*-simulated-loops` variants; each has five maps, trajectories and a report.
- `summary.csv`: all 20 method/experiment results.
- `verification.json`: primary-output hashes and verification results.
- `*.accuracy.json`: independently invoked map-scoring CLI output for every map.
- `repeat/`: fresh repeated experiments with byte-identical primary outputs.
- `verify_results.py`: re-scoring, independent Python trajectory-error/IoU
  calculations and repeatability checks, run inside the test container.
- Image ID: `sha256:02906a1131051188467131cc7efde1abfed451628ae4e377215c7986c1fd1b37`.

All 31 CTests passed after the changes below. Separately re-scored all 20 maps,
recomputed all 20 trajectory metrics from exported poses, and verified all four
experiments' reports, trajectories and map bundles repeat byte-for-byte.
Passing tests establishes execution/integrity, **not acceptable SLAM quality**.

## Results

Translation RMSE is in metres. Occupied IoU is on the full fixed 5 cm grid;
unknown reference-wall cells count as missed walls.

| Method | Apartment RMSE | Apartment IoU | Warehouse RMSE | Warehouse IoU |
| --- | ---: | ---: | ---: | ---: |
| Wheel only | 0.3996 | 5.73% | 5.3339 | 2.85% |
| LiDAR only | 1.9330 | 6.65% | 17.8926 | 2.65% |
| Gyro + wheel | 0.0556 | 29.83% | 0.6011 | 4.60% |
| Gyro + wheel + Ceres corrections | 0.0526 | 30.32% | 0.5984 | 4.77% |
| Default GTSAM graph | 0.0556 | 29.83% | 0.6011 | 4.60% |
| GTSAM + separately simulated loop sensor | 0.0422 | 27.31% | 0.4478 | 7.08% |

- Held-out Ceres matches: apartment **65/65**, warehouse **94/97**.
  The three warehouse rejections are `no_improvement`.
- LiDAR-only tracking fails to bootstrap: **0/129** apartment and **0/194**
  warehouse updates accepted; every rejection is `insufficient_support`.
  Only the initial scan enters those maps; later poses stay at the anchor.
- Gyro + wheel supplies nearly all trajectory improvement. Ceres corrections
  yield only small additional gains. High match acceptance is not high accuracy.
- Default graph factors contain only gyro/wheel increments; scan-matched poses
  initialize the graph but are not LiDAR measurement factors. Optimization
  therefore returns essentially the gyro/wheel solution, discarding the small
  scan-matching corrections. It is not a complete LiDAR pose-graph backend.
- Optional loop experiments accepted **17** apartment and **3** warehouse
  constraints. These come from an **extra simulated relative-pose instrument**,
  not automatic LiDAR loop detection. They reduce trajectory RMSE but do not
  uniformly improve maps: apartment occupied IoU regresses to 27.31%.
- Apartment fused known coverage is 47.84%, wall recall 35.25%; incomplete scene
  exploration limits its full-scene scores. Warehouse fused known coverage is
  88.85%, but wall recall is only 7.16%: its map is geometrically poor despite
  broad observation coverage.

## Bugs found and remaining work

The initial warehouse ellipse (13 x 8 m radii) put **9/195 reference poses
inside shelf solids**. Those preliminary results are superseded and retained
under `.build/slam-evaluation-xQ3owR/` with an explicit warning. Corrected radii
are 14 x 9 m. Regression tests check a 0.2 m-radius body along interpolated
reference motion and the offset laser origin. The new test fails on the old
binary and passes on the corrected one. Rejection histograms were added to
`fusion_report.json`, with count-consistency tests.

The evaluation does **not** establish a successful autonomous LiDAR SLAM
system. Next priorities are diagnosing insufficient-support bootstrap against
the sparse initial map, building a reliable sequential scan/submap frontend,
and adding actual LiDAR constraints/loop detection with appropriate evidence
handling. Do not relax unknown-space/rank gates just to improve acceptance, or
feed exact poses/reference maps to the estimator to make the benchmark pass.
Workshop input support remains separate work.

Apartment logs contain rejected Ceres trial evaluations. The matcher refuses
trials leaving valid EDT support. Ceres documents this feasibility mechanism
and its premature-convergence risk in its
[modeling FAQ](https://ceres-solver.readthedocs.io/latest/modeling_faqs.html#how-do-i-solve-problems-with-general-linear-non-linear-inequality-constraints-with-ceres-solver).
The warnings are retained; they are not hidden or counted as successful steps.

## Reproduce

```sh
DOCKER_CONFIG=/tmp/fusion-slam-docker-config SLAM_DOCKER_JOBS=28 \
  docker compose --env-file /dev/null run --build --rm test

# Choose a new directory: the CLI deliberately refuses existing outputs.
mkdir -p .build/my-evaluation
docker run --rm --init --user "$(id -u):$(id -g)" --cpus 28 \
  --mount "type=bind,source=$PWD/.build/my-evaluation,target=/results" \
  2d-fusion-slam-test bash -c '
    set -e
    for scene in apartment warehouse; do
      ./build-docker/slam_offline_fusion "$scene" "/results/$scene"
      ./build-docker/slam_offline_fusion "$scene" "/results/$scene-simulated-loops" \
        --synthetic-independent-loops
    done'
```

See [pipeline details](offline-fusion.md) for sensor assumptions and outputs.
