# Offline synthetic fusion matching options

The `slam_offline_fusion` CLI accepts the **same matching-control flags**
for both supported scenes, `apartment` (130 scans) and `warehouse`
(195 scans). The flags affect both the held-out fused scan matcher and the
incremental LiDAR-only matcher. They do not add a new input dataset or
change the independent simulator reference used only for scoring.

| Option | Default | Effect |
| --- | ---: | --- |
| `--match-every N` | 1 | Match every N eligible scan queries. Held-out queries are odd-indexed; skipped queries are not counted as rejections. LiDAR-only skipped queries are not inserted because they have no validated pose. |
| `--field-every N` | 1 | Rebuild LiDAR-only distance field after N scan frames (cached earlier-map snapshot otherwise). The held-out training map is frozen; its distance field is constructed once. |
| `--map-stride N` | 1 | Insert every Nth **valid** LiDAR hit into occupancy for all generated map variants (wheel-only, LiDAR-only, gyro-wheel, fused-matched, optional graph). This also controls held-out training-map insertion. |
| `--match-stride N` | 1 | Match every Nth **valid** deskewed or raw hit; does not change mapping beams. A 240-bin synthetic scan has at most 240/N matching hits. |
| `--distance-truncation METRES` | 0.7 | EDT truncation in both frozen and incremental matchers (0.2–3.0 m). |
| `--continuous-cost` | off | Use `match_ceres_continuous` for **both** matchers; requires Ceres, and fails instead of silently selecting the reference implementation if unavailable. |

Cadences are integers 1–200 and strides are integers 1–64. Invalid
values fail before an output directory is created. Without flags,
the implementation preserves previous matching selection, per-frame
EDT rebuilding, map insertion, and generated artifact behavior.

**Important:** `--map-stride` controls **every map** while
`--match-stride` controls only matching. For a matching-beam ablation,
keep `--map-stride 1` and all other options fixed.

## Docker build, tests, and execution

From repository root, on the
`experiment/fr079-ceres-ablation-beam-sweep` branch:

~~~bash
export SLAM_DOCKER_JOBS=28
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'offline_fusion|fusion_replay_http|cli_cxxopts_slam_offline_fusion|cli_cxxopts_slam_ceres_replay_export'
~~~

Use a **fresh output directory** for each run. The container writes
to the existing host volume `apps/demo/fusion`:

~~~bash
mkdir -p apps/demo/fusion
for scene in apartment warehouse; do
  docker compose run --rm \
    -v "$PWD/apps/demo/fusion:/fusion" \
    --entrypoint ./build-docker/slam_offline_fusion test \
    "$scene" "/fusion/${scene}-finite-$(date +%Y%m%d-%H%M%S)" \
    --match-every 1 --field-every 15 \
    --map-stride 1 --match-stride 4 \
    --distance-truncation 0.5 --continuous-cost
done
~~~

The output contains `fusion_report.json`, `fusion_replay.json`,
`fusion_profile.json`, `trajectories.json` and map bundles for each
fusion method. The report records all flags, held-out attempted/skipped
queries, LiDAR-only attempted/skipped/rebuilt-field counts and actual
matching-input beams. The replay reports per-frame match-input hit counts
and solver residual blocks for attempted held-out matches.

## Diagnosing the LiDAR-only bootstrap failure

In the first 24-run synthetic beam sweep, LiDAR-only accepted **0**
corrections in both scenes, with **all** attempts reported as
`insufficient_support` (129 apartment / 194 warehouse). This status
may result either from insufficient strict EDT support **at the initial
seed**, or loss of strict support **after Ceres solves**. It does not by
itself show which path occurred. The baseline predicts its next pose as
the previous accepted pose, with no inertial, wheel or ground-truth motion
prediction. When the first match fails, this can leave it stranded.

New fusion runs export a `lidar_only_match_audit.json` file. Every
attempt records the frame index, matching-beam count, strict seed
supported hits and overlap, rank, map obstacle presence and map revision,
solver residual blocks, whether a candidate was evaluated, candidate
support, and acceptance/rejection status. The fusion report additionally
contains `lidar_only_before_solver_insufficient_support` and
`lidar_only_after_solver_insufficient_support` totals.

The optional `--lidar-only-support-probe` performs an **observational
search only** at frames 1–3. It evaluates strict geometry on a grid
around the last LiDAR-only pose: X and Y offsets from -0.6 m to +0.6 m
in 0.1 m steps, and yaw offsets from -0.24 rad to +0.24 rad in 0.06
rad steps (1,521 trials per frame). The probe records how many poses
satisfy support and full-rank criteria and the offset with greatest
supported beam count. It **never changes estimated pose, matcher acceptance,
occupancy, or the trajectory**. Do not interpret its most-supported
candidate as an accurate registration or independent truth.

Run both scenes without re-running the full beam matrix:

~~~bash
for scene in apartment warehouse; do
  docker compose run --rm \
    -v "$PWD/apps/demo/fusion:/fusion" \
    --entrypoint ./build-docker/slam_offline_fusion test \
    "$scene" "/fusion/${scene}-support-probe-$(date +%Y%m%d-%H%M%S)" \
    --map-stride 1 --match-stride 1 \
    --distance-truncation 0.7 --lidar-only-support-probe
done
~~~

Then summarize the latest experiments:

~~~bash
python - <<'PY'
import json
from pathlib import Path
for scene in ("apartment","warehouse"):
    runs=sorted(Path("apps/demo/fusion").glob(f"{scene}-support-probe-*"))
    if not runs:
        print(scene,"no support-probe run");continue
    folder=runs[-1]
    report=json.loads((folder/"fusion_report.json").read_text())
    audit=json.loads((folder/"lidar_only_match_audit.json").read_text())
    print(f"\n{scene}: pre-solver support failures="
          f"{report['lidar_only_before_solver_insufficient_support']}, "
          f"post-solver support failures="
          f"{report['lidar_only_after_solver_insufficient_support']}")
    for frame in audit["frames"][:3]:
        print({k:frame[k] for k in (
            "index","input_beams","seed_supported_hits","seed_overlap",
            "seed_full_rank","map_has_obstacles","map_revision",
            "solver_residual_blocks","candidate_evaluated",
            "probe_supported_poses","probe_ranked_poses",
            "probe_best_supported_hits","probe_best_overlap",
            "probe_best_offset")})
PY
~~~

Only if a nearby pose exhibits independent strict geometry support
should we try a **separate, opt-in** coarse-to-fine bootstrap. Any
candidate must still satisfy rank, support, motion-bound and finite
fixed-cohort improvement checks, and improve **independent** synthetic
trajectory/map scores. A large number of supported candidates can
also indicate ambiguity, not reliable localization.

## Multi-scene matching-beam benchmark

A direct synthetic experiment is especially useful here because the
**simulated reference poses and walls are independent of the estimator**
and are never passed as matching inputs. It is better evidence for a
beam-count regression than the correlated FR079 reference map.

~~~bash
python tools/fusion_match_beam_sweep.py \
  --repo "$PWD" \
  --out-dir benchmarks/fusion-beams-001 \
  --docker \
  --scenes apartment warehouse \
  --matchers legacy continuous \
  --map-stride 1 \
  --match-every 1 --field-every 1 \
  --distance-truncation 0.7 \
  --match-strides 1 2 3 4 6 8
~~~

This produces 24 independent synthetic runs (two scenes × two matchers ×
six matching strides), with one scored row per generated map method in
`scorecard.csv`. Each run has its own full artifact directory and log.
The `scorecard.csv` includes actual mean matching-beam counts for
held-out and LiDAR-only matching, scan-match acceptances, translation
RMSE, wall occupied IoU, map fingerprint and processing throughput.

`wheel_only` and `gyro_wheel` maps act as negative controls because
changing the matching stride must **not** alter them. The sweep fails
if their map fingerprints differ across matching-beam configurations.

Use `--dry-run` to see the commands without creating any output.
Alternatively use `--binary /absolute/path/to/slam_offline_fusion`
instead of `--docker` for local native runs.

**Interpretation:** More beams do not have a mathematical guarantee to
improve closed-loop scan matching. Nonlinear objectives, correlated rays,
outliers, missing geometry and discontinuous acceptance may worsen the
estimate. Compare *actual* input/residual beam counts, accepted corrections,
independent map/trajectory scores and runtime. Do not select a production
default based on the number of accepted corrections alone.
