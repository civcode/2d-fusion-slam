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
