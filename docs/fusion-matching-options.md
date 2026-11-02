# Offline synthetic fusion matching options

The `slam_offline_fusion` CLI accepts the **same matching-control flags**
for the original `apartment` (130 scans) and `warehouse`
(195 scans), plus five additional independent synthetic geometries.
The flags affect both the held-out fused scan matcher and the
incremental LiDAR-only matcher. They do not add a new input dataset or
change the independent simulator reference used only for scoring.

| Option | Default | Effect |
| --- | ---: | --- |
| `--match-every N` | 1 | Match every N eligible scan queries. Held-out queries are odd-indexed; skipped queries are not counted as rejections. LiDAR-only skipped queries are not inserted because they have no validated pose. |
| `--field-every N` | 1 | Rebuild LiDAR-only distance field after N scan frames (cached earlier-map snapshot otherwise). The held-out training map is frozen; its distance field is constructed once. |
| `--map-stride N` | 1 | Insert every Nth **valid** LiDAR hit into occupancy for all generated map variants (wheel-only, LiDAR-only, gyro-wheel, fused-matched, optional graph). This also controls held-out training-map insertion. |
| `--match-stride N` | 1 | Match every Nth **valid** deskewed or raw hit; does not change mapping beams. A 240-bin synthetic scan has at most 240/N matching hits. |
| `--match-offset N` | 0 | Start decimation at valid-hit index `N % match_stride` (0–63); keeps the original phase-0 algorithm unchanged and tests sampling-pattern sensitivity. |
| `--distance-truncation METRES` | 0.7 | EDT truncation in both frozen and incremental matchers (0.2–3.0 m). |
| `--continuous-cost` | off | Use `match_ceres_continuous` for **both** matchers; requires Ceres, and fails instead of silently selecting the reference implementation if unavailable. |
| `--lidar-only-bootstrap` | off | Enable conservative scan-to-scan registration for the LiDAR-only baseline, with strict EDT refinement when real map support becomes available; other map variants remain unchanged. |
| `--lidar-only-predict-first` | off | Require a LiDAR-derived relative pose prediction before Ceres/EDT; preserve separate Ceres-first bootstrap for A/B. Requires `--lidar-only-bootstrap`. |

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

## Opt-in LiDAR-only scan-to-scan bootstrap

The initial strict support audit found that **all 129 apartment** and
**all 194 warehouse** LiDAR-only matching attempts failed **before Ceres
ran**. Their seed overlap was only 3.8–10% for the first three frames;
none of the 1,521 nearby poses tested per frame reached 45% strict EDT
support. Expanding the Ceres pose box or lowering the global EDT support
gate would not solve this initial-map geometry problem responsibly.

`--lidar-only-bootstrap` provides a separate, experimental LiDAR-only
bootstrap that uses neither wheel/gyro odometry nor simulator truth:

1. Preserve the existing EDT/Ceres matcher as first choice. With bootstrap
   enabled and an accepted LiDAR displacement available, seed it by composing
   that displacement with the previous accepted laser pose. Reusing a
   stationary seed after bootstrap can let the growing map accept near-zero
   motion and stall the trajectory. No changes occur without the new flag.
2. If that match fails, register the **current raw LiDAR hit cloud**
   against the **last accepted raw LiDAR scan**, using deterministic
   trimmed nearest-neighbor rigid SE(2) registration. Three yaw seeds
   (`0, ±0.12 rad` relative to the previous accepted LiDAR-derived
   scan-to-scan displacement) are evaluated.
3. Reject registrations that have fewer than 24 valid hits,
   degenerate 2D point-cloud covariance, insufficient reciprocal
   within-30-cm inliers (minimum **65% in both directions**),
   excessive inlier RMS error (**0.20 m**), or excessive displacement
   (translation **0.90 m**, yaw **0.32 rad** per accepted registration).
4. When a registered pose has genuine strict EDT support (minimum 18
   observed hits, **45% overlap**, full rank), try the existing
   Ceres matcher for further refinement, preserving its normal
   qualification gates and limiting disagreement from the scan-to-scan
   candidate. Otherwise, continue with the validated scan-pair pose.
5. Insert a scan into the incremental map **only after accepting** its
   LiDAR-derived pose. Keep the last accepted scan and relative
   transformation as the next bootstrap reference. Unlocalized scans
   are still excluded from occupancy.

This is not a globally verified solution to scan matching. Repetitive
warehouse geometry, partial visibility and nearest-neighbor ambiguity
can still cause drift. **Independent synthetic map/trajectory scores
must improve before considering any default change.** The existing
`wheel_only`, `gyro_wheel`, `fused_scan_matched`, and optional
graph maps must remain invariant to toggling LiDAR-only bootstrap.

### Experimental LiDAR-first prediction and EDT consistency gate

The first bootstrap benchmark restored LiDAR-only tracking, but its
60-beam warehouse continuous-cost run drifted badly: ground-truth
displacement was ~0.303 m per scan while the scan-pair registration
reported only ~0.123–0.142 m in the first eight frames. The 240-beam
run estimated ~0.30 m in those frames. This is **not** evidence that
more beams guarantee lower error; it is evidence of a density-dependent
local registration failure in the synthetic fixture.

New option: `--lidar-only-predict-first` **requires**
`--lidar-only-bootstrap`. It is deliberately separate from the Ceres-first bootstrap, which uses
the previous accepted relative displacement as its EDT seed. Predict-first
instead registers the current scan before attempting EDT refinement.

With both flags, each LiDAR-only query follows this order:

1. Register current scan to last accepted scan using only sampled
   LiDAR hit points and last accepted LiDAR-derived relative motion.
   Retry a bounded zero-motion multi-start if needed.
2. **Reject without insertion** when scan-pair geometry fails the
   reciprocal registration gates. Do **not** allow Ceres around the
   previous accepted *stale* pose to override that rejection.
3. Use the accepted scan-pair relative SE(2) pose to predict where
   the new laser pose is in the incremental occupancy map.
4. Attempt the existing strict EDT/Ceres scan matcher **at that
   predicted pose**, not at the previous accepted pose.
5. Apply the EDT correction only when all existing geometric/source/
   rank/improvement gates pass **and** its proposed relative motion
   differs by no more than **0.12 m** and **0.075 rad** from the LiDAR
   scan-pair prediction. If EDT is unsupported, rejected, or disagrees,
   retain the qualified scan-pair pose instead.
6. Insert the LiDAR scan only after the chosen relative motion has
   passed those checks. No simulator truth or wheel/gyro motion is used
   for localization in this path.

Per-frame audit now records `used_scan_prediction`,
`edt_prediction_disagreed`,
`predicted_edt_translation_delta_m`,
`predicted_edt_yaw_delta_rad`, and `registration_method`
(`edt_predicted`, `scan_to_scan`, or `none`).
Fusion report and sweep scorecards include
`lidar_only_predicted_edt_accepted` and
`lidar_only_edt_prediction_disagreements`.

This is an **experiment**, not a default. Motion estimates can still
be wrong when the raw scan-pair match is ambiguous. In particular,
the sparse 60-beam underestimation may persist even if Ceres no
longer reinforces a stale seed. The experiment measures whether
prediction-first reduces such drift; it does not manufacture a
monotonic improvement curve.

### Controlled A/B: original bootstrap versus LiDAR-first

Use a new output directory:

~~~bash
python tools/fusion_match_beam_sweep.py \
  --repo "$PWD" \
  --out-dir benchmarks/fusion-beams-predict-first-001 \
  --docker --scenes apartment warehouse \
  --matchers legacy continuous \
  --map-stride 1 --match-every 1 --field-every 1 \
  --distance-truncation 0.7 --match-strides 1 2 3 4 6 8 \
  --lidar-only-bootstrap --lidar-only-predict-first

python tools/fusion_bootstrap_compare.py \
  --old benchmarks/fusion-beams-bootstrap-001/scorecard.csv \
  --new benchmarks/fusion-beams-predict-first-001/scorecard.csv
~~~

The comparator checks all three non-LiDAR-only control fingerprints,
prints trajectory RMSE/IoU and acceptance statistics, and counts
adjacent beam-count increases that **worsen** RMSE. It does not impose
monotonicity, hide regressions, or use simulator reference as a
motion predictor.

### Ceres-first motion-seed regression

The stationary-seed regression is guarded by the existing requirement that
bootstrap translation RMSE beat the frozen baseline by at least 10% in both
scenes. With the LiDAR-motion EDT seed, Docker/Ceres RMSE changed from
2.129 to 0.241 m (apartment) and 10.773 to 3.852 m (warehouse). These are
synthetic measurements, not hardware accuracy claims. Warehouse occupied
IoU remains poor and decreased from 0.0344 to 0.0296 versus the broken
bootstrap run; trajectory improvement is not proof of map improvement.
Initialization matters for local registration; see the
[Open3D ICP tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html).

### Run one controlled experiment

~~~bash
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'scan_to_scan_bootstrap|offline_fusion|fusion_match_beam_tools|fusion_replay_http'

for scene in apartment warehouse; do
  docker compose run --rm \
    -v "$PWD/apps/demo/fusion:/fusion" \
    --entrypoint ./build-docker/slam_offline_fusion test \
    "$scene" "/fusion/${scene}-bootstrap-$(date +%Y%m%d-%H%M%S)" \
    --match-every 1 --field-every 1 --map-stride 1 \
    --match-stride 1 --distance-truncation 0.7 \
    --lidar-only-bootstrap
done
~~~

The `fusion_report.json` records
`lidar_only_bootstrap_attempted`, `lidar_only_bootstrap_accepted`,
`lidar_only_bootstrap_rejected`, `lidar_only_edt_accepted`,
and `lidar_only_edt_refined_accepted`. Each frame in
`lidar_only_match_audit.json` records
`registration_method` (`none`, `edt`, `scan_to_scan`, or
`edt_refined`), reciprocal overlaps, inlier RMS, and relative motion.
The original seed-only EDT support diagnostics remain available.

### Repeat the complete matching-beam sweep

Use a **new** output directory; do not overwrite the original
`benchmarks/fusion-beams-001`.

~~~bash
python tools/fusion_match_beam_sweep.py \
  --repo "$PWD" --out-dir benchmarks/fusion-beams-bootstrap-001 \
  --docker --scenes apartment warehouse \
  --matchers legacy continuous \
  --map-stride 1 --match-every 1 --field-every 1 \
  --distance-truncation 0.7 --match-strides 1 2 3 4 6 8 \
  --lidar-only-bootstrap
~~~

The new scorecard reports actual mean matching-beam counts, separate
scan-to-scan bootstrap acceptance, EDT-only acceptance and EDT-refined
acceptance, trajectory RMSE, occupied IoU and performance. Compare rows
with the matching scene/matcher/stride from the original sweep. A high
acceptance count is **not sufficient** if independent map agreement,
translation RMSE, or timing worsens.

## Generalization suite: varied maps, routes and beam phases

**Do not infer a universal beam-count trend from one warehouse.** Its
parallel, regularly spaced shelving is a particularly ambiguous case for
nearest-neighbor ICP. The following additional exact, deterministic maps
exercise different geometry and motion profiles:

| Scene | Geometry | Robot motion |
| --- | --- | --- |
| `apartment` | Two partitions with door gaps | Original compact 1.35 m-radius loop, 130 scans |
| `warehouse` | Four equally spaced parallel shelves | Original wide perimeter ellipse, 195 scans |
| `warehouse_asymmetric` | Uneven shelf lengths/positions and diagonal braces | **Same route** as warehouse, 195 scans: symmetry ablation |
| `office` | Uneven office partitions, corridors, oblique wall | Smaller elliptical entry-area loop, 150 scans |
| `loft` | Open room with an irregular furniture island | Mid-size elliptical loop, 168 scans |
| `atrium` | Nonrectangular central core, oblique ribs | Interior ring of a large enclosure, 200 scans |
| `industrial` | Heterogeneous rectangular machinery and diagonal objects | Wide, faster perimeter ring, 195 scans |

The new fixtures are generated using the same moving-scan LiDAR physics,
gyro/wheel sensor model, 240 raw acquisition bins and independently
withheld simulator truth as the original two. Their reference trajectories
are checked against wall geometry before any run. **The original
`apartment` and `warehouse` fixtures remain unchanged.**

This expands *synthetic* coverage; it is **not independent real-world
generalization**. The environments still share an analytic ray caster,
noise model and smooth elliptical routes. Follow this suite with diverse
recorded LiDAR datasets and independently validated ground truth before
selecting a production beam count or changing any matcher defaults.

### Stage A — seven maps, same beam phase

Run the complete seven-scene comparison with both Ceres variants
and six beam densities (7 × 2 × 6 = **84 runs**) using a NEW directory:

~~~bash
docker compose build test
docker compose run --rm --entrypoint ctest test \
  --test-dir build-docker --output-on-failure \
  -R 'fixtures_smoke|offline_fusion|fusion_scene_diversity|fusion_match_beam_tools'

python tools/fusion_match_beam_sweep.py \
  --repo "$PWD" --out-dir benchmarks/fusion-beams-diverse-001 \
  --docker \
  --scenes apartment warehouse warehouse_asymmetric office loft atrium industrial \
  --matchers legacy continuous \
  --map-stride 1 --match-every 1 --field-every 1 \
  --distance-truncation 0.7 --match-strides 1 2 3 4 6 8 \
  --match-offsets 0 \
  --lidar-only-bootstrap --lidar-only-predict-first

python tools/fusion_beam_generalization.py \
  --scorecard benchmarks/fusion-beams-diverse-001/scorecard.csv
~~~

If those regressions are sensitive to a particular scene, use Stage B
rather than trying to tune the matcher to the original warehouse.

### Stage B — sampling-phase sensitivity

`--match-offsets` changes which valid LiDAR hit starts decimation,
**without affecting the simulated sensor data or map insertion rays**.
The sweep schedules each unique *effective* phase per stride; with
strides 1, 2, 4 and 6 and requested offsets 0, 1, 2, that means
**1 + 2 + 3 + 3 = 9** phase/stride combinations per scene/matcher.

A focused continuous-Ceres multi-scene phase check (7 × 9 = **63 runs**):

~~~bash
python tools/fusion_match_beam_sweep.py \
  --repo "$PWD" --out-dir benchmarks/fusion-beams-diverse-phases-001 \
  --docker \
  --scenes apartment warehouse warehouse_asymmetric office loft atrium industrial \
  --matchers continuous \
  --map-stride 1 --match-every 1 --field-every 1 \
  --distance-truncation 0.7 --match-strides 1 2 4 6 \
  --match-offsets 0 1 2 \
  --lidar-only-bootstrap --lidar-only-predict-first

python tools/fusion_beam_generalization.py \
  --scorecard benchmarks/fusion-beams-diverse-phases-001/scorecard.csv
~~~

Report **per-scene** and **per-phase** LiDAR-only translation RMSE,
map IoU, true matching-beam count, Ceres/ICP acceptance and execution
cost. The generalization report shows median and worst-case RMSE by
density, within-scene-and-phase increases in error with more beams, and
the 240-beam/60-beam RMSE ratio **for paired phase-zero cases**.
Do not pool different phases as if they were independent map datasets.
Non-matching `wheel_only` and `gyro_wheel` map fingerprints must be
identical across beam configurations. The `fused_scan_matched` control
only needs to stay invariant when comparing bootstrap modes at the
**same** scene, matching stride and sampling phase; it legitimately
changes when changing the matched-beam subset.

The run manifest contains each scene/phase/stride tuple and command,
all artifacts remain in separate directories, and no existing benchmark
outputs are overwritten. `--dry-run` previews commands without running.

## Multi-scene matching-beam benchmark

The original two-scene synthetic comparison is reproducible, but
the broader seven-scene suite above should be used for generalization.
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
