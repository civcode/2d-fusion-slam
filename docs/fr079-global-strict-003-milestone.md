# FR079 Global-Strict 003 — Gold-Standard SLAM Milestone

**Status:** Current FR079 gold standard (visual selection and reference-relative benchmark)  
**Recorded:** 5 November 2026  
**Project:** [civcode/2d-fusion-slam](https://github.com/civcode/2d-fusion-slam) — branch `experiment/fr079-ceres-ablation-beam-sweep`, draft PR #5  
**Primary artifact:** `fr079-ceresgraph-global-strict-003.graph_optimized.slammap`  
**Related visualization:** `fr079-ceresgraph-global-strict-003.pgm` (generated from the optimized graph occupancy grid)

## Executive summary

**Global-strict 003 is our new FR079 gold-standard result.** The optimized PGM was visually assessed as the most consistent map produced to date. That judgement is supported by a major improvement in agreement with Freiburg's published scan-matched trajectory: **0.2167 m first-pose absolute trajectory error (ATE) RMSE**, compared with **1.1727 m** for the former pose-prior visual baseline (an improvement of approximately **81.5%**). Best-fit rigid XY-aligned ATE RMSE is **0.1799 m**, and reference-trajectory-derived occupied-endpoint IoU is **0.3507** with first-pose alignment or **0.3790** with rigid XY alignment.

The result demonstrates a valuable architecture: **discover revisits globally; verify them conservatively using reciprocal raw LiDAR scans; optimize only the geometrically supported constraints.** Global-strict accepted **10 loop closures** across 3,118 scans. Eight are the same constraints accepted by distributed-strict 003, while **two additional appearance-retrieved closures**, `1112 → 2768` and `1144 → 2800`, connect late observations to earlier locations. They are the leading explanation for the improvement, but their individual causal effects have not yet been isolated by ablation.

**Scope of the claim:** “Gold standard” means best observed FR079 result so far under the project's visual assessment and reference-relative indicators. The Freiburg scan-matched pose log is **not surveyed ground truth**, and trajectory-derived endpoint IoU is **not** a score of the exported occupancy grid against an independently surveyed map.

## 1. What global-strict does

Global-strict is the `--loop-registration scan-to-submap-global` mode with the strict reciprocal verification introduced after the earlier `global-002` experiment. It operates after the unchanged Ceres LiDAR/odometry frontend and builds a GTSAM pose graph from correlated frontend-relative sequential factors and selected raw-LiDAR loop factors.

1. **Build a causal frontend trajectory.** The Ceres frontend processes the 3,118 raw FR079 scans using embedded scan-time odometry. The Freiburg reference poses are never used for estimation, candidate retrieval, or registration.
2. **Discover local candidates.** For each sampled current keyframe (stride eight), test historical anchors predicted to lie within **2.8 m** by the Ceres trajectory, subject to temporal exclusion and the existing per-keyframe hypothesis limit. Build frozen 0.10 m historical occupied-endpoint submaps from short scan windows.
3. **Discover global candidates by appearance.** At every 16th current scan, compare a shape descriptor (normalized 16-bin histogram of occupied-point pair distances) against older submaps. Propose temporally separated clusters with descriptor similarity **≥0.82**, even when the Ceres-predicted positions are more than 2.8 m apart.
4. **Register scan to historical submap.** Local hypotheses use a bounded **0.54 m / 0.18 rad** search around the frontend relative pose. Appearance-retrieved hypotheses use a wider **4.5 m / 0.55 rad** correction window and stronger initial support/correlation gates (**≥70%** support and **≥0.60** correlation score). Flat or ambiguous correlation peaks are rejected.
5. **Apply the pose-prior-equivalent raw-scan gate.** The successful submap pose seeds registration between the **original two LiDAR scans**. Accept only a full-rank match with at least **80 inliers in each direction**, **≥82% reciprocal overlap**, a forward registration RMS **≤0.11 m**, and a successful reverse registration whose cycle closes within **0.14 m / 0.07 rad**. The correction from the submap seed must remain within **0.55 m / 0.17 rad**. For local loops, retain the frontend-relative innovation guard; for appearance loops, use the separate global-drift graph limits.
6. **Corroborate and optimize.** Global appearance candidates additionally need agreement from a nearby historical anchor (within **0.55 m / 0.12 rad**); these historical observations are correlated and do not constitute independent ground truth. Keep the best eligible candidates subject to redundant-pair suppression and a **1,000-loop ceiling**. The ceiling is nonbinding for FR079, but selection still permits at most one candidate per current keyframe. Feed the **reciprocally verified raw scan-to-scan transform** to GTSAM with conservative robust factors and a fixed first-pose gauge.
7. **Reconstruct the occupancy map.** Integrate the unchanged raw rays at the optimized graph poses and export `*.graph_optimized.slammap`. In new loop runs, the ordinary `.pgm` and `.yaml` previews represent **this graph-optimized map**, not the incremental frontend map.

### Why this differs from distributed-strict

| Dimension | Distributed-strict 003 | Global-strict 003 |
|---|---|---|
| Candidate source | Ceres-predicted local proximity | Same local candidates **plus** appearance-retrieved, out-of-proximity candidates |
| Additional registration search | Local only, 0.54 m / 0.18 rad | Appearance search extends to 4.5 m / 0.55 rad |
| Initial global match gate | Not applicable | Descriptor ≥0.82; support ≥70%; correlation ≥0.60; cross-anchor corroboration |
| Final reciprocal raw-scan gate | Same strict 82%/RMS/cycle/rank checks | **Same strict gate** |
| GTSAM initial-path discrepancy | Local graph limits | Global provenance allows up to 4.6 m / 0.60 rad for retrieved matches |
| Maximum loop budget | 1,000 | 1,000 |

The stronger global search and wider **initial-path** tolerance do **not** weaken the reciprocal raw-scan verification thresholds.

## 2. Quantitative benchmark milestone

All reported trajectory scores are against the **published Freiburg FR079 scan-matched pose estimate** on the same 3,118 scans, not surveyed ground truth. Lower ATE/RPE is better; higher occupied-endpoint IoU is better.

| Run | Accepted loops | First-pose ATE RMSE (m) ↓ | Rigid XY ATE RMSE (m) ↓ | First-pose endpoint IoU ↑ | Rigid XY endpoint IoU ↑ |
|---|---:|---:|---:|---:|---:|
| Ceres frontend, no loops | 0 | 1.8641 | 1.1122 | 0.1152 | 0.1130 |
| Pose-prior 001 — former visual baseline | 13 | 1.1727 | 0.8462 | 0.2100 | 0.1325 |
| Distributed 002 — earlier submap | 24 | 1.3041 | 0.9365 | 0.1799 | 0.1396 |
| Global 002 — earlier submap | 24 | 1.2062 | 0.7154 | 0.1974 | 0.1550 |
| Distributed-strict 003 | 8 | 1.6965 | 1.2177 | 0.2353 | 0.1031 |
| **Global-strict 003 — new gold standard** | **10** | **0.2167** | **0.1799** | **0.3507** | **0.3790** |

Additional global-strict metrics: **10-scan translation RPE RMSE 0.0447 m**; **50-scan translation RPE RMSE 0.0923 m**; **first-pose ATE 95th-percentile translation error 0.3520 m**. Distributed-strict's 50-scan RPE is slightly lower (0.0866 m), so global-strict is not the best in every diagnostic.

**Interpretation:** The first-pose ATE improvement is particularly compelling because both trajectories are brought into the same initial position and heading, and no later best-fit alignment can hide accumulated disagreement. The still lower rigid XY error supports agreement in overall trajectory shape. Endpoint IoU remains only a **trajectory-conditioned LiDAR-hit raster proxy**, not an actual exported `.slammap` quality measurement.

## 3. What the loop diagnostics show

| Diagnostic | Distributed-strict 003 | Global-strict 003 |
|---|---:|---:|
| Last keyframe searched | 3112 | 3112 |
| Submap registration attempts | 1,351 | 2,060 |
| Submap geometry verified (first stage) | 625 | 789 |
| Reciprocal raw-scan checks passed | 33 | 41 |
| Reciprocal forward checks rejected | 590 | 746 |
| Reciprocal cycle checks rejected | 2 | 2 |
| Verified candidate pool after per-keyframe filtering | 21 | 23 |
| Accepted loop factors | 8 | **10** |
| Late accepted factors (current scan ≥1769) | 1 | **3** |
| Appearance-retrieved registration attempts | 0 | **709** |
| Appearance geometry verified (first stage) | 0 | **164** |
| Appearance loops accepted by GTSAM | 0 | **2** |
| Configured loop ceiling | 1,000 | 1,000 |

The strict gate is selective: only **41 of 789** initially submap-verified global candidates (**5.2%**) passed raw-scan reciprocal checking. More proposals were not automatically promoted into graph constraints. The 1,000-loop ceiling did not determine the final count; geometric rejection, one-candidate-per-keyframe choice, corroboration, and redundancy limits did.

### Two additional long-range global loops

All **eight** distributed-strict accepted pairs also appear in global-strict. Global-strict adds exactly two appearance-retrieved factors:

| Historical → current scan | Separation | Reciprocal overlap | Scan-registration RMS | Reverse-cycle translation error |
|---|---:|---:|---:|---:|
| **1112 → 2768** | 1,656 scans | 94.10% | 0.0495 m | 0.0072 m |
| **1144 → 2800** | 1,656 scans | 100.00% | 0.0477 m | 0.0009 m |

Both satisfy the strict geometry and historical-anchor corroboration checks. They link nearly corresponding, temporally separated portions of the recording; their proximity to one another also means they are **not two independent physical confirmations**.

**Leading explanation—not yet proven:** These two global constraints are the primary structural difference between the two strict graph solutions and plausibly account for most of the ATE and visual-map improvement. An explicit one-factor-at-a-time ablation is still needed to establish their individual and combined effects.

## 4. The resulting optimized map

The uploaded `fr079-ceresgraph-global-strict-003.graph_optimized.slammap` is a verified FUSMAP2D v1 artifact with **655 × 749 cells at 0.10 m resolution** and **3,118 occupancy-grid revisions**. Its footprint and walls are reconstructed from the graph-optimized trajectory, rather than displayed from the no-loop frontend map. The user assessed the resulting PGM as the **most consistent FR079 map to date**, superseding `poseprior-001` as the visual gold standard.

The binary snapshot's footer checksum was verified before the map illustration in the companion Word document was rendered. The visualization is a cropped display of the **actual uploaded optimized occupancy grid**, not a simulated reconstruction or the original frontend PGM.

**Artifact fingerprint:** `14491087929584304320` (FUSMAP2D snapshot fingerprint).  
**Uploaded binary SHA-256:** `ded1c94067626a101671257b3f61b47c96157ef8ad5660cf08ee8a247da925d5`.

## 5. What this milestone establishes—and what remains open

### Established in the FR079 experiment

- Global shape-based retrieval finds additional long-range revisit hypotheses that Ceres-proximity-only discovery misses.
- Reciprocal raw-scan verification sharply filters one-way submap matches before GTSAM admission.
- A sparse set of **10** strictly verified loop factors is sufficient to produce a much better reference-relative trajectory than the older 24-factor submap variants.
- The resulting optimized map has become the project's **new visual benchmark**, according to inspection of its PGM, and the numerical benchmark strongly corroborates that milestone.

### Not yet independently established

- **Independent physical localization error:** The Freiburg `fr079-sm.log.gz` poses are another estimated trajectory, not surveyed ground truth.
- **Independent wall/map accuracy:** The available endpoint IoU reprojects scan hits; it does not score the full exported free/occupied/unknown occupancy cells against an independently registered survey.
- **Cause of the improvement:** The two extra appearance factors are the leading explanation, not a proven isolated causal result.
- **Graph-deformation safety:** `graph_shift_within_safety_limit` is **false** in this run; the reported maximum graph shift is **3.8946 m**. This is a diagnostic warning, not proof of a failed optimization, and merits closer examination.
- **Generalization:** One FR079 recording does not establish the same benefit across different datasets, sensors, geometries, or drift regimes.

### Next validation decisions

1. **Freeze and preserve `global-strict-003`** as an immutable gold-standard artifact, including its optimized `.slammap`, PGM/YAML preview, loop diagnostics, reference benchmark, input hash, and code revision.
2. **Ablate the two new global factors** using otherwise identical inputs: the eight common factors alone; plus `1112 → 2768`; plus `1144 → 2800`; and all ten. Compare maps, ATE/RPE, and graph deformation.
3. **Score actual occupancy grids** with the project's map-comparison tool under one shared frontend-to-reference rigid alignment. Report wall F1 at 15/30 cm, symmetric wall distance, 95th-percentile distance, occupied IoU, and known-area coverage. Develop separate wall-double-thickness and contradictory-free-space diagnostics; do not design them merely to force a preferred result to win.
4. **Evaluate robustness** on further sequences and inspect the late-loop region for false-positive similarity caused by repeated corridor structures.

## 6. Reproduction and provenance

Use the same raw FR079 dataset and frontend settings as the historical experiments; do not overwrite any previous run directory. Example of the strict global arm (requires the project's Docker test environment and FR079 log):

```bash
export FR079_LOG="$PWD/.build/datasets/fr079.log.gz"
export SLAM_DOCKER_JOBS=28

docker compose build test

docker compose run --rm \
  -v "$FR079_LOG:/input/fr079.log.gz:ro" \
  -v "$PWD/apps/demo/maps:/output" \
  -e SLAM_HOST_OUTPUT_DIR="$PWD/apps/demo/maps" \
  --entrypoint ./build-docker/slam_fr079_offline test \
  /input/fr079.log.gz /output/fr079-ceresgraph-global-strict-003 \
  --run-directory --continuous-cost --match-every 5 --field-every 15 \
  --map-stride 1 --match-stride 4 --distance-truncation 0.5 \
  --lidar-loop-closure --frontend-sequential-graph \
  --loop-registration scan-to-submap-global
```

The command uses a **new, exclusive run directory**: if `global-strict-003` already exists, choose a new suffix rather than overwriting the gold standard.

Evaluate with `tools/fr079_reference_benchmark.py` using the unchanged raw and published scan-matched logs; run actual occupancy-grid comparison separately with `tools/fr079_map_compare.py`. See [FR079 reference benchmark and reproducibility guide](fr079-reference-benchmark.md) for current commands and metric limitations.

**Evidence sources:**

- Uploaded `fr079-reference-global-strict-003.json` and `.csv`: 3,118-pose trajectory metrics and endpoint-map proxies.
- Uploaded `fr079-ceresgraph-global-strict-003.loops.json`: candidate counts, per-loop strict checks, accepted factor pairs and graph-shift diagnostics.
- Corresponding distributed-strict 003 reports: direct control comparison with the same Ceres frontend.
- Uploaded `fr079-ceresgraph-global-strict-003.graph_optimized.slammap`: verified graph occupancy-grid artifact and map rendering.
- Repository source: [`src/fr079_loop_closure.cpp`](https://github.com/civcode/2d-fusion-slam/blob/experiment/fr079-ceres-ablation-beam-sweep/src/fr079_loop_closure.cpp), [`src/pose_graph_gtsam.cpp`](https://github.com/civcode/2d-fusion-slam/blob/experiment/fr079-ceres-ablation-beam-sweep/src/pose_graph_gtsam.cpp), and the FR079 benchmark guide.

---

**Milestone conclusion:** Global-strict 003 combines long-range appearance-based loop discovery with the conservative reciprocal LiDAR evidence gate from pose-prior. It is the **current best FR079 mapping result**, with a visually consistent optimized occupancy grid and a step-change in reference-relative trajectory accuracy. Preserve this run as the baseline for future improvements, while keeping the absence of independent ground truth and the need for factor ablation explicit.
