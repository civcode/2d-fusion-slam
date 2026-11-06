# FR079 multi-view visibility consistency — diagnostic experiment

**Status:** opt-in diagnostic, no changes to SLAM estimation. **Baseline:** tagged `fr079-global-strict-003` (merged into `main`). This experiment is on a separate branch. It is **not** a new loop-closure verification or an accuracy claim.

## Question

The `global-strict-003` occupancy map is currently the visually preferred FR079 result, but some narrow transitions and wall regions appear inconsistent. Can errors in **local multi-scan pose alignment** explain these artifacts, rather than a missing long-range loop closure? Opposite sides of a partition can legitimately produce two *different* LiDAR surfaces. Therefore, occupied endpoint coincidence alone is **not** a valid expectation.

## Method

`tools/fr079_visibility_consistency.py` accepts the *original* raw CARMEN log, the unchanged optimized pose sequence from `*.loops.json`, and **two disjoint scan-index intervals**. It ray traces FLASER/RLASER hit beams into temporary 0.10 m cells, separately for each interval, and records:

- **Occupied evidence:** a valid LiDAR beam terminated at a cell.
- **Free-space evidence:** a valid beam traversed a cell *before* its endpoint (0.20 m safety buffer).
- **Unknown:** neither interval has adequate evidence for a cell; unknown is not free.
- **Within-interval ambiguous:** both occupied and free evidence are repeatedly reported in the same interval. These cells are excluded from unambiguous comparisons.
- **Frontend-vs-optimized control:** when `graph_control_poses` is present (as in the gold-standard FR079 run), the *same* ray evidence is evaluated both at the unchanged Ceres frontend poses and at the GTSAM loop-optimized poses. This can reveal whether a consistency problem existed before loop closure or was introduced or reduced by graph optimization. Neither pose set is independent truth.

An observation of a wall face from the opposite side need not coincide with the first face. A *contradiction* means **one interval repeatedly reports a hit in a cell another interval repeatedly observes as free**. This is a flag for inspection, not a claim of physical impossibility: moving objects, imperfect pose estimates, beam-angle/calibration errors, transparent obstacles, grid discretization and sensor noise can also cause disagreements.

Cells need at least two supporting **distinct scans** per observation class by default, not two beams from one scan. Samples are divided by original temporal interval, rather than comparing a scan to a global map that contains that scan. Both windows use the **same frozen graph-optimized coordinate frame**; the tool never estimates a new transform. Contradiction rates are conditional on overlapping observed cells; non-overlap is reported as unknown, not a zero error rate.

This tool reads the original log but **does not** consume Freiburg's published scan-matched pose log or map for estimation or scoring. It checks that the graph's `source_sha256` matches the decompressed raw log and that scan counts align. Neither log nor any `.slammap` is modified.

## Run

Use this from the repository root, with your existing gold-standard output directory and raw FR079 recording:

```bash
python3 tools/fr079_visibility_consistency.py \
  --raw-log .build/datasets/fr079.log.gz \
  --loops apps/demo/maps/fr079-ceresgraph-global-strict-003/fr079-ceresgraph-global-strict-003.loops.json \
  --window-a 1000:1060 \
  --window-b 2740:2800 \
  --out reports/fr079-multiview-example
```

The intervals above **illustrate syntax**, not a verified selection corresponding to either red rectangle. Use scan windows that actually observe the two marked regions; choose them by inspecting the optimized trajectory and scan positions. Every interval is half-open `[START, END)` and must be non-overlapping. To focus on a spatial patch, add:

```bash
  --region XMIN,YMIN,XMAX,YMAX
```

`--region` is in **world metres in the existing optimized SLAM coordinate frame**, not screenshot pixels. The screenshot alone cannot identify these exact metric bounds without the map viewer's transform. Check the diagnostic's `suggested_pose_proximity_pairs` for exploratory *scan-index suggestions* across different headings; those suggestions are **not** validated loop closures and are not necessarily first-traversal transitions. Adjacent, non-overlapping intervals may be examined explicitly to diagnose first traversals.

Other options include `--scan-stride 3`, `--beam-stride 4`, `--resolution 0.10`, `--min-observations 2`, and `--suggest-pairs 16`. For small transitions, use a narrower scan stride (for example `--scan-stride 1`) and keep parameters identical when comparing regions/runs. The tool writes a non-overwriting JSON report and the optimized-pose PNG classification overlay; if the graph-control poses exist, it also writes `*.control.png` and `frontend_control_metrics` in JSON. The PNG colors represent occupied cells from window A and B, common occupied cells, repeated contradictions (red), ambiguity (grey), and observed free/unknown cells. The PNG is **diagnostic ray evidence**, not a replacement for `graph_optimized.slammap`.

## Reading the report

The relevant fields are `metrics.a_occupied_b_free_cells`, `metrics.b_occupied_a_free_cells`, the conditional contradiction fractions, `metrics.joint_known_cells`, `metrics.joint_occupied_cells`, and within-window ambiguous counts. Compare these with the same keys under `frontend_control_metrics` (if available). The two outputs use the same world coordinate frame and scan groups but differ in pose estimates; the comparison is still not causal proof of a particular graph factor's effect. If `a_to_b_contradiction_fraction` is `null`, no occupied A cell was decisively observed by B: **there was insufficient common visibility**, not a perfect match. Compare paired windows and a small region before/after a suspect transition, and look for clusters in the overlay rather than relying on a single aggregate scalar.

## Validation and follow-up decision

The regression fixture deliberately covers (a) same-surface observations, (b) valid opposite wall faces, (c) a contradictory ray crossing an observed wall, (d) no common visibility, (e) invalid window/provenance, (f) PNG encoding and (g) an end-to-end synthetic CARMEN log with both frontend and optimized pose tracks. Run it locally:

```bash
python3 tests/fr079_visibility_consistency_tools_test.py
```

This experiment **does not change** the original 82% reciprocal scan-overlap gate or add new graph factors. First collect local consistency evidence from the two red-marked areas on `global-strict-003`. If the anomalies are attributable to local pose errors, design a separate multi-scan local consistency factor with hold-out rays and explicit observability. If the evidence instead reveals genuine long-range revisits with insufficient single-scan visibility, prototype a **separately named** bidirectional visibility-aware submap verifier in shadow mode, then test its false-positive behavior before permitting any new GTSAM constraint. Do not use the two neighboring overlapping historical submaps as independent confirmation, and do not reinterpret opposite wall faces as the same surface.

The tag `fr079-global-strict-003`, its trajectory/map outputs, and existing default estimator behavior remain frozen throughout.