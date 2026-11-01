# Held-out FR079 Ceres scan-to-frozen-map evaluation

**Status:** source-rebuilt, scan-index-excluded Ceres diagnostic evaluation.
It measures scan-to-map **geometric consistency**, not absolute trajectory
error, localization accuracy, real laser calibration, or independent truth.

## Strict split and what it actually verifies

The existing `build_fr079_training_map` constructs the frozen occupancy
map using only original scan indices satisfying BOTH:

- `index % 5 != 0` (exclude the held-out fifth of the dataset), and
- `index % 6 == 1` (sparse training insertion).

`slam_ceres_replay_export --verified-fr079-holdout` parses the **original**
recording, requires the exact FR079 training export source label, and
**rebuilds the training map from all source scans using the original
C++ training-map function**. It refuses to evaluate if the saved map's
fingerprint differs from the rebuilt map. After that check, only
`index % 5 == 0` scans can be evaluated. The command's positional
`scan-stride` in this mode is the stride *within the held-out subset*,
so `12` selects source indices `0, 60, 120, ...`.

The Python `match_ceres.py` wrapper also checks the source log SHA-256
against the exact recorded `FUSREPLAY1` session and checks that all
expected held-out indices appear, in order, without any missing or
training-eligible entries. It writes a `FUSMATCH1` sidecar plus a
standalone `FUSHOLDOUT_REPORT1` JSON report containing input hashes,
map fingerprint and descriptive metrics. Files are created exclusively;
neither the map nor the original replay is modified.

**This is strict scan-index nonleakage for this *specific* FR079
training-map recipe**, verified by recomputing the actual binary map.
It is *not* a guarantee against correlated adjacent scans or a proof
that the logged poses, sensor extrinsics or maps are physically correct.
FR079 has only scan-level timestamps in this pipeline; per-beam timing
remains unverified. The public frame label is a declaration, not a
surveyed frame transform. Never report ATE/RPE without a separate,
independently surveyed trajectory.

## Reproduce on FR079

Use the same original `fr079.log.gz` to build the reference training-only
FUSMAP2D export and the recorded replay session:

```sh
cmake -S . -B build -G Ninja \
  -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON
cmake --build build --target \
  slam_fr079_map_export slam_ceres_replay_export --parallel 2

mkdir -p apps/demo/maps
./build/slam_fr079_map_export /path/to/fr079.log.gz \
  apps/demo/maps/fr079-training
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED

python3 apps/demo/match_ceres.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json \
  apps/demo/maps/fr079-training.slammap \
  --executable build/slam_ceres_replay_export \
  --verified-fr079-holdout --scan-stride 12 --beam-stride 10
```

The command prints a JSON summary and writes two **separate** files
next to the session:

- `<session>.ceres.json`: validated per-frame FUSMATCH1 results
- `<session>.holdout-report.json`: source-linked aggregate report

To rerun with different settings, explicitly archive/remove the old
outputs first. The viewer has no write endpoints. Start
`python3 apps/demo/main.py --data-dir apps/demo/maps`, visit
`http://127.0.0.1:8765/replay`, select the session, and inspect the
new **training-excluded evaluation** banner. Orange pose/hit overlays
remain optional and are shown only for Ceres-accepted frames.

## Reproducible measurements and limitations

On the existing FR079 recording and matching training-only map,
`--scan-stride 12 --beam-stride 10` processed **52** truly excluded
scan indices: **43 accepted**, **5 insufficient support**, **4 no
improvement**. Mean EDT overlap was **84.29%**. Mean robust cost
was **69.81 before** and **66.02 after** the attempted match; mean
supported EDT distance was **0.1352 m**. These are **internal matcher
metrics** on a spatially correlated traversal, not surveyed pose error.
The average robust costs are aggregated over accepted *and rejected*
queries, so they are not a monotonic convergence certificate.

Input/output checks: native exporter is built with the actual Ceres
solver; Python validation preserves compatibility with older unrestricted
sidecars while refusing mismatched-map/recording and incomplete held-out
selections. Unit+HTTP tests, a synthetic native map-rebuild/nonleakage
regression and a strict-warning/UBSan native run passed locally. The
complete Ubuntu Docker CTest matrix remains **unverified here** (Docker
engine unavailable). No GitHub Actions workflow was run.

**Next:** obtain independent calibration and timed sensor data; evaluate
complete (non-subsampled) held-out trajectories when feasible and validate
against independently measured pose truth or ground-control geometry.
