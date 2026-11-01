# Freiburg FR079: opt-in public 2D LiDAR input

The **recordings are supplied by the user** and deliberately kept outside Git.
The repository tracks their SHA-256 identities, format decoder, unit tests
and opt-in complete-log validation targets. No build automatically downloads
a benchmark. Copy the four uploaded files into a local directory and check:

~~~sh
cd /path/to/local/fr079
sha256sum -c /path/to/2d-fusion-slam/datasets/fr079/SHA256SUMS
~~~

| File | Role |
| --- | --- |
| `fr079.log.gz` | Main CARMEN FLASER/ODOM log, full input |
| `fr079-sm.log.gz` | Variant with different logged pose fields, **not independent trajectory truth** |
| `fr079-maps.tgz` | Supplied CARMEN/Bee reference maps and GIFs at 3, 5, 10 cm; CARMEN v020 binary grids now decode, but map-frame registration and occupancy snapshot import remain pending |
| `map05cm-path.gif` | Supplied illustration of map and a route; not machine-readable pose ground truth |

## Reproduce real-data tests

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/extracted/Catch2-3.16.1 \
  -DSLAM_FR079_LOG=/path/to/fr079.log.gz \
  -DSLAM_FR079_SM_LOG=/path/to/fr079-sm.log.gz \
  -DSLAM_FR079_MAP=/path/to/extracted/fr079-maps/fr079-05cm.carmen.map
cmake --build build
ctest --test-dir build --output-on-failure
./build/slam_carmen_inspect /path/to/fr079.log.gz 3118
~~~

**Observed on the supplied reference CARMEN maps:** 3 cm: 1517 × 537 cells
(444,630 known / 369,999 unknown); 5 cm: 910 × 323 cells
(164,728 known / 129,202 unknown); 10 cm: 455 × 162 cells
(43,275 known / 30,435 unknown). No beam-to-map frame alignment is asserted.

**Observed on the supplied inputs, for each log:**
3,118 FLASER scans; 5,761 ODOM records; 1,122,480 range samples
(360 per scan), of which **1,113,178** are below or equal to the configured
80.99 m usable range and **9,302** fail the conservative range check.
The decoder follows the CARMEN record layout: original laser pose, logged
odometry pose, scan IPC time, logger time and host. The 180-degree front
scanner model uses 360 half-open angular bins at 0.5-degree increments,
ranging from -90 degrees to +89.5 degrees.

## Evidence boundaries

- **No per-beam acquisition timestamps.** The log gives a scan-level
  timestamp; it cannot validate LD19 packet acquisition or raw IMU fusion.
- **No trustworthy external ground truth is asserted.** The CARMEN laser
  pose and odometry pose fields are logged values; the `fr079-sm` version
  must not be silently treated as an independent reference trajectory.
- **No fabricated free-space returns.** Missing, zero or over-range values
  are classified as invalid. They are excluded from matcher point clouds
  and must not create free-space occupancy evidence.
- **Reference map file decoding and storage-axis verification are implemented, not full metric map registration.**
  `slam_carmen_map_inspect` reads CARMENMAPFILEv020 binary grids and checks
  float values; axis conventions, registration against the scan/log pose frame,
  import into authoritative occupancy snapshots and metric comparison remain future work.
  The GIF alone is a qualitative reference, not numerical ground truth.
- **No replay/SLAM benchmark claimed.** Full file parsing is only the
  first public-dataset integration test. The next experiments must hold
  out scans and report pose/mapping error where an independently verified
  truth source exists.

The parser library is plain C++20; only the optional gzip CLI uses zlib.
The input files and compressed third-party sources must not be pushed
to GitHub just to make CTest pass.


## Reproducible frame-layout and held-out diagnostic

The source `CARMENMAPFILEv020` payload is **X-major**, indexed as
`values[x * height + y]`. Its Y coordinate increases upward; for the
provided GIF the corresponding pixel row is `height - 1 - y`. The
independent optional verifier checks the source binary map directly against
the supplied original 5 cm GIF (not the annotated path image):

~~~sh
python -m pip install Pillow  # optional analysis dependency; NOT needed for C++
python tools/verify_fr079_map_layout.py \
  /path/to/fr079-05cm.carmen.map /path/to/map05cm.gif
~~~

On the uploaded files, all **16,803 high-occupancy map cells** aligned with
dark GIF pixels and all **125,987 near-zero free cells** aligned with white
GIF pixels under the documented thresholds. This proves **raster layout**,
not an external metric origin or pose alignment.

With local Catch2, zlib and the two FR079 artifacts supplied:

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/Catch2-3.16.1 \
  -DSLAM_FR079_SM_LOG=/path/to/fr079-sm.log.gz \
  -DSLAM_FR079_MAP=/path/to/fr079-05cm.carmen.map
cmake --build build
ctest --test-dir build --output-on-failure
./build/slam_fr079_alignment_inspect \
  /path/to/fr079-sm.log.gz /path/to/fr079-05cm.carmen.map
~~~

The diagnostic uses **scans whose index modulo 5 is nonzero** for
calibration (further subsampled), and **index modulo 5 equals zero** for
held-out queries. Its fixed grid search checks 8 axis-permutation/sign
hypotheses and translation offsets. **It does not use held-out queries
for transform selection.** All failures outside the map, inside unknown
cells or away from mapped walls are counted as nonmatches.

The uploaded `fr079-sm` recording selected a reflected-Y hypothesis:
`swap_xy=false, sign_x=1, sign_y=-1` with an apparent shift
`(22.25 m, 8 m)`. This has a **negative determinant**, so it
cannot be used as an `SE(2)` robot-pose transform. On the **624
held-out queries**, known-space coverage was **75.105%** and 15 cm
near-wall endpoint coverage **27.9112%**, at a fixed, diagnostic-only
threshold. These are **not localization accuracy** metrics.

The published map was built from the same source recording, so the
held-out scans are **not independent of the reference map**. Logged
laser poses are also estimates. Consequently this is **a frame
registration and data-integrity diagnostic only**, not an independently
held-out SLAM evaluation. See [versioned measured evidence](alignment-evidence.json).
A true benchmark needs a training-only map, independent ground truth
(or explicitly labeled proxy metrics), and physically justified frame
registration before running the Ceres tracker against FR079.


## FR079 training-only map and Ceres scan-matching proxy

The new `slam_fr079_train_only_benchmark` does **not** consume the
archived FR079 reference map, which contains observations from the full
recording. Instead it builds a new 0.10 m occupancy map from a
deterministic **training-only** subset of `fr079.log.gz`:
indices divisible by 5 are withheld entirely, and nonheldout indices
with remainder 1 modulo 6 contribute mapped scans. Training scans
alone define grid bounds. Withholding **includes** scan geometry and
logged poses; neither can influence the immutable training snapshot.

Every 12th held-out query is matched against that frozen map with
Ceres (or explicit `--reference` mode). Seed = the held-out
scan's logged pose estimate plus (0.07 m, -0.05 m, 0.02 rad).
No missing/invalid/range-over-18-m beam generates free-space evidence.
The synthetic leakage unit test mutates held-out laser observations
and poses and requires the training-map fingerprint to remain identical.

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CERES=ON -DSLAM_REQUIRE_CERES=ON \
  -DCeres_DIR=/path/to/ceres-prefix/lib/cmake/Ceres \
  -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/Catch2-3.16.1 \
  -DSLAM_FR079_LOG=/path/to/fr079.log.gz
cmake --build build
ctest --test-dir build --output-on-failure
./build/slam_fr079_train_only_benchmark /path/to/fr079.log.gz
~~~

This data is not shipped in Git and the real-data CTest only registers
when `SLAM_FR079_LOG` is specified. Ceres selection fails
closed when Ceres is absent. Append `--reference` to
explicitly select the lightweight local reference matcher.

**Results on supplied file:** 3,118 scans; 2,494 eligible training,
416 inserted map scans; 624 held out, 52 evaluated queries; 42 accepted
Ceres improvements, 10 unchanged. Overlap 0.840449 → 0.851909;
mean robust geometric cost 86.2551 → 80.4663.
The logged trajectory is used for both map placement and held-out
pose initialization. Such pose estimates can already incorporate scan
matching, and adjacent scans share geometry. **This is not external
ground-truth accuracy or a statistically independent traversal.**
Do not report ATE/RPE or physical beam-time claims from this test.
See [versioned result evidence](train-only-evidence.json).


## Export the training-only map

The optional zlib-enabled slam_fr079_map_export CLI exports the 416-scan **training-only** map to an authoritative versioned binary bundle and a lossy PGM/YAML visualization pair. It never uses held-out observations to compute map bounds or occupancy.

~~~sh
./build/slam_fr079_map_export /path/to/fr079.log.gz /tmp/fr079-training
./build/slam_map_inspect /tmp/fr079-training.slammap
~~~

Outputs are /tmp/fr079-training.slammap, .pgm, and .yaml. The binary is verified by a roundtrip import before the CLI reports success. The FR079 logged-laser coordinate origin remains **unverified** and is labeled accordingly in the bundle. The supplied full-recording reference map is **not** used.

With SLAM_FR079_LOG configured, CTest registers fr079_map_export followed by fr079_map_import using fixture ordering. These real-data checks are optional because recordings are kept outside Git.

Locally observed: 391 × 539 cells, 57,529 known, 153,220 unknown, revision 416 and fingerprint 13991754243820308900. See [bundle format](../../docs/map-bundle-v1.md).
