# Implementation status and verification evidence

**Updated:** 2026-10-31  
**Branch:** `main`  
**Scope:** Synthetic scan matching, verified FR079 map **raster layout**, train/held-out frame diagnostic and local Ceres solver-adapter runtime test; **not complete offline SLAM or a quantitative independent public-data SLAM benchmark**.

## Implemented and locally exercised

| Target | Working functionality | Evidence |
| --- | --- | --- |
| `slam_core` | Checked nanosecond/epoch time and SE(2) geometry | `core_smoke` |
| `slam_fixtures` | Deterministic exact-time apartment/warehouse synthetic ray casting | `fixtures_smoke` |
| `slam_sensors` and `slam_ld19` | Beam timing/quality, partial scan assembly, 47-byte LD19/CRC/frame/rollover parsing | `scan_mapping`, `ld19_motion` |
| `slam_motion` | Sampled SE(2) trajectory interpolation and deskew diagnostic safety | `ld19_motion` |
| `slam_mapping` | Known-mask log-odds grid and safe ray updates | `scan_mapping` |
| **`slam_distance_field`** | Immutable deep-copy occupancy snapshot with pinned revision and non-cryptographic fingerprint, exact 2-pass EDT, map-frame bilinear gradients, known-mask and nontruncated residual support | **`distance_field`** |
| **`slam_match_geometry`** | LiDAR geometry-only Huber robust EDT scoring and a small damped reference optimizer; synthetic pose recovery with overlap, rank/eigenvalues and source-coherence diagnostics | **`match_geometry`** |
| `slam_match_ceres` (optional) | Ceres single-scalar-per-beam cost-function adapter and bounded solver | Repository adapter file **compiled** in an isolated Ceres 2.2 consumer with interface-compatible headers; full repository Ceres regression still not run |
| `slam_carmen` + `slam_carmen_geometry` | Strict CARMEN `FLASER`/`ODOM` parsing and conservative native laser-point conversion (scan timestamp only) | Catch2 importer/geometry tests passed |
| `slam_carmen_inspect` | Local `.log`/`.log.gz` summary and optional expected scan count; zlib remains CLI-only | Both user-uploaded complete FR079 logs parsed in CTest |
| `slam_carmen_map`, `slam_carmen_map_inspect` | CARMENMAPFILEv020 binary grid decoding with X-major/Y-up addressing and native known/unknown samples | Synthetic Catch2 grid test; actual 3/5/10 cm files decoded, 5 cm map checked in CTest |
| `slam_fr079_alignment` | Training-only search over explicit map-frame hypotheses, held-out scan endpoint coverage and reflection/proper-rotation classification | 8/8 dataset-subset CTest passes, including FR079-sm full log/map run; not pose accuracy |

**Test evidence:** GCC 14, CMake 3.31.6, Ninja, provisioned Eigen3 on Linux x86-64. **6/6 pre-existing CTest tests passed** with network-free `SLAM_ENABLE_CATCH2=OFF`, including the normal build and a strict build using `-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-sanitize-recover=all`. The EDT tests compare all cells to exact Euclidean reference distances, test rotated grids, immutable source revisions, unknown stencils and truncation. The matching tests check geometry-only convergence toward synthetic truth, overlap, rank, cost reduction and unsupported-region rejection. These are **unit/synthetic tests**, not a public-dataset accuracy benchmark.

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

Use `-DSLAM_EIGEN_INCLUDE_DIR=/path/to/eigen3` if the Eigen package is not automatically found. Ceres 2.2.0 was compiled and installed locally from the uploaded source with Eigen3/miniglog; a standalone Ceres least-squares consumer solved successfully. The byte-identical `src/match_ceres.cpp` compiled against that installation in an isolated adapter build, **but the full repository Ceres CTest has not yet been run**. Catch2 3.16.1 was also built from the uploaded source; 6/6 additional local integration CTests passed (CARMEN parser, two complete log imports, native laser geometry, synthetic binary map, real 5 cm grid). That additional suite does **not replace** the original six-test regression suite. No GitHub Actions were created or executed.

## Guarantees and limitations

- **Snapshots are immutable after publication**, but capture is only coherent if the caller serializes copying with map insertion. There is no multi-threaded map owner/rebuild scheduler yet.
- **EDT is exact at cell centres**, with bilinear interpolation using four observed cells and entirely nontruncated stencil values. Unknown/out-of-map/saturated support is **not** a false zero residual. The derived cache is not map authority.
- **Matcher is a local, geometry-only baseline** seeded near the map pose. Its synthetic test uses independent, known synthetic landmark geometry but is not a full SLAM trajectory/loop benchmark. No relocalization/global search or verified sensor bias/clock estimator is implemented.
- The information matrix and its eigenvalues are **observability diagnostics**, not a calibrated independent covariance. A returned tracking pose is not an automatically graph-eligible factor. A local matcher can still have map/deskew correlations.
- **Ceres solver adapter compiles and runs in an isolated analytic harness; whole-program project test remains.** An unsupported trial evaluation fails rather than yielding an artificial zero-distance residual. The repository matcher end-to-end convergence, false-lock and boundary behavior still need a full Ceres-linked CTest and independent recordings.
- LD19 packet time-reference, MPU6050 gyro bias, real acquisition/MCAP, actual Pi performance, public-dataset SLAM accuracy and full online/offline SLAM remain unverified. FR079 log parsing is verified, **not** full SLAM benchmarking. FR079 CARMEN v020 grid files are decoded into read-only file-order values, but not yet frame-registered, converted to authoritative occupancy snapshots or used for accurate pose scoring.

## Next checkpoint

Run the complete project Catch2/Ceres CTests against the supplied offline dependencies, validate CARMEN reference-grid axis/frame conventions and implement a reproducible FR079 scan-to-map evaluation with held-out frames and truthful source timing, then add **deterministic map export and a simple offline pose-only GTSAM reference graph**, with conservative factor provenance and false-loop checks. Independently progress real Pi LD19/MPU6050 acquisition and data calibration; initial algorithm experiments continue using synthetic and verified public data.


## 2026-11-01: FR079 frame and Ceres progress

- **Raster layout verified:** Source binary occupancy is X-major with Y-up.
  The independent GIF verifier reports **100% agreement** at both 16,803
  strongly occupied and 125,987 near-zero free map cells. The map's
  **metric frame origin and handedness versus logged laser poses are not
  independently specified.**
- **Reference-map registration diagnostic built:** 3,118 `fr079-sm` scans
  split into 2,494 non-heldout and 624 heldout scans. Grid search uses
  only a deterministic subset of the non-heldout queries. Best fit is
  `swap_xy=false, sign_x=1, sign_y=-1`, an **improper (reflected)
  transform**; heldout known-space coverage 0.751050 and near-wall
  coverage 0.279112. This is not admissible as an SE(2) pose estimate.
- **Source-map leakage:** The supplied reference map was generated from
  the same recording. A held-out *query* is **not** an independently
  held-out *mapping observation*. Do not infer ATE/RPE or independent
  accuracy from the diagnostic.
- **Local tests:** 8/8 dataset/integration CTests pass with GCC 14,
  Catch2 3.16.1 and zlib, both normally and under
  `-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined`.
  This is a locally assembled dataset-subset build, **not the complete
  repository-wide CMake/Ceres suite**.
- **Ceres runtime:** The repository's byte-identical `src/match_ceres.cpp`
  was linked to Ceres 2.2.0 in an *isolated analytical EDT-style
  harness*. A 50-landmark optimization reduced robust cost from
  approximately 5.76697 to 7.0e-20, with final translation/yaw
  error near zero. **This synthetic harness does not substitute for**
  the official `match_ceres` CTest using real immutable map/EDT code.

The next **unverified** gate remains: the complete checkout's Ceres
CTest (including Jacobian and rank/outlier rejection), a justified
rigid map-registration model, and a training-only map with held-out
FR079 scan-to-map matching against a truthful reference or explicitly
proxy quality metrics. No GitHub Actions were used.


## 2026-11-01 — FR079 training-only scan matching

Implemented `slam_fr079_train_only`: build mutable occupancy only
from nonheldout scan indices and bounds, publish a frozen immutable
map/EDT generation, then evaluate withheld scans using either the
existing Ceres 2.2.0 matcher (if installed) or an explicitly selected
reference matcher. A unit regression modifies every withheld scan and
its logged pose, confirming the training-map fingerprint is unchanged.

**Local focused validation:** project-compatible C++20 map/matcher
interfaces linked with real Ceres 2.2.0, plus the existing synthetic
matcher Ceres regression test: **2/2 CTest tests passed** normally and
with `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined`.
This was a **focused locally assembled build**, not the whole-project
CMake test matrix. The supplied full `fr079.log.gz` gave 416 mapped
training frames, 624 heldout frames, 52 evaluated queries; 42 Ceres
accepted improvements and 10 no-improvements. Mean overlap improved
0.840449 → 0.851909 and robust cost dropped 86.2551 → 80.4663.

**Truth limitations:** this is a real-data **geometric proxy**.
The logged laser poses used for map placement and heldout seed
perturbations are estimates, not independent truth. Temporal
correlations and possible upstream scan-matching correlation remain.
No ATE/RPE, IMU timing or external map-frame pose accuracy is proven.
See [FR079 training-only evidence](../datasets/fr079/train-only-evidence.json).

**Remaining gates:** full source-tree Ceres/Catch2 test matrix, a
measured independent pose reference or separate traversal, detailed
false-lock/bias profiling, then deterministic offline map export/graph.


## 2026-11-01 — Versioned offline map export

Implemented FUSMAP2D-v1 authoritative binary export and import. It serializes a deep-copy immutable map snapshot, its separate known mask and exact log odds, revision/fingerprint, grid origin and resolution, integration policy, and descriptive coordinate-frame provenance. Import validates version, bounds, flags, finite cells, checksum and content fingerprint. The FNV-1a footer is **only accidental corruption detection**, not authentication.

The new slam_fr079_map_export CLI generates a binary map plus visualization-only PGM/YAML from the original **training-only** FR079 split. A full local run exported a **391 × 539** grid at 0.10 m, with **57,529 known / 153,220 unknown cells**, revision **416**, and fingerprint **13991754243820308900**. Repeated exports produced byte-identical binary maps and PGM rasters. Binary SHA-256: `fa6e9107b1eee7408f31c0bd0f3006b36de852a0bb67ed4ac5f81b20e11c7f5d`.

**Local validation:** A focused CMake build linked to Ceres 2.2.0; **3/3 CTest passed** normally and with GCC 14 warnings-as-errors and UBSan. Tests cover binary roundtrip, unknown/known separation, immutable fingerprint, EDT equality, corrupted/truncated versions and real-data export/inspect. The complete repository CI matrix has **not** been run after this change. The user reported a successful earlier manual CI run; no workflow was triggered for this step.

**Limitations:** The PGM/YAML loses authoritative known-mask/log-odds precision; use the binary snapshot for localization. FR079 map origin is not physically verified, its logged poses are estimates, and the bundle does not contain a pose graph, submap lifecycle, loop constraints or GTSAM results. See [format contract](map-bundle-v1.md).

## 2026-11-01 — GTSAM 4.3 offline Pose2 batch reference

- Built **GTSAM 4.3.0** from the user-supplied source ZIP (SHA-256 `44a5a8da6e553f03c229ebc75973c80a6474b7f3f485a7ef230ad64bce3a5f76`) without network access. A 4 GiB development container required a low-memory compile workaround; the complete shared library ultimately linked.
- Added `slam_offline_graph`, an optional **workstation-only** GTSAM Pose2 optimizer and a synthetic square trajectory example. The first pose is constrained to fix the gauge. Sequential odometry requires explicit measured/synthetic provenance; candidates for robust loop constraints require independent evidence, sufficient hit support, overlap, rank and path consistency. Duplicate, false and correlated candidates cannot silently enter the graph.
- **Local focused test evidence:** **2/2 tests passed** using GTSAM 4.3.0, normally and with `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined`. The synthetic demonstration reduced the internal factor objective **20.9136 → 1.97086**, with the first pose fixed. This is not a real-data ATE/RPE accuracy result.
- Offline CMake accepts `GTSAM_DIR` for the installed library, or `SLAM_GTSAM_SOURCE_DIR` to explicitly build the extracted source. No third-party archive is committed, no GTSAM download occurs implicitly, and Pi known-map localization does not link GTSAM.
- The *full repository* Catch2/Ceres/GTSAM build matrix has not been run locally for this new target; the existing manual GitHub Actions workflow remains unchanged and was not dispatched. The user's previous successful workflow run predates this addition and does not validate GTSAM.

**Still outstanding:** independently measured wheel/gyro or independently justified loop measurements, provenance-qualified factor uncertainty, source-correct mapping rebuild after optimization, false-loop evaluation on public data, and complete end-to-end offline SLAM. See [offline graph build and safety contract](offline-pose-graph.md).


## 2026-11-01 — Portable dependency cache, Stage 1

Built a **11,572,733-byte** versioned development dependency archive with
installed Catch2 3.16.1, Ceres 2.2.0, GTSAM 4.3.0, Eigen 3.4 headers,
license notices, source hashes, platform metadata and per-file checksums.
The SHA-256 of the archive is
`f459867e70f25e1de5f092476dd72b3953a5e985fdd3460b15726c1a1a89838b`.

**Local evidence:** Repeated packaging from identical installed prefixes
produced byte-identical archives; a fresh extraction at a different path
compiled, linked and executed a combined Catch2/Ceres/GTSAM/Eigen smoke
test (1/1). Mutating the manifest caused verification to fail.

**Scope/limitations:** This cache is built for **Debian 13, x86-64, GCC
14.2, glibc 2.41**. It must not be used on Ubuntu 24.04 or ARM64 without
rebuilding for that ABI. The complete project CTest matrix was **not** run
from this restored bundle. The archive is prepared for user publication
as a GitHub Release asset, but the current GitHub tool connection cannot
upload binary Release assets; the asset is **not yet on GitHub**.

The existing GitHub Actions workflow remains manual-only and was not run.
See [dependency recovery documentation](dependency-cache.md).

## 2026-11-01 — graph-to-map candidate generation and rollback

Implemented optional `slam_offline_generation` (requires GTSAM 4.3). The
pipeline checks independently described odometry, mapping scans and loop
sources before optimization, reconstructs the same verified local rays from
the initial and optimized trajectories, and publishes **two immutable
FUSMAP2D-compatible map candidates**. Held-out reference hit locations do
not participate in graph optimization or either map insertion. They are
evaluated only after both map snapshots have been frozen. Candidate
selection requires an accepted loop, sufficient reserved reference data,
no excessive regression in known-cell coverage and a minimum gain in
occupied hit agreement.

The separately measured `PoseEdge` and frontend match diagnostics can be
joined via `qualified_frontend_loop`; it rejects stale map fingerprints,
unsupported match statuses and unverified relative measurements. It never
converts a Ceres map-from-laser tracking result or its LiDAR Hessian into
an independent relative pose/covariance.

**Focused local tests** using GTSAM 4.3 and project-compatible map and
matcher interfaces passed **1/1 CTest** in Release and with GCC 14
`-Wall -Wextra -Wpedantic -Werror` plus UBSan. In the synthetic test,
independent held-out endpoint agreement increased **24.1% → 87.5%**,
promoting the rebuilt candidate. A misleading but graph-consistent
loop caused agreement to **fall 73.2% → 17.0%**, so the incumbent was
retained. Tests also cover false-loop rejection, no-loop rollback,
source double-counting, unverified beam timing, stale frontend map
fingerprints, held-out nonleakage, determinism and binary roundtrip.

**Limitations:** The focused build used equivalent local implementations
of several core/map primitives rather than a fully reconstructed,
byte-identical checkout. The complete repository CTest matrix is
therefore **not yet verified** for this milestone. An AddressSanitizer
attempt aborted at GTSAM's prebuilt shared-library initialization due
to an Eigen aligned-allocation mismatch, before entering these tests;
**ASan is not claimed as passing**. The UBSan build passed. The
verified source files for the new module and tests are byte-identical to
those compiled in the focused build.

This is a **synthetic** source-guard and map-generation benchmark, not a
real-data graph or external ATE/RPE measurement. An input provenance flag
remains an assertion by the caller; the module cannot independently
authenticate it. No durable filesystem transaction/persistent generation
registry is implemented. The returned selected immutable snapshot keeps
the incumbent if candidate validation fails, and any disk publication
remains an explicit caller action. See [pipeline design](offline-map-generation.md).

## 2026-11-01 — Docker APT build/test profile

Added Ubuntu 24.04 APT-based container setup (`docker/Dockerfile.apt`,
`scripts/docker-build-test.sh`, and [.dockerignore](../.dockerignore)).
The opt-in CMake `SLAM_APT_DEPENDENCIES=ON` profile permits the
Ubuntu-supported GTSAM **4.2** and Catch2 **3.4**, versus the existing
4.3/3.16 development baseline. Ceres remains 2.2. The container
requires all three dependencies, disallows Catch2 FetchContent fallback,
and checks that the Ceres/GTSAM/Catch2/rollback tests are registered
before invoking the full CTest suite. Build output is persisted in a
Git-ignored host-mounted directory; source is mounted read-only.

**Validation limits:** Docker is not installed in the current development
container, so the actual Ubuntu container image and CTest matrix have
**not** been executed. Local tests passed shell syntax and mock-Docker
invocation checks; the relaxed minimum CMake versions resolved against
locally available Ceres/GTSAM/Catch2 installed packages (which are
*newer* than Ubuntu's APT Catch2/GTSAM). Therefore actual runtime
compatibility of the distro-version APIs remains to be confirmed using
`bash scripts/docker-build-test.sh test` on a Docker host.

No GitHub Actions workflow was modified or run. See
[Docker development instructions](docker-development.md).


## 2026-11-01 — Linux persistent map generations

Added \`slam_map_registry\`: Linux-only standalone FUSMAP2D generation
store with advisory flock, fsync-then-rename publication of immutable
map files and an atomic checksummed \`CURRENT\` pointer. The store
records active/previous IDs and fingerprints and rejects stale
compare-and-swap publishers, malformed pointer data and corrupt maps.
Startup \`recover()\` can durably fall back to the previously selected
valid map; explicit rollback never rewrites either generation.

The optional GTSAM graph-to-map layer now exposes
\`publish_approved_generation\`. It checks the selected candidate is
the **quality-gate-approved** map and that the persistent incumbent
fingerprint exactly matches the evaluation's incumbent. Rejected
candidates produce **no disk publication**. The focused integration
test persists a synthetic approved result, rejects stale re-publication
and rolls back to the original immutable map.

**Local validation:** a focused C++20/GCC14 build linked to real GTSAM
4.3.0 ran **2/2 CTests**, both in Release and with strict warnings
(\`-Wall -Wextra -Wpedantic -Werror\`) and UBSan. Tests simulate
corrupt active map, corrupt registry, orphaned/pending files, restart
recovery, stale writes, explicit rollback and two concurrent writers.

**Unverified:** complete repository CMake/CTest, Ubuntu 24.04 Docker
runtime (no Docker engine available), actual power-loss fault injection,
network filesystem semantics, and real-data ground-truth-qualified map
promotion. This is a local Linux filesystem design and its integrity
checks are not an authenticity/security mechanism. GitHub Actions were
neither run nor modified. See [persistent registry guide](map-generation-registry.md).

## 2026-11-01 — Read-only offline Web UI

Implemented a lightweight Python 3.10+ standard-library dashboard with
responsive browser UI, spatial map viewer (pan/zoom, Y-up), per-cell
occupancy and world-coordinate inspection, frame/source metadata and
map-generation status. Its server reads authoritative **FUSMAP2D-v1**
binary bundles, checks footer and snapshot fingerprints, validates
cell bounds, observed masks and log-odds, and reads the **FUSGEN1**
`CURRENT` pointer without modifying it.

The API is deliberately **read-only**. No motor controls, live robot
pose, ROS telemetry, map publication, rollback or automatic recovery
are implemented. The server defaults to localhost and requires explicit
`--allow-network` to bind broadly for Docker, with guidance to bind the
published host port to loopback. Paths, request sizes, data formats and
symlinks are guarded; note FNV integrity is **not authentication**.

**Validation:** 11/11 Python unittest checks passed locally, including
real HTTP requests, corrupt map/pointer handling, malformed paths,
symlink rejection, and refusal of state-changing HTTP methods.
The real FR079 export (391 x 539, fingerprint
`13991754243820308900`) passed the new Python parser; a Python test
fixture was independently imported by the existing C++ map inspector.
Chromium rendered the real FR079 raster with working zoom and click-to-
inspect using local API fixtures: browser HTTP navigation was blocked by
the development platform, so a full browser-over-HTTP navigation check
could not be performed, although the HTTP API was tested independently.
No Docker engine or GitHub Actions were used in validation.

The Web UI has a CTest entry `web_ui_unit` whenever Python 3.10+ is
found; the Docker APT image now includes Python and the Docker wrapper
requires that test to appear. The full repository CMake/CTest matrix
and Ubuntu Docker run remain unverified for this commit.

See [Web UI guide](web-ui.md).

## 2026-11-01 — Web UI moved to apps/demo and root Docker stages

Aligned the layout with [hybrid-astar-cpp](https://github.com/civcode/hybrid-astar-cpp):
all dashboard server, map parser and static frontend files now live in
`apps/demo/web/`, with `apps/demo/main.py` as the entrypoint. The old
root `web/` package and `scripts/web-ui.py` have been removed; the
existing Python CTest imports the moved package. A new root `Dockerfile`
provides `dependencies`, `build`, `test` and default `demo`
stages, plus `compose.yaml` for loopback-only interactive maps and
explicit opt-in tests. The small demo stage contains Python, not native
Ceres/GTSAM/Catch2; the native test stage uses Ubuntu 24.04 APT.
The old `docker/Dockerfile.apt` is replaced by the root Dockerfile.

**Local validation:** moved UI passed 11/11 Python unit/HTTP tests,
JavaScript syntax validation, CLI launch and real FR079 map/API smoke
(391×539, 57,529 known cells, original fingerprint). The Docker
engine is unavailable in this workspace; Docker image compilation and
full Ubuntu CTest remain **unverified**. GitHub Actions were not run
or modified. See [demo README](../apps/demo/README.md) and
[Docker commands](docker-development.md).

## 2026-11-01 — Self-contained synthetic web demo and native-format regression

The default `docker compose up --build demo` can now show a visibly
labeled **synthetic** workshop without FR079 input or host-side C++
dependencies. The Python startup generates a temporary FUSMAP2D v1
fixture only when the mounted directory contains no .slammap and no
CURRENT registry. The underlying viewer remains read-only and never
writes into a user's map folder. Existing map files, including corrupt
ones, and registry pointers always take precedence.

The fixture is explicitly not a measurement, calibrated metric origin,
or independent robot pose truth. The new
`demo_fixture_native` CTest cross-verifies the fixture via the real
C++ FUSMAP2D importer, while `web_ui_unit` exercises the local HTTP
server and prevents fallback from masking a damaged registry.

**Local validation:** 17/17 Python unit and CLI HTTP tests passed;
a real synthetic fixture was successfully parsed by the original
`slam_map_inspect` executable with matching metadata and fingerprint;
Node verified the updated JavaScript syntax. The complete repo-wide
CTest and actual Docker image build remain unverified because a Docker
engine is unavailable; no GitHub Actions were invoked.
See [demo instructions](../apps/demo/README.md) and
[web UI documentation](web-ui.md).
