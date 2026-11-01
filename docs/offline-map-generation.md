# Offline map-generation candidate and promotion gate

**Status (2026-11-01):** a tested synthetic graph-to-map orchestration
layer that never publishes graph-refined maps merely because GTSAM lowered
its factor objective. It is an **offline workstation** component and
depends on the optional GTSAM 4.3 graph target. It is not enabled in the
Raspberry Pi known-map localizer.

## Module and safety boundary

The public C++20 interface in
`include/slam/offline/map_generation.hpp` provides:

- `qualified_frontend_loop`: combine independently measured relative
  `PoseEdge` with a frontend's scan-match **quality diagnostics** only
  if the scan result was accepted, the map fingerprint matches, the
  measurement has independently verified source/frame metadata and
  uncertainty values are finite. No pose or posterior covariance is
  manufactured from a geometry-only Ceres tracking result.
- `VerifiedKeyframe`: local rays in a fixed base frame, explicit node
  index, acquisition/deskew verification and factor provenance. Unknown
  or invalid ranges never create occupancy evidence.
- `HeldoutReference`: independently sourced, fixed map-frame hit
  endpoints reserved *only* for evaluation. CARMEN FR079 logged poses
  and the archived map are **not** trustworthy independent truth.
- `evaluate_map_generation`: first validate all source IDs and
  timing/frame claims; run the existing GTSAM Pose2 batch graph;
  reconstruct an incumbent map from initial poses and a candidate map
  from optimized poses using **the same training rays and grid**;
  deep-copy each to an immutable `MapSnapshot` with distinct content
  fingerprints; evaluate reserved independent endpoints without using
  them to make either map; select the candidate only if it passes
  explicit quality and source gates.

The module refuses odometry, loops, mapping observations or held-out
reference data that share source identifiers in unsupported ways. It
also refuses unverifiable frame/timing inputs and non-finite rays.
These conditions are **caller-provided assertions**; a source ID is not
cryptographic proof of statistical independence. Correlated samples
with different IDs remain a risk to resolve at data-preparation time.

## Promotion decision

A candidate is selected only if all gates pass:

1. At least one graph loop factor was accepted after the graph's
   geometric, rank, path and provenance checks.
2. Enough withheld, externally referenced hit endpoints were provided.
3. Known-cell center coverage does not regress more than the specified
   tolerance.
4. Fraction of reserved endpoints near a known occupied cell improves
   by at least `minimum_hit_fraction_gain`. Unknown, outside
   and unobserved cells do not count as occupied matches.

Otherwise `authoritative` remains the immutable incumbent snapshot.
The `candidate` bundle is still returned for diagnostics and can
be exported separately with the existing FUSMAP2D v1 map writer.
No existing authoritative map is mutated, and this module does **not**
atomically publish files or implement a crash-consistent persistent
map-generation registry. This first stage is an in-memory selection
gate; persistent publication/rollback must be added separately.

## Reproduce synthetic test

Install/restore GTSAM 4.3 and the other dependencies as described in
[dependency recovery](dependency-cache.md). A compatible setup:

~~~sh
cmake -S . -B build-graph -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_GTSAM=ON \
  -DSLAM_REQUIRE_GTSAM=ON \
  -DCMAKE_PREFIX_PATH='/path/to/bundle/install/eigen3;/path/to/bundle/install/catch2;/path/to/bundle/install/ceres;/path/to/bundle/install/gtsam'
cmake --build build-graph --parallel 2
ctest --test-dir build-graph --output-on-failure -R 'offline_map_promotion|pose_graph|map_bundle'
~~~

The standalone regression uses a synthetic five-keyframe square, 448
mapped hit rays from four keyframes and 112 separate held-out reference
hits at 0.1 m resolution. A source-verified synthetic loop raises
hit agreement **24.1% → 87.5%**; a biased but apparently plausible
loop reduces it **73.2% → 17.0%** and is rejected by the map-quality
gate. Tests also assert stable map fingerprints for identical inputs,
held-out data never influencing map fingerprints, rejection of duplicate
sources and unverified timing, and a candidate FUSMAP2D v1 roundtrip.

**Local evidence:** focused CMake/GCC14/GTSAM 4.3 build 1/1 CTest
passed in Release and strict warnings+UBSan. The focused compilation
did not use a completely byte-identical repository checkout for core
and mapping implementation files. Full repo CI validation remains
pending. Prebuilt GTSAM with mismatched Eigen alignment fails ASan
during library initialization; do not claim sanitizer completeness.

## Durable publication

The Linux-only [map-generation registry](map-generation-registry.md)
now stores immutable FUSMAP2D generations behind a checksummed,
atomically replaced active/previous pointer. The
`publish_approved_generation` bridge will persist the selected
candidate **only** when the same incumbent fingerprint is still active
on disk. Rejected results cannot alter the authoritative map; explicit
rollback and restart recovery are covered by focused tests.
This does not create independent ground truth or automatically
authenticate source provenance.

## Next gate

Provide independent factor evidence (calibrated gyro/wheel sources and
measurement covariances, verified loop relative transforms and source
lineage), then evaluate a **separate real traversal** before allowing
real-data map promotion. FR079 logged laser poses and the reference
map are not sufficient. Subsequently implement persistent generation
metadata, transactional file publication and crash-safe rollback.
