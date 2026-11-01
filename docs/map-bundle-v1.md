# FUSMAP2D v1 — authoritative offline occupancy-map bundle

**Status:** Implemented and tested locally on 2026-11-01.

The .slammap file is the **authoritative snapshot**. Its format uses little-endian fields, eight-byte magic FUSMAP2D, schema version 1, grid dimensions/resolution, map-from-grid planar origin/yaw, log-odds integration policy, occupancy revision, a deterministic content fingerprint, and descriptive source/frame/truth metadata. Each grid cell stores one observed/unknown byte plus a binary64 log-odds value, with Y increasing upward and cells ordered row-major. A footer FNV-1a checksum covers preceding bytes.

On import, the reader rejects unknown versions, invalid dimensions/configuration, nonfinite cells, unknown cells containing fake evidence, bad lengths, trailing data, checksum mismatches and snapshot fingerprint mismatches. Limits are **16 million cells, 150 MiB**, and 256 bytes per provenance label.

**Security:** FNV-1a is an accidental-corruption check, **not** a cryptographic MAC. Frame metadata and the metric-origin-verified flag are claims from the source application; they are **not independently verified** by the reader. The map owner must serialize capture against concurrent insertions.

The .pgm and .yaml files are **visualizations**, not authoritative map inputs. The PGM uses 8-bit occupancy grayscale and gray 205 for unknown, with the highest grid Y in its top row. Quantization/thresholding can cause observed near-neutral cells to appear unknown. The YAML gives the image basename, map resolution/origin and rendering thresholds. **Only .slammap preserves the known mask and log odds exactly.**

## Reproduce

With the repository configured and the FR079 recording available outside Git:

~~~sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_FR079_LOG=/path/to/fr079.log.gz
cmake --build build
ctest --test-dir build --output-on-failure
./build/slam_fr079_map_export /path/to/fr079.log.gz /tmp/fr079-training
./build/slam_map_inspect /tmp/fr079-training.slammap
~~~

Use SLAM_CATCH2_SOURCE_DIR, Eigen3_DIR, and Ceres_DIR for offline local dependencies when needed. The binary library and map inspection command **do not require Ceres, GTSAM or zlib**; the gzip FR079 export command requires zlib.

The FR079 export uses 416 training-only scans and the logged laser-frame estimates. Measured locally: 391 × 539 cells, 57,529 known, 153,220 unknown, revision 416, fingerprint 13991754243820308900. Repeated binary exports were byte-identical. The map frame is named fr079_logged_laser_frame_UNVERIFIED and metric_origin_verified is false. No independent pose truth is asserted.

The format does **not** yet include raw sensor observations, MCAP provenance, a GTSAM graph, submap generations, loop closures or independent world-frame alignment. A later version must explicitly version such extensions rather than silently changing v1.
