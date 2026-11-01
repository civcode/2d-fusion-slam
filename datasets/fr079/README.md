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
| `fr079-maps.tgz` | Supplied CARMEN/Bee reference maps and GIFs at 3, 5, 10 cm, **not yet ingested by the map builder** |
| `map05cm-path.gif` | Supplied illustration of map and a route; not machine-readable pose ground truth |

## Reproduce real-data tests

~~~sh
cmake -S . -B build -G Ninja \
  -DBUILD_TESTING=ON -DSLAM_ENABLE_CATCH2=ON \
  -DSLAM_CATCH2_SOURCE_DIR=/path/to/extracted/Catch2-3.16.1 \
  -DSLAM_FR079_LOG=/path/to/fr079.log.gz \
  -DSLAM_FR079_SM_LOG=/path/to/fr079-sm.log.gz
cmake --build build
ctest --test-dir build --output-on-failure
./build/slam_carmen_inspect /path/to/fr079.log.gz 3118
~~~

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
- **No map import yet.** The 5 cm reference GIF and archived CARMEN/Bee
  map files are available as qualitative reference assets, but decoding,
  frame registration and metric comparison are future tasks.
- **No replay/SLAM benchmark claimed.** Full file parsing is only the
  first public-dataset integration test. The next experiments must hold
  out scans and report pose/mapping error where an independently verified
  truth source exists.

The parser library is plain C++20; only the optional gzip CLI uses zlib.
The input files and compressed third-party sources must not be pushed
to GitHub just to make CTest pass.
