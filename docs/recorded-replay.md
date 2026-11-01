# Interactive recorded LiDAR replay (FUSREPLAY1)

**Scope:** a read-only scan/trajectory viewer for `apps/demo`, not a running
SLAM pipeline, independently verified trajectory benchmark or hardware driver.
The browser displays estimates from the recording. It cannot infer physical
pose accuracy, calibration or loop closures. **Real Ceres diagnostics** can now be
attached by the separate native exporter described in
[Ceres replay instructions](ceres-replay.md); the original replay converter
still supplies no such data.

## Quick start, no C++ build

From the repository root:

```sh
# First generate a session from an existing, trusted CARMEN log.
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED

# Start the browser with existing map exports and recorded sessions.
python3 apps/demo/main.py --data-dir apps/demo/maps --example-if-empty
# Open http://127.0.0.1:8765/replay
```

This accepts a `.log` or `.log.gz` CARMEN source. It processes **FLASER**
records; ODOM and PARAM lines are not replayed, but each FLASER carries a
logged odometry pose. The CARMEN parser uses the same angular indexing as
`CarmenReader` (`-π/2 + π*i/N`) and treats ranges `0 < r <= 80.99 m`
as hits. Invalid, missing and out-of-range returns do not become hit
points. It preserves the full beam and hit counts, but samples points at
`--beam-stride` (default 6) for bounded rendering. **Beam timestamps are
unknown**: these points are not deskewed. No scan-matcher is invoked.

For FR079, the converter reads **3,118 FLASER scans** and produces around
**3.8 MB** of JSON at stride 6. The supplied `--frame-id` is **only a
caller-provided descriptive name**, never proof of alignment. It can match
an exported map's frame label for display, but the reference map/frame
registration for FR079 remains unverified. If map and recording frame
labels differ, the optional overlay is refused.

Without a source recording, the explicitly requested `--example-if-empty`
mode creates a **temporary synthetic virtual replay** together with the
existing sample occupancy map, without writing to the user's map folder.
Real `.slammap`, `CURRENT`, or `.slamreplay.json` files always take
precedence, even if malformed. Synthetic playback is clearly labeled.

## Docker demo

```sh
# Store .slamreplay.json files in apps/demo/maps; they are Git-ignored.
docker compose up --build demo
# Open http://127.0.0.1:8765/replay
```

The default Docker demo is Python-only and runs as an unprivileged user.
The map directory is mounted **read-only**. The unauthenticated HTTP server
is bound to the container interface for loopback-only host port mapping;
never expose its port on a public interface. The generated session must
exist *before* starting the demo, or refresh the page after conversion.
Native C++/GTSAM tests remain a separate explicit action:

```sh
docker compose run --build --rm test
```

## HTTP interface

The API serves only `.slamreplay.json` files in the selected `--replay-dir`
(default: `--data-dir`) without allowing path traversal or symlinks:

| Endpoint | Returns |
| --- | --- |
| `GET /replay` | Interactive trajectory, scan/timeline and provenance page |
| `GET /api/replays` | Available session filenames (bounded list) |
| `GET /api/replays/{name}.slamreplay.json` | Source provenance, frame count, time span and logged trajectory |
| `GET /api/replays/{name}.slamreplay.json/frame/{i}` | One frame's logged pose, odometry pose, beam/hit counts and sampled endpoints |

The page exposes play/pause, speed controls, time slider, step controls,
panning, zooming, and an **optional** occupancy-map overlay. It only
overlays a map when the declared map and session frame IDs match, and
continues to label the overlay **unverified**. All endpoints are read-only;
POST/PUT/PATCH/DELETE are denied. The session is source-labelled and
validated before API use, then cached against its inode/size/timestamps.

### Format and limits

`FUSREPLAY1` is a **versioned visualization interchange**, not an
independent SLAM measurement log or a certified sensor format. Each JSON
file contains:

```json
{
  "format": "FUSREPLAY1",
  "source": {
    "dataset": "CARMEN FLASER recording",
    "frame_id": "carmen_logged_laser_frame_UNVERIFIED",
    "pose_kind": "logged_laser_estimate_not_ground_truth",
    "timestamp_kind": "CARMEN_IPC_scan_level",
    "per_beam_time_known": false,
    "map_alignment_verified": false,
    "source_sha256": "<sha256-of-original-recording>"
  },
  "frames": [
    {
      "index": 0, "t": 251.269183,
      "pose": [-1.208780, -0.139971, 0.273777],
      "odom": [-1.170270, -0.129157, 0.273777],
      "beam_count": 360, "hit_count": 354,
      "points": [[0.0, -1.74]], "matching": null
    }
  ]
}
```

(The points/counts above are **illustrative**, not a full first frame.)
The viewer requires monotonic finite scan times, finite poses/points,
sequential indices, a valid source SHA-256 description, and explicit
non-ground-truth provenance. It refuses unknown/claimed calibrated
pose types and invented scan-matching diagnostics. The file must be at
most **32 MiB**, with at most **6,000** scans and **720** displayed points
per scan. The converter refuses source ZIPs or logs over **128 MiB**, more
than 300,000 lines, malformed FLASER syntax and bad/oversized beam data.

The source SHA-256 is a **provenance reference**, not an authenticated
integrity signature over the JSON session. It must be verified against the
actual recording separately when source identity matters. The map
viewer continues to validate FUSMAP2D checksums independently. These
FNV hashes detect accidental corruption, not adversarial tampering.

## Validation evidence

Focused local Python unit and HTTP tests (including the previous viewer
suite) passed **27/27**, plus JavaScript syntax validation and a Chromium
page interaction check using the **actual converted FR079 data** supplied
through fixture-backed browser API responses. The test changed the slider
and advanced playback across recorded timestamps. The Python HTTP server
was separately tested over the loopback interface; direct Chromium
navigation to localhost was blocked by the development platform.
The full Docker/Ubuntu C++ matrix remains unverified here. No GitHub
Actions execution was triggered.

**Next:** independently validate LiDAR/IMU calibration and recorded timing,
separate map-training queries from evaluation, and then assess Ceres
tracking against independent measurements. Native Ceres diagnostic sidecars
are now available, but they are **not** ground truth or graph factors.
