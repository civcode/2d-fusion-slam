# Offline map Web UI — FUSMAP2D and map generations

**Status:** Available now as a **local, read-only map console** backed by
the project's existing authoritative FUSMAP2D v1 export and FUSGEN1
persistent-generation registry. This UI does **not** implement live
LiDAR streaming, robot pose tracking, GTSAM optimization controls,
acquisition, Pi hardware control or automatic map promotion. There is
no real robot session API to connect to yet.

## Use the dashboard

Requires **Python 3.10+** only (standard library; no pip/npm, ROS, Ceres
or GTSAM needed to *view an already exported map*). From the repository
checkout:

~~~sh
# Generate a real map using the existing native FR079 exporter
mkdir -p /tmp/slam-maps
./build/slam_fr079_map_export /path/to/fr079.log.gz /tmp/slam-maps/fr079-training

# Start read-only viewer; choose a directory containing *.slammap
python3 scripts/web-ui.py --data-dir /tmp/slam-maps --port 8765
~~~

Open **http://127.0.0.1:8765/**. The UI shows the available maps on the
left, an interactive occupancy raster in the center, and
frame/source/fingerprint and selected cell information on the right.
Mouse wheel zooms at cursor; drag pans; clicking a cell displays its
grid coordinates, map-frame metric coordinates and measured occupancy
probability. Use **Fit map** to reset the camera.

For the persistent registry, simply point \`--data-dir\` to the
application-owned generation directory containing \`CURRENT\` and
\`generation-<id>.slammap\` files. The UI labels the active and
previous generations. It reports invalid \`CURRENT\` metadata and
damaged map snapshots but **does not call recover(), roll back, or
change the files**. If recovery is needed, handle it through the
existing native \`GenerationStore\` API and refresh this viewer.

To view the prebuilt FR079 map provided in the development chat,
download \`fr079-training.slammap\` into a directory and pass it to
\`--data-dir\`. FR079 map-frame alignment remains **unverified** and
logged poses are **not external trajectory ground truth**.

## Docker — no Python or C++ dependencies on the host

The Ubuntu APT development image now includes Python 3. If you already
have Docker, no host-side Python, Eigen, Ceres or GTSAM is needed:

~~~sh
docker build -f docker/Dockerfile.apt -t 2d-fusion-slam:ubuntu24.04-apt .

docker run --rm -p 127.0.0.1:8765:8765 \
  -v "$(pwd):/workspace:ro" \
  -v /tmp/slam-maps:/maps:ro \
  2d-fusion-slam:ubuntu24.04-apt \
  python3 /workspace/scripts/web-ui.py --data-dir /maps \
  --host 0.0.0.0 --allow-network
~~~

Here \`/tmp/slam-maps\` must already contain exported \`.slammap\`
files. **Do not publish port 8765 on all host interfaces** or expose
the unauthenticated viewer to the Internet. The server normally binds
only to \`127.0.0.1\`; \`--allow-network\` is required for Docker's
container-side broad bind and the Docker host-side \`-p\` command is
intentionally restricted to loopback. The displayed hostname is always
the user-facing loopback URL, even when bound inside a container.

## Read-only HTTP API

| Endpoint | Description |
| --- | --- |
| \`GET /\` | Browser dashboard |
| \`GET /api/health\` | Local service status; no robot tracking claim |
| \`GET /api/maps\` | List direct \`.slammap\` files, active/previous pointer status |
| \`GET /api/maps/<filename>.slammap\` | Verified binary metadata and occupied/unknown counts |
| \`GET /api/maps/<filename>.slammap/image.png\` | Occupancy image, highest Y at top |
| \`GET /api/maps/<filename>.slammap/cell?x=…&y=…\` | Grid/metric cell inspection |

POST, PUT, PATCH and DELETE return 405. The service never calls the
graph optimizer, the map exporter or any robot-control action.
It only reads files directly inside the configured directory, refuses
symlinks, enforces map-size limits and validates checksums and binary
snapshot fingerprints before rendering or returning cell data. Browser
resources are same-origin with a restrictive Content Security Policy.

**The binary FNV checksums detect accidental corruption, not malicious
tampering or authenticity.** Do not treat a "verified" file as proof of
physical calibration or statistical independence. The source's
\`metric_origin_verified\` flag is a claim made by the producing
application, not independently measured truth. Native occupancy data
is authoritative; exported PGM/YAML is only a lossy visualization.

## Tests and scope

~~~sh
python3 -m unittest discover -s tests/web -p 'test_*.py' -v
node --check web/static/app.js   # optional JavaScript syntax check
~~~

If Python 3.10+ is available at CMake configure time and \`BUILD_TESTING=ON\`,
the \`web_ui_unit\` CTest is registered and runs with the native suite.
The Ubuntu Docker APT image explicitly installs Python and its test
wrapper requires that CTest entry, so it cannot silently skip the UI
tests.

**Local evidence (2026-11-01):** 11/11 unittest cases passed. The
real FR079 FUSMAP2D export was decoded with its known/unknown counts,
revision and original fingerprint preserved. A synthetic map generated
by the Python test fixture was imported successfully by the existing
C++ \`slam_map_inspect\` binary. A headless Chromium DOM/canvas smoke
test against those actual FR079 API values passed with working
map selection, zoom and cell inspection, with no browser exceptions.
The platform's browser blocked loopback HTTP navigation, so that
browser test used fixture-backed fetch; the separate Python HTTP tests
exercise real network requests. The full Docker/CTests remain unrun in
this environment.

**Next:** After a real calibrated recording/pose publication interface
exists, design an authenticated live status channel; do not invent
live robot positions in this offline dashboard.
