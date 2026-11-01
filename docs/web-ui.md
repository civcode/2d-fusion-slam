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
python3 apps/demo/main.py --data-dir /tmp/slam-maps --port 8765
# Or, if no map export is available yet, show a labeled synthetic example:
python3 apps/demo/main.py --data-dir apps/demo/maps --example-if-empty
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

## Application layout and Docker — no host toolchain

The Web UI lives entirely under [apps/demo](../apps/demo/README.md):
`apps/demo/main.py` launches the Python standard-library HTTP
server in `apps/demo/web/`, and its HTML/CSS/JS lives in
`apps/demo/web/static/`. The prior top-level `web/` package and
`scripts/web-ui.py` launcher have been removed. The native C++ SLAM
libraries are not needed to display an exported map.

Like [hybrid-astar-cpp](https://github.com/civcode/hybrid-astar-cpp),
the root [Dockerfile](../Dockerfile) has distinct stages for the Python
demo and native build/tests, and [compose.yaml](../compose.yaml) starts
the demo or runs tests explicitly.

```sh
# No map files necessary: the empty-directory demo shows a synthetic map.
docker compose up --build demo
# Open http://127.0.0.1:8765

# Opt-in full native C++ and Python test run (separate APT-backed image)
docker compose run --build --rm test
```

The demo stage installs **Python only**, not Ceres/GTSAM/Catch2, because
it is a read-only map viewer. Its `--example-if-empty` option supplies a
deterministic **synthetic** workshop when the read-only mounted map
directory contains neither a .slammap file nor a CURRENT registry.
The map is created only in an isolated temporary directory. Any real
map or registry takes precedence; malformed data stays visible as an
error rather than silently switching to the sample.
A prominent browser notice labels synthetic data as **not measured
LiDAR** and not verified metric world-frame alignment. The test target builds against Ubuntu 24.04
APT packages and requires the native solvers. The map files are mounted
read-only from `./apps/demo/maps` to `/maps`, and
are ignored by Git and excluded from the Docker build context.

Without Compose, using an existing exported map directory:

```sh
docker build --target demo -t fusion-slam:demo .
docker run --rm -p 127.0.0.1:8765:8765 \
  -v /path/to/slam-maps:/maps:ro \
  fusion-slam:demo
```

The Docker service binds all **container** interfaces to permit port
forwarding, but the **host** port is explicitly loopback-only.
The server has no authentication or TLS; never expose it on a public
interface. Bare-metal Python launches bind to `127.0.0.1` by default,
and `--allow-network` is still needed for an explicit broad bind.

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
node --check apps/demo/web/static/app.js   # optional JavaScript syntax check
~~~

If Python 3.10+ is available at CMake configure time and \`BUILD_TESTING=ON\`,
the \`web_ui_unit\` CTest is registered and runs with the native suite.
The root Dockerfile's native `test` target includes Python and its CTest
entry; the minimal `demo` target needs only Python to serve maps.

**Initial local evidence (2026-11-01):** 11/11 unittest cases passed. The
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

### Synthetic startup fixture and cross-language verification

`apps/demo/web/example.py` generates a **schematic** 120×90 cell
FUSMAP2D v1 workshop with occupied, free and unknown cells; it does not
model an LD19 acquisition or reconstruct a trajectory. Its metadata
explicitly says `truth_class=synthetic_exact` and
`metric_origin_verified=false`. The standalone viewer remains
read-only; only the startup option creates a disposable temporary
map, never a `generation-*.slammap` or user-data `CURRENT` pointer.
The regenerated binary is independently parsed by the C++ native
`slam_map_inspect` executable using the
`demo_fixture_native` CTest. The existing `web_ui_unit` CTest runs
the actual server CLI HTTP smoke, no-leakage checks, corrupted-registry
precedence and frontend-safe-browsing assertions.

**Extended local validation:** 17/17 Python tests, the real CLI over
loopback HTTP, Node JavaScript syntax and C++/Python native-format
parity passed. The actual Ubuntu Docker build and complete repository
CTest matrix remain unverified in this workspace.
