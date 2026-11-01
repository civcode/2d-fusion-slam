# Offline map Web UI — FUSMAP2D and map generations

**Recorded LiDAR replay is implemented:** open `/replay` from the
map explorer. Convert FLASER/FR079 logs with
`python3 apps/demo/convert_carmen.py INPUT.log.gz OUTPUT.slamreplay.json`.
The browser displays a timestamped logged-pose trajectory, sampled scan
hits, valid beam counts, pause/play/seek/speed and an optional
**unverified** same-frame map overlay. The [replay format and usage
guide](recorded-replay.md) documents the bounds, security and current
limitations. No Ceres scan matching is executed and there is no
independent truth or live robot source.

**Status:** Available now as a **local, read-only map console** backed by
the project's existing authoritative FUSMAP2D v1 export and FUSGEN1
persistent-generation registry. This UI does **not** implement live
LiDAR streaming, robot pose tracking, GTSAM optimization controls,
acquisition, Pi hardware control or automatic map promotion. There is
no real robot session API to connect to yet.

## Use the dashboard

The **default demo server is now C++20**, using native FUSMAP2D parsing,
zlib PNG rendering and OpenSSL SHA-256 sidecar provenance checks. It does
not link Ceres, GTSAM or Python to serve exported maps. From the repository
checkout (after building `fusion_slam_demo` with the commands below):

~~~sh
# Generate a real map using the existing native FR079 exporter
mkdir -p /tmp/slam-maps
./build/slam_fr079_map_export /path/to/fr079.log.gz /tmp/slam-maps/fr079-training

# Start read-only viewer; choose a directory containing *.slammap
./build-demo/fusion_slam_demo --data-dir /tmp/slam-maps --port 8765
# Or, if no map export is available yet, show a labeled synthetic example:
./build-demo/fusion_slam_demo --data-dir apps/demo/maps --example-if-empty
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

The Web UI lives under [apps/demo](../apps/demo/README.md):
`apps/demo/native/main.cpp` and `data.cpp` provide the native C++20
read-only HTTP API; `apps/demo/web/static/` retains the **same**
HTML/CSS/JavaScript browser application. The server uses the native
`slam::mapping::read_map_bundle` checksum/fingerprint verification.
The old Python `apps/demo/main.py` remains available as a temporary
fallback and for parity comparisons; it is no longer the Docker runtime.

Like [hybrid-astar-cpp](https://github.com/civcode/hybrid-astar-cpp),
the root [Dockerfile](../Dockerfile) has a **lightweight native C++ demo
build**, a Python-free native runtime and separate heavy solver build/tests, and [compose.yaml](../compose.yaml) starts
the demo or runs tests explicitly.

```sh
# No map files necessary: the empty-directory demo shows a synthetic map.
docker compose up --build demo
# Open http://127.0.0.1:8765

# Opt-in full native C++ and Python test run (separate APT-backed image)
docker compose run --build --rm test
```

The demo runtime installs **neither Python nor Ceres/GTSAM/Catch2**.
Its separate native builder needs Eigen3, zlib and OpenSSL development
headers. The built C++ binary reads a read-only mount and serves fixed
browser assets; Python is retained only for offline conversion and testing. Its `--example-if-empty` option supplies a
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
interface. Bare-metal native launches bind to `127.0.0.1` by default,
and `--allow-network` is still required for explicit broad binding.

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
The root Dockerfile's opt-in `test` target includes Python as a test
driver. The `demo` runtime is now compiled native C++, and
`native_web_api` verifies C++ HTTP endpoints, binary-map metadata,
PNG, replay, SHA-bound Ceres sidecars and rejection of write requests.
Python is **not** required on the demo host/container.

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


## Native C++20 server build and migration status

~~~sh
cmake -S . -B build-demo -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CERES=OFF -DSLAM_ENABLE_GTSAM=OFF \
  -DSLAM_ENABLE_CATCH2=OFF \
  -DSLAM_BUILD_NATIVE_DEMO=ON -DSLAM_REQUIRE_NATIVE_DEMO=ON
cmake --build build-demo --target fusion_slam_demo --parallel 2
ctest --test-dir build-demo -R '^native_web_api$' --output-on-failure
./build-demo/fusion_slam_demo \
  --data-dir apps/demo/maps --example-if-empty
~~~

The native data module enforces bounded, regular, no-symlink reads,
rejects duplicate JSON keys and invalid/non-finite observation fields,
checks FUSGEN1 pointers and resolved map fingerprints and verifies
FUSMATCH1 against the replay's exact SHA-256 bytes, source identity,
map revision/fingerprint/frame and conservative acceptance contract.
It never runs SLAM, changes maps or exposes control/mutation endpoints.
The native JSON parser retains 64-bit map fingerprints as strings rather
than converting them through floating point.

The C++ path is **committed but full native CMake/CTest and Docker
execution are not verified in the current environment** because a full
local repository checkout is unavailable. Treat the Python server as
the operational fallback until the native CTest and replay parity checks
pass on a full checkout; no automatic GitHub Actions were run. The
Docker demo has been switched to the native binary, so validate that
new image before replacing a working deployed demo.

The server remains a deliberately small *single-threaded developer
offline viewer* with bounded request sizes and per-client timeouts,
similar to the native model in hybrid-astar-cpp. It is not an
internet-facing HTTP service or a real-time SLAM telemetry interface.
Replay and binary map files are reparsed on demand, which is safe but
not optimized for high-frequency streaming.


## Manual full-checkout verification

Run `bash scripts/verify-native-demo.sh` from the repository root. It
configures a **local, disconnected CMake build**, builds the C++ viewer,
and runs the native JSON/HTTP API tests, native-versus-Python FUSMAP2D
fixture parity, and the original Python Web UI unit tests.

The JSON parser regression was compiled separately in the current
container using GCC 14, `-Wall -Wextra -Werror -Wpedantic` and
UndefinedBehaviorSanitizer; all parser tests passed. This is focused
evidence, **not** a full-repository build or validation of the
native HTTP server / Docker runtime.

