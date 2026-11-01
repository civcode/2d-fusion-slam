# Fusion SLAM map explorer demo

The application is a **local read-only viewer** for existing FUSMAP2D
`.slammap` files and the active/previous `FUSGEN1` generation registry.
It does **not** stream robot poses or control SLAM.

From the repository root, use Python 3.10+:

```sh
python3 apps/demo/main.py --data-dir apps/demo/maps
```

Open http://127.0.0.1:8765. To use a different map folder, pass its
path to `--data-dir`.

Like `civcode/hybrid-astar-cpp`, this repository also exposes a root
Dockerfile and Compose demo:

```sh
docker compose up --build demo
```

The Compose port is bound to **host loopback only**. Files inside
`apps/demo/maps` are mounted read-only at `/maps`. The container listens
on all of its own interfaces solely so Docker's loopback port forwarding
works; **do not publish the HTTP port publicly**, because the viewer
has no authentication. To run native SLAM tests manually:

```sh
docker compose run --build --rm test
```

For map format, CLI options and API details see
[the Web UI guide](../../docs/web-ui.md).

## Interactive recorded LiDAR replay

Open **http://127.0.0.1:8765/replay** after starting the demo. With
`--example-if-empty` and an empty map folder, a **disposable synthetic
session** is provided alongside the virtual workshop map. This is not
recorded laser data.

Convert an existing CARMEN/FR079 FLASER recording into the bounded,
read-only replay format:

```sh
python3 apps/demo/convert_carmen.py /path/to/fr079.log.gz \
  apps/demo/maps/fr079.slamreplay.json --beam-stride 6 \
  --frame-id fr079_logged_laser_frame_UNVERIFIED
python3 apps/demo/main.py --data-dir apps/demo/maps --example-if-empty
```

The converter preserves CARMEN scan times and logged laser/odometry
poses, but **does not run Ceres or infer scan-match diagnostics**.
FR079 pose estimates are not verified ground truth. Map overlays require
matching declared frame IDs, and their alignment is still *unverified*.
Scan points are downsampled for display while valid-hit/beam counts
remain attached to every frame. The UI supports play/pause, variable
speed, seek, stepping, zoom and pan. Existing user files take precedence
over examples. See [recorded replay format and limits](../../docs/recorded-replay.md).

## Offline Ceres scan-matching diagnostics

With a **separately frozen** `.slammap` in this directory and the original
CARMEN recording, build the native `slam_ceres_replay_export` executable,
then generate a `FUSMATCH1` sidecar with `python3 apps/demo/match_ceres.py`.
The replay will display genuine Ceres overlap, robust cost, rejected matches
and optional orange accepted-pose overlays. The browser never runs Ceres.
See [Ceres replay instructions](../../docs/ceres-replay.md).
