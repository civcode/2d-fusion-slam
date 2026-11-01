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
