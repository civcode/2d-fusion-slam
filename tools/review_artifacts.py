#!/usr/bin/env python3
"""Host-aware review inventory for generated SLAM experiments.

Docker cannot infer the host side of a bind mount. Set SLAM_HOST_OUTPUT_DIR
to the host directory mounted at SLAM_CONTAINER_OUTPUT_DIR (default /output).
For ordinary host execution, absolute file paths already are host paths.
No copies, uploads, networking, or overwrite of prior manifests occur.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Iterable


def host_path(file: Path) -> str | None:
    """Return a real host path or None for an unmapped container mount."""
    path = Path(file).absolute()
    root = Path(os.environ.get("SLAM_CONTAINER_OUTPUT_DIR") or "/output").absolute()
    try:
        relative = path.relative_to(root)
    except ValueError:
        return str(path)
    host = os.environ.get("SLAM_HOST_OUTPUT_DIR")
    if not host:
        return None
    return str((Path(host).expanduser().absolute() / relative))


def review_files(manifest: Path, experiment: str,
                 files: Iterable[tuple[str, Path, bool]]) -> dict:
    """Write an exclusive SLAM_REVIEW1 JSON and print file paths for review.

    files: (role, actual file path, upload_to_chat) triples.
    The inventory is metadata-only; no large binary file is duplicated.
    """
    manifest = Path(manifest)
    if manifest.exists():
        raise FileExistsError(f"review manifest already exists: {manifest}")
    entries = []
    for role, file, upload in files:
        file = Path(file)
        if not file.is_file():
            raise FileNotFoundError(f"missing {role} review artifact: {file}")
        entries.append({
            "role": role,
            "path": str(file.absolute()),
            "host_path": host_path(file),
            "upload": bool(upload),
        })
    content = {"format": "SLAM_REVIEW1", "experiment": experiment,
               "files": entries}
    manifest.parent.mkdir(parents=True, exist_ok=True)
    with manifest.open("x", encoding="utf-8") as stream:
        json.dump(content, stream, indent=2)
        stream.write("\n")
    print("\nReview artifacts (upload to ChatGPT):", flush=True)
    for entry in entries:
        if entry["upload"]:
            resolved = entry["host_path"] or entry["path"]
            note = " [container path; host mount not provided]" if not entry["host_path"] else ""
            print(f"  {entry['role']}: {resolved}{note}", flush=True)
    print("Other outputs (optional maps/previews):", flush=True)
    for entry in entries:
        if not entry["upload"]:
            print(f"  {entry['role']}: {entry['host_path'] or entry['path']}", flush=True)
    print(f"Review manifest: {host_path(manifest) or manifest.absolute()}", flush=True)
    if host_path(manifest) is None:
        print('To print host paths in Docker, pass '
              '-e SLAM_HOST_OUTPUT_DIR="$PWD/<mounted-host-directory>" '
              'with -v "$PWD/<mounted-host-directory>:/output".',
              flush=True)
    return content


def cli() -> None:
    """Inventory existing FR079 or synthetic-fusion results without rerunning."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="kind", required=True)
    fr079 = sub.add_parser("fr079", help="index an existing flat FR079 run")
    fr079.add_argument("prefix", type=Path,
                       help="host path without .report.json/.slammap suffix")
    fr079.add_argument("--manifest", type=Path,
                       help="optional alternate output path (e.g. reports/)")
    fusion = sub.add_parser("fusion", help="index an existing native fusion run")
    fusion.add_argument("directory", type=Path)
    fusion.add_argument("--manifest", type=Path)
    args = parser.parse_args()
    if args.kind == "fr079":
        prefix = str(args.prefix)
        manifest = args.manifest or Path(prefix + ".review.json")
        files = [
            ("report", Path(prefix + ".report.json"), True),
            ("trajectory", Path(prefix + ".trajectory.json"), True),
            ("matching_decisions", Path(prefix + ".matches.json"), True),
            ("replay", Path(prefix + ".slamreplay.json"), True),
            ("occupancy_map", Path(prefix + ".slammap"), False),
            ("map_preview", Path(prefix + ".pgm"), False),
            ("map_metadata", Path(prefix + ".yaml"), False),
        ]
        optional = Path(prefix + ".capture.csv")
        if optional.is_file():
            files.append(("capture_diagnostics", optional, True))
        experiment = "fr079_offline"
    else:
        folder = args.directory
        manifest = args.manifest or folder / "review-manifest.json"
        files = [
            ("report", folder / "fusion_report.json", True),
            ("trajectories", folder / "trajectories.json", True),
            ("matching_audit", folder / "lidar_only_match_audit.json", True),
            ("replay", folder / "fusion_replay.json", True),
            ("performance", folder / "fusion_profile.json", True),
            ("fused_map", folder / "fused_scan_matched.slammap", False),
        ]
        for role, name, upload in (
            ("optimized_map", "graph_optimized.slammap", False),
            ("mapping_ablation", "mapping_ablation.json", True),
        ):
            if (folder/name).is_file():
                files.append((role, folder/name, upload))
        experiment = "fusion_offline"
    if manifest.exists():
        print("Existing review manifest:", host_path(manifest) or manifest.absolute())
        for role, file, upload in files:
            if upload:
                print(f"  {role}: {host_path(file) or file.absolute()}")
    else:
        review_files(manifest, experiment, files)


if __name__ == "__main__":
    cli()
