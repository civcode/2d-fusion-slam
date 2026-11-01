#!/usr/bin/env python3
"""Verify an extracted dependency bundle, its contents and host compatibility."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


def check(root: Path, require_platform: bool = True) -> dict:
    root = root.resolve(strict=True)
    info = json.loads((root / "build-manifest.json").read_text())
    if info.get("schema") != 1 or not info.get("release_id", "").startswith("deps-"):
        raise ValueError("Unsupported dependency manifest")
    required = ["install/eigen3/include/eigen3/Eigen/Core",
                "install/catch2/lib/libCatch2.a", "install/catch2/lib/libCatch2Main.a",
                "install/ceres/lib/libceres.a", "install/gtsam/lib/libgtsam.so.4.3.0"]
    for p in required:
        if not (root / p).is_file():
            raise ValueError(f"Missing dependency: {p}")
    listed = set()
    for row in (root / "SHA256SUMS").read_text().splitlines():
        if len(row) < 67 or row[64:66] != "  ":
            raise ValueError("Invalid checksum listing")
        digest, relative = row[:64], row[66:]
        if not relative or relative in listed or ".." in Path(relative).parts:
            raise ValueError(f"Unsafe or duplicated checksum entry: {relative}")
        listed.add(relative)
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Missing or wrong-type payload: {relative}")
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(2 * 1024 * 1024), b""):
                h.update(chunk)
        if h.hexdigest() != digest:
            raise ValueError(f"Tampered dependency payload: {relative}")
    actual_files = {p.relative_to(root).as_posix() for p in root.rglob("*")
                    if p.is_file() and not p.is_symlink() and p.name != "SHA256SUMS"}
    if listed != actual_files:
        raise ValueError(f"Checksum inventory mismatch: missing={len(actual_files-listed)} extra={len(listed-actual_files)}")
    for path in root.rglob("*"):
        if path.is_symlink():
            target = os.readlink(path)
            if os.path.isabs(target) or not (path.parent / target).resolve().is_relative_to(root):
                raise ValueError(f"Untrusted symlink: {path}")
    if require_platform:
        osinfo = {}
        for line in Path("/etc/os-release").read_text().splitlines():
            if "=" in line:
                key, val = line.split("=", 1)
                osinfo[key] = val.strip('"')
        host = info["host"]
        compiler_major = int(subprocess.check_output(["c++", "-dumpfullversion"],text=True).split(".")[0])
        if (host["os_id"], host["os_version_id"], host["architecture"], int(host["compiler"].split("-")[1].split(".")[0])) != \
           (osinfo.get("ID"), osinfo.get("VERSION_ID"), platform.machine(), compiler_major):
            raise ValueError("Incompatible system OS/architecture/compiler; rebuild from pinned source")
        if host["glibc"] != "-".join(platform.libc_ver()):
            raise ValueError("glibc mismatch; prebuilt binary not verified for this environment")
    return info


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--inspect-only", action="store_true", help="Verify payload; do not certify host ABI")
    args = parser.parse_args()
    metadata = check(args.bundle, not args.inspect_only)
    print(f"Bundle integrity PASS: {metadata['release_id']} ({metadata['host']})")
