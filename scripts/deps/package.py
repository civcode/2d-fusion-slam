#!/usr/bin/env python3
"""Package installed third-party prefixes into a portable, checksum-locked release asset.

Requires source archives for verification; no GitHub network, credentials or
GitHub Actions. A compiled bundle is valid only for its recorded platform ABI.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
LOCK = json.loads((ROOT / "deps/lock.json").read_text())


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(2 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def os_release() -> dict[str, str]:
    result = {}
    for line in Path("/etc/os-release").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value.strip('"')
    return result


def check_archives(args: argparse.Namespace) -> None:
    for name, filename in [("Catch2", args.catch2_archive),
                           ("Ceres", args.ceres_archive),
                           ("GTSAM", args.gtsam_archive)]:
        expected = LOCK["dependencies"][name]["source_archive_sha256"]
        if sha256(filename) != expected:
            raise ValueError(f"{name} source archive SHA-256 mismatch")


def ensure_prefix(path: Path, subfile: str) -> None:
    if not (path / subfile).is_file():
        raise ValueError(f"Invalid installed prefix: {path} (missing {subfile})")


def copy_prefix(source: Path, target: Path) -> None:
    shutil.copytree(source, target, symlinks=True)
    for path in target.rglob("*"):
        if path.is_symlink():
            target_path = os.readlink(path)
            if os.path.isabs(target_path) or not (path.parent / target_path).resolve().is_relative_to(target.resolve()):
                raise ValueError(f"Nonrelocatable or escaping symlink: {path}: {target_path}")


def run(args: argparse.Namespace) -> None:
    check_archives(args)
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise ValueError("The first cached bundle must be built on Linux x86_64")
    host = os_release()
    expected = LOCK["target_platform"]
    if (host.get("ID"), host.get("VERSION_ID")) != (expected["os_id"], expected["os_version_id"]):
        raise ValueError("Host OS differs from dependency lock: build a new platform package")

    ensure_prefix(args.catch2_prefix, "lib/libCatch2.a")
    ensure_prefix(args.catch2_prefix, "lib/libCatch2Main.a")
    ensure_prefix(args.ceres_prefix, "lib/libceres.a")
    ensure_prefix(args.gtsam_prefix, "lib/libgtsam.so.4.3.0")
    if not (args.eigen_include / "Eigen/Core").is_file():
        raise ValueError("Eigen header root must contain Eigen/Core")
    if not (args.eigen_include / "unsupported/Eigen").is_dir():
        raise ValueError("Eigen root must include unsupported/Eigen")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    pkgname = "slam-" + LOCK["package_label"]
    archive = output / f"{pkgname}.tar.zst"
    with tempfile.TemporaryDirectory(prefix="slam-deps-") as tmp:
        root = Path(tmp) / pkgname
        install = root / "install"
        install.mkdir(parents=True)
        copy_prefix(args.catch2_prefix.resolve(), install / "catch2")
        copy_prefix(args.ceres_prefix.resolve(), install / "ceres")
        copy_prefix(args.gtsam_prefix.resolve(), install / "gtsam")
        ei = install / "eigen3/include/eigen3"
        ei.mkdir(parents=True)
        copy_prefix(args.eigen_include / "Eigen", ei / "Eigen")
        copy_prefix(args.eigen_include / "unsupported", ei / "unsupported")
        # This minimal CMake package uses only relative paths. Ceres requires
        # a findable Eigen3::Eigen target when imported from its installed prefix.
        config = install / "eigen3/lib/cmake/Eigen3"
        config.mkdir(parents=True)
        (config / "Eigen3Config.cmake").write_text('''get_filename_component(_SLAM_EIGEN_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(Eigen3_VERSION 3.4.0)
set(Eigen3_FOUND TRUE)
set(EIGEN3_INCLUDE_DIR "${_SLAM_EIGEN_PREFIX}/include/eigen3")
set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
unset(_SLAM_EIGEN_PREFIX)
''')
        (config / "Eigen3ConfigVersion.cmake").write_text('''set(PACKAGE_VERSION "3.4.0")
if(PACKAGE_FIND_VERSION VERSION_LESS_EQUAL PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
''')
        licenses = root / "licenses"
        licenses.mkdir()
        license_files = {
            "Catch2-LICENSE.txt": args.catch2_source / "LICENSE.txt",
            "Ceres-LICENSE": args.ceres_source / "LICENSE",
            "GTSAM-LICENSE": args.gtsam_source / "LICENSE",
            "GTSAM-LICENSE.BSD": args.gtsam_source / "LICENSE.BSD",
            "Eigen-COPYING.MPL2": args.gtsam_source / "gtsam/3rdparty/Eigen/COPYING.MPL2",
        }
        for name, file in license_files.items():
            if not file.is_file():
                raise ValueError(f"License source missing: {file}")
            shutil.copy2(file, licenses / name)

        stdlib = subprocess.check_output(["c++", "-dumpfullversion"], text=True).strip()
        if int(stdlib.split(".")[0]) != expected["compiler_major"]:
            raise ValueError("Compiler major differs from dependency lock")
        libgtsam = install / "gtsam/lib/libgtsam.so.4.3.0"
        elf = subprocess.check_output(["readelf", "-d", str(libgtsam)], text=True)
        if "$ORIGIN" not in elf:
            raise ValueError("GTSAM shared library lacks relative RUNPATH ($ORIGIN)")
        manifest = {
            "schema": 1,
            "release_id": LOCK["package_label"],
            "purpose": "reusable development-only compiled dependencies, not project runtime data",
            "host": {
                "os_id": host.get("ID"), "os_version_id": host.get("VERSION_ID"),
                "architecture": platform.machine(), "compiler": f"gcc-{stdlib}",
                "glibc": "-".join(platform.libc_ver()),
                "cmake": subprocess.check_output(["cmake", "--version"], text=True).splitlines()[0],
            },
            "dependencies": LOCK["dependencies"],
            "prefixes": ["install/eigen3", "install/catch2", "install/ceres", "install/gtsam"],
            "gtsam_runpath": "$ORIGIN",
            "limitations": [
                "Linux x86_64 Debian 13 GCC 14 ABI; do not reuse automatically across platforms",
                "GTSAM was built with its bundled Eigen; Ceres used Eigen 3.4.0 headers",
                "Ceres MINIGLOG=ON may conflict with another glog in one process",
                "No runtime hardware/FR079 data or source ZIPs inside the binary package",
            ],
        }
        (root / "build-manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        shutil.copyfile(root / "build-manifest.json", output / f"{pkgname}.manifest.json")
        # Check all regular files, keeping symbolic links in the tar stream.
        regular_files = sorted(p for p in root.rglob("*") if p.is_file() and not p.is_symlink())
        (root / "SHA256SUMS").write_text("".join(
            f"{sha256(p)}  {p.relative_to(root).as_posix()}\n" for p in regular_files
        ))
        # Normalize archive metadata; stored build artifacts are already fixed.
        tar = subprocess.Popen(["tar", "--sort=name", "--format=posix", "--owner=0", "--group=0",
                                "--numeric-owner", "--mtime=@0", "--pax-option=delete=atime,delete=ctime", "-C", tmp, "-cf", "-", pkgname],
                               stdout=subprocess.PIPE)
        with archive.open("wb") as stream:
            zstd = subprocess.run(["zstd", "-q", "-T2", "-9", "-c"], stdin=tar.stdout,
                                  stdout=stream, check=True)
        tar.stdout.close()
        if tar.wait() != 0:
            raise RuntimeError("tar failed")
    (archive.with_suffix(archive.suffix + ".sha256")).write_text(
        f"{sha256(archive)}  {archive.name}\n"
    )
    (output / "SHA256SUMS").write_text(
        f"{sha256(archive)}  {archive.name}\n"
        f"{sha256(output / f'{pkgname}.manifest.json')}  {pkgname}.manifest.json\n"
    )
    print(f"Archive: {archive} ({archive.stat().st_size} bytes)")
    print(f"SHA256:  {sha256(archive)}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("catch2", "ceres", "gtsam"):
        p.add_argument(f"--{name}-prefix", type=Path, required=True)
        p.add_argument(f"--{name}-source", type=Path, required=True)
        p.add_argument(f"--{name}-archive", type=Path, required=True)
    p.add_argument("--eigen-include", type=Path, required=True,
                   help="Eigen 3.4 headers directory containing Eigen/Core")
    p.add_argument("--output", type=Path, required=True)
    run(p.parse_args())
