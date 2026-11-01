# Persistent offline dependency cache — Stage 1

**Purpose:** Recover the already-built C++ dependencies after a new ChatGPT chat
or workspace reset. Keep scripts, source pins and integrity hashes in Git;
store the compiled archive in a GitHub Release and optionally on a local PC.
**This is a host-ABI-specific installation cache, not a full container.**

## Verified binary package

| Property | Value |
| --- | --- |
| Proposed Release tag | `deps-debian13-x86_64-gcc14-v1` |
| Archive | `slam-deps-debian13-x86_64-gcc14-v1.tar.zst` |
| Compressed size | **11,572,733 bytes** |
| SHA-256 | `f459867e70f25e1de5f092476dd72b3953a5e985fdd3460b15726c1a1a89838b` |
| Compiled environment | **Debian 13, x86-64, GCC 14.2, glibc 2.41** |
| Libraries | Catch2 **3.16.1**, Ceres **2.2.0**, GTSAM **4.3.0**, Eigen3 **3.4.0** headers |
| Layout | Installed headers, static Catch2/Ceres, shared GTSAM + bundled Metis/Cephes, CMake package exports, licenses |
| Excluded | Full build trees, source ZIPs, raw FR079, Raspberry Pi recordings and SLAM maps |

Relevant repository files: [pinned versions/source hashes](../deps/lock.json),
[observed build manifest](../deps/deps-debian13-x86_64-gcc14-v1.manifest.json),
[expected archive checksum](../deps/deps-debian13-x86_64-gcc14-v1.tar.zst.sha256),
[package utility](../scripts/deps/package.py),
[restore](../scripts/deps/restore.sh),
[verify](../scripts/deps/verify.py),
[rebuild fallback](../scripts/deps/build-from-sources.sh),
[smoke test](../tests/deps/dependency_smoke.cpp).

**Published and verified:** The [GitHub dependency Release](https://github.com/civcode/2d-fusion-slam/releases/tag/deps-debian13-x86_64-gcc14-v1)
contains the compiled archive, its `.sha256` sidecar, the build
manifest and `SHA256SUMS`. GitHub's SHA-256 digests match the committed
checksums for all four uploaded assets. The release tag points to
the corresponding Stage 1 dependency-cache commit.

For future *different* toolchain or ABI builds, create a new release
tag and attach the new package/checksums; do not replace the published
v1 files. Keeping an additional local PC backup is optional.

## Restoring on a compatible environment

After obtaining the archive and its `.sha256` sidecar, place them in the
same directory. From a checkout of this repository:

```sh
bash scripts/deps/restore.sh \
  /path/to/slam-deps-debian13-x86_64-gcc14-v1.tar.zst \
  /path/to/recovered-dependencies
```

Restore refuses overwrites and incompatible OS, CPU, GCC-major or glibc.
It checks the outer archive SHA-256, member paths, inner per-file hashes,
symlink containment, and then builds and runs a **fresh CMake/CTest smoke
executable linking all four packages**. It prints the final installation
location and suggested `CMAKE_PREFIX_PATH`.

You may rerun validation via:

```sh
bash scripts/deps/smoke.sh \
  /path/to/recovered-dependencies/slam-deps-debian13-x86_64-gcc14-v1
```

Configure the SLAM project using the four restored installation prefixes:

```sh
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_CATCH2=ON -DSLAM_ENABLE_CERES=ON \
  -DSLAM_REQUIRE_CERES=ON -DSLAM_ENABLE_GTSAM=ON \
  -DSLAM_REQUIRE_GTSAM=ON \
  -DCMAKE_PREFIX_PATH='/path/to/bundle/install/eigen3;/path/to/bundle/install/catch2;/path/to/bundle/install/ceres;/path/to/bundle/install/gtsam'
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

The project's CMake now looks for an installed Catch2 v3.16 package **before**
its optional FetchContent download. Ceres and GTSAM are discovered by their
installed CMake package configs.

**Compatibility:** Ubuntu 24.04, ARM64 Raspberry Pi, different GCC major or
glibc, and other distros must use a *new binary build*; never silently run
this Debian 13 asset there. Even a matching host check cannot prove all
external ABI details. The GTSAM shared library uses a relative `$ORIGIN`
runpath for the included Metis/Cephes libraries. Ceres was built with
`MINIGLOG=ON` and may conflict if another logging implementation is linked.

## Rebuilding when the environment changes

Keep or retrieve the original pinned sources, checking their SHA-256 values
against [deps/lock.json](../deps/lock.json), and extract the full source
directories. The fallback build script requires those three directories
plus **Eigen 3.4** headers:

```sh
bash scripts/deps/build-from-sources.sh \
  /path/to/work \
  /path/to/Catch2-3.16.1 \
  /path/to/ceres-solver-2.2.0 \
  /path/to/gtsam-4.3.0 \
  /path/to/eigen3
```

This installs four prefixes under `work/install/`. Its GTSAM step uses
low-memory compiler settings and one build job; on an adequately
provisioned workstation, tune those settings after measuring.

Package installed outputs, verifying all original ZIPs:

```sh
python3 scripts/deps/package.py \
  --catch2-prefix /path/to/work/install/catch2 \
  --catch2-source /path/to/Catch2-3.16.1 \
  --catch2-archive /path/to/Catch2-3.16.1.zip \
  --ceres-prefix /path/to/work/install/ceres \
  --ceres-source /path/to/ceres-solver-2.2.0 \
  --ceres-archive /path/to/ceres-solver-2.2.0.zip \
  --gtsam-prefix /path/to/work/install/gtsam \
  --gtsam-source /path/to/gtsam-4.3.0 \
  --gtsam-archive /path/to/gtsam-4.3.0.zip \
  --eigen-include /path/to/eigen3 \
  --output /path/to/release-assets
```

This exact package script checks source archive SHA-256, library existence,
GTSAM runpath and host ABI. It creates `.tar.zst`, a `.sha256` sidecar,
a standalone `.manifest.json` and release-level `SHA256SUMS`. Tar metadata
is normalized; two runs over **identical installed files** produce the
same archive bytes. Different compiler flags/builds need a new tag and
checksum, not a claim of bit-for-bit reproduction.

## Validation record and limits

- The current archive was generated from the actual uploaded source builds.
- Repeated packaging produced the **identical** 11,572,733-byte archive.
- Restore into a different directory verified each file, compiled/linked
  Ceres, Catch2, GTSAM and Eigen, then passed **1/1 CTest**.
- Changing the extracted manifest was correctly rejected by verification.
- The complete 2D Fusion SLAM CTest matrix has **not** been rerun against
  this restored archive; the all-from-source rebuild script was syntax-checked
  rather than fully reexecuted. The existing manual GitHub Actions workflow
  was **not** run or modified.
- Third-party licenses are included. Review all applicable transitive
  notices and redistribution requirements before making a public Release.

This Stage 1 archive is **not a portable Docker image**. A future GHCR
development image could preserve the entire Linux userland for environments
with container support, but it is separate from this milestone.
