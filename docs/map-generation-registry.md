# Durable offline map-generation registry (Linux)

**Status:** tested local filesystem persistence for authoritative FUSMAP2D
map generations. This is not a live localization service, multi-host store,
authentication boundary, real-world loop validator or replacement for
independently measured trajectory evidence.

## Data layout and crash-consistent selection

Given an application-owned directory (not writable by untrusted users):

~~~text
map-generations/
  .lock                          advisory flock for cooperating processes
  CURRENT                        versioned, checksummed active/previous refs
  generation-1.slammap           complete immutable FUSMAP2D map bundle
  generation-2.slammap           next complete immutable map bundle
  .pending-*                     ignored incomplete writes from crashes
~~~

The current map's authority comes **only** from the valid \`CURRENT\`
pointer. File names and creation order are never used to guess the
active map. The pointer contains a versioned active ID and fingerprint,
plus at most one previous ID and fingerprint. The pointer checksum and
FUSMAP2D integrity/fingerprint checks detect accidental damage; the
existing FNV checksums **are not cryptographic authenticity checks**.

The Linux writer takes an advisory \`flock\` on \`.lock\`, writes a
uniquely named temporary file, calls \`fsync\`, atomically renames the
complete map into place, and syncs the parent directory. Only then does
it atomically replace \`CURRENT\` and sync the directory again. Once a
generation is published, its file is not modified or overwritten.
Map generation IDs are monotonic relative to all existing generation
file names, including orphaned files from interrupted transactions.

A writer must explicitly supply the incumbent's fingerprint when
publishing a new generation. Concurrent or stale writers cannot advance
the same incumbent twice. An empty store is bootstrapped with
\`std::nullopt\`; later promotions require the exact active fingerprint.

## Startup, recovery and rollback

Call \`GenerationStore::recover()\` before using an existing store:

- If the active FUSMAP2D bundle validates, it remains selected.
- If the active bundle is damaged or unavailable but its recorded
  previous bundle validates, \`recover()\` *durably* selects the previous
  bundle and clears the previous pointer.
- If \`CURRENT\` is malformed, recovery **fails closed**. It does not
  choose an orphaned file or reconstruct a registry by guessing.
- Temporary files and unreferenced generation bundles are ignored.
  This initial implementation does not garbage-collect them.
- \`rollback(expected_active_fingerprint)\` explicitly selects the
  recorded previous valid bundle, leaving the rejected generation
  file available for diagnostics; it does not rewrite either map.

\`load_active()\` checks the active bundle's fingerprint and checksum.
It does not silently perform recovery: callers should explicitly call
\`recover()\` first and log that event if fallback occurred.

## Connecting GTSAM map promotion

The optional GTSAM map-generation module now offers
\`publish_approved_generation(store, evaluated)\`.
It returns no generation and writes nothing if the existing
held-out quality gate rejected the candidate. For accepted candidates,
the function confirms:

1. The selected authoritative immutable snapshot **is** the candidate.
2. Candidate and incumbent fingerprints differ.
3. The store's active map matches the evaluated incumbent snapshot.
4. A single compare-and-swap publication succeeds against that state.

Thus optimizing a graph cannot by itself replace an authoritative map.
This bridge still relies on externally justified, independent graph
factors and reserved evaluation data; source-ID assertions are not
proof of statistical independence. FR079 logged poses do not meet that
bar for real-data map promotion.

Minimal integration sketch (after constructing validated map bundles):

~~~cpp
slam::mapping::GenerationStore store("/var/lib/fusion-slam/maps");
auto recovered = store.recover();  // read/validate on restart

// Initial bootstrap, exactly once:
auto initial = store.publish(incumbent_bundle, std::nullopt);

// After a completely separate GTSAM + held-out quality evaluation:
auto committed = slam::offline::publish_approved_generation(store, result);
if (committed) {
  // Newly selected generation is durable.
}
~~~

Avoid using \`publish\` directly to bypass the higher-level approval gate
when producing graph-refined maps.

## Test and runtime boundary

~~~sh
# With system GTSAM installed or restored as a versioned dependency prefix:
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON \
  -DSLAM_ENABLE_GTSAM=ON -DSLAM_REQUIRE_GTSAM=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure \
  -R '^(map_registry|offline_map_promotion|map_bundle)$'
~~~

Local focused C++20/GCC14 tests linked against GTSAM 4.3 and the
relevant map modules: **2/2 passed** in Release and with
\`-Wall -Wextra -Wpedantic -Werror\` plus UndefinedBehaviorSanitizer.
Tests simulate corrupted active map, corrupted pointer, incomplete
temporary file, orphan map file, restart recovery, explicit rollback,
stale compare-and-swap, two competing publishers, and a rejected versus
approved graph candidate. The complete repository CTest matrix and
Ubuntu 24.04 Docker runtime still require a separate run.

**Limits:** Linux/POSIX local filesystem with durable atomic rename,
fsync and advisory flock semantics; not NFS/object storage. Directory
permissions and filesystem integrity are the owner's responsibility.
The store is not designed against a malicious local writer, has no
automatic retention/garbage collection policy, no multi-map frame
registry, no external ground-truth acquisition, and no tested
power-loss/fault-injection simulator. Preserve separate backups of
published authoritative maps.
