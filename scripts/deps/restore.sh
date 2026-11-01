#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then echo "Usage: $0 /path/to/slam-deps-*.tar.zst /path/to/empty/install-parent" >&2; exit 2; fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
ARCHIVE="$(realpath -- "$1")"
SIDE="$ARCHIVE.sha256"
[[ -f "$ARCHIVE" && -f "$SIDE" ]] || { echo 'Archive and .sha256 sidecar are both required' >&2; exit 1; }
(cd -- "$(dirname -- "$ARCHIVE")" && sha256sum --check -- "$(basename -- "$SIDE")")
mkdir -p -- "$2"
PARENT="$(cd -- "$2" && pwd -P)"
NAME="$(basename -- "$ARCHIVE" .tar.zst)"
[[ "$NAME" == slam-deps-* ]] || { echo 'Unexpected archive basename' >&2; exit 1; }
[[ ! -e "$PARENT/$NAME" ]] || { echo 'Destination already exists; refusing overwrite' >&2; exit 1; }
# Validate archive members before extraction (no absolute paths or traversal).
python3 - "$ARCHIVE" "$NAME" <<'PY'
import subprocess,sys
output=subprocess.check_output(['tar','-I','zstd','-tf',sys.argv[1]],text=True)
root=sys.argv[2]
for line in output.splitlines():
    parts=line.split('/')
    if line.startswith('/') or '..' in parts or parts[0]!=root:
        raise SystemExit('Unsafe archive member: '+line)
PY
TMP="$(mktemp -d -p "$PARENT")"
trap 'rm -rf -- "$TMP"' EXIT
tar -I zstd -xf "$ARCHIVE" -C "$TMP"
python3 "$ROOT/scripts/deps/verify.py" "$TMP/$NAME"
SLAM_SUPPRESS_USAGE=1 "$ROOT/scripts/deps/smoke.sh" "$TMP/$NAME"
mv -- "$TMP/$NAME" "$PARENT/$NAME"
echo "Restored and smoke-tested: $PARENT/$NAME"
echo 'To configure 2d-fusion-slam with the restored dependencies:'
printf '  cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="%s" -DSLAM_REQUIRE_CERES=ON -DSLAM_REQUIRE_GTSAM=ON\n' "$PARENT/$NAME/install/eigen3;$PARENT/$NAME/install/catch2;$PARENT/$NAME/install/ceres;$PARENT/$NAME/install/gtsam"
