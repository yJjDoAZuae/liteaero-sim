#!/usr/bin/env bash
# Remove build artifacts while keeping the CMake + Conan configuration.
#
# After clean, run ./build.sh for a fresh incremental build (no re-conan, no
# re-cmake needed unless you pass --full).
#
# Usage:
#   ./clean.sh         remove .obj/.a/.exe/.dll files only (make clean)
#   ./clean.sh --full  delete build/ entirely (next run must be ./rebuild.sh)

set -euo pipefail

export PATH="/c/msys64/ucrt64/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="build"
FULL=0

for arg in "$@"; do
    if [[ "$arg" == "--full" ]]; then
        FULL=1
    else
        echo "ERROR: unknown argument '$arg'  (valid: --full)" >&2
        exit 1
    fi
done

if [[ $FULL -eq 1 ]]; then
    echo ">>> Removing $BUILD_DIR/ (full clean — run ./rebuild.sh to reconfigure)"
    rm -rf "$BUILD_DIR"
    echo ">>> Done."
else
    if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
        echo "INFO: build/ not configured — nothing to clean."
        exit 0
    fi
    echo ">>> make clean (keeps CMake + Conan config — run ./build.sh to rebuild)"
    mingw32-make -C "$BUILD_DIR" clean
    echo ">>> Done."
fi
