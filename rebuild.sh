#!/usr/bin/env bash
# Full clean rebuild — deletes build/, re-runs conan install, cmake configure,
# and builds all targets.
#
# Usage:
#   ./rebuild.sh            Release build (default)
#   ./rebuild.sh Debug      Debug build
#   ./rebuild.sh --test     Release build + run ctest afterwards

set -euo pipefail

export PATH="/c/msys64/ucrt64/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="build"
BUILD_TYPE="Release"
RUN_TESTS=0

for arg in "$@"; do
    if [[ "$arg" == "--test" ]]; then
        RUN_TESTS=1
    elif [[ "$arg" == "Debug" || "$arg" == "Release" ]]; then
        BUILD_TYPE="$arg"
    else
        echo "ERROR: unknown argument '$arg'  (valid: Release | Debug | --test)" >&2
        exit 1
    fi
done

JOBS=$(nproc 2>/dev/null || echo 4)

echo "==================================================================="
echo " Full rebuild  |  type: $BUILD_TYPE  |  jobs: $JOBS"
echo "==================================================================="

# ---------------------------------------------------------------------------
# 1. Delete build/
# ---------------------------------------------------------------------------

if [[ -d "$BUILD_DIR" ]]; then
    echo ""
    echo "--- Removing $BUILD_DIR/ ---"
    rm -rf "$BUILD_DIR"
fi

# ---------------------------------------------------------------------------
# 2. Conan install
# ---------------------------------------------------------------------------

echo ""
echo "--- conan install ($BUILD_TYPE) ---"
conan install . \
    --output-folder="$BUILD_DIR" \
    --build=missing \
    --profile=liteaero-gcc \
    --settings "build_type=$BUILD_TYPE"

# ---------------------------------------------------------------------------
# 3. CMake configure
# ---------------------------------------------------------------------------

echo ""
echo "--- cmake configure ---"
cmake -B "$BUILD_DIR" -G "MinGW Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE="$BUILD_DIR/conan_toolchain.cmake" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON

# ---------------------------------------------------------------------------
# 4. Build all targets
# ---------------------------------------------------------------------------

echo ""
echo "--- build ---"
mingw32-make -C "$BUILD_DIR" -j"$JOBS"

# ---------------------------------------------------------------------------
# 5. Optional: run tests
# ---------------------------------------------------------------------------

if [[ $RUN_TESTS -eq 1 ]]; then
    echo ""
    echo "--- ctest ---"
    ctest --test-dir "$BUILD_DIR" --output-on-failure
fi

echo ""
echo "==================================================================="
echo " Rebuild complete ($BUILD_TYPE)"
echo "==================================================================="
