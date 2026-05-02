#!/usr/bin/env bash
# Incremental build — rebuilds only what has changed since the last build.
#
# Usage:
#   ./build.sh                   build all targets
#   ./build.sh live_sim          build only live_sim.exe
#   ./build.sh liteaero_sim_gdext  build only the Godot GDExtension DLL
#   ./build.sh live_sim liteaero_sim_gdext  build both
#   ./build.sh --test            build all, then run ctest
#
# Requires a configured build/ directory; run ./rebuild.sh if build/ is absent.

set -euo pipefail

export PATH="/c/msys64/ucrt64/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="build"
RUN_TESTS=0
TARGETS=()

for arg in "$@"; do
    if [[ "$arg" == "--test" ]]; then
        RUN_TESTS=1
    else
        TARGETS+=("$arg")
    fi
done

# ---------------------------------------------------------------------------
# Preflight check
# ---------------------------------------------------------------------------

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "ERROR: build/ is not configured.  Run ./rebuild.sh to do a full configure + build." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Proto staleness check
#
# The GDExtension target does not carry a direct Makefile dependency edge from
# liteaero_sim_gdext → the protoc codegen step.  When only that target is
# requested, make skips the stale-proto check.  We handle it here explicitly.
# (Full-target builds via make-all do traverse the dependency correctly.)
# ---------------------------------------------------------------------------

PROTO_SRC="proto/liteaerosim.proto"
PROTO_HDR="$BUILD_DIR/proto/liteaerosim.pb.h"

if [[ -f "$PROTO_SRC" && ( ! -f "$PROTO_HDR" || "$PROTO_SRC" -nt "$PROTO_HDR" ) ]]; then
    echo ">>> proto/liteaerosim.proto is newer than generated header — regenerating..."
    mingw32-make -C "$BUILD_DIR" liteaerosim_proto
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

JOBS=$(nproc 2>/dev/null || echo 4)

if [[ ${#TARGETS[@]} -gt 0 ]]; then
    echo ">>> Building targets: ${TARGETS[*]}"
    mingw32-make -C "$BUILD_DIR" -j"$JOBS" "${TARGETS[@]}"
else
    echo ">>> Building all targets"
    mingw32-make -C "$BUILD_DIR" -j"$JOBS"
fi

# ---------------------------------------------------------------------------
# Optional: run tests
# ---------------------------------------------------------------------------

if [[ $RUN_TESTS -eq 1 ]]; then
    echo ""
    echo ">>> Running tests"
    ctest --test-dir "$BUILD_DIR" --output-on-failure
fi

echo ""
echo ">>> Build complete."
