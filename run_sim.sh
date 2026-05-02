#!/usr/bin/env bash
# run_sim.sh — launch live_sim.exe + Godot viewer for small_uas_ksba.
#
# Usage:
#   ./run_sim.sh               scripted input (no joystick)
#   ./run_sim.sh --joystick    joystick input via python/gx12_config.json
#   ./run_sim.sh --device N    joystick device index (default 0, implies --joystick)
#
# Environment:
#   GODOT_EXE   override Godot executable (default: godot4 on PATH)

set -euo pipefail

export PATH="/c/msys64/ucrt64/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
CONFIG="./configs/small_uas_ksba.json"
TERRAIN="./data/terrain/small_uas_ksba/terrain_config.json"
JOYSTICK_CONFIG="./python/gx12_config.json"
LIVE_SIM_EXE="./build/tools/live_sim.exe"
GODOT_PROJECT="./godot"
_GODOT_DEFAULT="/c/Program Files/Godot/Godot_v4.6.2-stable_win64.exe"

USE_JOYSTICK=1
DEVICE_INDEX=0

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
for arg in "$@"; do
    case "$arg" in
        --joystick) USE_JOYSTICK=1 ;;
        --device)   shift; DEVICE_INDEX="$1"; USE_JOYSTICK=1 ;;
        *) echo "ERROR: unknown argument '$arg'  (valid: --joystick | --device <N>)" >&2; exit 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
if [[ ! -f "$LIVE_SIM_EXE" ]]; then
    echo "ERROR: $LIVE_SIM_EXE not found. Run ./rebuild.sh first." >&2
    exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
    echo "ERROR: aircraft config not found: $CONFIG" >&2
    exit 1
fi
if [[ ! -f "$TERRAIN" ]]; then
    echo "ERROR: terrain config not found: $TERRAIN" >&2
    echo "       Run python/tools/terrain/build_terrain.py small_uas_ksba first." >&2
    exit 1
fi

# Resolve Godot executable
if [[ -n "${GODOT_EXE:-}" ]]; then
    GODOT="$GODOT_EXE"
elif command -v godot4 &>/dev/null; then
    GODOT="godot4"
elif command -v godot &>/dev/null; then
    GODOT="godot"
elif [[ -f "$_GODOT_DEFAULT" ]]; then
    GODOT="$_GODOT_DEFAULT"
else
    echo "ERROR: Godot executable not found. Install godot4 or set GODOT_EXE." >&2
    exit 1
fi

# Absolute path to terrain config for Godot (avoids cwd dependency).
TERRAIN_ABS="$(cd "$(dirname "$TERRAIN")" && pwd)/$(basename "$TERRAIN")"

# ---------------------------------------------------------------------------
# Kill live_sim when this script exits (Ctrl+C, Godot closes, or error).
# ---------------------------------------------------------------------------
LIVE_SIM_PID=""
cleanup() {
    if [[ -n "$LIVE_SIM_PID" ]] && kill -0 "$LIVE_SIM_PID" 2>/dev/null; then
        echo ""
        echo "Stopping live_sim (pid $LIVE_SIM_PID)..."
        kill "$LIVE_SIM_PID" 2>/dev/null || true
        wait "$LIVE_SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Build live_sim command
# ---------------------------------------------------------------------------
LIVE_SIM_CMD=("$LIVE_SIM_EXE" --config "$CONFIG" --terrain "$TERRAIN")
if [[ $USE_JOYSTICK -eq 1 ]]; then
    if [[ ! -f "$JOYSTICK_CONFIG" ]]; then
        echo "ERROR: joystick config not found: $JOYSTICK_CONFIG" >&2
        exit 1
    fi
    LIVE_SIM_CMD+=(--joystick "$JOYSTICK_CONFIG" --device "$DEVICE_INDEX")
fi

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
echo "Starting live_sim..."
echo "  ${LIVE_SIM_CMD[*]}"
"${LIVE_SIM_CMD[@]}" &
LIVE_SIM_PID=$!

# Brief pause to let live_sim initialize before Godot connects.
sleep 0.5

if ! kill -0 "$LIVE_SIM_PID" 2>/dev/null; then
    echo "ERROR: live_sim.exe exited immediately. Check config paths and ucrt64 DLLs." >&2
    exit 1
fi

echo "live_sim running (pid $LIVE_SIM_PID)"
echo ""
echo "Starting Godot..."
echo "  $GODOT --path $GODOT_PROJECT -- --terrain $TERRAIN_ABS"
"$GODOT" --path "$GODOT_PROJECT" -- --terrain "$TERRAIN_ABS"

echo "Godot exited."
