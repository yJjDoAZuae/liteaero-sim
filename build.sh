#!/usr/bin/env bash
# =============================================================================
# build.sh — unified build manager for liteaero-sim
#
# Wraps the Conan 2.x + MSYS2 ucrt64 GCC + CMake (MinGW Makefiles) toolchain.
# Build individual components, clean, rebuild, run tests, and (re)build the
# Godot plugin. Run `./build.sh help` or read BUILD.md for full documentation.
#
# Quick examples:
#   ./build.sh                     # build everything (Release)
#   ./build.sh build py sim        # build the Python bindings + live_sim only
#   ./build.sh -d build test       # Debug build of the C++ test binary
#   ./build.sh gdext               # rebuild the Godot plugin into the addon
#   ./build.sh test BodyCollider   # build + run tests matching a regex
#   ./build.sh -v rebuild core     # verbose clean-rebuild of the core library
#   ./build.sh clean               # remove compiled objects (keep config/deps)
#   ./build.sh distclean           # delete build/ entirely (re-fetches deps)
# =============================================================================
set -euo pipefail

# --- Toolchain: ucrt64 must be first on PATH for conan/cmake/make/ctest. ------
export PATH="/c/msys64/ucrt64/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="build"
CONAN_PROFILE="liteaero-gcc"
GDEXT_DLL="godot/addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
PROTO_SRC="proto/liteaerosim.proto"
PROTO_HDR="build/proto/liteaerosim.pb.h"

# --- Defaults (overridable via options) --------------------------------------
BUILD_TYPE="Release"
JOBS="$(nproc 2>/dev/null || echo 4)"
VERBOSE=0
PYTHON_EXE="${LITEAERO_PYTHON_EXE:-}"   # pybind interpreter; auto-detected if empty

# --- Pretty output -----------------------------------------------------------
if [[ -t 1 ]]; then
    C_B=$'\e[1m'; C_G=$'\e[32m'; C_Y=$'\e[33m'; C_R=$'\e[31m'; C_0=$'\e[0m'
else
    C_B=''; C_G=''; C_Y=''; C_R=''; C_0=''
fi
log()  { printf '%s==>%s %s\n' "${C_G}${C_B}" "$C_0" "$*"; }
warn() { printf '%s!!%s %s\n'  "$C_Y"          "$C_0" "$*" >&2; }
die()  { printf '%sERROR:%s %s\n' "${C_R}${C_B}" "$C_0" "$*" >&2; exit 1; }

# --- Component alias -> CMake target(s) --------------------------------------
# Empty output means "the default 'all' target".
comp_targets() {
    case "$1" in
        core|lib)            echo "liteaerosim" ;;
        proto)               echo "liteaerosim_proto" ;;
        test|tests)          echo "liteaerosim_test" ;;
        py|bindings)         echo "liteaero_sim_py" ;;
        sim|live|live_sim)   echo "live_sim" ;;
        gdext|godot|plugin)  echo "liteaero_sim_gdext" ;;
        tools)               echo "live_sim mock_sim joystick_verify" ;;
        all|"")              echo "" ;;
        *) die "unknown component '$1'  (run: ./build.sh list)" ;;
    esac
}

# True if the argument is a known component alias (used so a bare component name
# is accepted as shorthand for `build <component>`).
is_component() {
    case "$1" in
        core|lib|proto|test|tests|py|bindings|sim|live|live_sim|gdext|godot|plugin|tools|all)
            return 0 ;;
        *) return 1 ;;
    esac
}

# --- Helpers -----------------------------------------------------------------
detect_python() {
    if [[ -n "$PYTHON_EXE" ]]; then printf '%s\n' "$PYTHON_EXE"; return; fi
    local g
    for g in "$HOME"/AppData/Roaming/uv/python/cpython-3.1*-windows-x86_64-none/python.exe; do
        [[ -f "$g" ]] && { printf '%s\n' "$g"; return; }
    done
    printf '\n'   # not found — CMake falls back to its own discovery
}

run_make() {
    local extra=()
    [[ $VERBOSE -eq 1 ]] && extra+=("VERBOSE=1")
    if ! mingw32-make -C "$BUILD_DIR" -j"$JOBS" "${extra[@]}" "$@"; then
        warn "build failed."
        warn "  If the Godot plugin failed with an 'undefined reference' to a godot:: symbol,"
        warn "  the vendored godot-cpp library is stale (a tag/version change make cannot detect):"
        warn "    recover with:  ./build.sh deps        (force-rebuild godot-cpp only)"
        warn "    full recovery: ./build.sh distclean && ./build.sh"
        exit 1
    fi
}

configured_build_type() {
    [[ -f "$BUILD_DIR/CMakeCache.txt" ]] || { printf '\n'; return; }
    grep -E '^CMAKE_BUILD_TYPE:' "$BUILD_DIR/CMakeCache.txt" | head -1 | cut -d= -f2
}

# The Godot plugin target carries no Makefile edge to the protoc codegen step,
# so a lone `gdext`/single-target build can miss a stale generated header.
# Regenerate explicitly when the .proto is newer than its generated header.
check_proto_stale() {
    if [[ -f "$PROTO_SRC" && ( ! -f "$PROTO_HDR" || "$PROTO_SRC" -nt "$PROTO_HDR" ) ]]; then
        log "proto changed — regenerating liteaerosim_proto"
        run_make liteaerosim_proto
    fi
}

# Full two-pass configure: protobuf sources must be generated before the Godot
# plugin's CMakeLists can find liteaerosim.pb.cc at configure time.
do_configure() {
    local pyargs=() py; py="$(detect_python)"
    if [[ -n "$py" ]]; then
        pyargs+=("-DPython3_EXECUTABLE=$py")
        log "pybind interpreter: $py"
    else
        warn "no uv MSVC Python found; the .pyd may not import under 'uv run python'"
    fi

    log "conan install ($BUILD_TYPE)"
    conan install . --output-folder="$BUILD_DIR" --build=missing \
        --profile="$CONAN_PROFILE" --settings "build_type=$BUILD_TYPE"

    log "cmake configure (pass 1/2 — generate protobuf sources)"
    cmake -B "$BUILD_DIR" -G "MinGW Makefiles" \
        -DCMAKE_TOOLCHAIN_FILE="$BUILD_DIR/conan_toolchain.cmake" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "${pyargs[@]}"

    log "generate protobuf sources (liteaerosim_proto)"
    run_make liteaerosim_proto

    log "cmake configure (pass 2/2 — enable Godot plugin)"
    cmake -B "$BUILD_DIR" -G "MinGW Makefiles" \
        -DCMAKE_TOOLCHAIN_FILE="$BUILD_DIR/conan_toolchain.cmake" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON "${pyargs[@]}"
}

# Configure only when needed (no tree, or build-type changed).
ensure_configured() {
    local cur; cur="$(configured_build_type)"
    if [[ -z "$cur" ]]; then
        log "no build tree — configuring ($BUILD_TYPE)"
        do_configure
    elif [[ "$cur" != "$BUILD_TYPE" ]]; then
        log "build type $cur -> $BUILD_TYPE — reconfiguring"
        do_configure
    fi
}

# Force-rebuild the vendored godot-cpp library. FetchContent re-checks-out the
# source with git-commit mtimes that can be OLDER than previously-built objects,
# so `make` does not notice a godot-cpp tag/version change and links a stale
# archive (symptom: the plugin fails with `undefined reference` to a godot::
# symbol). This clears godot-cpp's objects + archive (keeping the make rules) and
# rebuilds from the current source — far cheaper than a full `distclean`.
refresh_godot_cpp() {
    ensure_configured
    local gdir
    gdir="$(find "$BUILD_DIR/_deps" -type d -name 'godot-cpp.dir' 2>/dev/null | head -1)"
    [[ -n "$gdir" ]] || die "godot-cpp build dir not found — use: ./build.sh distclean && ./build.sh"
    log "clearing stale godot-cpp objects + archive"
    find "$gdir" -name '*.obj' -delete 2>/dev/null || true
    find "$BUILD_DIR" -name 'libgodot-cpp*.a' -delete 2>/dev/null || true
    log "rebuilding godot-cpp from source"
    run_make godot-cpp
}

# --- Commands ----------------------------------------------------------------
cmd_configure() { do_configure; }

cmd_deps() { refresh_godot_cpp; log "godot-cpp refreshed — now: ./build.sh gdext"; }

cmd_build() {
    ensure_configured
    check_proto_stale
    if [[ $# -eq 0 ]]; then
        log "build all ($BUILD_TYPE, -j$JOBS)"
        run_make
    else
        local targets=() c t
        for c in "$@"; do
            for t in $(comp_targets "$c"); do targets+=("$t"); done
        done
        if [[ ${#targets[@]} -eq 0 ]]; then
            log "build all ($BUILD_TYPE, -j$JOBS)"; run_make
        else
            log "build: ${targets[*]}  ($BUILD_TYPE, -j$JOBS)"
            run_make "${targets[@]}"
        fi
    fi
    log "done."
}

cmd_clean() {
    if [[ -f "$BUILD_DIR/Makefile" ]]; then
        log "make clean (keeps configuration and fetched deps)"
        run_make clean
    else
        warn "no build tree to clean"
    fi
}

cmd_distclean() {
    if [[ -d "$BUILD_DIR" ]]; then
        log "removing $BUILD_DIR/ (a full reconfigure will re-fetch godot-cpp)"
        rm -rf "$BUILD_DIR"
    else
        warn "no $BUILD_DIR/ to remove"
    fi
}

cmd_rebuild() {
    cmd_clean || true
    ensure_configured
    cmd_build "$@"
}

cmd_gdext() {
    ensure_configured
    check_proto_stale
    log "build Godot plugin -> godot/addons/liteaero_sim/bin/"
    run_make liteaero_sim_gdext
    if [[ -f "$GDEXT_DLL" ]]; then
        log "staged: $GDEXT_DLL  ($(date -r "$GDEXT_DLL" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo '?'))"
    else
        warn "expected $GDEXT_DLL not found after build"
    fi
}

cmd_test() {
    ensure_configured
    check_proto_stale
    log "build test binary (liteaerosim_test)"
    run_make liteaerosim_test
    local args=(--test-dir "$BUILD_DIR" --output-on-failure)
    if [[ $# -gt 0 ]]; then
        args+=(-R "$1"); log "ctest -R '$1'"
    else
        log "ctest (full suite)"
    fi
    ctest "${args[@]}"
}

cmd_list() {
    cat <<'EOF'
Components (alias -> CMake target):
  core | lib             liteaerosim                      core simulation library
  proto                  liteaerosim_proto                generated protobuf sources
  test | tests           liteaerosim_test                 C++ Google Test binary
  py | bindings          liteaero_sim_py                  Python pybind11 .pyd
  sim | live | live_sim  live_sim                         C++ joystick/terrain launcher
  gdext | godot | plugin liteaero_sim_gdext               Godot 4 GDExtension (.dll)
  tools                  live_sim mock_sim joystick_verify command-line tools
  all                    (default target)                 everything

Commands: configure | build | rebuild | clean | distclean | deps | test | gdext | list | help
Options:  -d/--debug  -j/--jobs N  -v/--verbose  -p/--python PATH  -h/--help
EOF
}

cmd_help() {
    cat <<EOF
${C_B}build.sh${C_0} — unified build manager for liteaero-sim

${C_B}USAGE${C_0}
  ./build.sh [options] <command> [components...]

${C_B}COMMANDS${C_0}
  build [comp...]    Build everything, or only the named component(s)  (default)
  rebuild [comp...]  make clean, then (re)configure if needed, then build
  configure          Force a fresh Conan install + two-pass CMake configure
  clean              Remove compiled objects (keeps config and fetched deps)
  distclean          Delete build/ entirely (next configure re-fetches godot-cpp)
  deps               Force-rebuild vendored godot-cpp (fixes a stale-plugin link)
  test [regex]       Build the test binary and run ctest (optional -R regex)
  gdext              Build the Godot plugin and stage it into the addon's bin/
  list               List component aliases and their CMake targets
  help               Show this help

${C_B}OPTIONS${C_0} (may appear anywhere on the line)
  -d, --debug        Configure/build the Debug type (default: Release)
  -j, --jobs N       Parallel build jobs (default: $(nproc 2>/dev/null || echo 4))
  -v, --verbose      Shell trace + VERBOSE=1 compiler command echo
  -p, --python PATH  Python interpreter for the pybind extension
                     (env: LITEAERO_PYTHON_EXE; auto-detected from uv otherwise)
  -h, --help         Show this help

${C_B}EXAMPLES${C_0}
  ./build.sh                        Build everything (Release)
  ./build.sh build py sim           Build the Python bindings and live_sim
  ./build.sh -d build test          Debug build of the C++ test binary
  ./build.sh gdext                  Rebuild the Godot plugin (fixes a stale .dll)
  ./build.sh test BodyCollider      Build tests and run those matching the regex
  ./build.sh -v rebuild core        Verbose clean-rebuild of the core library
  ./build.sh distclean && ./build.sh   Nuke and full rebuild

See ${C_B}BUILD.md${C_0} for full documentation.
EOF
}

# --- Parse arguments (global options may appear anywhere) --------------------
POSARGS=()
SHOW_HELP=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--debug)   BUILD_TYPE="Debug" ;;
        -v|--verbose) VERBOSE=1 ;;
        -j|--jobs)    JOBS="${2:?--jobs needs a number}"; shift ;;
        -p|--python)  PYTHON_EXE="${2:?--python needs a path}"; shift ;;
        -h|--help)    SHOW_HELP=1 ;;
        --)           shift; while [[ $# -gt 0 ]]; do POSARGS+=("$1"); shift; done; break ;;
        -*)           die "unknown option '$1'  (run: ./build.sh help)" ;;
        *)            POSARGS+=("$1") ;;
    esac
    shift
done

[[ $VERBOSE -eq 1 ]] && set -x
[[ $SHOW_HELP -eq 1 ]] && { cmd_help; exit 0; }

COMMAND="${POSARGS[0]:-build}"
ARGS=("${POSARGS[@]:1}")

case "$COMMAND" in
    configure) cmd_configure ;;
    build)     cmd_build "${ARGS[@]}" ;;
    rebuild)   cmd_rebuild "${ARGS[@]}" ;;
    clean)     cmd_clean ;;
    distclean) cmd_distclean ;;
    deps)      cmd_deps ;;
    test)      cmd_test "${ARGS[@]}" ;;
    gdext)     cmd_gdext ;;
    list)      cmd_list ;;
    help)      cmd_help ;;
    *)
        # Bare component name → shorthand for `build <component> [more...]`.
        if is_component "$COMMAND"; then
            cmd_build "$COMMAND" "${ARGS[@]}"
        else
            die "unknown command or component '$COMMAND'  (run: ./build.sh help)"
        fi
        ;;
esac
