# Building liteaero-sim

All builds are driven by [`build.sh`](build.sh) at the repository root — a single
entry point over the Conan 2.x + MSYS2 ucrt64 GCC + CMake (MinGW Makefiles)
toolchain. It can configure, build individual components, clean, rebuild, run
tests, and (re)build the Godot plugin.

```bash
./build.sh                  # build everything (Release)
./build.sh help             # full usage
```

## Prerequisites

The build is pinned to one toolchain (see [`CLAUDE.md`](CLAUDE.md) for the rationale):

| Tool | Location |
| --- | --- |
| Compiler | `C:/msys64/ucrt64/bin/c++.exe` (GCC 15.x) |
| Make | `C:/msys64/ucrt64/bin/mingw32-make.exe` |
| Conan | `C:/msys64/ucrt64/bin/conan.exe` (Conan 2.x, installed via MSYS2 pacman) |
| Generator | MinGW Makefiles |
| Conan profile | `~/.conan2/profiles/liteaero-gcc` |

`build.sh` prepends `C:/msys64/ucrt64/bin` to `PATH`, so you do not need to do it
yourself. Run it from an MSYS2 / Git-Bash shell.

**Python bindings.** The pybind11 extension (`liteaero_sim_py.pyd`) must link the
uv-managed MSVC Python so `uv run python` can import it. `build.sh` auto-detects
that interpreter under `~/AppData/Roaming/uv/python/cpython-3.1*-windows-x86_64-none/`
and passes it as `-DPython3_EXECUTABLE`. Override with `--python PATH` or the
`LITEAERO_PYTHON_EXE` environment variable.

## Commands

| Command | Purpose |
| --- | --- |
| `build [comp...]` | Build everything, or only the named component(s). **Default.** |
| `rebuild [comp...]` | `make clean`, (re)configure if needed, then build. |
| `configure` | Force a fresh Conan install + two-pass CMake configure. |
| `clean` | Remove compiled objects; keep configuration and fetched deps. |
| `distclean` | Delete `build/` entirely (next configure re-fetches godot-cpp). |
| `deps` | Force-rebuild the vendored `godot-cpp` library (recovers a stale-plugin link without a full `distclean`). |
| `test [regex]` | Build the test binary and run `ctest` (optional `-R` regex). |
| `gdext` | Build the Godot plugin and stage it into the addon's `bin/`. |
| `list` | List component aliases and their CMake targets. |
| `help` | Show usage. |

If `build/` is not configured (or you switch build type), the needed
commands configure it automatically — there is no separate first-run step.

## Components

Pass one or more component aliases to `build`/`rebuild`:

| Alias | CMake target | What it is |
| --- | --- | --- |
| `core`, `lib` | `liteaerosim` | core simulation library |
| `proto` | `liteaerosim_proto` | generated protobuf sources |
| `test`, `tests` | `liteaerosim_test` | C++ Google Test binary |
| `py`, `bindings` | `liteaero_sim_py` | Python pybind11 `.pyd` (for notebooks) |
| `sim`, `live` | `live_sim` | C++ joystick/terrain launcher (used by `run_sim.sh`) |
| `gdext`, `godot`, `plugin` | `liteaero_sim_gdext` | Godot 4 GDExtension `.dll` |
| `tools` | `live_sim mock_sim joystick_verify` | command-line tools |
| `all` | *(default target)* | everything |

## Options

Global options may appear anywhere on the command line.

| Option | Effect |
| --- | --- |
| `-d`, `--debug` | Configure/build the **Debug** type (default: Release). Triggers a Conan install + reconfigure for that type. |
| `-j`, `--jobs N` | Parallel build jobs (default: `nproc`). |
| `-v`, `--verbose` | Shell trace (`set -x`) **and** `VERBOSE=1` so the compiler/link commands are echoed. |
| `-p`, `--python PATH` | Python interpreter for the pybind extension. |
| `-h`, `--help` | Show help. |

## Examples

```bash
./build.sh                        # build everything (Release)
./build.sh build py sim           # only the Python bindings and live_sim
./build.sh -d build test          # Debug build of the C++ test binary
./build.sh gdext                  # rebuild + stage the Godot plugin
./build.sh test BodyCollider      # build tests, run those matching the regex
./build.sh -v rebuild core        # verbose clean-rebuild of the core library
./build.sh distclean && ./build.sh   # nuke and full rebuild
```

## The Godot plugin (`gdext`)

The Godot viewer loads a compiled GDExtension, `liteaero_sim_gdext.dll`, from
`godot/addons/liteaero_sim/bin/`. It is a separate artifact from `live_sim`: the
two communicate over UDP using the `SimulationFrameProto` protobuf message. **If
the broadcast schema or the receiver changes, the plugin must be rebuilt** —
otherwise the viewer silently reads stale fields (e.g. the aircraft appears
frozen/embedded because the receiver ignores the `viewer_x/y/z` it does not know
about).

```bash
./build.sh gdext
```

`build.sh gdext` builds the `liteaero_sim_gdext` target, which CMake stages
directly into `godot/addons/liteaero_sim/bin/` with the `lib` prefix stripped
(matching `liteaero_sim.gdextension`) and copies the required `libwinpthread-1.dll`
beside it. A full `./build.sh` (no args) also builds it.

> The plugin is built behind the `LITEAERO_SIM_BUILD_GODOT_PLUGIN` CMake option,
> enabled automatically by `configure`. The first configure of a fresh `build/`
> tree fetches `godot-cpp` (matched to the editor version in `godot/project.godot`)
> via FetchContent, which needs network access. Prefer `clean` over `distclean`
> to avoid re-fetching it.

### Recovering a stale plugin (`undefined reference` to a `godot::` symbol)

When the Godot editor version (and thus the godot-cpp tag) changes, FetchContent
re-checks-out the new godot-cpp source with git-commit timestamps that can be
**older** than the previously-built objects. `make` then does not rebuild the
vendored library and links a stale archive, so the plugin fails with
`undefined reference to godot::...`. The artifact on disk silently stays old.

```bash
./build.sh deps     # force-rebuild only godot-cpp, then:
./build.sh gdext    # relink + stage the plugin
```

`deps` clears godot-cpp's compiled objects and archive (keeping the make rules)
and rebuilds from the current source — much faster than the full
`./build.sh distclean && ./build.sh`. Any build that fails this way prints the
same hint.

## Build flow (what `configure` does)

The Godot plugin's CMakeLists needs the generated `liteaerosim.pb.cc` to exist at
configure time, so configuration is two-pass:

1. `conan install` for the selected build type → `build/conan_toolchain.cmake`.
2. CMake configure (plugin off) → generate Makefiles.
3. `make liteaerosim_proto` → emit `liteaerosim.pb.{h,cc}`.
4. CMake reconfigure with `-DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON`.

`build.sh` also re-runs step 3 on any build when `proto/liteaerosim.proto` is
newer than the generated header (the plugin target lacks a direct dependency edge
to the codegen step).

## Running the simulator

Building is separate from running. To launch `live_sim` + the Godot viewer:

```bash
./run_sim.sh                # scripted/joystick live sim + Godot viewer
```

After changing C++ that affects the sim or the broadcast, rebuild the relevant
artifacts before running, e.g.:

```bash
./build.sh sim gdext        # live_sim + the Godot plugin
./run_sim.sh
```

## Related scripts

| Script | Role |
| --- | --- |
| [`build.sh`](build.sh) | Unified build manager (this document). |
| [`rebuild.sh`](rebuild.sh) | Legacy full clean rebuild; `./build.sh distclean && ./build.sh` is the equivalent. |
| [`run_sim.sh`](run_sim.sh) | Launch the live simulation and the Godot viewer. |
