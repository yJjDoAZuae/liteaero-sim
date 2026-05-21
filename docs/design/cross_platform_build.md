# Cross-Platform Build — Architecture and Requirements

Design authority for building liteaero-sim and the GDExtension plugin on
Windows, macOS, and Linux.

Closely related to:
- [godot_plugin.md](godot_plugin.md) — GDExtension plugin design
- [docs/installation/README.md](../installation/README.md) — current Windows build instructions

---

## Current State

The project builds and runs on **Windows (MSYS2 ucrt64 / GCC 15.2)** only.
The C++ source code is already portable; the gaps are in the build system,
Conan profiles, and the `.gdextension` manifest.

---

## Source Code Portability

### GDExtension plugin (`godot/addons/liteaero_sim/src/`)

Already fully portable:

| File | Platform isolation |
| --- | --- |
| `SimulationReceiver.cpp` | `#ifdef _WIN32` / POSIX socket split — WinSock2 on Windows, `sys/socket.h` + `fcntl` on POSIX |
| `register_types.cpp` | `#ifdef _WIN32` WSAStartup / WSACleanup guards |

No source changes are required to build on macOS or Linux.

### Core simulation library (`src/`, `include/`)

No known platform-specific code. Eigen3, nlohmann/json, protobuf, and
trochoids are all cross-platform. The SDL2 joystick dependency used by
`live_sim` is cross-platform via Conan.

---

## CMakeLists Portability

### GDExtension plugin CMakeLists

| Item | Status |
| --- | --- |
| `ws2_32` WinSock link | Already guarded: `$<$<BOOL:${WIN32}>:ws2_32>` |
| `winpthread` static link | Already guarded: `$<$<BOOL:${MINGW}>:...>` |
| `PREFIX ""` (strip `lib`) | Fixed: `if(WIN32)` conditional — Windows only |
| Error message `mingw32-make` | Fixed: uses `cmake --build` |
| godot-cpp tag derivation | Cross-platform — parses `project.godot`, no OS dependency |

### Root CMakeLists / test CMakeLists

| Item | Status |
| --- | --- |
| `-mconsole -Wl,--subsystem,console` linker flags | Already guarded: `if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU\|Clang")` |
| SDL2 DLL warning | Windows-only warning, no action needed |

---

## Open Items

### OQ-CPB-1 — Conan profiles for macOS and Linux

**Status: Unresolved.**

The project currently ships one Conan profile (`~/.conan2/profiles/liteaero-gcc`)
targeting MSYS2 ucrt64 GCC 15.2 on Windows. No profiles exist for macOS or Linux.

#### Required profiles

| Platform | Compiler | Profile name (proposed) |
| --- | --- | --- |
| Windows (current) | MSYS2 ucrt64 GCC | `liteaero-gcc` |
| macOS | Apple Clang (Xcode) | `liteaero-appleclang` |
| macOS (alternative) | Homebrew GCC | `liteaero-homebrew-gcc` |
| Linux | System GCC (g++) | `liteaero-linux-gcc` |
| Linux (alternative) | Clang | `liteaero-linux-clang` |

#### Design decisions required

1. **macOS — Apple Clang vs. Homebrew GCC.** The project uses C++17 with no
   compiler-specific extensions. Either compiler works. Apple Clang is the
   lowest-friction choice (already present with Xcode). Homebrew GCC more
   closely mirrors the Windows build environment.

2. **`conan install` generator expressions.** The current `conan_toolchain.cmake`
   uses `$<CONFIG:Release>` generator expressions that are invisible to Debug
   builds (documented in CLAUDE.md). The same issue applies on macOS/Linux when
   switching between build types. Separate `conan install` passes per
   `build_type` are required on all platforms.

3. **Make tool.** `mingw32-make` is Windows/MSYS2-specific. On macOS/Linux, use
   `make` (or `cmake --build`) directly. All build instructions should use
   `cmake --build build` rather than invoking the make tool directly.

#### Profile template (macOS / Apple Clang)

```ini
[settings]
os=Macos
arch=x86_64          # or armv8 for Apple Silicon
compiler=apple-clang
compiler.version=15
compiler.libcxx=libc++
compiler.cppstd=17
build_type=Release

[options]

[tool_requires]

[conf]
tools.cmake.cmaketoolchain:generator=Ninja  # or Unix Makefiles
```

#### Profile template (Linux / GCC)

```ini
[settings]
os=Linux
arch=x86_64
compiler=gcc
compiler.version=12
compiler.libcxx=libstdc++11
compiler.cppstd=17
build_type=Release

[options]

[conf]
tools.cmake.cmaketoolchain:generator=Ninja  # or Unix Makefiles
```

---

### OQ-CPB-2 — `.gdextension` manifest: macOS and Linux entries

**Status: Unresolved.**

The current manifest only declares Windows targets:

```ini
[configuration]
entry_symbol = "liteaero_sim_init"
compatibility_minimum = "4.1"

[libraries]
windows.debug.x86_64   = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
windows.release.x86_64 = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
```

#### Required additions

Linux and macOS library files keep the `lib` prefix (conventional for
`.so` and `.dylib`) and must be listed separately in the manifest.

Proposed manifest additions (file paths are relative to the Godot project root):

```ini
linux.debug.x86_64   = "res://addons/liteaero_sim/bin/libliteaero_sim_gdext.so"
linux.release.x86_64 = "res://addons/liteaero_sim/bin/libliteaero_sim_gdext.so"
macos.debug          = "res://addons/liteaero_sim/bin/libliteaero_sim_gdext.dylib"
macos.release        = "res://addons/liteaero_sim/bin/libliteaero_sim_gdext.dylib"
```

#### Design decisions required

1. **Apple Silicon (arm64).** Godot 4 runs natively on Apple Silicon. The
   manifest key for arm64 macOS is `macos.debug` (no arch suffix — Godot uses
   universal binaries or selects by runtime arch). Confirm whether a universal
   binary (`lipo`) or separate arm64/x86_64 builds are required.

2. **Linux arm64.** If the project targets ARM Linux (e.g., Raspberry Pi, Jetson),
   add `linux.debug.arm64` / `linux.release.arm64` entries.

3. **Separate debug/release DLLs.** Currently Windows uses the same DLL for both
   debug and release. A proper debug build would produce
   `liteaero_sim_gdext.debug.dll` (or similar). This is a separate decision
   independent of platform portability.

---

### OQ-CPB-3 — Build instructions for macOS and Linux

**Status: Unresolved.**

`CLAUDE.md` and `docs/installation/README.md` are Windows-specific. The
platform-agnostic build sequence (using `cmake --build` instead of
`mingw32-make`) is:

```bash
# Step 1: Install Conan packages
conan install . --output-folder=build --build=missing --profile=<platform-profile>

# Step 2: Configure
cmake -B build -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release

# Step 3: Build
cmake --build build

# Step 4: Test
ctest --test-dir build --output-on-failure
```

The generator can be `Ninja` (recommended on macOS/Linux) or `Unix Makefiles`.
`MinGW Makefiles` is Windows/MSYS2-specific.

`CLAUDE.md` should be updated to document platform-specific deviations
(compiler path, profile name, generator) once the Conan profiles are established.

---

## Implementation Plan

Implement in the following order:

1. **OQ-CPB-1** — Create and document Conan profiles for macOS (Apple Clang)
   and Linux (GCC). Verify the core library (`liteaerosim`) builds and tests
   pass on each platform. This is the prerequisite for everything else.

2. **OQ-CPB-2** — Add Linux and macOS entries to `liteaero_sim.gdextension`.
   Build the GDExtension plugin on each platform and verify Godot loads it.

3. **OQ-CPB-3** — Update `CLAUDE.md` and `docs/installation/README.md` with
   platform-specific build instructions derived from validated steps in items 1–2.

---

## What Is Already Done

The following cross-platform work is complete and requires no further action:

- Socket abstraction in `SimulationReceiver.cpp` (`#ifdef _WIN32` / POSIX)
- WSAStartup/WSACleanup guards in `register_types.cpp`
- `ws2_32` and `winpthread` link guards in the plugin CMakeLists
- `PREFIX ""` scoped to Windows-only in the plugin CMakeLists
- godot-cpp tag derivation logic — platform-independent
- All core domain library code (no platform-specific code identified)
