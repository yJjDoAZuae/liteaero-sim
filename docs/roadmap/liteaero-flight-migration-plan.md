# LiteAero Flight Repository Migration — Implementation Plan

## Progress

| Step | Description | Status |
| --- | --- | --- |
| 0 | Extract history with `git filter-repo`; push to remote | Complete |
| 1 | Repository scaffolding and CMake skeleton | Complete |
| 2 | `liteaero::log`: ILogger and logging infrastructure | Complete |
| 3 | `liteaero::control`: `DynamicElement` and `SisoElement` | Complete |
| 4 | `liteaero::control`: Filter hierarchy | Complete |
| 5 | `liteaero::control`: SISO elements and scheduling infrastructure | Complete |
| 6 | `KinematicStateSnapshot` design document | Not started |
| 7 | Shared interface target: `KinematicStateSnapshot`, `AircraftCommand`, `NavigationState` | Not started |
| 8 | `liteaero::terrain`: Terrain element types and `V_Terrain` | Not started |
| 9 | LiteAero Flight stub headers | Not started |
| 10 | liteaero-flight build and test verification | Not started |
| 11 | LiteAero Sim internal namespace migration | Not started |
| 12 | Final build and regression verification | Not started |

---

## Context and Rationale

This plan covers FC-1 (liteaero-flight repository creation and repo split). It is the
implementation record for a three-phase operation — not just what to do, but why each
structural decision was made.

### Why the two-repository split

The liteaero-flight repository exists because flight code and simulation code must not be
co-located in the same library. Simulation depends on flight code (it hosts the flight
software and drives it); flight code must never depend on simulation. Co-locating them in
one repository makes that dependency boundary unenforceable at the build-system level.
Separating them into two repositories with a versioned one-way dependency makes the boundary
structural rather than a matter of convention.

The full rationale — deployment model, ICD ownership, namespace structure — is in
`docs/architecture/system/future/decisions.md`. That document records architectural
target-state decisions. This document records execution decisions: how the migration is
sequenced, why it is structured the way it is, and what was done at each step.

### Why per-step duplication closure

The original plan deferred all LiteAero Sim code removal to a batched Phase 2 after all
liteaero-flight migration was complete. That structure was rejected for the following reason:

No subsystem in either repository is considered stable at this stage of development. No
integration testing has been done. A bug found in a migrated subsystem during Steps 3–9
must be fixed in both repositories if the duplication window is still open. There is no
mechanism to enforce that — the two copies will silently diverge. The longer the window
stays open, the larger the potential divergence and the harder Phase 2 becomes.

The revised strategy closes the duplication window as the last act of each migration step.
After the liteaero-flight half of a step is complete and its tests pass, the corresponding
liteaero-sim code is removed and liteaero-sim's build is verified against the liteaero-flight
copy. At that point there is one copy, tested in both build contexts, and no divergence is
possible.

This requires liteaero-sim to be able to consume liteaero-flight before any code can be
removed. Establishing that dependency mechanism is therefore a prerequisite to Step 2's
sim-side cleanup and must be done first, once, before any sim-side code is deleted.

### Why migration decisions are in this document

`docs/architecture/system/future/decisions.md` records decisions about the architecture's
target state — namespace structure, ICD ownership, deployment model. Those decisions remain
valid indefinitely and belong in the architecture documentation.

Migration decisions — phasing, sequencing, duplication strategy, what gets done in what
order and why — are tactical. They apply to the execution of this migration and become
historical after it completes. They are recorded here so that the rationale is available
during execution, and so that decisions.md is not cluttered with execution-level detail
that has no bearing on the target architecture.

---

## Migration Strategy

### Structure

The migration has three phases:

- **Phase 0** (Step 0): Seed the `liteaero-flight` repository from LiteAero Sim using
  `git filter-repo`, extracting only the commits that touch migrating files. The original
  LiteAero Sim repository is not modified.

- **Phase 1** (Steps 1–10): Build out liteaero-flight and close the duplication window
  per step. Each subsystem step has two halves: the liteaero-flight migration (new namespace,
  new paths, tests passing) and the liteaero-sim cleanup (dependency updated, old code deleted,
  sim tests passing). No step is considered complete until both halves are done. Steps 9 and
  10 have no sim-side cleanup — they create new material (stubs, verification baseline) that
  does not exist in LiteAero Sim.

- **Phase 2** (Steps 11–12): Apply the `liteaero::simulation` namespace to LiteAero Sim
  code that stays in LiteAero Sim (aircraft dynamics, sim-internal control elements, environment
  implementation classes). By this point all migrated code has already been removed from
  LiteAero Sim step by step, so Phase 2 is only a namespace rename of the remaining code,
  followed by a final audit.

### Consuming liteaero-flight from liteaero-sim

Before Step 2's sim-side cleanup can delete any code, liteaero-sim's CMake build must be
able to find and link liteaero-flight's targets. Three mechanisms are available:

| Mechanism | When to use |
| --- | --- |
| `add_subdirectory(../liteaero-flight liteaero-flight-build)` | Co-located development: both repos are sibling directories on the same machine; CMake builds liteaero-flight as part of the liteaero-sim configure step |
| `FetchContent` with a local path or Git URL | CI or distributed builds; liteaero-flight is fetched at a specific commit or tag |
| `find_package` with `CMAKE_PREFIX_PATH` pointing to a liteaero-flight install | Production; requires liteaero-flight to have an install/export target |

The mechanism is decided and implemented as the first action of Step 2's sim-side cleanup.
The choice should match the development environment in use at that time. For co-located
development (both repos under `c:/Users/Alex/avraero/liteaero/`), `add_subdirectory` is
the simplest option: it builds liteaero-flight as part of the liteaero-sim build with no
additional infrastructure. If CI or distributed builds are needed, `FetchContent` is
preferred. Whatever mechanism is chosen, liteaero-sim's `CLAUDE.md` must be updated to
document it.

---

## Key Design Decisions

| Decision | Rationale |
| --- | --- |
| `git filter-repo` for history extraction | Preserves full LiteAero Sim development history for every migrated file; a plain fork would carry all terrain, aircraft, and environment history that has no place in a flight-code repository; `git filter-repo` discards that history precisely |
| Per-step duplication closure | Nothing is stable; bugs do not automatically propagate between two copies of the same code; the divergence risk grows with time; see Context section for full rationale |
| Dependency mechanism chosen at Step 2 sim-side cleanup | The choice depends on the development environment and infrastructure available at that time; prescribing it now would be premature |
| No forwarding aliases or backward-compat shims | Project is in initial development; no-backward-compat policy applies |
| `KinematicStateSnapshot` requires a design step | The current `KinematicState` is a rich class with computed properties and serialization; converting it to a plain value struct requires deciding which fields to store vs. derive, and how the control subsystem accesses derived quantities |
| Terrain type split requires a design step | `TerrainTile` currently bundles mesh data with serialization and file I/O; only the data elements migrate to `liteaero::terrain`; the serialization machinery stays in `liteaero::simulation`; a class-level split is needed before file moves begin |
| Shared interface target name is TBD | The CMake target and C++ namespace for `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement structs has not been decided; Step 6 resolves this |
| Tests migrate alongside the code | The liteaero-flight repo includes its own test suite; tests are moved from LiteAero Sim in Phase 1 and deleted from LiteAero Sim at the sim-side cleanup of each step |
| `ControlLoop` and `Control*` elements stay in LiteAero Sim | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, and `ControlLoop` are simulation-internal; they will use `liteaero::control` infrastructure after migration but remain in `liteaero::simulation` |
| Estimation stubs (`NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`) are created in liteaero-flight | Create them in `liteaero-flight` at Step 9, not in LiteAero Sim; any LiteAero Sim stub files for these must be removed at Step 9 |
| ICD ownership follows the component that defines the type | `liteaero-flight` owns the ICDs for its inputs and outputs (`AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, sensor measurement structs, `V_Terrain`, `ILogger`/`LogSource`); these live in `liteaero-flight/docs/interfaces/icds.md`; LiteAero Sim retains only `AtmosphericState` (ICD-2) and `EnvironmentState` (ICD-3); at Step 12 the liteaero-sim ICD entries for flight-owned types are replaced with cross-references |
| Conan for dependency management in both repos | Both repos use Conan 2.x (`conanfile.txt` with `CMakeDeps` + `CMakeToolchain`, profile `liteaero-gcc`); mcap is not in ConanCenter and uses FetchContent pattern 1b in both repos; trochoids and tinygltf are LiteAero Sim-only FetchContent deps that do not appear in `liteaero-flight` |

---

## Phase 0 — Seed the `liteaero-flight` Repository

### Step 0 — Extract History with `git filter-repo`

Clone LiteAero Sim into a new `liteaero-flight` directory and run `git filter-repo` to discard
every commit that does not touch at least one migrating file. The result is a repository
whose entire history is relevant to `liteaero-flight`. The original LiteAero Sim repository is
not modified.

**Prerequisites:** `git filter-repo` must be installed (`pip install git-filter-repo`).

**Procedure:**

```bash
# Clone LiteAero Sim — this clone becomes liteaero-flight
git clone /path/to/liteaero-sim liteaero-flight
cd liteaero-flight

# Remove the upstream remote; the clone is now independent
git remote remove origin

# Run filter-repo using the paths file (see below)
git filter-repo --paths-from-file ../liteaero-flight-migrate-paths.txt
```

**`liteaero-flight-migrate-paths.txt`** — one path per line, relative to repo root. Directories
are matched as prefixes (all files below them are kept). Keep this file alongside the repos
for reference; it is not committed to either repository.

```text
include/ILogger.hpp
include/logger/
src/logger/
test/Logger_test.cpp
include/DynamicElement.hpp
include/SisoElement.hpp
src/DynamicElement.cpp
src/SisoElement.cpp
include/control/Antiwindup.hpp
include/control/ControlLoop.hpp
include/control/Derivative.hpp
include/control/Filter.hpp
include/control/FilterFIR.hpp
include/control/FilterSS.hpp
include/control/FilterSS2.hpp
include/control/FilterSS2Clip.hpp
include/control/FilterTF.hpp
include/control/FilterTF2.hpp
include/control/Gain.hpp
include/control/Integrator.hpp
include/control/Limit.hpp
include/control/LimitBase.hpp
include/control/LimitElement.hpp
include/control/RateLimit.hpp
include/control/RectilinearTable.hpp
include/control/SISOPIDFF.hpp
include/control/TableAxis.hpp
include/control/Unwrap.hpp
include/control/control.hpp
include/control/filter_realizations.hpp
src/control/Antiwindup.cpp
src/control/Derivative.cpp
src/control/FilterFIR.cpp
src/control/FilterSS.cpp
src/control/FilterSS2.cpp
src/control/FilterSS2Clip.cpp
src/control/FilterSS_copy_from_SS2.cpp
src/control/FilterTF.cpp
src/control/FilterTF2.cpp
src/control/Integrator.cpp
src/control/Limit.cpp
src/control/RateLimit.cpp
src/control/SISOPIDFF.cpp
src/control/Unwrap.cpp
src/control/control.cpp
src/control/filter_realizations.cpp
test/Antiwindup_test.cpp
test/Derivative_test.cpp
test/FilterFIR_test.cpp
test/FilterRealizations_test.cpp
test/FilterSS2Clip_test.cpp
test/FilterSS2_test.cpp
test/FilterSS_test.cpp
test/FilterTF2_test.cpp
test/FilterTF_test.cpp
test/Gain_test.cpp
test/Integrator_test.cpp
test/Limit_test.cpp
test/RateLimit_test.cpp
test/RectilinearTable_test.cpp
test/SISOPIDFF_test.cpp
test/TableAxis_test.cpp
test/Unwrap_test.cpp
include/environment/GeodeticAABB.hpp
include/environment/GeodeticPoint.hpp
include/environment/LocalAABB.hpp
include/environment/Terrain.hpp
include/environment/TerrainFacet.hpp
include/environment/TerrainTile.hpp
include/environment/TerrainVertex.hpp
test/Terrain_test.cpp
test/TerrainTile_test.cpp
include/KinematicState.hpp
src/KinematicState.cpp
test/KinematicState_test.cpp
include/navigation/WGS84.hpp
src/navigation/WGS84.cpp
test/WGS84_test.cpp
```

**Notes on path selection:**

- `include/control/Control*.hpp` and `src/control/Control*.cpp`
  (`ControlAltitude`, `ControlHeading`, etc.) are intentionally excluded — they stay in
  LiteAero Sim as `liteaero::simulation` elements.
- `include/KinematicState.hpp` and `src/KinematicState.cpp` are included to preserve
  history for `KinematicStateSnapshot` (Step 7), even though the current class will be
  redesigned rather than moved directly.
- `include/navigation/WGS84.hpp` is included because `KinematicState` depends on it and
  it is a candidate for the shared interface target; include or exclude after resolving
  the `WGS84_Datum` dependency question in Step 6.
- Terrain source files (`src/environment/TerrainTile.cpp` etc.) are excluded until the
  Step 8 design resolves which implementation code, if any, migrates alongside the headers.

**After `git filter-repo` completes:**

The repository contains only the extracted files at their original LiteAero Sim paths. The
commit count will be significantly lower than LiteAero Sim's — only commits that touched at
least one kept path are retained. Verify with `git log --oneline | wc -l` and spot-check
that `git log include/DynamicElement.hpp` shows the expected history.

Add the new remote and push:

```bash
git remote add origin <liteaero-flight-remote-url>
git push -u origin main
```

**Verification:** `git log --oneline` shows a meaningful history. No files exist outside
the paths listed above. The repository does not build yet (no CMakeLists.txt) — that is
expected and addressed in Step 1.

---

## Phase 1 — Build liteaero-flight and Close Duplication Per Step

Each step in Phase 1 has two halves: the liteaero-flight migration and the LiteAero Sim
cleanup. A step is not complete until both halves are done and both repositories' test suites
pass.

### Step 1 — Repository Scaffolding and CMake Skeleton

The `liteaero-flight` repository already exists (seeded in Step 0) with source files at their
original LiteAero Sim paths. This step adds the CMake build system and reorganizes the
extracted files into the `liteaero-flight` directory layout using `git mv`. No namespace changes
yet — those are applied per subsystem in Steps 2–8.

**Directory structure (as delivered):**

```text
liteaero-flight/
  CMakeLists.txt          # root — Conan find_package for all deps; FetchContent for mcap
  conanfile.txt           # eigen/3.4.0, nlohmann_json/3.12.0, gtest/1.14.0, protobuf/3.21.12
  .gitignore
  CLAUDE.md
  include/
    liteaero/
      log/                # ILogger.hpp, Logger.hpp, LogSource.hpp, LogReader.hpp
      control/            # DynamicElement.hpp, SisoElement.hpp, Filter hierarchy, SISO elements
      terrain/            # V_Terrain.hpp, TerrainTile.hpp, TerrainVertex.hpp, …
      nav/                # WGS84.hpp
    KinematicState.hpp    # preserved from filter-repo; redesign deferred to Step 6
  src/
    log/                  # Logger.cpp, LogReader.cpp, mcap_impl.cpp
    control/              # DynamicElement.cpp, SisoElement.cpp, Filter impls, SISO impls
    nav/                  # WGS84.cpp
    KinematicState.cpp    # preserved from filter-repo; redesign deferred to Step 6
  test/
    log/                  # Logger_test.cpp
    control/              # 15 test files
    terrain/              # Terrain_test.cpp, TerrainTile_test.cpp
    nav/                  # WGS84_test.cpp
    KinematicState_test.cpp  # preserved from filter-repo; deferred to Step 6
  proto/
    liteaero_flight.proto # stub — messages added per step
  docs/
    guidelines/           # general.md, cpp.md (Conan-adapted), python.md
    interfaces/
      icds.md             # stub — populated at Step 7
    architecture/
      README.md
    algorithms/
      README.md
    testing/
      strategy.md
    installation/
      README.md
    dependencies/
      README.md
```

**Notes on Step 1 as delivered vs. original spec:**

- `guidance/` and `autopilot/` subdirectories are not created in Step 1; they are created in Step 9 when stub headers migrate.
- CMake targets are `INTERFACE` (not `STATIC`) in Step 1 because no source files carry the new `liteaero::` namespace yet; targets become `STATIC` as each subsystem's namespace migration is applied in Steps 2–8.
- mcap is not in ConanCenter; it uses FetchContent pattern 1b in `CMakeLists.txt`, consistent with LiteAero Sim.
- No `cmake/Dependencies.cmake` file — dependency declarations live directly in the root `CMakeLists.txt` dependency block.

**Verification (as executed):**

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" conan install . --output-folder=build --build=missing --profile=liteaero-gcc
PATH="/c/msys64/ucrt64/bin:$PATH" cmake -B build -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build
```

Result: proto target builds clean. All other targets are INTERFACE stubs (no compiled sources). Test executables are built but fail to compile because test files still use the old include paths — this is the expected known-failure state for Step 1; include paths are updated in Steps 2–8.

**No LiteAero Sim changes in Step 1.** The sim-side dependency mechanism is established in Step 2.

---

### Step 2 — `liteaero::log`: ILogger and Logging Infrastructure

**liteaero-flight half** (Complete)

**Delivered:**

| File | Change |
| --- | --- |
| `include/liteaero/log/ILogger.hpp` | Namespace `liteaerosim` → `liteaero::log` |
| `include/liteaero/log/LogSource.hpp` | Namespace `liteaerosim::logger` → `liteaero::log` |
| `include/liteaero/log/Logger.hpp` | Namespace + include `<liteaero/log/LogSource.hpp>` |
| `include/liteaero/log/LogReader.hpp` | Namespace `liteaerosim::logger` → `liteaero::log` |
| `src/log/Logger.cpp` | Namespace; include `<liteaero/log/Logger.hpp>`; proto header → `liteaero_flight.pb.h`; type → `liteaeroflight::FloatArray`; schema name → `"liteaeroflight.FloatArray"`; profile string → `"liteaeroflight"` |
| `src/log/LogReader.cpp` | Namespace; include `<liteaero/log/LogReader.hpp>`; proto header → `liteaero_flight.pb.h`; type → `liteaeroflight::FloatArray` |
| `src/log/mcap_impl.cpp` | No changes (private implementation TU) |
| `src/log/mcap_static.hpp` | No changes (private header) |
| `test/log/Logger_test.cpp` | Includes → `<liteaero/log/Logger.hpp>`, `<liteaero/log/LogReader.hpp>`; `using namespace liteaero::log` |
| `proto/liteaero_flight.proto` | Added `FloatArray` message (`package liteaeroflight`) |
| `src/CMakeLists.txt` | `liteaero_log` promoted from INTERFACE to STATIC; sources added; links: `nlohmann_json` (PUBLIC), `liteaero_flight_proto`, `protobuf::libprotobuf`, `mcap_headers` (all PRIVATE) |
| `test/log/CMakeLists.txt` | Removed stale step comment |

**Verification:** 6/6 Logger tests pass (`LoggerTest.*`).

**LiteAero Sim half** (Complete)

**Delivered:**

| Change | Detail |
| --- | --- |
| `liteaero-sim/CMakeLists.txt` | Added `add_subdirectory(../liteaero-flight liteaero-flight-build)` (co-located dev mechanism); removed mcap FetchContent block (mcap now comes transitively via `liteaero::log`); updated dependency registry comment |
| `liteaero-sim/src/CMakeLists.txt` | Replaced `mcap_headers` with `liteaero::log` (PUBLIC, so its include path propagates to sim public headers that reference `ILogger`) |
| `include/DynamicElement.hpp` | `#include "ILogger.hpp"` → `#include <liteaero/log/ILogger.hpp>`; all `ILogger*` and `ILogger&` occurrences qualified to `liteaero::log::ILogger` |
| `src/DynamicElement.cpp` | `attachLogger(ILogger*)` → `attachLogger(liteaero::log::ILogger*)` |
| `include/control/FilterSS2.hpp` | `onLog(liteaerosim::ILogger&)` → `onLog(liteaero::log::ILogger&)` |
| `src/control/FilterSS2.cpp` | `onLog(liteaerosim::ILogger&)` → `onLog(liteaero::log::ILogger&)` |
| `include/ILogger.hpp` | Deleted |
| `include/logger/` (Logger.hpp, LogSource.hpp, LogReader.hpp) | Deleted |
| `src/logger/` (Logger.cpp, LogReader.cpp, mcap_impl.cpp, mcap_static.hpp) | Deleted |
| `test/Logger_test.cpp` | Deleted |
| `liteaero-flight/CMakeLists.txt` | Tests only built when liteaero-flight is the top-level project (`CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR`). When consumed via `add_subdirectory`, tests are skipped — each migration step verifies liteaero-flight tests standalone. |
| `liteaero-flight/src/CMakeLists.txt` | `CMAKE_SOURCE_DIR` → `PROJECT_SOURCE_DIR` throughout (required for correct include resolution when liteaero-flight is a subdirectory); `liteaero::control` INTERFACE target declares `liteaero::log` as an INTERFACE dep (because `DynamicElement.hpp` includes `ILogger.hpp`) |

**Verification:** 463/465 LiteAero Sim tests pass (2 FilterTF failures are pre-existing,
unrelated to the logger migration). 6/6 `liteaero_log_test` tests pass in the standalone
liteaero-flight build.

---

### Step 3 — `liteaero::control`: `DynamicElement` and `SisoElement`

Migrate the root abstract lifecycle interfaces.

#### Step 3 — liteaero-flight (Complete)

**Delivered:**

| File | Change |
| --- | --- |
| `include/liteaero/control/DynamicElement.hpp` | `#include "ILogger.hpp"` → `<liteaero/log/ILogger.hpp>`; namespace `liteaerosim` → `liteaero::control`; `ILogger*`/`ILogger&` → `liteaero::log::ILogger*`/`&` |
| `include/liteaero/control/SisoElement.hpp` | `#include "DynamicElement.hpp"` → `<liteaero/control/DynamicElement.hpp>`; namespace → `liteaero::control` |
| `src/control/DynamicElement.cpp` | Include → `<liteaero/control/DynamicElement.hpp>`; namespace → `liteaero::control`; `attachLogger(ILogger*)` → `attachLogger(liteaero::log::ILogger*)` |
| `src/control/SisoElement.cpp` | Include → `<liteaero/control/SisoElement.hpp>`; namespace → `liteaero::control` |
| `src/CMakeLists.txt` | `liteaero_control` promoted from INTERFACE to STATIC; sources: `DynamicElement.cpp`, `SisoElement.cpp`; PUBLIC: `liteaero::log`, `nlohmann_json::nlohmann_json`; PRIVATE: `liteaero_flight_proto`, `protobuf::libprotobuf` |

**Verification:** `mingw32-make -C build liteaero_control` — compiles cleanly, no errors.

#### Step 3 — LiteAero Sim (Complete)

**Delivered:**

| Change | Detail |
| --- | --- |
| `include/control/Filter.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/Derivative.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/Integrator.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/Limit.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/LimitElement.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/RateLimit.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/Unwrap.hpp` | Include → `<liteaero/control/SisoElement.hpp>`; base → `liteaero::control::SisoElement` |
| `include/control/SISOPIDFF.hpp` | Include → `<liteaero/control/DynamicElement.hpp>`; base → `liteaero::control::DynamicElement` |
| `include/propulsion/Propulsion.hpp` | Include → `<liteaero/control/DynamicElement.hpp>`; base → `liteaero::control::DynamicElement` |
| `include/sensor/SensorAirData.hpp` | Include → `<liteaero/control/DynamicElement.hpp>`; base → `liteaero::control::DynamicElement` |
| `src/CMakeLists.txt` | Added `liteaero::control` (PUBLIC) to `liteaerosim` link libraries |
| `test/Filter*_test.cpp` (5 files) | Added `using liteaero::control::SisoElement;` — `SisoElement` was previously reachable via `using namespace liteaerosim` |
| `include/DynamicElement.hpp` | Deleted |
| `src/DynamicElement.cpp` | Deleted |
| `include/SisoElement.hpp` | Deleted |
| `src/SisoElement.cpp` | Deleted |

**Verification:** 463/465 LiteAero Sim tests pass (same 2 pre-existing FilterTF failures).

---

### Step 4 — `liteaero::control`: Filter Hierarchy

Migrate the full discrete-filter infrastructure.

#### Step 4 — liteaero-flight

**Source files to copy and adapt** (namespace `liteaerosim::control` → `liteaero::control`
throughout):

| LiteAero Sim source | liteaero-flight destination |
| --- | --- |
| `include/control/Filter.hpp` | `include/liteaero/control/Filter.hpp` |
| `include/control/FilterSS.hpp` | `include/liteaero/control/FilterSS.hpp` |
| `include/control/FilterSS2.hpp` | `include/liteaero/control/FilterSS2.hpp` |
| `include/control/FilterSS2Clip.hpp` | `include/liteaero/control/FilterSS2Clip.hpp` |
| `include/control/FilterTF.hpp` | `include/liteaero/control/FilterTF.hpp` |
| `include/control/FilterTF2.hpp` | `include/liteaero/control/FilterTF2.hpp` |
| `include/control/FilterFIR.hpp` | `include/liteaero/control/FilterFIR.hpp` |
| `include/control/filter_realizations.hpp` | `include/liteaero/control/filter_realizations.hpp` |
| `src/control/FilterSS.cpp` | `src/control/FilterSS.cpp` |
| `src/control/FilterSS2.cpp` | `src/control/FilterSS2.cpp` |
| `src/control/FilterSS2Clip.cpp` | `src/control/FilterSS2Clip.cpp` |
| `src/control/FilterTF.cpp` | `src/control/FilterTF.cpp` |
| `src/control/FilterTF2.cpp` | `src/control/FilterTF2.cpp` |
| `src/control/FilterFIR.cpp` | `src/control/FilterFIR.cpp` |
| `src/control/filter_realizations.cpp` | `src/control/filter_realizations.cpp` |
| `src/control/FilterSS_copy_from_SS2.cpp` | `src/control/FilterSS_copy_from_SS2.cpp` |
| `test/FilterSS_test.cpp` | `test/control/FilterSS_test.cpp` |
| `test/FilterSS2_test.cpp` | `test/control/FilterSS2_test.cpp` |
| `test/FilterSS2Clip_test.cpp` | `test/control/FilterSS2Clip_test.cpp` |
| `test/FilterTF_test.cpp` | `test/control/FilterTF_test.cpp` |
| `test/FilterTF2_test.cpp` | `test/control/FilterTF2_test.cpp` |
| `test/FilterFIR_test.cpp` | `test/control/FilterFIR_test.cpp` |
| `test/FilterRealizations_test.cpp` | `test/control/FilterRealizations_test.cpp` |

Preserve the GCC vtable fix: `FilterSS.hpp` must not include `FilterSS2.hpp`;
`FilterSS_copy_from_SS2.cpp` is the separate translation unit that includes both.

Move design authority document: copy `docs/algorithms/filters.md` to
`liteaero-flight/docs/algorithms/filters.md`. The LiteAero Sim copy is removed in this
step's sim-side cleanup.

**Verification:** All filter tests pass; `FilterTFTest.FirstOrderLP00` and
`FilterTFTest.SecondOrderLP00` are pre-existing failures and appear unchanged.

#### Step 4 — LiteAero Sim

Remove all filter headers, source files, and test files listed in the liteaero-flight half
above. Update all LiteAero Sim include sites to `<liteaero/control/...>` angle-bracket
includes. Update all `liteaerosim::control::Filter*` references at call sites to
`liteaero::control`. Remove `docs/algorithms/filters.md`; replace with a cross-reference to
`liteaero-flight/docs/algorithms/filters.md`.

**Verification:** All LiteAero Sim tests pass. No filter source files remain in
`liteaero-sim/src/control/` or headers in `liteaero-sim/include/control/` for the migrated
types.

#### Step 4 — Delivered

Both halves verified:

- liteaero-flight: 72/74 tests pass (2 pre-existing `FilterTF` failures unchanged, documented in `docs/testing/strategy.md`)
- liteaero-sim: 397/397 tests pass

**Execution notes:**

- `Limit` and `LimitBase` were migrated in Step 4 (not Step 5) because `FilterSS2Clip` contains `Limit` by value and requires it to compile. Step 5's scope for `Limit` is accordingly removed.
- `numerics.hpp` / `numerics.cpp` were created in liteaero-flight (`namespace liteaero::control`); liteaero-sim retains its own `numerics.hpp` (`namespace liteaerosim`) for sim-internal code.
- Four sim source files (`Aircraft.cpp`, `PropulsionEDF.cpp`, `PropulsionJet.cpp`, `PropulsionProp.cpp`) required `using liteaero::control::Mat21;` because they referenced `Mat21` unqualified and the old sim `control/control.hpp` is no longer in the inclusion chain.
- liteaero-flight `test/control/CMakeLists.txt` was changed from glob to an explicit list of Step 4 tests; Step 5 test stubs remain in the directory but are not yet included in the build.
- liteaero-flight `test/terrain/CMakeLists.txt` and `test/nav/CMakeLists.txt` were changed from glob to empty lists (terrain/nav not yet migrated).
- **Cleanup (addressed post-Step 4):** `control.hpp` was incorrectly including `numerics.hpp` (pulling in Eigen) when it only defines two enums. `Filter.hpp` and `filter_realizations.hpp` were relying on that transitive include rather than declaring their own dependency on `numerics.hpp`. Filter subclass headers also carried redundant direct `control.hpp` and `<Eigen/Dense>` includes. Addressed: `control.hpp` now includes only `<cstdint>`; `Filter.hpp` and `filter_realizations.hpp` include `<liteaero/control/numerics.hpp>` directly; redundant includes removed from `FilterSS2.hpp`, `FilterFIR.hpp`, `FilterSS.hpp`, `FilterTF.hpp`, `FilterTF2.hpp`.

---

### Step 5 — `liteaero::control`: SISO Elements and Scheduling Infrastructure

Migrate integrators, rate limiters, PID, antiwindup, gain scheduling, and related
infrastructure.

#### Step 5 — liteaero-flight

**Source files to copy and adapt:**

| LiteAero Sim source | liteaero-flight destination |
| --- | --- |
| `include/control/Integrator.hpp` | `include/liteaero/control/Integrator.hpp` |
| `include/control/Derivative.hpp` | `include/liteaero/control/Derivative.hpp` |
| `include/control/Limit.hpp` | `include/liteaero/control/Limit.hpp` |
| `include/control/LimitBase.hpp` | `include/liteaero/control/LimitBase.hpp` |
| `include/control/LimitElement.hpp` | `include/liteaero/control/LimitElement.hpp` |
| `include/control/RateLimit.hpp` | `include/liteaero/control/RateLimit.hpp` |
| `include/control/Unwrap.hpp` | `include/liteaero/control/Unwrap.hpp` |
| `include/control/SISOPIDFF.hpp` | `include/liteaero/control/SISOPIDFF.hpp` |
| `include/control/Antiwindup.hpp` | `include/liteaero/control/Antiwindup.hpp` |
| `include/control/Gain.hpp` | `include/liteaero/control/Gain.hpp` |
| `include/control/RectilinearTable.hpp` | `include/liteaero/control/RectilinearTable.hpp` |
| `include/control/TableAxis.hpp` | `include/liteaero/control/TableAxis.hpp` |
| `include/control/control.hpp` | `include/liteaero/control/control.hpp` |
| `src/control/Integrator.cpp` | `src/control/Integrator.cpp` |
| `src/control/Derivative.cpp` | `src/control/Derivative.cpp` |
| `src/control/Limit.cpp` | `src/control/Limit.cpp` |
| `src/control/RateLimit.cpp` | `src/control/RateLimit.cpp` |
| `src/control/Unwrap.cpp` | `src/control/Unwrap.cpp` |
| `src/control/SISOPIDFF.cpp` | `src/control/SISOPIDFF.cpp` |
| `src/control/Antiwindup.cpp` | `src/control/Antiwindup.cpp` |
| `src/control/control.cpp` | `src/control/control.cpp` |
| `test/Integrator_test.cpp` | `test/control/Integrator_test.cpp` |
| `test/Derivative_test.cpp` | `test/control/Derivative_test.cpp` |
| `test/Limit_test.cpp` | `test/control/Limit_test.cpp` |
| `test/RateLimit_test.cpp` | `test/control/RateLimit_test.cpp` |
| `test/Unwrap_test.cpp` | `test/control/Unwrap_test.cpp` |
| `test/SISOPIDFF_test.cpp` | `test/control/SISOPIDFF_test.cpp` |
| `test/Antiwindup_test.cpp` | `test/control/Antiwindup_test.cpp` |
| `test/Gain_test.cpp` | `test/control/Gain_test.cpp` |
| `test/RectilinearTable_test.cpp` | `test/control/RectilinearTable_test.cpp` |
| `test/TableAxis_test.cpp` | `test/control/TableAxis_test.cpp` |

**Verification:** All SISO element and PID tests pass.

#### Step 5 — LiteAero Sim

Remove all SISO headers, source files, and test files listed above. Update all LiteAero Sim
include sites and namespace references. The `liteaero-sim/include/control/` directory should
contain only `Control*.hpp` files (the sim-internal elements that remain) when this step is
done.

**Verification:** All LiteAero Sim tests pass.

#### Step 5 — Delivered

Both halves verified:

- liteaero-flight: 124/126 tests pass (2 pre-existing `FilterTF` failures unchanged)
- liteaero-sim: 345/345 tests pass (count reduction from 397 is expected — 9 test files migrated to liteaero-flight)

**Execution notes:**

- `Limit`, `LimitBase`, and `LimitElement` were already migrated in Step 4; excluded from Step 5 scope.
- `RectilinearTable`, `TableAxis`, and `Gain` have no `.cpp` implementation file in liteaero-sim — header-only; no new sources added to CMakeLists.
- `Unwrap.cpp` and `SISOPIDFF.cpp` used `math/math_util.hpp` (sim-internal, not available in liteaero-flight standalone); replaced with a file-scope `wrapToPi` helper using `<cmath>`.

---

### Step 6 — `KinematicStateSnapshot` Design Document

**Deliverable:** Design authority document at
`liteaero-flight/docs/architecture/kinematic_state_snapshot.md`.

Do not implement in this step. No LiteAero Sim changes.

**Design questions to resolve:**

- **Struct fields:** Which quantities from `KinematicState` are stored vs. derived?
  `KinematicState` currently computes `POM`, `TurnCircle`, `q_ns`, `q_nv`, `q_nl`,
  `crab()`, `crabRate()`, and other derived quantities on demand. The snapshot must decide
  which of these are stored values (computed once per step by the aircraft model) and which
  are removed from the interface. A full call-site audit across all LiteAero Sim files that
  consume `KinematicState` is required before writing this document.
- **Shared interface target name:** Decide the CMake target name and C++ namespace for
  `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement
  structs. Candidates: `liteaero::interfaces`, `liteaero::icd`, `liteaero::types`. Update
  `liteaero-flight/CMakeLists.txt` and `interfaces/` directory name accordingly. Record
  the decision here in this plan's Key Design Decisions table (update the TBD entry).
- **Control subsystem access to derived quantities:** The `ControlAltitude`, `ControlRoll`,
  etc. classes currently accept `const KinematicState&` and call methods like `roll()`,
  `pitch()`, `heading()`. After migration to `liteaero::control`, they will receive
  `KinematicStateSnapshot`. Any derived quantities they need must be present as stored
  fields or computable from the snapshot's plain fields.
- **`WGS84_Datum` dependency:** `KinematicState` includes a `WGS84_Datum` position. Decide
  whether `KinematicStateSnapshot` includes position as a `WGS84_Datum` or as a simpler
  ECEF/NED triple; the `WGS84_Datum` type and `WGS84.hpp` may need to migrate or be
  replicated.

---

### Step 7 — Shared Interface Target: `KinematicStateSnapshot`, `AircraftCommand`, `NavigationState`

Implement the shared interface target as designed in Step 6. Namespace and CMake target
name as decided in Step 6.

#### Step 7 — liteaero-flight

**Files to create** (using the namespace and target name decided in Step 6; `liteaero::ifc`
used as a placeholder below):

| File | Contents |
| --- | --- |
| `include/liteaero/ifc/AircraftCommand.hpp` | Extract from `Aircraft.hpp`; plain value struct |
| `include/liteaero/ifc/KinematicStateSnapshot.hpp` | New plain value struct per Step 6 design |
| `include/liteaero/ifc/NavigationState.hpp` | New plain value struct; at minimum a stub with the fields used by `Autopilot`, `PathGuidance`, `VerticalGuidance`, `ParkTracking`, `WindEstimator`, `FlowAnglesEstimator` |
| `include/liteaero/ifc/AirDataMeasurement.hpp` | Sensor measurement struct for air data (airspeed, altitude, AOA, sideslip) |
| `include/liteaero/ifc/GnssMeasurement.hpp` | Sensor measurement struct stub for GNSS |
| `include/liteaero/ifc/MagMeasurement.hpp` | Sensor measurement struct stub for magnetometer |
| `test/interfaces/Interfaces_test.cpp` | Static-assert plain-struct properties; round-trip layout tests |

All types in this target are plain value structs with no virtual methods, no lifecycle, and
no serialization machinery.

**Write ICD entries:** For each type created above, add a corresponding ICD entry to
`liteaero-flight/docs/interfaces/icds.md`. Each entry documents the producer, consumer(s),
transport, field list with units, and any constraints.

**Verification:**

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build liteaero_ifc_test
PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build -R Interfaces --output-on-failure
```

All static assertion tests pass.

#### Step 7 — LiteAero Sim

Update `Aircraft.hpp` to remove the `AircraftCommand` type definition; replace with an
include of `<liteaero/ifc/AircraftCommand.hpp>`. Update `Aircraft::state()` return type to
`KinematicStateSnapshot` (removing or replacing the `KinematicState` type). Update all
LiteAero Sim call sites. `KinematicState.hpp` and `KinematicState.cpp` are removed after
all call sites are updated; `KinematicState_test.cpp` is removed.

Replace the ICD-1 (`AircraftCommand`) and ICD-4 (`KinematicState`) entries in
`liteaero-sim/docs/architecture/system/present/icds.md` with cross-references to
`liteaero-flight/docs/interfaces/icds.md`.

**Verification:** All LiteAero Sim tests pass. No `KinematicState` references remain.

---

### Step 8 — `liteaero::terrain`: Terrain Element Types and `V_Terrain`

Migrate the shared terrain mesh types and the `V_Terrain` query interface.

**Design note — `TerrainTile` split:** The current `TerrainTile` class bundles mesh data
(`lod_`, list of `TerrainCell*`) with serialization and file I/O. Only the data elements
listed below migrate to `liteaero::terrain`. If `TerrainTile` cannot be cleanly separated,
create a new `TerrainTileData` plain struct in `liteaero::terrain` and keep the full
`TerrainTile` class (with serialization) in `liteaero::simulation`. Update `TerrainMesh`
and `LodSelector` to use the split accordingly. Resolve this before starting file moves.

#### Step 8 — liteaero-flight

| LiteAero Sim source | liteaero-flight destination | Namespace change |
| --- | --- | --- |
| `include/environment/TerrainVertex.hpp` | `include/liteaero/terrain/TerrainVertex.hpp` | `liteaerosim::environment` → `liteaero::terrain` |
| `include/environment/TerrainFacet.hpp` | `include/liteaero/terrain/TerrainFacet.hpp` | `liteaerosim::environment` → `liteaero::terrain` |
| `include/environment/TerrainTile.hpp` (or `TerrainTileData.hpp`) | `include/liteaero/terrain/TerrainTileData.hpp` | See design note |
| `include/environment/GeodeticPoint.hpp` | `include/liteaero/terrain/GeodeticPoint.hpp` | `liteaerosim::environment` → `liteaero::terrain` |
| `include/environment/GeodeticAABB.hpp` | `include/liteaero/terrain/GeodeticAABB.hpp` | `liteaerosim::environment` → `liteaero::terrain` |
| `include/environment/LocalAABB.hpp` | `include/liteaero/terrain/LocalAABB.hpp` | `liteaerosim::environment` → `liteaero::terrain` |
| `include/environment/Terrain.hpp` | `include/liteaero/terrain/V_Terrain.hpp` | `liteaerosim::environment` → `liteaero::terrain`; rename file to `V_Terrain.hpp` to match the class name |

The `liteaero::terrain` target has no source files (all headers are either pure abstractions
or plain structs) and no runtime dependencies. It links Eigen only.

Add a test file `test/terrain/Terrain_test.cpp` covering basic geometric property checks
for each migrated struct type.

**Verification:**

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build liteaero_terrain_test
PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build -R Terrain --output-on-failure
```

#### Step 8 — LiteAero Sim

Remove migrated terrain type headers from `liteaero-sim/include/environment/`. Update all
LiteAero Sim include sites. `TerrainMesh`, `LodSelector`, `FlatTerrain`, `TerrainCell`,
`MeshQualityVerifier`, and terrain file I/O stay in `liteaero::simulation`. Replace the
ICD-6 (`V_Terrain`) entry in `liteaero-sim/docs/architecture/system/present/icds.md` with
a cross-reference to `liteaero-flight/docs/interfaces/icds.md`.

**Verification:** All LiteAero Sim tests pass.

---

### Step 9 — LiteAero Flight Stub Headers

Create stub headers in `liteaero-flight` for all LiteAero Flight elements listed in `flight_code.md`
Current State table, under their target namespaces. Remove any corresponding stubs that
currently exist as untracked files in LiteAero Sim (`include/control/Autopilot.hpp`,
`include/guidance/`, `include/path/`).

**Stubs to create:**

| Target namespace | File | Notes |
| --- | --- | --- |
| `liteaero::autopilot` | `include/liteaero/autopilot/Autopilot.hpp` | Replace `include/control/Autopilot.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/PathGuidance.hpp` | Replace `include/guidance/PathGuidance.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/VerticalGuidance.hpp` | Replace `include/guidance/VerticalGuidance.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/ParkTracking.hpp` | Replace `include/guidance/ParkTracking.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/V_PathSegment.hpp` | Replace `include/path/V_PathSegment.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/PathSegmentHelix.hpp` | Replace `include/path/PathSegmentHelix.hpp` |
| `liteaero::guidance` | `include/liteaero/guidance/Path.hpp` | Replace `include/path/Path.hpp` |
| `liteaero::nav` | `include/liteaero/nav/NavigationFilter.hpp` | Create; design authority in `docs/architecture/navigation_filter.md` |
| `liteaero::nav` | `include/liteaero/nav/WindEstimator.hpp` | Create; design authority in `docs/architecture/wind_estimator.md` |
| `liteaero::nav` | `include/liteaero/nav/FlowAnglesEstimator.hpp` | Create; design authority in `docs/architecture/flow_angles_estimator.md` |

Each stub is a comment-only or minimal class declaration noting the design authority
document and the target namespace. No implementation.

Remove corresponding LiteAero Sim stubs. Update `flight_code.md` Current State table.

**Verification:**

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build
```

liteaero-flight builds cleanly with stubs in place.

---

### Step 10 — liteaero-flight Build and Test Verification

Run the complete liteaero-flight test suite. Confirm zero failures beyond the two known
pre-existing filter failures (`FilterTFTest.FirstOrderLP00`, `FilterTFTest.SecondOrderLP00`).

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build
PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build --output-on-failure
```

Document the test count baseline. This baseline is the regression reference for Phase 2.
No LiteAero Sim changes in this step.

---

## Phase 2 — LiteAero Sim Internal Namespace Migration

By the time Phase 2 begins, all migrated code has been removed from LiteAero Sim step by
step. What remains in LiteAero Sim is simulation-internal code — aircraft dynamics,
sim-internal control elements (`ControlLoop`, `ControlAltitude`, etc.), environment
implementation classes, sensors, propulsion, and aerodynamics. Phase 2 applies the
`liteaero::simulation` namespace to this remaining code and runs a final audit.

### Step 11 — LiteAero Sim Internal Namespace Migration

Apply the `liteaero::simulation` namespace to all LiteAero Sim code that was not migrated
to liteaero-flight.

**Namespace changes:**

| Old namespace | New namespace | Scope |
| --- | --- | --- |
| `liteaerosim` (root) | `liteaero::simulation` | `Aircraft`, `Aircraft6DOF`, `AeroCoeffEstimator`, all aerodynamics, airframe, propulsion, environment, sensor classes |
| `liteaerosim::control` | `liteaero::simulation` | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, `ControlLoop` |
| `liteaerosim::environment` | `liteaero::simulation` | All environment implementation classes remaining after terrain type split |

At this point `liteaerosim::logger` no longer exists in LiteAero Sim (removed in Step 2).
All migrated infrastructure namespaces (`liteaero::log`, `liteaero::control`,
`liteaero::terrain`) are already correct in all files.

Work in a feature branch. Keep at least one intermediate commit per subsystem renamed.

**Verification:** All LiteAero Sim tests pass. No `liteaerosim` references remain.

---

### Step 12 — Final Build and Regression Verification

Run both repositories' test suites and confirm:

1. **liteaero-flight:**

   ```bash
   PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build --output-on-failure
   ```

   Same pass/fail count as Step 10 baseline.

2. **LiteAero Sim:**

   ```bash
   PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build --output-on-failure
   ```

   Same pass count as before Step 11; zero regressions.

3. **Namespace audit:**

   ```bash
   grep -r "liteaerosim" include/ src/
   ```

   Returns no results in either repository.

4. **Dead header audit:** No files in `liteaero-sim/include/` or `liteaero-sim/src/` are
   copies of files now in `liteaero-flight/` (verify no stale duplicates).

5. **ICD cross-reference audit:** `liteaero-sim/docs/architecture/system/present/icds.md`
   contains cross-references (not definitions) for ICD-1, ICD-4, ICD-5, ICD-6, ICD-7.

Update `docs/roadmap/README.md` cross-cutting milestones table: mark liteaero-flight
repository creation ✅ and Repo split and namespace migration ✅.

Update `docs/roadmap/flight_code.md` Current State table: update all stub paths to their
`liteaero-flight` locations.

---

## Risk Notes

1. **`KinematicState` to `KinematicStateSnapshot` redesign (Step 6)** — the most
   significant design risk. The current `KinematicState` has 15+ computed methods called
   by the control subsystem and tests. A full call-site audit before writing the Step 6
   design document is required. Do not skip this; discovering post-implementation that the
   snapshot is missing fields used by the control subsystem requires reopening the Step 7
   implementation.

2. **`TerrainTile` split (Step 8)** — `TerrainTile` bundles tile geometry (which migrates)
   with serialization and LOD selection state (which does not). If the class cannot be split
   cleanly without breaking `TerrainMesh`, `LodSelector`, and `.las_terrain` I/O, introduce
   a `TerrainTileData` plain struct and keep `TerrainTile` in `liteaero::simulation` as a
   thin wrapper.

3. **Shared interface target name (Step 6)** — naming is a prerequisite for Steps 7, 11,
   and 12. Resolve before starting Step 7.

4. **`WGS84_Datum` in `KinematicStateSnapshot`** — `WGS84_Datum` is currently a LiteAero
   Sim type. If `KinematicStateSnapshot` includes it, `WGS84_Datum` must also migrate to
   `liteaero-flight`. If the snapshot uses lat/lon/alt floats instead, the dependency is
   avoided.

5. **liteaero-sim dependency mechanism (Step 2 prerequisite)** — the mechanism for
   liteaero-sim consuming liteaero-flight targets must be established before any code is
   deleted from liteaero-sim. If the chosen mechanism has build-system complications (e.g.,
   `add_subdirectory` conan_toolchain interaction, FetchContent cache issues), resolve them
   before any code deletion proceeds. Deleting code before the replacement build is verified
   is a one-way door.
