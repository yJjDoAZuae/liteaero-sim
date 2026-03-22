# LiteAero Flight Repository Migration — Implementation Plan

## Context

This plan covers FC-1 (liteaero-flight repository creation and repo split), the cross-cutting
milestone described in `docs/roadmap/README.md` and `docs/roadmap/flight_code.md`. It is
a three-phase operation:

- **Phase 0** (Step 0): Seed the `liteaero-flight` repository from LiteAero Sim using
  `git filter-repo`, extracting only the commits that touch migrating files. The resulting
  repository has full LiteAero Sim development history for every file that migrates, with
  all unrelated history discarded.
- **Phase 1** (Steps 1–10): Build out the `liteaero-flight` repository — add CMake scaffolding,
  reorganize paths into the `liteaero-flight` directory layout, apply the `liteaero::` namespace
  changes, add the shared interface target, and relocate LiteAero Flight stubs. LiteAero Sim is
  not touched. Each step produces a buildable, tested addition to `liteaero-flight`.
- **Phase 2** (Steps 11–13): Execute the LiteAero Sim repo split as a single coordinated
  event — update CMake to depend on `liteaero-flight`, apply `liteaero::simulation` namespace to
  all remaining LiteAero Sim code, and remove code that has migrated. The coordinated
  approach is required by the no-backward-compatibility policy and the single-step migration
  decision in `docs/architecture/system/future/decisions.md`.

Namespace decisions are final. All target namespaces are recorded in `decisions.md`.

---

## Key Design Decisions

| Decision | Rationale |
| --- | --- |
| `git filter-repo` for history extraction | Preserves full LiteAero Sim development history for every migrated file; a plain fork would also carry all terrain, aircraft, and environment history that has no place in a flight-code repository; `git filter-repo` discards that history precisely |
| Three-phase approach | Phase 0 seeds the repo from history; Phase 1 builds it out independently; Phase 2 is the single coordinated LiteAero Sim update |
| No forwarding aliases or backward-compat shims | Project is in initial development; no-backward-compat policy applies |
| `KinematicStateSnapshot` requires a design step | The current `KinematicState` is a rich class with computed properties and serialization; converting it to a plain value struct requires deciding which fields to store vs. derive, and how the control subsystem accesses derived quantities |
| Terrain type split requires a design step | `TerrainTile` currently bundles mesh data with serialization and file I/O; only the data elements migrate to `liteaero::terrain`; the serialization machinery stays in `liteaero::simulation`; a class-level split is needed |
| Shared interface target name is TBD | The CMake target and C++ namespace for `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement structs has not been decided; Step 6 resolves this |
| Tests migrate alongside the code | The liteaero-flight repo includes its own test suite; relevant tests are moved from LiteAero Sim in Phase 1 and removed from LiteAero Sim in Phase 2 |
| `ControlLoop` and `Control*` elements stay in LiteAero Sim | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, and `ControlLoop` are simulation-internal; they will use `liteaero::control` infrastructure after Phase 2 but remain in `liteaero::simulation` |
| Estimation stubs (`NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`) are part of Phase 1 | Create them in `liteaero-flight` at Step 9, not in LiteAero Sim; any LiteAero Sim stub files for these must be removed |
| ICD ownership follows the component that defines the type | `liteaero-flight` owns the ICDs for its inputs and outputs (`AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, sensor measurement structs, `V_Terrain`, `ILogger`/`LogSource`); these live in `liteaero-flight/docs/interfaces/icds.md`; LiteAero Sim owns only its internal ICDs (`AtmosphericState`, `EnvironmentState`, and any future sim-internal interfaces); at Step 12 the liteaero-sim ICD entries for flight-owned types are replaced with cross-references to `liteaero-flight/docs/interfaces/icds.md` |

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

## Phase 1 — Build the `liteaero-flight` Repository

### Step 1 — Repository Scaffolding and CMake Skeleton

The `liteaero-flight` repository already exists (seeded in Step 0) with source files at their
original LiteAero Sim paths. This step adds the CMake build system and reorganizes the
extracted files into the `liteaero-flight` directory layout using `git mv`. No namespace changes
yet — those are applied per subsystem in Steps 2–8.

**Directory structure:**

```text
liteaero-flight/
  CMakeLists.txt          # root — defines all targets, fetches dependencies
  cmake/
    Dependencies.cmake    # FetchContent for googletest, nlohmann/json, protobuf, Eigen
  CMakeLists.txt          # root — defines all targets, fetches dependencies
  cmake/
    Dependencies.cmake    # FetchContent for googletest, nlohmann/json, protobuf, Eigen
  include/
    liteaero/
      log/
      control/
      terrain/
      interfaces/         # name TBD — resolved in Step 6
      nav/
      guidance/
      autopilot/
  src/
    log/
    control/
    terrain/
    nav/
    guidance/
    autopilot/
  test/
    log/
    control/
    terrain/
    interfaces/
    nav/
    guidance/
    autopilot/
  proto/
    liteaero_flight.proto
  docs/
    guidelines/           # general.md and cpp.md copied from LiteAero Sim
    interfaces/
      icds.md             # stub — populated at Step 7
    architecture/         # populated per step alongside code
    algorithms/           # populated per step alongside code
    roadmap/              # populated per step alongside code
    testing/
      strategy.md
    installation/
      README.md
    dependencies/
      README.md
```

The root `CMakeLists.txt` defines one `STATIC` library target per subsystem, each with
the matching `liteaero::X` alias, and sets up `FetchContent` for Eigen, nlohmann/json,
protobuf, and googletest (same versions pinned in LiteAero Sim). All targets share the
root `include/` directory. No target links anything yet — dependency edges are added per step.

**Verification:** `cmake -B build && cmake --build build` produces an empty build with no
errors.

---

### Step 2 — `liteaero::log`: ILogger and Logging Infrastructure

Migrate the logging interface and sinks from LiteAero Sim to `liteaero-flight/`.

**Source files to copy and adapt:**

| LiteAero Sim source | liteaero-flight destination | Namespace change |
| --- | --- | --- |
| `include/ILogger.hpp` | `include/liteaero/log/ILogger.hpp` | `liteaerosim` → `liteaero::log` |
| `include/logger/LogSource.hpp` | `include/liteaero/log/LogSource.hpp` | `liteaerosim::logger` → `liteaero::log` |
| `include/logger/Logger.hpp` | `include/liteaero/log/Logger.hpp` | `liteaerosim::logger` → `liteaero::log` |
| `include/logger/LogReader.hpp` | `include/liteaero/log/LogReader.hpp` | `liteaerosim::logger` → `liteaero::log` |
| `src/logger/Logger.cpp` | `src/log/Logger.cpp` | `liteaerosim::logger` → `liteaero::log` |
| `src/logger/LogReader.cpp` | `src/log/LogReader.cpp` | `liteaerosim::logger` → `liteaero::log` |
| `src/logger/mcap_impl.cpp` | `src/log/mcap_impl.cpp` | (no namespace; MCAP implementation) |
| `src/logger/mcap_static.hpp` | `src/log/mcap_static.hpp` | (private header) |
| `test/Logger_test.cpp` | `test/log/Logger_test.cpp` | Update includes and namespace |

The `liteaero::log` CMake target links `nlohmann_json` and the MCAP include path. The
`ILogger` interface has no implementation dependencies.

**Verification:** `cmake --build build && test/log/liteaero_log_test` — all Logger tests
pass.

---

### Step 3 — `liteaero::control`: `DynamicElement` and `SisoElement`

Migrate the root abstract lifecycle interfaces.

**Source files to copy and adapt:**

| LiteAero Sim source | liteaero-flight destination | Namespace change |
| --- | --- | --- |
| `include/DynamicElement.hpp` | `include/liteaero/control/DynamicElement.hpp` | `liteaerosim` → `liteaero::control` |
| `src/DynamicElement.cpp` | `src/control/DynamicElement.cpp` | `liteaerosim` → `liteaero::control` |
| `include/SisoElement.hpp` | `include/liteaero/control/SisoElement.hpp` | `liteaerosim` → `liteaero::control` |
| `src/SisoElement.cpp` | `src/control/SisoElement.cpp` | `liteaerosim` → `liteaero::control` |

The `liteaero::control` target links `liteaero::log` (for `ILogger`), `nlohmann_json`, and
protobuf. No test files added in this step — `DynamicElement` and `SisoElement` are
abstract; tests arrive with their concrete subclasses.

**Verification:** `cmake --build build` — compiles cleanly, no errors.

---

### Step 4 — `liteaero::control`: Filter Hierarchy

Migrate the full discrete-filter infrastructure.

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

**Verification:** `cmake --build build && test/control/liteaero_control_test` — all filter
tests pass; `FilterTFTest.FirstOrderLP00` and `FilterTFTest.SecondOrderLP00` are
pre-existing failures and should appear unchanged.

---

### Step 5 — `liteaero::control`: SISO Elements and Scheduling Infrastructure

Migrate integrators, rate limiters, PID, antiwindup, gain scheduling, and related
infrastructure.

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

**Move design authority document:** Copy `docs/algorithms/filters.md` to
`liteaero-flight/docs/algorithms/filters.md`. Remove from `liteaero-sim` in Phase 2
(Step 12) once the code has migrated.

**Verification:** `cmake --build build && test/control/liteaero_control_test` — all SISO
element and PID tests pass.

---

### Step 6 — `KinematicStateSnapshot` Design Document

**Deliverable:** Design authority document at
`liteaero-flight/docs/architecture/kinematic_state_snapshot.md`.

Do not implement in this step.

**Design questions to resolve:**

- **Struct fields:** Which quantities from `KinematicState` are stored vs. derived?
  `KinematicState` currently computes `POM`, `TurnCircle`, `q_ns`, `q_nv`, `q_nl`,
  `crab()`, `crabRate()`, and other derived quantities on demand. The snapshot must decide
  which of these are stored values (computed once per step by the aircraft model) and which
  are removed from the interface.
- **Shared interface target name:** Decide the CMake target name and C++ namespace for
  `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement
  structs. Candidates: `liteaero::interfaces`, `liteaero::icd`, `liteaero::types`. Update
  `liteaero-flight/CMakeLists.txt` and `interfaces/` directory name accordingly. Record the
  decision in `docs/architecture/system/future/decisions.md`.
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
no serialization machinery (per the `decisions.md` module communication interface decision).

**Write ICD entries:** For each type created above, add a corresponding ICD entry to
`liteaero-flight/docs/interfaces/icds.md`. Each entry documents the producer, consumer(s),
transport, field list with units, and any constraints. These ICDs are the design authority
for the interface — `liteaero-flight` owns them because it defines the types. The existing
ICD-1 (`AircraftCommand`) and ICD-4 (`KinematicState`) entries in
`liteaero-sim/docs/architecture/system/present/icds.md` are not yet modified; they will be
replaced with cross-references at Step 12.

**Verification:** `cmake --build build && test/interfaces/liteaero_ifc_test` — all static
assertion tests pass.

---

### Step 8 — `liteaero::terrain`: Terrain Element Types and `V_Terrain`

Migrate the shared terrain mesh types and the `V_Terrain` query interface.

**Design note — `TerrainTile` split:** The current `TerrainTile` class bundles mesh data
(`lod_`, list of `TerrainCell*`) with serialization and file I/O. Only the data elements
listed in `decisions.md` migrate to `liteaero::terrain`. If `TerrainTile` cannot be cleanly
separated, create a new `TerrainTileData` plain struct in `liteaero::terrain` and keep the
full `TerrainTile` class (with serialization) in `liteaero::simulation`. Update `TerrainMesh`
and `LodSelector` to use the split accordingly. Resolve this before starting file moves.

**Files to create in `liteaero-flight/`:**

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

**Tests:** Add a test file `test/terrain/Terrain_test.cpp` covering basic geometric
property checks for each migrated struct type; copy relevant assertions from
`test/Terrain_test.cpp`.

**Verification:** `cmake --build build && test/terrain/liteaero_terrain_test` — all struct
tests pass.

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

**Update `flight_code.md` Current State table:** Replace the LiteAero Sim stub paths with
the new `liteaero-flight` paths. Mark the LiteAero Sim entries as removed.

**Verification:** `cmake --build build` — liteaero-flight builds cleanly with stubs in place.

---

### Step 10 — liteaero-flight Build and Test Verification

Run the complete liteaero-flight test suite. Confirm zero failures beyond the two known
pre-existing filter failures.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Document the test count baseline (expected: all tests from Steps 2–8 passing). This
baseline is the regression reference for Phase 2.

---

## Phase 2 — LiteAero Sim Repo Split

### Step 11 — LiteAero Sim CMake: Dependency on `liteaero-flight`

Update `liteaero-sim/CMakeLists.txt` and `liteaero-sim/cmake/Dependencies.cmake` to add
`liteaero-flight` as a versioned `FetchContent` or `find_package` dependency. Add `liteaero::log`,
`liteaero::control`, `liteaero::terrain`, and the shared interface target to the
`liteaero-sim` library's `target_link_libraries`. Do not yet remove any LiteAero Sim source
files — this step establishes that both copies coexist and the build does not break before
the namespace changes are applied.

**Verification:** `cmake --build build` — LiteAero Sim builds with liteaero-flight as a
dependency; all existing LiteAero Sim tests still pass.

---

### Step 12 — LiteAero Sim Namespace Migration and Code Removal

Execute the full namespace migration as a single coordinated change:

**Namespace changes applied across all LiteAero Sim source files:**

| Old namespace | New namespace | Scope |
| --- | --- | --- |
| `liteaerosim` (root) | `liteaero::simulation` | `Aircraft`, `KinematicState`, `AeroCoeffEstimator`, all aerodynamics, airframe, propulsion, environment, sensor classes |
| `liteaerosim::control` | `liteaero::simulation::control` | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, `ControlLoop` |
| `liteaerosim::environment` | `liteaero::simulation` | All environment implementation classes that remain in LiteAero Sim after terrain type split |
| `liteaerosim::logger` | `liteaero::log` | Logger and sinks now redirect to liteaero-flight's `liteaero::log` |

**Infrastructure headers replaced by liteaero-flight equivalents** (includes updated at all call
sites):

| Remove from LiteAero Sim | Replace with |
| --- | --- |
| `include/DynamicElement.hpp` | `liteaero/control/DynamicElement.hpp` from `liteaero::control` |
| `include/SisoElement.hpp` | `liteaero/control/SisoElement.hpp` from `liteaero::control` |
| `include/ILogger.hpp` | `liteaero/log/ILogger.hpp` from `liteaero::log` |
| `include/logger/Logger.hpp` | `liteaero/log/Logger.hpp` from `liteaero::log` |
| `include/logger/LogReader.hpp` | `liteaero/log/LogReader.hpp` from `liteaero::log` |
| `include/logger/LogSource.hpp` | `liteaero/log/LogSource.hpp` from `liteaero::log` |
| `include/control/Filter.hpp` and filter hierarchy | `liteaero/control/Filter.hpp` etc. from `liteaero::control` |
| `include/control/Integrator.hpp` etc. | Corresponding `liteaero::control` headers |
| `include/control/SISOPIDFF.hpp` | `liteaero/control/SISOPIDFF.hpp` from `liteaero::control` |
| `include/environment/TerrainVertex.hpp` etc. (migrated terrain element types) | Corresponding `liteaero::terrain` headers |
| `include/environment/Terrain.hpp` (`V_Terrain`) | `liteaero/terrain/V_Terrain.hpp` from `liteaero::terrain` |

**LiteAero Sim stub headers removed** (LiteAero Flight stubs now live in `liteaero-flight`):

- `include/control/Autopilot.hpp`
- `include/guidance/PathGuidance.hpp`, `VerticalGuidance.hpp`, `ParkTracking.hpp`
- `include/path/Path.hpp`, `PathSegmentHelix.hpp`, `V_PathSegment.hpp`

**`KinematicState` → `KinematicStateSnapshot`:** All LiteAero Sim call sites that use
`KinematicState` are updated to use `KinematicStateSnapshot` from the shared interface
target. The `Aircraft::state()` return type changes to `KinematicStateSnapshot`.
`KinematicState.hpp` and `KinematicState.cpp` are removed from LiteAero Sim after migration.
`KinematicState_test.cpp` is removed; tests for the snapshot struct already exist in the
liteaero-flight interfaces test (Step 7).

**`AircraftCommand`:** All LiteAero Sim references to `liteaerosim::AircraftCommand` are
updated to the shared interface target's `AircraftCommand`. The definition is removed from
`Aircraft.hpp`.

**ICD cross-references — replace flight-owned ICD entries in LiteAero Sim:** In
`liteaero-sim/docs/architecture/system/present/icds.md`, replace the entries for ICD-1
(`AircraftCommand`), ICD-4 (`KinematicState`/`KinematicStateSnapshot`), ICD-5
(`AirDataMeasurement`), ICD-6 (`V_Terrain`), and ICD-7 (`ILogger`/`LogSource`) with a
one-line cross-reference to `liteaero-flight/docs/interfaces/icds.md`. LiteAero Sim retains
only ICD-2 (`AtmosphericState`) and ICD-3 (`EnvironmentState`), which are
simulation-internal and never cross to flight code.

**Verification:** `cmake --build build && build/test/liteaerosim_test.exe` — all LiteAero Sim
tests pass. liteaero-flight tests are unchanged.

---

### Step 13 — Final Build and Regression Verification

Run both repositories' test suites in sequence and confirm:

1. **liteaero-flight:** All tests pass (same baseline as Step 10).
2. **LiteAero Sim:** All tests pass (same count as before Step 11; zero regressions).
3. **Namespace audit:** `grep -r "liteaerosim" include/ src/` returns no results in either
   repository (any remaining occurrences indicate an incomplete migration).
4. **Dead header audit:** No files in `liteaero-sim/include/` or `liteaero-sim/src/` are
   copies of files now in `liteaero-flight/` (verify no stale duplicates).

Update `docs/roadmap/README.md` cross-cutting milestones table: mark `liteaero-flight`
repository creation ✅ and Repo split and namespace migration ✅.

Update `docs/roadmap/flight_code.md` Current State table: update all stub paths to their
`liteaero-flight` locations.

---

## Risk Notes

1. **`KinematicState` to `KinematicStateSnapshot` redesign (Step 6)** — the most
   significant design risk. The current `KinematicState` has 15+ computed methods that are
   called by the control subsystem and tests. Determining which are stored vs. derived in the
   snapshot requires understanding all call sites. A full call-site audit before writing the
   Step 6 design document is recommended.

2. **`TerrainTile` split (Step 8)** — `TerrainTile` bundles tile geometry (which migrates)
   with serialization and LOD selection state (which does not). If the class cannot be split
   cleanly without breaking `TerrainMesh`, `LodSelector`, and `.las_terrain` I/O, introduce
   a `TerrainTileData` plain struct and keep `TerrainTile` in `liteaero::simulation` as a
   thin wrapper.

3. **Shared interface target name (Step 6)** — naming is a prerequisite for Steps 7, 11,
   and 12. Resolve before starting Step 7.

4. **`WGS84_Datum` in `KinematicStateSnapshot`** — `WGS84_Datum` is currently a LiteAero Sim
   type. If `KinematicStateSnapshot` includes it, `WGS84_Datum` must also migrate to
   `liteaero-flight`. If the snapshot uses lat/lon/alt floats instead, the dependency is avoided.

5. **Phase 2 is a large single changeset** — Step 12 touches every source file in
   LiteAero Sim. Work in a feature branch; keep CI green throughout the branch with at least
   one intermediate commit per subsystem updated.
