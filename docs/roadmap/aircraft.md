# Aircraft Class — Roadmap

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

Items are listed in dependency order. Each item follows TDD: write a failing test before
writing production code.

---

## Current State

| Component | File | Status |
| ----------- | ------ | -------- |
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `WGS84_Datum` | `include/navigation/WGS84.hpp` | ✅ Implemented |
| `AeroPerformance` | `include/aerodynamics/AeroPerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Inertia` | `include/airframe/Inertia.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `V_Propulsion` | `include/propulsion/V_Propulsion.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `V_Motor` | `include/propulsion/V_Motor.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `MotorElectric` | `include/propulsion/MotorElectric.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `MotorPiston` | `include/propulsion/MotorPiston.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `Aircraft` | `include/Aircraft.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Atmosphere` | `include/environment/Atmosphere.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AtmosphericState` | `include/environment/AtmosphericState.hpp` | ✅ Implemented |
| `Wind` | `include/environment/Wind.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Turbulence` | `include/environment/Turbulence.hpp` | ✅ Implemented + serialization (JSON) |
| `Gust` | `include/environment/Gust.hpp` | ✅ Implemented |
| `TurbulenceVelocity` | `include/environment/TurbulenceVelocity.hpp` | ✅ Implemented |
| `EnvironmentState` | `include/environment/EnvironmentState.hpp` | ✅ Implemented |
| `SurfaceGeometry` / `AircraftGeometry` | `include/aerodynamics/AircraftGeometry.hpp` | ✅ Implemented |
| `AeroCoeffEstimator` | `include/aerodynamics/AeroCoeffEstimator.hpp` | ✅ Implemented |

---

## Delivered

Design authority for all delivered items: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

| # | Item | Tests |
| --- | ------ | ------- |
| 1 | `AirframePerformance` — field renames, serialization (JSON + proto), `aircraft_config_v1` schema | `AirframePerformance_test.cpp` — 6 tests |
| 2 | `Inertia` — serialization (JSON + proto), `aircraft_config_v1` schema | `Inertia_test.cpp` — 6 tests |
| 3 | `Aircraft` class — `AircraftCommand`, `initialize()`, `reset()`, `state()` | `Aircraft_test.cpp` — 3 tests |
| 4 | `Aircraft::step()` — 9-step physics loop | `Aircraft_test.cpp` — 3 tests |
| 5 | `Aircraft` serialization — JSON + proto round-trips, schema version checks | `Aircraft_test.cpp` — 4 tests |
| 6 | JSON initialization — fixture-file tests (3 configs) and missing-field error path | `Aircraft_test.cpp` — 4 tests |
| 7 | `Logger` design — architecture, data model, MCAP + CSV formats, C++ interface reference | [`docs/architecture/logger.md`](../architecture/logger.md) |
| 8 | `Logger` implementation — `Logger`, `LogSource`, `LogReader`; MCAP + `FloatArray` proto; 6 tests | `test/Logger_test.cpp` — 6 tests |
| 9 | Environment model design — `Atmosphere` (ISA + ∆ISA + humidity), `Wind`, `Turbulence` (Dryden), `Gust` (1-cosine); rotational turbulence coupling to trim aero model defined | [`docs/architecture/environment.md`](../architecture/environment.md) |
| 10 | Aerodynamic coefficient estimation — derivation of all trim aero model inputs from wing/tail/fuselage geometry; DATCOM lift slope, Hoerner Oswald, Raymer $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$ | [`docs/algorithms/aerodynamics.md`](../algorithms/aerodynamics.md) |
| 11 | `Atmosphere` — ISA 3-layer + ΔT + humidity + density altitude; JSON + proto serialization | `Atmosphere_test.cpp` — 12 tests |
| 12 | `Wind` (Constant/PowerLaw/Log), `Turbulence` (Dryden 6-filter, Tustin-discretized), `Gust` (1-cosine MIL-SPEC-8785C); JSON serialization | `Wind_test.cpp` — 6 tests, `Turbulence_test.cpp` — 5 tests, `Gust_test.cpp` — 6 tests |
| 13 | `AeroCoeffEstimator` — geometry-to-coefficient derivation (Parts 1–8: AR, MAC, $C_{L_\alpha}$, $C_{L_\text{max}}$, Oswald $e$, $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$); `AeroPerformanceConfig` struct; four new `AeroPerformance` fields (`cl_q_nd`, `mac_m`, `cy_r_nd`, `fin_arm_m`); `AircraftGeometry`/`SurfaceGeometry` structs; `AeroPerformance` schema bumped to v2 | `AeroCoeffEstimator_test.cpp` — 11 tests |

---

## 1. `Terrain` — Elevation Model and Multi-Resolution Mesh

Design authority: [`docs/architecture/terrain.md`](../architecture/terrain.md).
Implementation plan: [`docs/roadmap/terrain-implementation-plan.md`](terrain-implementation-plan.md).

This item delivers the full terrain subsystem in two sub-deliverables:

**1a — `V_Terrain` + `FlatTerrain`** (prerequisite for sensors and guidance):

`V_Terrain` is the abstract base defining the elevation query interface used throughout the
Domain Layer.  `FlatTerrain` is the constant-elevation trivial implementation used in unit
tests and flat-earth simulation scenarios.

**1b — `TerrainMesh`** (multi-resolution mesh with LOD, simplification, and game engine integration):

`TerrainMesh` is a concrete `V_Terrain` backed by a `TerrainCell` hash-map keyed on tile
centroid position.  Seven LOD levels span 10 m – 10 km vertex spacing for a regional
area ≤ 100 km.  Tile vertices are stored as float32 ENU offsets from a per-tile
`GeodeticPoint` centroid (local grid encoding) — eliminating geodetic precision issues and
antimeridian discontinuities.  Key capabilities:

- Coordinate transforms: `toECEF()`, `toNED()`
- Queries: `queryLocalAABB()` (metric, vehicle-centered — primary simulation interface),
  `queryGeodeticAABB()` (ingestion), `querySphere()`
- LOD selection with hysteresis: `selectLodBySlantRange()` + stateful `LodSelector`
- Line-of-sight: `lineOfSight()` via Möller–Trumbore ray–triangle intersection
- Quality verification: `MeshQualityVerifier`
- Serialization: JSON + proto + `.las_terrain` binary
- Game engine export: `exportGltf()` → Godot 4 / GLB (MIT, open source, zero cost)
- Live rendering support: `SimulationFrame` value object + `TrajectoryFile` proto

### Sub-deliverable 1a — Interface sketch

```cpp
// include/environment/Terrain.hpp
namespace liteaerosim::environment {

class V_Terrain {
public:
    [[nodiscard]] virtual float elevation_m(double latitude_rad,
                                            double longitude_rad) const = 0;
    [[nodiscard]] float heightAboveGround_m(float  altitude_m,
                                            double latitude_rad,
                                            double longitude_rad) const;
    virtual ~V_Terrain() = default;
};

class FlatTerrain : public V_Terrain {
public:
    explicit FlatTerrain(float elevation_m = 0.f);
    [[nodiscard]] float elevation_m(double latitude_rad,
                                    double longitude_rad) const override;
};

} // namespace liteaerosim::environment
```

### Sub-deliverable 1a — Tests (4 tests, `test/Terrain_test.cpp`)

- `FlatTerrain(300.f)`: `elevation_m()` returns 300 regardless of lat/lon.
- `heightAboveGround_m(500.f, ...)` with `FlatTerrain(300.f)` returns 200.f.
- `heightAboveGround_m(300.f, ...)` returns 0.f (exactly at terrain level).
- `heightAboveGround_m(100.f, ...)` returns 0.f (below terrain — no negative output).

### Sub-deliverable 1a — New files

| File | Action |
| ---- | ------ |
| `include/environment/Terrain.hpp` | ✅ Done |
| `src/environment/Terrain.cpp` | ✅ Done |
| `test/Terrain_test.cpp` | ✅ Done — 4 tests |

### Sub-deliverable 1b — Tests

**Data model** (`test/TerrainTile_test.cpp` — 8 tests):

- Constructor accessors (`lod`, `centroid`, `bounds`, `vertices`, `facets`) match construction args.
- `facetCentroid(0)` for a right triangle returns the ENU-averaged centroid.
- `facetNormal(0)` for a horizontal face has outward Z-component > 0.99.
- JSON round-trip: all vertex ENU values and facet indices preserved ± 1e-4.
- Schema version mismatch throws `std::runtime_error`.
- `TerrainCell::addTile()` + `hasLod()` + `tile()` round-trip.
- `TerrainCell::tile(missing_lod)` throws `std::out_of_range`.
- `finestAvailableLod()` / `coarsestAvailableLod()` correct for mixed population.

**Core + transforms + LOS + serialization** (`test/TerrainMesh_test.cpp` — 26 tests):

- `addCell()` + `cellAt()` round-trip; offset point within extent also finds cell.
- `cellAt()` outside all cells returns `nullptr`.
- `elevation_m()` at known vertex position returns stored height ± 0.01 m.
- `elevation_m()` outside all cells returns 0.f.
- `toECEF()`: vertex at (0°N, 0°E, 0 m) centroid → ECEF x ≈ 6,378,137 m ± 0.1 m.
- `toECEF()`: vertex at (0°N, 90°E, 0 m) centroid → ECEF y ≈ 6,378,137 m ± 0.1 m.
- `toNED()`: vertex at centroid → NED (0, 0, 0); vertex 100 m east → NED (0, 100, 0) ± 0.1 m.
- `queryLocalAABB()`: returns tile within metric AABB; excludes tile 10 km away.
- `queryGeodeticAABB()`: returns tile overlapping bounds; excludes non-overlapping tile.
- `querySphere()`: returns tile within radius; excludes tile entirely outside.
- `max_lod` filtering: only returns tiles at or coarser than requested LOD.

**LOS** (`test/TerrainMesh_test.cpp` — 3 tests, ✅ Done):

- Clear sky above flat terrain → `lineOfSight()` returns true.
- Point below terrain elevation → returns false.
- Ridge mesh between two points → returns false.

**LOD selection** (`test/LodSelector_test.cpp` — 5 tests, ✅ Done):

- `selectLodBySlantRange()`: L0 at r < 300 m; L1 at r = 500 m.
- `LodSelector::select()` first call uses nominal formula.
- No transition when r moves to just above nominal boundary but below hysteresis threshold.
- Transition to coarser when r exceeds `r_b × (1 + δ)`.
- `LodSelector::reset()` clears committed state.

**Quality verification** (`test/MeshQualityVerifier_test.cpp` — 4 tests, ✅ Done):

- Equilateral mesh: `passes()` true, `min_interior_angle_deg` ≈ 60°.
- Zero-area triangle: `degenerate_facet_count > 0`; `passes()` false.
- Single-triangle mesh: `open_boundary_edge_count == 3`.
- Very thin triangle: `max_aspect_ratio > 15`; `passes()` false.

**Serialization** (`test/TerrainMesh_test.cpp` — 5 tests, ✅ Done):

- JSON round-trip: `TerrainMesh` with 2 tiles — tile count and centroid preserved.
- Proto round-trip.
- `.las_terrain` binary: write/read preserves all vertices and facet indices.
- Schema version mismatch → `std::runtime_error`.
- Empty `TerrainMesh` round-trips without error.

**glTF / Godot 4 export** (add to `test/TerrainMesh_test.cpp` — 4 tests):

- `exportGltf()` produces bytes with GLB magic `0x46546C67`.
- JSON chunk contains `"liteaerosim_terrain": true` in root node `extras`.
- `POSITION` accessor count == 3 × facet count (vertex duplication for per-facet color).
- `COLOR_0` accessor count equals `POSITION` count.

**Trajectory / live streaming** (`test/TrajectoryFile_test.cpp` — 2 tests):

- `TrajectoryFile` with 100 frames survives proto round-trip; frame count and timestamps preserved.
- All `TrajectoryFrame` fields survive round-trip within float/double precision.

### Sub-deliverable 1b — New files

| File | Action |
| ---- | ------ |
| `include/environment/GeodeticPoint.hpp` | ✅ Done |
| `include/environment/TerrainVertex.hpp` | ✅ Done — float32 ENU offsets |
| `include/environment/TerrainFacet.hpp` | ✅ Done — `FacetColor` + `TerrainFacet` |
| `include/environment/GeodeticAABB.hpp` | ✅ Done |
| `include/environment/LocalAABB.hpp` | ✅ Done — metric vehicle-centered AABB |
| `include/environment/TerrainTile.hpp` | ✅ Done — `TerrainLod` enum + `TerrainTile` |
| `include/environment/TerrainCell.hpp` | ✅ Done |
| `include/environment/TerrainMesh.hpp` | ✅ Done — Steps 4–10 |
| `include/environment/MeshQualityVerifier.hpp` | ✅ Done — Step 9 |
| `include/environment/LodSelector.hpp` | ✅ Done — Step 7 |
| `include/SimulationFrame.hpp` | Create — Domain Layer value object |
| `src/environment/TerrainTile.cpp` | ✅ Done — Steps 3, 10 (proto serialization) |
| `src/environment/TerrainCell.cpp` | ✅ Done |
| `src/environment/TerrainMesh.cpp` | ✅ Done — Steps 4–10 |
| `src/environment/LodSelector.cpp` | ✅ Done — Step 7 |
| `src/environment/MeshQualityVerifier.cpp` | ✅ Done — Step 9 |
| `test/TerrainTile_test.cpp` | ✅ Done — 8 tests |
| `test/TerrainMesh_test.cpp` | ✅ Done — 26 tests (Steps 4–10); Step 11 tests still to add |
| `test/LodSelector_test.cpp` | ✅ Done — 5 tests |
| `test/MeshQualityVerifier_test.cpp` | ✅ Done — 4 tests |
| `test/TrajectoryFile_test.cpp` | Create — 2 tests |
| `proto/liteaerosim.proto` | ✅ Done (Step 10) — `TerrainTileProto`, `TerrainMeshProto`, `TerrainMeshState` added; `TrajectoryFrame`/`TrajectoryFile` still pending |
| `cmake/Dependencies.cmake` | Modify — add `tinygltf` FetchContent (MIT, header-only) |

---

## 2. Air Data Sensors — `SensorAirData`, `SensorAA`, `SensorAAR`

Air data sensors derive indicated and calibrated quantities from the true atmospheric state
and aircraft kinematics. They model systematic bias and measurement noise. All sensors
consume the `KinematicState` and the `AtmosphericState` output of `Atmosphere`.

### `SensorAirData` — Pitot-Static System

Outputs: indicated airspeed (IAS), calibrated airspeed (CAS), equivalent airspeed (EAS),
true airspeed (TAS), barometric altitude, outside air temperature.

```cpp
// include/sensor/SensorAirData.hpp
struct AirDataMeasurement {
    float ias_mps;         // indicated airspeed (m/s)
    float cas_mps;         // calibrated airspeed (m/s)
    float eas_mps;         // equivalent airspeed (m/s)
    float tas_mps;         // true airspeed (m/s)
    float baro_altitude_m; // barometric altitude (m)
    float oat_k;           // outside air temperature (K)
};
```

### `SensorAA` — Angle-of-Attack Vane / `SensorAAR` — Sideslip Vane

Each outputs a scalar angle in radians. Modeled with a first-order lag and additive bias.

### Tests — `SensorAirData`

- At sea level ISA and `tas = 20 m/s`, IAS ≈ CAS ≈ EAS ≈ TAS to within 0.1 m/s.
- At altitude (3000 m) and same TAS, IAS < TAS.
- `baro_altitude_m` matches geometric altitude to within 10 m at ISA conditions.

### Tests — `SensorAA` / `SensorAAR`

- With zero lag (τ = 0), output equals the true angle immediately.
- With finite lag, output converges toward the true angle with the correct time constant.
- Bias shifts the mean output by the configured amount.

### CMake

Add sensor source files to `liteaerosim`.
Add `test/SensorAirData_test.cpp` and `test/SensorAngle_test.cpp` to the test executable.

---

## 3. `SensorRadAlt` — Radar / Laser Altimeter

`SensorRadAlt` outputs height above ground (HAG) derived from `Terrain::heightAboveGround_m`.
`SensorForwardTerrainProfile` returns a look-ahead terrain elevation vector along the
aircraft's projected track — used by terrain-following guidance.

### Tests — `SensorRadAlt`

- Over `FlatTerrain(0)` at 100 m altitude, output = 100 m (within noise bounds).
- Output is 0 when the aircraft is on the ground.
- When HAG exceeds the sensor's maximum range, output is clamped/saturated to `max_range_m`.

### Tests — `SensorForwardTerrainProfile`

- Over flat terrain, all profile samples equal the terrain elevation at the current position.
- Profile length and sample spacing match configuration parameters.

### CMake

Add `src/sensor/SensorRadAlt.cpp` and `src/sensor/SensorForwardTerrainProfile.cpp` to `liteaerosim`.
Add `test/SensorRadAlt_test.cpp` to the test executable.

---

## 4. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

A `Path` is an ordered sequence of `V_PathSegment` objects. Each segment exposes a
cross-track error, along-track distance, and desired heading at a query position. The
initial concrete segment type is `PathSegmentHelix` (straight line is a degenerate helix
with infinite radius).

### Interface sketch

```cpp
// include/path/V_PathSegment.hpp
namespace liteaerosim::path {

struct PathQuery {
    Eigen::Vector3f position_NED_m;
    float heading_rad;
};

struct PathResponse {
    float crosstrack_error_m;     // positive = right of path
    float along_track_m;          // distance from segment start
    float desired_heading_rad;    // commanded heading at query point
    float desired_altitude_m;
    bool  segment_complete;       // true when along_track_m >= segment_length_m
};

class V_PathSegment {
public:
    virtual PathResponse query(const PathQuery& q) const = 0;
    virtual float length_m() const = 0;
    virtual ~V_PathSegment() = default;
};

} // namespace liteaerosim::path
```

### Tests — `PathSegmentHelix`

- On a straight segment (infinite radius), cross-track error equals the perpendicular
  distance from the line.
- At the midpoint of a circular arc, `desired_heading_rad` is tangent to the arc.
- `segment_complete` is false before the end and true after `along_track_m >= length_m`.

### Tests — `Path`

- A path with two segments advances to the second segment when the first is complete.
- `Path::query()` delegates to the active segment.

### CMake

Add `src/path/PathSegmentHelix.cpp` and `src/path/Path.cpp` to `liteaerosim`.
Add `test/Path_test.cpp` to the test executable.

---

## 5. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

Guidance laws convert path and altitude errors into commanded load factors for `Aircraft::step()`.
They live in the Domain Layer and have no I/O. Each is a stateful element (contains filter
state) and implements `reset()`, `step()`, and JSON + proto serialization.

### `PathGuidance` — Lateral Path Tracking

Implements a nonlinear guidance law (L1 or similar) that commands lateral load factor `n_y`
to null cross-track error.

### `VerticalGuidance` — Altitude / Climb-Rate Hold

Commands normal load factor `n` to track a target altitude or climb rate profile.

### `ParkTracking` — Loiter / Station-Keep

Commands the aircraft to orbit a fixed ground point at a specified radius and altitude.

### Tests — `PathGuidance`

- With zero cross-track error and correct heading, commanded `n_y` is near zero.
- With a large cross-track error, `n_y` is bounded within `[-n_y_max, +n_y_max]`.

### Tests — `VerticalGuidance`

- With aircraft at target altitude, commanded `n` converges to `1.0 g`.
- With aircraft below target altitude, `n > 1.0 g`.

### Tests — Serialization

- JSON and proto round-trips preserve filter state; next `step()` output matches between
  original and restored instances.

### CMake

Add guidance source files to `liteaerosim`.
Add `test/Guidance_test.cpp` to the test executable.

---

## 6. Autopilot — Outer Loop Command Generation

`Autopilot` combines `PathGuidance`, `VerticalGuidance`, and `ParkTracking` into a single
class that consumes the `KinematicState` and `PathResponse` and produces an `AircraftCommand`
for `Aircraft::step()`. It also manages mode selection (path-following vs. loiter vs. manual
override).

### Interface sketch

```cpp
// include/control/Autopilot.hpp
namespace liteaerosim::control {

enum class AutopilotMode { PathFollow, Loiter, ManualOverride };

class Autopilot {
public:
    AircraftCommand step(const KinematicState& state,
                         const path::PathResponse& path_resp,
                         const Eigen::Vector3f& wind_NED_mps,
                         float rho_kgm3,
                         float throttle_nd);

    void setMode(AutopilotMode mode);
    AutopilotMode mode() const;

    void reset();

    nlohmann::json       serializeJson() const;
    void                 deserializeJson(const nlohmann::json& j);
    std::vector<uint8_t> serializeProto() const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);
};

} // namespace liteaerosim::control
```

### Tests

- In `PathFollow` mode with zero cross-track error and target altitude, `step()` produces
  `n ≈ 1.0` and `n_y ≈ 0.0`.
- Switching to `ManualOverride` causes `step()` to pass through the manual command unchanged.
- JSON and proto round-trips preserve inner guidance filter states.

### CMake

Add `src/control/Autopilot.cpp` to `liteaerosim`.
Add `test/Autopilot_test.cpp` to the test executable.

---

## 7. Plot Visualization — Python Post-Processing Tools

Python scripts to load logger output and produce time-series plots for simulation
post-flight analysis. These are Application Layer tools and live under `python/tools/`.

### Deliverables

- `python/tools/plot_flight.py`: CLI script that reads a `.csv` log file produced by
  `Logger` and plots a configurable set of channels vs. time. Outputs a `.png` or
  displays interactively.
- Channel groups: kinematics (position, velocity, attitude), aerodynamics (α, β, CL, CD),
  propulsion (thrust, throttle), guidance (cross-track error, altitude error).

### Tests

- `python/test/test_plot_flight.py`: verifies that `load_log(path)` returns a DataFrame
  with the expected columns and correct dtype; does not test rendering (matplotlib is
  not invoked in the test suite).

### Python dependencies

Add to `python/pyproject.toml`:

```toml
[dependency-groups]
dev = [
    ...,
    "matplotlib>=3.8",
    "pandas>=2.0",
]
```

---

## 8. Manual Input — Joystick and Keyboard

Manual input adapters translate human control inputs (joystick axes, keyboard state) into
an `AircraftCommand`. These live in the Interface Layer and have no physics logic.

### Deliverables

- `KeyboardInput`: maps configurable key bindings to throttle, roll, pitch, yaw increments.
  Operates at the simulation timestep rate.
- `JoystickInput`: reads a raw joystick device (SDL2 or platform API) and maps axes and
  buttons to the `AircraftCommand` fields. Applies a configurable dead zone and axis scaling.

### Interface sketch

```cpp
// include/input/ManualInput.hpp
namespace liteaerosim::input {

class V_ManualInput {
public:
    virtual AircraftCommand read() = 0;   // non-blocking; returns latest command
    virtual ~V_ManualInput() = default;
};

} // namespace liteaerosim::input
```

### Tests

- `KeyboardInput` with no keys pressed returns zero lateral/directional commands and
  configured idle throttle.
- `JoystickInput` axis value at dead-zone boundary maps to zero output.
- Axis scaling: full-deflection axis value maps to the configured maximum command.

### CMake

Add `src/input/KeyboardInput.cpp` and `src/input/JoystickInput.cpp` to `liteaerosim`.
Add a platform-conditional dependency on SDL2 for `JoystickInput`.

---

## 9. Execution Modes — Real-Time, Scaled, and Batch Runners

The simulation runner controls the wall-clock relationship to simulation time. Three modes
are required:

| Mode | Description |
|------|-------------|
| **Real-time** | Each `step()` is paced to its real elapsed wall time (`dt_s` per step). |
| **Scaled real-time** | Same as real-time but with a configurable speed multiplier (0.5×, 2×, etc.). |
| **Full-rate batch** | Steps run as fast as possible; no sleep; used for CI, Monte Carlo, and data generation. |

### Interface sketch

```cpp
// include/runner/SimRunner.hpp
namespace liteaerosim::runner {

enum class ExecutionMode { RealTime, ScaledRealTime, Batch };

struct RunnerConfig {
    ExecutionMode mode         = ExecutionMode::Batch;
    float         time_scale   = 1.0f;   // used only in ScaledRealTime mode
    double        duration_s   = 0.0;    // 0 = run until stop() is called
};

class SimRunner {
public:
    void initialize(const RunnerConfig& config, Aircraft& aircraft);
    void start();    // begins the run loop (blocks in Batch mode; spawns thread otherwise)
    void stop();
    bool is_running() const;
    double elapsed_sim_time_s() const;
};

} // namespace liteaerosim::runner
```

### Tests

- `Batch` mode with `duration_s = 1.0` and `dt_s = 0.01` calls `Aircraft::step()` exactly
  100 times and then stops.
- `RealTime` mode: elapsed wall time after 10 steps is within 10% of `10 * dt_s`.
- `stop()` called from outside (threaded test) terminates the run loop within one timestep.
- `elapsed_sim_time_s()` returns `n_steps * dt_s` after `n_steps` have been executed.

### CMake

Add `src/runner/SimRunner.cpp` to `liteaerosim`.
Add `test/SimRunner_test.cpp` to the test executable.
