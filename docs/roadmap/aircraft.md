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
| `Propulsion` | `include/propulsion/Propulsion.hpp` | ✅ Implemented — derives from `DynamicElement`; replaces `V_Propulsion` |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `Motor` | `include/propulsion/Motor.hpp` | ✅ Implemented — stateless abstract interface; replaces `V_Motor` |
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
| `DynamicElement` | `include/DynamicElement.hpp` | ✅ Implemented — see [dynamic_element.md](../architecture/dynamic_element.md) |
| `SisoElement` | `include/SisoElement.hpp` | ✅ Implemented — NVI SISO wrapper over `DynamicElement` |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | 🔲 Stub only |
| `SensorGnss` | `include/sensor/SensorGnss.hpp` | 🔲 Stub only |
| `SensorLaserAlt` | `include/sensor/SensorLaserAlt.hpp` | 🔲 Stub only |
| `SensorMag` | `include/sensor/SensorMag.hpp` | 🔲 Stub only |
| `SensorInsSimulation` | `include/sensor/SensorInsSimulation.hpp` | 🔲 Stub only |
| `SensorAA` | `include/sensor/SensorAA.hpp` | 🔲 Stub only |
| `SensorAAR` | `include/sensor/SensorAAR.hpp` | 🔲 Stub only |
| `SensorRadAlt` | `include/sensor/SensorRadAlt.hpp` | 🔲 Stub only |
| `SensorForwardTerrainProfile` | `include/sensor/SensorForwardTerrainProfile.hpp` | 🔲 Stub only |
| `SensorTrackEstimator` | `include/sensor/SensorTrackEstimator.hpp` | 🔲 Stub only |
| `NavigationFilter` | `include/estimation/NavigationFilter.hpp` | 🔲 Stub only — see [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `include/estimation/WindEstimator.hpp` | 🔲 Stub only — see [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `include/estimation/FlowAnglesEstimator.hpp` | 🔲 Stub only — see [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |
| `V_PathSegment` | `include/path/V_PathSegment.hpp` | 🔲 Stub only |
| `PathSegmentHelix` | `include/path/PathSegmentHelix.hpp` | 🔲 Stub only |
| `Path` | `include/path/Path.hpp` | 🔲 Stub only |
| `PathGuidance` | `include/guidance/PathGuidance.hpp` | 🔲 Stub only |
| `VerticalGuidance` | `include/guidance/VerticalGuidance.hpp` | 🔲 Stub only |
| `ParkTracking` | `include/guidance/ParkTracking.hpp` | 🔲 Stub only |
| `Autopilot` | `include/control/Autopilot.hpp` | 🔲 Stub only |

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
| 14 | `Terrain` subsystem — `V_Terrain`, `FlatTerrain`, `TerrainMesh` (7 LODs, LOS, glTF export), `LodSelector`, `MeshQualityVerifier`, `SimulationFrame`/`TrajectoryFile`; Python ingestion pipeline (download, mosaic, geoid correction, triangulate, colorize, simplify, verify, export) | C++: 53 tests across 6 test files; Python: 28 tests pass + 1 skip — see [`terrain-implementation-plan.md`](terrain-implementation-plan.md) |
| 15 | `DynamicElement` refactoring — unified root base for all stateful components; `SisoElement` NVI SISO wrapper; `Filter` → `SisoElement`; `Propulsion` / `Motor` bases; deleted `DynamicBlock`, `DynamicFilterBlock`, `DynamicLimitBlock`, `V_Sensor`, `V_Propulsion`, `V_Motor` | No new tests — all 378 pre-existing tests pass |
| 16 | `SISOBlock` removal — deleted `SISOBlock.hpp`; `SisoElement` derives from `DynamicElement` only; `LimitBase`/`Limit`/`RateLimit`/`Integrator`/`Derivative`/`Unwrap` migrated to `SisoElement` with full `DynamicElement` lifecycle; `SisoElement::step()` NVI call order fixed (previous `in_` available during `onStep()`); `resetTo()` replaces `reset(float)` overloads | `Limit_test.cpp`, `RateLimit_test.cpp`, `Integrator_test.cpp`, `Derivative_test.cpp`, `Unwrap_test.cpp` — 10 new tests; all 394 tests pass |
| 17 | `Antiwindup` redesign — `AntiwindupConfig` struct with `enum class Direction`; `update(float)` replaces `operator=(float)`; `configure()`, `reset()`, `serializeJson()`/`deserializeJson()` added; `name` field removed; uninitialized-boolean bug fixed; `Integrator` serialization extended to embed `"antiwindup"` array | `Antiwindup_test.cpp` — 12 tests; `Integrator_test.cpp` — 2 new tests; all 408 tests pass |

---

## 1. Control Subsystem Refactoring

Design authority: [`docs/architecture/control_interface_review.md`](../architecture/control_interface_review.md).

Nine interface design issues are identified and ordered by dependency. Implement in the
sequence below; each step is independently buildable and testable.

### Step A — `enum class` Promotion (Issue 8)

Convert `FilterError` and `DiscretizationMethod` in `control.hpp` from C-style unscoped
enums to `enum class`. Rename enumerators to drop abbreviations and `SCREAMING_CASE`:
`FwdEuler` → `ForwardEuler`, `BackEuler` → `BackwardEuler`, `PZMatch` → `PoleZeroMatch`,
`NONE` → `None`, `INVALID_DIMENSION` → `InvalidDimension`, etc.

Update all call sites that reference the old enumerator names (serialization casts,
switch cases, test comparisons).

**Tests:** existing tests updated to use new names; no new tests required.

### Step B — `Limit` / `RateLimit` Hierarchy Fix (Issues 5, 9)

With `RateLimit` removed from `LimitBase`, `LimitBase` would have exactly one concrete
subclass (`Limit`). A single-subclass abstract intermediate class is unnecessary indirection
and there is no current caller that holds a `LimitBase*`. Delete `LimitBase.hpp`; derive
`Limit` directly from `SisoElement`.

- Delete `include/control/LimitBase.hpp`.
- `Limit` derives from `SisoElement` directly; its `setLower`/`setUpper`/`enable`/`disable`
  interface is unchanged.
- `RateLimit` derives from `SisoElement` directly.
- Apply `snake_case_` member renames from Issue 9 to both classes during this pass:
  `_lowerLimit` → `lower_limit_`, `_dt` → `dt_s_`, `_enableLowerLimit` → `lower_enabled_`, etc.

**Tests:** existing `Limit_test.cpp` and `RateLimit_test.cpp` tests pass unchanged.

### Step C — `Gain` API Cleanup (Issue 4 partial)

Gain scheduling is a planned feature; the template structure is retained. This step
removes only the misleading unimplemented stubs and fixes API inconsistencies:

- Remove stub methods (`schedule()`, `readJSON()`, `readFile()`) and the unpopulated
  `RectilinearTable` member.
- `operator=(T K)` → `set(T value)`
- `operator double()` → `operator float()`
- `K()` → `value()`
- `_K` → `value_` (private)

Template parameters `T` and `NumAxes` are retained unchanged. The full scheduling
redesign is a separate roadmap item (see item 3 — Gain Scheduling Design).

Update `SISOPIDFF` and all call sites to use `set()` and `value()`.

**Tests:** existing tests pass unchanged; no new tests required.

### Step D — `FilterSS2Clip` NVI Migration (Issues 1, 2, 9)

Highest-priority concrete filter migration. This is the critical-path prerequisite for
Steps G and I.

- Remove `_in`/`_out` shadow members; use `in_`/`out_` from `SisoElement`.
- Remove shadow `in()`, `out()`, `operator float()` methods.
- Rename `step()` → `onStep()`; implement `onInitialize()`, `onSerializeJson()`,
  `onDeserializeJson()`, `schemaVersion()`, `typeName()`.
- Apply `snake_case_` member renames (Issue 9a): `_Phi` → `phi_`, `_dt` → `dt_s_`, etc.
- Fix `backsolve` → `backSolve` (Issue 9d); remove dead `delU` variable (Issue 9f);
  promote `dcTol` → `constexpr DC_TOLERANCE` (Issue 9f); fix `copy()` parameter to
  `const FilterSS2Clip&` (Issue 9f).

**Tests:** new `FilterSS2Clip_test.cpp` — `step()` through base pointer returns correct
value; `serializeJson()` through base pointer returns non-empty object; JSON round-trip
preserves filter state; `schemaVersion()` and `typeName()` return correct values.

### Step E — `FilterTF2` NVI Migration (Issues 1, 2, 9)

Same migration pattern as Step D applied to `FilterTF2`.

**Tests:** same categories as Step D for `FilterTF2`.

### Step F — `FilterTF`, `FilterFIR`, `FilterSS` — Evaluate and Migrate (Issues 1, 2, 9)

Assess whether `FilterTF` and `FilterTF2` can be unified with `FilterSS2` (they represent
the same mathematical object — a rational transfer function — with different internal
representations). Migrate or consolidate each. `FilterSS` is likely superseded by
`FilterSS2`; evaluate for removal.

**Tests:** as per Step D pattern for each surviving class.

### Step G — Remove `Filter` No-Op Defaults (Issue 2)

After Steps D–F, remove the no-op `onInitialize`, `onSerializeJson`, `onDeserializeJson`,
`schemaVersion`, `typeName`, and `onStep` defaults from `Filter`. Any filter that did not
complete migration will produce a compile error, making incomplete migration visible.

**Tests:** build succeeds with no filters falling back to base defaults.

### Step H — `Unwrap` Reference Field (Issue 6)

Replace `step(float u, float ref)` two-arg overload with a stored `ref_` field set via
`setReference(float)`. Route all steps through the NVI chain. Include `ref_` in
serialization.

**Tests:** new tests — `setReference()` affects output; `onLog()` is called on every step
(verified via a mock or logged output count); JSON round-trip preserves `ref_`.

### Step I — `SISOPIDFF` → `DynamicElement` Lifecycle (Issues 3, 7, 9e)

Make `SISOPIDFF` derive from `DynamicElement`. Implement `onInitialize()`,
`onReset()`, `onSerializeJson()`, `onDeserializeJson()`, `schemaVersion()`, `typeName()`.

Apply naming fixes from Issue 9e: `Kp` → `proportional_gain_`, `I` → `integrator_`,
`unwrapInputs` → `unwrap_inputs_`, etc.

Apply `Integrator`/`Derivative` member renames from Issue 9a: `_dt` → `dt_s_`,
`_Tau` → `tau_s_`, JSON key `"tau"` → `"tau_s"`, etc.

Fold limit visibility fix (Issue 7): make `valLimit`/`rateLimit` on `FilterSS2Clip` and
`limit` on `Integrator`/`Derivative` private; expose configuration methods.

**Tests:** `SISOPIDFF_test.cpp` — `initialize()` + `step()` + `serializeJson()` +
`deserializeJson()` lifecycle; JSON round-trip preserves filter state and next-step output
matches; schema version check throws on mismatch.

---

## 2. `SensorAirData` — Air Data System

Stub header exists at `include/sensor/SensorAirData.hpp`. `liteaerosim::DynamicElement` is the
abstract base (see [dynamic_element.md](../architecture/dynamic_element.md)).

`SensorAirData` models the pitot-static system at the physical transducer level: a
differential pressure sensor (dynamic pressure $q_c$) and an absolute pressure sensor
(static pressure $P_s$). All derived quantities — IAS, CAS, EAS, TAS, barometric altitude,
OAT — are computed from these two raw measurements and the `AtmosphericState`.

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

### Tests — `SensorAirData`

- At sea level ISA and `tas = 20 m/s`, IAS ≈ CAS ≈ EAS ≈ TAS to within 0.1 m/s.
- At altitude (3000 m) and same TAS, IAS < TAS.
- `baro_altitude_m` matches geometric altitude to within 10 m at ISA conditions.
- With additive differential-pressure noise σ, IAS noise propagates correctly to output.
- With additive static-pressure noise σ, barometric altitude noise propagates correctly.
- JSON and proto round-trips preserve sensor state (noise seeds, lag filter state).

### CMake

Add `src/sensor/SensorAirData.cpp` to `liteaerosim`.
Add `test/SensorAirData_test.cpp` to the test executable.

---

## 3. Gain Scheduling — Design and Implementation

`Gain<T, NumAxes>` currently holds template parameters for value type and scheduling
axis count, but the scheduling logic is unimplemented (stubs removed in roadmap item 1,
Step C). This item defines and implements the full gain scheduling architecture.

### Scope to Define

The design must address at minimum:

- **Lookup methods** — what interpolation strategies are supported (e.g. rectilinear
  table bilinear, nearest-neighbor, polynomial fit) and how they are selected.
- **Axis dimensions** — how `NumAxes` maps to physical scheduling variables
  (e.g. airspeed, altitude, angle of attack); how axes are labeled and units enforced.
- **Runtime update** — how a scheduled gain is evaluated at each step given the current
  scheduling variable values; whether evaluation is synchronous with `SISOPIDFF::step()`
  or driven externally.
- **Serialization** — how the gain table is stored and restored (JSON + proto).
- **Initialization** — whether the gain table is loaded from a file, embedded in config
  JSON, or populated programmatically.

### Deliverables

Design authority document at `docs/architecture/gain_scheduling.md` to be written
before implementation begins.

Implementation follows TDD: failing tests before production code.

---

## 4. Autopilot Gain Design — Python Tooling

Python workflow that derives autopilot control gains from the aircraft model. This is a
prerequisite for item 4 (`Autopilot`) — the C++ implementation is parameterized by gains
computed here.

Scope to be defined when this item is scheduled. Expected to use Python control-system
libraries (e.g. `python-control`, `scipy.signal`) applied to linearized models extracted
from `Aircraft` trim and `AeroCoeffEstimator` outputs.

---

## 4. Autopilot — Inner Loop Knobs-Mode Tracking

Stub header exists at `include/control/Autopilot.hpp`.

`Autopilot` implements the inner closed-loop layer. It tracks pilot-style set point commands
corresponding to "knobs" modes — altitude hold, vertical speed hold, heading hold, and roll
attitude hold — and produces an `AircraftCommand` for `Aircraft::step()`. Guidance (item 6)
is the outer loop that wraps around it and supplies the set point commands. Control gains are
derived by the Python gain design workflow (item 3).

### Interface sketch

_To be defined during design. Inputs will include `KinematicState`, `AtmosphericState`,
and a set point struct (target altitude, target vertical speed, target heading, target roll
attitude). Output is `AircraftCommand`._

### Tests

- Altitude hold: starting from a displaced altitude, output drives altitude error to zero
  within the expected settling time.
- Heading hold: starting from a heading offset, output drives heading error to zero without
  overshoot beyond a specified bound.
- Roll attitude hold: commanded roll angle is tracked with correct steady-state and transient.
- Vertical speed hold: commanded climb rate is tracked correctly.
- JSON and proto round-trips preserve filter states.

### CMake

Add `src/control/Autopilot.cpp` to `liteaerosim`.
Add `test/Autopilot_test.cpp` to the test executable.

---

## 5. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

Stub headers exist in `include/path/` for all three classes.

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

## 6. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

Stub headers exist in `include/guidance/` for all three classes.

Guidance is the outer loop that wraps around the `Autopilot`. It converts path and altitude
errors into set point commands (target altitude, vertical speed, heading, roll attitude) that
are fed to `Autopilot::step()`. Guidance classes live in the Domain Layer and have no I/O.
Each is a stateful element (contains filter state) and implements `reset()`, `step()`, and
JSON + proto serialization.

### `PathGuidance` — Lateral Path Tracking

Implements a nonlinear guidance law (L1 or similar) that commands a target heading or roll
attitude set point to null cross-track error against a path segment.

### `VerticalGuidance` — Altitude / Climb-Rate Hold

Commands target altitude or vertical speed set points to track a target altitude or climb
rate profile.

### `ParkTracking` — Loiter / Station-Keep

Commands the aircraft to orbit a fixed ground point at a specified radius and altitude by
producing heading and altitude set point commands to `Autopilot`.

### Tests — `PathGuidance`

- With zero cross-track error and correct heading, commanded heading set point matches
  current heading (no corrective input).
- With a large cross-track error, the commanded heading correction is bounded within a
  specified maximum bank angle equivalent.

### Tests — `VerticalGuidance`

- With aircraft at target altitude, commanded altitude set point matches current altitude
  (no corrective input).
- With aircraft below target altitude, commanded vertical speed set point is positive.

### Tests — Serialization

- JSON and proto round-trips preserve filter state; next `step()` output matches between
  original and restored instances.

### CMake

Add guidance source files to `liteaerosim`.
Add `test/Guidance_test.cpp` to the test executable.

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

---

## 10. Remaining Sensor Models

Not blocking any higher-priority item. Stub headers exist in `include/sensor/`.
Implement when needed; order within this group follows dependency.

| Class | Depends on | Hardware modeled |
|---|---|---|
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | Terrain (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | Terrain (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |
| `SensorInsSimulation` | `KinematicState` (done), `NavigationFilter` types | INS simulation replacement — truth-plus-error model producing `InsMeasurement` |
| `SensorForwardTerrainProfile` | Terrain (done), Guidance | Forward terrain profiling sensor (multi-beam LIDAR / line-scan radar) for terrain-following guidance |
| `SensorAA` | `KinematicState` (done) | Passive angle/angle sensor (imaging type) — measures two LOS angles, no range |
| `SensorAAR` | `KinematicState` (done) | Active angle/angle/range sensor (e.g. radar) — measures two LOS angles plus slant range |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` | Kinematic track estimator for a moving object observed via angle or angle/range measurements |

---

## 11. Estimation Subsystem

Flight code estimation algorithms. Stub headers exist in `include/estimation/` (to be
created). Each derives from `DynamicElement` directly. Design authorities listed below.

| Class | Depends on | Design authority |
|---|---|---|
| `NavigationFilter` | `SensorGnss`, `SensorAirData`, `SensorMag` | [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `NavigationFilter` or `SensorInsSimulation`, `SensorAirData` | [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `WindEstimator`, `SensorAirData` | [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |
