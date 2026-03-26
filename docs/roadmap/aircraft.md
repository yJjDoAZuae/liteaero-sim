# Aircraft Class — Roadmap

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

**Note on system scope.** LiteAero Sim is the simulation plant only. Autopilot, guidance,
path representation, navigation, and gain scheduling are LiteAero Flight components that are
architecturally separate. Their design and implementation items are in the LiteAero Flight
roadmap (`liteaero-flight/docs/roadmap/flight_code.md`; cross-reference at
[flight_code.md](flight_code.md)). Items in this document are LiteAero Sim items only.
The system architecture is complete — see [`docs/architecture/system/future/`](../architecture/system/future/) and the
project roadmap [README.md](README.md) for cross-cutting milestones.

**Item process.** All items follow a documentation-first process:

1. Architecture and design documents are produced and reviewed.
2. Authorization to begin implementation is granted.
3. Implementation follows TDD — a failing test is written before every production code change.
4. All implementation items include simulation scenarios and automated tests sufficient for
   integration testing, demonstration, and investigation of algorithmic alternatives.

---

## Current State

| Component | File | Status |
| ----------- | ------ | -------- |
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto); `alphaSep()` / `alphaSepNeg()` accessors added |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto); alpha-ceiling guard corrected for positive thrust (item 20); branch-continuation predictor with domain guard (item 0a) |
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
| `DynamicElement` | `liteaero-flight/include/liteaero/control/DynamicElement.hpp` | → LiteAero Flight — see [dynamic_element.md](../architecture/dynamic_element.md) |
| `SisoElement` | `liteaero-flight/include/liteaero/control/SisoElement.hpp` | → LiteAero Flight — NVI SISO wrapper over `DynamicElement` |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SensorGnss` | `include/sensor/SensorGnss.hpp` | 🔲 Planned — stub not yet created |
| `SensorLaserAlt` | `include/sensor/SensorLaserAlt.hpp` | 🔲 Planned — stub not yet created |
| `SensorMag` | `include/sensor/SensorMag.hpp` | 🔲 Planned — stub not yet created |
| `SensorINS` | `include/sensor/SensorINS.hpp` | 🔲 Stub only |
| `SensorAA` | `include/sensor/SensorAA.hpp` | 🔲 Stub only |
| `SensorAAR` | `include/sensor/SensorAAR.hpp` | 🔲 Stub only |
| `SensorRadAlt` | `include/sensor/SensorRadAlt.hpp` | 🔲 Stub only |
| `SensorForwardTerrainProfile` | `include/sensor/SensorForwardTerrainProfile.hpp` | 🔲 Stub only |
| `SensorTrackEstimator` | `include/sensor/SensorTrackEstimator.hpp` | 🔲 Stub only |
| `NavigationFilter` | — | → LiteAero Flight — see [flight_code.md](flight_code.md) FC-8 |
| `WindEstimator` | — | → LiteAero Flight — see [flight_code.md](flight_code.md) FC-8 |
| `FlowAnglesEstimator` | — | → LiteAero Flight — see [flight_code.md](flight_code.md) FC-8 |
| `V_PathSegment` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-6 |
| `PathSegmentHelix` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-6 |
| `Path` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-6 |
| `PathGuidance` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-7 |
| `VerticalGuidance` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-7 |
| `ParkTracking` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-7 |
| `Autopilot` | — | → LiteAero Flight stub — see [flight_code.md](flight_code.md) FC-5 |

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
| 18 | Control subsystem refactoring (Steps A–J, all nine issues in [`control_interface_review.md`](../architecture/control_interface_review.md)) — `FilterError`/`DiscretizationMethod` promoted to `enum class`; `LimitBase` deleted, `Limit`/`RateLimit` rebased to `SisoElement`; `Gain` API cleaned up (`set()`, `value()`, stubs removed); `FilterSS2Clip`, `FilterTF2`, `FilterTF`, `FilterFIR`, `FilterSS` migrated to NVI (`onStep()`/`onSerializeJson()`/`onDeserializeJson()`); shadow `_in`/`_out` members and no-op `Filter` defaults removed; `Unwrap` `ref_` field added (`setReference()`, NVI routing, serialization); `SISOPIDFF` derives from `DynamicElement` with full lifecycle, `snake_case_` member renames (`Kp`→`proportional_gain_`, `I`→`integrator_`, etc.), private limits, `ControlLoop` accessor renames (`out()`→`output()`, `pid`→`controller_`); `Integrator`/`Derivative` private member renames (`_dt`→`dt_s_`, `_Tau`→`tau_s_`, `limit`→`limit_`) | `FilterSS2Clip_test.cpp`, `FilterTF2_test.cpp`, `FilterFIR_test.cpp` (new), `FilterSS_test.cpp`, `Unwrap_test.cpp`, `SISOPIDFF_test.cpp` (new) — 29 new tests; 435 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 19 | `SensorAirData` — pitot-static air data computer; differential pressure ($q_c$) and static pressure ($P_s$) transducers with Gaussian noise, first-order Tustin lag, and fuselage crossflow pressure error (two-port symmetric crosslinked model); derives IAS, CAS, EAS, TAS, Mach, barometric altitude (Kollsman-referenced, troposphere + tropopause), OAT; RNG pimpl with seed + advance serialization; JSON + proto round-trips | `SensorAirData_test.cpp` — 19 tests; 454 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 20 | `LoadFactorAllocator` alpha-ceiling fix — Newton overshoot and fold guards corrected for positive-thrust case; the achievable-Nz ceiling is at $\alpha^*$ (where $f'(\alpha) = qSC_L'(\alpha) + T\cos\alpha = 0$), not at `alphaPeak()`, when $T > 0$; overshoot guard now clamps the proposed Newton step to the CL parabolic domain using `LiftCurveModel::alphaSep()` / `alphaSepNeg()` before checking $f'$, preventing escape into the flat separated plateau where $f' = T\cos\alpha$ stays positive until $\alpha > \pi/2$; bisects to locate $\alpha^*$ when the guard fires; fold guard stays at current iterate rather than snapping to `alphaPeak()`; `LiftCurveModel::alphaSep()` and `alphaSepNeg()` added to public interface; design documentation updated in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) and [`docs/algorithms/equations_of_motion.md`](../algorithms/equations_of_motion.md) | 4 new tests in `LoadFactorAllocator_test.cpp`; 19 tests total; 458 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 21 | **System architecture definition** — future-state system architecture model covering: originating requirements, use cases (UC-1 through UC-7), element registry (LiteAero Sim, LiteAero Flight, SimulationRunner, External Interface elements), data flow type and instance registries, interface control documents (ICD-8 through ICD-12), architectural decisions (30 recorded), open questions (all pre-design questions resolved; design-phase questions tracked); system boundary between LiteAero Sim simulation plant and LiteAero Flight established; Docker containerization model for SITL verification defined; `liteaero::` namespace structure and CMake target structure decided; repo split plan defined | No code tests — deliverable is the architecture document set under [`docs/architecture/system/future/`](../architecture/system/future/) |
| 22 | `LoadFactorAllocator` branch-continuation predictor — first-order warm-start $\alpha_0 = \alpha_\text{prev} + \delta n_z \cdot mg / f'(\alpha_\text{prev})$ and symmetric $\beta_0$ formula added to `solve()`; predictor is skipped at the stall ceiling ($f' \approx 0$) or when the raw prediction would fall outside $[\alpha_\text{sep\_neg}, \alpha_\text{sep}]$ (domain guard prevents cross-branch jumps on cold-start excess-demand calls); `_n_z_prev` and `_n_y_prev` added as serialized state fields in both JSON (`n_z_prev_nd`, `n_y_prev_nd`) and proto (`LoadFactorAllocatorState` fields 6–7); `iterations` (alpha solver iteration count) added to `LoadFactorOutputs`; `reset()` clears all four warm-start fields; [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Warm-Starting updated | 3 new tests in `LoadFactorAllocator_test.cpp`: `PredictorReducesIterationsOnLinearStep` (iterations == 1 for an exact linear-region prediction), `PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev`, `PredictorProtoRoundTrip_IncludesNzPrev`; 22 tests total |
| 23 | `LoadFactorAllocator` test coverage extension — 8 new tests close continuity and domain-coverage gaps identified by code review. **White-box tests** (4): full positive and negative Nz sweeps through stall verifying the clamp value against `alphaPeak()`/`alphaTrough()`; fine-step sweep across the C¹ Linear→IncipientStall boundary confirming both segments are traversed; stall warm-start limitation test documenting that `reset()` is required after a discontinuous Nz jump. **Black-box tests** (4): uniform 500-step monotonicity sweeps from 0 to ±10 g for T = 0 (positive and negative) and T = `kLargeThrust` (positive); point-wise perturbation test at 37 grid points (T = 0, −9 g to +9 g) and 19 grid points (T > 0, 0 to +9 g) using fresh allocators. Stall warm-start limitation documented in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Stall Warm-Start Limitation | 30 tests total in `LoadFactorAllocator_test.cpp` |
| 24 | **Aircraft command processing redesign** — all three command axes (`_nz_filter`, `_ny_filter`, `_roll_rate_filter`) converted to `setLowPassSecondIIR` (2nd-order LP); config params replaced: `cmd_deriv_tau_s`/`cmd_roll_rate_tau_s` → `nz_wn_rad_s`/`nz_zeta_nd`/`ny_wn_rad_s`/`ny_zeta_nd`/`roll_rate_wn_rad_s`/`roll_rate_zeta_nd`; `n_z_dot`/`n_y_dot` computed analytically from filter state after substep loop (no derivative filter lag); allocator receives shaped commands instead of raw clamped commands; Nyquist constraint enforced per axis (`wn * cmd_filter_dt_s < π`) at `initialize()`; proto `AircraftState` updated; `aircraft_config_v1` schema doc updated; fixture JSON files updated; design authority: [`docs/architecture/aircraft.md`](../architecture/aircraft.md) §Command Processing Architecture | 3 new Nyquist violation tests in `Aircraft_test.cpp` (`NyquistViolation_Nz_Throws`, `_Ny_Throws`, `_RollRate_Throws`); 345 pre-existing tests pass |

---

## 1. Landing Gear — Ground Contact Model Design

`LandingGear` is a Domain Layer physics component that models the contact forces and
moments exerted on the airframe by the landing gear during ground operations (taxi,
takeoff roll, and landing roll). It produces `ContactForces` (body-frame force vector,
body-frame moment vector, and `weight_on_wheels` flag) and is internal to both `Aircraft`
and `Aircraft6DOF`.
The integration architecture and fidelity target are decided — see
[`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md) §LandingGear integration model and fidelity target.

This item produces the design authority document at [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md).
Implementation is item 8. Do not implement in this item.

### Scope — LandingGear Design

The design must address at minimum:

- **Wheel contact model** — Pacejka "magic formula" for tyre longitudinal and lateral
  force generation as a function of slip ratio and slip angle. The formula captures the
  nonlinear saturation of tyre force with slip and reproduces the qualitative shape of
  measured tyre data without requiring high-fidelity tyre datasets.
- **Suspension dynamics** — second-order spring-damper model per strut. Configuration
  parameters are spring stiffness (N/m), damper coefficient (N·s/m), and strut
  preload force (N). Suspension travel is constrained between fully extended and
  fully compressed limits.
- **Wheel geometry** — each wheel unit is defined by its attachment point in the body
  frame and a unit vector defining the suspension travel axis, both expressed in body
  coordinates. This supports tricycle, taildragger, and multi-bogey layouts without
  special-casing.
- **Wheel rotational friction** — model tire rolling resistance and axle friction such that the aircraft decelerates to a stop in finite time during landing rollout on a level surface with no thrust or wind.
- **Computational efficiency** — the landing gear contact model must not force the
  aircraft simulation to run at a higher timestep rate than the rigid-body integrator.
  High-frequency strut and tyre dynamics that do not materially influence the aircraft
  trajectory response should be suppressed or filtered. An inner-step sub-loop — similar
  to the approach used for high-bandwidth FBW axis dynamics — should be evaluated as a
  mechanism to decouple stiff strut dynamics from the outer rigid-body loop if it allows
  the outer step rate to remain at the standard simulation rate.
- **Ground plane interface** — the model queries terrain height and surface normal at
  the projected wheel contact point; the interface must be compatible with both
  `FlatTerrain` and `TerrainMesh`. Define any required terrain model extensions for
  runway geometry: options include an inset planar patch, a fine heightmap grid, or an
  analytical runway definition with longitudinal slope and crowned lateral profile.
  Define surface friction parameterization for pavement, grass, dirt, gravel, and wet
  surfaces. The AGL sensor must respond to the same surface height used by the contact
  model.
- **Serialization** — full JSON and proto round-trip serialization of suspension state
  (strut deflection and deflection rate per wheel unit); RNG state if stochastic runway
  roughness is added.
- **Notebook visualization** — demonstrate the model through a Jupyter notebook covering landing contact, takeoff rotation, and liftoff scenarios, with plots of contact forces, friction moments, strut displacement, and wheel speed.

### Deliverables — Landing Gear

Design authority document at [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md) to be written
before implementation begins.

Implementation follows TDD: failing tests before production code.

---

## 2. Plot Visualization — Python Post-Processing Tools

Python scripts to load logger output and produce time-series plots for simulation
post-flight analysis. These are Application Layer tools and live under `python/tools/`.

### Deliverables — Plot Visualization

- `python/tools/plot_flight.py`: CLI script that reads a `.csv` log file produced by
  `Logger` and plots a configurable set of channels vs. time. Outputs a `.png` or
  displays interactively.
- Channel groups: kinematics (position, velocity, attitude), aerodynamics (α, β, CL, CD),
  propulsion (thrust, throttle), guidance (cross-track error, altitude error).

### Tests — Plot Visualization

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

## 3. Manual Input — Joystick and Keyboard

Manual input adapters translate human control inputs (joystick axes, keyboard state) into
an `AircraftCommand`. These live in the Interface Layer and have no physics logic.

### Deliverables — Manual Input

- `KeyboardInput`: maps configurable key bindings to throttle, roll, pitch, yaw increments.
  Operates at the simulation timestep rate.
- `JoystickInput`: reads a raw joystick device (SDL2 or platform API) and maps axes and
  buttons to the `AircraftCommand` fields. Applies a configurable dead zone and axis scaling.

### Interface Sketch — Manual Input

```cpp
// include/input/ManualInput.hpp
namespace liteaero::simulation {

class V_ManualInput {
public:
    virtual AircraftCommand read() = 0;   // non-blocking; returns latest command
    virtual ~V_ManualInput() = default;
};

} // namespace liteaero::simulation
```

### Tests — Manual Input

- `KeyboardInput` with no keys pressed returns zero lateral/directional commands and
  configured idle throttle.
- `JoystickInput` axis value at dead-zone boundary maps to zero output.
- Axis scaling: full-deflection axis value maps to the configured maximum command.

### CMake — Manual Input

Add `src/input/KeyboardInput.cpp` and `src/input/JoystickInput.cpp` to `liteaero-sim`.
Add a platform-conditional dependency on SDL2 for `JoystickInput`.

---

## 4. Execution Modes — Real-Time, Scaled, and Batch Runners

The simulation runner controls the wall-clock relationship to simulation time. Three modes
are required:

| Mode | Description |
| --- | --- |
| **Real-time** | Each `step()` is paced to its real elapsed wall time (`dt_s` per step). |
| **Scaled real-time** | Same as real-time but with a configurable speed multiplier (0.5×, 2×, etc.). |
| **Full-rate batch** | Steps run as fast as possible; no sleep; used for CI, Monte Carlo, and data generation. |

### Interface Sketch — SimRunner

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

### Tests — SimRunner

- `Batch` mode with `duration_s = 1.0` and `dt_s = 0.01` calls `Aircraft::step()` exactly
  100 times and then stops.
- `RealTime` mode: elapsed wall time after 10 steps is within 10% of `10 * dt_s`.
- `stop()` called from outside (threaded test) terminates the run loop within one timestep.
- `elapsed_sim_time_s()` returns `n_steps * dt_s` after `n_steps` have been executed.

### CMake — SimRunner

Add `src/runner/SimRunner.cpp` to `liteaero-sim`.
Add `test/SimRunner_test.cpp` to the test executable.

---

## 5. Remaining Sensor Models

Not blocking any higher-priority item. Stub headers in `include/sensor/` exist for
`SensorINS`, `SensorAA`, `SensorAAR`, `SensorRadAlt`, `SensorForwardTerrainProfile`, and
`SensorTrackEstimator`. Stubs for `SensorMag`, `SensorGnss`, and `SensorLaserAlt` have
not yet been created. Implement when needed; order within this group follows dependency.

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | Terrain (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | Terrain (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |
| `SensorINS` | `KinematicState` (done), `NavigationFilter` types | INS simulation replacement — truth-plus-error model producing `InsMeasurement` |
| `SensorForwardTerrainProfile` | Terrain (done), Guidance | Forward terrain profiling sensor (multi-beam LIDAR / line-scan radar) for terrain-following guidance |
| `SensorAA` | `KinematicState` (done) | Passive angle/angle sensor (imaging type) — measures two LOS angles, no range |
| `SensorAAR` | `KinematicState` (done) | Active angle/angle/range sensor (e.g. radar) — measures two LOS angles plus slant range |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` | Kinematic track estimator for a moving object observed via angle or angle/range measurements |

---

## 6. Aerodynamic Coefficient Design Study

Prerequisite for item 7 (`Aircraft6DOF` and `BodyAxisCoeffModel`). Cannot be designed
without first defining how aerodynamic coefficients will be obtained and running the model
design process through several example aircraft configurations. This item produces the
design study that resolves OQ-16(c).

### Deliverables — Aerodynamic Coefficient Design Study

Design study document defining: aerodynamic coefficient data sources (wind tunnel, CFD,
DATCOM, flight test); data formats and axes conventions; coefficient model format for
`BodyAxisCoeffModel` across the range of anticipated fixed-wing configurations. Must be
complete before item 7 begins.

---

## 7. Aircraft6DOF — Design and Implementation

Full 6DOF aircraft dynamics model. Depends on item 6 (aero coefficient design study).
Architecture placeholders are defined in [`docs/architecture/system/future/element_registry.md`](../architecture/system/future/element_registry.md).

### Scope

- **`V_AeroModel`** — abstract aerodynamic model interface; produces forces and moments in
  body frame; decouples the 6DOF integrator from the coefficient axis convention.
- **`BodyAxisCoeffModel`** — implements `V_AeroModel` using body-axis stability derivatives
  (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular
  rates. Coefficient model format defined by item 6.
- **`Aircraft6DOF`** — full 6DOF dynamics; depends on `V_AeroModel` for forces and moments;
  accepts `SurfaceDeflectionCommand` (control surface deflection angles + per-motor
  throttle); produces `KinematicStateSnapshot`; used directly by ArduPilot and PX4
  simulations (no FBW bridge in that topology).
- **`FBWController`** — inner-loop FBW control law bridging the existing load-factor
  `AircraftCommand` interface to `SurfaceDeflectionCommand` for `Aircraft6DOF`; when paired
  with `Aircraft6DOF` forms a drop-in replacement for `Aircraft` for side-by-side
  comparison; not used in ArduPilot/PX4 topologies.
- **`SurfaceDeflectionCommand`** — plain value struct: control surface deflection angles
  (elevator, aileron, rudder) and per-motor throttle.

### Deliverables — Aircraft6DOF

Design authority document. Implementation follows TDD. All new elements include JSON +
proto serialization and round-trip tests. `Aircraft6DOF` and `Aircraft` must both produce
`KinematicStateSnapshot` to enable transparent substitution.

---

## 8. LandingGear — Implementation

Implement the design produced in item 1. Architecture decisions are recorded in
[`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md) (LandingGear integration model and fidelity
target). Design authority document at [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md) must exist before
implementation begins (item 1 deliverable).

Produces `ContactForces` (6-component body-frame force + moment + `weight_on_wheels` flag).
Accepts nose wheel steering angle and differential brake demands. Implements 2nd-order
suspension per strut and tyre contact force model. Queries ground elevation via `V_Terrain`.

Interfaces to both `Aircraft` (load-factor disturbance path) and `Aircraft6DOF` (full
6-component EOM input). Follow TDD: write failing tests before production code.

---

## 9. External Interface Elements

Adapters that connect LiteAero Sim to external systems. All live in the Interface Layer.

| # | Element | Protocol | Depends on |
| --- | --- | --- | --- |
| LAS-ext-1 | `ArduPilotInterface` | ArduPilot SITL protocol | SimRunner (item 4); `Aircraft` or `Aircraft6DOF` |
| LAS-ext-2 | `PX4Interface` | PX4 SITL bridge | SimRunner (item 4); `Aircraft6DOF` |
| LAS-ext-3 | `QGroundControlLink` | MAVLink over UDP | SimRunner (item 4); `NavigationState` (LiteAero Flight) |
| LAS-ext-4 | `VisualizationLink` | UDP to Godot 4 GDExtension plugin at simulation rate | SimRunner (item 4); `SimulationFrame` (done) |

Each element requires a design document before implementation. `VisualizationLink` transport
and axis convention are decided (see [`docs/architecture/terrain.md`](../architecture/terrain.md) §Game Engine Integration
and [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md) §Game engine for real-time visualization).

---

## 10. Synthetic Perception Sensors — Proposed

The following sensor elements are proposed and not yet designed. They depend on `V_Terrain`
for geometry queries. Design items will be scheduled when prerequisite sensor and terrain
models are stable.

| Element | Responsibility |
| --- | --- |
| `SensorCamera` | Synthetic image sensor; generates imagery from terrain and scene model against `V_Terrain` |
| `SensorLidar` | Synthetic lidar; generates 3D point cloud by ray-casting against `V_Terrain` |
| `SensorLaserAGL` | Synthetic laser altimeter; computes AGL range by ray-casting against `V_Terrain` |
| `SensorLineOfSight` | Computes RF link quality and terrain occlusion by ray-casting against `V_Terrain` |
