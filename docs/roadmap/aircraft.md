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
| `SimRunner` | `include/runner/SimRunner.hpp` | ✅ Implemented — Batch / RealTime / ScaledRealTime; see [sim_runner.md](../architecture/sim_runner.md) |
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
| `WheelUnit` | `include/landing_gear/WheelUnit.hpp` | ✅ Implemented + serialization (JSON + proto) — see [landing_gear.md](../architecture/landing_gear.md) |
| `StrutState` | `include/landing_gear/StrutState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `ContactForces` | `include/landing_gear/ContactForces.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SurfaceFriction` | `include/landing_gear/SurfaceFriction.hpp` | ✅ Implemented |
| `SurfaceFrictionUniform` | `include/landing_gear/SurfaceFrictionUniform.hpp` | ✅ Implemented (named constructors: pavement/grass/dirt/gravel, wet/dry) |
| `LandingGear` | `include/landing_gear/LandingGear.hpp` | ✅ Implemented + serialization (JSON + proto); wired into `Aircraft::step()` |
| `FlightLogReader` | `python/tools/log_reader.py` | 🔲 Planned — see [post_processing.md](../architecture/post_processing.md) |
| `AnomalyDetector` | `python/tools/anomaly.py` | 🔲 Planned |
| `BehaviorVerifier` | `python/tools/behavior_verifier.py` | 🔲 Planned |
| `DataOverlay` | `python/tools/data_overlay.py` | 🔲 Planned |
| `ModeEventSeries` | `python/tools/mode_overlay.py` | 🔲 Planned |
| `TimeHistoryFigure` | `python/tools/time_history.py` | 🔲 Planned |
| `TrajectoryView` | `python/tools/trajectory_view.py` | 🔲 Planned |

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
| 26 | **Post-processing tools design** — full design authority document covering: use case decomposition (UC-PP1 through UC-PP10); actors (Sim Developer, Integration Tester, Flight Analyst, Guidance/Autopilot Developer); module architecture (`FlightLogReader`, `AnomalyDetector`, `BehaviorVerifier`, `DataOverlay`, `ModeEventSeries`, `TimeHistoryFigure`, `RibbonTrail`, `HudOverlay`, `TrajectoryView`); ribbon trail geometry (body-to-world rotation, wing half-span vector, `Poly3DCollection`); HUD overlay layout; command/response time history encoding; `BehaviorVerifier` criterion library and multi-source DataFrame interface; library choices (pandas, matplotlib, Plotly, mcap, numpy); test strategy (8 test files, rendering non-invocation requirement) | [`docs/architecture/post_processing.md`](../architecture/post_processing.md) |
| 25 | **Landing gear model design** — full design authority document covering: use case decomposition; class hierarchy (`LandingGear`, `WheelUnit`, `StrutState`, `ContactForces`, `SurfaceFriction`, `SurfaceFrictionUniform`); physical models (suspension spring-damper, Pacejka magic formula, wheel friction, surface friction parameterization, ground plane interface); force assembly; step interface; JSON + proto serialization contract; computational resource estimate; test strategy (unit, integration, scenario, serialization); visualization notebook designs (`landing_gear_contact_forces.ipynb`, `crab_landing_dynamics.ipynb`, `takeoff_roll.ipynb`, `terrain_contact_animation.ipynb`); touchdown animation design (`touchdown_animation.py` — pybind11 driver, layout, visual encoding, coordinate mapping, data flow) | [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md) |
| LG-1 | **LandingGear — C++ implementation** (Steps A–F) — `WheelUnit`, `StrutState`, `ContactForces`, `SurfaceFriction`, `SurfaceFrictionUniform`, `LandingGear`; quasi-static spring-damper strut; Pacejka magic formula tyre forces (longitudinal + lateral); friction-circle saturation; viscous wheel speed integration; nose wheel steering; differential braking; terrain elevation query via `V_Terrain`; wired into `Aircraft::step()` on the disturbance force path; `Aircraft::setTerrain()`, `contactForces()`, `weightOnWheels()` added; JSON + proto serialization throughout; `WheelUnitState`, `ContactForcesState`, `LandingGearState` proto messages added | `LandingGearTypes_test.cpp`, `SurfaceFriction_test.cpp`, `LandingGear_test.cpp`, `LandingGearTerrain_test.cpp` — 27 tests; `Aircraft_test.cpp` — 2 new tests; 388 total pass |
| Sim-1 | **SimRunner — Execution Modes** — `ExecutionMode` enum (`Batch`, `RealTime`, `ScaledRealTime`); `RunnerConfig` struct (`dt_s` float — output step, adequate precision for timestep values; `duration_s` double — needed for long runs compared to accumulated sim time; `time_scale` float; `mode`); `SimRunner` class with `initialize()`, `start()`, `stop()`, `is_running()`, `elapsed_sim_time_s()`; Batch mode blocks caller; RealTime/ScaledRealTime spawn `std::thread`; `std::atomic<uint64_t> step_count_` for elapsed time; termination condition `sim_time_s > duration_s + time_initial_s` (direct time comparison — no precomputed step count); `dt_s` widened to `double` once at loop entry for all arithmetic; late-step policy: no compensation; design authority: [`docs/architecture/sim_runner.md`](../architecture/sim_runner.md) | `SimRunner_test.cpp` — 10 tests |

---

## 1. Aerodynamic Coefficient Design Study

**Blocking dependencies:** None. `AeroCoeffEstimator` is implemented.

Design authority: [`docs/architecture/aero_coefficient_model.md`](../architecture/aero_coefficient_model.md).

Prerequisite for item 7 (`Aircraft6DOF` and `BodyAxisCoeffModel`). Resolves OQ-16(c).

### Deliverables — Aerodynamic Coefficient Design Study

- `aero_coefficient_model.md` completed and all open questions resolved.
- `AircraftGeometry` JSON files and coefficient tables for Cases A, B, and C.
- `BodyAxisCoeffModel` coefficient format decided and documented.
- Element registry updated (`AeroModel` named, `BodyAxisCoeffModel` format defined).

Must be complete before item 7 begins.

---

## 2. Post-Processing — Visualization Tools

**Blocking dependencies:** None. Depends only on external Python libraries (all
available). Does not depend on any logged channel schema or sensor implementation.

Implement the visualization modules from delivered item 26. Design authority:
[`docs/architecture/post_processing.md`](../architecture/post_processing.md) §Dependency
Status — Visualization Modules.

All tools live under `python/tools/`. Implementation follows TDD.

### Scope

| Module | File | External dependencies |
| --- | --- | --- |
| `FlightLogReader` | `python/tools/log_reader.py` | `mcap`, `pandas` |
| `ModeEventSeries` | `python/tools/mode_overlay.py` | `pandas` |
| `TimeHistoryFigure` | `python/tools/time_history.py` | `plotly` |
| `RibbonTrail` | `python/tools/trajectory_view.py` | `numpy`, `matplotlib` |
| `HudOverlay` | `python/tools/trajectory_view.py` | `matplotlib` |
| `TrajectoryView` | `python/tools/trajectory_view.py` | `matplotlib` |

The analysis modules (`AnomalyDetector`, `BehaviorVerifier`, `DataOverlay`) are deferred
to item 8. Do not implement them in this item.

### Tests — Visualization Tools

`python/test/test_log_reader.py`, `test_mode_events.py`, `test_time_history.py`,
`test_ribbon_trail.py`. No test may invoke `plt.show()`, `fig.show()`, or open a display.

### Python Dependencies

Add to `python/pyproject.toml`:

```toml
[dependency-groups]
dev = [
    "matplotlib>=3.8",
    "pandas>=2.0",
    "plotly>=5.18",
    "mcap>=1.1",
    "numpy>=1.26",
]
```

---

## 3. Manual Input — Joystick and Keyboard

**Blocking dependencies:** None. `AircraftCommand` is implemented.

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

## 4. Sensor Models — Implementable Subset

**Blocking dependencies:** None. `KinematicState` and `V_Terrain` are implemented.

Implement the four sensor models whose only dependencies are already available. The
remaining sensors (`SensorINS`, `SensorAA`, `SensorAAR`, `SensorForwardTerrainProfile`,
`SensorTrackEstimator`) are deferred to item 12; they depend on LiteAero Flight
components that are not yet designed.

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | `V_Terrain` (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | `V_Terrain` (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |

Each sensor requires a design document before implementation. Implement in the order
listed; `SensorLaserAlt` and `SensorRadAlt` outputs are required by the `AnomalyDetector`
`AltitudeBelowTerrain` rule (item 8).

---

## 5. Logged Channel Registry — Design

**Blocking dependencies:** SimRunner (delivered), LandingGear C++ (delivered), item 4
(sensor models subset). The registry must reflect the complete channel set produced by the
simulation loop, including gear contact channels and sensor channels.

Produces a formal specification of all `LogSource` registrations in the simulation loop:
source names, channel names, units, and sampling rates. This document is required before
`AnomalyDetector` rules and `BehaviorVerifier` criteria can reference channel names.

### Deliverables — Logged Channel Registry

Design document (`docs/architecture/channel_registry.md`) specifying:

- All `LogSource` instances registered by the simulation loop (e.g., `"aircraft"`,
  `"environment"`, `"landing_gear"`, `"sensors"`).
- For each source: the complete channel list with SI-unit-suffixed names, data type, and
  sampling rate.
- Naming conventions for channels from `KinematicStateSnapshot`, `AircraftState`,
  `ContactForces`, `AtmosphericState`, and each sensor output struct.
- Policy for how new subsystems register channels as they are added.

---

## 6. Real Flight Log Format — Design

**Blocking dependencies:** None. This is a standalone design decision.

Produces a design decision document defining how real aircraft flight logs are loaded by
`DataOverlay`. Must be resolved before `DataOverlay` can be implemented.

### Deliverables — Real Flight Log Format

Design document (`docs/architecture/flight_log_format.md`) covering:

- What log format(s) real aircraft produce (e.g., ArduPilot DataFlash, MAVLink ULOG,
  custom CSV, or a configurable adapter).
- Channel name mapping from the real-log format to the simulation channel naming
  convention defined in item 5.
- Policy for handling channels present in the real log but absent from the sim schema,
  and vice versa.
- Whether a translation/adapter layer is implemented in `FlightLogReader` or as a
  separate preprocessing step.

---

## 7. Aircraft6DOF — Design and Implementation

**Blocking dependencies:** Item 1 (aerodynamic coefficient design study).

Full 6DOF aircraft dynamics model. Architecture placeholders are defined in
[`docs/architecture/system/future/element_registry.md`](../architecture/system/future/element_registry.md).

### Scope — Aircraft6DOF

- **`V_AeroModel`** — abstract aerodynamic model interface; produces forces and moments in
  body frame; decouples the 6DOF integrator from the coefficient axis convention.
- **`BodyAxisCoeffModel`** — implements `V_AeroModel` using body-axis stability derivatives
  (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular
  rates. Coefficient model format defined by item 1.
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

## 8. Post-Processing — Analysis Tools Design Harmonization and Implementation

**Blocking dependencies:** Item 5 (Logged Channel Registry), item 6 (Real Flight Log
Format), and LiteAero Flight command channel schema (cross-repo dependency — track in
LiteAero Flight roadmap).

This item performs a second design pass on the analysis modules in
[`docs/architecture/post_processing.md`](../architecture/post_processing.md) to replace
placeholder channel names with concrete names from the channel registry, add the real
flight log format adapter to `DataOverlay`, and incorporate the LiteAero Flight command
channel schema into `BehaviorVerifier` criteria. It then implements those modules.

### Design Harmonization

Update `docs/architecture/post_processing.md` §Analysis Modules:

- Replace all channel name references in `AnomalyDetector` rules with names from the
  Logged Channel Registry (item 5).
- Specify the `DataOverlay` format adapter for the real flight log format (item 6).
- Define `BehaviorVerifier` command channel names from the LiteAero Flight command schema.
- Define the Scenario Reference Data Format for `WaypointReached` and similar criteria.

### Analysis Module Implementation

Implement in dependency order:

| Module | File | Blocked by |
| --- | --- | --- |
| `AnomalyDetector` + rule library | `python/tools/anomaly.py` | Item 5, sensor item 4 |
| `DataOverlay` | `python/tools/data_overlay.py` | Item 6 |
| `BehaviorVerifier` + criterion library | `python/tools/behavior_verifier.py` | Item 5, LiteAero Flight schema |

### Tests — Analysis Tools

`python/test/test_anomaly.py`, `test_data_overlay.py`, `test_behavior_verifier.py`.

---

## 9. LandingGear — Python Bindings and Scenario Tests

**Blocking dependencies:** LandingGear C++ (delivered, LG-1), item 2
(visualization tools exist for animation).

Implement Steps G–H from the design authority document
([`docs/architecture/landing_gear.md`](../architecture/landing_gear.md)).

### Steps G–H

#### Step G — Python Bindings (pybind11)

Expose `LandingGear`, `WheelUnit`, `StrutState`, and `ContactForces` to Python via
pybind11. Build target: `liteaero_sim_py` — a pybind11 extension module. Add to CMake
as an optional target gated on `LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON`.

Rewrite `python/scripts/touchdown_animation.py` to call `LandingGear::step()` via the
pybind11 module. Remove Python-side contact physics. The Python-side aerodynamics model
(parameterized from `AeroCoeffEstimator` JSON output) remains until a C++ aerodynamics
class with bindings exists.

#### Step H — Scenario Tests

Implement the four scenario tests from the design authority document as pytest fixtures:

| Scenario | File | Pass criterion |
| --- | --- | --- |
| Landing rollout | `python/test/test_landing_rollout.py` | Aircraft decelerates to rest; max $F_z$ within 10% of analytical impulse estimate |
| Crab landing | `python/test/test_crab_landing.py` | Lateral drift eliminated within 3 s; no diverging yaw oscillation |
| Takeoff roll | `python/test/test_takeoff_roll.py` | All wheels leave ground at rotation speed; `weight_on_wheels` → false |
| Bounce (light contact) | `python/test/test_bounce.py` | Contact duration < 0.3 s; $F_z$ peak < 1.5× static weight |

### CMake Addition

Add optional `liteaero_sim_py` pybind11 extension target controlled by
`LITEAERO_SIM_BUILD_PYTHON_BINDINGS`. Add pybind11 to `conanfile.txt` (ConanCenter).

---

## 10. External Interface Elements

**Blocking dependencies:** SimRunner (delivered) for all elements. Item 7 (Aircraft6DOF)
for `PX4Interface`. `NavigationState` from LiteAero Flight for `QGroundControlLink`.

Adapters that connect LiteAero Sim to external systems. All live in the Interface Layer.

| # | Element | Protocol | Depends on |
| --- | --- | --- | --- |
| LAS-ext-1 | `ArduPilotInterface` | ArduPilot SITL protocol | SimRunner (delivered); `Aircraft` or `Aircraft6DOF` |
| LAS-ext-2 | `PX4Interface` | PX4 SITL bridge | SimRunner (delivered); `Aircraft6DOF` (item 7) |
| LAS-ext-3 | `QGroundControlLink` | MAVLink over UDP | SimRunner (delivered); `NavigationState` (LiteAero Flight) |
| LAS-ext-4 | `VisualizationLink` | UDP to Godot 4 GDExtension plugin at simulation rate | SimRunner (delivered); `SimulationFrame` (done) |

Each element requires a design document before implementation. `VisualizationLink`
transport and axis convention are decided (see
[`docs/architecture/terrain.md`](../architecture/terrain.md) §Game Engine Integration and
[`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
§Game engine for real-time visualization).

---

## 11. Sensor Models — Deferred Subset

**Blocking dependencies:** LiteAero Flight components not yet designed (`NavigationFilter`
for `SensorINS`; Guidance for `SensorForwardTerrainProfile`). `SensorAA`, `SensorAAR`,
and `SensorTrackEstimator` require additional design work beyond their stated hardware
model.

Schedule when respective LiteAero Flight dependencies are available.

| Class | Blocked by |
| --- | --- |
| `SensorINS` | `NavigationFilter` types (LiteAero Flight FC-8) |
| `SensorForwardTerrainProfile` | Guidance design (LiteAero Flight) |
| `SensorAA` | Design work item needed |
| `SensorAAR` | Design work item needed |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` |

---

## 12. Synthetic Perception Sensors — Proposed

**Blocking dependencies:** Design items needed for each sensor before implementation
can begin. Schedule when prerequisite sensor and terrain models are stable.

| Element | Responsibility |
| --- | --- |
| `SensorCamera` | Synthetic image sensor; generates imagery from terrain and scene model against `V_Terrain` |
| `SensorLidar` | Synthetic lidar; generates 3D point cloud by ray-casting against `V_Terrain` |
| `SensorLaserAGL` | Synthetic laser altimeter; computes AGL range by ray-casting against `V_Terrain` |
| `SensorLineOfSight` | Computes RF link quality and terrain occlusion by ray-casting against `V_Terrain` |
