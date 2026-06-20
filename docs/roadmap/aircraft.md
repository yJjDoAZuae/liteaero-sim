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
2. All open questions in the design document that affect the implementation scope are
   resolved before authorization to implement is requested. If open questions exist,
   a design-resolution item precedes the implementation item in the queue.
3. Authorization to begin implementation is granted.
4. Implementation follows TDD — a failing test is written before every production code change.
5. If an open question is discovered during implementation, it is documented immediately
   in the design document. Implementation continues for all work that does not depend on
   resolution of that question. No code is written whose correctness depends on the
   unresolved choice.
6. All implementation items include simulation scenarios and automated tests sufficient for
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
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../design/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../design/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../design/propulsion.md) |
| `Motor` | `include/propulsion/Motor.hpp` | ✅ Implemented — stateless abstract interface; replaces `V_Motor` |
| `MotorElectric` | `include/propulsion/MotorElectric.hpp` | ✅ Implemented — see [propulsion.md](../design/propulsion.md) |
| `MotorPiston` | `include/propulsion/MotorPiston.hpp` | ✅ Implemented — see [propulsion.md](../design/propulsion.md) |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../design/propulsion.md) |
| `Aircraft` | `include/Aircraft.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SimRunner` | `include/runner/SimRunner.hpp` | ✅ Implemented — Batch / RealTime / ScaledRealTime; see [sim_runner.md](../design/sim_runner.md) |
| `Atmosphere` | `include/environment/Atmosphere.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AtmosphericState` | `include/environment/AtmosphericState.hpp` | ✅ Implemented |
| `Wind` | `include/environment/Wind.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Turbulence` | `include/environment/Turbulence.hpp` | ✅ Implemented + serialization (JSON) |
| `Gust` | `include/environment/Gust.hpp` | ✅ Implemented |
| `TurbulenceVelocity` | `include/environment/TurbulenceVelocity.hpp` | ✅ Implemented |
| `EnvironmentState` | `include/environment/EnvironmentState.hpp` | ✅ Implemented |
| `SurfaceGeometry` / `AircraftGeometry` | `include/aerodynamics/AircraftGeometry.hpp` | ✅ Implemented |
| `AeroCoeffEstimator` | `include/aerodynamics/AeroCoeffEstimator.hpp` | ✅ Implemented |
| `DynamicElement` | `liteaero-flight/include/liteaero/control/DynamicElement.hpp` | → LiteAero Flight — see [dynamic_element.md](../design/dynamic_element.md) |
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
| `WheelUnit` | `include/landing_gear/WheelUnit.hpp` | ✅ Implemented + serialization (JSON + proto) — see [landing_gear.md](../design/landing_gear.md) |
| `StrutState` | `include/landing_gear/StrutState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `ContactForces` | `include/physics/ContactForces.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SurfaceFriction` | `include/landing_gear/SurfaceFriction.hpp` | ✅ Implemented |
| `SurfaceFrictionUniform` | `include/landing_gear/SurfaceFrictionUniform.hpp` | ✅ Implemented (named constructors: pavement/grass/dirt/gravel, wet/dry) |
| `LandingGear` | `include/landing_gear/LandingGear.hpp` | ✅ Implemented + serialization (JSON + proto); wired into `Aircraft::step()` |
| `AeroModel` | `include/aerodynamics/AeroModel.hpp` | 🔲 Planned — abstract aero model interface; defined by item 6; see [aero_coefficient_model.md](../design/aero_coefficient_model.md) |
| `BodyAxisCoeffModel` | `include/aerodynamics/BodyAxisCoeffModel.hpp` | 🔲 Planned — body-axis stability derivative model; implements `AeroModel`; defined by item 5 + item 6; see [aero_coefficient_model.md](../design/aero_coefficient_model.md) |
| `PropulsionCouplingCoefficients` | TBD | 🔲 Planned — propulsion-aero coupling coefficient struct; defined by item 5; see [propulsion_coeff_estimator.md](../design/propulsion_coeff_estimator.md) |
| `FlightLogReader` | `python/tools/log_reader.py` | ✅ Reworked — per-field MCAP topic parsing (`source/field`), CSV `source_name` column, stateful `frames()` getter, `channel_names()` raises before load (DR-1, DR-9) |
| `ModeEventSeries` | `python/tools/mode_overlay.py` | ✅ Reworked — initial value emitted as first event, `name_map` moved to constructor (DR-2) |
| `TimeHistoryFigure` | `python/tools/time_history.py` | ✅ Reworked — `figure()` public accessor with caching, `_build()` internal (DR-3) |
| `RibbonTrail` | `python/tools/trajectory_view.py` | ✅ Reworked — `wing_span_m` parameter, CCW winding, time-based α fade via `alpha_at()`, Vispy `MeshVisual` from `mesh()` (DR-8) |
| `HudOverlay` | `python/tools/trajectory_view.py` | ✅ Reworked — Vispy `Text` visuals in 2D overlay view, α-fading mode banner (DR-7, DR-8) |
| `TrajectoryView` | `python/tools/trajectory_view.py` | ✅ Reworked — `CameraMode` enum (FPV/TRAIL/GODS_EYE/LOCAL_TOP), Vispy `SceneCanvas` embedded in `QMainWindow`, `load_terrain()` via `pygltflib`, `set_terrain_saturation()`, headless `animate()` (DR-7 through DR-13) |
| `ManualInput` | `include/input/ManualInput.hpp` | ✅ Implemented — abstract base; `ManualInputFrame`, `InputAction`, `hasAction()` |
| `KeyboardInput` | `include/input/KeyboardInput.hpp` | ✅ Implemented — integrating keyboard adapter; configurable key bindings; injected key-state provider |
| `JoystickInput` | `include/input/JoystickInput.hpp` | ✅ Implemented — SDL2 joystick adapter; axis pipeline, dead zone, trim, disconnect fallback; `enumerateDevices()` |
| `ScriptedInput` | `include/input/ScriptedInput.hpp` | ✅ Implemented — mutex-protected command slot; `push()` callable from Python via pybind11 |
| `joystick_verify` | `tools/joystick_verify.cpp` | ✅ Implemented — SDL polling loop; JSON lines + `--proto` output; DEVICE/READY protocol |
| `liteaero_sim_py` | `src/python/` | ✅ Implemented — pybind11 module; `Aircraft` (via `PyAircraft`), `KinematicState`, `SimRunner`, `RunnerConfig`, `ExecutionMode`, `AircraftCommand`, `ScriptedInput`, `JoystickInput.enumerate_devices()` |
| `manual_input_monitor.py` | `python/tools/manual_input_monitor.py` | ✅ Implemented — shared library (`InputMonitorConfig`, `enumerate_devices`, `build_args`, `stdout_reader`, `draw_gauge`, `InputMonitorFigure`) + standalone Qt app (`InputMonitorWindow`, `main()`) |
| `manual_input_demo.ipynb` | `python/manual_input_demo.ipynb` | ✅ Implemented — thin notebook importing library; ipympl inline display via `FuncAnimation` |
| `AnomalyDetector` | `python/tools/anomaly.py` | 🔲 Planned |
| `BehaviorVerifier` | `python/tools/behavior_verifier.py` | 🔲 Planned |
| `DataOverlay` | `python/tools/data_overlay.py` | 🔲 Planned |

---

## Delivered

Design authority for all delivered items: [`docs/design/aircraft.md`](../design/aircraft.md).

| # | Item | Tests |
| --- | ------ | ------- |
| 1 | `AirframePerformance` — field renames, serialization (JSON + proto), `aircraft_config_v1` schema | `AirframePerformance_test.cpp` — 6 tests |
| 2 | `Inertia` — serialization (JSON + proto), `aircraft_config_v1` schema | `Inertia_test.cpp` — 6 tests |
| 3 | `Aircraft` class — `AircraftCommand`, `initialize()`, `reset()`, `state()` | `Aircraft_test.cpp` — 3 tests |
| 4 | `Aircraft::step()` — 9-step physics loop | `Aircraft_test.cpp` — 3 tests |
| 5 | `Aircraft` serialization — JSON + proto round-trips, schema version checks | `Aircraft_test.cpp` — 4 tests |
| 6 | JSON initialization — fixture-file tests (3 configs) and missing-field error path | `Aircraft_test.cpp` — 4 tests |
| 7 | `Logger` design — architecture, data model, MCAP + CSV formats, C++ interface reference | [`docs/design/logger.md`](../design/logger.md) |
| 8 | `Logger` implementation — `Logger`, `LogSource`, `LogReader`; MCAP + `FloatArray` proto; 6 tests | `test/Logger_test.cpp` — 6 tests |
| 9 | Environment model design — `Atmosphere` (ISA + ∆ISA + humidity), `Wind`, `Turbulence` (Dryden), `Gust` (1-cosine); rotational turbulence coupling to trim aero model defined | [`docs/design/environment.md`](../design/environment.md) |
| 10 | Aerodynamic coefficient estimation — derivation of all trim aero model inputs from wing/tail/fuselage geometry; DATCOM lift slope, Hoerner Oswald, Raymer $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$ | [`docs/algorithms/aerodynamics.md`](../algorithms/aerodynamics.md) |
| 11 | `Atmosphere` — ISA 3-layer + ΔT + humidity + density altitude; JSON + proto serialization | `Atmosphere_test.cpp` — 12 tests |
| 12 | `Wind` (Constant/PowerLaw/Log), `Turbulence` (Dryden 6-filter, Tustin-discretized), `Gust` (1-cosine MIL-SPEC-8785C); JSON serialization | `Wind_test.cpp` — 6 tests, `Turbulence_test.cpp` — 5 tests, `Gust_test.cpp` — 6 tests |
| 13 | `AeroCoeffEstimator` — geometry-to-coefficient derivation (Parts 1–8: AR, MAC, $C_{L_\alpha}$, $C_{L_\text{max}}$, Oswald $e$, $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$); `AeroPerformanceConfig` struct; four new `AeroPerformance` fields (`cl_q_nd`, `mac_m`, `cy_r_nd`, `fin_arm_m`); `AircraftGeometry`/`SurfaceGeometry` structs; `AeroPerformance` schema bumped to v2 | `AeroCoeffEstimator_test.cpp` — 11 tests |
| 14 | `Terrain` subsystem — `V_Terrain`, `FlatTerrain`, `TerrainMesh` (7 LODs, LOS, glTF export), `LodSelector`, `MeshQualityVerifier`, `SimulationFrame`/`TrajectoryFile`; Python ingestion pipeline (download, mosaic, geoid correction, triangulate, colorize, simplify, verify, export) | C++: 53 tests across 6 test files; Python: 28 tests pass + 1 skip — see [`terrain-implementation-plan.md`](terrain-implementation-plan.md) |
| 15 | `DynamicElement` refactoring — unified root base for all stateful components; `SisoElement` NVI SISO wrapper; `Filter` → `SisoElement`; `Propulsion` / `Motor` bases; deleted `DynamicBlock`, `DynamicFilterBlock`, `DynamicLimitBlock`, `V_Sensor`, `V_Propulsion`, `V_Motor` | No new tests — all 378 pre-existing tests pass |
| 16 | `SISOBlock` removal — deleted `SISOBlock.hpp`; `SisoElement` derives from `DynamicElement` only; `LimitBase`/`Limit`/`RateLimit`/`Integrator`/`Derivative`/`Unwrap` migrated to `SisoElement` with full `DynamicElement` lifecycle; `SisoElement::step()` NVI call order fixed (previous `in_` available during `onStep()`); `resetTo()` replaces `reset(float)` overloads | `Limit_test.cpp`, `RateLimit_test.cpp`, `Integrator_test.cpp`, `Derivative_test.cpp`, `Unwrap_test.cpp` — 10 new tests; all 394 tests pass |
| 17 | `Antiwindup` redesign — `AntiwindupConfig` struct with `enum class Direction`; `update(float)` replaces `operator=(float)`; `configure()`, `reset()`, `serializeJson()`/`deserializeJson()` added; `name` field removed; uninitialized-boolean bug fixed; `Integrator` serialization extended to embed `"antiwindup"` array | `Antiwindup_test.cpp` — 12 tests; `Integrator_test.cpp` — 2 new tests; all 408 tests pass |
| 18 | Control subsystem refactoring (Steps A–J, all nine issues in [`control_interface_review.md`](../design/control_interface_review.md)) — `FilterError`/`DiscretizationMethod` promoted to `enum class`; `LimitBase` deleted, `Limit`/`RateLimit` rebased to `SisoElement`; `Gain` API cleaned up (`set()`, `value()`, stubs removed); `FilterSS2Clip`, `FilterTF2`, `FilterTF`, `FilterFIR`, `FilterSS` migrated to NVI (`onStep()`/`onSerializeJson()`/`onDeserializeJson()`); shadow `_in`/`_out` members and no-op `Filter` defaults removed; `Unwrap` `ref_` field added (`setReference()`, NVI routing, serialization); `SISOPIDFF` derives from `DynamicElement` with full lifecycle, `snake_case_` member renames (`Kp`→`proportional_gain_`, `I`→`integrator_`, etc.), private limits, `ControlLoop` accessor renames (`out()`→`output()`, `pid`→`controller_`); `Integrator`/`Derivative` private member renames (`_dt`→`dt_s_`, `_Tau`→`tau_s_`, `limit`→`limit_`) | `FilterSS2Clip_test.cpp`, `FilterTF2_test.cpp`, `FilterFIR_test.cpp` (new), `FilterSS_test.cpp`, `Unwrap_test.cpp`, `SISOPIDFF_test.cpp` (new) — 29 new tests; 435 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 19 | `SensorAirData` — pitot-static air data computer; differential pressure ($q_c$) and static pressure ($P_s$) transducers with Gaussian noise, first-order Tustin lag, and fuselage crossflow pressure error (two-port symmetric crosslinked model); derives IAS, CAS, EAS, TAS, Mach, barometric altitude (Kollsman-referenced, troposphere + tropopause), OAT; RNG pimpl with seed + advance serialization; JSON + proto round-trips | `SensorAirData_test.cpp` — 19 tests; 454 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 20 | `LoadFactorAllocator` alpha-ceiling fix — Newton overshoot and fold guards corrected for positive-thrust case; the achievable-Nz ceiling is at $\alpha^*$ (where $f'(\alpha) = qSC_L'(\alpha) + T\cos\alpha = 0$), not at `alphaPeak()`, when $T > 0$; overshoot guard now clamps the proposed Newton step to the CL parabolic domain using `LiftCurveModel::alphaSep()` / `alphaSepNeg()` before checking $f'$, preventing escape into the flat separated plateau where $f' = T\cos\alpha$ stays positive until $\alpha > \pi/2$; bisects to locate $\alpha^*$ when the guard fires; fold guard stays at current iterate rather than snapping to `alphaPeak()`; `LiftCurveModel::alphaSep()` and `alphaSepNeg()` added to public interface; design documentation updated in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) and [`docs/algorithms/equations_of_motion.md`](../algorithms/equations_of_motion.md) | 4 new tests in `LoadFactorAllocator_test.cpp`; 19 tests total; 458 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 21 | **System architecture definition** — future-state system architecture model covering: originating requirements, use cases (UC-1 through UC-7), element registry (LiteAero Sim, LiteAero Flight, SimulationRunner, External Interface elements), data flow type and instance registries, interface control documents (ICD-8 through ICD-12), architectural decisions (30 recorded), open questions (all pre-design questions resolved; design-phase questions tracked); system boundary between LiteAero Sim simulation plant and LiteAero Flight established; Docker containerization model for SITL verification defined; `liteaero::` namespace structure and CMake target structure decided; repo split plan defined | No code tests — deliverable is the architecture document set under [`docs/architecture/system/future/`](../architecture/system/future/) |
| 22 | `LoadFactorAllocator` branch-continuation predictor — first-order warm-start $\alpha_0 = \alpha_\text{prev} + \delta n_z \cdot mg / f'(\alpha_\text{prev})$ and symmetric $\beta_0$ formula added to `solve()`; predictor is skipped at the stall ceiling ($f' \approx 0$) or when the raw prediction would fall outside $[\alpha_\text{sep\_neg}, \alpha_\text{sep}]$ (domain guard prevents cross-branch jumps on cold-start excess-demand calls); `_n_z_prev` and `_n_y_prev` added as serialized state fields in both JSON (`n_z_prev_nd`, `n_y_prev_nd`) and proto (`LoadFactorAllocatorState` fields 6–7); `iterations` (alpha solver iteration count) added to `LoadFactorOutputs`; `reset()` clears all four warm-start fields; [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Warm-Starting updated | 3 new tests in `LoadFactorAllocator_test.cpp`: `PredictorReducesIterationsOnLinearStep` (iterations == 1 for an exact linear-region prediction), `PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev`, `PredictorProtoRoundTrip_IncludesNzPrev`; 22 tests total |
| 23 | `LoadFactorAllocator` test coverage extension — 8 new tests close continuity and domain-coverage gaps identified by code review. **White-box tests** (4): full positive and negative Nz sweeps through stall verifying the clamp value against `alphaPeak()`/`alphaTrough()`; fine-step sweep across the C¹ Linear→IncipientStall boundary confirming both segments are traversed; stall warm-start limitation test documenting that `reset()` is required after a discontinuous Nz jump. **Black-box tests** (4): uniform 500-step monotonicity sweeps from 0 to ±10 g for T = 0 (positive and negative) and T = `kLargeThrust` (positive); point-wise perturbation test at 37 grid points (T = 0, −9 g to +9 g) and 19 grid points (T > 0, 0 to +9 g) using fresh allocators. Stall warm-start limitation documented in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Stall Warm-Start Limitation | 30 tests total in `LoadFactorAllocator_test.cpp` |
| 24 | **Aircraft command processing redesign** — all three command axes (`_nz_filter`, `_ny_filter`, `_roll_rate_filter`) converted to `setLowPassSecondIIR` (2nd-order LP); config params replaced: `cmd_deriv_tau_s`/`cmd_roll_rate_tau_s` → `nz_wn_rad_s`/`nz_zeta_nd`/`ny_wn_rad_s`/`ny_zeta_nd`/`roll_rate_wn_rad_s`/`roll_rate_zeta_nd`; `n_z_dot`/`n_y_dot` computed analytically from filter state after substep loop (no derivative filter lag); allocator receives shaped commands instead of raw clamped commands; Nyquist constraint enforced per axis (`wn * cmd_filter_dt_s < π`) at `initialize()`; proto `AircraftState` updated; `aircraft_config_v1` schema doc updated; fixture JSON files updated; design authority: [`docs/design/aircraft.md`](../design/aircraft.md) §Command Processing Architecture | 3 new Nyquist violation tests in `Aircraft_test.cpp` (`NyquistViolation_Nz_Throws`, `_Ny_Throws`, `_RollRate_Throws`); 345 pre-existing tests pass |
| 26 | **Post-processing tools design** — full design authority document covering: use case decomposition (UC-PP1 through UC-PP10); actors (Sim Developer, Integration Tester, Flight Analyst, Guidance/Autopilot Developer); module architecture (`FlightLogReader`, `AnomalyDetector`, `BehaviorVerifier`, `DataOverlay`, `ModeEventSeries`, `TimeHistoryFigure`, `RibbonTrail`, `HudOverlay`, `TrajectoryView`); ribbon trail geometry (body-to-world rotation, wing half-span vector, `Poly3DCollection`); HUD overlay layout; command/response time history encoding; `BehaviorVerifier` criterion library and multi-source DataFrame interface; library choices (pandas, matplotlib, Plotly, mcap, numpy); test strategy (8 test files, rendering non-invocation requirement) | [`docs/design/post_processing.md`](../design/post_processing.md) |
| 25 | **Landing gear model design** — full design authority document covering: use case decomposition; class hierarchy (`LandingGear`, `WheelUnit`, `StrutState`, `ContactForces`, `SurfaceFriction`, `SurfaceFrictionUniform`); physical models (suspension spring-damper, Pacejka magic formula, wheel friction, surface friction parameterization, ground plane interface); force assembly; step interface; JSON + proto serialization contract; computational resource estimate; test strategy (unit, integration, scenario, serialization); visualization notebook designs (`landing_gear_contact_forces.ipynb`, `crab_landing_dynamics.ipynb`, `takeoff_roll.ipynb`, `terrain_contact_animation.ipynb`); touchdown animation design (`touchdown_animation.py` — pybind11 driver, layout, visual encoding, coordinate mapping, data flow) | [`docs/design/landing_gear.md`](../design/landing_gear.md) |
| LG-1 | **LandingGear — C++ implementation** (Steps A–F) — `WheelUnit`, `StrutState`, `ContactForces`, `SurfaceFriction`, `SurfaceFrictionUniform`, `LandingGear`; quasi-static spring-damper strut; Pacejka magic formula tyre forces (longitudinal + lateral); friction-circle saturation; viscous wheel speed integration; nose wheel steering; differential braking; terrain elevation query via `V_Terrain`; wired into `Aircraft::step()` on the disturbance force path; `Aircraft::setTerrain()`, `contactForces()`, `weightOnWheels()` added; JSON + proto serialization throughout; `WheelUnitState`, `ContactForcesState`, `LandingGearState` proto messages added | `LandingGearTypes_test.cpp`, `SurfaceFriction_test.cpp`, `LandingGear_test.cpp`, `LandingGearTerrain_test.cpp` — 27 tests; `Aircraft_test.cpp` — 2 new tests; 388 total pass |
| Sim-1 | **SimRunner — Execution Modes** — `ExecutionMode` enum (`Batch`, `RealTime`, `ScaledRealTime`); `RunnerConfig` struct (`dt_s` float — output step, adequate precision for timestep values; `duration_s` double — needed for long runs compared to accumulated sim time; `time_scale` float; `mode`); `SimRunner` class with `initialize()`, `start()`, `stop()`, `is_running()`, `elapsed_sim_time_s()`; Batch mode blocks caller; RealTime/ScaledRealTime spawn `std::thread`; `std::atomic<uint64_t> step_count_` for elapsed time; termination condition `sim_time_s > duration_s + time_initial_s` (direct time comparison — no precomputed step count); `dt_s` widened to `double` once at loop entry for all arithmetic; late-step policy: no compensation; design authority: [`docs/design/sim_runner.md`](../design/sim_runner.md) | `SimRunner_test.cpp` — 10 tests |
| PP-1 | **Post-processing visualization tools — first-pass TDD implementation** — `FlightLogReader` (CSV loading via `pd.read_csv`, JSON-encoded MCAP via `mcap.reader.make_reader`, per-source DataFrames keyed by filename stem / `channel.topic`); `ModeEventSeries` (step-channel transition parser, `from_dataframe()` classmethod, `name_map` parameter, initial value skipped); `TimeHistoryFigure` (Plotly multi-panel, `shared_xaxes=True`, secondary y-axis via `specs`, `Scattergl` above 50 000 points, `add_vline()` mode overlays, `load(frames)` + `build()` + `export_html()`); `RibbonTrail` (ZYX Euler rotation matrices, wing body vector `[0, half_width_m, 0]`, `Poly3DCollection` quads, RdBu_r roll colormap via `TwoSlopeNorm(±π/3)`, midpoint-roll quad color); `HudOverlay` (9 fixed-position `text2D` artists, fading mode-change banner, 60-frame countdown); `TrajectoryView` (ghost ribbon α=0.15, live ribbon last 200 quads, dual-source overlay, `FuncAnimation(blit=False)`); `conftest.py` Agg backend; all design decisions subsequently resolved in dedicated design sessions; see DR-1 through DR-13 in [`post_processing.md`](../design/post_processing.md) | `test_log_reader.py` — 4 tests; `test_mode_events.py` — 5 tests; `test_time_history.py` — 6 tests; `test_ribbon_trail.py` — 5 tests; 20 tests total |
| PP-D | **Post-processing tools design — full architecture and decision records** — resolved all 28 design questions across both visualization and analysis layers; selections: Vispy (OpenGL, DR-8) for 3D rendering; PySide6 Qt window (DR-7) for playback and camera controls; Panel + Plotly (DR-10) for live time history; pre-generated glTF terrain via `pygltflib` (DR-11); offline terrain ingestion pipeline (DR-12); `terrain_paths.py` shared path module with `data/terrain/<dataset>/source/` + `derived/` repository structure (DR-13); per-field MCAP topic convention `"source/field_name"` (DR-9); `FlightLogReader` stateful API (DR-1); `ModeEventSeries` constructor name-map (DR-2); `TimeHistoryFigure.figure()` accessor (DR-3); ring buffer polling via pybind11 (DR-5, DR-6); camera modes FPV/Trailing/God's-eye/Local-top defined (PP-F28–PP-F32); terrain saturation runtime API (DR-13 area); document restructured from open-question tracking format to settled architecture with Decision Records appendix | [`docs/design/post_processing.md`](../design/post_processing.md) |
| PP-2 | **Post-processing visualization rework** (Tasks A–G, TDD) — `FlightLogReader`: per-field MCAP topic parsing (`"source/field_name"`), CSV `source_name` column, stateful `frames()` getter, `channel_names()` raises before load (DR-1, DR-9); `ModeEventSeries`: initial value emitted as first event, `name_map` moved to constructor (DR-2); `TimeHistoryFigure`: `figure()` public accessor with caching, `_build()` internal (DR-3); `RibbonTrail`: `wing_span_m` parameter, CCW quad winding, time-based α fade via `alpha_at()`, Vispy `MeshVisual` from `mesh()` (DR-8); `HudOverlay`: Vispy `Text` visuals in 2D overlay view, α-fading mode-change banner (DR-7, DR-8); `TrajectoryView`: `CameraMode` enum (FPV/TRAIL/GODS_EYE/LOCAL_TOP), Vispy `SceneCanvas` embedded in `QMainWindow`, `load_terrain()` via `pygltflib`, `set_terrain_saturation()`, headless `animate()` returns canvas (DR-7 through DR-13); `terrain_paths.py`: `get_terrain_data_root()`, `dataset_dir()`, `source_dir()`, `derived_dir()`, `las_terrain_dir()`, `gltf_path()`, `metadata_path()`; `vispy>=0.14`, `pyside6>=6.6`, `pygltflib`, `mcap-protobuf-support>=0.5` added to `pyproject.toml` | `test_log_reader.py` — 9 tests; `test_mode_events.py` — 6 tests; `test_time_history.py` — 6 tests; `test_ribbon_trail.py` — 9 tests; `test_terrain_paths.py` — 11 tests; `test_trajectory_view.py` — 14 tests; 55 tests total; full suite 150 pass |
| SB-1 | **Aircraft and SimRunner — Python Bindings** — `KinematicState` (read-only class; 14 scalar attributes: `time_s`, `latitude_rad`, `longitude_rad`, `altitude_m`, `velocity_north/east/down_mps`, `heading_rad`, `pitch_rad`, `roll_rad`, `alpha_rad`, `beta_rad`, `airspeed_m_s`, `roll_rate_rad_s`); `Aircraft` Python class (wraps `PyAircraft` — owns `Propulsion` + `Aircraft`, tracks simulation time; `__init__(config, dt_s=0.02)` accepts JSON string or file path; `"propulsion"` section selects `PropulsionJet`, `PropulsionEDF`, or `PropulsionProp`+`MotorElectric`/`MotorPiston` — absent section uses zero-thrust stub; `reset()`, `step(cmd, dt_s, rho_kgm3)`, `state()`); `RunnerConfig(dt_s, duration_s, time_scale, mode)` — mode accepted as string (`"batch"`, `"realtime"`, `"scaled_realtime"`); `SimRunner` — `initialize(config, aircraft)`, `start()` / `stop()` (GIL released), `is_running()`, `elapsed_sim_time_s()`; `py::keep_alive<1,3>` on `initialize()` prevents Aircraft GC; `bind_aircraft.cpp`, `bind_runner.cpp`, `py_aircraft_types.hpp` added to `src/python/`; design authority: [`docs/design/python_bindings.md`](../design/python_bindings.md) — Aircraft and SimRunner section | `test_aircraft_bindings.py` — 12 tests; `test_runner_bindings.py` — 13 tests; 25 tests total; full Python suite 211 passed, 1 skipped |
| SB-2 | **SimRunner Live Ring Buffer** — `Sample` struct (`time_s`, `value`); `ChannelSubscriber` (RAII, per-subscriber circular buffer, `channel_name()`, `drain()` returns batches and resets, `write()` called under registry mutex; silent overflow drops oldest; `~ChannelSubscriber()` calls `registry_->unsubscribe()`); `ChannelRegistry` (`register_channel(name, sample_rate_hz, depth_s)` idempotent; `subscribe(name)` → `shared_ptr<ChannelSubscriber>` empty buffer / no backfill (PP-F37); `publish(name, time_s, value)` fan-out to all subscribers (PP-F36); `available_channels()`; registry-mutex → subscriber-mutex lock ordering prevents deadlock); `SimRunner` extended with `ChannelRegistry registry_` member and `channel_registry()` accessor; `initialize()` registers 14 kinematic channels at `1/dt_s` Hz, 60 s depth; `runLoop()` publishes all 14 after each `Aircraft::step()`; channel names `kinematic/time_s` … `kinematic/roll_rate_rad_s`; pybind11: `Sample`, `ChannelSubscriber`, `ChannelRegistry` bound in `bind_ring_buffer.cpp`; `channel_registry()` added to `SimRunner` binding in `bind_runner.cpp` (re-opening `py::class_<SimRunner>` raises "already defined" — method added in the original binding instead); design authority: [`docs/design/ring_buffer.md`](../design/ring_buffer.md) | C++: `RingBuffer_test.cpp` — 19 tests (registration, subscribe, write/drain, overflow, late-join, multi-subscriber, RAII, thread-safety); Python: `test_ring_buffer_bindings.py` — 13 tests; full C++ suite 440 pass; full Python suite 224 passed, 1 skipped |
| MI-1 | **Manual Input — Full Subsystem** — `ManualInput` abstract base; `KeyboardInput` (integrating keyboard adapter, configurable scancodes, action keys, injected key-state provider); `JoystickInput` (SDL2 adapter, axis pipeline: calibration/trim/normalization/dead-zone/scale, per-axis `raw_min`/`raw_max`/`raw_trim`, inversion, disconnect fallback, `captureTrim()`, `enumerateDevices()`); `ScriptedInput` (mutex-protected slot, `push(AircraftCommand)`); `SimRunner::setManualInput()` / `lastManualInputFrame()` with `SDL_WasInit` guard; `joystick_verify` tool (DEVICE/READY protocol, JSON lines + `--proto` output, `ManualInputFrameProto`/`AircraftCommandProto` added to `liteaerosim.proto`); MinGW runtime linked statically + SDL2.dll deployed alongside executable at build time; `manual_input_monitor.py` shared library + standalone Qt app (`InputMonitorConfig`, `InputMonitorFigure`, `InputMonitorWindow`, `_subprocess_env()`; persistent gauge artists — no `ax.clear()` on update); `manual_input_demo.ipynb` thin notebook using ipympl + `FuncAnimation`; `gx12_config.json` Radiomaster GX12 axis mapping; `liteaero_sim_py` pybind11 module (`AircraftCommand`, `ScriptedInput`, `JoystickInput.enumerate_devices()`); SDL2 and pybind11 added to `conanfile.txt` and dependency registry; `ipympl>=0.9` added to `pyproject.toml`; design authority: [`docs/design/manual_input.md`](../design/manual_input.md), [`docs/design/sim_runner.md`](../design/sim_runner.md), [`docs/design/python_bindings.md`](../design/python_bindings.md) | C++: `KeyboardInput_test.cpp` — 10 tests; `JoystickInput_test.cpp` — 15 tests; `ScriptedInput_test.cpp` — 4 tests; `SimRunner_test.cpp` — 4 new manual input tests; Python: `test_manual_input_bindings.py` — 7 tests; `test_manual_input_monitor.py` — 29 tests |
| LS-1 | **Live Simulation Viewer** — `tools/live_sim.cpp` (C++ joystick launcher) implemented; loads `TerrainMesh` from `las_terrain_path` in `terrain_config.json` and calls `aircraft.setTerrain()`; `build_terrain.py` writes `las_terrain_path` to `terrain_config.json` (TDD — 3 new tests); `V_Terrain` renamed to `Terrain` across liteaero-flight and liteaero-sim (`Terrain.hpp` created; `V_Terrain.hpp` replaced with `#error` tombstone; all headers, sources, tests, and CMakeLists comments updated); design authority: [`docs/design/live_sim_view.md`](../design/live_sim_view.md) | Python: `test_build_terrain.py` — 3 new tests for `las_terrain_path` (field presence, dataset name in path, absolute path); all prior LS-1 tests unchanged |
| GP-1 | **Godot Plugin GDExtension** — Steps 1–8: `visualization` section added to three aircraft config fixtures + `aircraft_config_v1.md` schema doc; `build_terrain.py` writes `aircraft_mesh_path` to `terrain_config.json` (TDD — tests written first); `TerrainLoader.gd` rewritten with `_load_aircraft_mesh()`, `_find_vehicle()`, dual-mode `_find_simulation_receiver()` (native GDExtension + GDScript placeholder) and `_set_world_origin()`; static `AircraftMesh` node removed from `World.tscn`; `register_types.hpp`, `register_types.cpp` (WSAStartup/WSACleanup + ClassDB registration), `SimulationReceiver.hpp`, `SimulationReceiver.cpp` (non-blocking UDP socket, `SimulationFrameProto::ParseFromArray`, ENU→Godot position + NED→Godot quaternion); `liteaero_sim.gdextension` manifest; `godot/assets/aircraft_lp.glb` copied; build system (`LITEAERO_SIM_BUILD_GODOT_PLUGIN=ON`) already in place; design authority: [`docs/design/godot_plugin.md`](../design/godot_plugin.md) | Manual integration test checklist (see GP-1 §Tests); Python: `test_build_terrain.py` — 2 new integration tests + 4 new unit tests for `_build_terrain_config()` |

---

## LS-1. Live Simulation Viewer

**Blocking dependencies:** SB-1 (Aircraft and SimRunner Python bindings), MI-1
(delivered — `JoystickInput`, `ScriptedInput`, `AircraftCommand`).

**SB-2 (ring buffer) is not a blocking dependency.** The MVP rendering path is
`SimRunner → SimulationFrame → UDP → Godot 4`. SB-2 is not in that path.

**Note — PP-2 is not a blocking dependency.** The Vispy post-processing infrastructure
(`TrajectoryView`, `RibbonTrail`, `HudOverlay`) is not used for live rendering. The live
viewer is an independent pipeline.

Implements the minimum viable product live simulation: a **Godot 4** 3D scene displaying
terrain, aircraft model, ribbon trail, and shadows, driven by the FBW joystick or
scripted input channel, launchable from CLI or notebook. This is the first item where the
full loop closes: C++ `SimRunner` drives `Aircraft` step-by-step in `RealTime` mode;
after each step it populates a `SimulationFrame` and broadcasts it via UDP to a fixed
local port; the Godot 4 scene receives the datagrams via a GDExtension plugin and updates
the vehicle actor transform each render frame. `JoystickInput` or `ScriptedInput` delivers
commands at the FBW interface via Python `SimSession`.

The renderer is Godot 4 (MIT license) — the architectural decision made during terrain
mesh implementation; see [`docs/design/terrain.md §Game Engine Integration`](../design/terrain.md)
and [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
decision row 26.

### Deliverables — Live Simulation Viewer Design Document

~~Produce `docs/design/live_sim_view.md` — delivered; see
[`docs/design/live_sim_view.md`](../design/live_sim_view.md).~~

**Open questions in the design document requiring resolution before implementation:**

- **OQ-LS-4** — Aircraft mesh asset sourcing: ~~resolved — `python/assets/aircraft_lp.glb`
  committed (323 triangles).~~
- **OQ-LS-5** — `JoystickInput` wiring approach: ~~resolved — Option A (C++ `live_sim`
  binary owns `JoystickInput` directly; no Python binding required).~~
- **OQ-LS-6** — Float precision of ring buffer position channels: deferred to SB-3
  (ring buffer redesign). Not blocking for LS-1 — Godot receives `double` geodetic
  directly in `SimulationFrame`; ring buffer is not in the MVP rendering path.
- **OQ-LS-7** — UDP datagram serialization format: ~~resolved — Option B (protobuf
  `SimulationFrameProto` added to `liteaerosim.proto`).~~
- **OQ-LS-8** — Godot scene launch mechanism: ~~resolved — Option C (developer opens
  Godot manually; Python/C++ broadcast regardless).~~
- **OQ-LS-9** — Aircraft mesh coordinate frame correction: ~~resolved — Option B (fixed
  `rotation_degrees` on `AircraftMesh` node in Godot scene; no mesh re-export).~~

**Architectural corrections — delivery status:**

1. ~~Implement `SimulationFrame` value struct and `ISimulationBroadcaster` interface.~~ ✅
2. ~~Implement `UdpSimulationBroadcaster`.~~ ✅
3. ~~Add `SimRunner::set_broadcaster()` and broadcast after each `step()`.~~ ✅
4. ~~Bind `SimRunner.set_broadcaster()` in `bind_runner.cpp`.~~ ✅
5. ~~`JoystickInput` wiring — resolved by OQ-LS-5 (C++ binary).~~ ✅
6. ~~Godot project and GDScript placeholder exist. GDExtension C++ plugin pending GP-1.~~ ✅ GP-1 delivered.

### Scope — Live Simulation Viewer

- ~~**`SimulationFrame`** — plain value struct; position, attitude, velocity NED.~~ ✅
- ~~**`UdpSimulationBroadcaster`** — UDP unicast; sends `SimulationFrameProto` to port
  14560 after each `SimRunner` step.~~ ✅
- ~~**`SimSession`** — Python class; owns `Aircraft`, `SimRunner`,
  `UdpSimulationBroadcaster`; wires manual input; starts in `RealTime` mode.~~ ✅
- ~~**CLI launcher** — `python/tools/live_sim.py` (scripted/notebook); `tools/live_sim.cpp` (joystick + `TerrainMesh` wiring).~~ ✅
- ~~**Terrain build pipeline** (`build_terrain.py`) — produces terrain GLB and
  `terrain_config.json` sidecar; terrain dataset built and in place.~~ ✅
- ~~**Godot 4 scene** — `World.tscn` basic structure exists; aircraft mesh and
  GDExtension plugin pending GP-1.~~ ✅ GP-1 delivered — see [GP-1](#gp-1-godot-plugin-gdextension-implementation).
- ~~**Aircraft mesh** — `python/assets/aircraft_lp.glb` committed; mesh wiring and
  configurable-per-aircraft-config path pending GP-1.~~ ✅
- ~~**LandingGear terrain interaction** — `live_sim.cpp` loads `TerrainMesh` from `las_terrain_path` in `terrain_config.json`; calls `aircraft.setTerrain()` before `runner.initialize()`.~~ ✅

### Tests — Live Simulation Viewer

C++ (`test/SimulationBroadcaster_test.cpp`) — 3 tests: `SimulationFrame` size;
`UdpSimulationBroadcaster` sends a loopback datagram; `SimRunner` broadcasts after step.

Python (`python/test/test_live_sim_session.py`) — 6 tests: `SimSession` initialize /
start / stop; broadcasts ≥ 1 datagram after run; datagram size matches `SimulationFrame`;
scripted input wiring; CLI argument parsing.

---

## GP-1. Godot Plugin GDExtension Implementation

**Blocking dependencies:** LS-1 design complete (delivered). Terrain dataset built
(`godot/terrain/terrain.glb` and `terrain_config.json` in place). `live_sim.exe`
built.

**Design authority:** [`docs/design/godot_plugin.md`](../design/godot_plugin.md)

The GDScript placeholder (`SimulationReceiver.gd`) is not used for the first live
simulation test. The C++ GDExtension is implemented directly. The GDScript file
remains in the repository until the GDExtension is confirmed working, then is
removed.

### Deliverables — Godot Plugin GDExtension Implementation

**All 8 steps delivered.**

#### ~~Step 1 — Aircraft config `visualization` section~~ ✅

Add `"visualization": {"mesh_res_path": "res://assets/aircraft_lp.glb"}` to:

- `configs/general_aviation_ksba.json`
- `configs/jet_trainer_ksba.json`
- `configs/small_uas_ksba.json`

Update `docs/schemas/aircraft_config_v1.md` to document the new `visualization`
section and its `mesh_res_path` field.

#### ~~Step 2 — `build_terrain.py`: write `aircraft_mesh_path` to `terrain_config.json`~~ ✅

Read `config.get("visualization", {}).get("mesh_res_path", "res://assets/aircraft_lp.glb")`
and write it as `aircraft_mesh_path` in the `terrain_config` dict. Regenerate
`godot/terrain/terrain_config.json` once to pick up the new field.

#### ~~Step 3 — `TerrainLoader.gd`: aircraft mesh loading and GDExtension compatibility~~ ✅

- Add `_load_aircraft_mesh(config)`: reads `aircraft_mesh_path` from config; loads
  via `ResourceLoader`; instantiates as child of `Vehicle` node; applies
  `rotation_degrees = Vector3(0, 90, 0)` (OQ-LS-9 body-frame correction).
- Add `_find_vehicle(node)`: depth-first search for the `Vehicle` node by name.
- Update `_find_simulation_receiver()`: check `node.get_class() == "SimulationReceiver"`
  (native C++ type) first, then fall back to script path check (GDScript placeholder).
- Update `_set_world_origin()`: call `receiver.set_world_origin(lat, lon, h)` when
  native type is detected; use direct property assignment for GDScript placeholder.

#### ~~Step 4 — `World.tscn`: remove static `AircraftMesh` node~~ ✅

Remove the `MeshInstance3D` `AircraftMesh` placeholder. `TerrainLoader` instantiates
the mesh at runtime.

#### ~~Step 5 — GDExtension C++ source files~~ ✅

Per [`docs/design/godot_plugin.md`](../design/godot_plugin.md):

- `godot/addons/liteaero_sim/src/register_types.hpp`
- `godot/addons/liteaero_sim/src/register_types.cpp` — `liteaero_sim_init` entry
  point; `WSAStartup` / `WSACleanup` on Windows; `ClassDB::register_class<SimulationReceiver>()`.
- `godot/addons/liteaero_sim/src/SimulationReceiver.hpp` — `Node3D` subclass;
  exported properties `broadcast_port`, `max_datagrams_per_frame`; `set_world_origin()`
  bound method; non-blocking UDP socket.
- `godot/addons/liteaero_sim/src/SimulationReceiver.cpp` — `_ready()` opens socket;
  `_process()` drains datagrams; `_apply_frame()` calls
  `SimulationFrameProto::ParseFromArray()`, computes ENU offset, sets `Vehicle`
  position and quaternion.

#### ~~Step 6 — `.gdextension` manifest~~ ✅

Create `godot/addons/liteaero_sim/liteaero_sim.gdextension` declaring
`entry_symbol = "liteaero_sim_init"` and
`windows.release.x86_64 = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"`.

#### ~~Step 7 — Build system~~ ✅

`CMakeLists.txt` already has `LITEAERO_SIM_BUILD_GODOT_PLUGIN=OFF` option and
`add_subdirectory(godot/addons/liteaero_sim/src)` guard.
`godot/addons/liteaero_sim/src/CMakeLists.txt` already exists with version-parsing
and error-handling logic. No further CMake changes required.

#### ~~Step 8 — Aircraft mesh asset placement~~ ✅

Copy `python/assets/aircraft_lp.glb` → `godot/assets/aircraft_lp.glb` so Godot
can find it at `res://assets/aircraft_lp.glb`.

### Build and First Run

```bash
# Configure with Godot plugin enabled
PATH="/c/msys64/ucrt64/bin:$PATH" cmake -B build -G "MinGW Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON

# Build (includes liteaero_sim_gdext.dll)
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build

# Regenerate terrain_config.json (picks up aircraft_mesh_path)
cd python && uv run python tools/terrain/build_terrain.py \
    --config ../configs/general_aviation_ksba.json

# Start physics broadcaster
../build/tools/live_sim.exe --config ../configs/general_aviation_ksba.json --dt 0.02

# Open Godot editor → godot/ → play World.tscn
```

### Tests — Godot Plugin GDExtension

The GDExtension has no automated test suite at this stage. The first-run
integration test is manual:

| Check | Pass criterion |
| --- | --- |
| Build | `liteaero_sim_gdext.dll` present in `godot/addons/liteaero_sim/bin/` |
| Plugin load | Godot editor opens without "GDExtension API mismatch" or "failed to load" errors |
| Terrain | Terrain tiles appear at scene start; LOD switches visible as camera moves |
| Aircraft mesh | Aircraft GLB appears at the world origin; correct orientation (nose forward) |
| Position tracking | Aircraft moves in response to `live_sim.exe` UDP datagrams |
| Attitude tracking | Aircraft rotates correctly with attitude changes |

Automated Godot scene tests are a future item once the GDTest framework is
evaluated.

---

## SB-3. Ring Buffer Redesign

**Blocking dependencies:** SB-2 (delivered — existing ring buffer implementation). SB-3
must be complete before any roadmap item that introduces a Python-side live telemetry
consumer (PP-3, PP-4, or any notebook that reads ring buffer position data). It is not
required for the Godot 3D rendering path.

**Design authority:** [`docs/design/ring_buffer.md`](../design/ring_buffer.md)
(to be revised — the revised document is the primary deliverable of this item).

The existing SB-2 ring buffer stores a scalar `float` value per sample (`Sample::value`).
This is insufficient for three reasons: (a) position channels (`latitude_rad` ≈ 0.7 rad)
have 0.53 m quantization at float32 precision — visible jitter in any Python position
consumer; (b) vector and quaternion channels require multiple scalar channels with no
atomicity guarantee between them, so a consumer draining a partial quaternion across a
step boundary receives meaningless data; (c) sensor, estimator, and guidance outputs will
need `double`, `Eigen::Vector3f`, and `Eigen::Quaternionf` types. The redesign resolves
these problems by choosing a type policy and specifying the migration path from SB-2.

### Deliverables — Ring Buffer Redesign

Revised [`docs/design/ring_buffer.md`](../design/ring_buffer.md) covering:

- **Type policy decision:** choose between (a) upgrading `Sample::value` from `float` to
  `double` (simple, fixes precision, still scalar); (b) typed `Channel<T>` with type
  erasure in the registry (supports vector/quaternion, higher complexity); or (c) a
  hybrid (scalar double registry + a separate structured-sample channel type).
- **Breaking change strategy:** SB-2 is already implemented and tested; the redesign
  must specify which APIs change, which tests are replaced, and what the migration path
  is.
- **Updated channel table:** revise the 14 kinematic channels registered by `SimRunner`
  to use the new type policy. Address OQ-LS-6 (position precision) as part of this
  channel table revision.
- **Python binding impact:** any type changes affect `bind_ring_buffer.cpp` and
  `test_ring_buffer_bindings.py`.

### Tests — Ring Buffer Redesign

None — this is a design-only item. Implementation of the redesigned ring buffer is a
separate follow-on item requiring an implementation plan.

---

## TB-1. Terrain Build Tool

**Blocking dependencies:** LS-1 (live simulation viewer delivered — establishes the
Godot scene that consumes the terrain GLB). No open questions block implementation —
OQ-TB-1, OQ-TB-2, OQ-TB-3, and OQ-TB-5 are all resolved in the design authority
document. OQ-TB-4 (triangulation mesh quality) is open but does not block this item.

**Design authority:** [`docs/design/terrain_build.md`](../design/terrain_build.md).

A single-function automation layer over the existing terrain ingestion tools
(`python/tools/terrain/`). Given an aircraft configuration JSON path, it derives all
pipeline parameters — geographic center, bounding box, cruise speed, 10-minute radius,
altitude-adaptive DEM resolution, download chunk count — and produces a complete terrain
dataset (`.las_terrain` + GLB) without further user interaction.

### Deliverables — Terrain Build Tool

1. **`python/tools/terrain/download.py`** — add `resolution_deg: float | None = None`
   keyword argument to `download_dem` and `download_imagery` (per OQ-TB-1 resolution);
   update `_bbox_tag` to include resolution so cache entries at different resolutions do
   not collide.
2. **`python/tools/terrain/build_terrain.py`** — `build_terrain(aircraft_config_path, *,
   name, radius_m, dem_source, imagery_source, force)` entry point as specified in
   design authority §Entry Point.
3. **`data/terrain/<name>/derived/metadata.json`** — provenance record written on
   successful build; schema in design authority §Output Layout.

### Tests — Terrain Build Tool

- `python/test/test_build_terrain.py` — synthetic flat DEM (no network); mocked
  `download_dem`/`download_imagery`; verifies parameter derivation from each of the
  three aircraft config fixtures; verifies `metadata.json` content; verifies that
  `force=True` removes cached files; verifies that zero-velocity config raises
  `ValueError`.

---

## 1. Sensor Models — Implementable Subset

**Blocking dependencies:** None for the subset below. `KinematicState` and `Terrain`
are implemented. Each sensor requires its own design document (with all open questions
resolved) before implementation can begin. The remaining sensors (`SensorINS`, `SensorAA`,
`SensorAAR`, `SensorForwardTerrainProfile`, `SensorTrackEstimator`) are deferred to
item 9; they depend on LiteAero Flight components not yet designed.

**Design authority:** No design documents yet — a design document is the first deliverable
for each sensor below. Implement in the order listed; `SensorLaserAlt` and `SensorRadAlt`
outputs are required by the `AnomalyDetector` `AltitudeBelowTerrain` rule (item 6).

The four sensors whose only C++ dependencies are already available:

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | `Terrain` (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | `Terrain` (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |

### Deliverables — Sensor Models Implementable Subset

For each sensor in the table above:

- Design document in `docs/design/` with all open questions resolved.
- C++ header `include/sensor/Sensor<Name>.hpp` and implementation `src/sensor/Sensor<Name>.cpp`.
- JSON + proto serialization and round-trip tests.
- Wired into `Aircraft::step()` and registered in the `ChannelRegistry`.

### Tests — Sensor Models Implementable Subset

One test file per sensor: `test/Sensor<Name>_test.cpp`. Minimum test categories: unit
(noise model statistics), integration (driven by `KinematicState` from a recorded
trajectory), and serialization (JSON + proto round-trip).

---

## 2. Logged Channel Registry — Design

**Blocking dependencies:** SimRunner (delivered), LandingGear C++ (delivered), item 1
(sensor models subset — registry must reflect complete channel set including sensor
channels).

**Design authority:** `docs/design/channel_registry.md` (to be produced — the
design document is the sole deliverable of this item).

A formal specification of all `LogSource` registrations in the simulation loop. Required
before `AnomalyDetector` rules and `BehaviorVerifier` criteria can reference channel
names, and before the ring buffer channel vocabulary (SB-2/SB-3) can be aligned with
the logger vocabulary (post_processing.md §DR-9).

### Deliverables — Logged Channel Registry

Design document (`docs/design/channel_registry.md`) specifying:

- All `LogSource` instances registered by the simulation loop (e.g., `"aircraft"`,
  `"environment"`, `"landing_gear"`, `"sensors"`).
- For each source: the complete channel list with SI-unit-suffixed names, data type, and
  sampling rate.
- Naming conventions for channels from `KinematicStateSnapshot`, `AircraftState`,
  `ContactForces`, `AtmosphericState`, and each sensor output struct.
- Policy for how new subsystems register channels as they are added.

### Tests — Logged Channel Registry

None — design-only item. Implementation of `LogSource` registrations is part of item 6.

---

## 3. Real Flight Log Format — Design

**Blocking dependencies:** None. This is a standalone design decision.

**Design authority:** `docs/design/flight_log_format.md` (to be produced — the
design document is the sole deliverable of this item).

A design decision document defining how real aircraft flight logs are loaded by
`DataOverlay`. Must be resolved before `DataOverlay` can be implemented (item 6).

### Deliverables — Real Flight Log Format

Design document (`docs/design/flight_log_format.md`) covering:

- What log format(s) real aircraft produce (e.g., ArduPilot DataFlash, MAVLink ULOG,
  custom CSV, or a configurable adapter).
- Channel name mapping from the real-log format to the simulation channel naming
  convention defined in item 2.
- Policy for handling channels present in the real log but absent from the sim schema,
  and vice versa.
- Whether a translation/adapter layer is implemented in `FlightLogReader` or as a
  separate preprocessing step.

### Tests — Real Flight Log Format

None — design-only item. Implementation of `DataOverlay` format adapter is part of item 6.

---

## 4. Aerodynamic Coefficient Design Study

**Blocking dependencies:** None. `AeroCoeffEstimator` is implemented. Must be complete
before item 5 (`Aircraft6DOF`) begins, and before Cfg-1 can be scoped.

**Design authority:**
- [`docs/design/aero_coefficient_model.md`](../design/aero_coefficient_model.md) — coefficient model format, sign conventions, propulsion integration
- [`docs/design/aero_coeff_estimator.md`](../design/aero_coeff_estimator.md) — estimation methods and `AeroCoeffEstimator` extension
- [`docs/design/propulsion_coeff_estimator.md`](../design/propulsion_coeff_estimator.md) — propulsion parameter estimation and propulsion-aero coupling

A design study that resolves how aerodynamic and propulsion coefficients are specified
for `Aircraft6DOF`. Defines `BodyAxisCoeffModel` format, sign conventions, and parameter
estimation pipeline. Resolves OQ-16(c).

### Deliverables — Aerodynamic and Propulsion Coefficient Design Study

- `aero_coefficient_model.md` completed and all open questions resolved.
- `aero_coeff_estimator.md` completed and all open questions resolved.
- `propulsion_coeff_estimator.md` completed and all open questions resolved.
- `AircraftGeometry` + `PropulsionGeometry` JSON files and coefficient tables for Cases A, B, and C.
- `BodyAxisCoeffModel` and `PropulsionCouplingCoefficients` formats decided and documented.
- Element registry updated (`AeroModel` named, `BodyAxisCoeffModel` and `PropulsionCoeffEstimator` formats defined).

### Tests — Aerodynamic Coefficient Design Study

None — design-only item. Implementation of `BodyAxisCoeffModel` and `AeroModel` is item 5.

---

## 5. Aircraft6DOF — Design and Implementation

**Blocking dependencies:** Item 4 (aerodynamic coefficient design study).

**Design authority:** Design document to be produced as the first deliverable of this
item. Architecture placeholders are defined in
[`docs/architecture/system/future/element_registry.md`](../architecture/system/future/element_registry.md).

### Scope — Aircraft6DOF

- **`AeroModel`** — abstract aerodynamic model interface; produces forces and moments in
  body frame; decouples the 6DOF integrator from the coefficient axis convention.
- **`BodyAxisCoeffModel`** — implements `AeroModel` using body-axis stability derivatives
  (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular
  rates. Coefficient model format defined by item 4.
- **`Aircraft6DOF`** — full 6DOF dynamics; depends on `AeroModel` for forces and moments;
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

- Design authority document (`docs/design/aircraft_6dof.md`) with all open
  questions resolved.
- `AeroModel` abstract interface header and implementation skeleton.
- `BodyAxisCoeffModel` implementing `AeroModel`; JSON + proto serialization.
- `Aircraft6DOF` class; JSON + proto serialization; wired to `LandingGear` and
  `SimRunner`.
- `FBWController` and `SurfaceDeflectionCommand`.

### Tests — Aircraft6DOF

`test/Aircraft6DOF_test.cpp` — unit, integration, and serialization categories per the
design authority document test strategy. `Aircraft6DOF` and `Aircraft` must both produce
identical `KinematicStateSnapshot` fields for a trimmed cruise case (transparent
substitution test).

---

## 6. Post-Processing — Analysis Tools Design Harmonization and Implementation

**Blocking dependencies:** Item 2 (Logged Channel Registry), item 3 (Real Flight Log
Format), and LiteAero Flight command channel schema (cross-repo dependency — track in
LiteAero Flight roadmap).

**Design authority:** [`docs/design/post_processing.md`](../design/post_processing.md)
§Analysis Modules.

A second design pass on the analysis modules in `post_processing.md` to replace
placeholder channel names with concrete names from the channel registry (item 2), add
the real flight log format adapter to `DataOverlay` (item 3), and incorporate the LiteAero
Flight command channel schema into `BehaviorVerifier` criteria. Implementation follows
the updated design.

### Deliverables — Analysis Tools

Design harmonization:

- Update `post_processing.md` §Analysis Modules: replace placeholder channel names with
  names from the Logged Channel Registry; specify `DataOverlay` format adapter per item 3;
  define `BehaviorVerifier` command channel names from the LiteAero Flight command schema;
  define the Scenario Reference Data Format for `WaypointReached` and similar criteria.

Implementation (in dependency order):

| Module | File | Blocked by |
| --- | --- | --- |
| `AnomalyDetector` + rule library | `python/tools/anomaly.py` | Item 2, item 1 sensor models |
| `DataOverlay` | `python/tools/data_overlay.py` | Item 3 |
| `BehaviorVerifier` + criterion library | `python/tools/behavior_verifier.py` | Item 2, LiteAero Flight schema |

### Tests — Analysis Tools

`python/test/test_anomaly.py` — rule evaluation, sensor channel names from registry.
`python/test/test_data_overlay.py` — real log format loading, channel mapping.
`python/test/test_behavior_verifier.py` — criterion evaluation, command channel names.

---

## 7. LandingGear — Python Bindings and Scenario Tests

**Blocking dependencies:** LandingGear C++ (delivered, LG-1), PP-1 (delivered —
visualization tools exist for animation). PP-2 (rework) is delivered. No open questions
block this item.

**Design authority:**
- [`docs/design/landing_gear.md`](../design/landing_gear.md) §Steps G–H and §Scenario Tests
- [`docs/design/python_bindings.md`](../design/python_bindings.md) §Landing Gear

Implements Steps G–H: expose `LandingGear`, `WheelUnit`, `StrutState`, and `ContactForces`
to Python via pybind11, rewrite `touchdown_animation.py` to use the C++ bindings, and
implement four pytest scenario tests using `FlatTerrain`.

### Deliverables — LandingGear Python Bindings and Scenario Tests

- `src/python/bind_landing_gear.cpp` — pybind11 bindings for `LandingGear`,
  `WheelUnit`, `StrutState`, `ContactForces`; registered in `src/python/bindings.cpp`
  and `src/python/CMakeLists.txt`.
- `python/scripts/touchdown_animation.py` rewritten to call `LandingGear::step()` via
  pybind11; Python-side contact physics removed.
- Four pytest scenario test files (Step H; all use `FlatTerrain` — accurate runway
  geometry deferred to LG-2).

### Tests — LandingGear Python Bindings and Scenario Tests

`python/test/test_landing_gear_bindings.py` — binding smoke tests: initialize, reset,
step; `StrutState` and `ContactForces` attribute access.

| Scenario test file | Pass criterion |
| --- | --- |
| `test_landing_rollout.py` | Aircraft decelerates to rest; max $F_z$ within 10% of analytical impulse estimate |
| `test_crab_landing.py` | Lateral drift eliminated within 3 s; no diverging yaw oscillation |
| `test_takeoff_roll.py` | All wheels leave ground at rotation speed; `weight_on_wheels` → false |
| `test_bounce.py` | Contact duration < 0.3 s; $F_z$ peak < 1.5× static weight |

---

## 8. External Interface Elements

**Blocking dependencies:** SimRunner (delivered) for all elements. Item 5 (Aircraft6DOF)
for `PX4Interface`. `NavigationState` from LiteAero Flight for `QGroundControlLink`.
Each element requires its own design document before implementation can begin.

**Design authority:** No design documents yet for most elements. `VisualizationLink`
transport and axis convention are decided — see
[`docs/design/terrain.md`](../design/terrain.md) §Game Engine Integration
and [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
§Game engine for real-time visualization.

Adapters that connect LiteAero Sim to external systems. All live in the Interface Layer.

| # | Element | Protocol | Depends on |
| --- | --- | --- | --- |
| LAS-ext-1 | `ArduPilotInterface` | ArduPilot SITL protocol | SimRunner (delivered); `Aircraft` or `Aircraft6DOF` |
| LAS-ext-2 | `PX4Interface` | PX4 SITL bridge | SimRunner (delivered); `Aircraft6DOF` (item 5) |
| LAS-ext-3 | `QGroundControlLink` | MAVLink over UDP | SimRunner (delivered); `NavigationState` (LiteAero Flight) |
| LAS-ext-4 | `VisualizationLink` | UDP to Godot 4 GDExtension plugin at simulation rate | SimRunner (delivered); `SimulationFrame` (done) |

### Deliverables — External Interface Elements

For each element: a design document; C++ header and implementation in the Interface
Layer; integration test demonstrating connection to the target external system.

### Tests — External Interface Elements

Per-element integration test. Automated unit tests covering protocol serialization;
connection and disconnection handled without blocking `SimRunner`.

---

## 9. Sensor Models — Deferred Subset

**Blocking dependencies:** LiteAero Flight components not yet designed (`NavigationFilter`
for `SensorINS`; Guidance for `SensorForwardTerrainProfile`). `SensorAA`, `SensorAAR`,
and `SensorTrackEstimator` require design work not yet started. Schedule each element
when its respective LiteAero Flight dependency is available.

**Design authority:** No design documents yet. Each sensor requires a design document
before implementation can begin.

| Class | Blocked by |
| --- | --- |
| `SensorINS` | `NavigationFilter` types (LiteAero Flight FC-8) |
| `SensorForwardTerrainProfile` | Guidance design (LiteAero Flight) |
| `SensorAA` | Design work item needed |
| `SensorAAR` | Design work item needed |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` |

### Deliverables — Sensor Models Deferred Subset

For each sensor: design document; C++ header and implementation; JSON + proto
serialization; wired into `Aircraft::step()`.

### Tests — Sensor Models Deferred Subset

One test file per sensor: `test/Sensor<Name>_test.cpp`. Same categories as item 1.

---

## 10. Synthetic Perception Sensors — Proposed

**Blocking dependencies:** Design documents needed for each sensor before implementation
can begin. Schedule when prerequisite sensor and terrain models are stable.

**Design authority:** No design documents yet.

| Element | Responsibility |
| --- | --- |
| `SensorCamera` | Synthetic image sensor; generates imagery from terrain and scene model against `Terrain` |
| `SensorLidar` | Synthetic lidar; generates 3D point cloud by ray-casting against `Terrain` |
| `SensorLaserAGL` | Synthetic laser altimeter; computes AGL range by ray-casting against `Terrain` |
| `SensorLineOfSight` | Computes RF link quality and terrain occlusion by ray-casting against `Terrain` |

### Deliverables — Synthetic Perception Sensors

For each sensor: design document; C++ header and implementation.

### Tests — Synthetic Perception Sensors

Per-sensor test file; scene-level integration test verifying ray-cast results against
a known synthetic terrain.

---

## Terrain-1. Rename `V_Terrain` to `Terrain` (liteaero-flight)

**Blocking dependencies:** None.

**Design authority:** [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
§Naming conventions (forbids `V_` Hungarian prefix).

`liteaero::terrain::V_Terrain` uses the forbidden `V_` Hungarian-notation prefix; the
correct name is `Terrain`. A cross-repository rename of the abstract base class in
liteaero-flight and all call sites in liteaero-sim.

### Deliverables — Terrain Rename

- `V_Terrain.hpp` in liteaero-flight renamed to `Terrain.hpp`; class renamed from
  `V_Terrain` to `Terrain`.
- All liteaero-sim headers (`include/Aircraft.hpp`, `include/landing_gear/LandingGear.hpp`,
  `include/environment/TerrainMesh.hpp`) and all `.cpp` files updated.
- `V_Terrain.hpp` replaced with a `#error` tombstone (pattern established by LS-1).
- Current State table, delivered item 14, item LG-1 description, and all sensor design
  documents that reference the interface by name updated.

### Tests — Terrain Rename

No new tests — all existing C++ tests must pass unchanged after the rename. The rename
is complete when `grep -r "V_Terrain" include/ src/ test/` returns zero results in
liteaero-sim.

---

## Log-1. Logging Subsystem — Architecture Design

**Blocking dependencies:** None. Resolves OQ-SR-3 in
[`docs/design/sim_runner.md`](../design/sim_runner.md). Must be complete
before item 2 (Logged Channel Registry) can finalize channel names, and before any
`Logger`-wired `SimRunner` can be implemented.

**Design authority:** `docs/design/logging_subsystem.md` (to be produced — the
design document is the sole deliverable of this item).

A formal specification of how all per-tick outputs from `SimRunner` are delivered to log
consumers. The mechanism must handle kinematic state, air data, manual input frames,
landing gear contact forces, and sensor outputs uniformly — partial solutions scoped to
a single output type are prohibited by OQ-SR-3.

### Deliverables — Logging Subsystem Architecture

Design document (`docs/design/logging_subsystem.md`) specifying:

- Delivery mechanism: push vs. pull; callback registration vs. ring buffer drain;
  relationship to SB-2 (ring buffer) and the existing `Logger` / `LogSource`
  infrastructure (delivered item 8).
- All per-tick outputs that must be logged: `KinematicStateSnapshot`,
  `EnvironmentState`, `AircraftState`, `ManualInputFrame`, `ContactForcesState`, and
  sensor output structs.
- Sampling policy: whether all channels share the `SimRunner` timestep or whether
  decimation is applied per source.
- Thread-safety contract: `SimRunner` writes from the simulation thread; consumers read
  from arbitrary threads.
- Integration with the Logged Channel Registry (item 2): how `Logger`-registered sources
  map to registry channel names and how that naming vocabulary is kept consistent with the
  SB-2 ring buffer channel names (see `post_processing.md` §DR-9).

### Tests — Logging Subsystem Architecture

None — design-only item. Implementation is a separate follow-on item.

---

## Config-1. Scenario Configuration Layering — Base Aircraft Models + Scenario Overlays

**Blocking dependencies:** None — may begin once the design document (the first deliverable)
is produced.

**Design authority:** `docs/design/scenario_configuration.md` (to be produced — defines the
reference and merge semantics; this design document is the first deliverable of this item).
References the field schema in [`docs/schemas/aircraft_config_v1.md`](../schemas/aircraft_config_v1.md).

A configuration system in which a scenario-specific config (a particular aircraft at a
particular runway or location) **references a base aircraft model** and carries **only the
values it adds or overrides** — never a copy of the aircraft's aerodynamic, inertia, or
lift-curve coefficients. Changing the location, or attaching scenario elements (terrain,
landing gear, body collider, visualization, initial state), must not duplicate the base
model's physics parameters; each aircraft's physics has a single source of truth.

Motivation: the current `configs/*_ksba.json` scenario files are full hand-copies of the base
models in `python/assets/aircraft_configs/`, so the two drift independently. That drift has
already caused defects — the `_ksba` configs silently lost their propulsion sections while the
base models retained engines (the go-around / FBW-command failure; see
[`landing_gear.md` OQ-LG-22](../design/landing_gear.md)). Scenarios as thin overlays over a
single base model eliminate this drift class entirely.

### Deliverables — Scenario Configuration Layering

- Design document defining: the reference/merge mechanism (a scenario config names a base
  model and supplies a deep-merge/override of added and changed keys), conflict and validation
  rules, how `schema_version` is reconciled across layers, and the resolved-config form passed
  to `Aircraft::initialize` and `make_propulsion`.
- A loader that resolves a scenario config against its referenced base model into a single
  validated config, used uniformly by the Python binding, `live_sim`, and `build_terrain`.
- Migration of the existing scenarios (`general_aviation_ksba`, `jet_trainer_ksba`,
  `small_uas_ksba`, `small_uas_ksba_flight`) to overlays carrying only scenario additions
  (`terrain`, `landing_gear`, `body_collider`, `visualization`, scenario `initial_state`) and
  overrides — with aerodynamic/inertia/lift-curve/propulsion coefficients sourced from the base
  models in `python/assets/aircraft_configs/`.
- Reconciliation of the base-model location (`python/assets/aircraft_configs/`) and the
  scenario location (`configs/`), with a short README in each documenting the base-vs-scenario
  convention.

### Tests — Scenario Configuration Layering

- Merge/resolution unit tests: a base + overlay resolves to the expected combined config;
  overrides replace base values; scenario-only sections are added; no base coefficients appear
  in the overlay source.
- Migration equivalence tests: each migrated scenario resolves to a config that constructs an
  `Aircraft` equivalent (within tolerance) to the pre-migration hand-copied config.
- Validation tests: a scenario referencing a missing base model, or resolving to an invalid
  config, fails with a clear error.
- Resolved-config JSON round-trip.

---

## Config-2. Complete `validate_aircraft_config.py` Field Coverage

**Blocking dependencies:** none.

**Design authority:** [`docs/schemas/aircraft_config_v1.md`](../schemas/aircraft_config_v1.md).

The Python config validator (`python/tools/validate_aircraft_config.py`) checks only a subset of
the `aircraft` section. The gear-model coupling parameters are now validated (added with the
non-dimensional parameterization), but the **command-response filter fields**
(`cmd_filter_substeps`, `nz_wn_rad_s`, `nz_zeta_nd`, `ny_wn_rad_s`, `ny_zeta_nd`,
`roll_rate_wn_rad_s`, `roll_rate_zeta_nd`) are required by `Aircraft::initialize()` but are **not**
checked by the validator. A config missing one passes validation yet throws at C++ initialization.
The validator should enforce every field the schema documents (and the C++ requires), with the
appropriate range/Nyquist checks, so validation is a faithful pre-flight of construction.

### Deliverables — validator field coverage

- Add the command-filter fields to the validator's required-field list with range checks
  (`cmd_filter_substeps` ≥ 1 int; frequencies and damping ratios > 0).
- Audit the validator against the full `aircraft_config_v1` schema for any other gaps.

### Tests — validator field coverage

- Per-field "missing raises" parametrized tests for each command-filter field.
- A test asserting every field required by the schema is enforced by the validator.

---

## PP-3. LiveTimeHistoryFigure

**Blocking dependencies:** SB-2 (ring buffer — live data source), Log-1 (logging
subsystem design — channel names must be finalized before panel definitions can reference
them).

**Design authority:** [`docs/design/post_processing.md`](../design/post_processing.md)
§`LiveTimeHistoryFigure` and §PP-F19–PP-F22.

A Panel + Plotly rolling time-history display that polls the ring buffer (SB-2) and
renders the same panel layout as `TimeHistoryFigure` with a rolling time window anchored
at the live edge (PP-F19–PP-F22). Panel definitions use the same arguments as
`TimeHistoryFigure.add_panel` so that a single specification works for both live and
post-processing views without modification (PP-F20). Includes user controls for time
axis scroll/zoom and per-panel y-axis reset (PP-F21–PP-F22).

### Deliverables — LiveTimeHistoryFigure

`python/tools/live_time_history.py` — `LiveTimeHistoryFigure` class with `add_panel()`,
`start()`, `stop()`, and Panel serving; ring buffer polling loop in a background thread.

### Tests — LiveTimeHistoryFigure

`python/test/test_live_time_history.py` — subscribe, buffer drain, rolling window update,
panel layout consistency with `TimeHistoryFigure`.

---

## PP-4. TrajectoryView Mode Segment Coloring

**Blocking dependencies:** Item 2 (Logged Channel Registry — concrete mode channel name
required); PP-2 (delivered).

**Design authority:** [`docs/design/post_processing.md`](../design/post_processing.md)
§Mode segment coloring.

Extend `RibbonTrail` and `TrajectoryView` to color each trajectory segment by the flight
mode active at that timestep, using colors from the `tab10` palette with a legend. The
mode channel name is taken from the Logged Channel Registry (item 2) to keep the channel
vocabulary consistent with the ring buffer (DR-5/DR-6).

### Deliverables — TrajectoryView Mode Segment Coloring

Updated `python/tools/trajectory_view.py` — `RibbonTrail` with per-mode color mapping;
`TrajectoryView` legend panel; mode channel name parameter from registry.

### Tests — TrajectoryView Mode Segment Coloring

New tests in `python/test/test_ribbon_trail.py` — per-mode color assignment, legend
entries, palette consistency with `tab10`.

---

## LG-2. Runway Geometry Extension (OQ-LG-3)

**Blocking dependencies:** LG-1 (delivered). Blocked on OQ-LG-3 — the open question
must be resolved and documented in `landing_gear.md` before implementation begins.

**Design authority:** [`docs/design/landing_gear.md`](../design/landing_gear.md)
§OQ-LG-3.

Resolve and implement the runway geometry extension deferred at OQ-LG-3: a planar runway
patch inset into `TerrainMesh`, or an analytical runway primitive added to `Terrain`, to
support accurate runway contact geometry for takeoff and landing scenario tests.

### Deliverables — Runway Geometry Extension

- Resolve OQ-LG-3 and document the decision in `landing_gear.md`.
- Implement the chosen approach in C++ (`TerrainMesh` or `Terrain` extension).
- `python/notebooks/takeoff_roll.ipynb` and `python/notebooks/crab_landing_dynamics.ipynb`
  — scenario notebooks specified in `landing_gear.md` requiring accurate runway surface
  geometry.

### Tests — Runway Geometry Extension

`test/LandingGear_Runway_test.cpp` — contact on analytical runway surface; `elevation_m`
returns runway elevation inside footprint; smooth height transition at runway boundary.

---

## Arch-1. `liteaero::` Namespace Migration

**Blocking dependencies:** None — all target namespaces are already final.

**Design authority:** [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
§Namespace adoption timing.

Single-step migration of all LiteAero Sim code to the `liteaero::simulation` namespace;
infrastructure types to `liteaero::control`, `liteaero::terrain`, and `liteaero::log` in
liteaero-flight. Per the architecture decision, this migration is executed as a single
coordinated event — not incrementally — to avoid a prolonged half-migrated codebase. All
public headers in `include/`, implementation files in `src/`, and test files in `test/`
are affected. Proto field names are unaffected (proto field names are snake_case strings,
not C++ identifiers). This is a cross-repository item (liteaero-flight + liteaero-sim).

### Deliverables — Namespace Migration

All `include/`, `src/`, and `test/` files updated to use `liteaero::simulation` namespace
wrapping. All documentation updated to reference the new namespace form. Coordinated with
the liteaero-flight repo split milestone.

### Tests — Namespace Migration

No new tests — all existing C++ tests must pass unchanged after the migration. The
migration is complete when `grep -r "namespace liteaerosim" include/ src/ test/` returns
zero results.

---

## Cfg-1. ParametricAircraftConfig — Proposed

**Blocking dependencies:** Item 4 (Aerodynamic Coefficient Design Study) must be complete
before this item can be scoped.

**Design authority:** [`docs/design/aero_coeff_estimator.md`](../design/aero_coeff_estimator.md)
§ParametricAircraftConfig.

Design and implement `ParametricAircraftConfig` (Python dataclass) and `VspGeometryBuilder`
for OpenVSP-driven geometry parameterization. Enables DOE coefficient sweeps (Latin
Hypercube, 500–2000 samples) using `AeroCoeffEstimator`. Not in scope for the current
design study — a dedicated design item precedes implementation.

### Deliverables — ParametricAircraftConfig

Design document extending `aero_coeff_estimator.md` §ParametricAircraftConfig.
`python/tools/parametric_config.py` — `ParametricAircraftConfig` dataclass and
`VspGeometryBuilder`; LHS sampling and batch `AeroCoeffEstimator` invocation.

### Tests — ParametricAircraftConfig

`python/test/test_parametric_config.py` — parameter sweep generates expected number of
configs; each config produces a valid `AeroCoeffEstimator` output.

---

## Path-1. PathSegmentTrochoid — Proposed

**Blocking dependencies:** Design document needed before this item can be scoped.

**Design authority:** No design document yet. `PathSegmentTrochoid` is proposed in
[`docs/architecture/overview.md`](../architecture/overview.md) as a planned path segment
type. A design document specifying the interface, parameterization, and integration with
the trajectory representation is required before implementation can begin.

### Deliverables — PathSegmentTrochoid

Design document; C++ header `include/path/PathSegmentTrochoid.hpp` and implementation;
JSON serialization.

### Tests — PathSegmentTrochoid

`test/PathSegmentTrochoid_test.cpp` — arc length, curvature, and point-on-path
queries against analytically known trochoid geometry; JSON round-trip.
