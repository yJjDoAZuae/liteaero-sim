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
| `AeroModel` | `include/aerodynamics/AeroModel.hpp` | 🔲 Planned — abstract aero model interface; defined by item 6; see [aero_coefficient_model.md](../architecture/aero_coefficient_model.md) |
| `BodyAxisCoeffModel` | `include/aerodynamics/BodyAxisCoeffModel.hpp` | 🔲 Planned — body-axis stability derivative model; implements `AeroModel`; defined by item 5 + item 6; see [aero_coefficient_model.md](../architecture/aero_coefficient_model.md) |
| `PropulsionCouplingCoefficients` | TBD | 🔲 Planned — propulsion-aero coupling coefficient struct; defined by item 5; see [propulsion_coeff_estimator.md](../architecture/propulsion_coeff_estimator.md) |
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
| `liteaero_sim_py` | `src/python/` | ✅ Implemented — pybind11 module; `AircraftCommand`, `ScriptedInput`, `JoystickInput.enumerate_devices()` |
| `manual_input_monitor.py` | `python/tools/manual_input_monitor.py` | ✅ Implemented — shared library (`InputMonitorConfig`, `enumerate_devices`, `build_args`, `stdout_reader`, `draw_gauge`, `InputMonitorFigure`) + standalone Qt app (`InputMonitorWindow`, `main()`) |
| `manual_input_demo.ipynb` | `python/manual_input_demo.ipynb` | ✅ Implemented — thin notebook importing library; ipympl inline display via `FuncAnimation` |
| `AnomalyDetector` | `python/tools/anomaly.py` | 🔲 Planned |
| `BehaviorVerifier` | `python/tools/behavior_verifier.py` | 🔲 Planned |
| `DataOverlay` | `python/tools/data_overlay.py` | 🔲 Planned |

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
| PP-1 | **Post-processing visualization tools — first-pass TDD implementation** — `FlightLogReader` (CSV loading via `pd.read_csv`, JSON-encoded MCAP via `mcap.reader.make_reader`, per-source DataFrames keyed by filename stem / `channel.topic`); `ModeEventSeries` (step-channel transition parser, `from_dataframe()` classmethod, `name_map` parameter, initial value skipped); `TimeHistoryFigure` (Plotly multi-panel, `shared_xaxes=True`, secondary y-axis via `specs`, `Scattergl` above 50 000 points, `add_vline()` mode overlays, `load(frames)` + `build()` + `export_html()`); `RibbonTrail` (ZYX Euler rotation matrices, wing body vector `[0, half_width_m, 0]`, `Poly3DCollection` quads, RdBu_r roll colormap via `TwoSlopeNorm(±π/3)`, midpoint-roll quad color); `HudOverlay` (9 fixed-position `text2D` artists, fading mode-change banner, 60-frame countdown); `TrajectoryView` (ghost ribbon α=0.15, live ribbon last 200 quads, dual-source overlay, `FuncAnimation(blit=False)`); `conftest.py` Agg backend; all design decisions subsequently resolved in dedicated design sessions; see DR-1 through DR-13 in [`post_processing.md`](../architecture/post_processing.md) | `test_log_reader.py` — 4 tests; `test_mode_events.py` — 5 tests; `test_time_history.py` — 6 tests; `test_ribbon_trail.py` — 5 tests; 20 tests total |
| PP-D | **Post-processing tools design — full architecture and decision records** — resolved all 28 design questions across both visualization and analysis layers; selections: Vispy (OpenGL, DR-8) for 3D rendering; PySide6 Qt window (DR-7) for playback and camera controls; Panel + Plotly (DR-10) for live time history; pre-generated glTF terrain via `pygltflib` (DR-11); offline terrain ingestion pipeline (DR-12); `terrain_paths.py` shared path module with `data/terrain/<dataset>/source/` + `derived/` repository structure (DR-13); per-field MCAP topic convention `"source/field_name"` (DR-9); `FlightLogReader` stateful API (DR-1); `ModeEventSeries` constructor name-map (DR-2); `TimeHistoryFigure.figure()` accessor (DR-3); ring buffer polling via pybind11 (DR-5, DR-6); camera modes FPV/Trailing/God's-eye/Local-top defined (PP-F28–PP-F32); terrain saturation runtime API (DR-13 area); document restructured from open-question tracking format to settled architecture with Decision Records appendix | [`docs/architecture/post_processing.md`](../architecture/post_processing.md) |
| PP-2 | **Post-processing visualization rework** (Tasks A–G, TDD) — `FlightLogReader`: per-field MCAP topic parsing (`"source/field_name"`), CSV `source_name` column, stateful `frames()` getter, `channel_names()` raises before load (DR-1, DR-9); `ModeEventSeries`: initial value emitted as first event, `name_map` moved to constructor (DR-2); `TimeHistoryFigure`: `figure()` public accessor with caching, `_build()` internal (DR-3); `RibbonTrail`: `wing_span_m` parameter, CCW quad winding, time-based α fade via `alpha_at()`, Vispy `MeshVisual` from `mesh()` (DR-8); `HudOverlay`: Vispy `Text` visuals in 2D overlay view, α-fading mode-change banner (DR-7, DR-8); `TrajectoryView`: `CameraMode` enum (FPV/TRAIL/GODS_EYE/LOCAL_TOP), Vispy `SceneCanvas` embedded in `QMainWindow`, `load_terrain()` via `pygltflib`, `set_terrain_saturation()`, headless `animate()` returns canvas (DR-7 through DR-13); `terrain_paths.py`: `get_terrain_data_root()`, `dataset_dir()`, `source_dir()`, `derived_dir()`, `las_terrain_dir()`, `gltf_path()`, `metadata_path()`; `vispy>=0.14`, `pyside6>=6.6`, `pygltflib`, `mcap-protobuf-support>=0.5` added to `pyproject.toml` | `test_log_reader.py` — 9 tests; `test_mode_events.py` — 6 tests; `test_time_history.py` — 6 tests; `test_ribbon_trail.py` — 9 tests; `test_terrain_paths.py` — 11 tests; `test_trajectory_view.py` — 14 tests; 55 tests total; full suite 150 pass |
| MI-1 | **Manual Input — Full Subsystem** — `ManualInput` abstract base; `KeyboardInput` (integrating keyboard adapter, configurable scancodes, action keys, injected key-state provider); `JoystickInput` (SDL2 adapter, axis pipeline: calibration/trim/normalization/dead-zone/scale, per-axis `raw_min`/`raw_max`/`raw_trim`, inversion, disconnect fallback, `captureTrim()`, `enumerateDevices()`); `ScriptedInput` (mutex-protected slot, `push(AircraftCommand)`); `SimRunner::setManualInput()` / `lastManualInputFrame()` with `SDL_WasInit` guard; `joystick_verify` tool (DEVICE/READY protocol, JSON lines + `--proto` output, `ManualInputFrameProto`/`AircraftCommandProto` added to `liteaerosim.proto`); MinGW runtime linked statically + SDL2.dll deployed alongside executable at build time; `manual_input_monitor.py` shared library + standalone Qt app (`InputMonitorConfig`, `InputMonitorFigure`, `InputMonitorWindow`, `_subprocess_env()`; persistent gauge artists — no `ax.clear()` on update); `manual_input_demo.ipynb` thin notebook using ipympl + `FuncAnimation`; `gx12_config.json` Radiomaster GX12 axis mapping; `liteaero_sim_py` pybind11 module (`AircraftCommand`, `ScriptedInput`, `JoystickInput.enumerate_devices()`); SDL2 and pybind11 added to `conanfile.txt` and dependency registry; `ipympl>=0.9` added to `pyproject.toml`; design authority: [`docs/architecture/manual_input.md`](../architecture/manual_input.md), [`docs/architecture/sim_runner.md`](../architecture/sim_runner.md), [`docs/architecture/python_bindings.md`](../architecture/python_bindings.md) | C++: `KeyboardInput_test.cpp` — 10 tests; `JoystickInput_test.cpp` — 15 tests; `ScriptedInput_test.cpp` — 4 tests; `SimRunner_test.cpp` — 4 new manual input tests; Python: `test_manual_input_bindings.py` — 7 tests; `test_manual_input_monitor.py` — 29 tests |

---

## 1. Sensor Models — Implementable Subset

**Blocking dependencies:** None. `KinematicState` and `V_Terrain` are implemented.

Implement the four sensor models whose only dependencies are already available. The
remaining sensors (`SensorINS`, `SensorAA`, `SensorAAR`, `SensorForwardTerrainProfile`,
`SensorTrackEstimator`) are deferred to item 11; they depend on LiteAero Flight
components that are not yet designed.

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | `V_Terrain` (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | `V_Terrain` (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |

Each sensor requires a design document before implementation. Implement in the order
listed; `SensorLaserAlt` and `SensorRadAlt` outputs are required by the `AnomalyDetector`
`AltitudeBelowTerrain` rule (item 7).

---

## 2. Logged Channel Registry — Design

**Blocking dependencies:** SimRunner (delivered), LandingGear C++ (delivered), item 2
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

## 3. Real Flight Log Format — Design

**Blocking dependencies:** None. This is a standalone design decision.

Produces a design decision document defining how real aircraft flight logs are loaded by
`DataOverlay`. Must be resolved before `DataOverlay` can be implemented.

### Deliverables — Real Flight Log Format

Design document (`docs/architecture/flight_log_format.md`) covering:

- What log format(s) real aircraft produce (e.g., ArduPilot DataFlash, MAVLink ULOG,
  custom CSV, or a configurable adapter).
- Channel name mapping from the real-log format to the simulation channel naming
  convention defined in item 3.
- Policy for handling channels present in the real log but absent from the sim schema,
  and vice versa.
- Whether a translation/adapter layer is implemented in `FlightLogReader` or as a
  separate preprocessing step.

---

## 4. Aerodynamic Coefficient Design Study

**Blocking dependencies:** None. `AeroCoeffEstimator` is implemented.

Design authority documents:

- Coefficient model format, sign conventions, and propulsion integration:
  [`docs/architecture/aero_coefficient_model.md`](../architecture/aero_coefficient_model.md)
- Aerodynamic coefficient estimation methods and `AeroCoeffEstimator` extension:
  [`docs/architecture/aero_coeff_estimator.md`](../architecture/aero_coeff_estimator.md)
- Propulsion parameter estimation and propulsion-aero coupling:
  [`docs/architecture/propulsion_coeff_estimator.md`](../architecture/propulsion_coeff_estimator.md)

Prerequisite for item 5 (`Aircraft6DOF` and `BodyAxisCoeffModel`). Resolves OQ-16(c).

### Deliverables — Aerodynamic and Propulsion Coefficient Design Study

- `aero_coefficient_model.md` completed and all open questions resolved.
- `aero_coeff_estimator.md` completed and all open questions resolved.
- `propulsion_coeff_estimator.md` completed and all open questions resolved.
- `AircraftGeometry` + `PropulsionGeometry` JSON files and coefficient tables for Cases A, B, and C.
- `BodyAxisCoeffModel` and `PropulsionCouplingCoefficients` formats decided and documented.
- Element registry updated (`AeroModel` named, `BodyAxisCoeffModel` and `PropulsionCoeffEstimator` formats defined).

Must be complete before item 5 begins.

---

## 5. Aircraft6DOF — Design and Implementation

**Blocking dependencies:** Item 4 (aerodynamic coefficient design study).

Full 6DOF aircraft dynamics model. Architecture placeholders are defined in
[`docs/architecture/system/future/element_registry.md`](../architecture/system/future/element_registry.md).

### Scope — Aircraft6DOF

- **`AeroModel`** — abstract aerodynamic model interface; produces forces and moments in
  body frame; decouples the 6DOF integrator from the coefficient axis convention.
- **`BodyAxisCoeffModel`** — implements `AeroModel` using body-axis stability derivatives
  (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular
  rates. Coefficient model format defined by item 5.
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

Design authority document. Implementation follows TDD. All new elements include JSON +
proto serialization and round-trip tests. `Aircraft6DOF` and `Aircraft` must both produce
`KinematicStateSnapshot` to enable transparent substitution.

---

## 6. Post-Processing — Analysis Tools Design Harmonization and Implementation

**Blocking dependencies:** Item 3 (Logged Channel Registry), item 4 (Real Flight Log
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
  Logged Channel Registry (item 3).
- Specify the `DataOverlay` format adapter for the real flight log format (item 4).
- Define `BehaviorVerifier` command channel names from the LiteAero Flight command schema.
- Define the Scenario Reference Data Format for `WaypointReached` and similar criteria.

### Analysis Module Implementation

Implement in dependency order:

| Module | File | Blocked by |
| --- | --- | --- |
| `AnomalyDetector` + rule library | `python/tools/anomaly.py` | Item 3, sensor item 2 |
| `DataOverlay` | `python/tools/data_overlay.py` | Item 4 |
| `BehaviorVerifier` + criterion library | `python/tools/behavior_verifier.py` | Item 3, LiteAero Flight schema |

### Tests — Analysis Tools

`python/test/test_anomaly.py`, `test_data_overlay.py`, `test_behavior_verifier.py`.

---

## 7. LandingGear — Python Bindings and Scenario Tests

**Blocking dependencies:** LandingGear C++ (delivered, LG-1), PP-1 (delivered —
visualization tools exist for animation). PP-2 (rework) is delivered.

Implement Steps G–H from the design authority document
([`docs/architecture/landing_gear.md`](../architecture/landing_gear.md)).

### Steps G–H

#### Step G — Python Bindings (pybind11)

Expose `LandingGear`, `WheelUnit`, `StrutState`, and `ContactForces` to Python via
pybind11. Design authority: [`docs/architecture/python_bindings.md`](../architecture/python_bindings.md) — Landing Gear section.

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

See [`docs/architecture/python_bindings.md`](../architecture/python_bindings.md) for the CMake target and conan dependency.

---

## 8. External Interface Elements

**Blocking dependencies:** SimRunner (delivered) for all elements. Item 5 (Aircraft6DOF)
for `PX4Interface`. `NavigationState` from LiteAero Flight for `QGroundControlLink`.

Adapters that connect LiteAero Sim to external systems. All live in the Interface Layer.

| # | Element | Protocol | Depends on |
| --- | --- | --- | --- |
| LAS-ext-1 | `ArduPilotInterface` | ArduPilot SITL protocol | SimRunner (delivered); `Aircraft` or `Aircraft6DOF` |
| LAS-ext-2 | `PX4Interface` | PX4 SITL bridge | SimRunner (delivered); `Aircraft6DOF` (item 5) |
| LAS-ext-3 | `QGroundControlLink` | MAVLink over UDP | SimRunner (delivered); `NavigationState` (LiteAero Flight) |
| LAS-ext-4 | `VisualizationLink` | UDP to Godot 4 GDExtension plugin at simulation rate | SimRunner (delivered); `SimulationFrame` (done) |

Each element requires a design document before implementation. `VisualizationLink`
transport and axis convention are decided (see
[`docs/architecture/terrain.md`](../architecture/terrain.md) §Game Engine Integration and
[`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md)
§Game engine for real-time visualization).

---

## 9. Sensor Models — Deferred Subset

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

## 10. Synthetic Perception Sensors — Proposed

**Blocking dependencies:** Design items needed for each sensor before implementation
can begin. Schedule when prerequisite sensor and terrain models are stable.

| Element | Responsibility |
| --- | --- |
| `SensorCamera` | Synthetic image sensor; generates imagery from terrain and scene model against `V_Terrain` |
| `SensorLidar` | Synthetic lidar; generates 3D point cloud by ray-casting against `V_Terrain` |
| `SensorLaserAGL` | Synthetic laser altimeter; computes AGL range by ray-casting against `V_Terrain` |
| `SensorLineOfSight` | Computes RF link quality and terrain occlusion by ray-casting against `V_Terrain` |

---

## Terrain-1. Rename `V_Terrain` to `Terrain` (liteaero-flight)

**Blocking dependencies:** None.

`liteaero::terrain::V_Terrain` uses the forbidden `V_` Hungarian-notation prefix.
The correct name is `Terrain`. This rename touches liteaero-flight (the abstract base
`V_Terrain.hpp`) and all liteaero-sim call sites that include
`<liteaero/terrain/V_Terrain.hpp>` or reference `liteaero::terrain::V_Terrain`:

- `include/Aircraft.hpp`
- `include/landing_gear/LandingGear.hpp`
- `include/environment/TerrainMesh.hpp` (inherits from `V_Terrain`)
- All `.cpp` files that use the pointer type directly

**Scope:** Cross-repository rename (liteaero-flight + liteaero-sim). Update the Current
State table, delivered item 14, item LG-1 description, items 2 and 11 of this roadmap,
and all sensor design documents that reference the interface by name once the rename is
complete.
