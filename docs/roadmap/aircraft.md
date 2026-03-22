# Aircraft Class — Roadmap

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

**Note on system scope.** Item 1 defines the system architecture, which establishes that
LiteAeroSim is the simulation component only. Autopilot, guidance, path representation,
and navigation are flight code components that are architecturally separate from the
simulation. Their design and implementation (items 2 onward that address flight code) will
follow the component boundaries established in item 1. Until the architecture is defined,
items addressing flight code components are placeholders; their repository location, build
system integration, and interface details are to be determined by item 1.

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
| `DynamicElement` | `include/DynamicElement.hpp` | ✅ Implemented — see [dynamic_element.md](../architecture/dynamic_element.md) |
| `SisoElement` | `include/SisoElement.hpp` | ✅ Implemented — NVI SISO wrapper over `DynamicElement` |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | ✅ Implemented + serialization (JSON + proto) |
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
| 18 | Control subsystem refactoring (Steps A–J, all nine issues in [`control_interface_review.md`](../architecture/control_interface_review.md)) — `FilterError`/`DiscretizationMethod` promoted to `enum class`; `LimitBase` deleted, `Limit`/`RateLimit` rebased to `SisoElement`; `Gain` API cleaned up (`set()`, `value()`, stubs removed); `FilterSS2Clip`, `FilterTF2`, `FilterTF`, `FilterFIR`, `FilterSS` migrated to NVI (`onStep()`/`onSerializeJson()`/`onDeserializeJson()`); shadow `_in`/`_out` members and no-op `Filter` defaults removed; `Unwrap` `ref_` field added (`setReference()`, NVI routing, serialization); `SISOPIDFF` derives from `DynamicElement` with full lifecycle, `snake_case_` member renames (`Kp`→`proportional_gain_`, `I`→`integrator_`, etc.), private limits, `ControlLoop` accessor renames (`out()`→`output()`, `pid`→`controller_`); `Integrator`/`Derivative` private member renames (`_dt`→`dt_s_`, `_Tau`→`tau_s_`, `limit`→`limit_`) | `FilterSS2Clip_test.cpp`, `FilterTF2_test.cpp`, `FilterFIR_test.cpp` (new), `FilterSS_test.cpp`, `Unwrap_test.cpp`, `SISOPIDFF_test.cpp` (new) — 29 new tests; 435 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 19 | `SensorAirData` — pitot-static air data computer; differential pressure ($q_c$) and static pressure ($P_s$) transducers with Gaussian noise, first-order Tustin lag, and fuselage crossflow pressure error (two-port symmetric crosslinked model); derives IAS, CAS, EAS, TAS, Mach, barometric altitude (Kollsman-referenced, troposphere + tropopause), OAT; RNG pimpl with seed + advance serialization; JSON + proto round-trips | `SensorAirData_test.cpp` — 19 tests; 454 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 20 | `LoadFactorAllocator` alpha-ceiling fix — Newton overshoot and fold guards corrected for positive-thrust case; the achievable-Nz ceiling is at $\alpha^*$ (where $f'(\alpha) = qSC_L'(\alpha) + T\cos\alpha = 0$), not at `alphaPeak()`, when $T > 0$; overshoot guard now clamps the proposed Newton step to the CL parabolic domain using `LiftCurveModel::alphaSep()` / `alphaSepNeg()` before checking $f'$, preventing escape into the flat separated plateau where $f' = T\cos\alpha$ stays positive until $\alpha > \pi/2$; bisects to locate $\alpha^*$ when the guard fires; fold guard stays at current iterate rather than snapping to `alphaPeak()`; `LiftCurveModel::alphaSep()` and `alphaSepNeg()` added to public interface; design documentation updated in `docs/implementation/equations_of_motion.md` and `docs/algorithms/equations_of_motion.md` | 4 new tests in `LoadFactorAllocator_test.cpp`; 19 tests total; 458 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 0a | `LoadFactorAllocator` branch-continuation predictor — first-order warm-start $\alpha_0 = \alpha_\text{prev} + \delta n_z \cdot mg / f'(\alpha_\text{prev})$ and symmetric $\beta_0$ formula added to `solve()`; predictor is skipped at the stall ceiling ($f' \approx 0$) or when the raw prediction would fall outside $[\alpha_\text{sep\_neg}, \alpha_\text{sep}]$ (domain guard prevents cross-branch jumps on cold-start excess-demand calls); `_n_z_prev` and `_n_y_prev` added as serialized state fields in both JSON (`n_z_prev_nd`, `n_y_prev_nd`) and proto (`LoadFactorAllocatorState` fields 6–7); `iterations` (alpha solver iteration count) added to `LoadFactorOutputs`; `reset()` clears all four warm-start fields; `docs/implementation/equations_of_motion.md` §Warm-Starting updated | 3 new tests in `LoadFactorAllocator_test.cpp`: `PredictorReducesIterationsOnLinearStep` (iterations == 1 for an exact linear-region prediction), `PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev`, `PredictorProtoRoundTrip_IncludesNzPrev`; 22 tests total |
| 0b | `LoadFactorAllocator` test coverage extension — 8 new tests close continuity and domain-coverage gaps identified by code review. **White-box tests** (4): full positive and negative Nz sweeps through stall verifying the clamp value against `alphaPeak()`/`alphaTrough()`; fine-step sweep across the C¹ Linear→IncipientStall boundary confirming both segments are traversed; stall warm-start limitation test documenting that `reset()` is required after a discontinuous Nz jump. **Black-box tests** (4): uniform 500-step monotonicity sweeps from 0 to ±10 g for T = 0 (positive and negative) and T = `kLargeThrust` (positive); point-wise perturbation test at 37 grid points (T = 0, −9 g to +9 g) and 19 grid points (T > 0, 0 to +9 g) using fresh allocators. Stall warm-start limitation documented in `docs/implementation/equations_of_motion.md` §Stall Warm-Start Limitation | 30 tests total in `LoadFactorAllocator_test.cpp` |

---

## 0. Aircraft Command Processing Redesign

The current `Aircraft` command processing is structurally wrong. The Nz, Ny, and roll rate
commands enter the physics loop through bare derivative filters (`FilterSS2Clip::setDerivIIR`)
and a 1st-order roll rate low-pass respectively. This does not model how a real FBW aircraft
responds to commanded load factors and roll rate. A real FBW inner loop has a finite closed-loop
bandwidth that shapes how the aircraft responds to pilot commands — the response to a step command
is not instantaneous but follows a characteristic 2nd-order (or higher) transient. The plant model
must include this behavior so that guidance and autopilot algorithms developed against it see a
physically realistic command response behavior.

### What Needs to Be Designed

**Axis command response filters.** Each of the three commanded axes needs a command shaping filter
that maps the raw pilot/autopilot command to the shaped command that drives the physics:

- **Nz axis** — a 2nd-order low-pass filter on the commanded Nz, parameterized by a natural
  frequency and damping ratio that represents the aircraft's closed-loop short-period/FBW bandwidth.
  The shaped Nz output drives the load factor allocator.
- **Ny axis** — same architecture as Nz, with its own bandwidth parameters.
- **Roll rate axis** — a 2nd-order or 1st-order command response filter on the commanded roll rate,
  parameterized by a bandwidth that represents the roll axis FBW closed-loop response.

**Derivative terms for feed-forward.** `LoadFactorAllocator::solve()` accepts `n_z_dot` and
`n_y_dot` as feed-forward terms for α-dot and β-dot estimation. These must be the time derivatives
of the *shaped* Nz and Ny commands (not the raw commands). The design must define how these
derivatives are produced — options include:

- Differentiating the 2nd-order filter output with a separate derivative filter.
- Computing the derivative analytically from the 2nd-order filter's state vector (exact, no
  additional lag introduced).

**Inner substep loop.** The command response filters run at a higher rate than the outer
rigid-body integrator step. The design must specify:

- The substep parameter (integer n: filter runs at `outer_dt / n`).
- Which quantities are substepped (command response filters only; propulsion and kinematic
  integration remain at the outer rate).
- How the substepped filter output is handed off to the outer-rate allocator and integrator.

**Nyquist constraints.** All filter bandwidths must be validated at initialization against the
Nyquist frequency of their respective update rates. The design must specify which update rate
(inner or outer) governs each parameter.

**Serialization.** Full JSON and proto round-trips for all new filter states. `AircraftState`
proto message updated accordingly. `aircraft_config_v1` schema document updated.

### Scope of Current Implementation to Discard or Rework

The following elements of the current `Aircraft` implementation were added without a design
authority document and must be reviewed against the design produced by this item before any
are retained:

- `_n_z_deriv`, `_n_y_deriv` (`setDerivIIR`) — derivative filters on the *raw* commands.
  Retain only if the design confirms this is the correct source for `n_z_dot`/`n_y_dot`.
- `_roll_rate_filter` (`setLowPassFirstIIR`) — 1st-order roll rate smoothing. To be replaced
  by the 2nd-order command response filter specified by the design.
- `_cmd_filter_substeps`, `_cmd_filter_dt_s`, `_cmd_deriv_tau_s`, `_cmd_roll_rate_tau_s` —
  config parameters added without a design; must be reconciled with or replaced by the
  parameters defined in the design document.

### Deliverables — Command Processing

1. ✅ Design authority document at `docs/architecture/aircraft.md` updated to cover the
   command processing architecture: filter topology (all three axes `setLowPassSecondIIR`),
   parameter definitions, Nyquist constraints, inner substep mechanics, derivative sourcing
   from filter state vector.
2. `aircraft_config_v1` schema document (`docs/schemas/aircraft_config_v1.md`) updated —
   the `aircraft` section currently documents only the five aero-geometry fields; the six
   command-filter parameters (`cmd_filter_substeps`, `nz_wn_rad_s`, `nz_zeta_nd`,
   `ny_wn_rad_s`, `ny_zeta_nd`, `roll_rate_wn_rad_s`, `roll_rate_zeta_nd`) are absent.
3. Implementation following TDD: failing tests before production code. The implementation
   still uses `setDerivIIR` for the Nz and Ny filters and `setLowPassFirstIIR` for the
   roll-rate filter, and reads `cmd_deriv_tau_s` / `cmd_roll_rate_tau_s` from config
   rather than the natural-frequency and damping-ratio parameters the design prescribes.
4. Fixture JSON files (`test/data/aircraft/`) updated to match new config schema.

---

## 1. Flight Code and Simulation Architecture Definition

Define a system architecture model that will inform all subsequent software development. The model will be encoded in Markdown documentation with Mermaid diagrams and tables, organized under `docs/architecture/`, and treated as the primary source of truth for system design. The model will include:

- System originating requirements
- System use cases
- System element registry — each element defines its ports and the data flows it produces and consumes
- Data flow type registry — each type defines the contents and character of a data flow
- Data flow instance registry — maps data flow instances to their producers, consumers, and routing
- Data flow diagrams
- Interface Control Documents for each data flow type — sufficient to identify the interface and confirm the architecture accommodates it; detailed field-level schema definitions are deferred to the software design phase

### Simulation Architecture

The simulation architecture has not been formally defined. The design must establish a flexible architecture that supports integration with external components, optional in-process or out-of-process autopilot and navigation functions, and the computational efficiency requirements of real-time and batch operation.

### Autopilot Architecture

The autopilot will be a separable software component designed to flight-code standards, usable both as embedded flight software and as a component within the simulation. It must support simulation use cases such as reset and initialization to arbitrary conditions for batch testing. Estimation functions are outside the autopilot boundary — they belong to the Navigation system. The architecture must accommodate integration with ArduPilot and PX4, using MAVLink where sufficient and defining custom interfaces where MAVLink does not meet requirements.

### Navigation and Perception Architecture

The navigation system is a separable flight code component that derives kinematic state estimates from sensor measurements. It must be usable both within the simulation and as flight software on a real aircraft. The architecture must also identify the kinds of perception functions anticipated (e.g., image-based navigation, inference-based state estimation) and set requirements that allow those capabilities to be added in the future without requiring a new architecture.

### External Components

- Game engine connection for real-time and scaled-real-time 3D visualization
- Pilot-in-the-loop input via joystick and RC transmitter (USB)
- QGroundControl connection
- PX4 hardware-in-the-loop and software-in-the-loop simulation interfaces
- ArduPilot hardware-in-the-loop and software-in-the-loop simulation interfaces
- Autopilot and navigation interfacing to ArduPilot and PX4

---

## 2. Gain Scheduling — Design

`Gain<T, NumAxes>` currently holds template parameters for value type and scheduling
axis count, but the scheduling logic is unimplemented (stubs removed in delivered item 18,
Step C). This item defines the full gain scheduling architecture. Gain scheduling is a
general-purpose library capability used beyond PID loops — it may parameterize model
coefficients, limits, or other algorithm inputs in both simulation and flight code.
Fundamentally, a gain is an object that produces a value parameterized by the current
aircraft state (a combination of flight condition and configuration). Within a PID it is a
multiplicative coefficient, but it may also enter an algorithm as an additive term or in
other ways. The `Gain` class does not need convenience methods for every mathematical
pattern of use; it only needs to expose its current value to the calling function.

### Scope to Define

The design must address at minimum:

- **Lookup methods** — what interpolation strategies are supported (e.g. rectilinear
  table bilinear, nearest-neighbor, polynomial fit) and how they are selected.
- **Axis dimensions** — how `NumAxes` maps to physical scheduling variables
  (e.g. airspeed, altitude, angle of attack); how axes are labeled and units enforced.
- **Lookup domain constraint functions** — how constraint functions are applied to ensure that lookup occurs only within the valid domain of the scheduling inputs.
- **Lookup parameterization** — which flight condition and aircraft configuration values are available as scheduling axes; use cases that drive axis requirements.
- **Runtime update** — how a scheduled gain is evaluated at each step given the current
  scheduling variable values; whether evaluation is synchronous with `SISOPIDFF::step()`
  or driven externally.
- **Serialization** — how the gain table is stored and restored (JSON + proto).
- **Initialization** — whether the gain table is loaded from a file, embedded in config
  JSON, or populated programmatically.

### Deliverables — Gain Scheduling

Design authority document at `docs/architecture/gain_scheduling.md`.
Plan implementation to follow TDD: failing tests before production code.
Do not implement in this task.

---

## 3. Gain Scheduling — Implementation

Implement the design produced in item 2. Follow TDD: write failing tests before production code.

---

## 4. Landing Gear — Ground Contact Model Design

`LandingGear` is a Domain Layer physics component that models the contact forces and
moments exerted on the airframe by the landing gear during ground operations (taxi,
takeoff roll, and landing roll). It produces forces and moments in the body frame that
are added to the aerodynamic and propulsion contributions in `Aircraft::step()`.

Produce documentation of requirements, architecture, algorithm design, verification, and implementation plan.

### Scope — Gain Scheduling

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

Design authority document at `docs/architecture/landing_gear.md` to be written
before implementation begins.

Implementation follows TDD: failing tests before production code.

---

## 5. Autopilot Gain Design — Python Tooling

Python workflow that derives autopilot control gains from the aircraft model. This is a
prerequisite for item 6 (`Autopilot`) — the C++ implementation is parameterized by gains
computed here.

Scope to be defined when this item is scheduled. Expected to use Python control-system
libraries (e.g. `python-control`, `scipy.signal`) applied to linearized models extracted
from `Aircraft` trim and `AeroCoeffEstimator` outputs.

---

## 6. Autopilot — Inner Loop Knobs-Mode Tracking

*Flight code component — not part of LiteAeroSim. Repository placement and build system
integration are to be determined by item 1 (Architecture Definition). A stub header exists
at `include/control/Autopilot.hpp` as a temporary placeholder and will be relocated once
the architecture is established.*

`Autopilot` implements the inner closed-loop layer. It tracks pilot-style set point commands
corresponding to "knobs" modes — altitude hold, vertical speed hold, heading hold, and roll
attitude hold. Guidance (item 8) is the outer loop that wraps around it and supplies the set
point commands. Control gains are derived by the Python gain design workflow (item 5). The
autopilot must support simulation use cases (reset and initialization to arbitrary conditions
for batch testing) as well as deployment as flight software.

### Interface Sketch — Autopilot

*To be defined during design. Inputs will include the kinematic state, atmospheric state,
and a set point struct (target altitude, target vertical speed, target heading, target roll
attitude). The output command type and its interface to the simulation plant model will be
defined by the architecture.*

### Tests — Autopilot

- Altitude hold: starting from a displaced altitude, output drives altitude error to zero
  within the expected settling time.
- Heading hold: starting from a heading offset, output drives heading error to zero without
  overshoot beyond a specified bound.
- Roll attitude hold: commanded roll angle is tracked with correct steady-state and transient.
- Vertical speed hold: commanded climb rate is tracked correctly.
- JSON and proto round-trips preserve filter states.
- Reset and re-initialization to arbitrary conditions produces correct initial output.

### Build Integration — Autopilot

To be determined by item 1. The component must be buildable as standalone flight code and
as a component co-resident with the LiteAeroSim simulation for integration testing.

---

## 7. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

*Flight code component — not part of LiteAeroSim. Repository placement and build system
integration are to be determined by item 1. Stub headers exist at `include/path/` as
temporary placeholders. The namespace shown below is provisional.*

A `Path` is an ordered sequence of `V_PathSegment` objects. Each segment exposes a
cross-track error, along-track distance, and desired heading at a query position. The
initial concrete segment type is `PathSegmentHelix` (straight line is a degenerate helix
with infinite radius).

### Interface Sketch — Path

```cpp
// Provisional — namespace and location subject to architecture definition (item 1)
namespace path {

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

} // namespace path
```

### Tests — `PathSegmentHelix`

- On a straight segment (infinite radius), cross-track error equals the perpendicular
  distance from the line.
- At the midpoint of a circular arc, `desired_heading_rad` is tangent to the arc.
- `segment_complete` is false before the end and true after `along_track_m >= length_m`.

### Tests — `Path`

- A path with two segments advances to the second segment when the first is complete.
- `Path::query()` delegates to the active segment.

### Build Integration — Path

To be determined by item 1.

---

## 8. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

*Flight code component — not part of LiteAeroSim. Repository placement and build system
integration are to be determined by item 1. Stub headers exist at `include/guidance/` as
temporary placeholders.*

Guidance is the outer loop that wraps around the `Autopilot`. It converts path and altitude
errors into set point commands (target altitude, vertical speed, heading, roll attitude) that
are fed to `Autopilot::step()`. Each guidance class is stateful and implements reset,
step, and serialization in accordance with the component lifecycle defined in item 1.

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

### Build Integration — Guidance

To be determined by item 1.

---

## 9. Airfield Traffic Pattern Operations

*This item is a flight code and integration item, not a LiteAeroSim item. LiteAeroSim
provides the simulation plant. The autopilot and guidance (items 6–8) are the flight code
components that execute the traffic pattern behavior. This item defines the operational
requirements, mode sequencing, and integration architecture for traffic pattern operations.*

Define autotakeoff and autolanding behavior for fixed-wing aircraft operating in a standard
traffic pattern. Behavior definitions are drawn from FAA non-towered airfield VFR operations
and AMA club field conventions.

### Scope — Traffic Pattern

- **Normal pattern operations** — pattern entry, upwind, crosswind, downwind, base, and
  final legs; pattern exit; go-around.
- **Off-nominal handling** — aborted takeoff, aborted landing, expedited approach,
  short-field and high-performance variants.
- **Pilot operator interface** — switch inputs mappable to RC transmitter channels for
  initiating aborts, go-arounds, and traffic sequencing; behavior must be visually
  interpretable and rule-compliant.
- **Traffic pattern definition** — wind-referenced flow direction; operator-side handedness
  (aircraft remain on the far side of the field from operators); geofence envelope; safe
  loiter point (altitude- and laterally-separated from the active pattern); RC link range
  margin; nominal flight path angles for climb, approach, and touchdown; touchdown zone and
  rotation initiation region geometry.
- **ArduPilot / PX4 integration architecture** — define the integration model: options
  include full replacement of the autopilot's internal autotakeoff/autolanding modes,
  partial override via a companion computer or offboard computation node, or mode
  sequencing via Lua scripting. The autopilot and guidance components may be collocated
  with the simulation or hosted on separate compute resources. The architecture must
  accommodate all of these configurations.

### Example behavior

A go-around from final approach initiates a full-power climb at safe airspeed, reconfigures
the aircraft for climb, and holds the extended runway centerline until terrain and obstacle
separation is confirmed. The aircraft then transitions to the climbout phase of the traffic
pattern as if departing from a normal takeoff.

---

## 10. Airfield Ground Operations

Define the operational sequence for ground operations at an RC airfield, from power-up through parking. The design document will cover (non-exhaustively):

- **Pre-flight** — hardware-in-the-loop simulation; system parameterization; power-up; GPS acquisition and EKF alignment; bench systems test; pre-takeoff systems test
- **Runway operations** — taxi to runway; runway survey; taxi to takeoff position; low-speed takeoff roll test and abort; high-speed takeoff roll test and abort; takeoff roll with transition to autotakeoff mode
- **Post-flight** — landing rollout; back-taxi; taxi pausing for traffic; taxi to parking
- **Sequencing and safety** — define go/no-go criteria at each phase transition; define abort procedures and recovery actions

---

## 11. Plot Visualization — Python Post-Processing Tools

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

## 12. Manual Input — Joystick and Keyboard

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
namespace liteaerosim::input {

class V_ManualInput {
public:
    virtual AircraftCommand read() = 0;   // non-blocking; returns latest command
    virtual ~V_ManualInput() = default;
};

} // namespace liteaerosim::input
```

### Tests — Manual Input

- `KeyboardInput` with no keys pressed returns zero lateral/directional commands and
  configured idle throttle.
- `JoystickInput` axis value at dead-zone boundary maps to zero output.
- Axis scaling: full-deflection axis value maps to the configured maximum command.

### CMake — Manual Input

Add `src/input/KeyboardInput.cpp` and `src/input/JoystickInput.cpp` to `liteaerosim`.
Add a platform-conditional dependency on SDL2 for `JoystickInput`.

---

## 13. Execution Modes — Real-Time, Scaled, and Batch Runners

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

Add `src/runner/SimRunner.cpp` to `liteaerosim`.
Add `test/SimRunner_test.cpp` to the test executable.

---

## 14. Remaining Sensor Models

Not blocking any higher-priority item. Stub headers exist in `include/sensor/`.
Implement when needed; order within this group follows dependency.

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
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

## 15. Estimation Subsystem

Flight code estimation algorithms. Stub headers exist in `include/estimation/` (to be
created). Each derives from `DynamicElement` directly. Design authorities listed below.

| Class | Depends on | Design authority |
| --- | --- | --- |
| `NavigationFilter` | `SensorGnss`, `SensorAirData`, `SensorMag` | [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `NavigationFilter` or `SensorInsSimulation`, `SensorAirData` | [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `WindEstimator`, `SensorAirData` | [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |
