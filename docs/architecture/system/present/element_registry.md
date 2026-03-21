# System Element Registry — Present State

Elements implemented in the current codebase. Elements marked **stub** are defined
architecturally and have header files but no implementation.

All elements reside within the LiteAeroSim C++ library unless noted as Python tooling.

---

## Root Abstractions

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `DynamicElement` | Infrastructure | NVI lifecycle base for all stateful components; enforces `initialize` / `reset` / `serialize` contract | JSON config | — |
| `SisoElement` | Infrastructure | SISO wrapper over `DynamicElement`; provides `step(float) → float` NVI for scalar signal-chain elements | `float u` | `float y` |

---

## Physics

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `Aircraft` | Domain | 6-DOF rigid body; integrates aerodynamic and propulsion forces; owns kinematic state | `AircraftCommand`, `EnvironmentState` | `KinematicState` |
| `KinematicState` | Domain | Value object: position (WGS84), NED velocity, attitude (quaternion + Euler), body angular rates, body acceleration | — | consumed by sensors, guidance |
| `LiftCurveModel` | Domain | Lift curve parameterization; config-only (no step) | aero coefficients config | — |
| `LoadFactorAllocator` | Domain | Distributes commanded load factor to control surface deflections | `n`, `n_y`, `alpha`, `beta` | surface deflections |
| `AeroCoeffEstimator` | Domain | Derives trim aero coefficients from airframe geometry (DATCOM/Hoerner methods) | `AircraftGeometry` | `AeroPerformance` |
| `AeroPerformance` | Domain | Value object: trimmed aerodynamic coefficient set | — | consumed by `Aircraft` |
| `AirframePerformance` | Domain | Value object: combined aero + propulsion performance | — | consumed by `Aircraft` |
| `Inertia` | Domain | Mass and moment-of-inertia tensor | — | consumed by `Aircraft` |

---

## Propulsion

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `Propulsion` (abstract) | Domain | Base interface for all propulsion models | throttle, `AtmosphericState`, airspeed | thrust_n |
| `PropulsionJet` | Domain | Turbojet thrust model (Mach + altitude lapse) | throttle, Mach, altitude | thrust_n |
| `PropulsionEDF` | Domain | Electric ducted fan thrust model | throttle, airspeed | thrust_n |
| `PropulsionProp` | Domain | Propeller + motor thrust model | throttle, airspeed, RPM | thrust_n, torque_nm |
| `Motor` (abstract) | Domain | Stateless motor torque/RPM interface | throttle | torque_nm, RPM |
| `MotorElectric` | Domain | Electric motor: torque–RPM–throttle relationship | throttle, RPM | torque_nm |
| `MotorPiston` | Domain | Piston engine: power curve model | throttle, RPM, altitude | torque_nm |

---

## Environment

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `Atmosphere` | Domain | ISA 3-layer model + temperature deviation + humidity; density altitude | geometric altitude (m) | `AtmosphericState` |
| `Wind` | Domain | Steady wind: Constant / PowerLaw / Log profile variants | altitude (m) | wind velocity NED (m/s) |
| `Turbulence` | Domain | Dryden continuous turbulence; 6-filter Tustin-discretized | `V_a` (m/s), altitude (m) | `TurbulenceVelocity` (body frame, m/s + rad/s) |
| `Gust` | Domain | 1-cosine discrete gust (MIL-SPEC-8785C) | time (s), gust activation | gust velocity body (m/s) |
| `EnvironmentState` | Domain | Aggregate value object: `AtmosphericState` + wind + turbulence + gust | — | consumed by `Aircraft`, sensors |

---

## Terrain

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `V_Terrain` (abstract) | Domain | Interface: height and surface normal query | NED position (m) | height_m, surface_normal |
| `FlatTerrain` | Domain | Trivial flat terrain at sea level | NED position | height = 0, normal = (0,0,−1) |
| `TerrainMesh` | Domain | LOD triangle mesh terrain; queries closest facet at range-selected LOD | NED position, observer range (m) | height_m, surface_normal, facet color |
| `TerrainTile` | Domain | Subdomain mesh tile (7 LOD levels) | — | — |
| `TerrainCell` | Domain | Single LOD mesh for one tile | — | — |
| `LodSelector` | Domain | Selects LOD for each active cell based on slant range with hysteresis | slant range (m) per cell | `TerrainLod` per cell |
| `MeshQualityVerifier` | Domain | Verifies minimum angle, aspect ratio, and degenerate facet criteria | `TerrainMesh` | `MeshQualityReport` |
| `SimulationFrame` | Domain | ENU coordinate frame origin (WGS84 reference point); converts NED ↔ ECEF | WGS84 datum | transform matrix |

---

## Sensors

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `SensorAirData` | Domain | Pitot-static ADC: differential pressure + static pressure transducers with noise, lag, crossflow error; derives IAS, CAS, EAS, TAS, Mach, baro altitude, OAT | airspeed_body_mps, `AtmosphericState` | `AirDataMeasurement` |
| `SensorGnss` | Domain | GNSS receiver model — **stub only** | `KinematicState` | `GnssMeasurement` |
| `SensorMag` | Domain | Triaxial magnetometer model — **stub only** | `KinematicState` | `MagMeasurement` |
| `SensorLaserAlt` | Domain | Laser altimeter — **stub only** | `KinematicState`, `V_Terrain` | `LaserAltMeasurement` |
| `SensorRadAlt` | Domain | Radar altimeter — **stub only** | `KinematicState`, `V_Terrain` | `RadAltMeasurement` |
| `SensorInsSimulation` | Domain | INS truth-plus-error replacement — **stub only** | `KinematicState` | `InsMeasurement` |
| `SensorAA` | Domain | Angle/angle passive sensor — **stub only** | `KinematicState`, target position | `AngleMeasurement` |
| `SensorAAR` | Domain | Angle/angle/range active sensor — **stub only** | `KinematicState`, target position | `AngleRangeMeasurement` |
| `SensorTrackEstimator` | Domain | Kinematic track estimator — **stub only** | angle/range measurements | `TrackEstimate` |
| `SensorForwardTerrainProfile` | Domain | Forward terrain profiling sensor — **stub only** | `KinematicState`, `V_Terrain` | terrain range returns |

---

## Control Library

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `SISOPIDFF` | Domain | PID with feedforward; derivative filter; integral antiwindup; computes error internally as `command − measurement` | `float command`, `float measurement` (or `float measurement_derivative`) | `float y` (actuator command) |
| `FilterTF`, `FilterSS`, `FilterSS2`, `FilterSS2Clip`, `FilterFIR` | Domain | Discrete-time filter bank (transfer-function, state-space, FIR forms) | `float u` | `float y` |
| `Integrator` | Domain | Trapezoid integrator with optional antiwindup limits | `float u` | `float y` |
| `Derivative` | Domain | Filtered derivative (first-order Tustin) | `float u` | `float y` |
| `Limit` | Domain | Symmetric or asymmetric scalar clamp | `float u` | `float y` |
| `RateLimit` | Domain | Rate-of-change limiter | `float u` | `float y` |
| `Unwrap` | Domain | Phase unwrapper for angular quantities | `float u` | `float y` |
| `Gain` | Domain | Scalar gain with scheduled-gain stub (scheduling not yet implemented) | `float u` | `float y` |

---

## Logging

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `Logger` | Application | Records logged values at each step; writes MCAP and CSV | named scalar / vector values | `.mcap`, `.csv` files |
| `LogSource` | Application | Per-component logging interface; submits named channels to `Logger` | component state | — |
| `LogReader` | Application | Reads and decodes MCAP log files | `.mcap` file | data frames |

---

## Navigation (Stubs)

| Element | Layer | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- | --- |
| `NavigationFilter` | Domain | EKF/UKF navigation filter — **stub only** | sensor measurements | `NavigationState` |
| `WindEstimator` | Domain | Wind estimation from navigation + air data — **stub only** | `NavigationState`, `AirDataMeasurement` | wind_NED_mps |
| `FlowAnglesEstimator` | Domain | Alpha/beta estimation — **stub only** | wind estimate, `AirDataMeasurement` | alpha_rad, beta_rad |

---

## Flight Code Stubs (Not Part of LiteAeroSim)

These elements have stub headers in the repository as temporary placeholders. Per the
future-state architecture, they will be relocated to a separate flight code component.

| Element | Responsibility |
| --- | --- |
| `Autopilot` | Inner-loop flight control — **stub only** |
| `PathGuidance`, `VerticalGuidance`, `ParkTracking` | Guidance laws — **stub only** |
| `V_PathSegment`, `PathSegmentHelix`, `Path` | Path representation — **stub only** |

---

## Python Tooling

| Element | Responsibility |
| --- | --- |
| Terrain pipeline (`las_terrain.py`, `download.py`, `mosaic.py`, `geoid_correct.py`, `triangulate.py`, `colorize.py`, `simplify.py`, `verify.py`, `export.py`, `export_gltf.py`) | Offline terrain data ingestion from DEM sources to `.las_terrain` |
