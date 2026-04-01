# System Element Registry — Future State

All present-state elements carry forward. This document lists additions, changes, and
the full future-state registry organized by component boundary.

---

## System Boundary

The future-state system comprises four top-level architectural components:

| Component | Boundary | Deployment |
| --- | --- | --- |
| **LiteAero Sim** | Simulation plant: aircraft physics, environment, terrain, sensors, logger | Desktop / test server |
| **LiteAero Flight** | Autopilot, guidance, path management, navigation | Flight hardware, companion computer; `liteaero-flight` Docker container (SITL verification); or co-resident with LiteAero Sim (development only) |
| **SimulationRunner** | Execution loop, timing control, batch / real-time modes | Co-resident with LiteAero Sim |
| **External Interfaces** | QGroundControl, game engine, manual input, ArduPilot/PX4 | Various |

---

## Container Topology

In SITL verification configurations, LiteAero Sim and LiteAero Flight run in separate Docker
containers on the same host. This ensures the flight code executes under the same runtime
environment as an active flight software load.

| Container | Contents | Network role |
| --- | --- | --- |
| `liteaero-sim` | LiteAero Sim plant, SimulationRunner, logger, external adapters | Exposes ICD-8 port; receives `AircraftCommand`; sends sensor measurements |
| `liteaero-flight` | Autopilot, guidance, path, navigation | Communicates with `liteaero-sim` over ICD-8 network transport |

The `liteaero-flight` container image is the flight software load image. No simulation-specific
code is included in this image. The container boundary enforces the ICD-8 architectural
boundary: no shared memory and no direct function calls between containers are possible.

For non-verification development configurations (developer workstations, CI smoke tests),
LiteAero Sim and LiteAero Flight may run in the same process or as co-processes without container
isolation. See `decisions.md` — Docker Containerization for SITL Verification.

---

## LiteAero Sim Elements

All present-state elements carry forward (see `present/element_registry.md`), with the
following additions and clarifications.

| Element | New / Changed | Notes |
| --- | --- | --- |
| `LandingGear` | **New** | Internal to both `Aircraft` and `Aircraft6DOF`; 2nd-order suspension per strut (spring-damper) for bounce modeling; tyre contact forces (vertical, longitudinal, lateral); nose wheel steering and differential brake inputs; produces `ContactForces`; queries ground elevation via `V_Terrain`; design target is developmental verification of autotakeoff and autolanding — ground contact, bounce, WOW establishment, rollout, taxi; validity bound for `Aircraft` (load-factor): gear-induced pitch and roll moment authority exceedance is not modeled |
| `ContactForces` | **New** | Plain value struct produced by `LandingGear`; `force_body_n` (Eigen::Vector3f, body frame), `moment_body_nm` (Eigen::Vector3f, body frame), `weight_on_wheels` (bool); consumed differently by each aircraft model: `Aircraft` passes vertical and lateral force components to `LoadFactorAllocator` as disturbance terms and applies moments directly to kinematic update; `Aircraft6DOF` applies all six components directly to equations of motion |
| `SimulationRunner` | **New** | Execution loop with real-time, scaled-real-time, and batch modes |
| Flight code stubs | **Relocated** | `Autopilot`, `Guidance`, `Path` stubs removed from LiteAero Sim; relocated to LiteAero Flight component |
| `V_AeroModel` | **Proposed — not yet designed** | Abstract aerodynamic model interface; produces forces and moments in body frame; decouples the 6DOF integrator from the coefficient axis convention; concrete implementations handle any required body-to-wind or wind-to-body transformation internally |
| `BodyAxisCoeffModel` | **Proposed — not yet designed** | Implements `V_AeroModel` using body-axis stability derivatives (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular rates; baseline convention for `Aircraft6DOF`; see OQ-16 |
| `Aircraft6DOF` | **Proposed — not yet designed** | Full 6DOF aircraft dynamics model; depends on `V_AeroModel` for forces and moments; accepts `SurfaceDeflectionCommand` (control surface angles + per-motor throttle) rather than the load-factor `AircraftCommand`; produces `KinematicStateSnapshot`; used directly by ArduPilot and PX4 simulations (no FBW bridge in that topology); see OQ-16 |
| `FBWController` | **Proposed — not yet designed** | Inner-loop FBW control law that bridges the existing load-factor `AircraftCommand` interface to `SurfaceDeflectionCommand` for `Aircraft6DOF`; when paired with `Aircraft6DOF` forms a drop-in replacement for the existing `Aircraft` model for side-by-side comparison; not used in ArduPilot/PX4 simulation topologies |
| `SurfaceDeflectionCommand` | **Proposed — not yet designed** | Plain value struct: control surface deflection angles (elevator, aileron, rudder) and per-motor throttle; the native command interface of `Aircraft6DOF`; produced by `FBWController` (when bridging from load-factor) or directly by ArduPilot/PX4 inner-loop controllers |
| `SensorCamera` | **Proposed — not yet designed** | Synthetic image sensor; generates imagery from terrain and scene model against `V_Terrain`; provides measurements for vision-based navigation in `liteaero::perception` |
| `SensorLidar` | **Proposed — not yet designed** | Synthetic lidar sensor; generates 3D point cloud by ray-casting against `V_Terrain`; provides measurements for terrain-relative navigation and obstacle avoidance |
| `SensorLaserAGL` | **Proposed — not yet designed** | Synthetic laser altimeter; computes range to terrain directly below by ray-casting against `V_Terrain`; provides above-ground-level height measurement |
| `SensorLineOfSight` | **Proposed — not yet designed** | Computes RF link quality and occlusion by ray-casting against `V_Terrain`; supports communication link planning and mission autonomy decisions |

---

## LiteAero Flight Elements

The LiteAero Flight component is a separable software system. It is not a subsystem of
LiteAero Sim. Its repository location is to be determined by this architecture definition.

| Element | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- |
| `Autopilot` | Inner-loop closed-loop control; tracks set-point commands for altitude hold, vertical speed hold, heading hold, roll attitude hold | `KinematicState` (or `NavigationState`), `AtmosphericState`, set-point struct | `AircraftCommand` |
| `PathGuidance` | Lateral path tracking; nonlinear guidance law (L1 or similar); nulls cross-track error | `PathResponse`, `KinematicState` | heading/roll set point |
| `VerticalGuidance` | Altitude / climb-rate tracking | target altitude, `KinematicState` | altitude / vertical-speed set point |
| `ParkTracking` | Loiter / station-keep | target point, `KinematicState` | heading and altitude set points |
| `V_PathSegment`, `PathSegmentHelix`, `Path` | Path representation; cross-track error, along-track distance, desired heading at a query point | `PathQuery` (position, heading) | `PathResponse` |
| `NavigationFilter` | EKF/UKF navigation: fuses GNSS, air data, magnetometer, INS, and vision/lidar measurements into navigation state | sensor measurements | `NavigationState` |
| `WindEstimator` | Estimates wind NED from navigation state and air data | `NavigationState`, `AirDataMeasurement` | wind_NED_mps |
| `FlowAnglesEstimator` | Estimates alpha and beta from wind estimate and air data | wind estimate, `AirDataMeasurement` | alpha_rad, beta_rad |
| `VisionNavigator` | **Proposed — not yet designed** (`liteaero::perception`): estimates position/attitude from synthetic or real imagery against terrain model; fuses into `NavigationFilter` | camera measurements, `V_Terrain` | position/attitude observation |
| `LidarTerrainEstimator` | **Proposed — not yet designed** (`liteaero::perception`): processes lidar point cloud against terrain model for terrain-relative navigation | lidar measurements, `V_Terrain` | position/altitude observation |
| `LinkBudgetEstimator` | **Proposed — not yet designed** (`liteaero::mission_autonomy`): assesses RF link quality using line-of-sight terrain occlusion analysis; informs waypoint planning and communication link selection | position, `V_Terrain` | link quality estimate |

**Key architectural constraints on LiteAero Flight:**

- No LiteAero Flight element may hold a pointer or reference into LiteAero Sim simulation
  internals. All communication is through defined interface data types.
- LiteAero Flight elements must support `reset()` and initialization to arbitrary conditions.
- Estimation functions (`NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`) are
  within the Navigation subsystem boundary; they are not within the Autopilot boundary.

**HITL hardware deployment:**

In HITL configurations, LiteAero Flight elements are distributed across two hardware processors:

| Processor | Typical LiteAero Flight elements | Notes |
| --- | --- | --- |
| Autopilot board | `Autopilot` (inner-loop control) | ArduPilot or PX4 firmware; stock or modified variant |
| Companion computer | `NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`, `PathGuidance`, `VerticalGuidance` | Communicates with autopilot board over MAVLink |

The split above applies to UC-3a through UC-3d. In modified-autopilot variants (UC-3c/UC-3d),
some navigation and guidance logic may run within the modified autopilot firmware rather than
exclusively on the companion computer; the exact split depends on the firmware extension
mechanism used (see OQ-10, OQ-11).

---

## SimulationRunner Elements

| Element | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- |
| `SimRunner` | Execution loop; paces steps to wall clock (real-time / scaled) or runs free (batch); calls `Aircraft::step()` and environment steps in correct order; optional `ManualInput` adapter injected via `setManualInput()`; echoes `ManualInputFrame` each tick via `lastManualInputFrame()` | `RunnerConfig`, optional `ManualInput*` | step counter, elapsed sim time, `ManualInputFrame` snapshot |

---

## External Interface Elements

| Element | Responsibility | Protocol / Notes |
| --- | --- | --- |
| `ManualInput` | Abstract base for all manual control adapters; returns `ManualInputFrame` (`AircraftCommand` + `InputAction` bitmask) from `read()` | — |
| `KeyboardInput` | Integrating keyboard adapter; configurable key bindings and named action keys | SDL2 keyboard state |
| `JoystickInput` | SDL2 joystick adapter; per-axis calibration, dead zone, trim; named action buttons; device selection by name or index; disconnect fallback | SDL2 joystick API (USB HID) |
| `ScriptedInput` | Mutex-protected command slot; `push(AircraftCommand)` callable from Python via pybind11 | pybind11 in-process |
| `joystick_verify` | Standalone C++ verification executable; SDL polling loop; streams `ManualInputFrame` to stdout (JSON lines or protobuf); joystick-only scope initially | stdout subprocess |
| `QGroundControlLink` | Telemetry and mission planning interface to QGroundControl | MAVLink over UDP |
| `ArduPilotInterface` | SITL/HITL interface to ArduPilot | ArduPilot SITL protocol / MAVLink |
| `PX4Interface` | SITL/HITL interface to PX4 | PX4 SITL bridge / MAVLink |
| `VisualizationLink` | Real-time or scaled-real-time state stream to game engine | TBD (JSON over WebSocket, or binary protocol) |
