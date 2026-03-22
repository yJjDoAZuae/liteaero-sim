# System Element Registry — Future State

All present-state elements carry forward. This document lists additions, changes, and
the full future-state registry organized by component boundary.

---

## System Boundary

The future-state system comprises four top-level architectural components:

| Component | Boundary | Deployment |
| --- | --- | --- |
| **LiteAeroSim** | Simulation plant: aircraft physics, environment, terrain, sensors, logger | Desktop / test server |
| **FlightCode** | Autopilot, guidance, path management, navigation | Flight hardware, companion computer; `flightcode` Docker container (SITL verification); or co-resident with LiteAeroSim (development only) |
| **SimulationRunner** | Execution loop, timing control, batch / real-time modes | Co-resident with LiteAeroSim |
| **External Interfaces** | QGroundControl, game engine, manual input, ArduPilot/PX4 | Various |

---

## Container Topology

In SITL verification configurations, LiteAeroSim and FlightCode run in separate Docker
containers on the same host. This ensures the flight code executes under the same runtime
environment as an active flight software load.

| Container | Contents | Network role |
| --- | --- | --- |
| `liteaerosim` | LiteAeroSim plant, SimulationRunner, logger, external adapters | Exposes ICD-8 port; receives `AircraftCommand`; sends sensor measurements |
| `flightcode` | Autopilot, guidance, path, navigation | Communicates with `liteaerosim` over ICD-8 network transport |

The `flightcode` container image is the flight software load image. No simulation-specific
code is included in this image. The container boundary enforces the ICD-8 architectural
boundary: no shared memory and no direct function calls between containers are possible.

For non-verification development configurations (developer workstations, CI smoke tests),
LiteAeroSim and FlightCode may run in the same process or as co-processes without container
isolation. See `decisions.md` — Docker Containerization for SITL Verification.

---

## LiteAeroSim Elements

All present-state elements carry forward (see `present/element_registry.md`), with the
following additions and clarifications.

| Element | New / Changed | Notes |
| --- | --- | --- |
| `LandingGear` | **New** | Ground contact forces and moments; Pacejka magic formula for tyre; 2nd-order suspension; compatible with `V_Terrain` interface |
| `SimulationRunner` | **New** | Execution loop with real-time, scaled-real-time, and batch modes |
| Flight code stubs | **Relocated** | `Autopilot`, `Guidance`, `Path` stubs removed from LiteAeroSim; relocated to FlightCode component |

---

## FlightCode Elements

The FlightCode component is a separable software system. It is not a subsystem of
LiteAeroSim. Its repository location is to be determined by this architecture definition.

| Element | Responsibility | Ports — Inputs | Ports — Outputs |
| --- | --- | --- | --- |
| `Autopilot` | Inner-loop closed-loop control; tracks set-point commands for altitude hold, vertical speed hold, heading hold, roll attitude hold | `KinematicState` (or `NavigationState`), `AtmosphericState`, set-point struct | `AircraftCommand` |
| `PathGuidance` | Lateral path tracking; nonlinear guidance law (L1 or similar); nulls cross-track error | `PathResponse`, `KinematicState` | heading/roll set point |
| `VerticalGuidance` | Altitude / climb-rate tracking | target altitude, `KinematicState` | altitude / vertical-speed set point |
| `ParkTracking` | Loiter / station-keep | target point, `KinematicState` | heading and altitude set points |
| `V_PathSegment`, `PathSegmentHelix`, `Path` | Path representation; cross-track error, along-track distance, desired heading at a query point | `PathQuery` (position, heading) | `PathResponse` |
| `NavigationFilter` | EKF/UKF navigation: fuses GNSS, air data, magnetometer, INS into navigation state | sensor measurements | `NavigationState` |
| `WindEstimator` | Estimates wind NED from navigation state and air data | `NavigationState`, `AirDataMeasurement` | wind_NED_mps |
| `FlowAnglesEstimator` | Estimates alpha and beta from wind estimate and air data | wind estimate, `AirDataMeasurement` | alpha_rad, beta_rad |

**Key architectural constraints on FlightCode:**

- No FlightCode element may hold a pointer or reference into LiteAeroSim simulation
  internals. All communication is through defined interface data types.
- FlightCode elements must support `reset()` and initialization to arbitrary conditions.
- Estimation functions (`NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`) are
  within the Navigation subsystem boundary; they are not within the Autopilot boundary.

**HITL hardware deployment:**

In HITL configurations, FlightCode elements are distributed across two hardware processors:

| Processor | Typical FlightCode elements | Notes |
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
| `SimRunner` | Execution loop; paces steps to wall clock (real-time / scaled) or runs free (batch); calls `Aircraft::step()` and environment steps in correct order | `RunnerConfig` | step counter, elapsed sim time |

---

## External Interface Elements

| Element | Responsibility | Protocol |
| --- | --- | --- |
| `ManualInput` | Translates joystick / RC transmitter inputs to `AircraftCommand` | USB HID (SDL2 or platform API) |
| `QGroundControlLink` | Telemetry and mission planning interface to QGroundControl | MAVLink over UDP |
| `ArduPilotInterface` | SITL/HITL interface to ArduPilot | ArduPilot SITL protocol / MAVLink |
| `PX4Interface` | SITL/HITL interface to PX4 | PX4 SITL bridge / MAVLink |
| `VisualizationLink` | Real-time or scaled-real-time state stream to game engine | TBD (JSON over WebSocket, or binary protocol) |
