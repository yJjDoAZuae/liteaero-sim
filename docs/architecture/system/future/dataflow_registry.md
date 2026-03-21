# Data Flow Registry — Future State

Present-state DFT-1 through DFT-8 carry forward (see `present/dataflow_registry.md`).

---

## New and Modified Data Flow Types

| ID | Type | Producer | Consumers | Description |
| --- | --- | --- | --- | --- |
| DFT-9 | `NavigationState` | `NavigationFilter` | `Autopilot`, `WindEstimator`, Logger | Estimated position (NED or WGS84), NED velocity, attitude, angular rates, covariance |
| DFT-10 | `SetPoint` | Guidance components | `Autopilot` | Target altitude (m), vertical speed (m/s), heading (rad), roll attitude (rad) |
| DFT-11 | `PathQuery` / `PathResponse` | `PathGuidance` | `Path` | Query: position NED, heading. Response: crosstrack error (m), along-track (m), desired heading (rad), desired altitude (m), segment-complete flag |
| DFT-12 | `GroundContactForces` | `LandingGear` | `Aircraft` | Body-frame forces (N) and moments (N·m) from all active wheel contact points |
| DFT-13 | `SimulationState` | `SimRunner` (aggregated) | External interfaces, Logger | Full simulation snapshot: `KinematicState` + sensor measurements + flight code state; used for HITL streaming, visualization, and logging |
| DFT-14 | Telemetry / command stream | `QGroundControlLink`, `ArduPilotInterface`, `PX4Interface` | External ground stations / autopilot platforms | MAVLink messages (attitude, position, airspeed, mode, mission items) |

---

## Data Flow Instance Registry

| Instance | Type | From | To | Notes |
| --- | --- | --- | --- | --- |
| aircraft-command | DFT-1 | `Autopilot` (or manual input) | `Aircraft::step()` | In closed-loop mode, produced by Autopilot; in open-loop, by test harness or `ManualInput` |
| navigation-state | DFT-9 | `NavigationFilter` | `Autopilot`, `WindEstimator`, Logger | In simulation, may be replaced by `SensorInsSimulation` output |
| set-point | DFT-10 | Guidance (`PathGuidance`, `VerticalGuidance`, `ParkTracking`) | `Autopilot` | Outer loop → inner loop |
| path-query | DFT-11 | `PathGuidance` | `Path` | Path geometry query each step |
| ground-contact | DFT-12 | `LandingGear` | `Aircraft::step()` | Active only when any wheel is in contact; zero force/moment when airborne |
| simulation-state | DFT-13 | `SimRunner` | Visualization, HITL link | Streamed at simulation update rate |
| sensor-measurements | DFT-5 (+ others) | All sensors | `NavigationFilter` (or `SensorInsSimulation`) | In SITL/HITL, also streamed to flight hardware |
| telemetry-up | DFT-14 | `SimRunner` → interface adapters | QGroundControl, ArduPilot/PX4 | State encoded as MAVLink or custom |
| command-down | DFT-14 | QGroundControl, ArduPilot/PX4 | Interface adapters → Autopilot or `ManualInput` | Mode changes, mission items, override commands |
