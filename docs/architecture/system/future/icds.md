# Interface Control Documents — Future State

At this stage, ICDs identify each interface and ensure the architecture accommodates it.
Field-level schema definitions are deferred to the software design phase.

---

## ICD-8 — Simulation ↔ FlightCode Interface (Plant Interface)

This is the primary runtime boundary between LiteAeroSim and the FlightCode component.

**Producers / Consumers:**

- LiteAeroSim → FlightCode: sensor measurements, navigation state (if produced by simulation)
- FlightCode → LiteAeroSim: `AircraftCommand`

**Transport options (any of the following; selected by deployment configuration):**

- Direct in-process function call (co-resident SITL, same process)
- Local IPC (co-resident SITL, separate processes on same host)
- Network message (HITL; companion-computer SITL)
- MAVLink over UDP (ArduPilot/PX4 integration)

**Data crossing this boundary:**

| Direction | Data | Type | Notes |
| --- | --- | --- | --- |
| LAS → FC | Sensor measurements | DFT-5 and others | Per-sensor measurement structs |
| LAS → FC | Simulation time | float (s) | Elapsed simulation time |
| FC → LAS | Aircraft command | DFT-1 | Normal/lateral load factor, throttle |

**Constraints:**

- All values SI units regardless of transport encoding.
- The interface must be encodable in MAVLink for ArduPilot/PX4 compatibility. Where
  MAVLink message types are insufficient, custom message definitions are required.
- Timing: in real-time SITL, commands must arrive within one timestep; late commands
  are held at last value (zero-order hold).
- The interface is stateless: LiteAeroSim does not depend on any FlightCode internal state.

---

## ICD-9 — FlightCode ↔ Navigation State Interface

**Producer:** `NavigationFilter` (or `SensorInsSimulation` as a simulation substitute)

**Consumer:** `Autopilot`, `WindEstimator`, Logger

**Transport:** Direct function call (FlightCode component-internal)

**Content:**

| Field group | Content | Unit |
| --- | --- | --- |
| Position | NED position or WGS84 | m or deg |
| Velocity | NED velocity estimate | m/s |
| Attitude | Estimated quaternion + Euler | rad |
| Angular rates | Body-frame rate estimate | rad/s |
| Covariance | Position, velocity, attitude estimate covariances | SI² |
| Validity flags | Fix quality, sensor health flags | nd |

**Constraints:**

- The autopilot consumes `NavigationState` only; it has no direct access to raw sensor data.
- In simulation, `SensorInsSimulation` may substitute for `NavigationFilter` to reduce
  computational cost (truth-plus-error model, ~35× faster than a full EKF).

---

## ICD-10 — External Ground Station Interface (QGroundControl)

**Producer/Consumer:** `QGroundControlLink` adapter (bidirectional)

**Transport:** MAVLink over UDP

**Key message types (inbound to system):**

- `MISSION_ITEM`, `MISSION_COUNT`, `MISSION_REQUEST` — mission upload
- `COMMAND_LONG` (`MAV_CMD_DO_SET_MODE`, etc.) — mode changes
- `RC_CHANNELS_OVERRIDE` — manual override from GCS

**Key message types (outbound from system):**

- `ATTITUDE`, `GLOBAL_POSITION_INT` — state telemetry
- `VFR_HUD` — airspeed, altitude, heading, throttle display
- `MISSION_CURRENT`, `MISSION_ITEM_REACHED` — mission progress
- `STATUSTEXT` — status and alert messages

**Constraints:**

- Where MAVLink provides insufficient expressiveness, the interface definition must
  identify the gap and propose a resolution (custom message, companion protocol, or
  architectural workaround).

---

## ICD-11 — Visualization Interface (Game Engine)

**Producer:** `SimRunner` / `VisualizationLink` adapter

**Consumer:** Game engine (external)

**Transport:** TBD (JSON over WebSocket and binary UDP protocol are candidate options)

**Content:** `SimulationState` (DFT-13): position, attitude, airspeed, control surface deflections, landing gear state, terrain mesh reference.

**Constraints:**

- Must support real-time streaming at simulation update rate.
- Must support scaled-real-time (playback faster or slower than real time).
- Coordinate frame conversion (NED → game-engine convention) is the responsibility of the adapter, not the simulation.

---

## ICD-12 — Manual Input Interface

**Producer:** Joystick / RC transmitter (USB HID)

**Consumer:** `ManualInput` adapter → `AircraftCommand`

**Transport:** USB HID (SDL2 or platform API)

**Content:**

- Axis values mapped to throttle, roll rate command, pitch command, yaw command.
- Button / switch states mapped to mode selections and override commands.

**Constraints:**

- Dead zone and axis scaling are configurable.
- `ManualInput` outputs `AircraftCommand` in SI units.
- Pilot inputs should be mappable to RC transmitter switch inputs for traffic pattern operator interface requirements.
