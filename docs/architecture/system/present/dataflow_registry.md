# Data Flow Registry — Present State

---

## Data Flow Types

| ID | Type | Producer | Consumers | Description |
| --- | --- | --- | --- | --- |
| DFT-1 | `AircraftCommand` | Simulation framework (or Autopilot, when implemented) | `Aircraft` | Commanded normal load factor (g), lateral load factor (g), load factor rates (1/s), wind-frame roll rate (rad/s), normalized throttle [0, 1] |
| DFT-2 | `AtmosphericState` | `Atmosphere` | `Aircraft`, `SensorAirData`, `Propulsion` | Ambient temperature (K), static pressure (Pa), density (kg/m³), speed of sound (m/s), relative humidity (nd), density altitude (m) |
| DFT-3 | `EnvironmentState` | Assembled by simulation framework from environment components | `Aircraft` | Aggregate: `AtmosphericState` + wind NED (m/s) + `TurbulenceVelocity` (body, m/s + rad/s) + gust velocity body (m/s) |
| DFT-4 | `KinematicState` | `Aircraft` | Sensors, Logger, (Autopilot — future) | Position (WGS84), NED velocity (m/s), attitude (quaternion + Euler angles), body angular rates (rad/s), body acceleration (m/s²), alpha (rad), beta (rad), airspeed (m/s) |
| DFT-5 | `AirDataMeasurement` | `SensorAirData` | Logger, (NavigationFilter — future) | IAS, CAS, EAS, TAS (m/s), Mach (nd), barometric altitude (m), OAT (K) |
| DFT-6 | `TerrainQuery` / `TerrainResponse` | Simulation framework | — | Input: NED position (m), observer range (m). Output: height_m (m), surface normal (unit vector NED) |
| DFT-7 | Log channel values | All components (via `LogSource`) | `Logger` | Named scalar or vector quantity at each timestep |
| DFT-8 | `TurbulenceVelocity` | `Turbulence` | `EnvironmentState` | Body-frame turbulence: u\_t, v\_t, w\_t (m/s); p\_t, q\_t, r\_t (rad/s) |

---

## Data Flow Instance Registry

| Instance | Type | From | To | Notes |
| --- | --- | --- | --- | --- |
| aircraft-command | DFT-1 | Simulation framework | `Aircraft::step()` | Currently driven by test harness; Autopilot will produce this in future |
| atmospheric-state | DFT-2 | `Atmosphere::step()` | `EnvironmentState`, `SensorAirData::step()` | Queried once per step at current geometric altitude |
| environment-state | DFT-3 | Assembled in simulation loop | `Aircraft::step()` | Wind from `Wind`; turbulence from `Turbulence`; gust from `Gust` |
| kinematic-state | DFT-4 | `Aircraft::step()` | `SensorAirData::step()`, Logger | Airspeed body vector extracted for sensor input |
| air-data-measurement | DFT-5 | `SensorAirData::step()` | Logger | Available for NavigationFilter (future) |
| terrain-query | DFT-6 | Sensors, future guidance | `V_Terrain` implementation | Currently used by terrain pipeline; will be used by RadAlt, LaserAlt |
| turbulence-velocity | DFT-8 | `Turbulence::step()` | `EnvironmentState` | Expressed in body frame |
