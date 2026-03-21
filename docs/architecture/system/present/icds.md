# Interface Control Documents — Present State

At the present stage, ICDs identify each interface, its data content, and the
architectural constraints that govern it. Field-level schema definitions are in the C++
headers and `proto/liteaerosim.proto`; they are not reproduced here.

---

## ICD-1 — AircraftCommand

**Producer:** Simulation framework (test harness); future Autopilot component.

**Consumer:** `Aircraft::step()`

**Transport:** Direct function-call argument (in-process, same translation unit).

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `n` | g | Commanded normal load factor |
| `n_y` | g | Commanded lateral load factor |
| `n_dot` | 1/s | Rate of change of `n` (for alpha-dot feed-forward) |
| `n_y_dot` | 1/s | Rate of change of `n_y` (for beta-dot feed-forward) |
| `rollRate_Wind_rps` | rad/s | Commanded wind-frame roll rate |
| `throttle_nd` | nd | Normalized throttle [0, 1] |

**Constraints:**

- All values SI. No unit conversion inside `Aircraft`.
- `n = 1` is 1 g (level, unaccelerated flight). `n = 0` is zero g (free fall).
- `throttle_nd` is clamped to [0, 1] inside `Aircraft`; the caller need not pre-clamp.

---

## ICD-2 — AtmosphericState

**Producer:** `Atmosphere::step()`

**Consumers:** `EnvironmentState` assembly; `SensorAirData::step()`; `Propulsion` subclasses.

**Transport:** Value struct passed by value or const reference.

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `temperature_k` | K | Static (ambient) temperature |
| `pressure_pa` | Pa | Static pressure |
| `density_kgm3` | kg/m³ | Moist air density |
| `speed_of_sound_mps` | m/s | Speed of sound |
| `relative_humidity_nd` | nd | Relative humidity [0, 1] |
| `density_altitude_m` | m | ISA altitude at which ρ\_ISA = ρ\_actual |

**Constraints:**

- Queried once per step at the current geometric altitude of the aircraft.
- The struct is immutable after production; no consumer modifies it.

---

## ICD-3 — EnvironmentState

**Producer:** Simulation loop (assembles from Atmosphere, Wind, Turbulence, Gust).

**Consumer:** `Aircraft::step()`

**Transport:** Value struct.

**Content:**

| Field | Type/Unit | Description |
| --- | --- | --- |
| `atmosphere` | `AtmosphericState` | See ICD-2 |
| `wind_NED_mps` | m/s (NED vector) | Steady ambient wind velocity |
| `turbulence` | `TurbulenceVelocity` | Body-frame continuous turbulence (m/s + rad/s) |
| `gust_body_mps` | m/s (body vector) | Discrete gust velocity |

**Constraints:**

- All velocities SI.
- Wind is expressed in NED frame; turbulence and gust in body frame.

---

## ICD-4 — KinematicState

**Producer:** `Aircraft::step()` (updates internal state; accessor returns const reference).

**Consumers:** All sensors; Logger; future Autopilot and Navigation components.

**Transport:** Const reference or value copy.

**Key content (selected fields):**

| Group | Content | Unit |
| --- | --- | --- |
| Position | WGS84 datum (latitude, longitude, altitude above ellipsoid) | deg, deg, m |
| NED velocity | `velocity_NED_mps` | m/s |
| Attitude | Quaternion `q_nb` (NED to body); Euler angles (roll, pitch, yaw) | rad |
| Body rates | `p`, `q`, `r` | rad/s |
| Body acceleration | `acceleration_body_mps2` | m/s² |
| Aerodynamic angles | `alpha_rad`, `beta_rad` | rad |
| Airspeed | `Va_mps` | m/s |

**Constraints:**

- `KinematicState` is owned by `Aircraft`; external components hold const references or
  copy the struct.
- Angular rates are body-frame; Euler angles use 3-2-1 (yaw–pitch–roll) sequence.

---

## ICD-5 — AirDataMeasurement

**Producer:** `SensorAirData::step()`

**Consumers:** Logger; future NavigationFilter; future Autopilot (altitude/airspeed hold).

**Transport:** Return value from `step()`.

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `ias_mps` | m/s | Indicated airspeed (incompressible Bernoulli, ρ₀ reference) |
| `cas_mps` | m/s | Calibrated airspeed (isentropic, ρ₀ reference) |
| `eas_mps` | m/s | Equivalent airspeed (dynamic pressure equivalent at ρ₀) |
| `tas_mps` | m/s | True airspeed |
| `mach_nd` | nd | Mach number |
| `baro_altitude_m` | m | Barometric altitude (Kollsman-referenced) |
| `oat_k` | K | Outside air temperature |

**Constraints:**

- All values include configured noise, lag, and crossflow pressure error.
- `ias_mps`, `cas_mps`, `eas_mps`, `tas_mps`, `mach_nd` are clamped to ≥ 0.

---

## ICD-6 — Terrain Query Interface (`V_Terrain`)

**Producer:** `FlatTerrain` or `TerrainMesh`

**Consumers:** Future `SensorRadAlt`, `SensorLaserAlt`; future landing gear contact model; future guidance.

**Transport:** Virtual function call.

**Interface:**

```cpp
float heightAboveSeaLevel_m(Eigen::Vector3f position_NED_m) const
Eigen::Vector3f surfaceNormal_NED(Eigen::Vector3f position_NED_m) const
```

**Constraints:**

- Height is above the WGS84 ellipsoid (approximately MSL for low-altitude operations).
- Surface normal is unit vector expressed in NED frame.
- Both `FlatTerrain` and `TerrainMesh` are conforming implementations.

---

## ICD-7 — Logger Write Interface

**Producer:** All domain components (via `LogSource`)

**Consumer:** `Logger`

**Transport:** Method call.

**Constraints:**

- All logged quantities must be in SI units.
- Channel names are strings; units are encoded in the name where not obvious.
- The `Logger` is write-only during simulation; reading uses `LogReader`.
