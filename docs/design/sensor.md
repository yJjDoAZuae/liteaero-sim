# Sensor Subsystem — Simulation Models of Sensor Hardware

This document is the design authority for all sensor simulation classes in LiteAero Sim.
It defines the lifecycle contract, the serialization requirements, and the conventions that
every concrete sensor class must follow.

Sensor classes model the behavior of real hardware sensors: noise, bias, quantization, update
rate, and failure modes. They are simulation components. They are not flight code and are not
deployed on aircraft hardware. On a real aircraft the physical sensor provides these outputs
directly; in simulation a sensor class generates equivalent outputs from the simulator's truth
state.

---

## Scope

All sensor classes in `include/sensor/` derive from `liteaerosim::DynamicElement` (see
[dynamic_element.md](dynamic_element.md)) and live in the `liteaerosim::sensor` namespace.
The base class enforces a common lifecycle and serialization contract via the Non-Virtual
Interface (NVI) pattern, while each concrete sensor defines its own type-specific `step()`
signature.

| Class | Header | Hardware modeled |
| --- | --- | --- |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | Pitot-static air data computer — IAS, CAS, EAS, TAS, Mach, baro altitude, OAT |
| `SensorGnss` | `include/sensor/SensorGnss.hpp` | GNSS receiver — WGS84 position, NED velocity, SOG/COG, MSL altitude, fix type, DOP |
| `SensorLaserAlt` | `include/sensor/SensorLaserAlt.hpp` | Laser altimeter — single-beam slant range and AGL altitude via terrain query |
| `SensorMag` | `include/sensor/SensorMag.hpp` | Triaxial magnetometer — body-frame magnetic field with hard-iron bias and soft-iron distortion |
| `SensorInsSimulation` | `include/sensor/SensorInsSimulation.hpp` | INS — truth-plus-error simulation replacement for `NavigationFilter`; same `InsMeasurement` output at ~35× lower cost |
| `SensorRadAlt` | `include/sensor/SensorRadAlt.hpp` | Radar altimeter — terrain-relative altitude via terrain query |
| `SensorAA` | `include/sensor/SensorAA.hpp` | Angle/angle passive sensor — azimuth and elevation to a target, no range |
| `SensorAAR` | `include/sensor/SensorAAR.hpp` | Angle/angle/range active sensor — azimuth, elevation, and slant range to a target |
| `SensorTrackEstimator` | `include/sensor/SensorTrackEstimator.hpp` | Track estimator — filtered position and velocity estimate of a moving object |
| `SensorForwardTerrainProfile` | `include/sensor/SensorForwardTerrainProfile.hpp` | Forward terrain profiling sensor — multi-beam LIDAR or line-scan radar range returns |

---

## Why Sensors Do Not Define a Common `step()` Signature

`SisoElement` can define a common `step(float u) -> float` because all SISO control
elements share the same I/O type. Sensors do not share a common I/O type:

- `SensorAirData::step()` takes a body-frame airspeed vector and `AtmosphericState`.
- `SensorInsSimulation::step()` takes a `KinematicState` truth and optional aiding pointers.
- `SensorRadAlt::step()` takes a terrain query interface and geometric altitude.
- `SensorAA::step()` takes `KinematicState` and a target position.

Imposing a type-erased or polymorphic `step()` on the base class would require variant
inputs, void-pointer casts, or type-erased output buffers — all of which add complexity
with no benefit, since the simulation loop already knows which concrete sensor type it is
calling.

`DynamicElement` therefore provides only the lifecycle methods. The simulation loop holds
concrete sensor types (or pointers to concrete types) and calls the typed `step()` directly.

---

## Lifecycle Contract

All concrete sensor classes must support the following lifecycle, in order:

```text
initialize(config)  ─── one-time setup from JSON config
       │
       ▼
    reset()         ─── zero filter states, re-seed RNG; may be called
       │               between runs without re-reading config
       ▼
  step(...)  ×N     ─── advance one timestep; N times per run
       │
       ▼
 serializeJson() / deserializeJson()  ─── checkpoint at any point after initialize()
```

`initialize()` must be called before any other method. Behavior is undefined if `step()`
or `serializeJson()` is called on an uninitialized sensor.

`reset()` returns the sensor to its post-`initialize()` state: filter states zeroed (or
initialized to steady-state, per the sensor's design), RNG re-seeded with the config seed.

The NVI pattern is implemented in `DynamicElement`. Schema version validation fires in
`DynamicElement::initialize()` and `DynamicElement::deserializeJson()` before the
protected hooks are called.

---

## Serialization Requirements

Every stateful concrete sensor class must implement:

1. **JSON round-trip** — `serializeJson()` / `deserializeJson()`. After deserialization,
   the next call to `step()` must produce output identical to what the original instance
   would have produced. A round-trip test is required (see Test Requirements below).

2. **Proto round-trip** — `serializeProto()` / `deserializeProto()` with a sensor-specific
   proto message. The proto message must be defined in `proto/liteaerosim.proto`. A
   round-trip test is required.

3. **Schema version field** — `"schema_version"` (JSON) and field `schema_version = 1`
   (proto) in every serialized object.

Stateless sensors (if any) need not implement serialization beyond the empty stubs
required to satisfy the abstract interface.

---

## RNG Serialization Convention

Sensor classes that contain an internal random number generator (`std::mt19937`) must
serialize the RNG state as a **seed + advance count** pair, not as raw engine bytes. This
ensures portability across platforms and proto compatibility.

On deserialization, the engine is re-seeded with the stored seed and then advanced by
`rng_advance` draws. The concrete class must maintain an `advance_count` that increments
once per variate drawn.

`seed == 0` in config means non-deterministic seeding via `std::random_device`. When
`seed == 0`, the actual seed used (from `std::random_device`) must be stored at
initialization time so that a subsequent `serializeJson()` can reproduce the sequence.
The stored seed is included in the serialized state.

---

## Naming Conventions

| Item | Convention | Example |
| --- | --- | --- |
| Concrete class name | `Sensor` + descriptor | `SensorAirData`, `SensorGnss` |
| Output struct | descriptor + `Measurement` or `Output` | `AirDataMeasurement` |
| Config struct | descriptor + `Config` | `AirDataConfig` |
| Proto config message | descriptor + `Config` (proto) | `AirDataConfig` |
| Proto state message | descriptor + `StateProto` | `AirDataStateProto` |
| Test file | `SensorXxx_test.cpp` | `SensorAirData_test.cpp` |
| Test class | `SensorXxxTest` | `SensorAirDataTest` |
| Header | `include/sensor/SensorXxx.hpp` | `include/sensor/SensorAirData.hpp` |
| Source | `src/sensor/SensorXxx.cpp` | `src/sensor/SensorAirData.cpp` |

---

## Test Requirements

These tests verify the `DynamicElement` base class contract as applied to sensors. They are
implemented once per concrete sensor class. Every concrete sensor implementation must pass
all applicable tests.

| ID | Test Name | Description |
| --- | --- | --- |
| TB-1 | `JsonRoundTrip_PreservesState` | Serialize after N steps; deserialize into a new instance; first `step()` output is identical between original and restored instance. |
| TB-2 | `ProtoRoundTrip_PreservesState` | Same as TB-1 using proto serialization. |
| TB-3 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `"schema_version"` ≠ expected value throws `std::runtime_error`. |
| TB-4 | `Reset_ReturnsToInitialCondition` | After N steps, `reset()` followed by the same input sequence produces output identical to a freshly initialized instance. |
| TB-5 | `IdenticalSeeds_IdenticalOutputs` | (Stochastic sensors only) Two instances with the same nonzero seed and same input sequence produce bitwise-identical output for N steps. |

Each concrete sensor's test file covers TB-1 through TB-5 (or notes which tests are
not applicable and why) in addition to sensor-specific behavioral tests.
