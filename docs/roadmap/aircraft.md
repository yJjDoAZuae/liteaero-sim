# Aircraft Class — Recommended Next Steps

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

Items are listed in dependency order. Each item follows TDD: write a failing test before
writing production code.

---

## Current State

| Component | File | Status |
|-----------|------|--------|
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `WGS84_Datum` | `include/navigation/WGS84.hpp` | ✅ Implemented |
| `AeroPerformance` | `include/aerodynamics/AeroPerformance.hpp` | ⚠️ Old-style stub — no namespace, no `LoadFactorAllocator` integration |
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ⚠️ Plain struct — no serialization |
| `Inertia` | `include/airframe/Inertia.hpp` | ⚠️ Plain struct — no serialization |
| `V_Propulsion` | `include/propulsion/V_Propulsion.hpp` | ❌ Stub comment only |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ❌ Stub comment only |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ❌ Stub comment only |
| `Aircraft` | `include/Aircraft.hpp` | ❌ Stub comment only |

---

## 1. Propulsion Virtual Interface (`V_Propulsion`)

`Aircraft` aggregates a propulsion model through a virtual base so that different engine
types can be substituted without changing the `Aircraft` class.

### Responsibilities

- Accept a normalized throttle demand and atmospheric conditions; return thrust (N).
- Maintain internal lag state between calls (e.g., engine spool-up time constant).

### Interface

```cpp
// include/propulsion/V_Propulsion.hpp
namespace liteaerosim::propulsion {

class V_Propulsion {
public:
    virtual ~V_Propulsion() = default;

    // Advance propulsion state by one timestep and return thrust magnitude (N).
    //   throttle_nd  — normalized demand [0, 1]
    //   tas_mps      — true airspeed (m/s), for ram-drag and power corrections
    //   rho_kgm3     — air density (kg/m³), for density-corrected thrust
    virtual float step(float throttle_nd, float tas_mps, float rho_kgm3) = 0;

    // Thrust output from the last step() call (N).
    virtual float thrust_n() const = 0;

    // Reset warm-start state to zero thrust.
    virtual void reset() = 0;
};

} // namespace liteaerosim::propulsion
```

### Tests

- A concrete subclass can be constructed, `step()` called, and `thrust_n()` returns the same value.
- After `reset()`, `thrust_n()` returns 0.

---

## 2. `PropulsionJet` — First-Order Thrust Model

A simple enclosed-jet model: static thrust limited by a first-order throttle-response lag.

### Parameters

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `thrust_max_n` | float | N | Maximum static thrust at sea level |
| `throttle_lag_s` | float | s | First-order time constant for throttle response; 0 = instantaneous |
| `dt_s` | float | s | Fixed timestep (configuration parameter, not a `step()` argument) |

### Algorithm

```
thrust_demand = throttle_nd * thrust_max_n
thrust_n += (thrust_demand - thrust_n) / throttle_lag_s * dt_s   // first-order lag
                                                                  // (skip lag if lag_s == 0)
```

Density correction and ram drag are deferred to a later iteration of this model.

### Tests

- `throttle_lag_s = 0`: single `step(1.0, ...)` returns `thrust_max_n` exactly.
- `throttle_lag_s > 0`: thrust converges asymptotically from 0 toward `thrust_max_n` over several steps.
- `step(0.0, ...)` after full throttle: thrust decreases monotonically back toward 0.
- `reset()` zeroes the internal thrust state regardless of prior steps.

### CMake

Add `src/propulsion/PropulsionJet.cpp` to the `liteaerosim` target.
Add `test/PropulsionJet_test.cpp` to the test executable.

---

## 3. `PropulsionProp` — Piston/Propeller Thrust Model

A simplified propeller model: thrust is proportional to throttle and air density, with an
airspeed-dependent efficiency factor.

### Parameters

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `power_max_w` | float | W | Maximum shaft power at sea level |
| `propeller_eta` | float | — | Propulsive efficiency η (0–1) |
| `dt_s` | float | s | Fixed timestep |

### Algorithm

```
T = eta * power_max_w * throttle * (rho / rho_SL) / max(tas_mps, V_min)
```

where `V_min` prevents division by zero at zero airspeed (clamp to a small positive value).

### Tests

- At sea-level density and moderate airspeed, thrust equals the analytical formula.
- Thrust decreases with altitude (lower ρ).
- Thrust is 0 at `throttle = 0`.

### CMake

Add `src/propulsion/PropulsionProp.cpp` to the `liteaerosim` target.
Add `test/PropulsionProp_test.cpp` to the test executable.

---

## 4. `AeroPerformance` Redesign

The existing `AeroPerformance` uses a load-factor-derived `CL(N)` formula that predates
`LiftCurveModel` and `LoadFactorAllocator`. It must be replaced with a model that:

1. Accepts α and β (outputs of `LoadFactorAllocator`) rather than load factor N.
2. Computes all three aerodynamic force components in the Wind frame.
3. Moves into the `liteaerosim::aerodynamics` namespace.

The old `AeroPerformance` class is deleted. Any existing call sites are updated directly
(no forwarding shims).

### New interface

```cpp
// include/aerodynamics/AeroPerformance.hpp
namespace liteaerosim::aerodynamics {

struct AeroForces {
    float x_n;   // Wind-frame X (along velocity, positive forward — thrust direction)
    float y_n;   // Wind-frame Y (positive right)
    float z_n;   // Wind-frame Z (positive down — lift is negative z)
};

class AeroPerformance {
public:
    // S_ref_m2   — reference wing area (m²)
    // ar         — aspect ratio
    // e          — Oswald efficiency factor
    // cd0        — zero-lift drag coefficient
    // cl_y_beta  — lateral force slope C_Yβ (rad⁻¹, < 0)
    AeroPerformance(float S_ref_m2, float ar, float e, float cd0, float cl_y_beta);

    // Compute aerodynamic forces in the Wind frame.
    //   alpha_rad, beta_rad — from LoadFactorAllocator::solve()
    //   q_inf_pa            — dynamic pressure (Pa)
    //   cl                  — lift coefficient from LiftCurveModel::evaluate(alpha_rad)
    AeroForces compute(float alpha_rad, float beta_rad,
                       float q_inf_pa, float cl) const;

    // Drag polar parameters (read-only access for diagnostics)
    float cd0() const { return _cd0; }
    float inducedDragK() const { return _k; }

private:
    float _S, _ar, _e, _cd0, _k;   // k = 1 / (π · e · AR)
    float _cl_y_beta;
};

} // namespace liteaerosim::aerodynamics
```

### Algorithm

```
k    = 1 / (π · e · AR)
CDi  = k · CL²
CD   = CD0 + CDi
CY   = C_Yβ · β

Fx   = -q · S · CD          // drag opposes motion (negative X in wind frame)
Fy   =  q · S · CY          // side force
Fz   = -q · S · CL          // lift opposes wind-Z (negative = upward)
```

### Tests

- Level flight: `Fz` consistent with `CL = W / (q·S)`.
- Zero airspeed (q = 0): all forces zero.
- Zero β: `Fy = 0`.
- Drag increases with CL (induced drag term).

### CMake

Replace the existing `AeroPerformance` source file in `src/CMakeLists.txt`.
Add `test/AeroPerformance_test.cpp` to the test executable.

---

## 5. `Aircraft` Class Definition

`Aircraft` owns and orchestrates the full physics update loop. It is not a `DynamicBlock`
(the interface is multi-input, multi-output), but it follows the project's lifecycle
convention (`initialize` → `reset` → `step` → `serialize` / `deserialize`).

### Ownership model

| Member | Type | Owned by |
|--------|------|----------|
| `_state` | `KinematicState` | Aircraft (value member) |
| `_liftCurve` | `LiftCurveModel` | Aircraft (value member) |
| `_allocator` | `LoadFactorAllocator` | Aircraft (value member) |
| `_aeroPerf` | `AeroPerformance` | Aircraft (value member) |
| `_propulsion` | `std::unique_ptr<V_Propulsion>` | Aircraft (owns, injected at construction) |
| `_mass_kg` | `float` | Aircraft |

### Inputs to `step()`

```cpp
struct AircraftCommand {
    float n;              // commanded normal load factor (g)
    float n_y;            // commanded lateral load factor (g)
    float n_dot;          // rate of change of n (1/s) — for alphaDot
    float n_y_dot;        // rate of change of n_y (1/s) — for betaDot
    float rollRate_Wind_rps; // commanded wind-frame roll rate (rad/s)
    float throttle_nd;    // normalized throttle [0, 1]
};
```

### Class interface

```cpp
// include/Aircraft.hpp
namespace liteaerosim {

class Aircraft {
public:
    explicit Aircraft(std::unique_ptr<propulsion::V_Propulsion> propulsion);

    void initialize(const nlohmann::json& config);
    void reset();

    // Advance the aircraft physics by one timestep.
    //   time_sec      — absolute simulation time (s)
    //   cmd           — autopilot-commanded inputs
    //   wind_NED_mps  — ambient wind vector in NED frame (m/s)
    //   rho_kgm3      — local air density (kg/m³)
    void step(double time_sec,
              const AircraftCommand& cmd,
              const Eigen::Vector3f& wind_NED_mps,
              float rho_kgm3);

    const KinematicState& state() const { return _state; }

    nlohmann::json serialize() const;
    void deserialize(const nlohmann::json& snapshot);

private:
    KinematicState                         _state;
    aerodynamics::LiftCurveModel           _liftCurve;
    aerodynamics::LoadFactorAllocator      _allocator;
    aerodynamics::AeroPerformance          _aeroPerf;
    std::unique_ptr<propulsion::V_Propulsion> _propulsion;
    float _mass_kg = 0.f;
    float _dt_s    = 0.f;
};

} // namespace liteaerosim
```

### Tests

- `Aircraft` constructs without throwing given a valid `PropulsionJet` and a representative config.
- `state()` returns the initial `KinematicState` before any `step()` call.

---

## 6. `Aircraft::step()` — Physics Integration Loop

The `step()` method is the closed-loop physics update. It must execute in this order:

```
1. Compute true airspeed from KinematicState and wind:
       V_air = (velocity_NED - wind_NED).norm()

2. Compute dynamic pressure:
       q_inf = 0.5 * rho * V_air²

3. Solve for α, β via LoadFactorAllocator:
       LoadFactorInputs in = { cmd.n, cmd.n_y, q_inf, thrust_n (previous step),
                               _mass_kg, cmd.n_dot, cmd.n_y_dot }
       LoadFactorOutputs out = _allocator.solve(in)

4. Evaluate lift coefficient:
       CL = _liftCurve.evaluate(out.alpha_rad)

5. Compute aerodynamic forces in Wind frame:
       AeroForces F = _aeroPerf.compute(out.alpha_rad, out.beta_rad, q_inf, CL)

6. Advance propulsion:
       float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3)

7. Compute net Wind-frame acceleration:
       // Thrust decomposition in Wind frame (see equations_of_motion roadmap)
       ax = (T * cosα * cosβ  + F.x_n) / m     // F.x_n is negative (drag)
       ay = (−T * cosα * sinβ + F.y_n) / m
       az = (−T * sinα        + F.z_n) / m     // F.z_n is negative (lift up)
       // Gravity is already embedded in the load factor — do not add separately

8. Advance KinematicState:
       _state.step(time_sec,
                   {ax, ay, az},           // acceleration_Wind_mps
                   cmd.rollRate_Wind_rps,
                   out.alpha_rad, out.beta_rad,
                   out.alphaDot_rps, out.betaDot_rps,
                   wind_NED_mps);
```

**Note on gravity:** The `LoadFactorAllocator` constraint `q·S·CL + T·sinα = n·m·g` already
encodes the gravitational load. The Wind-frame acceleration computed here is the kinematic
acceleration only — gravity must not be double-counted. Verify this against `KinematicState`
velocity integration before finalizing.

### Tests

- Straight-and-level at `n = 1`: speed is approximately constant over several steps.
- `throttle_nd = 0`, `n = 1`: aircraft decelerates (drag exceeds thrust).
- After `reset()`, position and velocity return to initial values.
- `step()` does not throw for any physically plausible command.

---

## 7. Serialization

`Aircraft` must implement both JSON and binary (protobuf) serialization, capturing the full
restart state of every owned subcomponent with warm-start state. The methods follow the
non-`DynamicBlock` pattern (plain public members, not NVI):

```cpp
nlohmann::json       serializeJson()                              const;
void                 deserializeJson(const nlohmann::json&        j);

std::vector<uint8_t> serializeProto()                            const;
void                 deserializeProto(const std::vector<uint8_t>& bytes);
```

### JSON schema

```json
{
    "schema_version": 1,
    "type": "Aircraft",
    "kinematic_state": { ... },         // KinematicState::serializeJson()
    "allocator_state": { ... },         // LoadFactorAllocator::serializeJson()
    "propulsion_state": { ... },        // V_Propulsion::serializeJson() (TBD per subclass)
    "params": {
        "mass_kg":  1000.0,
        "dt_s":     0.01
    }
}
```

`LiftCurveModel` and `AeroPerformance` are stateless; their parameters are re-read from the
config at `initialize()` time and are not duplicated in the state snapshot.

### Proto message

Add an `Aircraft` message to `proto/liteaerosim.proto` that embeds the existing
`KinematicState` and `LoadFactorAllocatorState` messages.

### Tests

- JSON round-trip: `deserializeJson(serializeJson())` yields identical `KinematicState` and allocator warm-start.
- Proto round-trip: same verification via `deserializeProto(serializeProto())`.
- Schema version mismatch on either deserialize throws `std::runtime_error`.

---

## 8. JSON Initialization

Once the JSON parameter schema (see `docs/roadmap/equations_of_motion.md §4`) is defined,
`Aircraft::initialize(config)` must read from a validated config file and construct all
owned subcomponents.

### Mapping from schema to Aircraft members

| JSON path | Aircraft member |
|-----------|-----------------|
| `aircraft.mass_kg` | `_mass_kg` |
| `aircraft.S_ref_m2` | `AeroPerformance`, `LoadFactorAllocator` |
| `aircraft.cl_y_beta` | `AeroPerformance`, `LoadFactorAllocator` |
| `lift_curve.*` | `LiftCurveParams` → `LiftCurveModel` |
| `initial_state.*` | `KinematicState` constructor |

The `validate_aircraft_config.py` script (see `docs/roadmap/equations_of_motion.md §4`)
must pass before `initialize()` is called. `initialize()` may throw `std::invalid_argument`
on malformed input but is not required to duplicate the full Python-side validation.

### Tests

- `initialize()` with each of the three example JSON files (`general_aviation.json`,
  `jet_trainer.json`, `small_uas.json`) succeeds without throwing.
- `initialize()` with a missing required field throws `std::exception`.
