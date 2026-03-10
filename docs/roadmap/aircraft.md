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
| `AeroPerformance` | `include/aerodynamics/AeroPerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ⚠️ Plain struct — no serialization |
| `Inertia` | `include/airframe/Inertia.hpp` | ⚠️ Plain struct — no serialization |
| `V_Propulsion` | `include/propulsion/V_Propulsion.hpp` | ✅ Implemented |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented |
| `V_Motor` | `include/propulsion/V_Motor.hpp` | ✅ Implemented |
| `MotorElectric` | `include/propulsion/MotorElectric.hpp` | ✅ Implemented |
| `MotorPiston` | `include/propulsion/MotorPiston.hpp` | ✅ Implemented |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Aircraft` | `include/Aircraft.hpp` | ❌ Stub comment only |

---

## ~~1. Propulsion Virtual Interface (`V_Propulsion`)~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — V_Propulsion`](../architecture/propulsion.md#v_propulsion--abstract-interface).

Implement `include/propulsion/V_Propulsion.hpp` per the design. The interface includes
`step()`, `thrust_n()`, `reset()`, and both JSON + proto serialization pure-virtuals.

### Tests

- A concrete stub subclass constructs, `step()` is callable, and `thrust_n()` returns the value from `step()`.
- After `reset()`, `thrust_n()` returns 0.
- `deserializeJson()` with mismatched `"type"` throws `std::runtime_error`.

---

## ~~2. `PropulsionJet` — Physics-Based Jet Model~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — PropulsionJet`](../architecture/propulsion.md#propulsionjet--physics-based-jet-engine-model).

The model covers: altitude lapse (density exponent from BPR), ram drag, idle thrust floor,
spool dynamics via `FilterSS2Clip`, and optional afterburner via a second `FilterSS2Clip`.
All per-step limits are tracked using `valLimit.setLower` / `setUpper` on each filter.

### Tests

- **Density lapse:** at 50% sea-level density, full-throttle thrust equals `T_sl * (0.5)^n`.
- **Ram drag:** at constant rho and increasing `tas_mps`, `thrust_n()` decreases by `rho * V² * A_inlet`.
- **Idle floor:** thrust never falls below `idle_fraction * thrust_sl * (rho/rho_sl)` regardless of throttle.
- **Spool lag:** after a step from 0 to full throttle, thrust after one step is less than `T_avail`; after many steps it converges to `T_avail`.
- **Afterburner:** `setAfterburner(true)` causes `thrust_n()` to increase beyond dry thrust; `setAfterburner(false)` causes it to decay at the AB time constant.
- **reset():** `thrust_n()` returns 0; spool and AB filter states are zeroed.
- **JSON round-trip (steady-state):** `deserializeJson(serializeJson())` after convergence produces `EXPECT_FLOAT_EQ` on the next `step()`.
- **JSON round-trip (mid-transient):** same snapshot taken after 5 steps (deep in spool transient) produces `EXPECT_FLOAT_EQ` — exact reconstruction via filter state vector `x`.
- **Proto round-trip:** both variants via `deserializeProto(serializeProto())`.
- **Type mismatch:** `deserializeJson()` with wrong `"type"` throws.

### CMake

Add `src/propulsion/PropulsionJet.cpp` to the `liteaerosim` target.
Add `test/PropulsionJet_test.cpp` to the test executable.

---

## ~~3. `PropulsionEDF` — Electric Ducted Fan Model~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — PropulsionEDF`](../architecture/propulsion.md#propulsionedf--electric-ducted-fan-proposed).

Structurally identical to `PropulsionJet` (one `FilterSS2Clip`, same per-step pattern)
with a fixed density exponent of 1.0, no afterburner, a shorter `rotor_tau_s`, and a
`batteryCurrent_a()` diagnostic using actuator disk momentum theory.

### Tests

- **Density lapse:** thrust at half-density equals `thrust_sl * 0.5`.
- **Ram drag:** same verification as PropulsionJet (different `inlet_area_m2` typical values).
- **Idle floor, spool lag, reset():** same patterns as PropulsionJet.
- **Battery current — static:** `batteryCurrent_a(0, rho_sl)` is positive and consistent with `T² / (2 * rho * A_disk)` at idle and full throttle.
- **Battery current — airspeed:** `batteryCurrent_a(tas_mps, rho)` increases with `tas_mps` at fixed thrust (momentum theory).
- **JSON and proto round-trips** (steady-state and mid-transient, `EXPECT_FLOAT_EQ`).
- **Type mismatch throws.**

### CMake

Add `src/propulsion/PropulsionEDF.cpp` to the `liteaerosim` target.
Add `test/PropulsionEDF_test.cpp` to the test executable.

---

## ~~4. `PropellerAero` — Propeller Coefficient Model~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — PropellerAero`](../architecture/propulsion.md#propelleraero--propeller-aerodynamics-model).

Plain value type (no virtual methods, no state). Computes $C_T(J)$ and $C_Q(J)$ from
geometric parameters and returns dimensional thrust and torque.

### Tests

- **Static thrust ($J=0$):** `thrust_n(Omega, 0, rho_sl)` matches $C_{T0} \rho n^2 D^4$.
- **Zero at $J_0$:** `thrustCoeff(J_zero)` is approximately 0.
- **Torque increases with $J$:** `torqueCoeff(J1) > torqueCoeff(J2)` when `J1 > J2 > 0`.
- **Dimensional scaling:** doubling `rho` doubles thrust and torque at the same $\Omega$ and $V$.

### CMake

Add `src/propulsion/PropellerAero.cpp` to the `liteaerosim` target.
Add `test/PropellerAero_test.cpp` to the test executable.

---

## ~~5. `V_Motor`, `MotorElectric`, `MotorPiston`~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — V_Motor`](../architecture/propulsion.md#v_motor--abstract-motor-interface),
[`MotorElectric`](../architecture/propulsion.md#motorelectric--bldc-motor-and-controller),
[`MotorPiston`](../architecture/propulsion.md#motorpiston--normally-aspirated-piston-engine).

`V_Motor` is a pure abstract interface (three pure virtuals). `MotorElectric` and
`MotorPiston` are concrete implementations. All are header-only or near-trivial to
implement (no filter state); tests focus on the formula correctness.

### Tests — `MotorElectric`

- `noLoadOmega_rps(1.0, rho_sl)` returns `kv * supply_voltage_v`.
- `noLoadOmega_rps(0.5, rho_sl)` returns half of full-throttle value.
- `noLoadOmega_rps(1.0, rho_sl / 2)` returns the same value (density-independent).
- `maxOmega_rps()` returns `kv * supply_voltage_v`.
- `batteryCurrent_a()` at no-load ($\Omega \approx \Omega_0$) is near zero.
- `batteryCurrent_a()` at stall ($\Omega = 0$, full throttle) is clamped to `I_max / eta_esc`.

### Tests — `MotorPiston`

- `noLoadOmega_rps(1.0, rho_sl)` returns `2 * peak_omega_rps`.
- `noLoadOmega_rps(0.5, rho_sl)` returns half of the full-throttle value.
- `noLoadOmega_rps(1.0, rho_sl / 2)` returns a lower value than at sea level.
- `maxOmega_rps()` returns `2 * peak_omega_rps`.

### CMake

Add `src/propulsion/MotorElectric.cpp` and `src/propulsion/MotorPiston.cpp` to `liteaerosim`.
Add `test/Motor_test.cpp` (covers both motor types) to the test executable.

---

## ~~6. `PropulsionProp` — Propeller Propulsion Model~~ ✅ Complete

Design authority: [`docs/architecture/propulsion.md — PropulsionProp`](../architecture/propulsion.md#propulsionprop--propeller-propulsion-model).

Owns a `PropellerAero` value member and a `std::unique_ptr<V_Motor>`. One `FilterSS2Clip`
(`_rotor_filter`) models rotor speed dynamics using the caller-supplied `rotor_tau_s`.

### Tests

- **Steady state:** after many steps at constant throttle and airspeed, `omega_rps()` converges to `motor.noLoadOmega_rps(throttle, rho)`.
- **Thrust from Omega:** at the converged `omega_rps()`, `thrust_n()` matches `propeller.thrust_n(omega_rps(), tas_mps, rho)`.
- **Density effect:** lower `rho_kgm3` reduces steady-state thrust.
- **Airspeed effect:** increasing `tas_mps` at fixed throttle and density reduces thrust (advance ratio effect via $C_T(J)$).
- **reset():** `thrust_n()` and `omega_rps()` return 0.
- **JSON and proto round-trips** (both `MotorElectric` and `MotorPiston` variants; steady-state and mid-transient, `EXPECT_FLOAT_EQ`).
- **Type mismatch throws.****

### CMake

Add `src/propulsion/PropulsionProp.cpp` to the `liteaerosim` target.
Add `test/PropulsionProp_test.cpp` to the test executable.

---

## 7. `Aircraft` Class Definition

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

## 8. `Aircraft::step()` — Physics Integration Loop

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
       // Thrust decomposition in Wind frame (see docs/algorithms/equations_of_motion.md)
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

## 9. Serialization

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

## 10. JSON Initialization

The JSON parameter schema is complete (see [`docs/schemas/aircraft_config_v1.md`](../schemas/aircraft_config_v1.md)).
`Aircraft::initialize(config)` must read from a validated config and construct all owned
subcomponents.

### Mapping from schema to Aircraft members

| JSON path | Aircraft member |
|-----------|-----------------|
| `aircraft.mass_kg` | `_mass_kg` |
| `aircraft.S_ref_m2` | `AeroPerformance`, `LoadFactorAllocator` |
| `aircraft.cl_y_beta` | `AeroPerformance`, `LoadFactorAllocator` |
| `lift_curve.*` | `LiftCurveParams` → `LiftCurveModel` |
| `initial_state.*` | `KinematicState` constructor |

`validate_aircraft_config.py` must pass before `initialize()` is called.  `initialize()`
may throw `std::invalid_argument` on malformed input but is not required to duplicate the
full Python-side validation.

### Tests

- `initialize()` with each of the three fixture files (`test/data/aircraft/general_aviation.json`,
  `test/data/aircraft/jet_trainer.json`, `test/data/aircraft/small_uas.json`) succeeds
  without throwing.
- `initialize()` with a missing required field throws `std::exception`.
