# Equations of Motion — Implementation Roadmap

This document tracks the remaining implementation work for the full equations-of-motion subsystem.
Each phase must pass its tests (green) before the next phase starts.

---

## Status Summary

| Phase | Description | Status |
|-------|-------------|--------|
| A | LiftCurveModel — 5-region piecewise lift curve | **Complete** (16 tests) |
| B | KinematicState — store `_q_nw`, complete `step()` | **Complete** (12 tests) |
| C | KinematicState — derived-quantity implementations | **Complete** (included in Phase B tests) |
| D | KinematicState — position integration (WGS84) | Not started |
| E | LoadFactorAllocator — implicit α/β Newton solver | Not started |

---

## Phase A — LiftCurveModel ✓ Complete

**Files:** `include/aerodynamics/LiftCurveModel.hpp`, `src/aerodynamics/LiftCurveModel.cpp`,
`test/LiftCurveModel_test.cpp`

**Interface:**

```cpp
struct LiftCurveParams {
    float cl_alpha;              // pre-stall lift-curve slope (rad⁻¹)
    float cl_max;                // peak CL (positive stall vertex)
    float cl_min;                // minimum CL (negative stall vertex, < 0)
    float delta_alpha_stall;     // angular distance: positive linear join → positive vertex (rad)
    float delta_alpha_stall_neg; // angular distance: negative linear join → negative vertex (rad)
    float cl_sep;                // positive post-stall plateau (< cl_max)
    float cl_sep_neg;            // negative post-stall plateau (> cl_min)
};

class LiftCurveModel {
    float evaluate(float alpha_rad) const;   // C_L(α)
    float derivative(float alpha_rad) const; // dC_L/dα
    float alphaPeak() const;                 // α at cl_max
    float alphaTrough() const;               // α at cl_min
};
```

**5-region model:**

| Region | Condition | C_L(α) |
|--------|-----------|--------|
| 1 — neg flat | α < α_sep_neg | cl_sep_neg |
| 2 — neg quadratic | α_sep_neg ≤ α < α_*_neg | a2n·α²+a1n·α+a0n (upward-opening) |
| 3 — linear | α_*_neg ≤ α ≤ α_* | cl_alpha·α |
| 4 — pos quadratic | α_* < α ≤ α_sep | a2·α²+a1·α+a0 (downward-opening) |
| 5 — pos flat | α > α_sep | cl_sep |

---

## Phase B — KinematicState: Store `_q_nw` ✓ Complete

**Files:** `include/KinematicState.hpp`, `src/KinematicState.cpp`, `test/KinematicState_test.cpp`

### What was done

- Added `Eigen::Quaternionf _q_nw` and `Eigen::Vector3f _wind_NED_mps` as stored members;
  removed `float _windVelNorth` / `float _windVelEast`.
- `q_nw()` is now an inline getter returning `_q_nw` (was an ODR violation — no body existed).
- All three constructors initialize `_q_nw` and `_wind_NED_mps`. Constructor 2 (takes `q_nb`
  directly) defaults `_q_nw` to `Identity()`.
- Completed the three `step()` TODOs: `diff_rot_n` (path-curvature), `roll_delta` (roll around
  Wind X), and `_q_nw = (diff_rot_n * _q_nw * roll_delta).normalized()`.
- Wind is updated at the top of `step()` from the `windSpeed_mps` / `windDirFrom_rad`
  parameters before being used in the velocity integration.
- `crab()` and `crabRate()` updated to use `_wind_NED_mps`.

### Tests (12 passing)

`QnwStoredFromConstructor`, `QnwIsIdentityFromSimpleConstructor`, `QnwChangesAfterRollStep`,
`QnbEqualsQnwTimesQwb` (Phase B), plus all Phase C tests listed below.

---

## Phase C — KinematicState: Derived Quantity Implementations ✓ Complete

**Files:** `src/KinematicState.cpp`, `test/KinematicState_test.cpp`

### What was done

Replaced four stubs and two placeholders:

```cpp
velocity_Wind_mps()      → _q_nw.toRotationMatrix().transpose() * (_velocity_NED_mps - _wind_NED_mps)
velocity_Body_mps()      → _q_nb.toRotationMatrix().transpose() * _velocity_NED_mps
acceleration_Wind_mps()  → _q_nw.toRotationMatrix().transpose() * _acceleration_NED_mps
acceleration_Body_mps()  → _q_nb.toRotationMatrix().transpose() * _acceleration_NED_mps
latitudeRate_rps()       → _positionDatum.latitudeRate(_velocity_NED_mps(0))
longitudeRate_rps()      → _positionDatum.longitudeRate(_velocity_NED_mps(1))
```

Not yet implemented (out of scope for this phase): `alpha()`, `beta()`, `alphaDot()`,
`betaDot()`, `rollRate_Wind_rps()`, `rollRate_rps()`, `pitchRate_rps()`, `headingRate_rps()`,
`velocity_Stab_mps()`, `q_nl()`, `q_ns()`, `POM()`, `turnCircle()`.

### Tests (included in KinematicState_test.cpp, all passing)

`VelocityWindEqualsGroundSpeedZeroWind`, `VelocityWindSubtractsWind`,
`VelocityBodyNonZeroAlpha`, `AccelerationWindRotatesFromNED`, `AccelerationBodyRotatesFromNED`,
`LatitudeRatePositiveForNorthwardVelocity`, `LongitudeRatePositiveForEastwardVelocity`,
`ZeroVelocityZeroPositionRate`.

Note: `latitudeRate_rps()` and `longitudeRate_rps()` are implemented here (Phase C)
rather than Phase D, since they delegate entirely to `WGS84_Datum` and require no
position-integration logic.

---

## Phase D — KinematicState: Position Integration

### WGS84 API (actual, verified against `include/navigation/WGS84.hpp`)

`WGS84_Datum` already provides:

| Method | Purpose |
|--------|---------|
| `latitudeGeodetic_rad()` | getter (not `latitude_rad`) |
| `setLatitudeGeodetic_rad(double)` | setter |
| `longitude_rad()` | getter |
| `setLongitude_rad(double)` | setter |
| `height_WGS84_m()` | getter (not `altitude_m`) |
| `setHeight_WGS84_m(float)` | setter |
| `latitudeRate(double Vnorth)` | returns `dφ/dt` in rad/s |
| `longitudeRate(double Veast)` | returns `dλ/dt` in rad/s |

### Position integration in `step()` (add after velocity update)

```cpp
const float height_prev = _positionDatum.height_WGS84_m();

_positionDatum.setLatitudeGeodetic_rad(
    _positionDatum.latitudeGeodetic_rad() + latitudeRate_rps() * dt);
_positionDatum.setLongitude_rad(
    _positionDatum.longitude_rad() + longitudeRate_rps() * dt);
// NED convention: positive D is down, so altitude increases when V_D < 0
_positionDatum.setHeight_WGS84_m(
    height_prev - 0.5f * (_velocity_NED_mps(2) + prev_vel_D) * dt);
```

Where `prev_vel_D` is `_velocity_NED_mps(2)` saved before the velocity update.

### Tests

```
Test — northward velocity increases latitude
    V = [50, 0, 0] NED, dt = 1 s.
    New latitude > original latitude; magnitude matches latitudeRate * dt.

Test — climbing reduces height (V_D < 0 → altitude increases)
    V = [0, 0, -10] NED (climbing), dt = 1 s.
    height_WGS84_m increases by ≈ 10 m.

Test — zero velocity produces no position change
    V = 0, any dt. Position unchanged.
```

---

## Phase E — LoadFactorAllocator

### New files

- `include/aerodynamics/LoadFactorAllocator.hpp`
- `src/aerodynamics/LoadFactorAllocator.cpp`
- `test/LoadFactorAllocator_test.cpp`

### Interface

```cpp
struct LoadFactorInputs {
    float n;         // commanded normal load factor (g)
    float n_y;       // commanded lateral load factor (g)
    float q_inf;     // dynamic pressure (Pa)
    float thrust_n;  // thrust magnitude (N)
    float mass_kg;   // aircraft mass (kg)
};

struct LoadFactorOutputs {
    float alpha_rad;
    float beta_rad;
    bool  stall;     // true when α demand exceeds C_L ceiling
};

class LoadFactorAllocator {
public:
    explicit LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                 float S_ref_m2,
                                 float cl_y_beta);   // C_Yβ < 0
    LoadFactorOutputs solve(const LoadFactorInputs& in);
    void reset(float alpha0_rad = 0.0f, float beta0_rad = 0.0f);

private:
    const LiftCurveModel& _lift;
    float _S;
    float _cl_y_beta;
    float _alpha_prev;
    float _beta_prev;
    static constexpr int   kMaxIter = 20;
    static constexpr float kTol     = 1e-6f;
};
```

### Implicit equations

**Normal (α):** `f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0`
`f'(α) = q·S·C_L'(α) + T·cos(α)`

**Lateral (β):** `g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0`
`g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)` (always negative → unique root)

### α solver algorithm

1. Predictor: `α₀ = α_prev + δn·m·g / f'(α_prev)`
2. Newton iteration: `α_{k+1} = α_k − f(α_k)/f'(α_k)` (up to `kMaxIter`)
3. Fold guard: if `f'(α) → 0`, set `stall = true`, clamp `α = α_sep`
4. Store `α_prev = α`

### β solver algorithm

1. Predictor: `β₀ = β_prev + δn_y·m·g / g'(β_prev)`
2. Newton: `β_{k+1} = β_k − g(β_k)/g'(β_k)` (uses `T·cos(α)` from solved α)
3. Store `β_prev = β`

### Tests

```
Test — zero demand produces zero angles
    n=0, n_y=0, T=0 → α=0, β=0.

Test — small n, linear region, T=0
    Analytic: α ≈ n·m·g / (q·S·C_Lα) for small α in linear region.

Test — stall flag raised when n exceeds ceiling
    n > (C_L,sep·q·S + T) / (m·g) → stall = true.

Test — monotonically increasing n stays on pre-stall branch
    Step n up from 0 in small increments; verify α_prev tracking avoids jump to post-stall branch.

Test — lateral, T=0
    Analytic: β = n_y·m·g / (q·S·C_Yβ).
```

**Note:** The `LiftCurveModel` interface used in Phase E (`alphaPeak()`, `alphaTrough()`,
`evaluate()`, `derivative()`) matches the completed Phase A implementation.

---

## Build Notes

To build and run tests (DLL PATH must include ucrt64):
```bash
PATH="/c/msys64/ucrt64/bin:/c/Program Files/CMake/bin:$PATH" cmake --build build
PATH="/c/msys64/ucrt64/bin:$PATH" ./build/test/liteaerosim_test.exe
```

New `.cpp` source files are picked up automatically by `GLOB_RECURSE` in
`src/CMakeLists.txt`. New `_test.cpp` files are picked up by `test/CMakeLists.txt`.
No CMake edits required when adding files under `src/` or `test/`.
