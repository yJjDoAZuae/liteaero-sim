# Equations of Motion — Integration Algorithm Design

## 1. Current Approach

`KinematicState::step()` uses a mixed first-order scheme:

- **Velocity** is updated with a trapezoidal (average) acceleration:
  `v_new = v_old + 0.5 * (a_old + a_new) * dt`
- **Altitude** is updated with trapezoidal velocity:
  `alt_new = alt_old + 0.5 * (vD_old + vD_new) * dt`
- **Latitude / longitude** are updated with forward Euler using the new velocity.

Global truncation error: **O(h)**. One derivative evaluation per step. No error estimate.

---

## 2. Algorithm Options for Fixed-Step Integration

The table below covers algorithms suited to a real-time fixed-step simulation loop.
Speed is the primary constraint; embedded error estimation is a secondary nice-to-have.

| Algorithm | Order | Evals / step | Error estimate | Notes |
| ----------- | ------- | :------------: | :--------------: | ------- |
| Forward Euler | 1st | 1 | No | Current; sufficient only at very high rates |
| Midpoint (RK2) | 2nd | 2 | No | Halves global error vs. Euler |
| Heun (RK2 + Euler) | 2nd | 2 | Yes (1st-order embedded) | Low-cost error estimate; rarely worth the complexity |
| **Classic RK4** | **4th** | **4** | **No** | **Recommended** — best accuracy/cost for fixed step |
| Runge–Kutta 3/8 | 4th | 4 | No | RK4 variant with slightly different Butcher tableau; no practical advantage here |
| Dormand–Prince (RK45) | 4th / 5th | 6 | Yes (5th-order embedded) | Standard for adaptive step; +50 % cost vs. RK4 for fixed-step use |
| Adams–Bashforth 4 | 4th | 1 (multistep) | No | Requires 3-step startup; stores history; not recommended |

### Recommendation: Classic RK4

Classic RK4 delivers 4th-order accuracy (global error O(h⁴)) at a fixed cost of four
derivative evaluations per step. It requires no startup buffer, has no hidden state, and
is straightforward to implement and validate. The extra cost compared to Euler or Midpoint
is bounded and predictable, which matters for real-time scheduling.

Dormand–Prince (RK45) is the right choice if adaptive step-size control is ever needed
(e.g., for off-line trajectory generation or non-real-time batch simulation). For the
current fixed-step real-time loop, the extra two evaluations per step are unnecessary
overhead.

---

## 3. Architecture for Multi-Vehicle Support

The key principle: **separate the ODE right-hand side (the vehicle model) from the
integrator (the numerical method)**. The current `KinematicState::step()` fuses both
concerns.

### 3.1 State Representation

All integrable scalars are packed into a flat `KinematicStateVector` (Eigen vector).
Suggested layout:

```text
[ lat_rad, lon_rad, alt_m, vN_mps, vE_mps, vD_mps, qw, qx, qy, qz ]
```

Quaternion components `(qw, qx, qy, qz)` represent the NED-to-wind frame attitude
`q_nw`. Non-integrated quantities (α, β, body rates) are outputs of the vehicle model,
not state variables.

### 3.2 Derivative Structure

```cpp
struct KinematicDerivative {
    double          lat_dot_rps;       // d(lat)/dt
    double          lon_dot_rps;       // d(lon)/dt
    float           alt_dot_mps;       // d(alt)/dt
    Eigen::Vector3f vel_dot_NED_mps2;  // d(vNED)/dt — total acceleration in NED
    Eigen::Quaternionf q_nw_dot;       // quaternion kinematics: 0.5 * q * [0, ω]
};
```

### 3.3 Vehicle Dynamics Interface

```cpp
// Forward declaration — defined per vehicle model
struct VehicleInputs;

class IVehicleDynamics {
public:
    virtual ~IVehicleDynamics() = default;

    // Compute state derivative at a given state and input.
    // dt_s: time since last full step (used by some models for rate limiting).
    virtual KinematicDerivative computeDerivative(
        const KinematicState& state,
        const VehicleInputs&  inputs,
        float                 dt_s) const = 0;
};
```

### 3.4 Generic RK4 Integrator

```cpp
// Apply a single RK4 step.
// f: callable matching IVehicleDynamics::computeDerivative signature.
template<typename DynamicsFn>
KinematicState rk4Step(
    const KinematicState& s0,
    const VehicleInputs&  u,
    float                 dt_s,
    DynamicsFn            f);
```

The implementation evaluates `f` at four points in `[t, t+dt]`, combines the derivatives
with the standard RK4 weights `(1/6, 1/3, 1/3, 1/6)`, and returns the updated state.

### 3.5 Integration into KinematicState

`KinematicState::step()` becomes a thin wrapper:

```cpp
void KinematicState::step(double time_sec,
                          const IVehicleDynamics& dynamics,
                          const VehicleInputs& inputs,
                          float dt_s) {
    *this = rk4Step(*this, inputs, dt_s, dynamics);
    _time_sec = time_sec;
}
```

Backward compatibility with the current scalar-input signature is intentionally not
preserved (see project convention: no backward-compatibility shims).

---

## 4. Vehicle Model Comparison

| Model | Inputs | Integrated states | Angular dynamics |
| ------- | -------- | ------------------- | ----------------- |
| **Trim aero (current)** | n, n_y, α, β, ṙ_wind | position, velocity, q_nw | Algebraic (load-factor constraint; no moment equations) |
| **Trim aero + RK4** | same | same | Same — only integration accuracy improves |
| **Full 6DOF** | forces & moments | position, velocity, q_nb, ω_body | Euler's rigid-body equations; integrates angular rates |

The trim-aero model is appropriate for trajectory planning, guidance law development, and
scenarios where aerodynamic moments can be assumed in equilibrium at each step. Full 6DOF
is required for dynamic maneuver modeling, departure analysis, and autopilot inner-loop
design.

---

## 5. Migration Path

The following steps migrate from the current fused `step()` to the pluggable RK4
architecture while maintaining a working codebase at each stage:

1. **Extract `computeDerivative()`** — move the derivative calculation out of `step()`
   into a free function or private method. `step()` calls it once (equivalent to forward
   Euler at first). All existing tests continue to pass.

2. **Implement `rk4Step()`** — write the generic template function and validate with a
   simple test (e.g., constant-acceleration trajectory with known analytic solution).

3. **Replace `step()` internals** — `step()` delegates to `rk4Step()`, passing the
   extracted derivative function. Update all call sites to use the new `step()` signature.
   Verify tests pass.

4. **Introduce `IVehicleDynamics`** — define the interface and adapter for the existing
   trim-aero model. This decouples the integrator from any specific vehicle model.

5. **Extend with 6DOF** — implement `SixDofDynamics : public IVehicleDynamics` that adds
   moment equations and integrates body angular rates. The integrator code does not change.

---

## 6. References

- Hairer, Nørsett & Wanner, *Solving Ordinary Differential Equations I*, 2nd ed.
- Dormand & Prince, "A family of embedded Runge-Kutta formulae", *J. Comput. Appl. Math.*,
  6(1):19–26, 1980.
- Stevens, Lewis & Johnson, *Aircraft Simulation and Control*, 3rd ed. — coordinate frames
  and quaternion kinematics conventions.
