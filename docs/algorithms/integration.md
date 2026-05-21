# Equations of Motion — Integration Algorithm Design

## 1. Current Approach

`KinematicState::step()` uses a mixed first-order scheme:

- **Velocity** is updated with a trapezoidal (average) acceleration:

$$\mathbf{v}_{k+1} = \mathbf{v}_k + \tfrac{\Delta t}{2}\left(\mathbf{a}_k + \mathbf{a}_{k+1}\right)$$

- **Altitude** is updated with trapezoidal NED-down velocity:

$$h_{k+1} = h_k + \tfrac{\Delta t}{2}\left(v_{D,k} + v_{D,k+1}\right)$$

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

The derivative bundle groups the six geodetic rate components and the quaternion
kinematic rate into a single value-type struct:

$$
\dot{\mathbf{x}} = \bigl(\dot\varphi,\; \dot\lambda,\; \dot h,\; \dot v_N,\; \dot v_E,\; \dot v_D,\; \dot{q}_{nw}\bigr)
$$

where $\dot{q}_{nw} = \tfrac{1}{2}\, q_{nw} \otimes [0,\; \boldsymbol{\omega}_{W/N}^W]^T$ is the quaternion kinematic equation.

### 3.3 Vehicle Dynamics Interface

The vehicle dynamics interface maps a state and input tuple to the state derivative:

$$
f: (\mathbf{x},\; \mathbf{u},\; \Delta t) \mapsto \dot{\mathbf{x}}
$$

where $\mathbf{x}$ is the kinematic state, $\mathbf{u}$ is the vehicle input set, and $\Delta t$ is the elapsed time used by models with rate-limited internal state. Different vehicle models (trim aero, 6DOF) implement the same mapping — the integrator is vehicle-agnostic.

### 3.4 Generic RK4 Integrator

A single RK4 step advances state $\mathbf{x}_k$ to $\mathbf{x}_{k+1}$ by evaluating $f$ at four points within $[t,\; t + \Delta t]$:

$$
k_1 = f(\mathbf{x}_k,\; \mathbf{u},\; \Delta t)
$$

$$
k_2 = f\!\left(\mathbf{x}_k + \tfrac{\Delta t}{2}\,k_1,\; \mathbf{u},\; \Delta t\right)
$$

$$
k_3 = f\!\left(\mathbf{x}_k + \tfrac{\Delta t}{2}\,k_2,\; \mathbf{u},\; \Delta t\right)
$$

$$
k_4 = f\!\left(\mathbf{x}_k + \Delta t\,k_3,\; \mathbf{u},\; \Delta t\right)
$$

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}\bigl(k_1 + 2k_2 + 2k_3 + k_4\bigr)
$$

The weights $(1/6, 1/3, 1/3, 1/6)$ are the standard Butcher tableau coefficients for the classic RK4 method.

### 3.5 Integration into KinematicState

The kinematic state step function becomes a thin delegation to the RK4 integrator,
passing the vehicle dynamics callable as $f$. The integrator evaluates $f$ four times per
step and returns the updated state. No persistent buffer, startup history, or
step-size adaptation is required.

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

## 6. References

- Hairer, Nørsett & Wanner, *Solving Ordinary Differential Equations I*, 2nd ed.
- Dormand & Prince, "A family of embedded Runge-Kutta formulae", *J. Comput. Appl. Math.*,
  6(1):19–26, 1980.
- Stevens, Lewis & Johnson, *Aircraft Simulation and Control*, 3rd ed. — coordinate frames
  and quaternion kinematics conventions.
