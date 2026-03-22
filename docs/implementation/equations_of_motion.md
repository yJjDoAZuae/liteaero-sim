# Equations of Motion — Implementation Notes

Implementation decisions for the EOM subsystem. For algorithm design and math, see
[docs/algorithms/equations_of_motion.md](../algorithms/equations_of_motion.md).

---

## Files

| File | Role |
| --- | --- |
| `include/KinematicState.hpp` | Kinematic state interface |
| `src/KinematicState.cpp` | KinematicState implementation |
| `include/aerodynamics/LiftCurveModel.hpp` | 5-region lift curve interface |
| `src/aerodynamics/LiftCurveModel.cpp` | LiftCurveModel implementation |
| `include/aerodynamics/LoadFactorAllocator.hpp` | α/β Newton solver interface |
| `src/aerodynamics/LoadFactorAllocator.cpp` | LoadFactorAllocator implementation |

---

## KinematicState

### Wind Frame Quaternion (`_q_nw`)

`_q_nw` (Wind-to-NED rotation) is stored as `Eigen::Quaternionf`.  It is propagated each
`step()` by composing two rotations applied in order:

1. **Path curvature** (`diff_rot_n`): differential rotation in NED from the velocity
   direction change, computed via `setFromTwoVectors(v_prev, v_new)`.
2. **Roll** (`roll_delta`): rotation about Wind X by `rollRate_Wind_rps * dt`.

```cpp
_q_nw = (diff_rot_n * _q_nw * roll_delta).normalized();
```

Constructor 1 accepts an initial `q_nw` and stores it directly.  Constructor 2 (which
takes `q_nb` directly) derives both α and β exactly from the body-frame velocity
projection, then computes `_q_nw`:

```text
// body-frame velocity components (u forward, v right, w down)
v_body = C_BN * v_NED

// exact inversion of  u = V cosα cosβ,  v = V cosα sinβ,  w = V sinα
alpha = atan2(w, sqrt(u^2 + v^2))
beta  = atan2(v, u)

// q_wb = Ry(alpha) * Rz(-beta)  →  q_wb^{-1} = Rz(beta) * Ry(-alpha)
_q_nw = (q_nb * Rz(beta) * Ry(-alpha)).normalized()
```

Below the small-V threshold (0.1 m/s), α and β default to 0 and `_q_nw = q_nb`.

### Wind Representation

`step()` accepts wind as a full 3D NED vector `wind_NED_mps` (`Eigen::Vector3f`). The
2D-to-3D conversion (from meteorological speed and direction-from) happens in the
environment or scenario layer before the call to `step()`. A north wind blowing at
$V_w$ m/s corresponds to `wind_NED_mps = {-Vw, 0, 0}`.

### Position and Velocity Integration (RK4)

Position and velocity are jointly integrated with classical fourth-order Runge-Kutta
(RK4) over the six-component state `PVState{lat_rad, lon_rad, alt_m, vN_mps, vE_mps,
vD_mps}`, defined in an anonymous namespace in `src/KinematicState.cpp`.

The four RK4 derivative evaluations call `pvDerivative()`, which queries
`WGS84_Datum::latitudeRate()` and `WGS84_Datum::longitudeRate()` at intermediate
position/velocity values. Because the input acceleration `a_NED` is constant during each
step, all four velocity slope values are identical and the velocity result reduces to
`v_new = v_old + a_NED * dt`. The position result achieves fourth-order accuracy in the
nonlinear WGS84 geodetic rate functions.

#### WGS84_Datum API

| Method | Purpose |
| --- | --- |
| `latitudeGeodetic_rad()` / `setLatitudeGeodetic_rad(double)` | Geodetic latitude |
| `longitude_rad()` / `setLongitude_rad(double)` | Longitude |
| `height_WGS84_m()` / `setHeight_WGS84_m(float)` | WGS84 ellipsoidal altitude |
| `latitudeRate(double Vnorth)` | Returns dφ/dt (rad/s) |
| `longitudeRate(double Veast)` | Returns dλ/dt (rad/s) |

### Derived Quantities

All implemented as rotation-matrix products on stored state:

```text
velocity_Wind_mps()      = C_WN * (v_NED − wind_NED)
velocity_Body_mps()      = C_BN * v_NED
acceleration_Wind_mps()  = C_WN * a_NED
acceleration_Body_mps()  = C_BN * a_NED
latitudeRate_rps()       = _positionDatum.latitudeRate(v_NED[0])
longitudeRate_rps()      = _positionDatum.longitudeRate(v_NED[1])
```

### `step()` Parameters

`KinematicState::step()` is the integration sink. Each parameter is produced by a
specific upstream subsystem and passed in each simulation step:

| Parameter | Source | Meaning |
| --- | --- | --- |
| `time_sec` | Simulation clock | Absolute simulation time |
| `acceleration_Wind_mps` | Aerodynamic / propulsion model | Net Wind-frame acceleration (lift + drag + thrust + gravity expressed in Wind frame) |
| `rollRate_Wind_rps` | Roll-control model | Wind-axis roll rate $p_W$ — drives `_q_nw` propagation |
| `alpha_rad` | Aerodynamic model | Angle of attack — used to propagate `_q_nb` and stored in `_alpha_rad` |
| `beta_rad` | Aerodynamic model | Sideslip angle — same uses as above |
| `alphaDot_rps` | Aerodynamic model | Rate of change of angle of attack — stored in `_alphaDot_rps` |
| `betaDot_rps` | Aerodynamic model | Rate of change of sideslip — stored in `_betaDot_rps` |
| `wind_NED_mps` | Environment model | Ambient 3D wind velocity in NED |

The integrator does not call back into the aerodynamic model within a step. Coupling
is one-directional: aero/propulsion outputs → kinematic inputs. The aerodynamic model
reads `alpha()`, `beta()`, and `velocity_Wind_mps()` from the kinematic state at the
start of the next step.

### `q_nl()` Semantics

`q_nl()` returns `Quaternionf(_positionDatum.qne().cast<float>())`, where `qne()` is
the ECEF-to-NED rotation at the current position. This is used as the NED-to-Local-Level
rotation.

**Open question:** For long-range flights, the Local Level frame (tangent plane at the
current aircraft position) diverges from the NED frame (fixed at the initial datum). The
current implementation returns the ECEF-to-NED rotation at the current position, which
is the rotation of the tangent plane, not a fixed NED frame. Confirm whether this is the
intended behavior before depending on `q_nl()` for inertial navigation calculations.

---

## LoadFactorAllocator

### Newton Solver Guards

Two guards prevent divergence when the demanded load factor exceeds the achievable ceiling.
The ceiling is not simply `alphaPeak()` when thrust is positive: the normal thrust component
$T\sin\alpha$ continues to grow past $\alpha_{peak}$, so the load-factor ceiling
$N_{z,max}(\alpha) = (qS C_L(\alpha) + T\sin\alpha)/(mg)$ peaks at α* where
$f'(\alpha^*) = qS C_L'(\alpha^*) + T\cos\alpha^* = 0$, which lies above `alphaPeak()`
whenever $T > 0$.

**Overshoot guard** (inside the Newton loop): before evaluating $f'$ at the proposed step
`alpha_new`, clamp it to the CL parabolic domain using `alphaSep()` / `alphaSepNeg()`.
In the flat separated plateau $f'(\alpha) = T\cos\alpha$, which stays positive until
$\alpha > \pi/2$, so without this clamp large thrust would let Newton escape the physical
domain entirely.  After clamping, if `f'(alpha_hi) <= 0`, the f′-zero crossing lies between
the current iterate and the clamped point: bisect the interval (30 iterations, ~30 bits of
precision) to pin it.  Set `stall = true` and break.  For $T = 0$ the crossing coincides
exactly with `alphaPeak()` / `alphaTrough()`, preserving the zero-thrust behavior.

**Fold guard**: if `|f'(α)| < kTol` at the current iterate, the iterate is already at the
f′-zero crossing.  Stay at the current α (do not snap to `alphaPeak()`) and set
`stall = true`.  For $T = 0$ this fires at `alphaPeak()` as before; for $T > 0$ it fires
at the correct crossing above `alphaPeak()`.

The β solver's derivative `g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)` is always ≤ 0 for
C_Yβ < 0, guaranteeing a unique root and no fold condition.

### Warm-Starting and Branch-Continuation Predictor

`_alpha_prev`, `_beta_prev`, `_n_z_prev`, and `_n_y_prev` persist between `solve()`
calls.  Before the Newton loop, a first-order branch-continuation predictor computes:

```text
α₀ = α_prev + δn_z · m·g / f′(α_prev)
β₀ = β_prev + δn_y · m·g / g′(β_prev)
```

In the linear lift region `f′(α)` is constant (`q·S·C_Lα`), so the predictor places
Newton exactly at the solution and the solver converges in a single iteration.  The
predictor is guarded by two conditions:

- **f′ guard**: skip if `|f′(α_prev)| < kTol` (at the stall ceiling, no meaningful step
  direction).
- **Domain guard**: the raw prediction is only applied when it stays within
  `[alphaSepNeg, alphaSep]`.  If the demand jump would project the warm-start into
  the flat separated plateau (e.g., a cold-start excess-demand call), the predictor
  falls back to `α_prev` so that the overshoot guards operate correctly.

`reset()` sets `_alpha_prev`, `_beta_prev`, `_n_z_prev`, and `_n_y_prev` all to zero.
Call `reset()` before a discontinuous change in demand.  All four fields are included
in both JSON and proto serialization.

### Stall Warm-Start Limitation

When the fold guard or overshoot guard clamps α at the stall ceiling, `_alpha_prev` is left
at the clamped value where $f'(\alpha) = 0$ by definition.  On the next `solve()` call:

- The predictor f′ guard fires (`|f′(α_prev)| < kTol`), so the warm-start falls back to
  α_prev (the clamped value).
- The Newton loop starts at the clamped α.  In floating-point arithmetic `f′(alphaPeak)` for
  $T = 0$ evaluates to a small nonzero residual (~3 × 10⁻⁶ for typical parameters) that
  exceeds `kTol = 1e-6`.  The fold guard therefore does not fire at the first Newton
  iteration.  Newton takes a large-magnitude step and the overshoot guard may pin α on the
  opposite-sign branch — converging to the wrong solution.

**Consequence**: a single discontinuous Nz jump past the stall ceiling (e.g., a direct step
from 1 g to 1.5 × $N_{z,\text{max}}$) leaves the warm-start at the fold point.  The
subsequent sub-ceiling `solve()` call is likely to converge to the wrong branch.

**Required mitigation**: call `reset()` before any discontinuous change in demand, including
re-engagement after a stall event.  In normal closed-loop simulation with small $dt$ the Nz
command evolves continuously within one step, so the warm-start stays near the true solution
and this limitation does not arise in practice.

See `AllocatorFixture.StallRecovery_RequiresReset` in `test/LoadFactorAllocator_test.cpp`.

---

## Test Coverage

| Suite | Tests | File |
| --- | --- | --- |
| KinematicState | 52 | `test/KinematicState_test.cpp` |
| LiftCurveModel | 16 | `test/LiftCurveModel_test.cpp` |
| LoadFactorAllocator | 30 | `test/LoadFactorAllocator_test.cpp` |

---

## Build

New `.cpp` files under `src/` and new `*_test.cpp` files under `test/` are picked up
automatically by `GLOB_RECURSE`; no CMake edits are needed when adding files in those
directories.

On Windows with the ucrt64 toolchain, the ucrt64 DLLs must be in PATH:

```bash
PATH="/c/msys64/ucrt64/bin:/c/Program Files/CMake/bin:$PATH" cmake --build build
PATH="/c/msys64/ucrt64/bin:$PATH" ./build/test/liteaerosim_test.exe
```
