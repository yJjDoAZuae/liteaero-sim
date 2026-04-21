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

### Alpha Limit Box Constraint

`alpha_max_rad` and `alpha_min_rad` (`AirframePerformance` fields from the `airframe`
config section) are enforced as a box constraint **inside** the Newton iteration, not as
a post-solve clamp.  After each Newton step the iterate is projected:

```cpp
alpha_new = std::clamp(alpha_k - f(alpha_k) / f_prime(alpha_k),
                       airframe.alpha_min_rad, airframe.alpha_max_rad);
```

**Boundary exit condition.** If the projected iterate is pinned at a boundary and the
residual at that boundary still has the sign that would push Newton further outside the
domain, the solver accepts the boundary as the constrained solution and sets an
alpha-limit flag:

- Pinned at `alpha_max_rad` with `f(alpha_max) > 0` → return `alpha_max_rad`, set flag.
- Pinned at `alpha_min_rad` with `f(alpha_min) < 0` → return `alpha_min_rad`, set flag.

The alpha-limit flag is separate from the stall flag.  Both can be set simultaneously
(e.g., if `alpha_max_rad` falls past the fold point and the fold guard also fires).

See [docs/algorithms/equations_of_motion.md §Alpha Limits as a Box Constraint](../algorithms/equations_of_motion.md) for the full mathematical treatment and
interaction with the fold/overshoot guards.

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

## Stall Dynamics Implementation Plan

Three related features to implement, in dependency order.  Each item follows TDD: failing
tests first, then production code.  See
[docs/algorithms/equations_of_motion.md](../algorithms/equations_of_motion.md) for the
full algorithm design.

### Item 1 — Alpha Rate Limiting (Stall Recovery Bridge)

**New state in `LoadFactorAllocator`:**

```cpp
float _alpha_dot_max_rad_s;   // from AirframePerformance; default π/2 (non-binding)
```

**New field in `AirframePerformance`:**

```cpp
float alpha_dot_max_rad_s = kHalfPi;   // rad/s; pass-through default
```

**New proto fields:**

- `AirframePerformanceParams`: field 8 `alpha_dot_max_rad_s`
- `LoadFactorAllocatorState`: field 10 `alpha_dot_max_rad_s`

**Algorithm change in `LoadFactorAllocator::solve()`:**

After the existing Newton solve produces `alpha_eq` (the unconstrained equilibrium), apply
the rate limit before storing `_alpha_prev`:

```cpp
const float max_step = _alpha_dot_max_rad_s * in.dt_s;
const float alpha_out = _alpha_prev
    + std::clamp(alpha_eq - _alpha_prev, -max_step, +max_step);
_alpha_prev = alpha_out;
```

`alpha_out` replaces `alpha_eq` in the output struct and in the realized-Nz computation.
The `dt_s` field must be added to `LoadFactorInputs`.

**Introduce `_cl_recovering` in Item 1:**

`_cl_recovering` (and `_cl_recovering_neg`) are introduced here so that the realized-Nz
formula is written once and never needs replacement when Items 2/3 are added.  While not
stalled, `_cl_recovering` simply tracks the nominal lift curve:

```cpp
_cl_recovering = _lift.evaluate(alpha_out);   // non-stalled: tracks nominal
```

Items 2/3 add the stalled update branch without touching the realized-Nz formula.

**Realized Nz:**

```cpp
const float n_realized = (qS * _cl_recovering + T * std::sin(alpha_out)) / mg;
```

Return `n_realized` and `_cl_recovering` (as `cl_eff`) in `LoadFactorOutputs`.  The
aerodynamic force computation in `Aircraft.cpp` uses `cl_eff` in place of
`_liftCurve->evaluate(alpha_out)` — this is the only change needed in `Aircraft.cpp`
for the stall dynamics to propagate correctly through to `KinematicState`.

**New fields in `LoadFactorOutputs`:**

```cpp
float n_z_realized = 0.f;   // load factor actually delivered (may differ from commanded during bridge)
bool  alpha_bridging = false; // true while the rate limiter is active
```

**Tests to write (in `LoadFactorAllocator_test.cpp`):**

1. `AlphaBridge_Inactive_InNormalPreStall` — small step demand, `alpha_out == alpha_eq`
2. `AlphaBridge_LimitsStepSize` — large discontinuous demand step; verify
   `|alpha_out - alpha_prev| == alpha_dot_max * dt` and `alpha_bridging == true`
3. `AlphaBridge_ConvergesAndDeactivates` — run several steps; bridge clears when
   `alpha_out` reaches `alpha_eq`
4. `AlphaBridge_NRealizedComputedFromAlphaOut` — confirm `n_z_realized` is computed from
   `alpha_out`, not commanded `n`
5. `AlphaBridge_DefaultPassthrough` — default `alpha_dot_max = π/2` with `dt = 0.01s`;
   verify bridge never activates for typical pre-stall steps
6. `AirframePerformance` serialization round-trip tests extended to cover
   `alpha_dot_max_rad_s` (JSON and proto)

**Config change (`airframe` section):**

```json
"alpha_dot_max_rad_s": 0.52
```

Update all fixture files and inline test configs.

---

### Item 2 — Stall Hysteresis Flags

**New state in `LoadFactorAllocator`:**

```cpp
bool  _stalled;       // positive-side hysteresis flag
bool  _stalled_neg;   // negative-side hysteresis flag
```

Both initialized to `false` in constructor and `reset()`.

**Algorithm change in `solve()`:**

After `alpha_out` is determined (post rate-limit), update flags:

```cpp
// Entry: alpha has passed alphaPeak and is now decreasing
if (alpha_out < _alpha_prev && _alpha_prev >= _lift.alphaPeak()) _stalled     = true;
if (alpha_out > _alpha_prev && _alpha_prev <= _lift.alphaTrough()) _stalled_neg = true;
// Exit: threshold (alphaStar) or snap-down (nominal meets plateau in ascending quadratic)
if (alpha_out <= _lift.alphaStar() ||
    (alpha_out < _lift.alphaPeak() && _lift.evaluate(alpha_out) <= _lift.clSep()))
    _stalled = false;
if (alpha_out >= _lift.alphaStarNeg() ||
    (alpha_out > _lift.alphaTrough() && _lift.evaluate(alpha_out) >= _lift.clSepNeg()))
    _stalled_neg = false;
```

Note: `_alpha_prev` here is the value from the *previous* step (before updating it to
`alpha_out`), so it reflects the alpha from which the current step descended.

**New fields in `LoadFactorOutputs`:**

```cpp
bool stalled     = false;   // positive-side hysteresis active
bool stalled_neg = false;   // negative-side hysteresis active
```

**Serialization:** add `_stalled` and `_stalled_neg` to JSON and proto serialization of
`LoadFactorAllocator` state.

**Tests to write:**

1. `Hysteresis_FlagSetsWhenDecreasingFromAlphaPeak` — push alpha to `alphaPeak`, then
   step it down; verify `stalled` sets on the first decreasing step
2. `Hysteresis_FlagDoesNotSetOnAscent` — push alpha past `alphaPeak` on the way up;
   verify `stalled` is false while ascending
3. `Hysteresis_FlagClearsAtAlphaStar` — configuration with `clSep() < clAlpha() * alphaStar()`; reduce alpha through stall region; flag remains set until alpha crosses `alphaStar()`, clears immediately after
4. `Hysteresis_FlagClearsAtSnapDown` — configuration with `clSep() > clAlpha() * alphaStar()`; flag clears when `_lift.evaluate(alpha_out)` first drops to `clSep()` in the ascending-quadratic domain, above `alphaStar()`
5. `Hysteresis_CLFlatThroughDescentRegion` — verify `cl_eff == C_L_sep` throughout
   the descent from `alphaPeak` to `alphaStar`, not the nominal descending parabola
6. `Hysteresis_AlphaBounceDoesNotClearFlag` — alpha descends partway, then increases
   again without reaching `alphaStar()`; flag stays set
7. `Hysteresis_NoFlagWithoutStallEntry` — normal pre-stall operation; flag never sets
8. `Hysteresis_NegativeSideSymmetric` — negative-side entry and exit
9. `Hysteresis_SerializationRoundTrip` — JSON and proto round-trip with flag set

---

### Item 3 — CL Recovery Rate Limiting

**New state in `LoadFactorAllocator`:**

```cpp
float _cl_recovering;      // current effective CL during post-stall recovery (positive side)
float _cl_recovering_neg;  // current effective CL (negative side)
```

Initialized in constructor and `reset()` to `0.0f`.  Under OQ-10-B, every non-stalled
`solve()` call overwrites `_cl_recovering` before computing `n_realized`; the initial
value is never used without being overwritten first.

**Three-phase CL behavior:**

| Phase | Condition | Effective CL |
| --- | --- | --- |
| Stalled descent | `_stalled == true` (alpha ≥ `alphaStar()`) | `min(clSep(), C_L_nom(alpha))` — snaps to nominal if nominal falls below `C_L_sep` |
| Recovery | `_stalled == false`, `_cl_recovering < C_L_nom(alpha)` | Rate-limited rise toward nominal at `cl_dot_max` |
| Normal | `_stalled == false`, `_cl_recovering == C_L_nom(alpha)` | Tracks nominal lift curve |

If alpha rises back into the descending quadratic while `_stalled == false` (recovery in
progress), `C_L_nom(alpha)` falls along the nominal parabola.  Since `_cl_recovering` is
clamped to `min(C_L_nom, _cl_recovering + cl_dot_max * dt)`, CL descends immediately with
the nominal curve — no rate limit on downward movement.  This is the correct behavior:
CL cannot be sustained above the nominal model.

**Algorithm change in `solve()`:**

After updating hysteresis flags (Item 2), update `_cl_recovering`:

```cpp
const float cl_dot_max = _lift.clAlpha() * _alpha_dot_max_rad_s;
const float cl_nom = _lift.evaluate(alpha_out);

// Positive side
if (_stalled) {
    // min(): hold plateau, but snap instantly to nominal if nominal falls below clSep
    _cl_recovering = std::min(_lift.clSep(), cl_nom);
} else {
    // Rate-limit upward approach to nominal; instant downward snap
    _cl_recovering = std::min(cl_nom, _cl_recovering + cl_dot_max * in.dt_s);
}

// Negative side — mirrors positive with inverted direction (max instead of min)
if (_stalled_neg) {
    // max(): hold plateau, but snap instantly to nominal if nominal rises above clSepNeg
    _cl_recovering_neg = std::max(_lift.clSepNeg(), cl_nom);
} else {
    // Rate-limit downward approach to nominal; instant upward snap
    _cl_recovering_neg = std::max(cl_nom, _cl_recovering_neg - cl_dot_max * in.dt_s);
}
```

When `_stalled` is true, the Newton loop is replaced by the explicit solve:

```cpp
// alpha_eq from explicit fully-separated form with _cl_recovering as the plateau
if (T > kTol) {
    const float sin_alpha_eq = (in.n_z * mg - qS * _cl_recovering) / T;
    alpha_eq = (std::abs(sin_alpha_eq) <= 1.0f) ? std::asin(sin_alpha_eq) : _alpha_prev;
} else {
    alpha_eq = _alpha_prev;   // T=0: no alpha solution; hold current
}
// Alpha bridge then steps toward alpha_eq at the rate limit (same as non-stalled path)
```

The realized Nz uses `_cl_recovering`:

```cpp
const float cl_eff = _cl_recovering;   // always valid: tracks nominal when not stalled
const float n_realized = (qS * cl_eff + T * std::sin(alpha_out)) / mg;
```

Note: `_cl_recovering` tracks nominal when `_stalled == false` and the bridge has
converged, so using it unconditionally is correct in all phases.

**alphaDot during stall (OQ-15-A):**

While `_stalled` or `_stalled_neg`, the Newton loop is bypassed so `positive_stall` and
`negative_stall` remain `false`.  The correct IFT derivative for the explicit solve (flat
CL, `dC_L/dα = 0`) is `T·cos(α)` alone.  Replace the existing `fprime_alpha` computation:

```cpp
const float fprime_alpha = _stalled     ?  T * std::cos(alpha_out)
                         : _stalled_neg ? -T * std::cos(alpha_out)
                         : qS * _lift.derivative(alpha_out) + T * std::cos(alpha_out);
float alphaDot = 0.f;
if (!positive_stall && !negative_stall && std::abs(fprime_alpha) > kTol) {
    alphaDot = mg * in.n_z_dot / fprime_alpha;
}
```

The existing Newton fold-guard (`positive_stall` / `negative_stall`) remains and fires only
in the non-stalled path.

**`LiftCurveModel` accessors needed:**

```cpp
float clAlpha()  const;  // returns the pre-stall lift-curve slope C_Lα
float clSep()    const;  // returns C_L_sep — positive-side post-stall plateau CL
float clSepNeg() const;  // returns C_L_sep_neg — negative-side post-stall plateau CL
```

Add any not already present.  `clSep()` and `clSepNeg()` are included in Implementation
Order Step 1.

**New fields in `LoadFactorOutputs`:**

```cpp
float cl_eff = 0.f;          // effective CL used (recovering or nominal)
bool  cl_recovering = false;  // true while CL recovery bridge is active (_cl_recovering < C_L_nom(alpha_out))
```

**Serialization:** add `_cl_recovering` and `_cl_recovering_neg` to JSON and proto
serialization.

**Tests to write:**

1. `CLRecovery_FlatThroughEntireDescentToAlphaStar` — enter stall at `alphaPeak`, reduce
   alpha to `alphaStar()`; verify `cl_eff == C_L_sep` throughout.  Valid only when
   `C_L_sep < C_L_star`; use a lift curve where `clSep() < clAlpha() * alphaStar()`.
2. `CLRecovery_SnapToNominalBeforeAlphaStar` — use a lift curve where
   `C_L_sep > C_L_star`; enter stall; verify `cl_eff` tracks `min(clSep(), cl_nom)` and
   snaps to the nominal descending quadratic at the intersection alpha (above `alphaStar()`)
   with no CL discontinuity.
3. `CLRecovery_RateLimitedBelowAlphaStar` — alpha crosses `alphaStar()`; verify CL climbs
   toward nominal at exactly `cl_alpha * alpha_dot_max * dt` per step
4. `CLRecovery_InstantDownwardFollow` — during recovery (below `alphaStar()`), raise alpha
   back into the descending quadratic; verify `cl_eff` follows nominal parabola downward
   immediately without rate limiting
5. `CLRecovery_BridgeDeactivatesAtNominal` — run recovery to completion; verify
   `cl_recovering == false` and `cl_eff == nominal`
6. `CLRecovery_NzFromEffectiveCL` — confirm realized Nz uses `cl_eff` throughout all phases
7. `CLRecovery_TZeroHoldsAlpha` — T=0 while stalled; verify `alpha_out` held at `_alpha_prev`
8. Serialization round-trip for `_cl_recovering` / `_cl_recovering_neg`

---

### Design Decisions

The following questions were raised during planning and resolved as documented.

| # | Question | Decision |
| --- | --- | --- |
| OQ-1 | Alpha bridge activation scope (unconditional vs. stall-conditional) | **Unconditional.** Apply the rate limit to all alpha steps. Physics are always correct; with a generous `alpha_dot_max_rad_s` default (π/2 rad/s) the bridge is transparent in normal pre-stall operation. |
| OQ-2 | Newton residual while `_stalled` is true (nominal CL vs. recovering CL) | **Explicit solve using recovering CL.** While `_stalled`, Newton is replaced by `α_eq = arcsin((n·mg − qS·_cl_recovering) / T)`. The alpha bridge is the only mechanism moving alpha; it drives toward this target. Realized Nz uses `_cl_recovering`. |
| OQ-3 | `dt_s` delivery to `solve()` | **Add `dt_s` to `LoadFactorInputs`.** Explicit per-call field; supports variable-rate stepping; no constructor or serialization changes to `LoadFactorAllocator`. |
| OQ-4 | `LiftCurveModel` accessors needed | **Add `alphaStar()`, `alphaStarNeg()`, `clAlpha()` as public accessors.** Consistent with the existing `alphaPeak()`, `alphaSep()` interface; no design trade-off. |
| OQ-5 | Hysteresis flag serialization | **Serialize `_stalled` and `_stalled_neg`.** Recomputing from `_alpha_prev` fails in the hysteretic region where `alphaStar < alpha_prev < alphaSep` but the flag should still be set. |
| OQ-6 | CL recovery state serialization | **Serialize `_cl_recovering` and `_cl_recovering_neg`.** Recomputing from stall flags loses the in-progress recovery position; two additional floats in the serialization block is negligible cost. |

The OQ-2 decision in the table above has been revised (was two-pass; corrected to explicit
solve using recovering CL).  OQ-7 through OQ-9 were raised and resolved as follows and
incorporated into the Design Decisions table:

| # | Question | Decision |
| --- | --- | --- |
| OQ-7 | Newton residual while stalled — contradiction with OQ-2 | **B: explicit solve using recovering CL** (see OQ-2 row above). OQ-2 revised accordingly. Algorithm doc §"Interaction with the Newton Solver" is correct as written. |
| OQ-8 | Hysteresis entry condition (`alphaSep()` vs. `alphaPeak()`) | **Entry when alpha has passed `alphaPeak()` and begins to reduce**, not at `alphaSep()`. |
| OQ-9 | Re-entry behavior when alpha increases while `_stalled` is true | **Effective CL is `min(clSep(), clNom(alpha))` throughout the stalled phase** (see OQ-16). If the nominal descends below C_L_sep before alphaStar(), CL snaps to nominal at that crossing with no discontinuity. Once below alphaStar(), the CL recovery bridge activates. If alpha rises back into the transition and CL overshoots nominal, it snaps instantly to nominal. If alpha rises past alphaPeak() and decreases again, `_stalled` re-enters. |
| OQ-10 | When to introduce `_cl_recovering` | **B: Introduce in Item 1.** The realized-Nz formula is written once and never replaced; Items 2/3 add only the stalled update branch. |
| OQ-11 | `_cl_recovering` initialization value | **Initialize to zero.** Under OQ-10-B, every non-stalled `solve()` call sets `_cl_recovering = _lift.evaluate(alpha_out)` before computing `n_realized`; the initial value is overwritten before first use. |
| OQ-12 | Hysteresis entry check — one-step delay | **A: Accept one-step delay.** At 100 Hz the error is one step of the nominal parabola; negligible and self-correcting. |
| OQ-13 | `clSep()` / `clSepNeg()` vs. `params()` accessor | **A: Add `clSep()` and `clSepNeg()` as individual accessors.** Consistent with `alphaPeak()`, `alphaSep()` pattern. Added to Implementation Order Step 1. |
| OQ-14 | `cl_eff` delivery to `Aircraft.cpp` | **A: Add `cl_eff` to `LoadFactorOutputs`.** `Aircraft.cpp` replaces `_liftCurve->evaluate(lfa_out.alpha_rad)` at line 203 with `lfa_out.cl_eff`. |
| OQ-15 | `alphaDot` computation during stall | **A: While `_stalled`, substitute `fprime_alpha = T * cos(alpha_out)`.** Exact IFT derivative for the flat-CL explicit solve; zero if T ≈ 0. Same substitution applies for `_stalled_neg`. |
| OQ-16 | Stalled and negative-side CL formulas | **Stalled branch: `_cl_recovering = min(clSep(), clNom)`.** Allows instant snap to nominal when nominal descends below C_L_sep — no discontinuity at the intersection, recovery can begin above alphaStar(). Non-stalled recovery: `min(clNom, _cl_recovering + clDotMax * dt)`. Negative side mirrors with inverted sign throughout: stalled `max(clSepNeg(), clNom)`; recovery `max(clNom, _cl_recovering_neg - clDotMax * dt)`. |

### Open Questions

All questions from OQ-10 through OQ-16 have been resolved; see Design Decisions above.  No open questions remain before implementation.

---

### Implementation Order

| Step | Action |
| --- | --- |
| 1 | Add `alphaStar()`, `alphaStarNeg()`, `alphaTrough()`, `clAlpha()`, `clSep()`, `clSepNeg()` accessors to `LiftCurveModel` (with tests) |
| 2 | Add `alpha_dot_max_rad_s` to `AirframePerformance` (proto, JSON, round-trip tests) |
| 3 | Update all fixture files and inline configs for new `alpha_dot_max_rad_s` field |
| 4 | Add `dt_s` to `LoadFactorInputs` |
| 5 | Implement Item 1 (alpha rate bridge) — tests first |
| 6 | Implement Item 2 (hysteresis flags) — tests first |
| 7 | Implement Item 3 (CL recovery) — tests first |
| 8 | Update `Aircraft.cpp` to pass `alpha_dot_max_rad_s` and `dt_s` through |

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
