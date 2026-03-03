# KinematicState Task 2 — Implementation Plan

Expands the high-level description in
[docs/roadmap/equations_of_motion.md §2](../roadmap/equations_of_motion.md#2-unimplemented-kinematicstate-stubs)
into a phase-by-phase TDD plan.  Each phase follows the project rule: **write a
failing test first, then write the minimal production code to make it pass.**

---

## Stubs to Implement

| Method | Return type | Derivation source |
|---|---|---|
| `alpha()` | `float` | Stored from `alpha_rad` parameter supplied by the aerodynamic model to `step()` |
| `beta()` | `float` | Stored from `beta_rad` parameter supplied by the aerodynamic model to `step()` |
| `alphaDot()` | `float` | Stored from `alphaDot` parameter supplied by the aerodynamic model to `step()` |
| `betaDot()` | `float` | Stored from `betaDot` parameter supplied by the aerodynamic model to `step()` |
| `rollRate_Wind_rps()` | `float` | Stored from `rollRate_Wind_rps` parameter supplied by the roll-control model to `step()` |
| `rollRate_rps()` | `float` | `BodyRatesToEulerRates(eulers(), _rates_Body_rps)(0)` |
| `pitchRate_rps()` | `float` | `BodyRatesToEulerRates(eulers(), _rates_Body_rps)(1)` |
| `headingRate_rps()` | `float` | `BodyRatesToEulerRates(eulers(), _rates_Body_rps)(2)` |
| `q_ns()` | `Eigen::Quaternionf` | `_q_nw * Ry(alpha())` |
| `velocity_Stab_mps()` | `Eigen::Vector3f` | `q_ns().toRotationMatrix().transpose() * _velocity_NED_mps` |
| `q_nl()` | `Eigen::Quaternionf` | `_positionDatum.qne()` cast to float (see §Phase K) |
| `POM()` | `const PlaneOfMotion&` | Frenet frame of airmass velocity path (see §Phase L) |
| `turnCircle()` | `const TurnCircle&` | Curvature center from POM (see §Phase L) |

---

## Storage Changes Required

### New member variables

Add to `KinematicState` (protected section of `include/KinematicState.hpp`):

```cpp
float _alpha_rad;           // angle of attack (rad)    — stored from step() parameter
float _beta_rad;            // sideslip angle (rad)     — stored from step() parameter
float _alphaDot_rps;        // angle-of-attack rate (rad/s) — stored from step() parameter
float _betaDot_rps;         // sideslip rate (rad/s)        — stored from step() parameter
float _rollRate_Wind_rps;   // Wind-axis roll rate p_W (rad/s) — stored from step() parameter

mutable PlaneOfMotion _pom;          // cached; recomputed by POM()
mutable TurnCircle    _turn_circle;  // cached; recomputed by turnCircle()
```

### Constructor updates

All constructors must initialize all five stored fields to `0.f`.
Constructor 2 receives `alpha`, `beta`, `alphaDot`, `betaDot`, and `rollRate_Wind_rps`
as parameters — store them directly.

### `step()` signature — no change

`alpha_rad`, `beta_rad`, `alphaDot`, `betaDot`, and `rollRate_Wind_rps` remain as
explicit `step()` parameters supplied by the aerodynamic and roll-control models.
`step()` stores all five:

```cpp
_alpha_rad          = alpha_rad;
_beta_rad           = beta_rad;
_alphaDot_rps       = alphaDot;
_betaDot_rps        = betaDot;
_rollRate_Wind_rps  = rollRate_Wind_rps;
```

---

## API Change: `POM()` and `turnCircle()` Return Type

The header currently declares:
```cpp
PlaneOfMotion &POM() const;
TurnCircle    &turnCircle() const;
```

A `const` method cannot return a non-`const` reference to a member without
`mutable`.  Since both methods compute ephemeral derived quantities and there
are no existing call sites (these are unimplemented stubs), change both return
types to `const` reference:

```cpp
const PlaneOfMotion &POM() const;
const TurnCircle    &turnCircle() const;
```

---

## Implementation Phases

### Phase F — `alpha()` and `beta()`

`alpha_rad` and `beta_rad` are outputs of the aerodynamic model and are passed to
`step()` as explicit parameters (same pattern as `alphaDot` and `betaDot`).
Phase F stores the incoming values so that the `alpha()` and `beta()` getters have
something to return.

```cpp
float KinematicState::alpha() const { return _alpha_rad; }
float KinematicState::beta()  const { return _beta_rad;  }
```

In `step()`: `_alpha_rad = alpha_rad; _beta_rad = beta_rad;`
In Constructor 2: store the constructor arguments directly.
In Constructor 1: initialize to `0.f`.

**Tests** (add to `test/KinematicState_test.cpp` under a `// ── Phase F` banner):

| Test name | Setup | Assert |
|---|---|---|
| `AlphaBetaZeroInitially` | Constructor 1 (no alpha/beta arg) | `alpha()=0`, `beta()=0` |
| `AlphaBetaStoredFromConstructor` | Constructor 2 with `alpha=0.2`, `beta=0.1` | `alpha()≈0.2`, `beta()≈0.1` |
| `AlphaBetaStoredFromStep` | Constructor 1; call `step()` with `alpha=0.2`, `beta=0.1` | `alpha()≈0.2`, `beta()≈0.1` |
| `AlphaBetaUpdatedEachStep` | Two consecutive `step()` calls with different values | getters reflect the most recent call |

---

### Phase G — `alphaDot()` and `betaDot()`

#### Implementation

`alphaDot` and `betaDot` are explicit parameters of `step()` and Constructor 2.
The aerodynamic model computes them from its own state — for example,
$\dot\alpha = \dot n \cdot mg / (C_{L\alpha}\,q_\infty S)$ from the load factor
derivative — and passes them in.  Phase G simply stores the incoming values so that
the `alphaDot()` and `betaDot()` getters have something to return.

In `step()`, after updating `_alphaDot_rps` and `_betaDot_rps`:

```cpp
_alphaDot_rps = alphaDot;
_betaDot_rps  = betaDot;
```

In Constructor 2, do the same:

```cpp
_alphaDot_rps = alphaDot;
_betaDot_rps  = betaDot;
```

In Constructor 1, initialize to zero:

```cpp
_alphaDot_rps = 0.f;
_betaDot_rps  = 0.f;
```

#### Analytical kinematic relationship (for reference)

The forward body-rate decomposition relates `alphaDot`, `betaDot`, and the
Wind-frame angular velocity components $[p_W, q_W, r_W]^T =
\boldsymbol{\omega}_{W/N}^W$ to the body rates:

$$
\begin{bmatrix}p\\q\\r\end{bmatrix}
=
\underbrace{\begin{bmatrix}
\cos\alpha\cos\beta & -\cos\alpha\sin\beta & -\sin\alpha \\
\sin\beta           &  \cos\beta           &  0 \\
\sin\alpha\cos\beta & -\sin\alpha\sin\beta &  \cos\alpha
\end{bmatrix}}_{C_{BW}}
\begin{bmatrix}p_W\\q_W\\r_W\end{bmatrix}
+
\begin{bmatrix}\dot\beta\sin\alpha\\\dot\alpha\\-\dot\beta\cos\alpha\end{bmatrix}
$$

Inverting (defining the residual $\mathbf{b} = [p,q,r]^T - C_{BW}[p_W,q_W,r_W]^T$):

$$\boxed{\dot\alpha = b_y = q - p_W\sin\beta - q_W\cos\beta}$$

$$\boxed{\dot\beta = p\sin\alpha - r\cos\alpha + r_W}$$

where $q_W = -a_{W,z}/V_{air}$ and $r_W = a_{W,y}/V_{air}$ from the transport
theorem ($p_W$, $q_W$, $r_W$ are true angular velocity components of the Wind
frame w.r.t. NED, **not** Euler rates).

This kinematic relationship is satisfied automatically when the aerodynamic model
and kinematic integrator are self-consistent.  It is not used as the implementation
path for alphaDot/betaDot — that responsibility lies with the aerodynamic model.

#### Tests (under `// ── Phase G` banner)

| Test name | Setup | Expected |
|---|---|---|
| `AlphaDotBetaDotZeroInitially` | Constructor 1 (no alphaDot/betaDot arg) | `alphaDot()=0`, `betaDot()=0` |
| `AlphaDotStoredFromConstructor` | Constructor 2 with `alphaDot=0.05`, `betaDot=0.01` | `alphaDot()≈0.05`, `betaDot()≈0.01` |
| `AlphaDotStoredFromStep` | Construct; call `step()` with `alphaDot=0.05`, `betaDot=0.01` | `alphaDot()≈0.05`, `betaDot()≈0.01` |
| `AlphaDotUpdatedEachStep` | Two consecutive `step()` calls with different alphaDot values | `alphaDot()` reflects the most recent call |

---

### Phase G — Worked Example: Load-Factor Ramp

#### Problem statement

The **sole free input** is the commanded load factor profile.  Every other
quantity — angle of attack, angle-of-attack rate, body rates, attitude,
flight path angle, and altitude — is derived from it via the coupled
aerodynamic-kinematic equations.  The example verifies that the aerodynamic
model correctly supplies $\dot\alpha$ as the derivative of the load factor, and
that this value is consistent with the kinematic body-rate decomposition.

#### Aircraft parameters

| Parameter | Symbol | Value |
|---|---|---|
| Lift-curve slope (linear region) | $C_{L\alpha}$ | 5.73 rad⁻¹ |
| Mass | $m$ | 1 200 kg |
| Wing area | $S$ | 16 m² |
| Airspeed (constant) | $V$ | 100 m/s |
| Air density | $\rho$ | 1.225 kg/m³ |
| Dynamic pressure | $q_\infty = \tfrac{1}{2}\rho V^2$ | 6 125 Pa |

#### Sole external input

$$n(t) = 1 + \frac{t}{5}, \quad \frac{dn}{dt} = 0.2\;\text{s}^{-1} \quad \text{(constant ramp, 1 → 2 over 5 s)}$$

This determines the net Wind-frame acceleration:

$$a_{W,z}(t) = -(n-1)\,g = -\frac{t\,g}{5}\;\text{m/s}^2, \qquad a_{W,y}(t) = 0, \qquad p_W = 0, \quad \beta = 0$$

#### Aerodynamic coupling — derive $\alpha$ and $\dot\alpha$ from $n$

In the linear lift region, $n = C_{L\alpha}\,\alpha\,q_\infty S\,/\,(mg)$, so:

$$\alpha(t) = \frac{n(t)\,m\,g}{C_{L\alpha}\,q_\infty\,S}
= n(t)\;\underbrace{\frac{m\,g}{C_{L\alpha}\,q_\infty\,S}}_{\alpha_{trim}}
= n(t)\;\frac{1200\times 9.81}{5.73\times 6125\times 16}
= n(t)\times 0.02096\;\text{rad}$$

At $n=1$: $\alpha_{trim} = 0.02096\;\text{rad} = 1.20°$.

Differentiating:

$$\boxed{\dot\alpha = \frac{dn/dt\cdot m\,g}{C_{L\alpha}\,q_\infty\,S}
= \frac{0.2\times 11\,772}{5.73\times 98\,000}
= 0.004\,193\;\text{rad/s} = 0.240°/\text{s}} \quad \text{(constant)}$$

This confirms the user's original expectation: **$\dot\alpha$ is proportional to the
time derivative of the vertical load factor**, scaled by the aircraft lift
parameters.

#### Kinematic route — $q_W$ and the resulting body pitch rate

From the transport theorem (see analytical kinematic relationship in Phase G):

$$q_W(t) = \frac{-a_{W,z}}{V} = \frac{(n-1)\,g}{V} = \frac{t\times 9.81}{500} = 0.01962\,t\;\text{rad/s}$$

The body pitch rate follows from the body-rate decomposition (not an input — it is
the kinematic state propagated by `step()`):

$$q(t) = \dot\alpha + q_W(t) = 0.004\,193 + 0.01962\,t\;\text{rad/s}$$

**Consistency check** — the aerodynamic and kinematic routes agree:
- Aerodynamic: $\dot\alpha = \dot n \cdot mg / (C_{L\alpha}\,q_\infty S)$
  — computed by the aerodynamic model (`LoadFactorAllocator`) and passed to `step()` as a parameter
- Kinematic: $\dot\alpha = q - q_W$ — the same value recoverable from body rates
  and path curvature after the fact, as documented in the analytical relationship above
  (this is a consistency property, not the implementation path)

#### Integrated state time history

$$\alpha(t) = n(t)\,\alpha_{trim} = \bigl(1+t/5\bigr)\times 0.02096\;\text{rad}$$

$$\gamma(t) = \int_0^t q_W\,d\tau = \frac{g\,t^2}{10V} = 0.009\,81\,t^2\;\text{rad}$$

$$\theta(t) = \alpha(t)+\gamma(t) = 0.02096\,(1+t/5)+0.009\,81\,t^2\;\text{rad}$$

$$\dot\theta(t) = q(t) = 0.004\,193 + 0.01962\,t\;\text{rad/s}$$

Verify: $\dot\theta = \dot\alpha + \dot\gamma = \dot\alpha + q_W$ ✓ (transport theorem, $\mu=0$, $\dot\chi=0$).

$$V_{z,\uparrow}(t) \approx V\,\gamma(t) = 0.981\,t^2\;\text{m/s}, \qquad
h(t) = \int_0^t V_{z,\uparrow}\,d\tau = 0.327\,t^3\;\text{m}$$

#### Summary table

| $t$ (s) | $n$ | $\alpha$ (°) | $\dot\alpha$ (°/s) | $q_W$ (°/s) | $q$ (°/s) | $\gamma$ (°) | $\theta$ (°) | $V_{z\uparrow}$ (m/s) | $h$ (m) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 1.00 | 1.20 | 0.24 | 0.00 | 0.24 | 0.00 | 1.20 | 0.00 | 0.00 |
| 1 | 1.20 | 1.44 | 0.24 | 1.12 | 1.36 | 0.56 | 2.00 | 0.98 | 0.33 |
| 2 | 1.40 | 1.68 | 0.24 | 2.25 | 2.49 | 2.25 | 3.93 | 3.92 | 2.62 |
| 3 | 1.60 | 1.92 | 0.24 | 3.37 | 3.61 | 5.06 | 6.98 | 8.83 | 8.83 |
| 4 | 1.80 | 2.16 | 0.24 | 4.50 | 4.74 | 8.99 | 11.15 | 15.70 | 20.93 |
| 5 | 2.00 | 2.40 | 0.24 | 5.62 | 5.86 | 14.08 | 16.48 | 24.33 | 40.94 |

$\dot\alpha$ is constant throughout because $dn/dt$ is constant and the lift curve is linear.
$q = \dot\alpha + q_W$ is the **derived** body pitch rate — it is not an input.

#### Time history plots

![Phase G worked example — time history](figures/task2_phase_g_example.png)

**Key observations:**
1. $\alpha$ grows linearly because $n$ grows linearly and $\alpha = n\,\alpha_{trim}$ in the linear lift region.
2. $\dot\alpha = 0.240°/\text{s}$ is **constant** and equals $\dot n \cdot m\,g\,/\,(C_{L\alpha}\,q_\infty\,S)$ — explicitly dependent on the derivative of the load factor and the aircraft lift parameters.
3. $q_W$ grows linearly with $n-1$ — the path curvature increases as the velocity vector arcs upward more steeply.
4. The derived body pitch rate $q = \dot\alpha + q_W$ has two additive components: the small constant $\dot\alpha$ contribution from the rising AoA, and the growing $q_W$ from the load factor.  $q_W$ dominates after $t \approx 0.2\,\text{s}$.
5. Phase G recovers the same $\dot\alpha$ from the stored body rates without ever accessing $C_{L\alpha}$, $q_\infty$, or $S$ — the kinematic and aerodynamic routes are consistent.

---

### Phase H — Euler rate methods

**Implementation:**
```cpp
float KinematicState::rollRate_rps()   const {
    return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(0);
}
float KinematicState::pitchRate_rps()  const {
    return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(1);
}
float KinematicState::headingRate_rps() const {
    return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(2);
}
```

`BodyRatesToEulerRates` already exists and handles gimbal lock by returning
zero at `|cos(pitch)| < 1e-6`.

**Tests** (under `// ── Phase H` banner):

| Test name | Setup | Assert |
|---|---|---|
| `EulerRatesAllZeroForZeroBodyRates` | Constructor 2, `q_nb=I`, `rates=[0,0,0]` | all three getters = 0 |
| `HeadingRateEqualsYawBodyRateAtLevelFlight` | `q_nb=I`, `rates=[0,0,r]` | `headingRate_rps()=r`, others≈0 |
| `PitchRateEqualsBodyPitchAtLevelFlight` | `q_nb=I`, `rates=[0,q,0]` | `pitchRate_rps()=q`, others≈0 |
| `RollRateEqualsBodyRollAtLevelFlight` | `q_nb=I`, `rates=[p,0,0]` | `rollRate_rps()=p`, others≈0 |

> **Note:** These four tests use `q_nb = Identity` (zero Euler angles), which
> makes `BodyRatesToEulerRates` = identity mapping.  A combined non-trivial
> angle test is deferred to Phase H integration (low risk — the matrix is
> already unit-tested by the static function).

---

### Phase I — `rollRate_Wind_rps()`

`rollRate_Wind_rps` (p_W) is an explicit `step()` input from the roll-control model.
Phase I stores the incoming value so that the getter has something to return.

```cpp
float KinematicState::rollRate_Wind_rps() const { return _rollRate_Wind_rps; }
```

In `step()`: `_rollRate_Wind_rps = rollRate_Wind_rps;`
In Constructor 2: store the constructor argument directly.
In Constructor 1: initialize to `0.f`.

#### Kinematic relationship (for reference)

The body-rate decomposition gives the consistency identity:

$$
p_W = \bigl(C_{WB}\,\mathbf{r}_{body}\bigr)_0 - \sin\beta\cdot\dot\alpha
$$

where $C_{WB} = (C_y(\alpha)\,C_z(-\beta))^T$ and $\mathbf{r}_{body}$ = `_rates_Body_rps`.
This is a consistency property, not the implementation path.

**Tests** (under `// ── Phase I` banner):

| Test name | Setup | Assert |
|---|---|---|
| `RollRateWindZeroInitially` | Constructor 1 | `rollRate_Wind_rps()=0` |
| `RollRateWindStoredFromConstructor` | Constructor 2 with `rollRate_Wind=0.3` | `rollRate_Wind_rps()≈0.3` |
| `RollRateWindStoredFromStep` | Constructor 1; call `step()` with `rollRate_Wind=0.3` | `rollRate_Wind_rps()≈0.3` |
| `RollRateWindUpdatedEachStep` | Two consecutive `step()` calls with different values | getter reflects the most recent call |

---

### Phase J — `q_ns()` and `velocity_Stab_mps()`

**Math:**

The Stability frame (S) is the Wind frame with β=0 by definition — it lies in
the aircraft symmetry plane.  Starting from the Wind frame and pitching up by α:
```
q_ns = _q_nw * Ry(alpha())
```

Stability-frame velocity:
```
velocity_Stab_mps = q_ns().toRotationMatrix().transpose() * _velocity_NED_mps
```

**Tests** (under `// ── Phase J` banner):

| Test name | Setup | Assert |
|---|---|---|
| `QnsEqualsQnwAtZeroAlpha` | `q_nb=I`, `v=[50,0,0]` (α=0) | `q_ns().matrix() ≈ _q_nw.matrix()` |
| `QnsRotatedByAlphaFromQnw` | `q_nb=Ry(α)`, `v=[50,0,0]` | `q_ns().matrix() ≈ Ry(α).matrix()` |
| `VelocityStabXIsAirspeedAtZeroAeroAngles` | `q_nb=I`, `v=[50,0,0]`, no wind, α=β=0 | `velocity_Stab_mps().x() ≈ 50`, y≈0, z≈0 |
| `VelocityStabWithBetaHasZeroYComponent` | nonzero β | `velocity_Stab_mps().y() ≈ 0` (β projected out) |

---

### Phase K — `q_nl()`

**Background:** `WGS84_Datum::qne()` returns the `double`-precision quaternion
that rotates vectors from the Earth-fixed (ECEF) frame to the local NED frame
at the datum position.  The Local Level frame (L) is the horizontal tangent
plane at the current aircraft position; its relationship to the fixed NED frame
in this codebase is defined as:

```
q_nl = Quaternionf(_positionDatum.qne().cast<float>())
```

This treats the ECEF-to-NED rotation at the current position as the NED-to-Local
Level rotation — appropriate when NED is defined at a local datum close to the
aircraft.

> **Open question for review:** Confirm whether `q_nl` is intended to represent
> the rotation between the *fixed* NED datum and the *current* tangent plane
> (i.e., accounting for geodetic curvature as the aircraft moves), or simply
> the ECEF-to-NED rotation at the current position.  For short flights these
> are nearly identical.  The plan assumes the latter; correct if needed before
> implementation.

**Tests** (under `// ── Phase K` banner):

| Test name | Setup | Assert |
|---|---|---|
| `QnlIsUnitQuaternion` | Default datum (lat=0, lon=0, h=0) | `q_nl().norm() ≈ 1` |
| `QnlAtEquatorMapsNEDCorrectly` | Default datum | NED north vector maps to expected ECEF direction |

---

### Phase L — `POM()` and `turnCircle()`

#### API fix first

Change header declarations before any test is written:
```cpp
const PlaneOfMotion &POM() const;
const TurnCircle    &turnCircle() const;
```

Add `mutable PlaneOfMotion _pom` and `mutable TurnCircle _turn_circle` to the
protected section.

#### `POM()` math

The Plane of Motion is the Frenet frame of the airmass-relative velocity path:

```
v    = _velocity_NED_mps − _wind_NED_mps          // airmass velocity in NED
V    = v.norm()
xhat = v / V                                       // tangent (velocity direction)

a    = _acceleration_NED_mps
a_par  = a.dot(xhat) * xhat                        // tangential component
a_perp = a − a_par                                 // centripetal component

if a_perp.norm() < threshold:
    // straight flight: POM undefined; return identity q_np (no rotation)
else:
    yhat = a_perp / a_perp.norm()                  // toward center of curvature
    zhat = xhat.cross(yhat)                        // binormal

    C_NP = [xhat | yhat | zhat]  (columns)
    _pom.q_np = Quaternionf(C_NP)
```

Below the small-V threshold or when `a_perp` is near zero (straight flight),
`q_np` defaults to Identity.

#### `turnCircle()` math

```
turnCircle depends on POM().

curvature = a_perp.norm() / (V * V)      // 1/R  (1/m)
R = 1 / curvature                        // turn radius (m)

// Center of curvature is one radius in the yhat direction from the aircraft
_turn_circle.pom = _pom
_turn_circle.turnCenter_deltaNED_m = R * yhat   // displacement from aircraft to center, in NED
```

#### Tests (under `// ── Phase L` banner):

| Test name | Setup | Assert |
|---|---|---|
| `POMIdentityForStraightFlight` | `a_NED = [0,0,0]` or parallel to velocity | `q_np ≈ Identity` |
| `POMXAxisAlignedWithVelocity` | Any nonzero velocity and centripetal accel | First column of `q_np.matrix() ≈ v.normalized()` |
| `POMYAxisTowardCurvatureCenter` | Horizontal turn: `v=[V,0,0]`, `a=[0,a,0]` | Second column of `q_np.matrix() ≈ [0,1,0]` |
| `TurnCircleRadiusCorrect` | `V=50`, centripetal `a_perp=5 m/s²` | `turnCenter_deltaNED_m.norm() ≈ V²/a_perp = 500` |
| `TurnCircleCenterInCurvatureDirection` | Same setup as above | `turnCenter_deltaNED_m.normalized() ≈ yhat` |

---

## Implementation Order Summary

```
Phase G   alphaDot(), betaDot()           [trivial storage; no signature change]
Phase F   alpha(), beta()                 [independent of G]
Phase H   rollRate_rps(), pitchRate_rps(), headingRate_rps()   [independent]
Phase I   rollRate_Wind_rps()             [depends on Phase F]
Phase J   q_ns(), velocity_Stab_mps()    [depends on Phase F]
Phase K   q_nl()                          [independent; resolve open question first]
Phase L   POM(), turnCircle()             [API fix required; independent of F–K]
```

Phases F, G, H, K, and L are independent of each other and may be done in any
order.  Phase I and Phase J depend on Phase F completing first.

---

## Aerodynamic Model Interface

`KinematicState::step()` is the integration sink.  It receives the outputs of the
aerodynamic model (and environment model) as its parameters:

| `step()` parameter | Source | Meaning |
|---|---|---|
| `acceleration_Wind_mps` | Aerodynamic/propulsion model | Net Wind-frame acceleration (lift + drag + thrust + gravity), expressed in Wind frame |
| `rollRate_Wind_rps` | Roll-control model | Wind-axis roll rate $p_W$ — drives `_q_nw` propagation |
| `alpha_rad` | Aerodynamic model | Angle of attack — used to propagate `_q_nb` and compute `omega_bw_b` |
| `beta_rad` | Aerodynamic model | Sideslip angle — same uses as above |
| `alphaDot` | Aerodynamic model | Rate of change of angle of attack — stored in `_alphaDot_rps` |
| `betaDot` | Aerodynamic model | Rate of change of sideslip — stored in `_betaDot_rps` |
| `windSpeed_mps`, `windDirFrom_rad` | Environment model | Ambient horizontal wind |

### Data flow (one integration step)

```
Environment model → windSpeed, windDirFrom
                              ↓
Aerodynamic/propulsion model → a_W, alpha, alphaDot, beta, betaDot
                              ↓
Roll-control model           → rollRate_Wind_rps (p_W)
                              ↓
KinematicState::step(t, a_W, p_W, alpha, alphaDot, beta, betaDot, windSpeed, windDirFrom)
  Store  : _alphaDot_rps = alphaDot, _betaDot_rps = betaDot
  Step   : velocity, position, _q_nw, _q_nb, _rates_Body_rps updated
```

The kinematic integrator does not call back into the aerodynamic model.  The
coupling is one-directional per step: aero model outputs → kinematic inputs.
The aerodynamic model reads `alpha()`, `beta()`, and `velocity_Wind_mps()` from the
kinematic state at the start of the next step to compute its own next-step outputs.

---

## Files to Modify

| File | Change |
|---|---|
| `include/KinematicState.hpp` | Add `_alpha_rad`, `_beta_rad`, `_alphaDot_rps`, `_betaDot_rps` members; add `mutable _pom`, `_turn_circle`; fix `POM()`/`turnCircle()` return types |
| `src/KinematicState.cpp` | Update all constructors; update `step()`; add Phase F–L implementations |
| `test/KinematicState_test.cpp` | Add Phase F–L test suites |

No CMake changes are needed.
