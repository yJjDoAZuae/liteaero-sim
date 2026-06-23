# Load-Factor Allocation

## Overview

The load-factor `Aircraft` model is commanded by **load factor**, not by angle of attack or control
surface. Each step it receives a normal load factor $n_z$ and a lateral load factor $n_y$ (both in
g's, after command-filter shaping) and must determine the **angle of attack** $\alpha$ and **sideslip**
$\beta$ at which the airframe flies to deliver them. *Load-factor allocation* is that inversion: given
$(n_z, n_y)$, the current dynamic pressure, thrust, and mass, solve the force-balance equations for
$(\alpha, \beta)$.

The inversion is nonlinear because the lift coefficient $C_L(\alpha)$ is nonlinear (it rounds over at
stall and falls to a separated plateau). It is solved by Newton iteration, warm-started from the
previous step for branch continuity, with the angle of attack confined to a configured box and to the
branch of physically achievable load factors. This document gives the equations, the solution method,
the achievable-load-factor envelope, the post-stall hysteresis and lift recovery, the analytical rate
derivatives, and — importantly for low-speed ground behavior — the numerical conditioning.

It is cited by the `Aircraft` design document
([aircraft.md](../design/aircraft.md)); the lift-curve model $C_L(\alpha)$ it inverts is derived in
[aerodynamics.md](aerodynamics.md).

Symbols used throughout:

| Symbol | Meaning | Units |
| --- | --- | --- |
| $n_z,\ n_y$ | commanded normal / lateral load factor | g (dimensionless) |
| $q = \tfrac{1}{2}\rho V^2$ | dynamic pressure | Pa |
| $S$ | reference wing area | m² |
| $m,\ g$ | aircraft mass, gravitational acceleration | kg, m/s² |
| $T$ | thrust magnitude | N |
| $\alpha,\ \beta$ | angle of attack, sideslip angle | rad |
| $C_L(\alpha)$ | lift coefficient (lift-curve model) | — |
| $C_{L_\alpha}$ | linear-region lift-curve slope, $dC_L/d\alpha$ | rad⁻¹ |
| $C_{Y_\beta}$ | side-force-due-to-sideslip derivative | rad⁻¹ |
| $C_{L_\text{max}},\ C_{L_\text{sep}}$ | stall-peak and separated-plateau lift coefficients | — |
| $\alpha_\text{peak},\ \alpha^\star$ | lift-curve peak AoA; achievable-$n_z$ fold AoA | rad |
| $\alpha_\text{min},\ \alpha_\text{max}$ | configured AoA box limits (envelope protection) | rad |

---

## Normal-Axis Inversion ($n_z \to \alpha$)

The angle of attack that delivers the commanded normal load factor.

### Continuous Formulation

The normal load factor is produced by aerodynamic lift plus the normal component of thrust, balanced
against the commanded inertial load:

$$
f(\alpha) \;=\; q\,S\,C_L(\alpha) \;+\; T\sin\alpha \;-\; n_z\,m\,g \;=\; 0 .
$$

Gravity does not appear explicitly: the allocator generates the lift that *equals* $n_z\,m\,g$, and at
$n_z = 1$ in steady level flight that lift cancels weight. The derivative used by the solver is

$$
f'(\alpha) \;=\; q\,S\,C_L'(\alpha) \;+\; T\cos\alpha ,
$$

where $C_L'(\alpha) = dC_L/d\alpha$ comes from the lift-curve model (linear slope $C_{L_\alpha}$ in
the attached region, rolling to zero at the stall peak; see [aerodynamics.md](aerodynamics.md) Part 3).

### Solution Method

$f(\alpha) = 0$ is solved by **Newton iteration**:

$$
\alpha_{k+1} \;=\; \alpha_k - \frac{f(\alpha_k)}{f'(\alpha_k)} .
$$

Two features make the iteration robust and continuous across steps:

- **Branch-continuation warm start.** The first iterate is a first-order extrapolation from the
  converged solution of the previous step using the implicit-function-theorem sensitivity,
  $$
  \alpha_0 \;=\; \alpha_\text{prev} + \big(n_z - n_{z,\text{prev}}\big)\,\frac{m\,g}{f'(\alpha_\text{prev})} ,
  $$
  which keeps the solver on the same physical branch when the lift curve is multivalued in $C_L$
  (the same $C_L$ occurs pre- and post-stall).
- **Box constraint.** Each proposed iterate is projected onto $[\alpha_\text{min}, \alpha_\text{max}]$.
  These limits represent the FBW angle-of-attack envelope protection — a structural/geometric ceiling
  independent of the aerodynamic stall boundary.

### Achievable Load-Factor Envelope (the fold)

The achievable normal load factor as a function of angle of attack,

$$
N_z(\alpha) \;=\; \frac{q\,S\,C_L(\alpha) + T\sin\alpha}{m\,g} ,
$$

is **not monotonic**: past the lift-curve peak the falling $C_L$ outweighs the rising $T\sin\alpha$, so
$N_z(\alpha)$ has a maximum at the angle $\alpha^\star$ where

$$
\frac{dN_z}{d\alpha}\bigg|_{\alpha^\star} = 0 \quad\Longleftrightarrow\quad f'(\alpha^\star) = 0 .
$$

$\alpha^\star$ is the **fold** of the inversion — the largest load factor the airframe can produce at
the current $q$ and $T$. When the command exceeds $N_z(\alpha^\star)$ no real root exists beyond the
fold, so the solver detects $|f'(\alpha)| \to 0$ and **clamps $\alpha$ at $\alpha^\star$** rather than
continuing onto the post-stall branch (which would be a discontinuous jump). For $T \le 0$ the fold
coincides with the lift-curve peak, $\alpha^\star = \alpha_\text{peak}$ (where $C_L' = 0$); for $T > 0$
it occurs at a higher angle, because the thrust term keeps contributing past the aerodynamic peak. At
the clamp the realized load factor is the maximum achievable, $N_z(\alpha^\star) < n_z$, and no error
is raised.

### Numerical Properties

In the attached (linear) region $C_L(\alpha) \approx C_{L_\alpha}\,\alpha$, and for $T$ small relative
to $qS C_{L_\alpha}$ the solution is approximately

$$
\boxed{\;\alpha \;\approx\; \frac{n_z\,m\,g}{q\,S\,C_{L_\alpha}} \;\propto\; \frac{n_z}{q}\;}
$$

so the inversion is **ill-conditioned as $q \to 0$**: at vanishing dynamic pressure any nonzero $n_z$
demands an unbounded angle of attack. This is benign in flight (a small $n_z$ at low $q$ is simply not
achievable and the fold clamps it), but it matters on the ground during roll-out, where the solved
$\alpha$ feeds the body attitude: a small *residual* $n_z$ that has not been driven exactly to zero is
divided by a vanishing $q$, exceeds the (equally vanishing) achievable envelope $N_z(\alpha^\star)$,
and is therefore clamped at the fold $\alpha^\star$ — a large, $q$-independent angle, even though the
realized lift is negligible. Note that a static cap on the commanded $n_z$ at the achievable envelope
does not remedy this, because the envelope maximum is itself reached at $\alpha^\star$. Attenuating the
commanded $n_z$ by a dynamic-pressure effectiveness weight (proportional to the marginal effectiveness
$\partial N_z/\partial\alpha = q\,S\,C_{L_\alpha}/(m g) \propto q$) at the command-processing stage of
the using model — so the commanded $n_z$, and therefore the commanded $\alpha$, decays to zero as the
effector loses authority — is the subject of the using design document
([landing_gear.md](../design/landing_gear.md), the low-speed roll-out behavior).

---

## Lateral-Axis Inversion ($n_y \to \beta$)

The sideslip that delivers the commanded lateral load factor, solved **after** $\alpha$ (it uses the
converged $\alpha$).

### Lateral Continuous Formulation

$$
g(\beta) \;=\; q\,S\,C_{Y_\beta}\,\beta \;-\; T\cos\alpha\,\sin\beta \;-\; n_y\,m\,g \;=\; 0,
\qquad
g'(\beta) \;=\; q\,S\,C_{Y_\beta} \;-\; T\cos\alpha\,\cos\beta .
$$

The side force is modeled linearly in $\beta$ (slope $C_{Y_\beta}$); the thrust term removes the
component of thrust that already acts laterally once the body yaws by $\beta$.

### Lateral Solution Method

Newton iteration $\beta_{k+1} = \beta_k - g(\beta_k)/g'(\beta_k)$, warm-started from the previous
step, $\beta_0 = \beta_\text{prev} + (n_y - n_{y,\text{prev}})\,m g / g'(\beta_\text{prev})$, and
terminating when the step falls below tolerance or $|g'| \to 0$.

---

## Stall Hysteresis and Lift Recovery

Beyond the fold the airframe is stalled, and lift does not snap back to the nominal curve when the
angle of attack is reduced — separated flow reattaches over a finite time. The allocator therefore
distinguishes the **nominal** lift coefficient $C_L(\alpha)$ from the **effective** coefficient
$C_L^\text{eff}$ it actually reports, with hysteresis.

- **Stall entry / exit (hysteresis).** A positive-side stall flag is set once $\alpha$ has passed the
  peak vertex and begins to reverse; it clears when $\alpha$ falls back below a recovery threshold
  $\alpha^\star_\text{thresh}$ (or when the nominal curve drops to the separated plateau). The negative
  side mirrors this about the negative stall trough.
- **Plateau hold.** While stalled, $C_L^\text{eff}$ is held at the separated plateau
  $C_{L_\text{sep}}$ (the post-stall flat-lift value), not at the nominal curve.
- **Rate-limited recovery.** When the stall clears, $C_L^\text{eff}$ returns to nominal **rate-limited
  upward**,
  $$
  C_L^\text{eff}[k+1] \;=\; \min\!\big(\,C_L(\alpha),\; C_L^\text{eff}[k] + \dot C_{L,\text{max}}\,\Delta t\,\big),
  \qquad \dot C_{L,\text{max}} = C_{L_\alpha}\,\dot\alpha_\text{max},
  $$
  until it catches the nominal value, after which it tracks nominal directly. The rate is set by a
  configured maximum angle-of-attack rate $\dot\alpha_\text{max}$ (the reattachment timescale).
- **Gating.** The rate limit applies **only during genuine post-stall recovery** — armed by an actual
  stall (captured at step entry, so the step on which the stall clears still begins recovery) and
  released once $C_L^\text{eff}$ has caught nominal. In attached flow it does **not** act:
  $C_L^\text{eff} = C_L(\alpha)$ directly. (The design rationale and the consequences of getting this
  gate wrong are in [aircraft.md §Stall Recovery](../design/aircraft.md).)

The realized normal load factor reported back uses the effective coefficient,
$N_{z,\text{realized}} = \big(q S\,C_L^\text{eff} + T\sin\alpha\big)/(m g)$, which equals the command
in attached flow and is below it while stalled or recovering.

---

## Analytical Rate Derivatives ($\dot\alpha,\ \dot\beta$)

The solver also returns the time derivatives $\dot\alpha,\ \dot\beta$ analytically — not by finite
differencing — so downstream consumers (e.g. body-rate construction) have lag-free rates. They follow
from the **implicit function theorem** applied to the converged force balance. Treating $f(\alpha,
n_z) = 0$ as an implicit relation $\alpha(n_z)$,

$$
\frac{\partial f}{\partial \alpha}\,\dot\alpha + \frac{\partial f}{\partial n_z}\,\dot n_z = 0
\quad\Longrightarrow\quad
\dot\alpha \;=\; -\frac{\partial f/\partial n_z}{\partial f/\partial \alpha}\,\dot n_z
\;=\; \frac{m\,g}{f'(\alpha)}\,\dot n_z ,
$$

since $\partial f/\partial n_z = -m g$ and $\partial f/\partial \alpha = f'(\alpha)$. The commanded
rate $\dot n_z$ is itself obtained analytically from the command-filter state (see
[aircraft.md §Derivative Sourcing](../design/aircraft.md)). The lateral derivative is the analogue,

$$
\dot\beta \;=\; \frac{m\,g}{g'(\beta)}\,\dot n_y .
$$

While stalled the Newton branch is inactive, so $f'$ is evaluated on the flat-$C_L$ (thrust-only)
expression and the rate is taken as zero where the achievable envelope is saturated.

---

## References

- Stevens, B. L., Lewis, F. L., & Johnson, E. N. *Aircraft Control and Simulation*, 3rd ed., Wiley,
  2016 — trim/force-balance inversion and load-factor relations.
- Etkin, B., & Reid, L. D. *Dynamics of Flight: Stability and Control*, 3rd ed., Wiley, 1996 — lift
  curve and stall behavior.
- Press, W. H., et al. *Numerical Recipes*, 3rd ed., Cambridge, 2007 — Newton–Raphson iteration and
  conditioning.
- The lift-curve model $C_L(\alpha)$ parameters are derived in [aerodynamics.md](aerodynamics.md);
  the implicit function theorem follows any standard real-analysis text.
