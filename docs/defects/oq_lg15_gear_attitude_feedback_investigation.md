# Defect Investigation — Gear–Attitude Feedback Artifact (`LandingGear_FullStop_SpeedNearZero`)

**Status:** Root cause confirmed. Fix designed and recorded in
[`landing_gear.md` — Integration Contract — `Aircraft` §2](../design/landing_gear.md);
the open question that tracked this defect is [OQ-LG-15](../design/landing_gear.md) (resolved).
Filter parameterization for the fix is [OQ-LG-17](../design/landing_gear.md).

**Scope of this report:** the diagnostic investigation only — the symptom, method, confirmed
root cause, and supporting data. The *fix* is design content and lives in the design document
(Integration Contract — `Aircraft` §2); it is not repeated here.

---

## Symptom

The `LandingGear_FullStop_SpeedNearZero` scenario test — aircraft at V = 15 m/s must
decelerate to < 0.5 m/s within 90 s via rolling resistance — instead settles into a
slowly-decaying limit cycle near 5 m/s and never reaches the threshold.

**Test fixture aircraft model.** The fixture uses `makeConfig()`, a synthetic C172-class
aircraft: mass = 1045 kg, S_ref = 16.2 m², AR = 7.47, CL_max = 1.80, V_ne = 82.3 m/s. These
parameters match published Cessna 172S data closely (wing area is exact); the fixture is not
formally declared as a C172.

> **Superseded hypotheses (2026-05-31).** Earlier analysis hypothesized an aerodynamic "energy
> injection" via $F_\text{wind,x} = -F_\text{spring}\sin\alpha_0$ and a gear-spring bounce
> limit cycle at the spring natural frequency (1.21 Hz). The full-resolution diagnostic refuted
> both. The actual mechanism is the numerical artifact below.

## Method

The diagnostic test `LandingGear_FullStop_OQ_LG15_Diagnostic` logs every state, force, and
attitude at the full 0.02 s timestep for 300 s, and `WheelUnit::step()` was instrumented to
log the per-wheel contact-force breakdown (slip ratio, contact-patch velocity, force
components) through a spike event. Figures are generated from
`build/test/oq_lg15_diagnostic.csv` by `docs/defects/img/plot_oq_lg15.py`.

*Diagnostic instrumentation still in tree (TEMPORARY — remove when the fix lands):*
`WheelUnit::ContactDiag` struct + `_diag` member + `lastContactDiag()` accessor in
`WheelUnit.hpp/.cpp`, and the `nose_*` columns in the diagnostic test CSV.

## Root cause (confirmed)

**1. The aircraft wheelbarrows on the nose gear from t = 38.4 s onward.** As speed drops
through V ≈ 6.3 m/s the main gear leaves the ground and never returns (Figures 2, 3). The
LFA holds α = 0° throughout (lift fully suppressed by `n_contact_z_filt` = 1), so body pitch
tracks the flight-path angle. The nose-down attitude during descent puts the nose wheel lower
than the mains; the aircraft rides on the nose wheel alone for the rest of the run (Figure 4
confirms main-gear contact patches stay 30–50 mm above terrain).

**2. The limit cycle has two timescales** (Figure 1):

- A **high-frequency backward-impulse train** at 17.3 Hz. Each nose-wheel contact lasts
  exactly one timestep and applies a backward force averaging −1,686 N (wind-frame x),
  removing ≈ 0.0115 m/s per impulse. About 40 such impulses occur per limit-cycle period.
- A **low-frequency forward spike** every 2.411 s (0.41 Hz). A single timestep applies a
  **+22,717 N forward impulse** (≈ 22 × aircraft weight), adding **+0.476 m/s** in one step.
  This exactly resets the speed the backward-impulse train bled off.

The two nearly balance, producing a quasi-stable speed near 5 m/s that decays only very slowly
(the test's required 0.2 m/s² rolling deceleration is absent because the aircraft is airborne
65% of the time and the single contacting wheel carries little normal force).

**3. The deep penetration is driven by the pitch attitude through the nose-wheel lever arm,
NOT by descent.** At the spike the strut deflection is δ ≈ 0.077 m, but the CG descends only
−0.0016 m on that step (vD ≈ 0.30 m/s gives at most 0.006 m of sink per 0.02 s step). The
missing 13× comes from attitude: because the LFA holds α = 0, body pitch **equals the
flight-path angle**, θ = atan2(−vD, vN) (verified: corr(pitch, FPA) = 1.0000, mean
|pitch − FPA| = 0.001° over the run). Each one-step bounce reverses vD, swinging the
flight-path angle — hence pitch — by **3–5° per step** (mean per-step Δpitch = 2.96°, max
5.10°). The nose wheel sits 2 m forward of the CG, so a pitch swing of Δθ moves the nose
contact point vertically by ≈ 2·sin(Δθ): a 2.4° swing dips the nose 0.085 m. The strut
deflection tracks this lever-arm dip, not the CG motion (max deflection 0.085 m = 14× the
maximum per-step descent of 0.006 m).

The implied pitch rates are **148 deg/s mean, 255 deg/s peak** — physically impossible for
this airframe. A C172-class aircraft (Iyy = 1825 kg·m²) under even a 60 kN·m gear moment
pitches at 32 rad/s² and cannot slew attitude at hundreds of deg/s. The attitude moves this
fast only because it has **no rotational inertia**: in the load-factor model pitch is a
kinematic function of the velocity vector (θ = FPA + α, α held by the LFA), tracking velocity
changes instantaneously.

The finite-difference rate estimate then reads δ̇ = (0.077 − 0)/0.02 ≈ 3.9 m/s — a spurious
4 m/s closing rate that is really the lever-arm sweep, not strut compression — driving a strut
normal force **F_body_z = −34,364 N** (3.4 × weight): spring plus a ~10 kN damping term.

**4. The forward force is a body-frame longitudinal force, NOT an attitude projection.** At the
spike, F_ned_x = +21,823 N forward while pitch = 1.51°. Projecting the 34,364 N normal force at
1.51° yields only |F_body_z|·sin(1.51°) ≈ 905 N — **24× too small**. Because α = 0 and β ≈ 0,
the wind frame coincides with the body frame (confirmed: F_wind_z = F_body_z = −34,364 N
exactly), so F_wind_x (+22,736 N) = **F_body_x** — a genuine forward longitudinal force, the
Pacejka F_x = +24 kN at κ ≈ +0.95. A free-rolling wheel can only produce backward rolling
resistance (≈ −687 N); the 22.7 kN forward force is the energy source.

### Confirmed sub-mechanism (instrumented force breakdown)

The +22.7 kN forward body-x force is the **Pacejka longitudinal force** $F_x = +24{,}169$ N,
driven by a slip ratio $\kappa = +0.946$. The decomposition is exact:

$$F_{body,x} = \underbrace{F_x \cos\theta_w}_{+24{,}122} + \underbrace{F_{rr}\cos\theta_w}_{+656}
+ \underbrace{F_z\,(\hat n_x)}_{-2{,}043} = +22{,}736\ \text{N}$$

The normal-force projection term contributes only −2,043 N, definitively ruling out attitude
projection. The slip ratio is positive because:

1. The contact-patch forward velocity $V_{cx} = (\mathbf{v}_{body} +
   \boldsymbol\omega\times\mathbf{c}_{body})\cdot\hat{\mathbf{w}}_{fwd}$ is slightly
   **negative** (−0.176 m/s) at the spike. The $\boldsymbol\omega\times\mathbf{c}$ term — large
   spurious body rates (from the velocity-derived attitude swinging through the bounce) acting
   through the nose wheel's 2 m lever arm — nearly cancels the +5 m/s forward translation and
   tips $V_{cx}$ negative.
2. The free-roll clamp (`WheelUnit.cpp` line 152-153) sets $\omega = \max(0, V_{cx}/r)$ — the
   wheel cannot spin backward, so $\omega = 0$.
3. The slip ratio is then $\kappa = (r\omega - V_{cx})/V_{ref} = -V_{cx}/(|V_{cx}|+\varepsilon)
   \approx +0.95$ for **any** negative $V_{cx}$, however small.
4. Pacejka reads $\kappa \approx 1$ as near-peak forward traction:
   $F_x \approx \mu F_z = 0.73 \times 32{,}885 \approx 24$ kN.

### The non-physical feedback loop

The effects close a loop that runs at the integration cadence: (i) airborne descent → α = 0
makes the attitude nose-down → the 2 m lever arm dips the nose wheel ~0.1 m below the CG;
(ii) the dipped nose registers a deep one-step "penetration" → 34 kN normal force + 24 kN
forward F_x; (iii) the impulse reverses vD and adds forward speed → the velocity-derived
attitude instantly swings nose-up → the nose lifts out of contact → back to airborne. With no
rotational inertia to set a slower timescale, the loop cycles every ~3 steps → 17.3 Hz. A 6DOF
model with real Iyy would limit the attitude rate and break the loop. **No real gear injects
22 kN of forward thrust during a taxi rollout; the entire behavior is an artifact of attaching
a long-lever-arm contact force to a zero-inertia, velocity-slaved attitude.**

### Contributing model defects, in causal order

1. **Zero-inertia, velocity-slaved attitude (root cause).** In the load-factor model pitch is a
   kinematic function of the velocity vector (θ = FPA + α, α held by the LFA). It has no
   rotational inertia, so it swings 3–5° per step (148–255 deg/s) as vD reverses each bounce —
   rates the airframe (Iyy = 1825) physically cannot achieve. This makes everything downstream
   possible; a 6DOF model with real Iyy would limit the rate and break the loop. This is the
   same trim-aero attitude property (attitude derived from the velocity vector, not integrated
   as a rigid-body state) documented in the related defect report
   [`defect_kinematic_attitude_model.md`](defect_kinematic_attitude_model.md); that report
   covers a distinct correctness bug (q_nw mis-tracking after the terrain hard constraint),
   but both stem from the velocity-derived, inertia-free attitude.
2. **Long nose-wheel lever arm.** The nose wheel is 2 m forward of the CG, so the inertia-free
   3–5° pitch swing displaces the nose contact point 0.10–0.18 m vertically each step — 14× the
   actual CG descent. Source of the deep one-step "penetration."
3. **Quasi-static strut reads geometric penetration as deflection.** The strut sets
   δ = penetration (0.077 m) and δ̇ = δ/Δt ≈ 4 m/s — a spurious closing rate that is really the
   lever-arm sweep — giving F_z = 34 kN (spring + ~10 kN damping).
4. **Slip model returns near-peak traction for any negative V_cx.** With the wheel clamped at
   ω = 0, κ ≈ +0.95 for any negative V_cx, commanding F_x ≈ μF_z = 24 kN forward — scaled by
   the inflated F_z from defect 3. The `V_ref` regularization (lines 142-156) bounds κ to
   [−1,1] and the code comment there anticipated this failure, but the fix is incomplete:
   κ = +0.95 still commands near-peak force.

The α = 0 wheelbarrowing (main gear airborne) determines *which wheel* contacts. Attitude does
**not** project the force forward — the forward force is body-frame longitudinal (defect 4) —
but attitude is central, via defects 1–2, as the driver of the deep penetration that scales it.

## Quantified data (t = 290–300 s)

| Component | Value |
| --- | --- |
| Forward spike — body-x force | +22,736 N (= F_body_x; +0.476 m/s per event) |
| Slip ratio / longitudinal force | κ = +0.946 → F_x = +24,169 N (≈ μF_z) |
| Strut normal force at spike | −34,364 N (F_body_z, 3.4 × weight) |
| Projection check | 905 N max from 1.5° pitch — 24× too small to explain the forward force |
| Strut deflection at spike | 0.077 m — driven by pitch/lever-arm, **not** descent |
| CG descent that step | −0.0016 m (vD≈0.30 → 0.006 m max; deflection is 14× larger) |
| Per-step pitch swing | 2.96° mean, 5.10° max → 148–255 deg/s (airframe cannot do this) |
| pitch vs flight-path angle | corr = 1.0000 (α = 0; attitude slaved to velocity) |
| Forward spike period | 2.411 s (0.41 Hz) |
| Backward impulse (high-freq) | −1,686 N avg, −0.0115 m/s each, 17.3 Hz |
| Backward impulses per cycle | ≈ 40 |
| Net speed change per cycle | ≈ 0 (40 × −0.0115 + 0.476 ≈ +0.02 m/s) |
| Speed at t = 300 s | 5.068 m/s (slowly-decaying limit cycle, not a hard equilibrium) |
| Aircraft airborne fraction | 65.4% |
| Theoretical gear natural frequency | 1.21 Hz (not the observed frequency) |

## Figures

![Impulse mechanism](img/oq_lg15_impulse_mechanism.png)

**Figure 1 — THE MECHANISM.** Wind-frame gear force (top) and forward speed (bottom),
t = 290–296 s. The high-frequency backward-impulse train bleeds speed down; a single +22.7 kN
forward spike every 2.41 s (red markers) resets it.

![Spike anatomy](img/oq_lg15_spike_anatomy.png)

**Figure 2 — Anatomy of one forward-spike event.** Strut deflection, vertical speed,
pitch/flight-path angle, and gear force across 12 timesteps spanning one spike. The deep
single-step penetration, the vD reversal, the nose-up pitch flip, and the +22.7 kN force all
occur on the same step.

![Forward speed 0–300 s](img/oq_lg15_forward_speed.png)

**Figure 3 — Forward speed over 300 s.** Speed at t = 300 s is 5.068 m/s vs. 5.136 m/s at
t = 90 s — a slowly-decaying limit cycle, not a hard equilibrium.

![Full timeline: struts, speed, pitch](img/oq_lg15_full_timeline.png)

**Figure 4 — Full 300 s timeline.** Main gear leaves the ground at t = 38.4 s and never
returns. Pitch oscillates through ±2°; alpha is 0° throughout.

![Wheelbarrowing transition detail](img/oq_lg15_wheelbarrow_transition.png)

**Figure 5 — Wheelbarrowing transition, t = 34–45 s.** Main gear deflection decreases from
~0.015 m to zero as speed drops through 6.3 m/s.

![Per-wheel contact patch AGL](img/oq_lg15_wheel_agl.png)

**Figure 6 — Per-wheel contact patch AGL, t = 290–300 s.** The nose wheel oscillates across
the ground line at 17.3 Hz; both main gear contact patches remain 30–50 mm above terrain.

![Per-wheel strut deflections 290–300 s](img/oq_lg15_strut_deflection.png)

**Figure 7 — Per-wheel strut deflections, t = 290–300 s.** Left and right main are zero;
nose strut maximum = 0.084 m.

![Forward and vertical speed 290–300 s](img/oq_lg15_vn_vd_coupled.png)

**Figure 8 — Forward and vertical speed, t = 290–300 s.** Contact events aligned with vertical
speed reversal.

![Contact force and energy 290–300 s](img/oq_lg15_contact_power.png)

**Figure 9 — Contact force and energy decomposition, t = 290–300 s.** The NED-frame force
logged here uses the post-step attitude and does not equal the wind-frame force the EOM applies
with the start-of-step attitude; the wind-frame force in Figures 1–2 is what moves the aircraft.

## Secondary finding — LFA dynamic-pressure washout gap

A physical FBW system generates pitch moment through control surfaces:
$M_\text{pitch} = q\,S\,\bar{c}\,C_m$. As dynamic pressure $q$ falls during rollout from
15 m/s to 5 m/s, pitch authority drops to $(5/15)^2 = 11\%$ of its approach-speed value — a
real controller would lose the ability to hold the nose-down attitude at taxi speed. The
`Aircraft` LFA commands α directly with no dynamic-pressure weighting, so it retains full
authority at 5 m/s, sustaining the α = 0 single-wheel wheelbarrowing condition. This is a model
fidelity gap that sustains (but does not by itself cause) the artifact.

## Relationship to the fix

The fix — the gear-F&M integration (a gear-load-driven body rotation-deviation state feeding
α/CL and the gear geometry, plus a lagged n_z-command relaxation) — is design content recorded
in [`landing_gear.md` — Integration Contract — `Aircraft` §2](../design/landing_gear.md).
Verification: after implementing, re-run `LandingGear_FullStop_OQ_LG15_Diagnostic` and confirm
both the +22.7 kN forward spike and the 17.3 Hz bounce vanish and the aircraft decelerates to
rest within the original 90 s.
