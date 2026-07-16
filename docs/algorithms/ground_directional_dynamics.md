# On-Ground Lateral–Directional Dynamics

## Overview

This document develops the lateral–directional (heading / sideslip / ground-track) model for the
velocity-slaved *trimaero* aircraft, and its extension from free flight onto the ground through the
landing-gear coupling. The model must satisfy two requirements:

1. **Reduce exactly to the flight model** as the gear contact force vanishes (at lift-off / rotation),
   with no discontinuity.
2. **Couple the airframe to the ground through the gear**: on the runway the wheels tie the heading to
   the ground track, so a crosswind holds a *static* sideslip (the aircraft does not free-weathervane),
   the commanded lateral (directional) input steers the ground track, and the aircraft weathervanes only
   when the crosswind moment exceeds the gear's yaw capability.

The result is a per-step rule for the **attitude-reference velocity** that the wind-frame quaternion
`q_nw` slaves to, plus the ground-track curvature already produced by the wind-frame lateral force. It
carries no rigid-body yaw degree of freedom (no $I_{zz}\ddot\psi$) — it stays inside the velocity-slaved
kinematic model.

This is the mathematics behind [aircraft.md §On-Ground Gear-Aero Yaw Balance](../design/aircraft.md) and
the resolutions of OQ-AC-4/AC-5/AC-7/AC-8.

---

## Reference Frames and Angles

All directional angles are **azimuths** in the horizontal (North-East) plane, measured from North,
positive toward East (a right-handed rotation about local Down). Wrap all differences to $(-\pi,\pi]$.

| Symbol | Definition | Meaning |
| --- | --- | --- |
| $\mathbf v_g$ | ground (inertial) velocity, NED | CG velocity over the ground |
| $\mathbf w$ | steady air-mass velocity (wind), NED | — |
| $\mathbf v_a = \mathbf v_g - \mathbf w$ | aerodynamic velocity (relative wind) | the flow the airframe feels |
| $V_g = \lVert\mathbf v_g\rVert$, $V_a = \lVert\mathbf v_a\rVert$ | ground speed, airspeed | horizontal magnitudes on the ground |
| $\chi_g = \operatorname{atan2}(v_{gE}, v_{gN})$ | ground-track azimuth | direction of travel |
| $\chi_a = \operatorname{atan2}(v_{aE}, v_{aN})$ | aero-velocity azimuth | airspeed direction |
| $\psi$ | heading | nose (body-$x$) azimuth |
| $c \equiv \chi_a - \chi_g$ | **crab** | wind-triangle azimuth offset |
| $\beta \equiv \chi_a - \psi$ | **sideslip** | airspeed azimuth minus heading |
| $\lambda \equiv \chi_g - \psi$ | **tyre slip** | ground-track azimuth minus heading |

These four azimuths satisfy one identity:

$$\boxed{\ \beta = c + \lambda\ }\qquad(\chi_a-\psi)=(\chi_a-\chi_g)+(\chi_g-\psi).$$

Two limiting orientations of the heading anchor the whole model:

- **Weathervaned** ($\psi=\chi_a$): $\beta=0$, $\lambda=-c$. The nose points along the relative wind. The
  airframe feels no sideslip; the tyres skid at the full crab angle.
- **Track-held** ($\psi=\chi_g$): $\lambda=0$, $\beta=c$. The nose points along the ground track (wheels
  rolling straight). The airframe carries a static sideslip equal to the crab.

The whole on-ground problem is *where between these two the heading sits*, and how fast the ground track
itself curves.

---

## Flight-Model Baseline (the reduction target)

In free flight the attitude is velocity-slaved to the **aerodynamic** velocity (OQ-AC-4): `q_nw` tracks
$\mathbf v_a$, so

$$\psi = \chi_a \quad\Longrightarrow\quad \beta = 0 \quad(\text{coordinated flight}).$$

The commanded lateral load factor $n_y$ produces a wind-frame lateral specific force $a_y = n_y\,g$, which
curves the aerodynamic velocity vector at the turn rate

$$\dot\chi_a = \frac{a_y}{V_a} = \frac{n_y\,g}{V_a},
\qquad\text{bounded by}\qquad |\dot\chi_a|\le \frac{V_a}{R_\text{flight}}
\ \ \Longleftrightarrow\ \ |\kappa|\le \frac1{R_\text{flight}},$$

the path-curvature authority limit (OQ-AC-2 slew saturation, OQ-AC-6 $n_y$ authority), with
$R_\text{flight}=$ `qnw_min_turn_radius_m`. With a steady wind $\dot{\mathbf w}=0$ so $\dot\chi_g=\dot\chi_a$
and the crab $c$ is constant. The heading is slaved: $\dot\psi=\dot\chi_a$.

**Any on-ground model must return this exactly as the gear force $\to 0$.** The reduction is verified in
§Reduction to the Flight Model.

---

## On-Ground Forces and Moments

On the runway the wheels roll along the heading (nose-steering handled below). Three directional effects
act on the heading, and the same forces curve the ground track.

### FBW-enforced heading hold and its steering authority

On the ground the heading is held by the **flight control law actively enforcing the commanded lateral
load factor $n_y$**, not by a passive tyre stiffness. When the FBW commands $n_y$ (with $n_y=0$ the
straight-rollout case) it applies a **yaw control moment** to the airframe to null the lateral/yaw
disturbance, up to an estimated steering authority. This moment is a control input applied to the airframe
— it is **not** produced through the gear friction forces (not differential braking, not tyre side
forces) and **not** through a modeled nose-wheel deflection. It is the emulated moment-producing
capability of the on-ground directional control (OQ-AC-5): its magnitude is *estimated* and *scaled with
contact*, rather than derived from a friction mechanism.

The authority is estimated as a moment-arm coefficient times the contact normal load:

$$N_{\text{steer,max}} \;=\; c_\text{steer}\,F_{z,\text{contact}},
\qquad F_{z,\text{contact}}=\sum_i F_{z,i}\;\;\propto\;\text{weight on wheels},$$

with $c_\text{steer}$ (m) the effective steering-authority arm (config parameter). Because it scales with
the contact normal load, $N_{\text{steer,max}}\to 0$ at lift-off — the smooth flight↔ground handover — and
grows in proportion to how firmly the aircraft is on its wheels. It is deliberately not tied to a specific
effector; the trimaero model emulates the closed-loop directional-control capability, not the linkage that
produces it.

### Aerodynamic weathervane stiffness

The airframe carries a side-force-versus-sideslip derivative $C_{Y_\beta}$ (`cl_y_beta`) but **no**
$C_{n_\beta}$. The weathervane yaw moment is emulated (OQ-AC-7) as the modeled side force
$F_{\text{aero},y}=C_{Y_\beta}\,\beta\,qS$ acting at the aerodynamic side-force lever $x_\text{acy}$ aft of
the CG, $N_\text{aero}=C_{Y_\beta}\,qS\,x_\text{acy}\,\beta$ with $q=\tfrac12\rho V_a^2$. Its magnitude per
unit sideslip is the **aerodynamic weathervane stiffness**

$$k_a \;=\; |C_{Y_\beta}|\,qS\,x_\text{acy}\;\;\propto\; q\;\propto\;V_a^2 ,$$

so a strong wind / high airspeed weathervanes harder. The moment that would fully weathervane the heading
(from $\beta=c$ down to $\beta=0$) is $k_a\,|c|$.

### Commanded directional input

The same lateral channel commands $n_y$. On the ground the FBW realizes a **nonzero** $n_y$ as a commanded
yaw that steers the **ground track** $\chi_g$ (through the wind-frame lateral force, below,
authority-limited by the ground minimum radius $R_\text{ground}$, OQ-AC-6). A sustained $n_y$ keeps curving
the track — a commanded ground loop. The heading-hold here is the $n_y=0$ (straight) enforcement; a
commanded steer and the crosswind hold draw on the same steering-moment budget $N_{\text{steer,max}}$.

---

## Heading: FBW Authority Against the Weathervane

The trimaero model carries no yaw inertia term; the heading is set by the FBW enforcing $n_y$ against the
aerodynamic weathervane, up to the steering-moment authority. A fully weathervaned heading
($\psi=\chi_a$, $\beta=0$) is opposed by the FBW holding it toward the ground track ($\psi=\chi_g$,
$\beta=c$); the fraction it can hold is the moment authority over the demand — the **gear hold fraction**

$$\boxed{\ w_\text{hold} \;=\; \min\!\left(1,\ \frac{N_{\text{steer,max}}}{k_a\,|c|}\right)\in[0,1]\ }
\qquad (w_\text{hold}=1 \text{ when } c\to 0),$$

with heading $\psi=\chi_a-w_\text{hold}\,c$ and sideslip

$$\boxed{\ \beta = w_\text{hold}\,c\ },\qquad \lambda=-(1-w_\text{hold})\,c .$$

- $N_{\text{steer,max}}\ge k_a|c|$ (normal contact, moderate wind): $w_\text{hold}=1$ — the FBW **fully
  holds** the heading to the ground track with a static $\beta=c$ (the crab). This is the firm hold a
  wheeled aircraft has on the runway.
- $N_{\text{steer,max}} < k_a|c|$ (strong wind / high dynamic pressure, or light contact near
  touchdown/lift-off): $w_\text{hold}=N_{\text{steer,max}}/(k_a|c|)<1$ — the FBW authority saturates and the
  heading weathervanes partway toward the wind on the excess.
- $N_{\text{steer,max}}=0$ (airborne): $w_\text{hold}=0$ — free weathervane (the flight model).

There is **no passive-stiffness term**: the hold is a true authority limit (a saturating FBW loop against a
finite, contact-scaled steering moment), not a soft spring. $N_{\text{steer,max}}\propto F_z$ carries the
transition smoothly through touchdown/lift-off.


---

## Ground Track: Path-Curvature Dynamics

The heading balance above sets the nose orientation; the **ground track** $\chi_g$ curves under the net
horizontal force on the CG, exactly as the aero velocity does in flight. In the wind frame the lateral
specific force is

$$a_y \;=\; \underbrace{n_y\,g}_{\text{commanded}} \;+\; \underbrace{\frac{F_{\text{aero},y}}{m}}_{\substack{\text{aero side force}\\ \text{from }\beta}} \;+\; \underbrace{\frac{1}{m}\sum_i F_{y,i}}_{\text{tyre reaction}} ,$$

and the ground track curves at $\dot\chi_g = a_y/V_g$, authority-limited to the applicable curvature

$$|\dot\chi_g|\le \frac{V_g}{R},\qquad
\frac1R = \frac{1-w_\text{az}}{R_\text{flight}} + \frac{w_\text{az}}{R_\text{ground}},$$

the flight↔ground blended minimum radius of OQ-AC-6 (blend weight $w_\text{az}$ = WoW-load fraction ×
below-stall smoothstep). A **sustained** commanded $n_y$ therefore keeps curving the ground track — a
commanded ground loop, as intended — while the tyre reaction curves the track toward the heading and the
crosswind-driven aero side force ($\beta=w_\text{hold}c\neq 0$ once on the ground) is what the tyres resist.

The two channels are cleanly separated: **the commanded directional input ($n_y$) steers the ground track
$\chi_g$; the crosswind weathervane balance ($w_\text{hold}$) sets the heading $\psi$ relative to it.** Both
are mediated by the gear (through $R_\text{ground}$ and through the contact-scaled steering authority
$N_{\text{steer,max}}$ respectively).

---

## Reduction to the Flight Model (verification)

At lift-off/rotation the contact normal load vanishes, $F_{z,\text{contact}}\to 0$, so the steering
authority vanishes:

$$N_{\text{steer,max}}=c_\text{steer}\,F_{z,\text{contact}}\to 0
\quad\Longrightarrow\quad w_\text{hold}=\min\!\Big(1,\tfrac{N_{\text{steer,max}}}{k_a|c|}\Big)\to 0 .$$

Then

$$\psi=\chi_a-w_\text{hold}c\to\chi_a,\qquad \beta=w_\text{hold}c\to 0,$$

i.e. the attitude re-slaves to the aerodynamic velocity with zero sideslip — the coordinated flight model.
Simultaneously the OQ-AC-6 blended radius returns $1/R\to 1/R_\text{flight}$ (the WoW weight $w_\text{az}\to
0$), so the commanded-$n_y$ path curvature returns to the flight authority. The transition is continuous
because every gear term is proportional to $F_z$, which is itself continuous through touchdown/lift-off.
There is no regime switch and no rigid-body yaw DOF is introduced.

---

## Discrete Realization

The model is realized through the **attitude-reference velocity** that `commitAttitude` slaves `q_nw` to.
Writing the reference as the blend whose azimuth is $\psi=\chi_a-w_\text{hold}c$:

$$\boxed{\ \mathbf v_\text{att,ref} \;=\; \mathbf v_g \;-\; (1-w_\text{hold})\,\mathbf w\ }
\qquad\big(=\mathbf v_a \text{ when } w_\text{hold}=0;\ =\mathbf v_g \text{ when } w_\text{hold}=1\big).$$

Its horizontal azimuth interpolates $\chi_a$ (airspeed) at $w_\text{hold}=0$ to $\chi_g$ (ground track) at
$w_\text{hold}=1$, reproducing $\psi=\chi_a-w_\text{hold}c$ to first order and exactly at the endpoints.
This is the sole change from the OQ-AC-4 reference $\mathbf v_\text{att,ref}=\mathbf v_a=\mathbf v_g-\mathbf
w$: the wind is added back in proportion to the gear hold fraction. `stepQnw` then tracks this reference
strictly under the OQ-AC-2 slew saturation, so the strict `q_nw.x = v̂_ref` invariant is preserved and the
attitude rate stays bounded as $V\to 0$. Per step:

1. From the current state form $\mathbf v_g$, $\mathbf v_a=\mathbf v_g-\mathbf w$, $\chi_g$, $\chi_a$,
   $c=\chi_a-\chi_g$, $q=\tfrac12\rho V_a^2$.
2. $k_a=|C_{Y_\beta}|\,qS\,x_\text{acy}$; from the total contact normal load $F_{z,\text{contact}}$ the
   steering authority $N_{\text{steer,max}}=c_\text{steer}\,F_{z,\text{contact}}$ (OQ-AC-8).
3. $w_\text{hold}=\min\!\big(1,\ N_{\text{steer,max}}/(k_a|c|)\big)$, taking $w_\text{hold}=1$ as
   $|c|\to0$.
4. $\mathbf v_\text{att,ref}=\mathbf v_g-(1-w_\text{hold})\mathbf w$; pass to `commitAttitude`.

The commanded-$n_y$ ground-track curvature and its $R_\text{ground}$ authority limit are unchanged
(OQ-AC-6); the balance here only conditions the heading. The sideslip reported by the model is then
$\beta=w_\text{hold}\,c$, and the aero side force $F_{\text{aero},y}=C_{Y_\beta}\beta qS$ it produces feeds
the wind-frame $a_y$ as usual — now with a physically nonzero, gear-held $\beta$ on the ground rather than
the identically-zero $\beta$ of the airborne coordinated model.

---

## Symbol Reference

| Symbol | Units | Meaning |
| --- | --- | --- |
| $\psi,\ \chi_g,\ \chi_a$ | rad | heading, ground-track, aero-velocity azimuths |
| $\beta,\ \lambda,\ c$ | rad | sideslip $\chi_a-\psi$, tyre slip $\chi_g-\psi$, crab $\chi_a-\chi_g$ |
| $V_g,\ V_a$ | m/s | ground speed, airspeed (horizontal) |
| $q=\tfrac12\rho V_a^2$ | Pa | dynamic pressure |
| $C_{Y_\beta}$ | 1/rad | side-force-vs-sideslip derivative (`cl_y_beta`, $<0$) |
| $x_\text{acy}$ | m | aerodynamic side-force lever aft of CG (config `x_acy_m`) |
| $c_\text{steer}$ | m | FBW steering-authority arm (config `steering_authority_m`) |
| $F_{z,\text{contact}}$ | N | total contact normal load (weight on wheels) |
| $k_a=|C_{Y_\beta}|qS\,x_\text{acy}$ | N·m/rad | aerodynamic weathervane stiffness ($\propto V_a^2$) |
| $N_{\text{steer,max}}=c_\text{steer}F_{z,\text{contact}}$ | N·m | FBW steering-moment authority ($\propto F_z$) |
| $w_\text{hold}$ | — | gear hold fraction $\in[0,1]$ |
| $R_\text{flight},\ R_\text{ground}$ | m | flight / ground minimum turn radii (OQ-AC-6) |

---

## References

- [aircraft.md §On-Ground Gear-Aero Yaw Balance](../design/aircraft.md) — design authority and
  configuration (`x_acy`), and the OQ-AC-4/AC-5/AC-7/AC-8 resolutions.
- [aircraft.md §Aerodynamic Wind Frame and Crab](../design/aircraft.md) — the aero-velocity slaving and
  crab construction this model extends.
- [aircraft.md §Velocity-Slaved Attitude and Low-Speed Slew Saturation](../design/aircraft.md) — the
  `commitAttitude` reference-slew saturation the discrete realization tracks under.
- [equations_of_motion.md](equations_of_motion.md) — the point-mass kinematic integrator and wind-frame
  force decomposition.
- [load_factor_allocation.md](load_factor_allocation.md) — the $n_y\!\to\!\beta$ side-force solve.
