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

### Tyre lateral force and the ground directional stiffness

The component of the CG ground velocity perpendicular to the heading is the tyre lateral slip velocity

$$V_{\text{lat}} = V_g\,\sin\lambda,\qquad \lambda=\chi_g-\psi .$$

Each wheel $i$ develops a cornering (side) force opposing the slip, linear for small slip and saturating
at the friction circle:

$$F_{y,i} = -\,\operatorname{sat}\!\big(C_{\alpha,i}\,\lambda,\ \mu_y F_{z,i}\big),$$

with cornering stiffness $C_{\alpha,i}$ (N/rad), lateral friction $\mu_y$ (`SurfaceFrictionUniform`
lateral peak), and normal load $F_{z,i}$. These side forces produce a yaw moment about the CG through
each wheel's longitudinal arm $x_i$ (positive aft):

$$N_\text{tyre} = \sum_i F_{y,i}\,x_i .$$

For small, unsaturated slip this is a restoring moment toward $\lambda=0$ with **ground directional
stiffness**

$$k_g \;=\; \frac{\partial N_\text{tyre}}{\partial(\psi-\chi_g)}
       \;=\; \sum_i C_{\alpha,i}\,x_i \;\;\propto\; F_z .$$

Because $C_{\alpha,i}\propto F_{z,i}$, $k_g$ scales with the total contact normal force and $\to 0$ at
lift-off — this is the smooth flight↔ground handover.

### Aerodynamic weathervane stiffness

The airframe carries a side-force-versus-sideslip derivative $C_{Y_\beta}$ (`cl_y_beta`) but **no**
$C_{n_\beta}$. The weathervane yaw moment is emulated (OQ-AC-7) as the modeled side force
$F_{\text{aero},y}=C_{Y_\beta}\,\beta\,qS$ acting at the aerodynamic side-force lever $x_\text{acy}$ aft of
the CG:

$$N_\text{aero} = F_{\text{aero},y}\,x_\text{acy} = C_{Y_\beta}\,qS\,x_\text{acy}\;\beta,
\qquad q=\tfrac12\rho V_a^2 .$$

With $C_{Y_\beta}<0$ this is a **restoring** moment toward $\beta=0$ (nose into the relative wind), with
**aerodynamic weathervane stiffness**

$$k_a \;=\; \left|\frac{\partial N_\text{aero}}{\partial\beta}\right|
       \;=\; |C_{Y_\beta}|\,qS\,x_\text{acy}\;\;\propto\; q\;\propto\;V_a^2 .$$

$k_a$ grows with dynamic pressure — a strong wind / high airspeed weathervanes harder.

### Commanded directional moment

The FBW lateral channel commands a lateral load factor $n_y$ (the directional command). On the ground the
FBW realizes it through nose-wheel steering and differential braking as a commanded yaw that steers the
**ground track**. This is *not* gated by the weathervane balance — it is the pilot/autopilot directional
input, and it is the reason a commanded input still turns the aircraft on the runway. It enters as the
commanded lateral specific force $a_{y,\text{cmd}}=n_y g$ curving the ground track (below), authority-limited
by the ground minimum radius $R_\text{ground}$ (OQ-AC-6).

### Gear yaw capability

The maximum restoring yaw moment the gear can supply before the tyres/steering saturate is (OQ-AC-8)

$$N_{\text{gear,max}} = N_\text{main} + N_\text{nose},\qquad
  N_\text{main}=\!\!\sum_{\text{mains}}\!\mu_y F_{z,i}\,|x_i|,\qquad
  N_\text{nose}=\mu_y F_{z,\text{nose}}\,x_\text{nose},$$

each term $\propto F_z$ so $N_{\text{gear,max}}\to 0$ at lift-off.

---

## Heading: the Yaw-Moment Balance

The trimaero model carries no yaw inertia term, so the heading is set by the **quasi-static yaw-moment
balance** — the heading sits where the aerodynamic weathervane moment and the gear restoring moment cancel
(the commanded directional input acts on the ground track, not on this balance):

$$N_\text{aero}(\beta) + N_\text{tyre}(\lambda) = 0 .$$

Using the linear stiffnesses and $\beta=\chi_a-\psi$, $\lambda=\chi_g-\psi$ (restoring toward $\chi_a$ and
$\chi_g$ respectively):

$$k_a\,(\chi_a-\psi) + k_g\,(\chi_g-\psi) = 0
\qquad\Longrightarrow\qquad
\boxed{\ \psi = \frac{k_a\,\chi_a + k_g\,\chi_g}{k_a + k_g}\ }.$$

The heading is the **stiffness-weighted mean** of the airspeed azimuth (pulled by the aero weathervane)
and the ground-track azimuth (pulled by the tyres). Define the **gear hold fraction**

$$\boxed{\ w_\text{hold} \;\equiv\; \frac{k_g}{k_a+k_g} \;\in[0,1]\ } .$$

Then $\psi = \chi_a - w_\text{hold}\,c$ (since $\chi_a-\chi_g=c$), and from $\beta=\chi_a-\psi$:

$$\boxed{\ \beta = w_\text{hold}\,c\ },\qquad \lambda=-(1-w_\text{hold})\,c .$$

- $w_\text{hold}=0$ (no gear, $k_g=0$): $\psi=\chi_a$, $\beta=0$ — **free weathervane** (flight model).
- $w_\text{hold}=1$ ($k_g\gg k_a$): $\psi=\chi_g$, $\beta=c$ — **track held, static sideslip = crab**.

### Capability saturation

The unsaturated balance holds only while the tyre moment it demands is within the gear capability. The
moment the gear must supply to hold the heading at $\psi$ is $|N_\text{tyre}| = k_g\,|\lambda| =
k_g(1-w_\text{hold})|c|$; equivalently it must balance the aero moment $|N_\text{aero}| = k_a|\beta| =
k_a\,w_\text{hold}\,|c|$. Setting the required moment equal to the aero weathervane moment at full hold and
capping it at $N_{\text{gear,max}}$ gives the **saturated hold fraction**

$$\boxed{\ w_\text{hold} = \min\!\left(\frac{k_g}{k_a+k_g},\ \frac{N_{\text{gear,max}}}{k_a\,|c|}\right)\ }
\qquad(w_\text{hold}=1 \text{ when } c\to 0).$$

- When $k_a|c|\le N_{\text{gear,max}}$ the gear holds the heading as fully as the stiffness ratio allows.
- When $k_a|c| > N_{\text{gear,max}}$ (strong wind / high dynamic pressure, or light contact) the gear
  saturates: $w_\text{hold}=N_{\text{gear,max}}/(k_a|c|)<1$, the heading weathervanes partway toward the
  wind, and the static sideslip $\beta=w_\text{hold}\,c$ is the fraction the gear can still hold.

Both $k_g$ and $N_{\text{gear,max}}$ carry the $F_z$-scaling, so $w_\text{hold}$ transitions smoothly from
$0$ (airborne) to near $1$ (firmly on the wheels at taxi speed).

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
are mediated by the gear (through $R_\text{ground}$ and through $k_g,N_{\text{gear,max}}$ respectively).

---

## Reduction to the Flight Model (verification)

At lift-off/rotation the contact normal loads vanish, $F_{z,i}\to 0$, so **both** gear couplings vanish:

$$k_g=\sum_i C_{\alpha,i}x_i\to 0 \quad\text{and}\quad N_{\text{gear,max}}\to 0
\quad\Longrightarrow\quad w_\text{hold}=\min\!\Big(\tfrac{k_g}{k_a+k_g},\tfrac{N_{\text{gear,max}}}{k_a|c|}\Big)\to 0 .$$

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
2. $k_a=|C_{Y_\beta}|\,qS\,x_\text{acy}$; from the gear normal loads $F_{z,i}$ and geometry,
   $k_g=\sum_i C_{\alpha,i}x_i$ and $N_{\text{gear,max}}=N_\text{main}+N_\text{nose}$ (OQ-AC-8).
3. $w_\text{hold}=\min\!\big(k_g/(k_a+k_g),\ N_{\text{gear,max}}/(k_a|c|)\big)$, taking $w_\text{hold}=1$ as
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
| $x_\text{acy}$ | m | aerodynamic side-force lever aft of CG (config) |
| $C_{\alpha,i}$ | N/rad | tyre cornering stiffness of wheel $i$ ($\propto F_{z,i}$) |
| $x_i,\ x_\text{nose}$ | m | wheel longitudinal arms from CG (positive aft) |
| $\mu_y$ | — | lateral friction (`SurfaceFrictionUniform` lateral peak) |
| $F_{z,i}$ | N | wheel normal load |
| $k_a=|C_{Y_\beta}|qS\,x_\text{acy}$ | N·m/rad | aerodynamic weathervane stiffness ($\propto V_a^2$) |
| $k_g=\sum_i C_{\alpha,i}x_i$ | N·m/rad | ground directional stiffness ($\propto F_z$) |
| $N_{\text{gear,max}}$ | N·m | gear yaw capability ($\propto F_z$) |
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
