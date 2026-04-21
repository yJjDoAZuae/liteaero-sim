# Equations of Motion

## Overview

LiteAero Sim uses a **point-mass kinematic model** driven by externally computed accelerations. The aerodynamics, propulsion, and atmosphere subsystems compute the net acceleration in the Wind frame; the kinematic integrator propagates position, velocity, and attitude. This separation keeps each subsystem testable in isolation.

```mermaid
flowchart LR
    ATM["Atmosphere<br/>ρ, a, p, T"] --> AERO
    PROP["Propulsion<br/>Thrust"] --> AERO
    AERO["Aerodynamics<br/>Forces → acceleration"] --> KIN
    GUS["Gust / Wind<br/>v_wind"] --> KIN
    KIN["KinematicState<br/>Integrator"] --> OUT["Position<br/>Velocity<br/>Attitude"]
```

---

## Reference Frames

### Frame Definitions

| Frame | Symbol | Origin / Orientation |
| --- | --- | --- |
| NED | $N$ | North-East-Down, fixed to Earth surface at datum point |
| Body | $B$ | Origin at aircraft CG; $x$ forward, $y$ right, $z$ down |
| Wind | $W$ | $x$ along airmass-relative velocity, $z$ in body symmetry plane pointing down |
| Stability | $S$ | $x$ in symmetry plane along velocity vector projection, $z$ down |
| Local Level | $L$ | Horizontal plane tangent to Earth at current position |
| Velocity | $V$ | $x$ along ground-relative velocity; $y$ horizontal, perpendicular to velocity; $z$ in vertical velocity plane, positive downward |
| Velocity-Wind | $VW$ | Same as $V$ using airmass-relative velocity $\mathbf{v}_{air} = \mathbf{v}_N - \mathbf{v}_{wind}$ |
| Plane of Motion | $P$ | Frenet frame of the airmass velocity path: $x$ along velocity, $y$ toward center of curvature ($\hat{y}_P \cdot \mathbf{a} > 0$), $z$ binormal ($\hat{z}_P = \hat{x}_P \times \hat{y}_P$, direction of $\mathbf{v} \times \mathbf{a}$) |

### Frame Relationships

```mermaid
flowchart LR
    N["NED (N)"] -->|"q_nb"| B["Body (B)"]
    N -->|"q_nw"| W["Wind (W)"]
    N -->|"q_ns"| S["Stability (S)"]
    N -->|"q_nl"| L["Local Level (L)"]
    N -->|"q_nv"| V["Velocity (V)"]
    N -->|"q_nvw"| VW["Velocity-Wind (VW)"]
```

All rotations are quaternions $q_{XY}$ mapping vectors from frame $Y$ to frame $X$:

$$\mathbf{v}_X = q_{XY} \otimes \mathbf{v}_Y \otimes q_{XY}^*$$

Where matrix arithmetic is more convenient, $C_{XY}$ denotes the **Direction Cosine Matrix** (DCM) — the coordinate transformation matrix — corresponding to the same rotation:

$$\mathbf{v}_X = C_{XY}\,\mathbf{v}_Y$$

The DCM and the quaternion are two algebraic realizations of the same underlying geometric rotation. The choice between them is one of computational convenience; neither is "the rotation" itself.

---

## Velocity and Velocity-Wind Frames

### Velocity Frame (V)

The Velocity frame $V$ has its $x$-axis along the ground-relative velocity vector $\mathbf{v}_N$ and its $y$-axis confined to the local horizontal plane. Starting from NED, it is reached by yawing to the track angle $\chi$ then pitching by $-\gamma$:

$$
C_{VN} = C_y(-\gamma)\,C_z(\chi)
$$

| Axis | Direction |
| --- | --- |
| $\hat{x}_V$ | Unit ground-relative velocity vector |
| $\hat{y}_V$ | Horizontal, perpendicular to velocity, positive to the right of the velocity heading |
| $\hat{z}_V$ | In the vertical plane containing $\hat{x}_V$; perpendicular to $\hat{x}_V$; positive downward |

where $\chi = \arctan(v_E / v_N)$ is the track angle and $\gamma = -\arcsin(v_D / V_G)$ is the flight path angle ($V_G = \|\mathbf{v}_N\|$).

### Velocity-Wind Frame (VW)

The Velocity-Wind frame $VW$ has identical axis geometry to $V$ but is constructed from the airmass-relative velocity:

$$
\mathbf{v}_{air} = \mathbf{v}_N - \mathbf{v}_{wind}
$$

where $\mathbf{v}_{wind}$ is the wind transport velocity expressed in NED. The DCM from NED to $VW$ is:

$$
C_{VW,N} = C_y(-\gamma_a)\,C_z(\chi_a)
$$

with $\chi_a$, $\gamma_a$ the track and flight path angles of $\mathbf{v}_{air}$. When wind is zero, $V = VW$.

### Relationship to Wind Frame W

The Wind frame $W$ and $VW$ share the same $x$-axis (airmass-relative velocity vector). They differ by a roll about that axis by the **wind-axis bank angle** $\mu$:

$$
C_{VW,W} = C_x(\mu)
$$

For coordinated, wings-level flight $\mu = 0$ and the frames coincide.

---

## Aerodynamic Angles

### Angle of Attack $\alpha$

The angle of attack is measured in the body symmetry plane between the velocity vector projection and the body $x$-axis:

$$
\alpha = \arctan\!\left(\frac{v_{B,z}}{v_{B,x}}\right)
$$

### Sideslip Angle $\beta$

The sideslip is the angle between the velocity vector and the body symmetry plane:

$$
\beta = \arcsin\!\left(\frac{v_{B,y}}{\|\mathbf{v}_B\|}\right)
$$

### Wind-to-Body Rotation

The rotation from Wind frame to Body frame is defined by $\alpha$ and $\beta$. Its DCM is the product of two elementary rotation matrices:

$$
C_{BW} = C_y(\alpha)\, C_z(-\beta)
$$

The Wind-to-NED quaternion follows from the Body-to-NED quaternion and the Wind-to-Body relationship:

$$
q_{nw} = q_{nb} \otimes q_{bw}
$$

---

## Attitude Kinematics — Quaternion ODE

Attitude is represented as the unit quaternion $q_{nb}$ (Body-to-NED). The quaternion evolves according to:

$$
\dot{q}_{nb} = \frac{1}{2}\, q_{nb} \otimes \boldsymbol{\omega}_{B/N,\times}^B
$$

where $\boldsymbol{\omega}_{B/N}^B = [p,\ q,\ r]^T$ is the angular velocity of the Body frame relative to NED, expressed in Body frame coordinates (rad/s), and $\boldsymbol{\omega}_{B/N,\times}^B$ denotes the pure quaternion $[0,\ p,\ q,\ r]^T$.

Discretized with forward Euler at timestep $\Delta t$:

$$
q_{nb,k+1} = q_{nb,k} + \frac{\Delta t}{2}\, q_{nb,k} \otimes [0,\ p_k,\ q_k,\ r_k]^T
$$

The result is renormalized to unit length after each step to prevent drift:

$$
q_{nb,k+1} \leftarrow \frac{q_{nb,k+1}}{\|q_{nb,k+1}\|}
$$

---

## Velocity Kinematics — Wind Frame Input

The kinematic model accepts net acceleration expressed in the Wind frame $\mathbf{a}_W$ (output of the aerodynamics and propulsion subsystems). It is rotated to NED before integration. Using the DCM $C_{NW}$ or equivalently the quaternion $q_{nw}$:

$$
\mathbf{a}_N = C_{NW}\, \mathbf{a}_W = q_{nw} \otimes \mathbf{a}_W \otimes q_{nw}^*
$$

Velocity and position are jointly integrated with classical fourth-order Runge-Kutta
(RK4) over the six-component state $(\varphi, \lambda, h, v_N, v_E, v_D)$. Because the
input acceleration is held constant throughout each timestep, all four RK4 slope
evaluations for velocity are identical, and the velocity update reduces to:

$$
\mathbf{v}_{N,k+1} = \mathbf{v}_{N,k} + \Delta t\, \mathbf{a}_{N,k}
$$

The four-point evaluation provides full fourth-order accuracy for position, where the
WGS84 geodetic rate functions are nonlinear in velocity (see
[Position Kinematics](#position-kinematics--wgs84-integration)).

---

## Position Kinematics — WGS84 Integration

Position is stored as a WGS84 geodetic datum $(\varphi, \lambda, h)$ (latitude, longitude, altitude). The NED velocity is converted to geodetic rates using the Earth radii of curvature.

### Radii of Curvature

The WGS84 ellipsoid has semi-major axis $a = 6{,}378{,}137.0\,\text{m}$ and flattening $f = 1/298.257223563$. The first eccentricity squared is $e^2 = 2f - f^2$.

The **meridional radius** (north–south):

$$
M(\varphi) = \frac{a(1 - e^2)}{\left(1 - e^2 \sin^2\varphi\right)^{3/2}}
$$

The **normal radius** (east–west):

$$
N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2\varphi}}
$$

### Geodetic Rate Equations

$$
\dot{\varphi} = \frac{v_N}{M(\varphi) + h}, \qquad
\dot{\lambda} = \frac{v_E}{\bigl(N(\varphi) + h\bigr)\cos\varphi}, \qquad
\dot{h} = -v_D
$$

where $v_N$, $v_E$, $v_D$ are the NED velocity components.

Integrated jointly with velocity via RK4. The four slope evaluations sample
$\dot\varphi(\mathbf{v})$, $\dot\lambda(\mathbf{v})$, $\dot h(\mathbf{v})$ at four
intermediate velocity values — current, two half-step midpoints, and end-of-step — giving
fourth-order accuracy in the nonlinear geodetic rate functions:

$$
\varphi_{k+1} = \varphi_k + \Delta t\,\frac{\dot\varphi_1 + 2\dot\varphi_2 + 2\dot\varphi_3 + \dot\varphi_4}{6}
$$

and analogously for $\lambda$ and $h$.

---

## Euler Angles (3-2-1 Convention)

Euler angles $(\phi,\, \theta,\, \psi)$ — roll, pitch, heading — parameterize the 3-2-1 (ZYX) rotation sequence from Body to NED. The DCM of this rotation factors as:

$$
C_{NB} = C_z(\psi)\, C_y(\theta)\, C_x(\phi)
$$

From the quaternion $q = [w,\ x,\ y,\ z]^T$:

$$
\phi   = \arctan\!\left(\frac{2(wx + yz)}{1 - 2(x^2 + y^2)}\right)
$$

$$
\theta = \arcsin\!\bigl(2(wy - zx)\bigr)
$$

$$
\psi   = \arctan\!\left(\frac{2(wz + xy)}{1 - 2(y^2 + z^2)}\right)
$$

---

## Body Rate–Euler Rate Kinematics

The relationship between body angular rates $[p,\, q,\, r]^T$ and Euler angle rates $[\dot\phi,\, \dot\theta,\, \dot\psi]^T$ is:

$$
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
=
\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix}
$$

Inverted (for extracting Euler rates from body rates):

$$
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix}
=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi           & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

This expression is singular at $\theta = \pm 90°$ (gimbal lock). The quaternion ODE is used for attitude propagation; Euler angles are extracted for output and control only.

---

## Body Rates from Wind Frame Rates

The Body frame attitude is determined by applying $\alpha$ and $\beta$ to the Wind frame — the Wind frame orientation is established first (from the velocity vector and wind), and the Body frame follows from the aerodynamic angles. Body angular rates $[p,\,q,\,r]^T$ are therefore derived from the Wind frame angular velocity $[p_W,\,q_W,\,r_W]^T$ and the rates of the aerodynamic angles.

### Direction Cosine Matrix: Wind to Body

The rotation from Wind frame to Body frame is composed of two elementary rotations: a yaw by $-\beta$ about $\hat{z}_W$, followed by a pitch by $\alpha$ about the intermediate $y$-axis. Its DCM — which transforms component vectors from Wind to Body coordinates — is:

$$
C_{BW} = C_y(\alpha)\,C_z(-\beta)
$$

where $C_y(\cdot)$ and $C_z(\cdot)$ denote the DCMs of elementary rotations about the respective axes. Expanding the product:

$$
C_{BW} =
\begin{bmatrix}
\cos\alpha\cos\beta & -\cos\alpha\sin\beta & -\sin\alpha \\
\sin\beta           &  \cos\beta           &  0          \\
\sin\alpha\cos\beta & -\sin\alpha\sin\beta &  \cos\alpha
\end{bmatrix}
$$

Verification: the velocity vector $\mathbf{v}_W = V[1,0,0]^T$ maps to $C_{BW}\,\mathbf{v}_W = V[\cos\alpha\cos\beta,\ \sin\beta,\ \sin\alpha\cos\beta]^T$, recovering the standard definitions of $\alpha$ and $\beta$.

### Angular Velocity Decomposition

The angular velocity of the Body frame relative to NED equals the angular velocity of the Wind frame relative to NED plus the angular velocity of Body relative to Wind, with all terms expressed in Body frame coordinates:

$$
\boldsymbol{\omega}_{B/N}^B = C_{BW}\,\boldsymbol{\omega}_{W/N}^W + \boldsymbol{\omega}_{B/W}^B
$$

where $\boldsymbol{\omega}_{W/N}^W = [p_W,\,q_W,\,r_W]^T$ is the Wind frame angular velocity relative to NED, expressed in Wind frame coordinates.

### Contribution from Aerodynamic Angle Rates

Because the rotation from Wind to Body is parameterized by $\alpha$ and $\beta$, its DCM $C_{BW}$ changes whenever $\alpha$ or $\beta$ change, contributing angular velocity:

- $C_z(-\beta)$: intermediate frame S rotates relative to Wind about $\hat{z}_W$ at rate $-\dot\beta$. Expressed in Body frame: $C_y(\alpha)\,[0,\,0,\,-\dot\beta]^T$.
- $C_y(\alpha)$: Body rotates relative to S about $\hat{y}_B$ at rate $+\dot\alpha$. Expressed in Body frame: $[0,\,\dot\alpha,\,0]^T$.

$$
\boldsymbol{\omega}_{B/W}^B
= C_y(\alpha)\begin{bmatrix}0\\0\\-\dot\beta\end{bmatrix} + \begin{bmatrix}0\\\dot\alpha\\0\end{bmatrix}
= \begin{bmatrix}\dot\beta\sin\alpha\\\dot\alpha\\-\dot\beta\cos\alpha\end{bmatrix}
$$

### Body Rate Equations

Combining the Wind frame contribution and the aerodynamic angle rate contribution:

$$
\boxed{
\begin{bmatrix}p\\q\\r\end{bmatrix}
=
\begin{bmatrix}
\cos\alpha\cos\beta & -\cos\alpha\sin\beta & -\sin\alpha \\
\sin\beta           &  \cos\beta           &  0 \\
\sin\alpha\cos\beta & -\sin\alpha\sin\beta &  \cos\alpha
\end{bmatrix}
\begin{bmatrix}p_W\\q_W\\r_W\end{bmatrix}
+
\begin{bmatrix}\dot\beta\sin\alpha\\\dot\alpha\\-\dot\beta\cos\alpha\end{bmatrix}
}
$$

Expanded:

$$
p = p_W\cos\alpha\cos\beta - q_W\cos\alpha\sin\beta - r_W\sin\alpha + \dot\beta\sin\alpha
$$

$$
q = p_W\sin\beta + q_W\cos\beta + \dot\alpha
$$

$$
r = p_W\sin\alpha\cos\beta - q_W\sin\alpha\sin\beta + r_W\cos\alpha - \dot\beta\cos\alpha
$$

### Wind Frame Angular Velocity Components

| Component | Description | Source |
| --- | --- | --- |
| $p_W$ | Roll rate about the velocity vector | Input to `KinematicState::step()` |
| $q_W$ | Pitch rate of the velocity vector (rate of change of flight path angle $\gamma_a$) | Derived from the Plane of Motion frame — see below |
| $r_W$ | Yaw rate of the velocity vector (horizontal curvature of the airmass-relative path) | Derived from the Plane of Motion frame — see below |

$p_W$ is a direct control input. $q_W$ and $r_W$ are computed from the centripetal acceleration decomposed in the VW frame; the derivation is given in [Plane of Motion Frame](#plane-of-motion-frame) below.

### Path Angle Rates

The VW frame angular velocity $\boldsymbol{\omega}_{VW/N}^{VW}$ can be related to the time derivatives of the aerodynamic path angles by decomposing the DCM sequence $C_{VW,N} = C_y(-\gamma_a)\,C_z(\chi_a)$:

$$
\boldsymbol{\omega}_{VW/N}^{VW}
= \begin{bmatrix}p_W \\ q_W \\ r_W\end{bmatrix}
= \begin{bmatrix}-\dot\chi_a\sin\gamma_a + \dot\mu \\ \dot\gamma_a \\ \dot\chi_a\cos\gamma_a\end{bmatrix}
$$

Inverted:

$$
\dot\gamma_a = q_W, \qquad
\dot\chi_a  = \frac{r_W}{\cos\gamma_a}, \qquad
\dot\mu     = p_W + r_W\tan\gamma_a
$$

For coordinated wings-level flight ($\mu = 0$, $\dot\mu = 0$): $q_W$ drives flight path angle change, $r_W$ drives track angle change, and $p_W = -r_W\tan\gamma_a$ (a precession term that is nonzero during climbing turns as the velocity roll axis sweeps in azimuth).

---

## Plane of Motion Frame

The Plane of Motion (POM) frame $P$ is the Frenet frame of the airmass-relative velocity path. It is an ephemeral, non-stored frame computed at each step from the instantaneous airmass velocity $\mathbf{v}$ (speed $V = |\mathbf{v}|$) and net acceleration $\mathbf{a}$.

### Frame Definition

**1. Tangent** (first axis, along velocity):

$$
\hat{x}_P = \frac{\mathbf{v}}{V}
$$

**2. Centripetal acceleration** (component of $\mathbf{a}$ perpendicular to velocity):

$$
\mathbf{a}_\perp = \mathbf{a} - (\mathbf{a} \cdot \hat{x}_P)\,\hat{x}_P
$$

**3. Normal** (second axis, toward center of curvature):

$$
\hat{y}_P = \frac{\mathbf{a}_\perp}{|\mathbf{a}_\perp|}
$$

**4. Binormal** (third axis, in direction of $\mathbf{v} \times \mathbf{a}$):

$$
\hat{z}_P = \hat{x}_P \times \hat{y}_P
$$

The velocity vector rotates entirely within the $\hat{x}_P$–$\hat{y}_P$ (tangent–normal) plane; $\hat{z}_P$ is perpendicular to the plane of motion.

### Wind Frame Turning Rates

The velocity direction changes at the curvature rate $|\mathbf{a}_\perp| / V$ (rotation about the binormal $\hat{z}_P$). To obtain $q_W$ and $r_W$, express $\mathbf{a}_\perp$ in the VW frame. Since $\mathbf{a}_\perp \perp \hat{x}_{VW}$ by construction, only the $y$- and $z$-components are nonzero:

$$
\mathbf{a}_\perp^{VW} = \begin{bmatrix} 0 \\ a_{\perp,y} \\ a_{\perp,z} \end{bmatrix}
$$

where $a_{\perp,y}$ is the horizontal component (positive rightward) and $a_{\perp,z}$ is the in-plane vertical component (positive downward in the velocity plane). The turning rates follow from $\boldsymbol{\omega}_{W/N}^{VW} \times \hat{x}_{VW} = \mathbf{a}_\perp^{VW}/V$:

$$
q_W = -\frac{a_{\perp,z}}{V}, \qquad r_W = \frac{a_{\perp,y}}{V}
$$

**Sign convention**: $q_W > 0$ when the velocity vector pitches upward (flight path angle $\gamma_a$ increases); $r_W > 0$ when the velocity vector yaws rightward.

### POM from Aerodynamic Forces

Given lift $\mathbf{L}$ (perpendicular to $\mathbf{v}$), drag $\mathbf{D}$ (opposing $\mathbf{v}$), thrust $\mathbf{T}$, and gravity $m\mathbf{g}$ (in NED):

$$
\mathbf{a} = \frac{\mathbf{L} + \mathbf{D} + \mathbf{T}}{m} + \mathbf{g}
$$

The tangential acceleration (changes speed, does not curve the path):

$$
a_\parallel = \mathbf{a} \cdot \hat{x}_P = \frac{T - D}{m} - g\sin\gamma_a
$$

The centripetal acceleration (defines the POM orientation):

$$
\mathbf{a}_\perp = \mathbf{a} - a_\parallel\,\hat{x}_P = \frac{\mathbf{L}}{m} + \mathbf{g}_\perp
$$

where $\mathbf{g}_\perp = \mathbf{g} - (\mathbf{g} \cdot \hat{x}_P)\hat{x}_P$ is the component of gravity perpendicular to the velocity vector. The total acceleration $\mathbf{a}$ lies in the POM by construction — the POM is defined as the plane spanned by $\hat{x}_P$ and $\hat{a}$. Individual force components (lift, thrust, gravity) need not each lie in the POM; it is their vector sum that determines the normal direction $\hat{y}_P$ and therefore the binormal $\hat{z}_P$.

---

## Aerodynamic Angle Rates

The angles $\alpha$ and $\beta$, together with their rates $\dot\alpha$ and $\dot\beta$, are computed by the Aerodynamics subsystem and passed as inputs to `KinematicState::step()`. They are derived from aerodynamic forces using first-order (linear) aerodynamic models.

### Angle of Attack Rate $\dot\alpha$

In the linear aerodynamic model, lift is proportional to $\alpha$:

$$
L = q_\infty S C_{L_\alpha}\,\alpha, \qquad q_\infty = \tfrac{1}{2}\rho V^2
$$

where $S$ is the reference area and $C_{L_\alpha}$ is the lift-curve slope. The instantaneous angle of attack:

$$
\alpha = \frac{L}{q_\infty S C_{L_\alpha}}
$$

Differentiating and applying the quasi-steady approximation ($\dot{q}_\infty \ll q_\infty / \Delta t$ over one timestep):

$$
\dot\alpha \approx \frac{\dot L}{q_\infty S C_{L_\alpha}}
$$

### Sideslip Rate $\dot\beta$

By the same argument applied to the lateral aerodynamic force (side force) $Y$:

$$
Y = q_\infty S C_{Y_\beta}\,\beta
$$

$$
\beta = \frac{Y}{q_\infty S C_{Y_\beta}}, \qquad \dot\beta \approx \frac{\dot Y}{q_\infty S C_{Y_\beta}}
$$

where $C_{Y_\beta}$ is the side-force derivative (typically negative for a statically stable aircraft, with the convention that positive $Y$ acts in the $+\hat{y}_B$ direction).

### Geometric Interpretation

$\dot\alpha$ is the angular rate at which the body $x$-axis rotates relative to the velocity vector within the body symmetry plane. $\dot\beta$ is the rate of lateral drift of the velocity vector out of the symmetry plane. Both terms appear directly in the $\boldsymbol{\omega}_{B/W}^B$ contribution to the body rate equations.

### Worked Example — Linear Load-Factor Ramp

This example ties together $\alpha$, $\dot\alpha$, $q_W$, and body pitch rate $q$ for
a coordinated pull-up with no sideslip ($\beta = 0$, $p_W = 0$).

**Aircraft parameters:**

| Parameter | Value |
| --- | --- |
| $C_{L\alpha}$ | 5.73 rad⁻¹ |
| $m$ | 1 200 kg |
| $S$ | 16 m² |
| $V$ (constant) | 100 m/s |
| $\rho$ | 1.225 kg/m³ → $q_\infty = 6125$ Pa |

**Input:** load factor ramps linearly: $n(t) = 1 + t/5$, $\dot n = 0.2$ s⁻¹.

From the linear lift model and the $\dot\alpha$ formula above:

$$\alpha(t) = n(t)\,\frac{mg}{C_{L\alpha}\,q_\infty\,S} = n(t)\times 0.02096\;\text{rad}$$

$$\dot\alpha = \dot n\;\frac{mg}{C_{L\alpha}\,q_\infty\,S} = 0.00419\;\text{rad/s} = 0.240°/\text{s (constant)}$$

The flight-path angular rate from the centripetal acceleration:

$$q_W(t) = \frac{(n-1)\,g}{V} = 0.01962\,t\;\text{rad/s}$$

Body pitch rate (from the body-rate decomposition at $\beta=0$):

$$q(t) = \dot\alpha + q_W(t)$$

| $t$ (s) | $n$ | $\alpha$ (°) | $\dot\alpha$ (°/s) | $q_W$ (°/s) | $q$ (°/s) | $\gamma$ (°) | $h$ (m) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 1.00 | 1.20 | 0.24 | 0.00 | 0.24 | 0.00 | 0.00 |
| 1 | 1.20 | 1.44 | 0.24 | 1.12 | 1.36 | 0.56 | 0.33 |
| 2 | 1.40 | 1.68 | 0.24 | 2.25 | 2.49 | 2.25 | 2.62 |
| 3 | 1.60 | 1.92 | 0.24 | 3.37 | 3.61 | 5.06 | 8.83 |
| 4 | 1.80 | 2.16 | 0.24 | 4.50 | 4.74 | 8.99 | 20.93 |
| 5 | 2.00 | 2.40 | 0.24 | 5.62 | 5.86 | 14.08 | 40.94 |

$\dot\alpha$ is constant because $\dot n$ is constant and the lift curve is linear.
$q = \dot\alpha + q_W$ shows two additive contributions: a small constant term from the
rising AoA and a growing term from the curving velocity path. $q_W$ dominates after
$t \approx 0.2$ s.

![Load-factor ramp time history](../implementation/figures/task2_phase_g_example.png)

---

## Aerodynamic Force Vectors

Lift $L$, drag $D$, and side force $Y$ are defined as the projections of the total aerodynamic force onto the Wind frame axes:

| Force | Wind frame vector | Sign convention |
| --- | --- | --- |
| Drag | $\mathbf{D}^W = [-D,\ 0,\ 0]^T$ | $D \geq 0$; opposes the velocity vector |
| Side force | $\mathbf{Y}^W = [0,\ Y,\ 0]^T$ | Positive in $+\hat{y}_W$ (to the right of the velocity vector) |
| Lift | $\mathbf{L}^W = [0,\ 0,\ -L]^T$ | $L \geq 0$; in $-\hat{z}_W$ (upward within the body symmetry plane) |

The total aerodynamic force in the Wind frame:

$$
\mathbf{F}_{aero}^W = \begin{bmatrix}-D \\ Y \\ -L\end{bmatrix}
$$

### In NED Frame

Apply the Wind-to-NED DCM:

$$
\mathbf{F}_{aero}^N = C_{NW}\,\mathbf{F}_{aero}^W, \qquad C_{NW} = C_{NB}\,C_{BW}
$$

where $C_{NB}$ is the rotation matrix corresponding to the body quaternion $q_{nb}$ and $C_{BW} = C_y(\alpha)\,C_z(-\beta)$. Equivalently, in quaternion form:

$$
\mathbf{F}_{aero}^N = q_{nw} \otimes \mathbf{F}_{aero}^W \otimes q_{nw}^*, \qquad q_{nw} = q_{nb} \otimes q_{bw}
$$

### In Body Frame

Expanding $\mathbf{F}_{aero}^B = C_{BW}\,\mathbf{F}_{aero}^W$:

$$
\mathbf{F}_{aero}^B
= \begin{bmatrix}
-D\cos\alpha\cos\beta - Y\cos\alpha\sin\beta + L\sin\alpha \\
-D\sin\beta + Y\cos\beta \\
-D\sin\alpha\cos\beta - Y\sin\alpha\sin\beta - L\cos\alpha
\end{bmatrix}
$$

For small angles ($\alpha,\beta \ll 1$) this reduces to approximately $[-D + L\alpha,\ Y,\ -L]^T$, recovering the familiar body-axis load directions: drag opposes $+\hat{x}_B$, side force acts along $\hat{y}_B$, and lift acts in $-\hat{z}_B$.

---

## Lift Curve Model

The linear model $C_L(\alpha) = C_{L_\alpha}\,\alpha$ is valid only within the pre-stall envelope. The following five-region piecewise model extends it symmetrically through positive and negative stall breaks and into fully separated flow on both sides.

### Input Parameters

| Parameter | Symbol | Description |
| ----------- | -------- | ------------- |
| `cl_alpha` | $C_{L_\alpha}$ | Pre-stall lift-curve slope (rad$^{-1}$) |
| `cl_max` | $C_{L,max}$ | Peak lift coefficient (positive stall vertex) |
| `cl_min` | $C_{L,min}$ | Minimum lift coefficient (negative stall vertex, $< 0$) |
| `delta_alpha_stall` | $\Delta\alpha_s$ | Angular distance from positive linear join to positive stall vertex (rad) |
| `delta_alpha_stall_neg` | $\Delta\alpha_{s,neg}$ | Angular distance from negative linear join to negative stall vertex (rad) |
| `cl_sep` | $C_{L,sep}$ | Positive post-stall plateau ($C_{L,sep} < C_{L,max}$) |
| `cl_sep_neg` | $C_{L,sep,neg}$ | Negative post-stall plateau ($C_{L,sep,neg} > C_{L,min}$) |

### Derived Breakpoints

**Positive side:**

$$
\alpha_{peak} = \frac{C_{L,max}}{C_{L_\alpha}} + \frac{\Delta\alpha_s}{2}, \qquad
\alpha_* = \alpha_{peak} - \Delta\alpha_s
$$

**Negative side:**

$$
\alpha_{peak,neg} = \frac{C_{L,min}}{C_{L_\alpha}} - \frac{\Delta\alpha_{s,neg}}{2}, \qquad
\alpha_{*,neg} = \alpha_{peak,neg} + \Delta\alpha_{s,neg}
$$

### Piecewise Definition

$$
C_L(\alpha) =
\begin{cases}
C_{L,sep,neg} & \alpha < \alpha_{sep,neg} \\[4pt]
a_{2n}\alpha^2 + a_{1n}\alpha + a_{0n} & \alpha_{sep,neg} \leq \alpha < \alpha_{*,neg} \\[4pt]
C_{L_\alpha}\,\alpha & \alpha_{*,neg} \leq \alpha \leq \alpha_* \\[4pt]
a_2\alpha^2 + a_1\alpha + a_0 & \alpha_* < \alpha \leq \alpha_{sep} \\[4pt]
C_{L,sep} & \alpha > \alpha_{sep}
\end{cases}
$$

The linear region has $C^1$ joins at both $\alpha_*$ (positive onset) and $\alpha_{*,neg}$ (negative onset). The flat regions join with $C^0$ continuity at $\alpha_{sep}$ and $\alpha_{sep,neg}$; the slope discontinuity at each separation boundary is physically representative of an abrupt flow reattachment boundary.

### Quadratic Coefficients

**Positive stall** (downward-opening parabola, vertex at $(\alpha_{peak},\, C_{L,max})$):

$$
a_2 = -\frac{C_{L_\alpha}}{2\,\Delta\alpha_s}, \qquad
a_1 = -2\,a_2\,\alpha_{peak}, \qquad
a_0 = a_2\,\alpha_{peak}^2 + C_{L,max}
$$

The $C^1$ join at $\alpha_*$ is satisfied by construction: the parabola's slope there equals $-2a_2\alpha_* + a_1 = -2a_2(\alpha_{peak} - \Delta\alpha_s) + a_1 = C_{L_\alpha}$. The separation angle is found by solving $C_L(\alpha_{sep}) = C_{L,sep}$:

$$
\alpha_{sep} = \alpha_{peak} + \sqrt{\frac{C_{L,sep} - C_{L,max}}{a_2}}
$$

**Negative stall** (upward-opening parabola, vertex at $(\alpha_{peak,neg},\, C_{L,min})$):

$$
a_{2n} = +\frac{C_{L_\alpha}}{2\,\Delta\alpha_{s,neg}}, \qquad
a_{1n} = -2\,a_{2n}\,\alpha_{peak,neg}, \qquad
a_{0n} = a_{2n}\,\alpha_{peak,neg}^2 + C_{L,min}
$$

The separation angle on the negative side:

$$
\alpha_{sep,neg} = \alpha_{peak,neg} - \sqrt{\frac{C_{L,sep,neg} - C_{L,min}}{a_{2n}}}
$$

### Lift-Curve Slope

$$
C_L'(\alpha) =
\begin{cases}
0 & \alpha < \alpha_{sep,neg} \\[4pt]
2a_{2n}\alpha + a_{1n} & \alpha_{sep,neg} \leq \alpha < \alpha_{*,neg} \\[4pt]
C_{L_\alpha} & \alpha_{*,neg} \leq \alpha \leq \alpha_* \\[4pt]
2a_2\alpha + a_1 & \alpha_* < \alpha \leq \alpha_{sep} \\[4pt]
0 & \alpha > \alpha_{sep}
\end{cases}
$$

### Key Boundary Angles

`alphaPeak()` returns $\alpha_{peak}$ — the angle at which the positive quadratic reaches $C_{L,max}$ with zero slope. `alphaTrough()` returns $\alpha_{peak,neg}$ — the angle at which the negative quadratic reaches $C_{L,min}$ with zero slope.

`alphaSep()` returns $\alpha_{sep}$ — the angle at which the positive descending parabola meets the flat post-stall plateau ($C_L = C_{L,sep}$). `alphaSepNeg()` returns the symmetric angle on the negative side. These mark the outer boundary of the parabolic CL domain; beyond them $C_L' = 0$ and the derivative of $f$ reduces to $T\cos\alpha$.

---

## Thrust Decomposition and Load Factor Allocation

When the thrust vector is aligned with the body $x$-axis it rotates with the aerodynamic angles. Its Wind frame representation follows from the first column of $C_{WB} = C_{BW}^T$:

$$
\mathbf{T}^W = T\,C_{BW}^T\begin{bmatrix}1\\0\\0\end{bmatrix}
= T\begin{bmatrix}\cos\alpha\cos\beta \\ -\cos\alpha\sin\beta \\ -\sin\alpha\end{bmatrix}
$$

The three Wind frame components have distinct kinematic roles:

| Component | Expression | Effect |
| --- | --- | --- |
| Tangential ($x_W$) | $T\cos\alpha\cos\beta$ | Accelerates/decelerates along the velocity vector |
| Lateral ($y_W$) | $-T\cos\alpha\sin\beta$ | Sideward force; must be balanced by $Y$ for coordinated flight |
| Normal ($-z_W$, upward) | $T\sin\alpha$ | Acts in the lift direction; contributes to load factor |

The **normal component $T\sin\alpha$ is independent of $\beta$** — it depends only on angle of attack.

### Implicit Equation for $\alpha$

The normal load factor $n$ (positive upward, perpendicular to velocity) is the total upward normal force divided by weight:

$$
n = \frac{L + T\sin\alpha}{mg}
\quad\Longrightarrow\quad
L + T\sin\alpha = n\,mg
$$

Substituting $L = q_\infty S C_L(\alpha)$ from the [Lift Curve Model](#lift-curve-model):

$$
\boxed{q_\infty S\,C_L(\alpha) + T\sin\alpha = n\,mg}
$$

This is an implicit equation for $\alpha$ given commanded load factor $n$, thrust $T$, dynamic pressure $q_\infty = \tfrac{1}{2}\rho V^2$, and mass $m$. The residual and its derivative for Newton's method are:

$$
f(\alpha) = q_\infty S\,C_L(\alpha) + T\sin\alpha - n\,mg, \qquad
f'(\alpha) = q_\infty S\,C_L'(\alpha) + T\cos\alpha
$$

$$
\alpha_{k+1} = \alpha_k - \frac{f(\alpha_k)}{f'(\alpha_k)}
$$

In the linear regime ($\alpha \leq \alpha_*$), $f'(\alpha) = q_\infty S C_{L_\alpha} + T\cos\alpha > 0$: the equation is strictly monotone and Newton converges in 2–3 iterations. In the stall transition and fully-separated regimes the behavior is more complex — see Issues 3 and 4 below.

### Alpha Limits as a Box Constraint

`alpha_min_rad` and `alpha_max_rad` (from the `airframe` config section) are hard
physical ceilings on angle of attack imposed by geometry, structural load paths, or
control authority.  They are **independent of the lift curve** — the lift curve
determines the aerodynamic shape of the load-factor envelope; the alpha limits
determine the physical domain within which the solver is permitted to operate.

The limits are enforced as a **box constraint inside the Newton iteration**, not as a
post-solve clamp.  After each Newton step the iterate is projected onto
$[\alpha_{\min},\ \alpha_{\max}]$:

$$
\alpha_{k+1} \leftarrow \text{clamp}\!\left(\alpha_k - \frac{f(\alpha_k)}{f'(\alpha_k)},\ \alpha_{\min},\ \alpha_{\max}\right)
$$

**Boundary recognition.** If the projected iterate reaches a boundary and the residual
$f(\alpha_{\text{boundary}})$ still has the sign that would push Newton further outside
the domain, the boundary is the constrained solution — the solver accepts it and exits.
Specifically:

- At $\alpha_{\max}$: if $f(\alpha_{\max}) > 0$ (lift + thrust still exceeds demand at
  the upper limit), the demanded load factor is unachievable within the physical
  envelope.  Return $\alpha_{\max}$ and set the alpha-limit flag.
- At $\alpha_{\min}$: if $f(\alpha_{\min}) < 0$ (lift + thrust is below demand at the
  lower limit), return $\alpha_{\min}$ and set the alpha-limit flag.

When neither boundary is active the projected Newton iteration converges normally;
the alpha-limit flag is clear.

**Interaction with the fold/overshoot guards.** The alpha limits and the stall fold
guards are independent.  The fold guard fires when $f'(\alpha_k) \approx 0$ (the
aerodynamic ceiling); the alpha limit fires when the physical domain boundary is
reached.  Either can be binding first, depending on the aircraft configuration:

- If $\alpha_{\max} < \alpha_{\text{fold}}$: the alpha limit is always binding before
  the aerodynamic fold point is reached.
- If $\alpha_{\max} \geq \alpha_{\text{fold}}$: the fold guard may fire first; the
  alpha limit is a secondary ceiling that applies to whatever alpha the fold guard
  produces.

In both cases the two guards compose correctly because the alpha-limit projection is
applied at every Newton step, including steps taken after the fold guard triggers a
bisection fallback.

---

### Branch Continuation and Root Tracking

To resolve root ambiguity across timesteps, the solver carries the previously accepted angle of attack $\alpha_{prev}$ as persistent state, initialized to zero.

**Predictor.** Let $\delta n = n_k - n_{k-1}$ be the step change in commanded load factor. Differentiating $f(\alpha) = 0$ with respect to $n$ gives $\partial\alpha/\partial n = mg / f'(\alpha)$. The first-order predictor for the new initial guess is therefore:

$$
\alpha_0 = \alpha_{prev} + \frac{\delta n \cdot mg}{f'(\alpha_{prev})}
$$

Newton's method is then run from $\alpha_0$ rather than from a fixed starting point. For small $\delta n$, $\alpha_0$ lands on the same branch as $\alpha_{prev}$ and convergence to the correct root is assured.

**Branch identification.** The sign of $f'(\alpha_{prev})$ identifies which branch is active:

| $f'(\alpha_{prev})$ | Branch | Interpretation |
| --- | --- | --- |
| $> 0$ | Pre-stall | $\alpha$ on the ascending or linear limb of $f$ |
| $< 0$ | Post-stall | $\alpha$ on the descending limb in the transition region |
| $= 0$ | Fold point | Aircraft is at the stall limit; predictor is ill-conditioned |

**Fold point and stall limit.** $f'(\alpha_{prev}) \to 0$ signals that the aircraft is at the load factor ceiling — the fold point where $f'(\alpha) = q_\infty S C_L'(\alpha) + T\cos\alpha = 0$. This fold point is **not** fixed at $\alpha_{peak}$: for $T = 0$ it coincides with $\alpha_{peak}$ (where $C_L' = 0$), but for $T > 0$ it lies above $\alpha_{peak}$ because the $T\cos\alpha$ term keeps $f'$ positive past the CL peak. Two outcomes:

1. If the commanded $n_k\,mg > \max_\alpha f(\alpha) + n_{k-1}\,mg$, no root exists — the load factor demand exceeds what lift and thrust can provide. Saturate $\alpha$ at $\alpha_{prev}$ (the fold point), set a stall-limit flag, and clamp $n$ to the achievable maximum.
2. If $n_k\,mg$ has decreased (load factor released), the fold is exiting and a root on the pre-stall side can be found by bracketed search in $(\alpha_*, \alpha_{prev})$.

**Initialization and regime switching.** On the first call (or after a large discontinuous $\delta n$), the predictor may land far from the correct root. In this case, evaluate $f$ at $\alpha_*$, $\alpha_{peak}$, and $\alpha_{sep}$ to identify which intervals contain sign changes, then pick the initial guess from the interval consistent with the current branch state.

**Thrust-reduction branch switch.** A qualitatively different regime change occurs when thrust drops while $\alpha_{prev}$ is above $\alpha_{peak}$.  The fold point $\alpha^*$ where $f'(\alpha) = q_\infty S C_L'(\alpha) + T\cos\alpha = 0$ moves toward $\alpha_{peak}$ as $T$ decreases.  If thrust drops enough that $f'(\alpha_{prev}) < 0$ with the new (lower) $T$, the aircraft is no longer at a stable operating point — the net vertical force is decreasing with $\alpha$ at the current attitude.  A real fly-by-wire system responds by commanding nose-down to reduce $\alpha$ until a stable pre-fold operating point is recovered.

The solver must replicate this behavior.  The detection condition is:

$$
f'(\alpha_{prev};\,T_{\text{new}}) = q_\infty S C_L'(\alpha_{prev}) + T_{\text{new}}\cos\alpha_{prev} < 0
\quad\text{and}\quad
f(\alpha_{prev};\,T_{\text{new}}) > 0
$$

The first condition means $\alpha_{prev}$ is now on the wrong side of the new fold (negative gradient — demand increases faster than lift + thrust can track).  The second means the load factor demand is still unsatisfied (the aircraft is not simply over-lifting).  Together they identify the thrust-reduction stall: $\alpha_{prev}$ is above the new fold with insufficient force to meet the demand.

When this condition is detected, the predictor **discards $\alpha_{prev}$ and resets the warm-start to $\alpha = 0$**.  Newton then converges to the pre-stall root that satisfies the demand with the reduced thrust.  This produces a nose-down transient: $\alpha$ drops from the post-fold value to the new pre-stall solution, mirroring the FBW pitch-down response.

If the demand is unachievable even on the pre-stall side (e.g., thrust dropped so much that even $\alpha_{peak}$ cannot provide enough lift), Newton sweeps to the fold and the fold guard clamps $\alpha$ at the new $\alpha^*$ — the best achievable attitude with the current thrust.  The stall flag is set.

Note: if $f(\alpha_{prev}) \leq 0$ (the aircraft is generating more than enough force — e.g., a throttle cut at low commanded Nz), there is no instability and no branch switch is needed.  The standard predictor step handles this case correctly.

### Alpha Rate Limiting: Stall Recovery Bridge

#### Problem Statement

When operating above $\alpha_{peak}$ — whether due to a large Nz command with sufficient thrust, or because the branch-continuation predictor has tracked alpha into the post-fold region — a reduction in thrust or commanded Nz can create a discontinuity: the Newton equilibrium solution $\alpha_{eq}$ jumps from a post-fold value to a pre-stall value in one timestep.  Even with the thrust-reduction branch switch above, the solver's warm-start reset produces an instantaneous step in $\alpha$, which violates physical continuity.

A real FBW aircraft with finite pitch authority cannot change $\alpha$ instantaneously.  The pitch channel drives $\alpha$ toward the equilibrium at a rate bounded by the achievable pitch authority.  The alpha rate-limiting mechanism models this constraint.

#### Design Concept: Bridge, Not Filter

This mechanism is **not** a continuous lag filter on $\alpha$.  It is a **bridge**: a transient device that applies only when the Newton equilibrium $\alpha_{eq}$ is not reachable in a single timestep at the configured pitch authority.  The moment $\alpha$ converges to $\alpha_{eq}$, the bridge deactivates and the solver is solution-locked at the Newton equilibrium.

The distinction is critical: a continuous lag filter would slow down normal pre-stall maneuvers — every pull-up would be sluggish.  The bridge activates only when alpha would otherwise jump discontinuously (across the fold or away from a post-fold operating point).

#### Mechanism

Let $\alpha_{k}$ be the current alpha (from the previous timestep), $\alpha_{eq}$ be the Newton equilibrium target computed by the existing solver, and $\dot\alpha_{\max}$ be the maximum pitch authority rate (rad/s), a new aircraft configuration parameter.

The rate-limited output $\alpha_{k+1}$ is:

$$
\alpha_{k+1} = \alpha_{k} + \text{clamp}\!\left(\alpha_{eq} - \alpha_{k},\ -\dot\alpha_{\max}\Delta t,\ +\dot\alpha_{\max}\Delta t\right)
$$

That is: step toward $\alpha_{eq}$ at the full Newton solution, but limit the change per timestep to $\dot\alpha_{\max}\Delta t$.  When $|\alpha_{eq} - \alpha_k| \leq \dot\alpha_{\max}\Delta t$, the clamp is inactive and $\alpha_{k+1} = \alpha_{eq}$ — the bridge deactivates and normal solution-locked behavior is restored.

#### Load Factor from Rate-Limited Alpha

Because $\alpha_{k+1}$ may differ from $\alpha_{eq}$ during the bridging transient, the realized load factor is not the commanded $n$ but the load factor achievable at the rate-limited alpha:

$$
n_{\text{realized}} = \frac{q_\infty S\,C_L(\alpha_{k+1}) + T\sin(\alpha_{k+1})}{mg}
$$

This is the load factor actually delivered to the kinematic integrator.  The commanded $n$ is the request; $n_{\text{realized}}$ is what the equations of motion see.

This saturation of $n_{\text{realized}}$ is physically correct and acceptable: the aircraft is already near its structural or aerodynamic ceiling when the bridge is active.  Any shortfall in realized Nz during the bridge transient is a consequence of limited pitch authority, not a modeling error.

#### When the Bridge Is Active

The bridge is active if and only if:

$$
|\alpha_{eq} - \alpha_k| > \dot\alpha_{\max}\Delta t
$$

In normal pre-stall operation this condition is never true: each Newton step produces an $\alpha_{eq}$ that is only slightly different from $\alpha_k$ (the predictor places the warm-start near the solution), and $\dot\alpha_{\max}\Delta t$ is much larger than the inter-timestep change.  The bridge imposes no cost on normal operation.

The bridge activates only when the Newton equilibrium target has moved by more than one step of pitch authority from the current alpha — which occurs precisely in the scenarios described above: post-fold operating points under reduced thrust, or large step demands that sweep alpha across the fold.

#### Relationship to the Thrust-Reduction Branch Switch

The thrust-reduction branch switch (warm-start reset to $\alpha = 0$) and the alpha rate limiter are complementary:

- The branch switch ensures the Newton solver finds the correct pre-stall root after a thrust reduction event.
- The rate limiter ensures that the transition from the old $\alpha_{prev}$ (post-fold) to the new $\alpha_{eq}$ (pre-stall) is physically continuous.

Without the rate limiter, the branch switch produces an instantaneous jump.  With the rate limiter, $\alpha$ walks from the post-fold value to the pre-stall equilibrium at $\dot\alpha_{\max}$ — modeling the FBW nose-down recovery.  Once $\alpha$ reaches $\alpha_{eq}$, the bridge deactivates and solution-locked operation resumes.

#### Configuration Parameter

`alpha_dot_max_rad_s` — maximum pitch authority rate (rad/s).  Resides in the `airframe` section of the aircraft configuration (alongside `alpha_max_rad` and `alpha_min_rad`).  Represents the combined effect of pitch control authority and pitch moment of inertia: how fast the FBW system can drive $\alpha$ given pitch damping and available control moment.

A default of $\pi/2$ rad/s is a generous upper bound that makes the bridge effectively inactive for most configurations; operators with specific authority models should set this to the achievable pitch rate at the maneuver Nz condition.

### Stall Hysteresis and CL Recovery

#### Physical Motivation

In the nominal lift curve model, CL recovers immediately as alpha decreases from the post-stall region — the model is stateless, so the same $\alpha$ always produces the same $C_L$.  Real aerodynamics do not behave this way.  Once separated flow has established over the wing, reducing alpha does not immediately reattach it.  The boundary layer remains separated — and CL stays at the post-stall plateau value — until alpha has dropped far enough for reattachment to occur.  Reattachment happens approximately at $\alpha_*$, the angle where the parabolic stall transition meets the linear regime.  Below $\alpha_*$, attached flow is recovered and the linear lift model applies again.

This hysteresis means the CL curve traces different paths depending on the direction of alpha change:

- **Increasing alpha past $\alpha_{sep}$**: CL follows the nominal model — linear to $\alpha_*$, parabolic to $\alpha_{peak}$, parabolic descent to $\alpha_{sep}$, then flat at $C_{L,sep}$.
- **Decreasing alpha from stall**: CL stays flat at $C_{L,sep}$ until $\alpha$ falls below $\alpha_*$, at which point the linear model reapplies.

The region $\alpha_* \leq \alpha \leq \alpha_{sep}$ is therefore hysteretic: CL depends on whether the aircraft arrived from above or below.

#### State

The hysteresis state is a boolean flag `_stalled` (positive side) and `_stalled_neg` (negative side) carried by `LoadFactorAllocator`.  The flags are initialized to `false`.

**Entry condition** (set flag): the flag sets when alpha has passed $\alpha_{peak}$ and the rate-limited output $\alpha_{k+1}$ begins to decrease:

$$
\alpha_{k+1} < \alpha_k \quad \text{and} \quad \alpha_k \geq \alpha_{peak}
\quad\Longrightarrow\quad \texttt{\_stalled} = \text{true}
$$

Symmetrically for the negative side (alpha has passed $\alpha_{trough}$ and begins to increase).

**Exit condition** (clear flag):
$$
\alpha_{k+1} \leq \alpha_* \quad\Longrightarrow\quad \texttt{\_stalled} = \text{false}
$$
$$
\alpha_{k+1} \geq \alpha_{*,neg} \quad\Longrightarrow\quad \texttt{\_stalled\_neg} = \text{false}
$$

The flags are evaluated using the rate-limited $\alpha_{k+1}$ (after the alpha bridge has been applied), not the Newton equilibrium target $\alpha_{eq}$.

#### Effective CL While Stalled

While `_stalled` is true, the effective lift coefficient is fixed at the post-stall plateau regardless of alpha:

$$
C_{L,\text{eff}} = C_{L,sep} \quad \text{while } \texttt{\_stalled} = \text{true}
$$

This is the **returning curve** — CL stays flat at $C_{L,sep}$ throughout the entire descent from $\alpha_{peak}$ through the stall transition region, all the way down to $\alpha_*$.  The nominal descending parabola is bypassed entirely on the way back.

If alpha reverses and increases while `_stalled` is true (a new Nz demand before recovery completes), CL remains flat at $C_{L,sep}$ — the boundary layer has not reattached and no lift recovery occurs until alpha falls back to $\alpha_*$.  If alpha rises back past $\alpha_{peak}$ and then decreases again, the entry condition fires again and `_cl_recovering` resets to $C_{L,sep}$.

When $\alpha_{k+1}$ falls below $\alpha_*$, `_stalled` clears.  If alpha then begins to rise, it re-enters the nominal lift curve at the linear regime $C_L = C_{L_\alpha}\,\alpha$; CL descends along the nominal parabola only if alpha rises far enough to re-enter the stall transition.  At that point, if alpha once again passes $\alpha_{peak}$ and decreases, `_stalled` sets again.

#### CL Recovery Rate Limiting

At the moment `_stalled` clears ($\alpha_{k+1}$ falls below $\alpha_*$), the nominal CL at $\alpha_*$ is:

$$
C_{L,\text{nom}}(\alpha_*) = C_{L_\alpha}\,\alpha_*
$$

which is higher than the returning curve value $C_{L,sep}$ (since $C_{L,sep} < C_{L,max}$ and $C_{L,max} > C_{L_\alpha}\,\alpha_*$ by construction).  The CL cannot jump instantaneously to the nominal value — lift rebuilds as boundary layer flow reattaches, which takes finite time.

The maximum rate of CL recovery is related to `alpha_dot_max_rad_s` through the lift-curve slope:

$$
\dot{C}_{L,\max} = C_{L_\alpha} \cdot \dot\alpha_{\max}
$$

This has a clear physical basis: the same pitch authority that limits how fast $\alpha$ can change also bounds how fast the aerodynamic angle of attack seen by the wing can change, and therefore how fast attached-flow lift can rebuild.  No additional configuration parameter is required.

The CL recovery bridge operates analogously to the alpha rate bridge:

$$
C_{L,k+1} = C_{L,k} + \text{clamp}\!\left(C_{L,\text{nom}}(\alpha_{k+1}) - C_{L,k},\ 0,\ \dot{C}_{L,\max}\Delta t\right)
$$

The clamp is one-sided (lower bound zero): CL may only increase during recovery, never overshoot.  The bridge is inactive — $C_{L,k+1} = C_{L,\text{nom}}(\alpha_{k+1})$ — as soon as the gap closes.

**Instant jump when nominal is lower.** If at any point $C_{L,\text{nom}}(\alpha_{k+1}) < C_{L,k}$ (e.g., the aircraft is in a region where the nominal curve is below the returning plateau), the clamp produces a zero step and $C_{L,k+1} = C_{L,k}$.  But this condition — nominal below the returning value — cannot occur in the linear regime below $\alpha_*$ where the flag has cleared.  It can occur transiently above $\alpha_*$ if alpha has just crossed and nominal is still rising; in that case the one-sided clamp correctly holds CL at its current value rather than jumping down.  The only direction where an instantaneous jump is permitted is downward: if $C_{L,\text{nom}}(\alpha_{k+1}) < C_{L,k}$ strictly (nominal is below current), accept the jump — this represents a condition where the nominal model would produce less lift than the recovering model, and there is no physical basis for sustaining the higher value.

Concretely: replace the one-sided clamp with a full-range clamp that permits downward steps but limits upward steps:

$$
C_{L,k+1} = C_{L,k} + \text{clamp}\!\left(C_{L,\text{nom}}(\alpha_{k+1}) - C_{L,k},\ -\infty,\ \dot{C}_{L,\max}\Delta t\right)
$$

Equivalently: $C_{L,k+1} = \min\!\left(C_{L,\text{nom}}(\alpha_{k+1}),\ C_{L,k} + \dot{C}_{L,\max}\Delta t\right)$.

#### Interaction with the Newton Solver

While `_stalled` is true, the Newton loop is bypassed entirely.  The alpha equilibrium target is computed by a single explicit solve using the current recovering CL as a fixed constant:

$$
f(\alpha) = q_\infty S\,C_{L,k} + T\sin\alpha - n\,mg = 0
\quad\Longrightarrow\quad
\alpha_{eq} = \arcsin\!\left(\frac{n\,mg - q_\infty S\,C_{L,k}}{T}\right)
$$

This is the fully-separated explicit form (Issue 4 above) with $C_{L,k}$ substituted for $C_{L,sep}$.  From the solver's perspective, CL is a constant — it is not varying with alpha during the stall — so $f'(\alpha) = T\cos\alpha$ and no iteration is required.

The alpha bridge then steps toward $\alpha_{eq}$ at the rate limit.  **The alpha bridge is the only mechanism moving alpha while stalled** — no unconstrained jump occurs.

If $T = 0$, the explicit solve has no solution (the equation becomes $q_\infty S\,C_{L,k} = n\,mg$, which is either always true or never true independent of alpha).  In this case $\alpha_{eq}$ is undefined; the alpha bridge holds $\alpha_{k+1} = \alpha_k$ and the stall flag remains set until alpha is externally driven below $\alpha_*$.

Once `_stalled` clears and the CL recovery bridge deactivates ($C_{L,k}$ has reached $C_{L,\text{nom}}(\alpha_{k+1})$), the nominal Newton loop resumes for both equilibrium solve and derivative computation.

#### Summary of `LoadFactorAllocator` State

| State variable | Purpose |
| --- | --- |
| `_alpha_prev` | Warm-start and branch-continuation predictor |
| `_beta_prev` | Warm-start for lateral solver |
| `_n_z_prev` | Branch-continuation predictor for alpha |
| `_n_y_prev` | Branch-continuation predictor for beta |
| `_stalled` | Positive-side hysteresis flag |
| `_stalled_neg` | Negative-side hysteresis flag |
| `_cl_recovering` | Current effective CL during post-stall recovery (positive side) |
| `_cl_recovering_neg` | Current effective CL during post-stall recovery (negative side) |

### Implicit Equation for $\beta$

The lateral load factor $n_y$ (positive in $+\hat{y}_W$, to the right of the velocity vector) is the commanded lateral force per unit weight. It is provided jointly by the aerodynamic side force $Y$ and the lateral component of thrust $-T\cos\alpha\sin\beta$:

$$
n_y = \frac{Y - T\cos\alpha\sin\beta}{mg}
\quad\Longrightarrow\quad
Y - T\cos\alpha\sin\beta = n_y\,mg
$$

Substituting the linear side force model $Y = q_\infty S C_{Y_\beta}\,\beta$:

$$
\boxed{q_\infty S C_{Y_\beta}\,\beta - T\cos\alpha\sin\beta = n_y\,mg}
$$

**Sequential solve.** The $\alpha$ equation involves $T\sin\alpha$ (which has no $\beta$ dependence); the $\beta$ equation involves $T\cos\alpha$, which is now fixed from the preceding $\alpha$ solution. Solve for $\alpha$ first; use it as a constant in the $\beta$ equation.

The residual and its derivative:

$$
g(\beta) = q_\infty S C_{Y_\beta}\,\beta - T\cos\alpha\sin\beta - n_y\,mg, \qquad
g'(\beta) = q_\infty S C_{Y_\beta} - T\cos\alpha\cos\beta
$$

$$
\beta_{k+1} = \beta_k - \frac{g(\beta_k)}{g'(\beta_k)}
$$

Since $C_{Y_\beta} < 0$ (side-force stability derivative) and $T\cos\alpha\cos\beta > 0$, both terms of $g'(\beta)$ are negative: $g'(\beta) < 0$ everywhere. The equation is **strictly monotone decreasing** and Newton converges to a unique root from any initial guess. No branch ambiguity, fold point, or stall-limit analog exists for the lateral equation.

**Predictor.** Carrying $\beta_{prev}$ as persistent state (initialized to zero), the first-order predictor is:

$$
\beta_0 = \beta_{prev} + \frac{\delta n_y \cdot mg}{g'(\beta_{prev})}, \qquad \delta n_y = n_{y,k} - n_{y,k-1}
$$

Because $g' < 0$, the predictor correctly directs $\beta$ opposite to $n_y$: a rightward demand ($\delta n_y > 0$) predicts a more negative $\beta$ (the body yaws left relative to the velocity, generating a rightward side force and rightward thrust component).

### Tangential Force Balance

Along the velocity vector, the net tangential acceleration drives airspeed change. With $\alpha$ and $\beta$ both resolved:

$$
a_\parallel = \frac{T\cos\alpha\cos\beta - D}{m} - g\sin\gamma_a
$$

The factor $\cos\alpha\cos\beta$ accounts for the full projection of the thrust vector onto the velocity direction.

### Issues and Limitations

**1. Thrust misalignment.** If the engine has a fixed incidence offset $\delta_T$ relative to $\hat{x}_B$, replace $\alpha$ with $\alpha + \delta_T$ in the thrust decomposition. The implicit equation becomes $q_\infty S\,C_L(\alpha) + T\sin(\alpha + \delta_T) = nmg$; the Newton derivative changes to $f'(\alpha) = q_\infty S\,C_L'(\alpha) + T\cos(\alpha + \delta_T)$.

**2. Zero-$\alpha$ lift.** A cambered airfoil generates lift at $\alpha = 0$: $L = q_\infty S(C_{L_0} + C_{L_\alpha}\,\alpha)$. The implicit equation becomes $q_\infty S(C_{L_0} + C_{L_\alpha}\,\alpha) + T\sin\alpha = nmg$. The Newton step is unchanged; the residual constant shifts by $q_\infty S C_{L_0}$.

**3. Multiple roots in the stall transition.** In the transition region $(\alpha_*, \alpha_{sep})$, $f'(\alpha) = q_\infty S C_L'(\alpha) + T\cos\alpha$ can change sign. $C_L'(\alpha)$ decreases linearly from $C_{L_\alpha}$ at $\alpha_*$ to $2\Delta C_L/\Delta\alpha + C_{L_\alpha}$ at $\alpha_{sep}$ (negative for a significant stall break). When $q_\infty S |C_L'(\alpha)| > T\cos\alpha$, $f'$ turns negative, $f$ develops a local maximum, and the equation $f(\alpha) = 0$ can have two roots in the transition interval — one on the ascending limb ($\alpha < \alpha_{peak}$) and one on the descending limb. The physically relevant root is the lower $\alpha$ (pre-stall side). Newton's method starting from the linear-regime solution converges to the correct root; starting above $\alpha_{peak}$ risks converging to the post-stall root or diverging if $f' \approx 0$. Safe practice: bracket $f$ at $\alpha_*$, $\alpha_{peak}$, and $\alpha_{sep}$ before iterating.

**4. Explicit solution in the fully-separated regime.** For $\alpha > \alpha_{sep}$, $C_L(\alpha) = C_{L,sep}$ is constant, and the equation reduces to:
$$
q_\infty S\,C_{L,sep} + T\sin\alpha = n\,mg
\quad\Longrightarrow\quad
\alpha = \arcsin\!\left(\frac{n\,mg - q_\infty S\,C_{L,sep}}{T}\right)
$$
No iteration is required. A solution in this region exists only when $T > 0$ and $0 \leq n\,mg - q_\infty S\,C_{L,sep} \leq T$. If $T = 0$ (glide), $f$ is constant in this region and either always zero or never zero; there is no $\alpha$ to solve for. If the commanded $n\,mg$ exceeds $q_\infty S\,C_{L,sep} + T$ (the ceiling of achievable normal force), no solution exists anywhere and the load factor demand is infeasible.

**5. Lateral coupling.** For $\beta \neq 0$ the lateral thrust component $-T\cos\alpha\sin\beta$ must be balanced by the aerodynamic side force $Y = q_\infty S C_{Y_\beta}\,\beta$. For large thrust this couples the $\alpha$ and $\beta$ equations. In practice, coordinated flight ($\beta \approx 0$) is maintained by the yaw channel, decoupling them.

**6. Quasi-static assumption.** The equation $L + T\sin\alpha = nmg$ is an instantaneous force balance. It gives the $\alpha$ required for a steady load factor but does not capture pitch dynamics or $q$-dependent lift ($C_{L_q}$ terms). In a rapidly pitching maneuver the actual $\alpha$ will lag the quasi-static solution.

---

## Derived Quantities

| Quantity | Expression | Unit |
| --- | --- | --- |
| Airspeed | $V = \|\mathbf{v}_W\|$ | m/s |
| Ground speed | $V_G = \|\mathbf{v}_N\|$ | m/s |
| Flight path angle | $\gamma = -\arcsin(v_D / V_G)$ | rad |
| Track angle | $\chi = \arctan(v_E / v_N)$ | rad |
| Crab angle | $\xi = \chi - \psi$ | rad |
| Load factor | $n = \|\mathbf{a}_W\| / g$ | — |

---

## Integration Scheme Summary

```mermaid
flowchart TD
    IN["Inputs at step k<br/>a_W, rollRate_W, α, β, α̇, β̇<br/>wind_NED"] -->
    ROT["Rotate a_W → NED<br/>a_N = C_NW · a_W"] -->
    PVRK4["RK4 over joint state (φ, λ, h, v_N, v_E, v_D)<br/>4 evaluations of WGS84 geodetic rates<br/>v_{k+1} = v_k + Δt · a_N  (const-accel equivalent)"] -->
    ATT["Propagate quaternion<br/>q_{nb,k+1} = q_{nb,k} + Δt/2 · q ⊗ ω<br/>then renormalize"] -->
    OUT["KinematicState k+1"]
```

Position and velocity are integrated jointly with classical **fourth-order Runge-Kutta
(RK4)** at a fixed timestep $\Delta t$. Attitude uses forward Euler on the quaternion ODE.

---

## Implementation Notes

- Attitude: `KinematicState::_q_nb` (Body-to-NED quaternion, `Eigen::Quaternionf`)
- Body rates: `KinematicState::_rates_Body_rps` (p, q, r in rad/s)
- NED velocity: `KinematicState::_velocity_NED_mps`
- NED acceleration: `KinematicState::_acceleration_NED_mps`
- Position: `KinematicState::_positionDatum` (WGS84 lat/lon/alt)
- Aerodynamic angles α, β are inputs to `step()` — computed by aerodynamics subsystem, not integrated here
- All stored values are SI: radians, meters, seconds
