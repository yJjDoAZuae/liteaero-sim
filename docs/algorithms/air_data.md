# Air Data System — Algorithm Design

This document defines the measurement model for `SensorAirData`. It covers the pitot-static physical model, the two-transducer noise and lag architecture, and the full airspeed derivation chain from impact pressure through IAS, CAS, EAS, TAS, Mach, barometric altitude, and OAT. References: ICAO Doc 7488/3 (Manual of the ICAO Standard Atmosphere), ICAO Annex 2 (airspeed definitions), MIL-HDBK-1797 (Flying Qualities of Piloted Aircraft).

---

## Notation

| Symbol | Description | Unit |
| --- | --- | --- |
| $P_s$ | Static (ambient) pressure | Pa |
| $P_t$ | Total (stagnation) pressure | Pa |
| $q_c$ | Impact pressure $= P_t - P_s$ | Pa |
| $P_0$ | Sea-level ISA pressure = 101 325 | Pa |
| $\rho$ | Air density at flight altitude | kg/m³ |
| $\rho_0$ | Sea-level ISA density = 1.225 | kg/m³ |
| $a$ | Speed of sound at flight altitude | m/s |
| $a_0$ | Sea-level ISA speed of sound = 340.294 | m/s |
| $T_s$ | Static (ambient) temperature | K |
| $T_0$ | Sea-level ISA temperature = 288.15 | K |
| $M$ | Mach number $= V_{TAS}/a$ | — |
| $\gamma$ | Specific heat ratio = 1.4 | — |
| $R_d$ | Dry air gas constant = 287.058 | J/(kg·K) |
| $g_0$ | Standard gravity = 9.806 65 | m/s² |
| $\Delta T$ | ISA temperature deviation = $T_s - T_{ISA}(h_{geo})$ | K |
| $P_{sfc}$ | Actual sea-level pressure (`AtmosphereConfig::surface_pressure_pa`) | Pa |
| $\Delta P_{sfc}$ | Sea-level pressure deviation from ISA = $P_{sfc} - P_0$ | Pa |
| $P_{Koll}$ | Altimeter Kollsman setting (QNH) | Pa |

---

## Pitot-Static Physical Model

A pitot-static system measures two pressures: total (stagnation) pressure $P_t$ at the pitot tube opening, and static pressure $P_s$ at the flush static port. The air data computer derives airspeed from the difference $q_c = P_t - P_s$.

For isentropic (subsonic, adiabatic, reversible) flow, the stagnation pressure is related to static pressure and Mach number by:

$$P_t = P_s\left(1 + \frac{\gamma-1}{2}M^2\right)^{\frac{\gamma}{\gamma-1}}$$

The impact pressure is therefore:

$$q_c = P_t - P_s = P_s\left[\left(1 + \frac{\gamma-1}{2}M^2\right)^{\frac{\gamma}{\gamma-1}} - 1\right]$$

For $M < 0.3$ (typical UAS cruise), the binomial expansion of the isentropic formula gives $q_c \approx \tfrac{1}{2}\rho V_{TAS}^2$ with a compressibility error of less than 2.5%. The full isentropic formula is used for all Mach numbers in this simulation; no low-speed approximation is made.

The exponents $\tfrac{\gamma}{\gamma-1}$ and $\tfrac{\gamma-1}{\gamma}$ appear repeatedly in what follows.

---

## True Quantities

These are the ideal (noiseless, unlagged) values that the transducer channels observe. They are computed from `step()` inputs before noise and lag are applied.

Given input `airspeed_body_mps` $= \mathbf{v}^B_{a/w}$ (body-frame wind-relative velocity vector):

$$V_{TAS} = \|\mathbf{v}^B_{a/w}\|$$

$$M = \frac{V_{TAS}}{a}$$

where $a$ = `AtmosphericState::speed_of_sound_mps`.

True impact pressure:

$$q_c^{true} = P_s\left[\left(1 + \frac{\gamma-1}{2}M^2\right)^{\frac{\gamma}{\gamma-1}} - 1\right]$$

True static pressure: $P_s^{true}$ = `AtmosphericState::pressure_pa`.

These two scalar quantities — $q_c^{true}$ and $P_s^{true}$ — are the primary inputs to the two transducer models. The individual body-frame components $(u, v, w)$ are also retained: $v$ and $w$ are used to compute $\sin\beta$ and $\sin\alpha$ for the static pressure fuselage crossflow error model (see §Static Pressure Transducer).

---

## Transducer Models

### Differential Pressure Transducer

The differential pressure transducer measures $q_c = P_t - P_s$. Noise is additive Gaussian, and the signal is passed through a first-order Tustin-discretized lag.

**Noise:**

$$q_c^{noisy} = q_c^{true} + n_{qc}, \qquad n_{qc} \sim \mathcal{N}(0,\,\sigma_{qc}^2)$$

where $\sigma_{qc}$ = `differential_pressure_noise_pa`.

**First-order Tustin lag** (time constant $\tau_{qc}$, simulation timestep $T_s$):

Define:

$$\alpha_{qc} = \frac{T_s}{2\tau_{qc} + T_s}, \qquad \beta_{qc} = \frac{2\tau_{qc} - T_s}{2\tau_{qc} + T_s}$$

The recurrence at step $k$:

$${q_c^{meas}}_k = \alpha_{qc}\left({q_c^{noisy}}_k + {q_c^{noisy}}_{k-1}\right) + \beta_{qc}\,{q_c^{meas}}_{k-1}$$

When $\tau_{qc} = 0$: $\alpha_{qc} = 1$, $\beta_{qc} = -1$, and the recurrence reduces to $q_c^{meas} = q_c^{noisy}$ (identity, no lag). The implementation detects $\tau_{qc} = 0$ explicitly and skips the lag computation.

---

### Static Pressure Transducer

#### Fuselage Crossflow Pressure Error — Two-Port Symmetric Crosslinked Model

A static port flush-mounted on the fuselage side reads a pressure that deviates from the
true freestream static pressure when the fuselage is at nonzero angle of attack or
sideslip. The deviation is caused by the local velocity perturbation around the fuselage
cross-section.

The model here uses two static ports symmetric about the body XZ plane (one starboard, one
port), connected by a pneumatic crosslink. The transducer measures the pressure at the
midpoint of the crosslink tube; when the two port tubes have equal resistance, the
transducer reads the arithmetic average of the two port pressures. This averaging
inherently cancels the component of the static pressure error that is antisymmetric about
the XZ plane. Because the β-induced error IS antisymmetric (Port A sees higher pressure,
Port B sees lower pressure by equal amounts for pure sideslip), the crosslinked average
provides partial or complete mitigation of β effects depending on port location.

The derivation uses the 2D potential flow model for an infinite circular cylinder in
uniform crossflow, accurate for port locations well away from the nose and tail.

---

**Potential flow surface pressure.** For 2D potential flow around a circular cylinder in
uniform crossflow $V_c$, the surface tangential velocity at angle $\psi$ from the forward
stagnation point is $2 V_c \sin\psi$. The Bernoulli equation gives:

$$C_p(\psi) = \frac{P_{surface} - P_\infty}{q_{cross}} = 1 - 4\sin^2\psi$$

where $q_{cross} = \tfrac{1}{2}\rho V_c^2$ is the crossflow dynamic pressure, $C_p = 1$
at the forward and rear stagnation points ($\psi = 0°$, $180°$), and $C_p = -3$ at the
equator ($\psi = 90°$, maximum suction).

---

**Port geometry.** In the body cross-section (looking aft), define angle $\theta$ from the
fuselage top (12 o'clock), positive clockwise:

| $\theta$ | Position |
| --- | --- |
| $0°$ | Top of fuselage |
| $90°$ | Starboard |
| $180°$ | Bottom of fuselage |
| $270°$ | Port |

The static port angle $\phi_{port}$ (from the algorithm doc convention: above the
horizontal waterline, positive up) maps to the circle angle as:

$$\theta_{A} = 90° - \phi_{port} \quad \text{(starboard port)}$$
$$\theta_{B} = 270° + \phi_{port} \quad \text{(port-side port, mirror of A)}$$

Note: $\theta_B - \theta_A = 180° + 2\phi_{port}$. For $\phi_{port} = 0°$ the ports are
diametrically opposite (3 and 9 o'clock); for $\phi_{port} = 90°$ both are at the top.

---

**Crossflow vector.** The crossflow is the component of the body-frame airspeed vector
perpendicular to the fuselage axis. In the body cross-section plane, expressed in
$(y_{body},\ -z_{body})$ = (starboard, up) coordinates:

$$\vec{V}_c = \bigl(v_{body},\; -w_{body}\bigr)
            = \bigl(V_{TAS}\sin\beta,\; -V_{TAS}\sin\alpha\bigr)$$

where $(u, v, w)^T$ = `airspeed_body_mps` in body axes ($z$-axis down).
Crossflow magnitude and dynamic pressure:

$$V_c = V_{TAS}\sqrt{\sin^2\alpha + \sin^2\beta}$$

$$q_{cross} = \tfrac{1}{2}\rho V_{TAS}^2\!\left(\sin^2\alpha + \sin^2\beta\right)$$

---

**Stagnation angle.** The crossflow impinges on the cylinder at the point toward which
the crossflow vector points. In the circle-angle convention ($\theta = 0°$ at top,
clockwise), the stagnation angle is:

$$\delta = \operatorname{atan2}(\sin\beta,\; -\sin\alpha)$$

Verification:

- Pure $\alpha > 0$ (nose up): $\delta = \operatorname{atan2}(0, -\sin\alpha) = 180°$ → stagnation at belly ✓
- Pure $\beta > 0$ (flow from starboard): $\delta = \operatorname{atan2}(\sin\beta, 0) = 90°$ → stagnation at starboard ✓

---

**Angles from stagnation to each port:**

$$\psi_A = \theta_A - \delta = (90° - \phi_{port}) - \delta$$
$$\psi_B = \theta_B - \delta = (270° + \phi_{port}) - \delta$$

Expanding the sines:

$$\sin\psi_A = \sin\bigl((90° - \phi_{port}) - \delta\bigr) = \cos\phi_{port}\cos\delta - \sin\phi_{port}\sin\delta
            = \cos(\phi_{port} + \delta)$$

$$\sin\psi_B = \sin\bigl((270° + \phi_{port}) - \delta\bigr) = -\cos(\phi_{port} - \delta)$$

So $\sin^2\psi_A = \cos^2(\phi_{port} + \delta)$ and $\sin^2\psi_B = \cos^2(\phi_{port} - \delta)$.

---

**Averaged pressure coefficient.** The crosslinked transducer reads the average of
$C_p(\psi_A)$ and $C_p(\psi_B)$:

$$\bar{C}_p = \tfrac{1}{2}\bigl(C_p(\psi_A) + C_p(\psi_B)\bigr) = 1 - 2\bigl(\sin^2\psi_A + \sin^2\psi_B\bigr)$$

Using the identity $\cos^2 A + \cos^2 B = 1 + \cos(A+B)\cos(A-B)$ with
$A = \phi_{port} + \delta$, $B = \phi_{port} - \delta$:

$$\sin^2\psi_A + \sin^2\psi_B = 1 + \cos(2\phi_{port})\cos(2\delta)$$

$$\boxed{\bar{C}_p = -1 - 2\cos(2\phi_{port})\cos(2\delta)}$$

---

**Pressure deviation at the crosslinked static port:**

$$\Delta P_{geo} = \bar{C}_p \cdot q_{cross}
    = \bigl[-1 - 2\cos(2\phi_{port})\cos(2\delta)\bigr]
      \cdot \tfrac{1}{2}\rho V_{TAS}^2\!\left(\sin^2\alpha + \sin^2\beta\right)$$

where $\delta = \operatorname{atan2}(\sin\beta,\, -\sin\alpha)$.

**Airspeed vector inputs:**

$$V_{TAS} = \|\mathbf{v}^B_{a/w}\|, \quad
\alpha = \operatorname{atan2}(w,\, u), \quad
\beta = \operatorname{arcsin}(v / V_{TAS})$$

---

**Decomposition into $\alpha$ and $\beta$ contributions.** Evaluating at $\beta = 0$
($\delta = 180°$, $\cos 2\delta = 1$) and at $\alpha = 0$ ($\delta = 90°$,
$\cos 2\delta = -1$):

$$\Delta P_{geo}^{(\alpha)} = \bigl(-1 - 2\cos(2\phi_{port})\bigr)
                              \cdot \tfrac{1}{2}\rho V_{TAS}^2\sin^2\alpha$$

$$\Delta P_{geo}^{(\beta)} = \bigl(-1 + 2\cos(2\phi_{port})\bigr)
                             \cdot \tfrac{1}{2}\rho V_{TAS}^2\sin^2\beta
                           = \bigl(1 - 4\sin^2\phi_{port}\bigr)
                             \cdot \tfrac{1}{2}\rho V_{TAS}^2\sin^2\beta$$

The $\beta$ error factor $1 - 4\sin^2\phi_{port}$ is the same formula as the single-port
$\alpha$ error factor from the pitch-plane-only model — but rotated 90° in port angle.

---

**Port location design table.**

| $\phi_{port}$ | $\alpha$ error factor | $\beta$ error factor | Notes |
| --- | --- | --- | --- |
| $0°$ (waterline) | $-3$ | $+1$ | Ports at 3 and 9 o'clock |
| $30°$ | $-2$ | $0$ | β fully cancelled by crosslink |
| $45°$ | $-1$ | $-1$ | Equal sensitivity to α and β |
| $60°$ | $0$ | $-2$ | α fully cancelled; single-port ideal |
| $90°$ (top/bottom) | $+1$ | $-3$ | Ports at 12 and 6 o'clock |

At $\phi_{port} = 30°$, the β error is completely eliminated by the crosslinked averaging.
The remaining α error ($-2 q_{cross,\alpha}$) is non-zero and cannot be removed with a
symmetric two-port layout — it requires an asymmetric correction or a different port
location.

At $\phi_{port} = 60°$, the α error is zero (the single-port ideal location) but the β
residual is $-2 q_{cross,\beta}$.

At $\phi_{port} = 45°$, both effects are equal in magnitude, minimizing the worst-case
error across the α/β flight envelope.

---

**β cancellation mechanism.** For pure sideslip (β only), Port A (starboard) and Port B
(port) are placed symmetrically relative to the stagnation point at
$\theta_{stag} = 90°$. Port A sits at $\psi_A = -\phi_{port}$ from stagnation
(upwind side) and Port B at $\psi_B = 180° + \phi_{port}$ (downwind side). Using the
Cp symmetry $C_p(\psi) = C_p(180° + \psi + \epsilon)$ only holds exactly when
$\phi_{port} = 0$; for other port angles the two Cp values differ. However, the crosslink
averages them, and the average happens to evaluate to $(1 - 4\sin^2\phi_{port})$ for all
β magnitudes. Setting $\phi_{port} = 30°$ drives this average to zero, achieving complete
β cancellation regardless of sideslip angle. The crosslink does NOT cancel asymmetric
disturbances that are NOT symmetric about the stagnation axis (e.g., the α contribution
at $\phi_{port} \neq 60°$).

---

**Assumptions and limitations:**

- Infinite cylinder — ignores 3D pressure relief near the nose and tail.
- Potential flow — ignores viscous effects and flow separation; error increases with
  angle of attack as the real flow separates from the lee side of the fuselage.
- Circular cross-section — actual fuselages are often oval; scale by local curvature
  ratio for a more accurate correction.
- Equal tube resistance on both crosslink legs — if the pneumatic resistance from Port A
  to the junction differs from Port B, the measured pressure is a weighted average rather
  than a simple average, and the β cancellation at $\phi_{port} = 30°$ is partial rather
  than complete.
- The crosslink tube itself has a pneumatic time constant (see Pneumatic Tube Delay
  section) that introduces lag in the crosslink equilibration. The crosslink leg lag is
  additive with the main tube lag and uses the same Tustin first-order model.

---

#### Bias, Noise, and Lag

The total static pressure input to the transducer model combines the geometric error
derived above with a constant installation bias and additive noise:

$$P_s^{noisy} = P_s^{true} + \Delta P_{geo} + \epsilon_{bias} + n_{Ps},
\qquad n_{Ps} \sim \mathcal{N}(0,\,\sigma_{Ps}^2)$$

where $\Delta P_{geo}$ = fuselage crossflow pressure error (derived above),
$\epsilon_{bias}$ = `static_pressure_bias_pa` (residual installation offset not captured
by the geometric model), and $\sigma_{Ps}$ = `static_pressure_noise_pa`.

**First-order Tustin lag** (time constant $\tau_{Ps}$):

$$\alpha_{Ps} = \frac{T_s}{2\tau_{Ps} + T_s}, \qquad \beta_{Ps} = \frac{2\tau_{Ps} - T_s}{2\tau_{Ps} + T_s}$$

$${P_s^{meas}}_k = \alpha_{Ps}\left({P_s^{noisy}}_k + {P_s^{noisy}}_{k-1}\right) + \beta_{Ps}\,{P_s^{meas}}_{k-1}$$

When $\tau_{Ps} = 0$: $P_s^{meas} = P_s^{noisy}$ (identity).

---

### Pneumatic Tube Delay

The Tustin lag on the differential pressure channel models the delay introduced by the
pneumatic tubing run from the pitot probe to the differential pressure transducer. The
section below derives the lag time constant from first principles so that `AirDataConfig`
can be populated from physical tube geometry rather than measured empirically.

#### Physical Model

A pitot tube connected to a transducer by a length of small-bore pneumatic tubing forms
a first-order resistive-capacitive (RC) pneumatic network. The tube acts as a viscous
flow resistance; the transducer sensing cavity acts as a compressible volume (capacitance).
For laminar flow in a tube of circular cross-section, the Hagen-Poiseuille resistance is:

$$R_{tube} = \frac{8\,\mu\,L_{tube}}{\pi\,r_{tube}^4} \qquad (\text{Pa}{\cdot}\text{s/m}^3)$$

where $\mu$ is the dynamic viscosity of air (Pa·s), $L_{tube}$ is the tube length (m), and
$r_{tube}$ is the tube internal radius (m).

The transducer cavity of volume $V_{cav}$ stores compressible gas. For isothermal
compression, the pneumatic capacitance is:

$$C_{cav} = \frac{V_{cav}}{P} \qquad \text{(m³/Pa)}$$

where $P$ is the ambient static pressure. This capacitance varies with altitude because
lower pressure means the same cavity volume stores proportionally more volume change per
Pascal.

The resulting first-order transfer function from probe pressure $P_{probe}$ to transducer
pressure $P_{trans}$ is:

$$\frac{P_{trans}(s)}{P_{probe}(s)} = \frac{1}{\tau_{pneu}\,s + 1}$$

with time constant:

$$\tau_{pneu} = R_{tube} \cdot C_{cav} = \frac{8\,\mu\,L_{tube}\,V_{cav}}{\pi\,r_{tube}^4\,P}$$

The assumption of laminar flow (Hagen-Poiseuille) holds when the Reynolds number inside
the tube is below approximately 2300:

$$Re = \frac{2\,\rho\,Q_{mean}}{\pi\,r_{tube}\,\mu} \ll 2300$$

For typical pitot installations the pressure perturbations are small and flow velocities
in the tube are low, so the laminar assumption holds in practice.

#### Altitude Dependence

Because $\tau_{pneu} \propto 1/P$ and pressure decreases with altitude, the pneumatic
time constant grows with altitude. Relative to its sea-level value:

$$\tau_{pneu}(h) = \tau_{pneu,0} \cdot \frac{P_0}{P(h)}$$

At 5 000 m ISA ($P \approx 54\,000\,\text{Pa}$), $\tau_{pneu}$ is approximately 1.9×
its sea-level value. At 10 000 m ($P \approx 26\,500\,\text{Pa}$), approximately 3.8×.

For constant-$\tau$ implementations (the default in `AirDataConfig`), evaluate
$\tau_{pneu}$ at the nominal operating altitude. For missions spanning a wide altitude
range, the lag can be recomputed each step using `AtmosphericState::pressure_pa` via the
formula above; this requires storing $L_{tube}$, $r_{tube}$, and $V_{cav}$ in the config
rather than the precomputed $\tau$.

#### Reference Values

Dynamic viscosity of air varies weakly with temperature:

$$\mu(T) \approx \mu_{ref}\left(\frac{T}{T_{ref}}\right)^{0.76}$$

with $\mu_{ref} = 1.81 \times 10^{-5}\,\text{Pa}{\cdot}\text{s}$ at $T_{ref} = 288.15\,\text{K}$.
For the altitude range 0–10 000 m, $\mu$ varies by less than 15%; using the sea-level
value is adequate for time-constant estimation.

#### Worked Example

Typical small UAS pitot installation: $L_{tube} = 0.4\,\text{m}$,
$r_{tube} = 1.0\,\text{mm}$, $V_{cav} = 0.2\,\text{cm}^3 = 2 \times 10^{-7}\,\text{m}^3$,
sea-level ISA ($P_0 = 101\,325\,\text{Pa}$, $\mu = 1.81 \times 10^{-5}\,\text{Pa}{\cdot}\text{s}$):

$$\tau_{pneu} = \frac{8 \times 1.81\!\times\!10^{-5} \times 0.4 \times 2\!\times\!10^{-7}}
                     {\pi \times (10^{-3})^4 \times 101\,325}
             = \frac{1.157\!\times\!10^{-11}}{3.183\!\times\!10^{-7}}
             \approx 3.6\!\times\!10^{-5}\,\text{s} \approx 0.036\,\text{ms}$$

This is negligible at any flight computer update rate. For a longer or thinner tube
($L_{tube} = 1.5\,\text{m}$, $r_{tube} = 0.5\,\text{mm}$):

$$\tau_{pneu} = \frac{8 \times 1.81\!\times\!10^{-5} \times 1.5 \times 2\!\times\!10^{-7}}
                     {\pi \times (5\!\times\!10^{-4})^4 \times 101\,325}
             = \frac{4.34\!\times\!10^{-11}}{1.989\!\times\!10^{-8}}
             \approx 2.2\!\times\!10^{-3}\,\text{s} \approx 2.2\,\text{ms}$$

At this scale, the lag becomes relevant for autopilot control loops running at 100 Hz or
faster. The static pressure tube exhibits the same physics; its time constant is computed
identically with the appropriate tube geometry for the static port run.

---

### Filter Initialization on `reset()`

On `reset()`, the lag filter states are initialized to the current noiseless true values:

$${q_c^{meas}}_{-1} = q_c^{true}, \qquad {q_c^{noisy}}_{-1} = q_c^{true}$$
$${P_s^{meas}}_{-1} = P_s^{true}, \qquad {P_s^{noisy}}_{-1} = P_s^{true}$$

This places each filter in steady state at the true value. The first call to `step()` after `reset()` returns the noiseless, zero-lag derived quantities (subject only to the noise drawn on that step). Initializing to zero would cause a transient lag artifact on the first step.

---

## Derived Quantities

All derived quantities are computed from $q_c^{meas}$ and $P_s^{meas}$ (the outputs of the two transducer models), plus `AtmosphericState` fields where required.

### Mach Number

Invert the isentropic impact pressure equation using the measured pressures alone:

$$M^{meas} = \sqrt{\frac{2}{\gamma-1}\left[\left(\frac{q_c^{meas}}{P_s^{meas}} + 1\right)^{\frac{\gamma-1}{\gamma}} - 1\right]}$$

This is the ADC Mach number. It requires only the ratio of the two measured pressures; no atmospheric model is consulted. It is the central quantity from which TAS, CAS, and EAS are subsequently derived.

---

### True Airspeed (TAS)

$$V_{TAS}^{meas} = M^{meas} \cdot a$$

where $a$ = `AtmosphericState::speed_of_sound_mps`. Unlike the pure pitot-static derivation of Mach, TAS requires knowledge of the local speed of sound and therefore depends on the atmospheric model.

---

### Indicated Airspeed (IAS)

IAS is the raw instrument indication. It applies the incompressible Bernoulli equation referenced to sea-level ISA density $\rho_0$:

$$V_{IAS} = \sqrt{\frac{2\,q_c^{meas}}{\rho_0}}$$

IAS does not apply any compressibility correction. At low Mach ($M < 0.3$), the compressibility error relative to CAS is less than 1.7%. At $M = 0.3$, $V_{IAS}$ underreads $V_{CAS}$ by approximately 1.7%.

---

### Calibrated Airspeed (CAS)

CAS removes the compressibility error inherent in IAS by applying the full isentropic formula, but still referenced to sea-level ISA conditions. CAS is defined as the airspeed that would produce the measured impact pressure $q_c^{meas}$ under sea-level ISA:

$$V_{CAS} = a_0\sqrt{\frac{2}{\gamma-1}\left[\left(\frac{q_c^{meas}}{P_0} + 1\right)^{\frac{\gamma-1}{\gamma}} - 1\right]}$$

At sea-level ISA ($P_s = P_0$, $\rho = \rho_0$), CAS = TAS. At altitude, CAS < TAS because the lower ambient pressure reduces $q_c$ for the same TAS.

---

### Equivalent Airspeed (EAS)

EAS is defined as the airspeed that produces the same aerodynamic dynamic pressure at sea-level ISA density as the actual flight dynamic pressure:

$$\frac{1}{2}\rho_0 V_{EAS}^2 = \frac{1}{2}\rho V_{TAS}^2$$

$$V_{EAS} = V_{TAS}^{meas}\sqrt{\frac{\rho}{\rho_0}}$$

where $\rho$ = `AtmosphericState::density_kgm3`. EAS is the airspeed relevant to aerodynamic force calculations; the aerodynamic loads on the airframe are proportional to $\tfrac{1}{2}\rho_0 V_{EAS}^2$.

EAS and CAS are related by the compressibility correction factor $f_c$:

$$V_{EAS} = V_{CAS} \cdot f_c$$

$$f_c = \sqrt{\frac{P_s^{meas}}{P_0} \cdot \frac{\left(q_c^{meas}/P_s^{meas} + 1\right)^{\frac{\gamma-1}{\gamma}} - 1}{\left(q_c^{meas}/P_0 + 1\right)^{\frac{\gamma-1}{\gamma}} - 1}}$$

At sea level ($P_s = P_0$), $f_c = 1$ and EAS = CAS. At altitude with $M < 0.3$, $f_c$ deviates from 1 by less than 2%.

The implementation uses the $V_{TAS} \cdot \sqrt{\rho/\rho_0}$ formula, which is exact given $V_{TAS}^{meas}$ and $\rho$ from the atmospheric state. The $V_{CAS} \cdot f_c$ formula is provided here as a cross-reference for validation.

---

### Barometric Altitude

The altimeter inverts the ISA pressure–altitude relationship using the Kollsman setting
$P_{Koll}$ as the sea-level pressure reference. No temperature correction is applied —
the standard definition of barometric altitude assumes ISA temperature regardless of the
actual day. Non-standard temperature and non-standard surface pressure both cause the
indicated altitude to differ from geometric altitude; this is physically correct and is
addressed by the Kollsman setting (for pressure) and by standard altimetry corrections
(for temperature).

**Troposphere** ($P_s^{meas} > P_{Koll} \cdot P_{11000}/P_0$):

$$h_{indicated} = \frac{T_0}{L}\left[1 - \left(\frac{P_s^{meas}}{P_{Koll}}\right)^{\frac{R_d L}{g_0}}\right]$$

with temperature lapse rate $L = -0.006\,5\,\text{K/m}$ and exponent:

$$\frac{R_d L}{g_0} = \frac{287.058 \times (-0.006\,5)}{9.806\,65} = -0.190\,263$$

The tropopause boundary pressure scales with the Kollsman setting:
$P_{11000}^{Koll} = P_{11000} \cdot P_{Koll}/P_0 = 22\,632.1 \cdot P_{Koll}/P_0\,\text{Pa}$.

**Tropopause / lower stratosphere** ($P_s^{meas} \leq P_{Koll} \cdot P_{11000}/P_0$):

$$h_{indicated} = 11\,000 - \frac{R_d T_{trop}}{g_0}\ln\frac{P_s^{meas}}{P_{11000}^{Koll}}$$

with $T_{trop} = 216.65\,\text{K}$.

When $P_{Koll} = P_0 = 101\,325\,\text{Pa}$ (ISA standard, the default), both formulas
reduce to the standard pressure altitude formula with $P_{11000} = 22\,632.1\,\text{Pa}$.

---

### Kollsman Correction and Non-Standard Surface Pressure

On a day with non-standard surface pressure $P_{sfc} = P_0 + \Delta P_{sfc}$, the static
pressure at geometric altitude $h_{geo}$ is (from the atmosphere model):

$$P_s^{true}(h_{geo}) = P_{ISA}(h_{geo}) \cdot \frac{P_{sfc}}{P_0}$$

**With correctly set Kollsman** ($P_{Koll} = P_{sfc}$):

$$\frac{P_s^{true}}{P_{Koll}} = \frac{P_{ISA}(h_{geo}) \cdot P_{sfc}/P_0}{P_{sfc}} = \frac{P_{ISA}(h_{geo})}{P_0}$$

The indicated altitude equals the standard pressure altitude of the geometric altitude —
correct MSL altitude to within the temperature error (which is independent of the Kollsman
setting). The non-standard surface pressure is fully cancelled.

**With stale or incorrect Kollsman** ($P_{Koll} \neq P_{sfc}$):

Differentiating the tropospheric formula with respect to $P_{Koll}$ at $P_{Koll} = P_0$:

$$\frac{\partial h_{indicated}}{\partial P_{Koll}} = \frac{R_d T_0}{g_0 P_0} \approx \frac{287.058 \times 288.15}{9.806\,65 \times 101\,325} \approx 8.23\,\text{m/Pa}$$

The altitude error from an incorrect Kollsman is therefore:

$$\Delta h_{Koll} \approx 8.23\,\frac{\text{m}}{\text{Pa}} \times (P_{sfc} - P_{Koll})$$

$$\approx 8.23\,\frac{\text{m}}{\text{hPa}} \times (P_{sfc} - P_{Koll})/100$$

**Sign convention:** $P_{sfc} > P_{Koll}$ (high-pressure day, Kollsman not updated) → $\Delta h_{Koll} > 0$ → altimeter **underreads** (indicates lower than true MSL altitude). $P_{sfc} < P_{Koll}$ → altimeter **overreads** — the operationally dangerous case (aircraft believes it is higher than it is).

Representative errors:

| Condition | $P_{sfc} - P_{Koll}$ | Altitude error |
| --- | --- | --- |
| High pressure (+10 hPa), $P_{Koll} = P_0$ | +1 000 Pa | −82 m (underread) |
| Low pressure (−10 hPa), $P_{Koll} = P_0$ | −1 000 Pa | +82 m (overread) |
| Stale QNH, 5 hPa off | ±500 Pa | ±41 m |
| Transition from MSL to local QFE (400 m field elevation at ISA) | varies | ~400 m offset |

The Kollsman error is independent of altitude (the 8.23 m/Pa scale factor is essentially
constant through the troposphere to within ~10% variation). It is also independent of
temperature: the $\Delta T$ and $\Delta P_{Koll}$ errors are additive to first order.

**INS aiding.** An external source of geometric altitude — GPS (WGS84 altitude corrected
for geoid undulation) or a laser altimeter / radar altimeter with terrain database — can
back-compute the correct QNH and update the Kollsman setting:

$$P_{Koll}^{updated} = P_s^{meas} \cdot \left(\frac{T_0}{T_0 + L\,h_{geo}^{ext}}\right)^{R_d L / g_0}$$

where $h_{geo}^{ext}$ is the externally measured geometric altitude. When the INS applies
this update, the altimeter's indicated altitude converges to the true MSL altitude (subject
to remaining temperature error). See `docs/architecture/sensor_air_data.md §setKollsman()`
for the runtime interface.

---

### Outside Air Temperature (OAT)

OAT is the static ambient temperature with additive Gaussian noise:

$$T_{OAT} = T_s + n_{OAT}, \qquad n_{OAT} \sim \mathcal{N}(0,\,\sigma_{OAT}^2)$$

where $T_s$ = `AtmosphericState::temperature_k` and $\sigma_{OAT}$ = `oat_noise_k`. No lag is applied.

The total air temperature (TAT) recovery correction $T_{TAT} = T_s\!\left(1 + \frac{\gamma-1}{2}r\,M^2\right)$, where $r$ is the probe recovery factor, is not modeled. For $M < 0.3$ the recovery correction is less than 2% of $T_s$ even at $r = 1$, and is negligible for the flight regimes of interest.

---

## Non-Standard Temperature: Errors Without an OAT Sensor

An aircraft equipped with only a pitot-static system — no OAT probe — must substitute a
temperature assumption to derive TAS and air density. The standard substitute is ISA
temperature at the current barometric altitude:

$$T_s^{ADC} = T_{ISA}(h_{baro}) = T_0 + L\,h_{baro} \qquad \text{(troposphere)}$$

When the atmosphere deviates from ISA by $\Delta T = T_s^{true} - T_{ISA}(h_{geo})$, this
substitution introduces a systematic error in every derived quantity that requires
temperature. This section identifies which quantities are affected and quantifies the error.

---

### Temperature-Independent Quantities

Five of the seven `AirDataMeasurement` fields depend only on $q_c^{meas}$ and
$P_s^{meas}$ — no temperature input at any stage:

| Quantity | Formula (pressure-only form) | Temperature needed? |
| --- | --- | --- |
| Mach | $M = \sqrt{\frac{2}{\gamma-1}\bigl[(q_c/P_s + 1)^{(\gamma-1)/\gamma} - 1\bigr]}$ | No |
| IAS | $V_{IAS} = \sqrt{2\,q_c/\rho_0}$ | No |
| CAS | $V_{CAS} = a_0\sqrt{\frac{2}{\gamma-1}\bigl[(q_c/P_0 + 1)^{(\gamma-1)/\gamma} - 1\bigr]}$ | No |
| EAS | $V_{EAS} = M\sqrt{\gamma P_s/\rho_0}$ | No |
| Baro altitude | $h_{baro} = f(P_s)$ via ISA inversion | No |

The EAS pressure-only form follows directly from substituting $a = \sqrt{\gamma R_d T_s}$
and $\rho = P_s / (R_d T_s)$ into $V_{EAS} = V_{TAS}\sqrt{\rho/\rho_0} = M\,a\sqrt{\rho/\rho_0}$:

$$V_{EAS} = M\sqrt{\gamma R_d T_s} \cdot \sqrt{\frac{P_s}{R_d T_s\,\rho_0}} = M\sqrt{\frac{\gamma P_s}{\rho_0}}$$

The temperature $T_s$ cancels exactly. A pitot-static system with no temperature sensor
computes IAS, CAS, EAS, Mach, and baro altitude without any error attributable to
non-standard conditions.

---

### TAS Error

TAS is the only `AirDataMeasurement` field that cannot be derived from pressure alone:

$$V_{TAS} = M \cdot a = M\sqrt{\gamma R_d T_s^{true}}$$

Mach is measured correctly regardless of temperature; the error enters only through the
speed-of-sound factor. An ADC substituting $T_s^{ADC}$ computes:

$$V_{TAS}^{ADC} = M^{meas}\sqrt{\gamma R_d\,T_s^{ADC}}$$

The relative TAS error is:

$$\frac{V_{TAS}^{ADC} - V_{TAS}^{true}}{V_{TAS}^{true}}
  = \sqrt{\frac{T_s^{ADC}}{T_s^{true}}} - 1
  \approx \frac{T_s^{ADC} - T_s^{true}}{2\,T_s^{true}}
  = \frac{-\Delta T}{2(T_{ISA} + \Delta T)}$$

The sign is negative: a warm day ($\Delta T > 0$) means $T_s^{ADC} < T_s^{true}$, so the ADC
**underreads TAS**. A cold day ($\Delta T < 0$) causes the ADC to **overread TAS**.

Representative values (first-order approximation $\Delta T \ll T_{ISA}$):

| Condition | $\Delta T$ (K) | $T_{ISA}$ at SL (K) | $ | \delta V_{TAS}/V_{TAS} | $ |
| --- | --- | --- | --- |
| ISA+20 K, sea level | +20 | 288.15 | 3.5 % |
| ISA−20 K, sea level | −20 | 288.15 | 3.6 % (overread) |
| ISA+20 K, 5 000 m | +20 | 255.65 | 3.9 % |
| ISA+20 K, 10 000 m | +20 | 223.15 | 4.5 % |
| ISA+30 K, sea level | +30 | 288.15 | 5.2 % |

The error grows with altitude because $T_{ISA}$ decreases, making the $\Delta T / 2 T_{ISA}$
fraction larger. At the tropopause ($T_{ISA} = 216.65\,\text{K}$), even ISA+15 K gives a
3.5% TAS error identical in magnitude to ISA+20 K at sea level.

---

### Compound Error: Baro Altitude Coupling

On a warm day the atmosphere is less dense, so the pressure at any given geometric altitude
is lower than ISA predicts. The barometric altimeter inverts the ISA pressure–altitude
relationship and therefore reads **lower** than the true geometric altitude:

$$h_{baro} < h_{geo} \quad \text{(warm day)}$$

Because the ADC's temperature assumption $T_s^{ADC} = T_{ISA}(h_{baro})$ is evaluated at
the (already low) barometric altitude, rather than at the true altitude, the assumed
temperature is lower than $T_{ISA}(h_{geo})$, which is itself lower than $T_s^{true}$.
Both errors act in the same direction on a warm day:

$$T_s^{ADC} = T_{ISA}(h_{baro}) < T_{ISA}(h_{geo}) < T_s^{true}$$

Approximating the warm-day baro–altitude error as
$h_{baro} \approx h_{geo}\,T_{ISA} / (T_{ISA} + \Delta T)$ (constant ΔT column
approximation), the compound temperature assumption error is:

$$T_s^{ADC} \approx T_0 + L\,h_{geo}\frac{T_{ISA}}{T_{ISA} + \Delta T}$$

$$T_s^{ADC} - T_s^{true} \approx -\Delta T \left(1 + \frac{L\,h_{geo}}{T_{ISA} + \Delta T}\right)$$

The second term in the parenthesis is typically small at low altitudes. At 3 000 m ISA+20 K:
$L\,h_{geo}/(T_{ISA}+\Delta T) = (-0.0065 \times 3000)/289.4 \approx -0.067$, so the
compound factor is $1 - 0.067 = 0.93$ — a 7% reduction of the ΔT contribution compared to
the first-order estimate. The coupling partially self-compensates because the lower baro
altitude corresponds to a slightly warmer ISA temperature.

The baro altitude error itself — independently of TAS — has operational consequences:
obstacle clearance margins are based on indicated altitude, and the altitude error can reach
tens of meters on hot days at high terrain.

---

### Density Altitude Without OAT

Density altitude $h_d$ is the ISA altitude at which the ISA density equals the actual
density $\rho = P_s / (R_d T_s^{true})$. Without $T_s^{true}$, the actual density cannot
be computed from the pitot-static measurements alone. An ADC that assumes ISA temperature
computes:

$$\rho^{ADC} = \frac{P_s}{R_d\,T_{ISA}(h_{baro})}$$

On a warm day, $T_{ISA}(h_{baro}) < T_s^{true}$, so $\rho^{ADC} > \rho^{true}$: the ADC
**overestimates density** and therefore **underestimates density altitude**. Engine and
propeller performance is degraded more than the ADC indicates.

An external TAS source (e.g., GPS ground speed corrected for wind) can provide
$V_{TAS}^{GPS}$, from which the ADC can back-compute $T_s^{true} = (V_{TAS}^{GPS}/M)^2 / (\gamma R_d)$
and then derive the correct density.

---

### Summary

| Quantity | Depends on temperature? | Error on non-standard day (no OAT) |
| --- | --- | --- |
| Mach | No | None |
| IAS | No | None |
| CAS | No | None |
| EAS | No | None |
| Baro altitude (standard pressure altitude, $P_{Koll} = P_0$) | No | +8.23 m/Pa × $(P_{sfc} - P_0)$; positive $\Delta P_{sfc}$ → underread |
| Baro altitude (with correct $P_{Koll} = P_{sfc}$) | No | None — pressure error fully cancelled |
| TAS | Yes | $\approx \Delta T / (2 T_{ISA})$; warm day → underread |
| OAT | Yes (it IS temperature) | Not applicable — absent |
| Density altitude | Yes | Underestimated on warm day; requires GPS-TAS to correct |

---

### Non-Standard Surface Pressure

Non-standard surface pressure $\Delta P_{sfc}$ affects only the barometric altitude output
and only when the Kollsman setting does not match the actual surface pressure. All other
outputs — Mach, IAS, CAS, EAS, TAS — are determined by the ratio $q_c/P_s$ or by $q_c$
alone. Multiplying both $q_c$ and $P_s$ by the same factor $P_{sfc}/P_0$ leaves all
pressure ratios unchanged. Therefore:

- **TAS** is unaffected by $\Delta P_{sfc}$ (only Mach and speed of sound determine TAS;
  Mach is a ratio).
- **EAS** is unaffected — $V_{EAS} = M\sqrt{\gamma P_s/\rho_0}$, and the $P_s$ in this
  formula is the actual measured pressure, giving the correct dynamic pressure.
- **Baro altitude** IS affected by $\Delta P_{sfc}$ when $P_{Koll} \neq P_{sfc}$.

The combined altimetry error on a non-standard day (both $\Delta T \neq 0$ and
$P_{Koll} \neq P_{sfc}$) is to first order the sum of the two independent errors:

$$\Delta h_{total} \approx \underbrace{h_{geo} \cdot \frac{-\Delta T}{T_{ISA}}}_{\text{temperature (§above)}} + \underbrace{8.23\,\frac{\text{m}}{\text{Pa}} \cdot (P_{sfc} - P_{Koll})}_{\text{Kollsman/pressure}}$$

The temperature term grows with altitude; the Kollsman term is altitude-independent to
first order. For long-range operations at high altitude with large ΔT, the temperature
term dominates. For low-level terrain following with large pressure gradient (weather
front), the Kollsman term may dominate.

### Applicability to `SensorAirData`

Within `SensorAirData`, TAS and EAS are computed from the `AtmosphericState` passed in by
the SimulationLoop, which carries the **true** speed of sound and density. The sensor
therefore produces correct TAS and EAS regardless of OAT noise or the absence of an OAT
channel (`oat_noise_k = 0` with `oat_k` unused by the caller).

The errors in this section arise at the **vehicle computer level**: an avionics processor
that consumes the raw `AirDataMeasurement` outputs and lacks access to the atmospheric
state. Such a processor would compute TAS as
$M^{meas} \times \sqrt{\gamma R_d T_{ISA}(h_{baro})}$, introducing the error quantified
above. To reproduce this scenario in simulation, the vehicle model should apply the ISA
substitution to `AirDataMeasurement::mach_nd` and `baro_altitude_m` rather than using
`tas_mps` directly.

---

## Noise Propagation

These expressions are approximate linearized propagation formulas. They are provided as a reference for test design (T5, T6 in the architecture document) and are not used in the implementation.

**IAS from $q_c$ noise:**

Differentiating $V_{IAS} = \sqrt{2q_c/\rho_0}$:

$$\delta V_{IAS} = \frac{1}{\sqrt{2\rho_0 q_c}}\,\delta q_c = \frac{1}{\rho_0 V_{IAS}}\,\delta q_c$$

IAS noise is inversely proportional to airspeed. At very low airspeed ($V_{IAS} \to 0$), a fixed $\sigma_{qc}$ produces unbounded IAS uncertainty. This is physically correct — a pitot-static system is inherently imprecise at low dynamic pressures.

**Barometric altitude from $P_s$ noise (troposphere):**

Differentiating the tropospheric baro altitude formula:

$$\delta h_{baro} = -\frac{T_0}{g_0}\frac{R_d}{P_s}\left(\frac{P_s}{P_0}\right)^{R_d L/g_0}\,\delta P_s$$

At sea level this simplifies to $\delta h_{baro} \approx -R_d T_0 / (g_0 P_0)\,\delta P_s \approx -8.43\,\text{m/Pa} \times \delta P_s$. At altitude, the lower pressure amplifies the altitude noise per Pascal — higher altitude means larger altitude uncertainty for the same $\sigma_{Ps}$.

---

## Implementation Notes

- All floating-point arithmetic uses `float` (32-bit IEEE 754) for consistency with the rest of the Domain Layer. All ISA constants are `static constexpr float`.
- When $q_c^{meas} < 0$ — possible when noise exceeds the true impact pressure at near-zero airspeed — clamp $q_c^{meas}$ to 0.0f before airspeed derivation. All airspeed outputs (IAS, CAS, EAS, TAS) and Mach are set to 0.0f for zero impact pressure.
- When $\tau = 0$ for either channel, skip the lag recurrence entirely and set $q_c^{meas} = q_c^{noisy}$ (or $P_s^{meas} = P_s^{noisy}$). Do not evaluate the Tustin coefficients in this case; the formula is singular as $\tau \to 0$.
- ISA boundary constants as `static constexpr float`:
  - $P_0 = 101\,325\,\text{Pa}$
  - $P_{11000} = 22\,632.1\,\text{Pa}$
  - $\rho_0 = 1.225\,\text{kg/m}^3$
  - $a_0 = 340.294\,\text{m/s}$
  - $T_0 = 288.15\,\text{K}$
  - $T_{trop} = 216.65\,\text{K}$
- RNG: `std::mt19937` engine with `std::normal_distribution<float>`, wrapped in a `RngState` pimpl (same pattern as `Turbulence`). The destructor of the owning class must be defined in the `.cpp` translation unit.
- The three noise draws per step ($n_{qc}$, $n_{Ps}$, $n_{OAT}$) are made in a fixed order each step. This order must not change between releases, as it determines the serialized advance count semantics.
