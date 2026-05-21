# PropulsionCoeffEstimator — Design Document

This document is the design authority for the estimation of propulsion model parameters
from first-principles data, manufacturer specifications, and databases. It is the
propulsion-system parallel to
[`aero_coeff_estimator.md`](aero_coeff_estimator.md) and should be read alongside it.

**Scope:** parameter estimation for `PropulsionProp` (electric and piston motor),
`PropulsionJet`, and `PropulsionEDF`. Estimation of the **propulsion-aero coupling
coefficients** that enter `BodyAxisCoeffModel` when thrust and slipstream effects are
non-negligible. See [`aero_coefficient_model.md`](aero_coefficient_model.md) for the
coefficient format.

**Relationship to other documents:**

- Propulsion class hierarchy and physics models:
  [`docs/architecture/propulsion.md`](propulsion.md)
- Aerodynamic coefficient format and sign conventions:
  [`docs/architecture/aero_coefficient_model.md`](aero_coefficient_model.md)
- Aerodynamic coefficient estimation methods:
  [`docs/architecture/aero_coeff_estimator.md`](aero_coeff_estimator.md)
- Roadmap item that drives this work:
  [`docs/roadmap/aircraft.md`](../roadmap/aircraft.md) — item 1 (Aerodynamic and
  Propulsion Coefficient Design Study)

---

## What Needs to Be Estimated

Each propulsion model type has a distinct parameter set. The table below maps each
parameter to its estimation path.

### `PropulsionProp` + `MotorElectric`

| Parameter | Symbol | Source path | Notes |
| --- | --- | --- | --- |
| `kv_rps_per_v` | $K_V$ | Manufacturer spec sheet | Typically listed in RPM/V; convert: $K_V [\text{rad/s/V}] = K_V [\text{RPM/V}] \cdot \pi/30$ |
| `r_terminal_ohm` | $R_m$ | Spec sheet or bench measurement | Often $0.01–0.3\,\Omega$; measured with milliohm meter at terminals |
| `inertia_motor_kg_m2` | $I_m$ | Hollow cylinder estimate or pendulum test | Rarely on spec sheet; estimate: $I_m = \tfrac{1}{2}m_\text{rotor}(r_i^2+r_o^2)$ |
| `supply_voltage_v` | $V_s$ | Battery cell count × nominal voltage | LiPo nominal: 3.7 V/cell; HV: 3.85 V/cell |
| `i_max_a` | $I_{max}$ | ESC spec sheet | Continuous current rating |
| `esc_efficiency_nd` | $\eta_{ESC}$ | ESC datasheet or benchmark | Typically 0.90–0.98 |

### `PropulsionProp` — Propeller (`PropellerAero`)

| Parameter | Symbol | Source path | Notes |
| --- | --- | --- | --- |
| `diameter_m` | $D$ | Prop spec (stamped on hub) | |
| `pitch_m` | $p$ | Prop spec | Geometric pitch; $p = D \times \text{pitch-in-inches} \times 0.0254$ |
| `blade_count` | $N_b$ | Physical inspection | |
| `blade_solidity` | $\sigma$ | Geometry measurement or database | $\sigma = N_b \bar{c}/(\pi R)$; $\bar{c}$ from planform image or UIUC entry |
| $C_T(J)$, $C_Q(J)$ curves | — | UIUC database (primary); BET (secondary) | See estimation methods below |

### `PropulsionProp` + `MotorPiston`

| Parameter | Symbol | Source path | Notes |
| --- | --- | --- | --- |
| `power_max_w` | $P_{max}$ | Engine spec sheet | Sea-level maximum power |
| `peak_omega_rps` | $\Omega_\text{peak}$ | Spec sheet peak RPM; convert to rad/s | |
| `altitude_exponent` | $n_{alt}$ | Engine type: 1.0 for naturally aspirated, <1 for turbocharged | Naturally aspirated: $P \propto (\rho/\rho_{SL})^{1.0}$ |
| `inertia_kg_m2` | $I$ | Crankshaft data or estimate from engine mass | |

### `PropulsionJet`

| Parameter | Symbol | Source path | Notes |
| --- | --- | --- | --- |
| `thrust_sl_n` | $T_{SL}$ | Engine spec sheet (SLS thrust) | Convert from lbf if needed ($1\,\text{lbf} = 4.448\,\text{N}$) |
| `bypass_ratio` | BPR | Engine documentation | Determines density exponent: $n = 1/\sqrt{1+\text{BPR}}$ |
| `inlet_area_m2` | $A_{inlet}$ | Physical measurement: $\pi r_{inlet}^2$ for circular inlets | |
| `idle_fraction` | $f_{idle}$ | Spec sheet or literature for engine class | Typically 0.04–0.10 |
| `spool_tau_s` | $\tau_{spool}$ | Step response test or engine class literature | 1–6 s for gas turbines |
| `ab_thrust_sl_n` | $T_{AB,SL}$ | Spec sheet or ratio: typically 20–100% of dry $T_{SL}$ | 0 if no afterburner |
| `ab_spool_tau_s` | $\tau_{AB}$ | Literature for engine class | 0.5–2 s |

### `PropulsionEDF`

| Parameter | Symbol | Source path | Notes |
| --- | --- | --- | --- |
| `thrust_sl_n` | $T_{SL}$ | Static thrust test or spec sheet | |
| `fan_diameter_m` | $D_{fan}$ | Physical measurement | Sets actuator disk area |
| `inlet_area_m2` | $A_{inlet}$ | Duct geometry | |
| `idle_fraction` | $f_{idle}$ | Spec or default 0.05 | |
| `rotor_tau_s` | $\tau_{rotor}$ | Bench step-response test | Much shorter than gas turbine: 0.05–0.5 s |
| `supply_voltage_v`, `fan_efficiency_nd`, `esc_efficiency_nd` | — | Battery spec; actuator-disk benchmark; ESC spec | |

---

## Estimation Methods

### Method P1 — Manufacturer Specification and Datasheets (Primary)

Most propulsion parameters are available directly from manufacturer documentation for
commercial UAV components:

- **Electric motors**: KV, $R_m$, max current, mass, dimensions — universally published.
  Inertia is the notable exception.
- **ESCs**: max current, supply voltage range, efficiency (sometimes) — from spec sheet.
- **Propellers**: diameter, pitch, blade count — from spec label. $C_T/C_Q$ curves —
  usually not published; must use database or test.
- **Jet engines** (RC/UAV scale, e.g. JetCat, AMT): SLS thrust, spool time constant,
  idle fraction — from manufacturer documentation.
- **Piston engines**: maximum power, peak RPM, fuel consumption — from spec sheet.

**Limitation.** Datasheets vary in completeness and accuracy. Motor KV and $R_m$ values
are often measured at a single operating point; actual performance varies with temperature
and loading.

---

### Method P2 — UIUC Propeller Database

The University of Illinois Propeller Database (UIUC PROPDB) provides wind-tunnel-measured
$C_T(J)$ and $C_Q(J)$ curves for several hundred commercial propellers at multiple
airspeeds and RPM settings, covering propeller diameters from 4 to 22 inches — the full
range relevant to small UAVs.

**Coverage.** APC, Graupner, Master Airscrew, XOAR, and other common brands.

**Data format.** Tabular $C_T$ vs $J$ and $C_Q$ vs $J$ at multiple RPM values; also
static (zero airspeed) thrust and torque.

**Integration path.** The parabolic $C_T(J)$ model in `PropellerAero` is a two-parameter
fit ($C_{T0}$, $J_0$) to the UIUC measured curve. A `UiucPropDbReader` Python module
would:

1. Load the UIUC tabular data for a given propeller.
2. Fit the parabolic model $C_T = C_{T0}(1 - J/J_0)^2$ via least-squares.
3. Fit the torque model $C_Q = C_{Q0} + k_{CQ} J^2$.
4. Output `PropellerAero` parameter struct (JSON).

**Limitation.** Not all propellers are in the database. Static thrust data ($J=0$) may
not extrapolate accurately to high-$J$ cruise conditions.

---

### Method P3 — Blade Element Theory (BET)

BET divides the propeller blade into radial strips and applies 2D airfoil theory to each
strip, integrating to obtain $C_T$ and $C_Q$ as functions of advance ratio.

**Inputs required.**

| Input | Source |
| --- | --- |
| Chord distribution $c(r/R)$ | Blade geometry from caliper measurement or planform scan |
| Twist distribution $\theta(r/R)$ | Measured from physical blade or spec (if available) |
| Section $C_l(\alpha)$, $C_d(\alpha)$ | Airfoil database (NACA, UIUC Airfoil Database) |
| Tip loss correction | Prandtl tip loss factor $F(r)$ |

**Accuracy.** ±5–15% on $C_T$, ±10–20% on $C_Q$ without correction; better with
empirical tip-loss and stall-delay calibration.

**Tools of interest.**

| Tool | Language | License | Notes |
| --- | --- | --- | --- |
| QPROP (Drela, MIT) | Fortran | Free | BET + BEMT for prop-motor system matching; widely used |
| JavaProp | Java (web) | Free | Online BET tool; good for preliminary sizing |
| OpenProp | MATLAB/Python | MIT | Lifting-line propeller design; outputs $C_T$, $C_Q$ |
| XROTOR (Drela) | Fortran | Free | More advanced than QPROP; noise prediction included |

---

### Method P4 — Bench Test Measurement

For configurations where database data is unavailable (custom props, large UAV
propulsion systems), bench tests provide direct measurement of key parameters:

| Measurement | Equipment | Parameters obtained |
| --- | --- | --- |
| Static thrust and torque vs throttle | Thrust stand + torque sensor | $C_{T0}$, max thrust at SL |
| Thrust vs airspeed | Wind tunnel or vehicle tow test | $C_T(J)$ curve |
| Step throttle response | Thrust stand + data logger | $\tau_{spool}$, $\tau_{rotor}$ |
| Motor speed vs voltage | Tachometer + PSU | $K_V$ (cross-check vs spec) |
| Motor resistance | Milliohm meter | $R_m$ |
| Motor inertia | Deceleration test or torsional pendulum | $I_m$ |

---

### Method P5 — Actuator Disk and Momentum Theory (EDF / Fan)

For the EDF model, actuator disk theory provides first-principles estimates of
$T_{SL}$, $\eta_{fan}$, and current draw without requiring test data:

$$
T_{SL} = \sqrt{2\,\rho_{SL}\,A_{disk}\,P_{fan}^2 \cdot \eta_{fan}}
$$

$$
v_i = \sqrt{\frac{T}{2\,\rho\,A_{disk}}}
\quad \text{(hover / static)}
$$

This is useful for sizing the EDF model from shaft power specifications when static
thrust has not been measured directly.

---

## Propulsion-Aerodynamic Coupling

This is the interface between the propulsion estimator and the aerodynamic coefficient
model. When propulsion and aerodynamics are modeled separately, coupling effects that
depend on both must be computed and fed into one or both models. The effects below are
all physically significant for UAV configurations and appear in the `Aircraft6DOF`
equations of motion.

### Thrust Line Offset — Pitching Moment

If the thrust line does not pass through the center of gravity, thrust produces a direct
pitching moment:

$$
\Delta M_y = T \cdot z_{T}, \qquad \Delta M_z = T \cdot y_{T}
$$

where $z_T$ and $y_T$ are the body-frame offsets of the thrust application point from
the CG ($z_T > 0$ = thrust point below CG in body frame; $y_T > 0$ = right of CG). For
a tractor propeller mounted on the nose centerline with CG below the thrust line,
$z_T < 0$ (body $z$ down), so $\Delta M_y > 0$ — a nose-up pitching moment.

This is **not** a stability derivative — it is an additive moment term that depends on
instantaneous thrust $T$, not on aerodynamic state. `Aircraft6DOF` must add it
explicitly rather than folding it into $C_m$.

### Propwash — Tail Effectiveness Augmentation

A tractor propeller generates a high-velocity slipstream that increases dynamic pressure
over the horizontal and vertical tails, augmenting their aerodynamic effectiveness:

$$
\eta_{HT,\text{eff}} = \eta_{HT} + \Delta\eta_{HT}(\delta_T)
$$

The increment is proportional to the kinetic energy added by the propeller:

$$
\Delta\eta_{HT}(\delta_T) = k_{slip} \cdot \frac{T}{\frac{1}{2}\rho V^2 S_{HT}}
$$

where $k_{slip}$ depends on the fraction of the tail span immersed in the slipstream and
the axial velocity distribution of the wake. This augments $C_{m_q}$, $C_{Z_q}$, and
$C_{m_{\delta_e}}$ at high throttle and low airspeed — the takeoff and climbout
conditions where propwash is strongest.

The corrected stability derivatives are:

$$
C_{Z_q,\text{eff}} = C_{Z_q} + \Delta C_{Z_q}(\delta_T), \qquad
C_{m_q,\text{eff}} = C_{m_q} + \Delta C_{m_q}(\delta_T)
$$

**Estimation.** $k_{slip}$ can be estimated from actuator disk theory for the axial
velocity distribution at the tail station:

$$
k_{slip} \approx \frac{S_{prop,\text{intersect}}}{S_{HT}}
\left(1 + \sqrt{1 + \frac{2T}{\rho V^2 A_{prop}}}\right) - 1
$$

where $S_{prop,\text{intersect}}$ is the projected propeller disk area that intersects
the horizontal tail planform.

### P-Factor — Propeller Asymmetric Thrust at Angle of Attack

At non-zero angle of attack, the advancing blade (moving downward relative to the
aircraft) has a higher effective velocity than the retreating blade. This creates an
asymmetric thrust distribution and a net yawing moment (P-factor):

$$
\Delta C_n = C_{n_P} \cdot \frac{C_{T} D^2}{S b} \cdot \alpha
$$

For a right-hand tractor propeller at positive $\alpha$: the advancing blade is on the
right side → more thrust on the right → yawing moment to the left →
$C_{n_P} < 0$ for positive $\alpha$.

The effect is proportional to $\alpha$ and to the thrust loading $T/(q_\infty S b)$.
It is significant at high throttle, high AoA — exactly the takeoff and missed-approach
conditions that `Aircraft6DOF` needs to model.

### Gyroscopic Moment — Spinning Propeller or Fan

A spinning propeller has angular momentum $H = I_{prop}\,\Omega$. A pitch rate $q$ on
the aircraft creates a gyroscopic yawing moment:

$$
M_{z,gyro} = I_{prop}\,\Omega\,q
$$

Similarly, a yaw rate $r$ creates a gyroscopic pitching moment:

$$
M_{y,gyro} = -I_{prop}\,\Omega\,r
$$

These terms are additive to the aerodynamic moments in the equations of motion.
For a large, slow-spinning propeller (high $I_{prop}$, high $\Omega$) or at high
pitch/yaw rates, the gyroscopic terms are non-negligible.

### Slipstream Rolling Moment

The rotating propeller wake imparts angular momentum to the airframe, creating a steady
rolling moment proportional to thrust:

$$
\Delta C_l = -C_{l_T} \cdot \frac{T}{q_\infty S b}
$$

where $C_{l_T}$ is a small coefficient (typically $|C_{l_T}| < 0.1$) that depends on
the swirl velocity in the wake and the fraction of the wing immersed in the slipstream.
The sign convention: for a right-hand tractor propeller (viewed from behind, rotates
clockwise), the wake rotation is clockwise → the reaction on the aircraft is
counter-clockwise → left roll → $\Delta C_l < 0$.

### Multi-Engine Asymmetric Thrust

For multi-engine configurations, asymmetric throttle creates a lateral thrust differential
that generates yaw and (to a lesser extent) roll moments:

$$
\Delta M_z = \sum_i T_i \cdot y_{T,i}, \qquad \Delta M_x = \sum_i T_i \cdot z_{T,i}
$$

where $y_{T,i}$ and $z_{T,i}$ are the lateral and vertical offsets of engine $i$ from
the CG. This is an additive thrust moment, not a stability derivative.

### Summary: Which Effects Enter Which Model

| Effect | Where it enters | Modeling approach |
| --- | --- | --- |
| Thrust line offset pitch/yaw moment | `Aircraft6DOF` — additive moment | $T \cdot [z_T, y_T]$; parameters from geometry |
| Propwash tail effectiveness augmentation | `BodyAxisCoeffModel` — $\delta_T$-dependent correction to $C_{Z_q}$, $C_{m_q}$, $C_{m_{\delta_e}}$ | Actuator disk estimate; included as additive $\Delta C(\delta_T)$ |
| P-factor yawing moment | `BodyAxisCoeffModel` — additive $C_n(\alpha, \delta_T)$ term | Geometry-based; proportional to $\alpha$ and thrust loading |
| Gyroscopic pitch/yaw moment | `Aircraft6DOF` — additive moment | $I_{prop} \Omega q$ / $I_{prop} \Omega r$; $\Omega$ from propulsion model |
| Slipstream rolling moment | `BodyAxisCoeffModel` — additive $C_l(\delta_T)$ | Actuator disk + empirical factor |
| Multi-engine asymmetric thrust | `Aircraft6DOF` — additive moment | Sum over engines; from geometry and per-engine thrust |

---

## PropulsionCoeffEstimator — Architecture

`PropulsionCoeffEstimator` is a proposed stateless utility class parallel to
`AeroCoeffEstimator`. It estimates propulsion model parameters from specification data
and geometry, and computes the propulsion-aero coupling coefficients.

### Target Interface (Proposed — Not Yet Implemented)

```cpp
// Proposed — not yet implemented
struct PropulsionGeometry {
    float thrust_line_x_m;       // body x of thrust application point
    float thrust_line_y_m;       // body y (0 for symmetric single engine)
    float thrust_line_z_m;       // body z (negative = above CG in body z-down frame)
    float prop_diameter_m;       // for slipstream / propwash calculations
    float prop_disk_overlap_ht;  // fraction of HT span immersed in slipstream (0–1)
};

struct PropulsionCouplingCoefficients {
    // Thrust-line offset: dimensional offsets (m) — applied as T * offset in Aircraft6DOF
    float thrust_offset_z_m;   // z_T: pitch moment arm
    float thrust_offset_y_m;   // y_T: yaw moment arm (multi-engine or offset nacelle)

    // Propwash augmentation slopes (per unit of T / (q_inf * S)):
    float d_cz_q_d_thrust_loading;  // delta_CZ_q per unit thrust loading
    float d_cm_q_d_thrust_loading;  // delta_Cm_q per unit thrust loading
    float d_cm_de_d_thrust_loading; // delta_Cm_de per unit thrust loading

    // P-factor
    float cn_p_factor_nd;         // C_n_P: yaw per unit alpha × thrust loading

    // Gyroscopic: propeller inertia (kg·m²) — combined prop + motor rotor
    float propeller_inertia_kg_m2;

    // Slipstream rolling moment slope (per unit thrust loading)
    float d_cl_d_thrust_loading;
};

class PropulsionCoeffEstimator {
public:
    // Estimate coupling coefficients from geometry and propulsion configuration
    [[nodiscard]] static PropulsionCouplingCoefficients
        estimateCoupling(const AircraftGeometry&    aero_geom,
                         const PropulsionGeometry&  prop_geom,
                         const PropellerAero&       propeller);

    // Estimate PropellerAero parameters from UIUC database fit
    [[nodiscard]] static PropellerAero
        estimateFromUiuc(const std::string& prop_name,
                         const std::string& uiuc_data_path);

    // Fit PropellerAero parabolic model to measured CT/CQ data
    [[nodiscard]] static PropellerAero
        fitFromData(const std::vector<float>& J,
                    const std::vector<float>& CT,
                    const std::vector<float>& CQ);
};
```

### Integration with `Aircraft6DOF`

`Aircraft6DOF` computes total forces and moments as:

```
F_total = F_aero(BodyAxisCoeffModel) + F_thrust(V_Propulsion) + F_gravity
M_total = M_aero(BodyAxisCoeffModel) + M_thrust_offset + M_gyroscopic
        + M_propwash_correction(delta_T)
```

The propulsion-aero coupling terms that depend on instantaneous thrust (thrust-line
offset, gyroscopic moment) are computed directly by `Aircraft6DOF` from the current
thrust output and the `PropulsionCouplingCoefficients`. The throttle-dependent
aerodynamic corrections (propwash) are folded into the aerodynamic force computation
as a correction factor applied before calling `BodyAxisCoeffModel::compute()`.

---

## Relationship to the DOE / Surrogate Pipeline

The `ParametricAircraftConfig` described in
[`aero_coeff_estimator.md`](aero_coeff_estimator.md) includes propulsion geometry
parameters ($z_T$, $y_T$, prop diameter, HT overlap fraction). These flow into
`PropulsionCoeffEstimator::estimateCoupling()` at each DOE sample point, generating
coupling coefficients alongside the aerodynamic stability derivatives.

The surrogate model (Method 8 in `aero_coeff_estimator.md`) can therefore predict both
the aerodynamic *and* propulsive coupling coefficients as a function of aircraft geometry,
making the full `Aircraft6DOF` parameter set estimable from geometry alone.

```text
ParametricAircraftConfig (includes PropulsionGeometry)
    │
    ├──►  AeroCoeffEstimator.estimateBodyAxis()   → BodyAxisCoefficients
    └──►  PropulsionCoeffEstimator.estimateCoupling() → PropulsionCouplingCoefficients
                    │
                    ▼
            Aircraft6DOF config (complete)
```

---

## Data Sources

| Source | What it covers | Notes |
| --- | --- | --- |
| UIUC Propeller Database | $C_T(J)$, $C_Q(J)$ for common UAV propellers | Primary source for `PropellerAero` parameters |
| eCalc motordb | Motor KV, $R_m$, max current | Community-maintained; cross-check against spec sheets |
| QPROP / XROTOR | BET-based $C_T$/$C_Q$ for custom propellers | Useful when prop is not in UIUC database |
| Manufacturer datasheets | All primary parameters for commercial motors, ESCs, jets | Required starting point for any propulsion model |
| AFRL Propulsion Data | Jet spool time constants, inlet area for military engines | Not publicly available for most engines |
| Actuator disk theory | EDF static thrust from shaft power; propwash velocity | First-principles estimate; no additional data required |

---

## Open Questions

| ID | Summary | Blocking | Recommendation |
| --- | --- | --- | --- |
| OQ-P1 | **Propwash model complexity.** The propwash correction $\Delta C_{m_q}(\delta_T)$ assumes the tail is uniformly immersed in the slipstream. For most UAV configurations the tail is only partially in the wake. Should the overlap fraction $k_{slip}$ be a single scalar, or should the correction model account for the axial decay of the slipstream with distance? | `PropulsionCoeffEstimator.estimateCoupling()` | Use single scalar `prop_disk_overlap_ht` for initial implementation; document as a calibration parameter; revisit if propwash moment error is significant in design study validation. |
| OQ-P2 | **P-factor model accuracy.** The linear $C_{n_P} \cdot \alpha$ model is a first-order approximation. For large propellers at high AoA (takeoff rotation, stall recovery), the effect may be significantly nonlinear. Should `BodyAxisCoeffModel` support a nonlinear P-factor term? | `BodyAxisCoeffModel` format | Evaluate P-factor magnitude for Cases A–C using the linear model; if residual is $> 5\%$ of $C_{n_\beta}\beta$ at representative AoA, extend the model. |
| OQ-P3 | **Where do coupling coefficients live in the config?** `PropulsionCouplingCoefficients` is a new data type not currently in `BodyAxisCoeffModel`'s JSON config. Should it be a separate `propulsion_coupling` sub-object in the `Aircraft6DOF` config, or folded into the aerodynamic coefficient JSON? | `Aircraft6DOF` config schema | Keep separate: `propulsion_coupling` is physically distinct from aerodynamic derivatives and has a different estimation path. Separate object enables independent update without re-running the aero estimator. |
| OQ-P4 | **Gyroscopic term in `Aircraft6DOF` equations of motion.** $I_{prop}\Omega$ must be computed each step from the current propulsion model state. Does `V_Propulsion` need to expose `omega_rps()` generically, or is this only available on `PropulsionProp`? | `Aircraft6DOF` implementation | Add `omega_rps()` to `V_Propulsion` interface; return 0 for jet and EDF (no gyroscopic contribution). |
| OQ-P5 | **UIUC database integration path.** Should `UiucPropDbReader` be a Python module (offline, used during config generation) or a C++ class (used at runtime)? | `PropulsionCoeffEstimator` | Python module only; UIUC data processing is a one-time offline step. The fitted `PropellerAero` parameters are stored in JSON; the C++ simulation loads parameters, not raw UIUC data. |
| OQ-P6 | **Multi-engine configuration representation.** `Aircraft6DOF` currently has one `V_Propulsion`. For multi-engine asymmetric thrust modeling, it needs a vector of `(V_Propulsion, PropulsionGeometry)` pairs. This is an interface change. Is multi-engine required for the current design study, or deferred? | `Aircraft6DOF` design | Deferred; add as OQ to the `Aircraft6DOF` design document when it is created. |
