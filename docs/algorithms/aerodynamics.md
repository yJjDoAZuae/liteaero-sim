# Aerodynamic Coefficient Estimation from Aircraft Geometry

This document defines the derivation of every aerodynamic coefficient required by the
trim aero model (`AeroPerformance`, `LiftCurveModel`, and `LoadFactorAllocator`) from
major aircraft dimensional parameters.  All formulas are subsonic, incompressible
($M < 0.3$) unless noted.  References: USAF Stability and Control DATCOM (1978),
Raymer *Aircraft Design: A Conceptual Approach* (6th ed.), Etkin & Reid *Dynamics of
Flight* (3rd ed.).

---

## Trim Aero Model — Required Coefficients

| Symbol | C++ field | Class | Description |
| -------- | ----------- | ------- | ------------- |
| $S$ | `S_ref_m2` | `AeroPerformance`, `LoadFactorAllocator` | Reference wing area (m²) |
| $A\!\!R$ | `ar` | `AeroPerformance` | Wing aspect ratio |
| $e$ | `e` | `AeroPerformance` | Oswald efficiency factor |
| $C_{D_0}$ | `cd0` | `AeroPerformance` | Zero-lift drag coefficient |
| $C_{Y_\beta}$ | `cl_y_beta` | `AeroPerformance`, `LoadFactorAllocator` | Lateral force slope (rad⁻¹) |
| $C_{L_\alpha}$ | `cl_alpha` | `LiftCurveParams` | 3D lift-curve slope (rad⁻¹) |
| $C_{L_\text{max}}$ | `cl_max` | `LiftCurveParams` | Peak lift coefficient |
| $C_{L_\text{min}}$ | `cl_min` | `LiftCurveParams` | Minimum lift coefficient |
| $\Delta\alpha_\text{stall}$ | `delta_alpha_stall` | `LiftCurveParams` | Positive stall region half-width (rad) |
| $\Delta\alpha_{\text{stall}_{neg}}$ | `delta_alpha_stall_neg` | `LiftCurveParams` | Negative stall region half-width (rad) |
| $C_{L_\text{sep}}$ | `cl_sep` | `LiftCurveParams` | Positive post-stall plateau |
| $C_{L_{\text{sep}_{neg}}}$ | `cl_sep_neg` | `LiftCurveParams` | Negative post-stall plateau |
| $C_{L_q}$ | `cl_q_nd` | `AeroPerformance` *(proposed)* | Pitch-rate damping derivative (rad⁻¹) |
| $\bar{c}$ | `mac_m` | `AeroPerformance` *(proposed)* | Mean aerodynamic chord (m) |
| $C_{Y_r}$ | `cy_r_nd` | `AeroPerformance` *(proposed)* | Yaw-rate lateral force derivative (rad⁻¹) |
| $l_{VT}$ | `fin_arm_m` | `AeroPerformance` *(proposed)* | Vertical tail moment arm (m) |

*(Proposed)* fields are required by the rotational turbulence coupling defined in
[`environment.md`](environment.md) but are not yet implemented in `AeroPerformance`.

---

## Input Parameters

### Wing

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $b$ | Tip-to-tip span | m |
| $S$ | Reference area | m² |
| $\Lambda_0$ | Leading-edge sweep | rad |
| $\lambda$ | Taper ratio $c_\text{tip}/c_\text{root}$ | — |
| $x_{LE}$ | Body $x$ of root leading edge | m |

### Horizontal Tail

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $b_{HT}$ | Span (tip-to-tip) | m |
| $S_{HT}$ | Reference area | m² |
| $\Lambda_{0_{HT}}$ | Leading-edge sweep | rad |
| $\lambda_{HT}$ | Taper ratio | — |
| $x_{LE_{HT}}$ | Body $x$ of root leading edge | m |

### Vertical Tail

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $b_{VT}$ | Height (root to tip) | m |
| $S_{VT}$ | Reference area | m² |
| $\Lambda_{0_{VT}}$ | Leading-edge sweep | rad |
| $\lambda_{VT}$ | Taper ratio | — |
| $x_{LE_{VT}}$ | Body $x$ of root leading edge | m |

### Fuselage

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $l_f$ | Total length | m |
| $d_f$ | Maximum diameter (or equivalent diameter) | m |

### Airfoil Section Properties

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $(t/c)$ | Thickness-to-chord ratio of the wing section | — |
| $C_{l_{\alpha_{2D}}}$ | 2D section lift slope (≈ $2\pi$ rad⁻¹ for thin airfoils) | rad⁻¹ |
| $C_{l_{\max_{2D}}}$ | 2D section maximum lift coefficient | — |

### Mass and CG

| Symbol | Description | Unit |
| -------- | ------------- | ------ |
| $m$ | Aircraft mass | kg |
| $x_\text{CG}$ | Body $x$ of center of gravity | m |

---

## Part 1 — Derived Geometry

### 1.1 Wing

**Aspect ratio:**
$$A\!\!R = \frac{b^2}{S}$$

**Root chord:**
$$c_\text{root} = \frac{2S}{b(1+\lambda)}$$

**Tip chord:**
$$c_\text{tip} = \lambda\,c_\text{root}$$

**Quarter-chord sweep** (standard sweep-conversion formula):
$$\tan\Lambda_{QC} = \tan\Lambda_0 - \frac{1}{A\!\!R}\cdot\frac{1-\lambda}{1+\lambda}$$

**Half-chord sweep:**
$$\tan\Lambda_{c/2} = \tan\Lambda_0 - \frac{2}{A\!\!R}\cdot\frac{1-\lambda}{1+\lambda}$$

**Mean aerodynamic chord (MAC):**
$$\bar{c} = \frac{2}{3}\,c_\text{root}\,\frac{1+\lambda+\lambda^2}{1+\lambda}$$

**MAC spanwise station:**
$$\bar{y} = \frac{b}{6}\cdot\frac{1+2\lambda}{1+\lambda}$$

**MAC leading-edge body $x$:**
$$x_{LE_{\bar{c}}} = x_{LE} + \bar{y}\tan\Lambda_0$$

**Wing aerodynamic center body $x$** (at quarter-chord of MAC):
$$x_{ac_w} = x_{LE_{\bar{c}}} + \frac{\bar{c}}{4}$$

---

### 1.2 Horizontal Tail

Apply identical formulas with subscript $HT$:

$$A\!\!R_{HT} = \frac{b_{HT}^2}{S_{HT}}, \quad
c_{\text{root}_{HT}} = \frac{2S_{HT}}{b_{HT}(1+\lambda_{HT})}, \quad
\bar{c}_{HT} = \frac{2}{3}\,c_{\text{root}_{HT}}\,\frac{1+\lambda_{HT}+\lambda_{HT}^2}{1+\lambda_{HT}}$$

$$\tan\Lambda_{QC_{HT}} = \tan\Lambda_{0_{HT}} - \frac{1}{A\!\!R_{HT}}\cdot\frac{1-\lambda_{HT}}{1+\lambda_{HT}}$$

$$\bar{y}_{HT} = \frac{b_{HT}}{6}\cdot\frac{1+2\lambda_{HT}}{1+\lambda_{HT}}$$

**Tail aerodynamic center body $x$:**
$$x_{ac_{HT}} = x_{LE_{HT}} + \bar{y}_{HT}\tan\Lambda_{0_{HT}} + \frac{\bar{c}_{HT}}{4}$$

**Tail moment arm:**
$$l_{HT} = x_{ac_{HT}} - x_\text{CG}$$

---

### 1.3 Vertical Tail

Apply identical formulas with subscript $VT$:

$$A\!\!R_{VT} = \frac{b_{VT}^2}{S_{VT}}, \quad
c_{\text{root}_{VT}} = \frac{2S_{VT}}{b_{VT}(1+\lambda_{VT})}, \quad
\bar{c}_{VT} = \frac{2}{3}\,c_{\text{root}_{VT}}\,\frac{1+\lambda_{VT}+\lambda_{VT}^2}{1+\lambda_{VT}}$$

$$\tan\Lambda_{QC_{VT}} = \tan\Lambda_{0_{VT}} - \frac{1}{A\!\!R_{VT}}\cdot\frac{1-\lambda_{VT}}{1+\lambda_{VT}}$$

$$\bar{y}_{VT} = \frac{b_{VT}}{6}\cdot\frac{1+2\lambda_{VT}}{1+\lambda_{VT}}$$

**Vertical tail aerodynamic center body $x$:**
$$x_{ac_{VT}} = x_{LE_{VT}} + \bar{y}_{VT}\tan\Lambda_{0_{VT}} + \frac{\bar{c}_{VT}}{4}$$

**Vertical tail moment arm:**
$$l_{VT} = x_{ac_{VT}} - x_\text{CG}$$

---

## Part 2 — Lift Curve Slope $C_{L_\alpha}$

### 2.1 3D Wing — Helmbold / DATCOM (DATCOM §4.1.3.2)

$$C_{L_\alpha} = \frac{2\pi A\!\!R}{2 + \sqrt{4 + \dfrac{A\!\!R^2\,(1 + \tan^2\Lambda_{c/2})}{\eta^2}}}$$

where $\eta = \sqrt{1 - M^2}$ (Prandtl-Glauert factor; set $\eta = 1$ for $M < 0.3$).

For a straight, unswept wing ($\Lambda_{c/2} = 0$, $\eta = 1$) this reduces to:

$$C_{L_\alpha} \approx \frac{2\pi A\!\!R}{A\!\!R + 2}$$

**Validity:** $0 \le M \le 0.6$; straight-tapered planform; linear-lift regime.

> **Note:** The formula gives the isolated wing slope. For the complete aircraft, the tail
> contribution adds approximately $\eta_{HT} (S_{HT}/S) C_{L_{\alpha_{HT}}}$ where $\eta_{HT} \approx 0.9$.
> For the trim aero model the wing-alone value is used; tail effects are absorbed into the
> Oswald efficiency and drag polar.

---

## Part 3 — Lift Curve Model Parameters

### 3.1 Peak Lift Coefficient $C_{L_\text{max}}$

$$C_{L_\text{max}} = C_{l_{\max_{2D}}}\cos\Lambda_{QC}$$

This is the standard leading-edge suction correction for swept wings (Raymer §12.2).

**Typical range:** $1.0$ to $1.6$ for clean configurations; $1.4$ to $2.5$ with flaps.

### 3.2 Minimum Lift Coefficient $C_{L_\text{min}}$

For a symmetric airfoil section: $C_{L_\text{min}} = -C_{L_\text{max}}$.

For cambered sections, $|C_{L_\text{min}}|$ is typically $0.1$ to $0.2$ less than
$C_{L_\text{max}}$ in magnitude. Use $C_{L_\text{min}} = -(0.85 \ldots 1.0)\,C_{L_\text{max}}$
as a default estimate.

### 3.3 Stall Region Half-Width $\Delta\alpha_\text{stall}$

The angle from the upper end of the linear regime to the peak:

$$\Delta\alpha_\text{stall} = \frac{C_{L_\text{max}} - C_{L_\alpha}\,\alpha^*}{C_{L_\alpha}}$$

In practice, the stall onset angle $\alpha^* = C_{L_\text{max}} / C_{L_\alpha}$ is the
linear-regime boundary, and $\Delta\alpha_\text{stall}$ describes how quickly the peak is
reached after that boundary.

A reliable empirical default for moderate-$A\!\!R$ wings:

$$\Delta\alpha_\text{stall} \approx 0.05 \text{ to } 0.10 \text{ rad}  \quad (3° \text{ to } 6°)$$

Thin, highly swept wings stall more abruptly; use a smaller value ($\approx 0.03$ rad).
Thick, washout-optimized wings stall more gently; use $\approx 0.10$ rad.

$\Delta\alpha_{\text{stall}_{neg}}$ is taken equal to $\Delta\alpha_\text{stall}$ for symmetric
sections, or slightly smaller in magnitude for cambered sections.

### 3.4 Post-Stall Plateau $C_{L_\text{sep}}$

The separated-flow plateau is empirically $50\%$ to $70\%$ of $C_{L_\text{max}}$:

$$C_{L_\text{sep}} \approx 0.6\,C_{L_\text{max}}$$

Similarly: $C_{L_{\text{sep}_{neg}}} \approx 0.6\,C_{L_\text{min}}$ (maintaining the sign).

---

## Part 4 — Induced Drag and Oswald Efficiency $e$

### 4.1 Induced Drag Factor

$$k = \frac{1}{\pi\,e\,A\!\!R}$$

The induced drag polar is $C_D = C_{D_0} + k\,C_L^2$.

### 4.2 Oswald Efficiency Estimate (Hoerner / Raymer §12.6)

For a straight-tapered, moderate-sweep wing in isolation:

$$e \approx \frac{1}{1 + 0.007\pi A\!\!R}$$

This is the Hoerner leading-edge suction correction and applies for $A\!\!R < 12$, $\Lambda_{QC} < 30°$.

For higher sweep, apply a correction (Raymer Eq. 12.49):
$$e = e_0 \cdot \cos(\Lambda_{QC} - 0.09)$$
where $e_0$ is the unswept value above and the angle is in radians.

**Typical range:** $e = 0.70$ to $0.90$ for conventional configurations.

---

## Part 5 — Zero-Lift Drag $C_{D_0}$

$C_{D_0}$ is the parasite drag at $C_L = 0$. It cannot be derived from planform geometry
alone — it depends on surface finish, interference fits, and detailed wetted areas.
The component buildup method (Raymer §12.4) provides the most reliable analytical estimate.

### 5.1 Component Buildup

$$C_{D_0} = \frac{1}{S}\sum_i C_{f,i}\,FF_i\,Q_i\,S_{\text{wet},i} + C_{D_\text{misc}}$$

where the sum is over wing, horizontal tail, vertical tail, and fuselage.

**Flat-plate skin friction coefficient** (turbulent, Prandtl–Schlichting):

$$C_f = \frac{0.455}{(\log_{10} Re_l)^{2.58}}$$

where $Re_l = \rho\,V\,l / \mu$ and $l$ is the component reference length (chord for surfaces, $l_f$ for fuselage).

**Form factor — lifting surfaces** (Raymer Eq. 12.30):

$$FF = \left[1 + \frac{0.6}{(x/c)_m}(t/c) + 100(t/c)^4\right]\left[1.34\,M^{0.18}\cos^{0.28}\Lambda_{c/2}\right]$$

where $(x/c)_m \approx 0.3$ (location of maximum thickness) is a reasonable default.

**Form factor — fuselage** (Raymer Eq. 12.31):

$$FF_f = 1 + \frac{60}{\lambda_f^3} + \frac{\lambda_f}{400}, \quad \lambda_f = \frac{l_f}{d_f}$$

**Interference factor:**

| Component | $Q_i$ |
| ----------- | -------- |
| Wing (mid-fuselage) | 1.0 |
| Tail surfaces | 1.04 to 1.05 |
| Fuselage | 1.0 |

**Wetted areas:**

| Component | $S_\text{wet}$ |
| ----------- | ---------------- |
| Wing (both sides, exposed) | $2.003\,S$ for $(t/c) \approx 0.12$ (Raymer Eq. 12.6) |
| Horizontal tail | $2.003\,S_{HT}$ |
| Vertical tail | $2.003\,S_{VT}$ |
| Fuselage (cylindrical approx.) | $\pi\,d_f\,l_f\left(1 - \frac{2}{\lambda_f}\right)^{2/3}\left(1 + \frac{1}{\lambda_f^2}\right)$ |

**Miscellaneous drag** $C_{D_\text{misc}}$ covers cooling inlets, antennas, landing-gear
bumps, and similar items.  A conservative default is $C_{D_\text{misc}} = 0.002$ to
$0.005$ for a clean UAV configuration.

---

## Part 6 — Lateral Force Slope $C_{Y_\beta}$

The dominant contribution is from the vertical tail (Etkin & Reid §3.9):

$$C_{Y_\beta} = -\eta_{VT}\,\frac{S_{VT}}{S}\,C_{L_{\alpha_{VT}}}\left(1 + \frac{d\sigma}{d\beta}\right)$$

where:

| Symbol | Description | Typical value |
| -------- | ------------- | --------------- |
| $\eta_{VT}$ | Tail efficiency (dynamic pressure ratio) | 0.85 to 0.95 |
| $C_{L_{\alpha_{VT}}}$ | Vertical tail 3D lift slope (rad⁻¹) — from §2.1 applied to vertical tail | — |
| $d\sigma/d\beta$ | Sidewash gradient at the vertical tail | ≈ 0 for conventional rear-fuselage location |

The fuselage contribution is destabilizing (positive) and typically small:

$$C_{Y_{\beta_\text{fus}}} \approx +\frac{k_2 - k_1}{2}\cdot\frac{S_{\text{fus}_{side}}}{S\,b}\cdot b$$

where $k_2 - k_1 \approx 0.5$ for a typical fuselage fineness ratio and
$S_{\text{fus}_{side}} = l_f d_f$.  This term is often neglected for aircraft with large
vertical tails.

**Combined estimate:**

$$C_{Y_\beta} \approx -\eta_{VT}\,\frac{S_{VT}}{S}\,C_{L_{\alpha_{VT}}}$$

$C_{Y_\beta}$ is negative for a statically stable configuration (sideslip to the right
generates a restoring leftward force).

---

## Part 7 — Pitch-Damping Derivative $C_{L_q}$ *(proposed)*

$C_{L_q}$ is the rate of change of lift coefficient with non-dimensional pitch rate
$\hat{q} = q\bar{c}/(2V_a)$.

### 7.1 Wing Contribution (DATCOM §5.2.1.2)

$$C_{L_{q_\text{wing}}} = C_{L_\alpha}\left[\frac{A\!\!R + 2\cos\Lambda_{QC}}{2(A\!\!R + 4\cos\Lambda_{QC})}\cdot\frac{A\!\!R}{\cos\Lambda_{QC}} + \frac{1+2\lambda}{3(1+\lambda)}\tan^2\Lambda_{QC}\right]$$

For an unswept wing ($\Lambda_{QC} = 0$):

$$C_{L_{q_\text{wing}}} = C_{L_\alpha}\cdot\frac{A\!\!R(A\!\!R+2)}{2(A\!\!R+4)}$$

### 7.2 Tail Contribution

$$C_{L_{q_\text{tail}}} = 2\,\eta_{HT}\,C_{L_{\alpha_{HT}}}\,\frac{l_{HT}\,S_{HT}}{S\,\bar{c}}$$

This term dominates for conventional tailed aircraft because $l_{HT} \gg \bar{c}$.

### 7.3 Combined

$$C_{L_q} = C_{L_{q_\text{wing}}} + C_{L_{q_\text{tail}}}$$

**Typical range:** $3$ to $12\,\text{rad}^{-1}$ for UAVs.

---

## Part 8 — Yaw-Rate Lateral Derivative $C_{Y_r}$ *(proposed)*

The yaw rate $r$ moves the vertical tail sideways at velocity $r\,l_{VT}$, changing its
effective sideslip angle by $\beta_{VT} = r\,l_{VT} / V_a$. The resulting side force is:

$$C_{Y_r} = 2\,\eta_{VT}\,C_{L_{\alpha_{VT}}}\,\frac{l_{VT}\,S_{VT}}{S\,b}$$

$C_{Y_r}$ is positive (right yaw rate generates a rightward side-force from the tail).

---

## Part 9 — Derivation Chain Summary

The following table maps each trim-aero-model coefficient to the input parameters it
requires.

| Coefficient | Inputs required |
| ------------- | ----------------- |
| $S$ | $S$ (direct) |
| $A\!\!R$ | $b$, $S$ |
| $\bar{c}$ | $b$, $S$, $\lambda$ |
| $e$ | $A\!\!R$, $\Lambda_{QC}$ |
| $C_{L_\alpha}$ | $A\!\!R$, $\Lambda_{c/2}$, $M$ |
| $C_{L_\text{max}}$ | $C_{l_{\max_{2D}}}$, $\Lambda_{QC}$ |
| $C_{L_\text{min}}$ | $C_{L_\text{max}}$ |
| $\Delta\alpha_\text{stall}$ | $C_{L_\text{max}}$, $C_{L_\alpha}$ (empirical) |
| $C_{L_\text{sep}}$ | $C_{L_\text{max}}$ (empirical factor) |
| $C_{D_0}$ | $(t/c)$, $\Lambda_{c/2}$, $S$, $l_f$, $d_f$, $S_{HT}$, $S_{VT}$, $\rho$, $V$ |
| $C_{Y_\beta}$ | $S_{VT}$, $S$, $A\!\!R_{VT}$, $\Lambda_{0_{VT}}$, $\lambda_{VT}$, $\eta_{VT}$ |
| $C_{L_q}$ | $A\!\!R$, $\lambda$, $\Lambda_{QC}$, $C_{L_\alpha}$, $l_{HT}$, $S_{HT}$, $\bar{c}$, $C_{L_{\alpha_{HT}}}$, $\eta_{HT}$ |
| $C_{Y_r}$ | $S_{VT}$, $S$, $b$, $l_{VT}$, $C_{L_{\alpha_{VT}}}$, $\eta_{VT}$ |
| $l_{VT}$, $l_{HT}$ | $x_\text{CG}$, $x_{LE_{HT/VT}}$, $b_{HT/VT}$, $\lambda_{HT/VT}$, $\Lambda_{0_{HT/VT}}$ |

### Parameters That Cannot Be Derived from Geometry Alone

| Parameter | Reason | Typical approach |
| ----------- | -------- | ------------------ |
| $C_{D_0}$ | Depends on surface finish, gaps, protuberances | Component buildup (§5.1) + empirical correction |
| $e$ | Depends on detailed wake and tip shape | Hoerner estimate (§4.2); refine from flight test or CFD |
| $C_{l_{\max_{2D}}}$ | Depends on airfoil profile selection | Look up from airfoil database (NACA, UIUC) |
| $\eta_{HT}$, $\eta_{VT}$ | Depends on fuselage boundary layer thickness | Default $0.9$; refine from test |
| $\Delta\alpha_\text{stall}$ | Depends on stall character (abrupt vs. gentle) | Default $0.07$ rad; adjust from observed stall behavior |
