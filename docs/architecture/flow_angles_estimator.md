# Flow Angles Estimator — Architecture and Interface Design

This document is the design authority for `FlowAnglesEstimator` within the
`liteaerosim::estimation` namespace. It specifies a 4-state Kalman filter that estimates
angle of attack (α), sideslip angle (β), a lift coefficient bias (ΔCL), and a sideforce
coefficient bias (ΔCY) by fusing aerodynamic model inversions with kinematic observations
from the GNSS–wind triangle. An integrity monitoring subsystem cross-validates the two
measurement branches and outputs protection levels for both flow angles.

`FlowAnglesEstimator` derives from `liteaerosim::DynamicElement`. The lifecycle contract,
NVI pattern, and base class requirements are defined in
[`docs/architecture/dynamic_element.md`](dynamic_element.md).

---

## Scope

`FlowAnglesEstimator` provides α and β estimates consumed by:

- `WindEstimator` — needs α to observe the vertical airspeed component
  (see [`docs/architecture/wind_estimator.md`](wind_estimator.md))
- Guidance and control layers — need α for stall margin, load factor, and commanded
  normal acceleration; need β for coordination and sideslip limit monitoring
- Simulation analysis — α and β verification against the aerodynamic model

### Measurement Sources

| Source | Observable | Quality driver |
| --- | --- | --- |
| **Lift model inversion** | α via $CL_{meas} = -f_z^B m/(q_{dyn}S)$ | CLα known ±10%; accurate at cruise; degrades below min q_dyn; biased by weight and thrust misalignment |
| **Sideforce model inversion** | β via $CY_{meas} = f_y^B m/(q_{dyn}S)$ | CYβ known ±20–30%; ~15–20× noisier than lift-α at same conditions; degrades below min q_dyn |
| **Wind triangle (kinematic)** | α and β from GNSS velocity, attitude, wind estimate | Requires converged `WindEstimate`; independent of aero model; degrades when wind is unknown or GNSS absent |
| **Angular rates (process)** | Kinematic propagation of α̇ and β̇ | High-bandwidth; drifts without measurement correction |

### Aerodynamic Bias States

Both aerodynamic inversions carry systematic offsets that are unrelated to the true
flow angles. A dedicated bias state for each absorbs these offsets as a slowly varying
Gauss-Markov process, preventing them from biasing the flow angle estimates.

**ΔCL sources** (additive to predicted lift coefficient):

| Source | Typical magnitude |
| --- | --- |
| Weight uncertainty (fuel burn, payload change) | 0.5–3% of CL |
| Thrust z-component (down-thrust, prop wash) | 1–5% of CL |
| Static-port position error residual | 0.3–1% of CL |
| Trim moment effects on normal force | 0.5–2% of CL |

**ΔCY sources** (additive to predicted sideforce coefficient):

| Source | Typical magnitude |
| --- | --- |
| Vertical tail efficiency uncertainty | 5–15% of CYβ·β |
| Fuselage sideforce (nonlinear, hard to model) | 3–8% of CYβ·β |
| Propeller slipstream asymmetry | 2–10% of CYβ·β |
| Rudder trim offset | 2–5% of CYβ·β |

### Reliability Comparison: Lift vs. Sideforce Inversion

For the same accelerometer noise $\sigma_f$ and conditions:

$$
\frac{\sigma_\beta^{side}}{\sigma_\alpha^{lift}} = \frac{CL_\alpha}{|CY_\beta|}
$$

At typical values $CL_\alpha \approx 5.7$, $|CY_\beta| \approx 0.3$: the β observation
from sideforce is **~19× noisier** than the α observation from lift. Despite this, the
sideforce measurement provides a valuable independent constraint on β when the kinematic
source is absent, and participates in integrity cross-checking at cruise speeds.

CYβ has significantly larger model uncertainty than CLα (~20–30% vs. ~10%), reflected in
a wider measurement noise `R_side`. The ΔCY state absorbs the steady-state offset from
this model error.

### Coordinate Convention

Body frame: x-forward, y-right, z-down.

$$
\alpha = \text{atan2}(-v_{air,z}^B,\; v_{air,x}^B), \quad
\beta  = \text{asin}\!\left(\frac{v_{air,y}^B}{\|\mathbf{v}_{air}^B\|}\right)
$$

Positive α: nose up. Positive β: nose left (airflow from the right).

---

## Aerodynamic Force Model

### Normal Force (Lift Inversion — α observable)

Body-frame z-component of aerodynamic force, z-down positive:

$$
F_z^{aero} \approx -(CL_0 + CL_\alpha\,\alpha + CL_q\,\hat{q} + \Delta CL)\;q_{dyn}\,S
$$

where $\hat{q} = q\,\bar{c}/(2\,TAS)$ is the non-dimensional pitch rate. Drag is
neglected in the z-component at small α; its contribution is absorbed by ΔCL.

### Sideforce Inversion (β observable)

Body-frame y-component of aerodynamic force:

$$
F_y^{aero} \approx (CY_\beta\,\beta + CY_p\,\hat{p} + CY_r\,\hat{r} + \Delta CY)\;q_{dyn}\,S
$$

where $\hat{p} = p\,b/(2\,TAS)$, $\hat{r} = r\,b/(2\,TAS)$, and $b$ is the wingspan.
CYp and CYr corrections remove the roll-rate and yaw-rate contributions before
inverting for β. These corrections use the same `wingspan_m` geometry parameter as the
primary sideforce coefficient.

Dynamic pressure from EAS:

$$
q_{dyn} = \tfrac{1}{2}\,\rho_0\,EAS^2, \quad \rho_0 = 1.225\;\text{kg/m}^3
$$

---

## State Vector

$$
\mathbf{x} = \begin{bmatrix} \alpha & \beta & \Delta CL & \Delta CY \end{bmatrix}^T
$$

| Index | State | Units | Description |
| --- | --- | --- | --- |
| 0 | $\alpha$ | rad | Angle of attack |
| 1 | $\beta$ | rad | Sideslip angle |
| 2 | $\Delta CL$ | — | Lift coefficient bias (additive to lift model prediction) |
| 3 | $\Delta CY$ | — | Sideforce coefficient bias (additive to sideforce model prediction) |

---

## Process Model

For near-wings-level flight with small sideslip ($|\beta| < 10°$):

$$
\dot{\alpha} = q
  + \frac{g}{TAS}\!\left(\cos\theta\cos\phi\cos\alpha - \sin\theta\sin\alpha\right)
$$

$$
\dot{\beta} = p\sin\alpha - r\cos\alpha + \frac{g}{TAS}\cos\theta\sin\phi
$$

$$
\dot{\Delta CL} = -\frac{\Delta CL}{\tau_{CL}} + w_{CL}, \qquad
\dot{\Delta CY} = -\frac{\Delta CY}{\tau_{CY}} + w_{CY}
$$

where $p, q, r$ are body angular rates from the INS, $\theta, \phi$ from the attitude
quaternion. Discrete-time propagation (Euler):

$$
\alpha_{k+1} = \alpha_k + \dot{\alpha}_k\,dt, \quad
\beta_{k+1}  = \beta_k  + \dot{\beta}_k\,dt, \quad
\Delta CL_{k+1} = e^{-dt/\tau_{CL}}\Delta CL_k, \quad
\Delta CY_{k+1} = e^{-dt/\tau_{CY}}\Delta CY_k
$$

### Linearized F Matrix (4×4)

$$
F = \begin{bmatrix}
F_{\alpha\alpha} & 0 & 0 & 0 \\
F_{\beta\alpha}  & 0 & 0 & 0 \\
0 & 0 & -1/\tau_{CL} & 0 \\
0 & 0 & 0 & -1/\tau_{CY}
\end{bmatrix}
$$

$$
F_{\alpha\alpha} = \frac{g}{TAS}(-\cos\theta\cos\phi\sin\hat{\alpha} - \sin\theta\cos\hat{\alpha}), \quad
F_{\beta\alpha} = p\cos\hat{\alpha} + r\sin\hat{\alpha}
$$

Discrete-time: $\Phi \approx I + F\,dt$.

### Process Noise (continuous-time, diagonal)

| State | $q_{c,i}$ |
| --- | --- |
| $\alpha$ | $\sigma_\alpha^2$ |
| $\beta$ | $\sigma_\beta^2$ |
| $\Delta CL$ | $2\,\sigma_{CL}^2\,/\,\tau_{CL}$ |
| $\Delta CY$ | $2\,\sigma_{CY}^2\,/\,\tau_{CY}$ |

---

## Measurement Models

All updates use the Joseph-form covariance correction for numerical stability. A square
root (Cholesky, UDU) formulation is not used; the 4×4 system is well-conditioned at
`float` precision with the Joseph form.

### Measurement 1 — Lift Inversion (α)

Applied when $q_{dyn} \geq q_{dyn,min}$.

Measured CL after pitch-rate correction:

$$
CL_{meas} = -\frac{f_z^B \cdot m}{q_{dyn} \cdot S} - CL_q\,\frac{q\,\bar{c}}{2\,TAS}
$$

Observation and expected value:

$$
z_{lift} = CL_{meas} - CL_0 = CL_\alpha\,\alpha + \Delta CL + \nu_{lift}
$$

$$
H_{lift} = \begin{bmatrix} CL_\alpha & 0 & 1 & 0 \end{bmatrix} \quad (1 \times 4)
$$

Measurement noise (inflates as $1/q_{dyn}^2$; naturally down-weights at low speed):

$$
R_{lift} = \left(\frac{\sigma_{fz} \cdot m}{q_{dyn} \cdot S}\right)^2
$$

**Update skipped** when $q_{dyn} < q_{dyn,min}$ (default 30 Pa, ~7 m/s EAS).

### Measurement 2 — Sideforce Inversion (β)

Applied when $q_{dyn} \geq q_{dyn,min}$.

Measured CY after roll-rate and yaw-rate corrections:

$$
CY_{meas} = \frac{f_y^B \cdot m}{q_{dyn} \cdot S} - CY_p\,\frac{p\,b}{2\,TAS} - CY_r\,\frac{r\,b}{2\,TAS}
$$

Observation and expected value:

$$
z_{side} = CY_{meas} = CY_\beta\,\beta + \Delta CY + \nu_{side}
$$

$$
H_{side} = \begin{bmatrix} 0 & CY_\beta & 0 & 1 \end{bmatrix} \quad (1 \times 4)
$$

Measurement noise (same q_dyn structure as lift; additionally inflated by CYβ model
uncertainty $\sigma_{CY\beta}$ relative to CLα model uncertainty):

$$
R_{side} = \left(\frac{\sigma_{fz} \cdot m}{q_{dyn} \cdot S}\right)^2
           + \left(\frac{CY_{meas} \cdot \sigma_{CY\beta/CY\beta}}{CY_\beta}\right)^2
$$

The second term accounts for the proportional uncertainty in the CYβ coefficient value
itself. `cy_beta_relative_uncertainty_nd` (default 0.25, i.e., 25% coefficient
uncertainty) multiplies the current CY measurement magnitude. This term dominates over
accelerometer noise at cruise when β is nonzero.

**Update skipped** when $q_{dyn} < q_{dyn,min}$.

### Measurement 3 — Kinematic (Wind Triangle)

Applied when `wind_est != nullptr` and $TAS \geq TAS_{min}$.

$$
\mathbf{v}_{air}^B = R_{B \leftarrow N}(\hat{q})\,(\mathbf{v}_{gnss}^N - \hat{\mathbf{w}}^N)
$$

$$
\alpha_{kin} = \text{atan2}(-v_{air,z}^B,\; v_{air,x}^B), \quad
\beta_{kin}  = \text{asin}\!\left(\tfrac{v_{air,y}^B}{\|\mathbf{v}_{air}^B\|}\right)
$$

Observation and Jacobian ($2 \times 4$):

$$
\mathbf{z}_{kin} = \begin{bmatrix} \alpha_{kin} \\ \beta_{kin} \end{bmatrix}, \quad
H_{kin} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}
$$

$$
R_{kin} = \sigma_{kin}^2 \cdot I_2
$$

$\sigma_{kin}$ is inflated by `kinematic_unconverged_scale` (default 5×) when
`wind_est->is_converged == false`. **Update skipped** when $TAS < TAS_{min}$.

---

## Integrity Monitoring

When multiple measurement sources are active, their raw pre-fusion observations are
compared directly. Disagreement between sources indicates a failure that the filter
state estimate alone cannot reveal.

### Innovation Consistency Test

At each measurement update, the normalized innovation squared (NIS) for source $j$ is:

$$
\epsilon_j = \mathbf{r}_j^T\,S_j^{-1}\,\mathbf{r}_j, \quad
S_j = H_j\,P\,H_j^T + R_j
$$

where $\mathbf{r}_j = \mathbf{z}_j - H_j\,\hat{\mathbf{x}}$ is the innovation and
$S_j$ is the innovation covariance. For a healthy source, $\epsilon_j$ is
chi-squared distributed with $\dim(\mathbf{z}_j)$ degrees of freedom; its expected
value equals the number of observations.

A source is flagged suspect when:

$$
\epsilon_j > \chi^2_{dof,\,p_{alert}}
$$

where $p_{alert}$ = `integrity_chi2_threshold_nd` (default: 95th percentile of the
chi-squared distribution — e.g., 3.84 for 1 DOF, 5.99 for 2 DOF).

A running mean of $\epsilon_j$ over the last $N_{window}$ steps is also monitored; a
sustained mean > 1.5 (indicating persistent model misfit) triggers the same flag even
if no single step exceeds the threshold.

### Cross-Source Consistency Check

When both an aerodynamic and a kinematic measurement are available, their pre-fusion
raw angle estimates are compared:

**α cross-check** (lift and kinematic):

$$
r_{\alpha,cross} = \alpha_{kin} - \alpha_{lift,raw}, \quad
\alpha_{lift,raw} = \frac{CL_{meas} - CL_0 - \Delta\hat{CL}_{prev}}{CL_\alpha}
$$

$$
\sigma_{\alpha,cross}^2 = R_{lift}/CL_\alpha^2 + \sigma_{kin}^2 + P_{prev}[2,2]/CL_\alpha^2
$$

**β cross-check** (sideforce and kinematic):

$$
r_{\beta,cross} = \beta_{kin} - \beta_{side,raw}, \quad
\beta_{side,raw} = \frac{CY_{meas} - \Delta\hat{CY}_{prev}}{CY_\beta}
$$

The cross-check variance includes the previous-step filter uncertainty in the bias
states (P[2,2] for ΔCL, P[3,3] for ΔCY) to avoid false alerts when the bias states
have not yet converged.

Inconsistency is declared when:

$$
| r_{\alpha,cross} | > k_{cross}\,\sigma_{\alpha,cross}, \quad
| r_{\beta,cross} | > k_{cross}\,\sigma_{\beta,cross}
$$

with $k_{cross}$ = `integrity_cross_check_sigma_nd` (default 4.0).

### Failure Mode Coverage

| Failure mode | Detected by |
| --- | --- |
| Icing (lift curve flattened, stall onset shifted) | lift NIS ↑, α cross-check fails |
| Pitot tube failure (q_dyn zero or wrong) | lift NIS ↑, sideforce NIS ↑ simultaneously |
| Wind estimate divergence | kinematic NIS ↑ (both α and β innovations large) |
| GNSS failure | kinematic update absent; aero-only mode, no cross-check possible |
| Structural damage (asymmetric drag) | sideforce NIS ↑ persistently; ΔCY saturates against its bounds |
| Weight change outside ΔCL range | lift NIS ↑ persistently; ΔCL walks to edge of uncertainty |

### Integrity Status and Protection Level

The protection level is a worst-case bound on the flow angle estimate error that holds
with probability $p_{protect}$ given the current filter state and the most recent
source consistency:

$$
\alpha_{PL} = k_{protect}\,\sqrt{P[0,0]}, \quad
\beta_{PL}  = k_{protect}\,\sqrt{P[1,1]}
$$

where $k_{protect}$ = `integrity_protection_sigma_nd` (default 3.0 for ~99.7%
coverage of Gaussian errors). This bound is **valid only when `integrity_status ==
Nominal`**. When a source is suspect, the protection level is widened to reflect
operation from a reduced observation set.

```cpp
enum class FlowAnglesIntegrityStatus : int32_t {
    Nominal  = 0,  // all active sources consistent; protection level valid
    Degraded = 1,  // one source flagged; estimate still viable from remaining
    Failed   = 2,  // insufficient consistent sources; estimate unreliable
};
```

| Status | Condition |
| --- | --- |
| `Nominal` | No source suspect; or only one source active (no cross-check possible but NIS healthy) |
| `Degraded` | Exactly one source flagged suspect; other source(s) remain consistent |
| `Failed` | All active sources flagged suspect simultaneously, or fewer than one source available after exclusion |

---

## Coupling with WindEstimator

At each simulation step $k$, update order in the caller:

```text
1. FlowAnglesEstimator::step(wind_est_{k-1}, ...)   → flow_angles_{k}
2. WindEstimator::step(flow_angles_{k}.alpha_rad, ...) → wind_est_{k}
```

The one-step delay is stable because wind changes on a ~10–100 s timescale; at 100 Hz
the resulting phase lag is negligible. During warm-up (no wind estimate yet), pass
`wind_est = nullptr` and the kinematic update is skipped.

---

## Data Structures

### `FlowAnglesEstimate`

```cpp
// include/estimation/FlowAnglesEstimator.hpp
namespace liteaerosim::estimation {

struct FlowAnglesEstimate {
    // --- Flow angles ---
    float alpha_rad;   // estimated angle of attack (rad)
    float beta_rad;    // estimated sideslip angle (rad)

    // --- Bias estimates ---
    float cl_bias_nd;  // estimated ΔCL (non-dimensional)
    float cy_bias_nd;  // estimated ΔCY (non-dimensional)

    // --- Estimation uncertainty ---
    std::array<float, 4> covariance_diagonal;  // 1σ² per state [α, β, ΔCL, ΔCY]

    // --- Integrity ---
    float alpha_protection_rad;          // k_protect · sqrt(P[0,0]); valid when Nominal
    float beta_protection_rad;           // k_protect · sqrt(P[1,1]); valid when Nominal
    FlowAnglesIntegrityStatus integrity_status;
    bool  aero_source_suspect;           // lift or sideforce NIS / cross-check failed
    bool  kinematic_source_suspect;      // kinematic NIS / cross-check failed

    bool  is_converged;  // true when α σ < convergence_alpha_sigma_rad
                         // and β σ < convergence_beta_sigma_rad
};

} // namespace liteaerosim::sensor
```

### `FlowAnglesConfig`

```cpp
// include/estimation/FlowAnglesEstimator.hpp
namespace liteaerosim::estimation {

struct FlowAnglesConfig {
    // --- Lift curve ---
    float cl_zero_nd         = 0.3f;   // CL0
    float cl_alpha_per_rad   = 5.7f;   // CLα (per rad)
    float cl_q_per_rad       = 0.f;    // CLq (per rad; typically 3–8; 0 = disabled)
    float mean_aero_chord_m  = 0.3f;   // c̄ (m)

    // --- Sideforce model ---
    float cy_beta_per_rad    = -0.3f;  // CYβ (per rad; negative)
    float cy_p_per_rad       = 0.f;    // CYp (per rad)
    float cy_r_per_rad       = 0.f;    // CYr (per rad)
    float wingspan_m         = 1.5f;   // b (m); used for p̂, r̂ corrections

    // --- CYβ model reliability ---
    // Fractional uncertainty in the CYβ coefficient value (not in β).
    // 0.25 = 25% coefficient uncertainty; inflates R_side when β is nonzero.
    float cy_beta_relative_uncertainty_nd = 0.25f;

    // --- Common geometry ---
    float wing_area_m2 = 0.5f;   // S (m²)
    float mass_kg      = 3.0f;   // m (kg)

    // --- Lift bias model ---
    float cl_bias_instability_nd     = 0.03f;  // Gauss-Markov steady-state 1σ
    float cl_bias_correlation_time_s = 120.f;  // τ_CL (s)
    float initial_cl_bias_uncertainty_nd = 0.1f;

    // --- Sideforce bias model ---
    float cy_bias_instability_nd     = 0.04f;  // Gauss-Markov steady-state 1σ (larger than CL)
    float cy_bias_correlation_time_s = 120.f;  // τ_CY (s)
    float initial_cy_bias_uncertainty_nd = 0.15f;

    // --- Process noise ---
    float alpha_process_noise_rad_per_sqrthz = 0.02f;
    float beta_process_noise_rad_per_sqrthz  = 0.02f;

    // --- Aerodynamic measurement ---
    float accel_noise_mps2 = 0.05f;   // specific force 1σ (m/s²)
    float min_qdyn_pa      = 30.f;    // below this, aero updates suspended

    // --- Kinematic measurement ---
    float kinematic_noise_rad         = 0.05f;  // σ_kin when wind converged (rad)
    float kinematic_unconverged_scale = 5.f;    // multiplier before convergence
    float min_tas_for_kinematic_mps   = 5.f;

    // --- Initial state ---
    float initial_alpha_rad             = 0.08f;  // ~4.6°
    float initial_beta_rad              = 0.f;
    float initial_alpha_uncertainty_rad = 0.3f;
    float initial_beta_uncertainty_rad  = 0.3f;

    // --- Convergence ---
    float convergence_alpha_sigma_rad = 0.02f;
    float convergence_beta_sigma_rad  = 0.02f;

    // --- Integrity ---
    float integrity_chi2_threshold_nd      = 3.84f;  // 95th pct, 1-DOF chi-squared
    float integrity_cross_check_sigma_nd   = 4.0f;   // k for cross-source comparison
    int   integrity_nis_window_steps       = 50;      // sliding window for mean NIS
    float integrity_protection_sigma_nd    = 3.0f;   // k_protect for protection level

    float dt_s           = 0.01f;
    int   schema_version = 1;
};

} // namespace liteaerosim::sensor
```

---

## Step Interface

```cpp
FlowAnglesEstimate step(const Eigen::Vector3f&    specific_force_body_mps2,
                        const Eigen::Vector3f&    angular_rate_body_rads,
                        const Eigen::Quaternionf& q_body_from_ned,
                        const AirDataMeasurement& air_data,
                        const WindEstimate*       wind_est);    // nullptr during warm-up
```

---

## Class Interface

```cpp
// include/estimation/FlowAnglesEstimator.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <deque>
#include <nlohmann/json.hpp>
#include "DynamicElement.hpp"
#include "sensor/SensorAirData.hpp"
#include "estimation/WindEstimator.hpp"

namespace liteaerosim::estimation {

class FlowAnglesEstimator : public liteaerosim::DynamicElement {
public:
    FlowAnglesEstimate step(const Eigen::Vector3f&    specific_force_body_mps2,
                            const Eigen::Vector3f&    angular_rate_body_rads,
                            const Eigen::Quaternionf& q_body_from_ned,
                            const AirDataMeasurement& air_data,
                            const WindEstimate*       wind_est);

    void serializeProto(liteaerosim::FlowAnglesStateProto& proto) const;
    void deserializeProto(const liteaerosim::FlowAnglesStateProto& proto);

protected:
    void onInitialize(const nlohmann::json& config) override;
    void onReset() override;
    nlohmann::json onSerializeJson() const override;
    void onDeserializeJson(const nlohmann::json& state) override;

private:
    FlowAnglesConfig config_;

    Eigen::Vector4f x_;   // [α, β, ΔCL, ΔCY]
    Eigen::Matrix4f P_;

    // Integrity monitoring state
    std::deque<float> nis_aero_window_;   // sliding NIS history for aero source
    std::deque<float> nis_kin_window_;    // sliding NIS history for kinematic source
    bool              aero_source_suspect_   = false;
    bool              kinematic_source_suspect_ = false;
};

} // namespace liteaerosim::sensor
```

---

## Serialization Contract

### JSON State Fields

| JSON key | Description |
| --- | --- |
| `"schema_version"` | Integer; must equal 1. Throws on mismatch. |
| `"alpha_rad"` | α state estimate (rad). |
| `"beta_rad"` | β state estimate (rad). |
| `"cl_bias_nd"` | ΔCL state estimate. |
| `"cy_bias_nd"` | ΔCY state estimate. |
| `"P_upper"` | 10-element upper triangle of the 4×4 symmetric P, row-major. |
| `"nis_aero_window"` | Array of NIS values in the sliding window (≤ `integrity_nis_window_steps` elements). |
| `"nis_kin_window"` | Array of NIS values in the kinematic sliding window. |

Schema version: **1**.

---

## Proto Messages

```proto
// proto/liteaerosim.proto

message FlowAnglesStateProto {
    int32          schema_version = 1;
    float          alpha_rad      = 2;
    float          beta_rad       = 3;
    float          cl_bias_nd     = 4;
    float          cy_bias_nd     = 5;
    repeated float p_upper        = 6;   // 10 elements; upper triangle of P, row-major
    repeated float nis_aero       = 7;   // sliding NIS window
    repeated float nis_kin        = 8;   // sliding NIS window
}
```

---

## Computational Cost

### Memory Footprint

| Component | Size |
| --- | --- |
| State vector (4 floats) | 16 bytes |
| Covariance matrix (4×4 floats) | 64 bytes |
| NIS sliding windows (2 × 50 floats, default) | ~400 bytes |
| `FlowAnglesConfig` | ~116 bytes |
| **Total active state** | **~596 bytes** |

### Operations per `step()` Call

**Propagation (every step):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| θ, φ from quaternion (partial) | ~12 | |
| α̇, β̇ from body rates + gravity | ~18 | |
| Euler state propagation (4 states) | ~8 | 2 Euler + 2 Gauss-Markov |
| Φ assembly (sparse F — 2 off-diagonal terms) | ~4 | |
| Covariance ΦPΦᵀ + Q (4×4 sparse) | ~60 | |
| **Propagation total** | **~102** | |

**Lift update (when q_dyn ≥ threshold):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| q_dyn, CL_meas, CLq correction | ~12 | |
| Innovation + NIS | ~8 | includes χ² test |
| KF update (4-state scalar obs, Joseph form) | ~45 | |
| **Lift update total** | **~65** | |

**Sideforce update (when q_dyn ≥ threshold):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| CY_meas, CYp/CYr correction, R_side | ~15 | includes coefficient uncertainty term |
| Innovation + NIS | ~8 | |
| KF update (4-state scalar obs, Joseph form) | ~45 | |
| **Sideforce update total** | **~68** | |

**Kinematic update (when wind available and TAS ≥ threshold):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| v_air_body, α_kin, β_kin | ~28 | quaternion rotation + atan2/asin |
| Innovation + NIS (2-obs) | ~12 | |
| KF update (4-state rank-2 obs, Joseph form) | ~80 | |
| **Kinematic update total** | **~120** | |

**Cross-source checks (when pairs available):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| α cross-check (lift vs. kinematic) | ~12 | |
| β cross-check (sideforce vs. kinematic) | ~12 | |
| **Cross-check total** | **~24** | |

**Total per step with all three sources active:** ~379 FLOPs, 0 RNG draws.
**Total per step with aero only (no wind):** ~235 FLOPs.
**Total per step, propagation only:** ~102 FLOPs.

---

## Test Requirements

All tests reside in `test/FlowAnglesEstimator_test.cpp`, test class
`FlowAnglesEstimatorTest`.

| ID | Test Name | Description |
| --- | --- | --- |
| T1 | `SteadyLevelFlight_AlphaConvergesToTrim` | True α = 0.07 rad, β = 0; constant fz at TAS 40 m/s; no wind input; 500 steps: α within 0.01 rad. |
| T2 | `SteadyFlight_BetaFromSideforce_Converges` | True β = 0.05 rad; no wind; 1000 steps: β within 0.02 rad of true. |
| T3 | `AeroUpdates_SkippedBelowMinQdyn` | q_dyn = 10 Pa: covariance unchanged beyond propagation. |
| T4 | `KinematicUpdate_ConvergesBothAngles` | True α = 0.07 rad, β = 0.05 rad; wind converged; 500 steps: both within 0.01 rad. |
| T5 | `ClBias_Estimated_PreventsBiasOnAlpha` | True ΔCL = 0.08; 1000 steps: `cl_bias_nd` within 0.03 of true; α estimate within 0.01 rad regardless. |
| T6 | `CyBias_Estimated_PreventsBiasOnBeta` | True ΔCY = 0.05; 1000 steps: `cy_bias_nd` within 0.02 of true; β estimate within 0.015 rad regardless. |
| T7 | `PitchRateCorrection_ReducesAlphaError` | q = 0.2 rad/s with nonzero CLq; corrected version has lower α error than uncorrected after 200 steps. |
| T8 | `RollYawRateCorrection_ReducesBetaError` | p = 0.3 rad/s with nonzero CYp; corrected version has lower β error after 200 steps. |
| T9 | `IntegrityNominal_WhenSourcesConsistent` | Both aero and kinematic active, sources consistent: `integrity_status == Nominal`. |
| T10 | `IntegrityDegraded_LiftSourceFails` | Lift corrupted (CL offset = 0.5, large NIS): `aero_source_suspect == true`, `integrity_status == Degraded`, estimate continues from kinematic. |
| T11 | `IntegrityDegraded_KinematicSourceFails` | Wind estimate corrupted: `kinematic_source_suspect == true`, `integrity_status == Degraded`. |
| T12 | `IntegrityFailed_BothSourcesFail` | Both sources corrupted simultaneously: `integrity_status == Failed`. |
| T13 | `CrossCheck_DetectsLiftKinematicDisagreement` | True α = 0.07 rad; lift forced to return α_raw = 0.20 rad (icing simulation): `aero_source_suspect == true`. |
| T14 | `ProtectionLevel_WidensWhenDegraded` | `Degraded` status: `alpha_protection_rad` is larger than in `Nominal` status for the same filter covariance. |
| T15 | `SideforceNoise_IncreasesWithCoefficientUncertainty` | High `cy_beta_relative_uncertainty_nd`: β covariance converges more slowly than with low uncertainty. |
| T16 | `JsonRoundTrip_PreservesFilterState` | Serialize after 200 steps including NIS windows; deserialize; next step output identical. |
| T17 | `ProtoRoundTrip_PreservesFilterState` | Same as T16 with proto serialization. |
| T18 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `schema_version != 1` throws `std::runtime_error`. |
| T19 | `Reset_ReturnsToInitialState` | After 500 steps, `reset()`: state and covariance return to initial config values; NIS windows cleared. |
