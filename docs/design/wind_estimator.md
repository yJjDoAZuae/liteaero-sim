# Wind Estimator — Architecture and Interface Design

This document is the design authority for `WindEstimator` within the
`liteaerosim::estimation` namespace. It specifies a Kalman filter that estimates horizontal
ambient wind, TAS system bias, and persistent sideslip bias from GNSS ground velocity,
true airspeed, and attitude; plus a decoupled scalar estimator for vertical wind using
angle of attack.

`WindEstimator` derives from `liteaerosim::DynamicElement`. The lifecycle contract, NVI
pattern, and base class requirements are defined in
[`docs/architecture/dynamic_element.md`](dynamic_element.md).

---

## Scope

`WindEstimator` solves the aerodynamic wind triangle to separate ambient wind from
aircraft airspeed. It models two systematic error sources that corrupt a naive wind
triangle computation:

| Error source | Physical cause | Effect on naive horizontal wind estimate |
| --- | --- | --- |
| **TAS bias** `δ_tas` | Pitot installation factor, uncompensated compressibility, in-flight calibration residual | Wind error proportional to TAS; rotates with heading |
| **Sideslip bias** `β_bias` | Trim sideslip from rudder offset, asymmetric thrust, or CG offset | Wind error proportional to TAS × trim sideslip; 90° off heading |

Both are treated as slowly varying Gauss-Markov processes. They become separately
observable from wind because their NED projections rotate differently as the aircraft
changes heading.

### Horizontal / Vertical Decoupling

The vertical wind component requires angle of attack (`α`) to compute the vertical
component of the airspeed vector (`TAS · sin(α)`). At typical cruise AoA of 3–6°
and TAS of 50 m/s, this term is 2.6–5.2 m/s — large enough to dominate the vertical
wind estimate if omitted.

Horizontal wind estimation does not require AoA. The forward-component approximation
`cos(α) ≈ 1` introduces an error of less than 0.3% at 6° AoA. The bias states
`δ_tas` and `β_bias` do not affect the vertical measurement.

Therefore, horizontal and vertical wind are estimated by independent estimators:

| Estimator | States | Measurement | AoA required |
| --- | --- | --- | --- |
| Horizontal KF | `[w_N, w_E, δ_tas, β_bias]` | 2D: NE components of wind triangle | No |
| Vertical scalar KF | `w_D` | 1D: GNSS vertical velocity − TAS·sin(α) | Yes |

If `aoa_rad` is unavailable, the vertical estimator is suspended and the vertical wind
covariance grows at the process noise rate; no assumption about zero vertical wind is
made.

### Numerical Method — Standard EKF with Joseph Form

`WindEstimator` uses a standard EKF with the Joseph-form covariance update:

$$
P \leftarrow (I - KH)\,P\,(I - KH)^T + K\,R_{meas}\,K^T
$$

A square root (Cholesky, UDU, or Potter) formulation is **not used**. Justification:
the horizontal filter is 4×4 and the vertical filter is scalar. At this size, the
Joseph form provides sufficient numerical stability at `float` precision; the condition
number of a well-initialized 4×4 covariance is small and does not grow pathologically
for the process and observation rates this filter encounters. A square root
implementation would add meaningful code complexity for no observable benefit here.

---

## Aerodynamic Velocity Vector

The full body-frame airspeed vector (forward–right–down body frame):

$$
\mathbf{v}_{air}^B =
TAS_{meas} \begin{bmatrix}
    \cos\alpha \cos\beta \\
    \sin\beta \\
    \sin\alpha \cos\beta
\end{bmatrix}
\approx
\begin{bmatrix}
    TAS_{meas}(1 - \delta_{tas}/TAS_{meas}) \\
    TAS_{meas}\,\beta_{bias} \\
    TAS_{meas}\,\alpha
\end{bmatrix}
$$

for small angles, where:

- $\alpha$ = angle of attack (rad); a direct measurement supplied by the caller (from
  air data computer or pitch-rate / lift-based estimate).
- $\beta_{bias}$ = persistent trim sideslip (rad); **estimated by the horizontal KF**.
  This is not the instantaneous aerodynamic sideslip from a dynamic maneuver — it is
  the steady-state offset due to trim, asymmetric thrust, or CG. An aircraft flying a
  coordinated turn has instantaneous $\beta = 0$ but may still have a nonzero $\beta_{bias}$.
- $\delta_{tas}$ = TAS system bias (m/s); **estimated by the horizontal KF**.

The horizontal components are dominated by the forward term; the vertical component is
dominated by the AoA term. This asymmetry motivates the horizontal/vertical split.

---

## Horizontal Wind KF

### State Vector

$$
\mathbf{x}_h = \begin{bmatrix} w_N & w_E & \delta_{tas} & \beta_{bias} \end{bmatrix}^T
$$

| Index | State | Units | Description |
| --- | --- | --- | --- |
| 0 | $w_N$ | m/s | North component of ambient wind |
| 1 | $w_E$ | m/s | East component of ambient wind |
| 2 | $\delta_{tas}$ | m/s | TAS system bias (positive = measured exceeds true) |
| 3 | $\beta_{bias}$ | rad | Persistent trim sideslip (positive = nose-left trim) |

### Process Model

$$
F_h = \mathrm{diag}\!\left(0,\; 0,\; -\tfrac{1}{\tau_{tas}},\; -\tfrac{1}{\tau_\beta}\right)
$$

$$
\Phi_h = \mathrm{diag}\!\left(1,\; 1,\; e^{-dt/\tau_{tas}},\; e^{-dt/\tau_\beta}\right)
$$

Diagonal $\Phi_h$ means the covariance propagation $P_h \leftarrow \Phi_h P_h \Phi_h^T + Q_{d,h}$
reduces to per-element row/column scaling plus a diagonal addition — approximately 25 FLOPs.

Continuous-time process noise (diagonal):

| State | $q_{c,i}$ |
| --- | --- |
| $w_N, w_E$ | $\sigma_w^2$ |
| $\delta_{tas}$ | $2\,\sigma_{tas}^2\,/\,\tau_{tas}$ |
| $\beta_{bias}$ | $2\,\sigma_\beta^2\,/\,\tau_\beta$ |

Discrete: $Q_{d,h} = Q_{c,h} \cdot dt$.

### Measurement Model

Applied when `gnss_velocity_ned_mps != nullptr`. Uses only the North and East
components of the wind triangle to avoid AoA contamination of the horizontal estimate.

Let $\mathbf{r}_1 = R_{N \leftarrow B}[:,0]$ (body forward direction in NED) and
$\mathbf{r}_2 = R_{N \leftarrow B}[:,1]$ (body right direction in NED). Define the
2D projections:

$$
r_{1,h} = \begin{bmatrix} r_{1,N} \\ r_{1,E} \end{bmatrix}, \qquad
r_{2,h} = \begin{bmatrix} r_{2,N} \\ r_{2,E} \end{bmatrix}
$$

Observation (2-vector):

$$
\mathbf{z}_h = \begin{bmatrix} v_{gnd,N} \\ v_{gnd,E} \end{bmatrix}
             - TAS_{meas} \begin{bmatrix} r_{1,N} \\ r_{1,E} \end{bmatrix}
$$

Jacobian ($2 \times 4$):

$$
H_h = \begin{bmatrix} I_2 & -\mathbf{r}_{1,h} & TAS_{meas}\,\mathbf{r}_{2,h} \end{bmatrix}
$$

Measurement noise: $R_h = \sigma_{vel}^2 \cdot I_2$.

**Update skipped** when $TAS_{meas} < 1.0\,\text{m/s}$ (ill-conditioned $\beta_{bias}$
column of $H_h$).

### KF Update (Joseph Form)

$$
K_h = P_h H_h^T \left(H_h P_h H_h^T + R_h\right)^{-1}
$$

$$
\hat{\mathbf{x}}_h \mathrel{+}= K_h\,\left(\mathbf{z}_h - H_h\,\hat{\mathbf{x}}_h\right)
$$

$$
P_h \leftarrow (I - K_h H_h)\,P_h\,(I - K_h H_h)^T + K_h\,R_h\,K_h^T
$$

The 2×2 innovation covariance $S = H_h P_h H_h^T + R_h$ is inverted analytically
(closed-form 2×2 inverse).

---

## Vertical Wind Scalar KF

### State

$$
x_D = w_D \quad \text{(m/s, positive = downward in NED)}
$$

### Vertical Process Model

Random walk: $P_D \leftarrow P_D + q_{d,D} \cdot dt$, where
$q_{d,D} = \sigma_{wD}^2$ is the vertical wind process noise power spectral density.

### Vertical Measurement Model

Applied when both `gnss_velocity_ned_mps != nullptr` and `aoa_rad` is finite (not
`NaN`).

The vertical component of the airspeed vector in NED (for near-level flight where the
pitch contribution from body z-axis to NED is dominated by AoA):

$$
v_{air,D}^N \approx -TAS_{meas} \sin\alpha \approx -TAS_{meas}\,\alpha
$$

(negative because a positive AoA tilts the velocity vector upward, i.e., negative NED-D).

Observation:

$$
z_D = v_{gnd,D} - v_{air,D}^N = v_{gnd,D} + TAS_{meas}\,\alpha
$$

Jacobian: $H_D = 1$ (scalar).

Measurement noise: $R_D = \sigma_{vel,D}^2 + (TAS_{meas} \cdot \sigma_\alpha)^2$,
where $\sigma_\alpha$ = `aoa_noise_rad` accounts for uncertainty in the AoA measurement.
This inflates the vertical wind measurement noise when AoA is noisy (e.g., derived from
an accelerometer-based estimator rather than a dedicated vane).

### Scalar KF Update

$$
K_D = P_D\,/\,(P_D + R_D)
$$

$$
\hat{x}_D \mathrel{+}= K_D\,(z_D - \hat{x}_D)
$$

$$
P_D \leftarrow (1 - K_D)\,P_D
$$

(Joseph form is trivially identical to the simplified form for scalar updates.)

---

## Observability

### Horizontal States

| Flight condition | Observability |
| --- | --- |
| Constant heading | $w_N, w_E$: observable. $\delta_{tas}, \beta_{bias}$: unobservable — their NED projections are collinear with fixed heading. |
| 90° heading change | $\delta_{tas}$: observable (effect rotates to orthogonal NE axis). |
| 180° heading change | $\delta_{tas}$: fully observable. $\beta_{bias}$: partially observable. |
| Full circuit or S-turn | All 4 states observable. |

The filter covariance reflects this automatically: $P_{h,2,2}$ and $P_{h,3,3}$ remain
large during straight flight and decrease through maneuvers.

### Vertical State

$w_D$ is observable at every step when GNSS velocity and a valid AoA measurement are
available. AoA quality (encoded in $\sigma_\alpha$) directly controls how quickly the
vertical wind estimate converges.

---

## Data Structures

### `WindEstimate`

```cpp
// include/estimation/WindEstimator.hpp
namespace liteaerosim::estimation {

struct WindEstimate {
    Eigen::Vector3f wind_ned_mps;             // estimated ambient wind in NED frame (m/s)
    float           tas_bias_mps;             // estimated TAS system bias (m/s)
    float           sideslip_bias_rad;        // estimated persistent trim sideslip (rad)

    // 1σ² per state: indices [w_N, w_E, δ_tas, β_bias, w_D]
    std::array<float, 5> covariance_diagonal;

    bool is_converged;   // true when horizontal wind σ < convergence_wind_sigma_mps
                         // and vertical wind σ < convergence_wind_sigma_mps (if AoA available)
};

} // namespace liteaerosim::sensor
```

### `WindEstimatorConfig`

```cpp
// include/estimation/WindEstimator.hpp
namespace liteaerosim::estimation {

struct WindEstimatorConfig {
    // --- Horizontal process noise ---
    float wind_process_noise_mps_per_sqrthz = 0.5f;   // horizontal wind random walk σ_w (m/s/√Hz)

    // --- TAS bias model ---
    float tas_bias_instability_mps           = 1.0f;   // Gauss-Markov steady-state 1σ (m/s)
    float tas_correlation_time_s             = 600.f;  // τ_tas (s)
    float initial_tas_bias_uncertainty_mps   = 2.0f;   // initial 1σ (m/s)

    // --- Sideslip bias model ---
    float sideslip_bias_instability_rad      = 0.02f;  // Gauss-Markov steady-state 1σ (rad ~1.1°)
    float sideslip_correlation_time_s        = 300.f;  // τ_β (s)
    float initial_sideslip_uncertainty_rad   = 0.1f;   // initial 1σ (rad ~5.7°)

    // --- Initial horizontal wind uncertainty ---
    float initial_wind_h_uncertainty_mps    = 10.f;   // w_N, w_E initial 1σ (m/s)

    // --- Vertical wind ---
    float wind_vertical_process_noise_mps_per_sqrthz = 0.3f;  // w_D random walk σ_wD (m/s/√Hz)
    float initial_wind_d_uncertainty_mps    = 10.f;   // w_D initial 1σ (m/s)

    // --- AoA measurement noise ---
    // Used to inflate vertical wind measurement noise R_D = σ_vel_D² + (TAS·σ_α)².
    // Set to 0 if AoA is from a dedicated vane with known accuracy; use a larger value
    // (~0.05 rad) for AoA derived from an accelerometer-based estimator.
    float aoa_noise_rad = 0.02f;   // 1σ AoA measurement uncertainty (rad ~1.1°)

    // --- Measurement noise ---
    float gnss_velocity_noise_mps = 0.1f;   // GNSS velocity 1σ per NED component (m/s)

    // --- Convergence ---
    float convergence_wind_sigma_mps = 1.0f;  // declare converged when all wind σ < this

    float dt_s           = 0.01f;
    int   schema_version = 1;
};

} // namespace liteaerosim::sensor
```

---

## Step Interface

```cpp
WindEstimate step(const Eigen::Quaternionf& q_body_from_ned,
                  float                    tas_mps,
                  float                    aoa_rad,                    // NaN if unavailable
                  const Eigen::Vector3f*   gnss_velocity_ned_mps);    // nullptr if unavailable
```

**`q_body_from_ned`**: attitude quaternion rotating a NED vector to body frame. Used to
build $R_{N \leftarrow B}$ and extract $\mathbf{r}_{1,h}$, $\mathbf{r}_{2,h}$.

**`tas_mps`**: true airspeed from the air data computer (m/s). Includes the TAS system
bias `δ_tas` that the filter estimates.

**`aoa_rad`**: angle of attack (rad). Supply `std::numeric_limits<float>::quiet_NaN()`
when unavailable. When NaN, the vertical wind estimator runs propagation only; no
measurement update is applied and vertical wind covariance grows.

**`gnss_velocity_ned_mps`**: pointer to GNSS NED velocity (m/s); `nullptr` when GNSS
is unavailable. Both the horizontal KF and vertical estimator are propagation-only when
`nullptr`.

---

## Class Interface

```cpp
// include/estimation/WindEstimator.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include "DynamicElement.hpp"

namespace liteaerosim::estimation {

class WindEstimator : public liteaerosim::DynamicElement {
public:
    WindEstimate step(const Eigen::Quaternionf& q_body_from_ned,
                      float                    tas_mps,
                      float                    aoa_rad,
                      const Eigen::Vector3f*   gnss_velocity_ned_mps);

    void serializeProto(liteaerosim::WindEstimatorStateProto& proto) const;
    void deserializeProto(const liteaerosim::WindEstimatorStateProto& proto);

protected:
    void onInitialize(const nlohmann::json& config) override;
    void onReset() override;
    nlohmann::json onSerializeJson() const override;
    void onDeserializeJson(const nlohmann::json& state) override;

private:
    WindEstimatorConfig config_;

    // Horizontal KF (4-state)
    Eigen::Matrix<float, 4, 1> x_h_;   // [w_N, w_E, δ_tas, β_bias]
    Eigen::Matrix<float, 4, 4> P_h_;

    // Precomputed horizontal process model (constant once initialized)
    Eigen::Matrix<float, 4, 1> phi_h_diag_;   // diagonal of Φ_h
    Eigen::Matrix<float, 4, 1> q_d_h_diag_;   // diagonal of Q_{d,h}

    // Vertical scalar KF
    float x_d_  = 0.f;   // w_D estimate (m/s)
    float P_d_  = 0.f;   // w_D variance (m/s)²
    float q_d_d_ = 0.f;  // vertical process noise per step (precomputed at init)
};

} // namespace liteaerosim::sensor
```

---

## Serialization Contract

### JSON State Fields

| JSON key | Description |
| --- | --- |
| `"schema_version"` | Integer; must equal 1. Throws on mismatch. |
| `"x_h"` | 4-element array `[w_N, w_E, δ_tas, β_bias]`. |
| `"P_h_upper"` | 10-element upper triangle of the 4×4 symmetric $P_h$, row-major. |
| `"x_d"` | Float; vertical wind estimate $w_D$ (m/s). |
| `"P_d"` | Float; vertical wind variance (m/s)². |

Schema version: **1**.

---

## Proto Messages

```proto
// proto/liteaerosim.proto

message WindEstimatorStateProto {
    int32  schema_version  = 1;
    float  w_n_mps         = 2;
    float  w_e_mps         = 3;
    float  tas_bias_mps    = 4;
    float  sideslip_bias_rad = 5;
    repeated float p_h_upper = 6;   // 10 elements; upper triangle of P_h, row-major
    float  w_d_mps         = 7;
    float  p_d_nd          = 8;     // vertical wind variance
}
```

---

## Computational Cost

### Memory Footprint

| Component | Size |
| --- | --- |
| Horizontal KF state (4 floats) | 16 bytes |
| Horizontal KF covariance (4×4 floats) | 64 bytes |
| Precomputed Φ_h diagonal + Q_{d,h} diagonal (8 floats) | 32 bytes |
| Vertical scalar state + variance + process noise (3 floats) | 12 bytes |
| `WindEstimatorConfig` | ~60 bytes |
| **Total active state** | **~184 bytes** |

No RNG state — the filter is deterministic given measurements.

### Operations per `step()` Call

**Horizontal propagation (every step):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| Diagonal Φ_h scaling of P_h | ~16 | 4 row + 4 column scales (diagonal structure) |
| Q_{d,h} addition (4 diagonal elements) | 4 | |
| **Horizontal propagation total** | **~20** | |

**Vertical propagation (every step):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| $P_D \mathrel{+}= q_{d,D}$ | 1 | Scalar add |
| **Vertical propagation total** | **1** | |

**Horizontal update (when GNSS available):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| Rotation matrix columns r₁, r₂ from q | ~18 | Two quaternion-frame transforms |
| 2D projection r_{1,h}, r_{2,h} | 4 | Extract NE elements |
| Innovation z_h (2-vector) | ~6 | GNSS NE − TAS·r_{1,h} |
| H_h assembly (2×4) | ~4 | Set columns |
| H_h P_h (2×4 × 4×4 → 2×4) | ~32 | |
| H_h P_h H_hᵀ + R_h (→ 2×2 S) | ~16 | |
| S⁻¹ (2×2, analytical) | 6 | |
| P_h H_hᵀ (4×4 × 4×2 → 4×2) | ~32 | |
| K_h = P_h H_hᵀ S⁻¹ (4×2 × 2×2 → 4×2) | ~16 | |
| Innovation correction K_h z_h (4×2 × 2×1) | 8 | |
| State update | 4 | |
| K_h H_h (4×2 × 2×4 → 4×4) | ~32 | |
| Joseph term 1: (I−K_hH_h) P_h (I−K_hH_h)ᵀ | ~64 | |
| Joseph term 2: K_h R_h K_hᵀ (4×2 × 2×2 × 2×4) | ~32 | |
| P_h symmetrization | 6 | |
| **Horizontal update total** | **~280** | |

**Vertical update (when GNSS and AoA available):**

| Sub-task | FLOPs | Notes |
| --- | --- | --- |
| $z_D = v_{gnd,D} + TAS \cdot \alpha$ | 2 | |
| $R_D = \sigma_{vel}^2 + (TAS \cdot \sigma_\alpha)^2$ | 3 | Precomputed σ_vel² cached |
| $K_D = P_D / (P_D + R_D)$ | 2 | |
| State + covariance update | 3 | |
| **Vertical update total** | **~10** | |

**Summary:**

| Scenario | FLOPs/step | Notes |
| --- | --- | --- |
| Propagation only (no GNSS) | ~21 | |
| GNSS, no AoA | ~301 | Horizontal KF update + propagation |
| GNSS + AoA | ~311 | Both updates |

At 100 Hz with GNSS updates at 10 Hz: average ~(9×21 + 1×311)/10 ≈ **50 FLOPs/step**.

---

## Test Requirements

All tests reside in `test/WindEstimator_test.cpp`, test class `WindEstimatorTest`.

| ID | Test Name | Description |
| --- | --- | --- |
| T1 | `ConstantHeading_HorizWindConvergesBiasesDoNot` | 1000 steps, constant heading, true wind `{5, 3, 0}` m/s, zero TAS bias, zero sideslip: horizontal wind estimate within 1 m/s. `covariance_diagonal[2]` (TAS bias) and `[3]` (sideslip) remain > 50% of initial. |
| T2 | `HeadingChange90_TasBiasBecomesObservable` | After straight flight, 90° heading change, 500 more steps: `covariance_diagonal[2]` < 50% of initial. |
| T3 | `TasBiasInjected_EstimatedAfterCircuit` | True TAS bias = 2 m/s; full 360° heading sweep; `tas_bias_mps` within 0.5 m/s of true. |
| T4 | `SideslipBiasInjected_EstimatedAfterCircuit` | True sideslip = 0.05 rad; full 360° heading sweep at 30 m/s TAS; `sideslip_bias_rad` within 0.02 rad of true. |
| T5 | `VerticalWind_EstimatedWithAoA` | True $w_D$ = −2 m/s (upward); AoA = 0.08 rad (≈4.6°); TAS = 40 m/s; 500 steps: `wind_ned_mps.z()` within 0.5 m/s of −2 m/s. |
| T6 | `VerticalWind_SuspendedWithoutAoA` | `aoa_rad = NaN` for 500 steps: `covariance_diagonal[4]` grows monotonically; `wind_ned_mps.z()` unchanged from initial. |
| T7 | `VerticalWind_AoANoise_IncreasesRD` | Two instances: `aoa_noise_rad = 0` vs `0.05 rad`; after same number of steps, instance with higher noise has larger `covariance_diagonal[4]`. |
| T8 | `NoGnss_CovariancesGrow` | 200 steps, `gnss_velocity_ned_mps = nullptr`: `covariance_diagonal[0]` and `[4]` at step 200 > at step 0. |
| T9 | `LowTas_HorizontalUpdateSkipped` | `tas_mps = 0.5 m/s` (below threshold): horizontal covariance unchanged beyond propagation. |
| T10 | `ConvergenceFlag_SetWhenBelowThreshold` | After full-circuit maneuver with both GNSS and AoA available: `is_converged == true`. |
| T11 | `JsonRoundTrip_PreservesFilterState` | Serialize after 200 steps; deserialize into new instance; next `step()` output is identical. |
| T12 | `ProtoRoundTrip_PreservesFilterState` | Same as T11 using `serializeProto()` / `deserializeProto()`. |
| T13 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `schema_version != 1` throws `std::runtime_error`. |
| T14 | `Reset_ReturnsToInitialCovariance` | After 500 steps, `reset()`: `covariance_diagonal` returns to initial uncertainty values. |
