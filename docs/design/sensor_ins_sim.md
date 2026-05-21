# INS Sensor — Simulation Model

This document is the design authority for `SensorInsSimulation` within the
`liteaerosim::sensor` namespace. It specifies a truth-plus-error INS simulation model
that produces identical `InsMeasurement` outputs to `NavigationFilter` (the flight code EKF
implementation) at approximately 35× lower computational cost.

`SensorInsSimulation` derives from `liteaerosim::sensor::Sensor`. The `DynamicElement`
lifecycle contract, NVI pattern, and base class requirements are defined in
[`docs/architecture/dynamic_element.md`](dynamic_element.md).

The flight code EKF implementation, including the 18-state error-state Kalman filter
and ArduPilot integration, is documented in
[`docs/architecture/navigation_filter.md`](navigation_filter.md).

---

## Design Rationale

The full EKF in `NavigationFilter` runs a 18×18 covariance propagation every step (~2 000
FLOPs, dominant cost). For batch simulation or Monte-Carlo analysis with many concurrent
aircraft this cost is unnecessary because the simulator already knows the truth state.

`SensorInsSimulation` replaces the EKF with a **truth-plus-error** model:

1. The truth navigation state is taken directly from the simulator.
2. Grade-parameterized bias errors evolve via first-order Gauss-Markov processes.
3. Error states (position, velocity, heading) accumulate or are bounded depending on
   operating mode and available aiding.
4. The `InsMeasurement` output equals truth plus the current error state plus sensor
   noise, mimicking what the EKF would produce without running the filter.

The result captures the behaviorally significant INS characteristics — error growth
rates, dead reckoning drift, alignment, wind estimation, bias instability — while
eliminating all matrix operations.

### Design Trade-off Summary

| Property | `NavigationFilter` (EKF) | `SensorInsSimulation` (truth+error) |
| --- | --- | --- |
| Mandatory FLOPs/step | ~2 100 | ~65 |
| RNG draws/step | 12 | 6 aided / 6 DR |
| Covariance propagation | Full 18×18 EKF | Analytical formula from grade params |
| Position error (aided) | Filter-converged, consistent | Bounded Gaussian noise from GNSS spec |
| Dead reckoning drift | Emerges from EKF dynamics | Explicit bias-driven integration |
| Wind estimation | EKF state (observable via TAS+GNSS) | Low-pass filter on GNSS−TAS residual |
| Reproduces exact noise sequence | Yes (seeded RNG) | Yes (seeded RNG) |
| ArduPilot HIL fidelity | Full | Not intended for HIL |

Use `NavigationFilter` when:

- Producing reference truth data for INS algorithm validation.
- Running HIL simulation where the EKF output drives an actual ArduPilot.
- Validating the ArduPilot integration directly.

Use `SensorInsSimulation` when:

- Running Monte-Carlo, batch, or many-aircraft simulation.
- The guidance/control subsystem consumes INS outputs and INS fidelity is not the
  primary variable under study.

---

## INS Grade Presets

`InsGrade` parameterizes the error model. Explicit overrides are available in
`InsSimConfig` for each noise parameter; when a field is left at 0, the grade preset
is used.

```cpp
// include/sensor/SensorInsSimulation.hpp
namespace liteaerosim::sensor {

enum class InsGrade : int32_t {
    Consumer   = 0,  // tactical MEMS IMU (e.g., ICM-42688, MPU-9250 class)
    Tactical   = 1,  // tactical FOG / high-grade MEMS (e.g., ADIS16488 class)
    Navigation = 2,  // navigation-grade IMU (e.g., HG1700 class)
};

} // namespace liteaerosim::sensor
```

Preset values:

| Grade | σ_ba (m/s²) | τ_a (s) | σ_bg (rad/s) | τ_g (s) | σ_att_align (rad) | σ_hdg_align (rad) |
| --- | --- | --- | --- | --- | --- | --- |
| Consumer | 0.05 | 100 | 1.0×10⁻³ | 100 | 0.009 (~0.5°) | 0.087 (~5°) |
| Tactical | 1.0×10⁻³ | 300 | 1.0×10⁻⁴ | 300 | 0.002 (~0.1°) | 0.009 (~0.5°) |
| Navigation | 1.0×10⁻⁴ | 600 | 1.0×10⁻⁵ | 600 | 0.0003 (~0.02°) | 0.0009 (~0.05°) |

`σ_ba` = 1σ accelerometer bias instability; `σ_bg` = 1σ gyroscope bias instability;
`τ_a`, `τ_g` = Gauss-Markov correlation times; `σ_att_align` = 1σ tilt error after
alignment; `σ_hdg_align` = 1σ heading error after static mag alignment.

---

## Data Structures

### `InsMeasurement`

Identical to the struct defined in [`navigation_filter.md`](navigation_filter.md). Both `NavigationFilter` and
`SensorInsSimulation` produce `InsMeasurement`; the simulation loop need not
distinguish between them.

### `InsSimConfig`

```cpp
// include/sensor/SensorInsSimulation.hpp
namespace liteaerosim::sensor {

struct InsSimConfig {
    // --- Grade preset (used when explicit override fields below are 0) ---
    InsGrade grade = InsGrade::Consumer;

    // --- Explicit IMU error overrides (0 = use grade preset) ---
    float accel_bias_mps2          = 0.f;   // 1σ accel bias instability (m/s²)
    float accel_correlation_time_s = 0.f;   // accel Gauss-Markov τ (s); 0 = grade preset
    float gyro_bias_rads           = 0.f;   // 1σ gyro bias instability (rad/s)
    float gyro_correlation_time_s  = 0.f;   // gyro Gauss-Markov τ (s); 0 = grade preset

    // --- Alignment errors (0 = use grade preset) ---
    float alignment_tilt_noise_rad    = 0.f;  // 1σ tilt (roll/pitch) error after alignment
    float alignment_heading_noise_rad = 0.f;  // 1σ heading error after static mag alignment

    // --- Aiding sensor noise (used for covariance output and bounded error in aided mode) ---
    float gnss_horizontal_noise_m = 3.f;   // GNSS horizontal position 1σ (m)
    float gnss_vertical_noise_m   = 5.f;   // GNSS vertical position 1σ (m)
    float gnss_velocity_noise_mps = 0.1f;  // GNSS velocity 1σ per component (m/s)
    float baro_noise_m            = 2.f;   // barometric altitude 1σ (m)

    // --- Magnetic reference (for heading correction in dead reckoning) ---
    Eigen::Vector3f mag_reference_ned_nt = {20000.f, 0.f, -45000.f};
    float           mag_declination_rad  = 0.f;

    // --- Alignment ---
    float alignment_time_s              = 5.f;   // minimum static duration before alignment (s)
    float min_gnss_sog_for_heading_mps  = 1.0f;  // minimum GNSS SOG to use COG for heading init

    // --- Simulation ---
    float    dt_s         = 0.01f;
    uint32_t seed         = 0;   // 0 = non-deterministic
    int      schema_version = 1;
};

} // namespace liteaerosim::sensor
```

---

## Step Interface

```cpp
InsMeasurement step(const KinematicState&     truth,
                    const GnssMeasurement*    gnss,       // nullptr if unavailable
                    const AirDataMeasurement* air_data,  // nullptr if unavailable
                    const MagMeasurement*     mag,        // nullptr if unavailable
                    const WindEstimate*       wind_est);  // nullptr = zero wind output
```

**`truth`**: true navigation state from the simulator (position, velocity, attitude).
This replaces the IMU integration performed by `NavigationFilter`. The INS output is `truth`
plus the internal error state.

**`wind_est`**: output of a separately maintained `WindEstimator` instance. The fields
`InsMeasurement::wind_ned_mps`, `accel_bias_mps2` (TAS bias context), and the wind
covariance elements of `covariance_diagonal[15–17]` are copied from `wind_est` when
non-null. Pass `nullptr` to output zero wind with maximum wind uncertainty. See
[`docs/architecture/wind_estimator.md`](wind_estimator.md).

**Aiding pointers**: same semantics as `NavigationFilter`. Each may be `nullptr`
independently. Mode transitions depend on GNSS availability.

---

## Error Model

All error state variables are scalar or `Eigen::Vector3f`; no matrix operations.

### Bias Evolution (both modes, every step)

Both accelerometer and gyroscope biases evolve as first-order Gauss-Markov processes.
The discrete update per step:

$$
\mathbf{b}_a \leftarrow \left(1 - \frac{dt}{\tau_a}\right)\mathbf{b}_a + \sigma_{b_a}\sqrt{\frac{2\,dt}{\tau_a}}\,\mathbf{n}_a, \quad \mathbf{n}_a \sim \mathcal{N}(0, I_3)
$$

$$
\mathbf{b}_g \leftarrow \left(1 - \frac{dt}{\tau_g}\right)\mathbf{b}_g + \sigma_{b_g}\sqrt{\frac{2\,dt}{\tau_g}}\,\mathbf{n}_g, \quad \mathbf{n}_g \sim \mathcal{N}(0, I_3)
$$

Six RNG draws per step (3 accel + 3 gyro). These bias states are output directly as
`InsMeasurement::accel_bias_mps2` and `InsMeasurement::gyro_bias_rads`.

### Fully Aided Error Model

When `mode == FullyAided` (GNSS available), position and velocity errors are bounded by
the GNSS fix quality. A fresh noise sample is drawn each step:

$$
\delta\mathbf{p}^N = \begin{bmatrix}
    \sigma_{h} n_N,\; \sigma_{h} n_E,\; \sigma_{v} n_D
\end{bmatrix}^T, \quad n_i \sim \mathcal{N}(0,1)
$$

$$
\delta\mathbf{v}^N = \sigma_{vel}\,\mathbf{n}_v, \quad \mathbf{n}_v \sim \mathcal{N}(0, I_3)
$$

where $\sigma_h$ = `gnss_horizontal_noise_m`, $\sigma_v$ = `gnss_vertical_noise_m`,
$\sigma_{vel}$ = `gnss_velocity_noise_mps`.

Six additional RNG draws in aided mode (3 position + 3 velocity). These are _not_ drawn
in dead reckoning mode, reducing the per-step draw count by half.

Attitude error is not accumulated in aided mode because the EKF would converge it to a
small, bounded level. The tilt components of `att_error_rad_` decay toward zero with time
constant `tau_att = 30 s` (approximating the EKF's steady-state convergence rate):

$$
\delta\boldsymbol{\psi}_{tilt} \leftarrow \delta\boldsymbol{\psi}_{tilt}\,e^{-dt/30}
$$

Heading error is partially corrected by the magnetometer (see
[Dead Reckoning Error Model](#dead-reckoning-error-model), magnetometer correction
sub-section). In aided mode the same correction applies; heading error is bounded.

Barometric altitude is bounded independently by baro noise:

$$
\delta h_{baro} = \sigma_{baro}\,n_b, \quad n_b \sim \mathcal{N}(0,1)
$$

This draw is not additional; it reuses the D-component position draw above when GNSS is
available (GNSS altitude and baro altitude noise are treated as correlated after EKF
aiding).

### Dead Reckoning Error Model

When `mode == DeadReckoning`, position and velocity errors accumulate driven by the
bias states. Let $t_{DR}$ be the elapsed time since GNSS was lost.

**Velocity error** integrates accelerometer bias projected to NED:

$$
\delta\mathbf{v}^N \leftarrow \delta\mathbf{v}^N + R_{N \leftarrow B}(\hat{q})\,\mathbf{b}_a\,dt
$$

where $\hat{q}$ is the current (truth) attitude quaternion.

**Position error** integrates velocity error:

$$
\delta\mathbf{p}^N \leftarrow \delta\mathbf{p}^N + \delta\mathbf{v}^N\,dt
$$

At GNSS loss, $\delta\mathbf{p}^N$ and $\delta\mathbf{v}^N$ are reset to the last aided
noise sample (from the final FullyAided step) so continuity is maintained.

**Heading error** accumulates from gyro z-bias (body-to-NED projection simplified to
the z-axis for near-level flight):

$$
\delta\psi \leftarrow \delta\psi + b_{g,z}\,dt
$$

**Magnetometer heading correction** (applied when `mag != nullptr`):
The expected body-frame field is computed from the truth attitude and the NED reference:

$$
\hat{\mathbf{B}}^B = R_{B \leftarrow N}(\hat{q})\,\mathbf{B}_{ref}^N
$$

The heading residual (scalar, from the horizontal projection of the field error):

$$
\epsilon_{mag} = \hat{B}_{h,y} - B_{meas,h,y}
$$

where the subscript $h$ denotes the horizontal-plane component after tilt compensation.
A proportional correction is applied:

$$
\delta\psi \leftarrow \delta\psi - k_{mag}\,\epsilon_{mag}\,dt, \quad k_{mag} = 0.5\;\text{rad}^{-1}
$$

This bounds heading error in dead reckoning to approximately $\sigma_{bg,z} / k_{mag}$
at steady state (for Consumer grade: ~1.0×10⁻³ / 0.5 ≈ 0.002 rad ≈ 0.1°), which
matches EKF steady-state heading uncertainty at this grade.

### Wind Output

`SensorInsSimulation` does not estimate wind internally. The caller is responsible for
maintaining a `WindEstimator` instance and passing its output as the `wind_est` argument
to `step()`. The fields `InsMeasurement::wind_ned_mps`, `tas_bias_mps` (in context),
and `covariance_diagonal[15–17]` are populated by copying directly from `wind_est`.

This design separates the wind estimation algorithm — a 5-state Kalman filter that
handles TAS error and sideslip bias — from the INS navigation error model. See
[`docs/architecture/wind_estimator.md`](wind_estimator.md) for the full
design and configuration of `WindEstimator`.

### Alignment

On `initialize()`, the model enters `InsMode::Aligning`. State is zeroed. The
`accel_bias_` and `gyro_bias_` Gauss-Markov processes run during alignment.

Alignment is declared complete after `alignment_time_s` has elapsed, using the same
logic as `NavigationFilter`:

- Roll and pitch from mean specific force: $\hat{\phi} = \text{atan2}(\bar{f}_y, \bar{f}_z)$,
  $\hat{\theta} = \text{atan2}(-\bar{f}_x, \sqrt{\bar{f}_y^2 + \bar{f}_z^2})$.
  (Derived from truth specific force, not from noisy IMU — the model represents the INS
  output, not the alignment algorithm itself.)
- Heading from tilt-compensated magnetometer, or from GNSS COG if
  `speed_over_ground_mps ≥ min_gnss_sog_for_heading_mps`.

Initial error state is drawn once at alignment completion:

$$
\delta\phi_0,\; \delta\theta_0 \sim \mathcal{N}(0,\; \sigma_{att\_align}^2)
$$

$$
\delta\psi_0 \sim \mathcal{N}(0,\; \sigma_{hdg\_align}^2)
$$

These represent the residual error after a real alignment procedure, parameterized by
`InsGrade`.

---

## Operating Modes

Mode transitions are identical to `NavigationFilter`:

```text
Uninitialized ──initialize()──► Aligning ──alignment complete──► FullyAided
                                                                      │    ▲
                                                               GNSS lost│    │GNSS regained (3 steps)
                                                                        ▼    │
                                                                  DeadReckoning
```

On GNSS regain (three consecutive valid GNSS fixes), the model re-enters `FullyAided`.
The accumulated position and velocity errors are replaced with fresh bounded noise
samples, representing the EKF re-anchor behavior.

---

## Covariance Diagnostic Output

`InsMeasurement::covariance_diagonal` is computed analytically rather than from a
running EKF. The 18 elements follow the error-state indexing defined in [`navigation_filter.md`](navigation_filter.md).

**In `FullyAided` mode** (constant values from config):

| Elements | Value |
| --- | --- |
| 0–1 (N, E position) | `gnss_horizontal_noise_m`² |
| 2 (D position / altitude) | `gnss_vertical_noise_m`² |
| 3–5 (velocity NED) | `gnss_velocity_noise_mps`² |
| 6–7 (tilt angles) | `alignment_tilt_noise_rad`² (grade preset) |
| 8 (heading) | `alignment_heading_noise_rad`² (grade preset) |
| 9–11 (accel bias) | `accel_bias_mps2`² (grade preset) |
| 12–14 (gyro bias) | `gyro_bias_rads`² (grade preset) |
| 15–17 (wind NED) | Copied from `WindEstimate::covariance_diagonal[0–2]`; maximum wind uncertainty if `wind_est == nullptr` |

**In `DeadReckoning` mode**, position and velocity covariances grow with elapsed DR
time $t_{DR}$:

$$
\sigma_{pos,h}^2(t_{DR}) = \sigma_{gnss,h}^2 + \tfrac{1}{2}\,\sigma_{ba}^2\,t_{DR}^4 + \sigma_{ba}^2\,\tau_a\,t_{DR}^2
$$

$$
\sigma_{vel,h}^2(t_{DR}) = \sigma_{gnss,vel}^2 + \sigma_{ba}^2\,t_{DR}^2
$$

The quartic and quadratic terms approximate the variance of position/velocity errors
driven by a random-walk bias. Heading covariance grows linearly:

$$
\sigma_{hdg}^2(t_{DR}) = \sigma_{hdg,0}^2 + \sigma_{bg}^2\,t_{DR}^2
$$

(bounded by magnetometer correction when mag is available). All other covariance
elements remain at the aided values.

---

## Class Interface

```cpp
// include/sensor/SensorInsSimulation.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <nlohmann/json.hpp>
#include "sensor/Sensor.hpp"
#include "sensor/SensorGnss.hpp"
#include "sensor/SensorAirData.hpp"
#include "sensor/SensorMag.hpp"
#include "estimation/WindEstimator.hpp"

namespace liteaerosim::sensor {

class SensorInsSimulation : public liteaerosim::sensor::Sensor {
public:
    InsMeasurement step(const KinematicState&     truth,
                        const GnssMeasurement*    gnss,
                        const AirDataMeasurement* air_data,
                        const MagMeasurement*     mag,
                        const WindEstimate*       wind_est);

    void serializeProto(liteaerosim::InsSimStateProto& proto) const;
    void deserializeProto(const liteaerosim::InsSimStateProto& proto);

protected:
    void onInitialize(const nlohmann::json& config) override;
    void onReset() override;
    nlohmann::json onSerializeJson() const override;
    void onDeserializeJson(const nlohmann::json& state) override;

private:
    struct RngState;
    InsSimConfig config_;

    // Resolved grade parameters (set at initialize() from grade preset or explicit overrides)
    float sigma_ba_;    // accel bias instability 1σ (m/s²)
    float tau_a_;       // accel Gauss-Markov τ (s)
    float sigma_bg_;    // gyro bias instability 1σ (rad/s)
    float tau_g_;       // gyro Gauss-Markov τ (s)
    float sigma_att_align_;  // tilt alignment error 1σ (rad)
    float sigma_hdg_align_;  // heading alignment error 1σ (rad)

    // Error state
    Eigen::Vector3f accel_bias_;         // body frame (m/s²)
    Eigen::Vector3f gyro_bias_;          // body frame (rad/s)
    Eigen::Vector3f pos_error_ned_m_;    // NED position error (m)
    Eigen::Vector3f vel_error_ned_mps_;  // NED velocity error (m/s)
    float           att_error_roll_rad_  = 0.f;
    float           att_error_pitch_rad_ = 0.f;
    float           att_error_yaw_rad_   = 0.f;

    // Mode and timing
    InsMode mode_             = InsMode::Uninitialized;
    bool    alignment_complete_ = false;
    float   align_elapsed_s_  = 0.f;
    float   dr_elapsed_s_     = 0.f;    // time since GNSS loss; 0 in FullyAided
    int     gnss_regain_count_ = 0;

    // Alignment accumulators (truth-derived, no IMU noise)
    int             align_count_        = 0;
    Eigen::Vector3f align_accel_sum_;
    Eigen::Vector3f align_mag_sum_;

    std::unique_ptr<RngState> rng_;

    // Internal helpers
    void resolvGradeParams();
    std::array<float, 18> computeCovariance() const;
    InsMeasurement buildOutput(const KinematicState& truth) const;
};

} // namespace liteaerosim::sensor
```

---

## Serialization Contract

### JSON State Fields

| JSON key | Description |
| --- | --- |
| `"schema_version"` | Integer; must equal 1. Throws on mismatch. |
| `"mode"` | `InsMode` as integer. |
| `"alignment_complete"` | Boolean. |
| `"align_elapsed_s"` | Elapsed alignment time (s). |
| `"align_count"` | Alignment step counter. |
| `"align_accel_sum"` | 3-element array (m/s²). |
| `"align_mag_sum"` | 3-element array (nT). |
| `"dr_elapsed_s"` | Elapsed dead reckoning time (s). |
| `"gnss_regain_count"` | Integer (0–3). |
| `"accel_bias_mps2"` | 3-element array (m/s²). |
| `"gyro_bias_rads"` | 3-element array (rad/s). |
| `"pos_error_ned_m"` | 3-element array (m). |
| `"vel_error_ned_mps"` | 3-element array (m/s). |
| `"att_error_roll_rad"` | Float (rad). |
| `"att_error_pitch_rad"` | Float (rad). |
| `"att_error_yaw_rad"` | Float (rad). |
| `"rng_seed"` | RNG seed actually used. |
| `"rng_advance"` | Number of variate draws since seeding. |

Schema version: **1**.

---

## Proto Messages

```proto
// proto/liteaerosim.proto

message InsSimStateProto {
    int32  schema_version    = 1;
    int32  mode              = 2;    // InsMode cast to int32
    bool   alignment_complete = 3;
    float  align_elapsed_s   = 4;
    int32  align_count       = 5;
    float  align_accel_sum_x = 6;
    float  align_accel_sum_y = 7;
    float  align_accel_sum_z = 8;
    float  align_mag_sum_x   = 9;
    float  align_mag_sum_y   = 10;
    float  align_mag_sum_z   = 11;
    float  dr_elapsed_s      = 12;
    int32  gnss_regain_count = 13;
    float  accel_bias_x_mps2 = 14;
    float  accel_bias_y_mps2 = 15;
    float  accel_bias_z_mps2 = 16;
    float  gyro_bias_x_rads  = 17;
    float  gyro_bias_y_rads  = 18;
    float  gyro_bias_z_rads  = 19;
    float  pos_error_n_m     = 20;
    float  pos_error_e_m     = 21;
    float  pos_error_d_m     = 22;
    float  vel_error_n_mps   = 23;
    float  vel_error_e_mps   = 24;
    float  vel_error_d_mps   = 25;
    float  att_error_roll_rad  = 26;
    float  att_error_pitch_rad = 27;
    float  att_error_yaw_rad   = 28;
    uint32 rng_seed          = 29;
    uint64 rng_advance       = 30;
}
```

---

## Computational Cost

### Memory Footprint

| Component | Size |
| --- | --- |
| Error state (6 vectors × 3 floats + 3 float scalars) | ~84 bytes |
| Wind estimate | 12 bytes |
| Mode, timing, counters | ~28 bytes |
| Alignment accumulators | ~28 bytes |
| `InsSimConfig` | ~120 bytes |
| `RngState` pimpl (`std::mt19937`) | ~2.5 KB |
| **Total active state (excl. RNG)** | **~280 bytes** |

### Operations per `step()` Call

| Sub-task | FLOPs | Draws | Notes |
| --- | --- | --- | --- |
| Accel bias Gauss-Markov (3 axes) | ~6 | 3 | Scale + add |
| Gyro bias Gauss-Markov (3 axes) | ~6 | 3 | Scale + add |
| Velocity error integration (DR only) | ~9 | 0 | R·b_a·dt + accumulate |
| Position error integration (DR only) | 3 | 0 | δp += δv·dt |
| Heading error drift + mag correction | ~6 | 0 | Scalar; mag correction optional |
| Attitude error decay (aided only) | 2 | 0 | Multiply by scalar |
| Wind output copy from `WindEstimate` | 3 | 0 | Copy 3 floats |
| Covariance diagonal computation | ~20 | 0 | 18 analytical terms |
| Attitude error → quaternion perturbation | ~10 | 0 | Small rotation applied to truth q |
| Output copy + Euler extraction | ~15 | 0 | |
| Aided position noise (aided only) | ~3 | 6 | 3 draws × multiply |
| **Total (aided)** | **~56** | **12** | |
| **Total (DR)** | **~53** | **6** | No position draws; different branches |

### Comparison with `NavigationFilter`

| Component | FLOPs/step | RNG draws/step | Approx. wall-clock |
| --- | --- | --- | --- |
| `SensorInsSimulation` — aided | ~56 | 12 | ~0.13 μs |
| `SensorInsSimulation` — DR | ~53 | 6 | ~0.11 μs |
| `WindEstimator` — update step | ~640 | 0 | ~1.3 μs |
| `WindEstimator` — propagation only | ~30 | 0 | ~0.06 μs |
| `NavigationFilter` — aided (EKF, incl. wind states) | ~4 200 | 12 | ~5–10 μs |
| `NavigationFilter` — propagation only | ~2 100 | 12 | ~3–6 μs |
| **`SensorInsSimulation` + `WindEstimator` vs. `NavigationFilter` aided** | **~25–40×** | | |

The saving comes almost entirely from eliminating the 18×18 covariance propagation
(~2 000 FLOPs) and EKF measurement updates (~2 100 FLOPs for all aiding). The
bias Gauss-Markov evolution is common to both models and is not saved.

At 100 Hz and 100 concurrent aircraft, `SensorInsSimulation` consumes approximately
100 × 100 × 0.15 μs = 1.5 ms/s — under 0.2% of available single-core time.
`NavigationFilter` under the same conditions would consume ~50–100 ms/s (~5–10% of a core),
making the simulation model a meaningful choice at scale.

---

## Test Requirements

All tests reside in `test/SensorInsSimulation_test.cpp`, test class
`SensorInsSimulationTest`.

| ID | Test Name | Description |
| --- | --- | --- |
| T1 | `StaticAlignment_ProducesCorrectAttitude` | After `alignment_time_s`, mode is `FullyAided` and attitude output matches truth to within 3× `alignment_tilt_noise_rad` (Consumer grade). |
| T2 | `FullyAided_PositionBoundedByGnssNoise` | 1000 steps with truth stationary: sample std of output position equals `gnss_horizontal_noise_m` within 20% (N and E axes). |
| T3 | `FullyAided_VelocityBoundedByGnssNoise` | Same setup: sample std of velocity output equals `gnss_velocity_noise_mps` within 20%. |
| T4 | `WindEstimate_CopiedFromExternalInput` | Pass a `WindEstimate` with `wind_ned_mps = {5, 3, 0}`: output `wind_ned_mps` equals `{5, 3, 0}` exactly. Pass `nullptr`: output `wind_ned_mps` equals `{0, 0, 0}`. |
| T5 | `DeadReckoning_PositionGrowsWithTime` | GNSS removed; 300 steps: output position error magnitude at step 300 > error at step 1 (monotonically increasing mean). |
| T6 | `DeadReckoning_CovarianceGrowsWithTime` | Same setup: `covariance_diagonal[0]` at step 300 > `covariance_diagonal[0]` at step 1. |
| T7 | `GnssRegain_ReanchorsPosition` | After 200 DR steps (position error > 10 m for Consumer grade), GNSS restored: next step position error drops to `gnss_horizontal_noise_m` order. |
| T8 | `BiasOutput_MatchesGaussMarkovDistribution` | N = 5000 steps; bias output sample std within 20% of configured `accel_bias_mps2`. |
| T9 | `IdenticalSeeds_IdenticalOutputs` | Two instances, same seed, same inputs: bitwise-identical output for N = 100 steps. |
| T10 | `JsonRoundTrip_PreservesState` | Serialize after 100 steps; deserialize into a new instance; next step output is identical for all fields. |
| T11 | `ProtoRoundTrip_PreservesState` | Same as T10 using `serializeProto()` / `deserializeProto()`. |
| T12 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `schema_version != 1` throws `std::runtime_error`. |
| T13 | `Reset_ReturnsToAligning` | After 200 steps, `reset()`: `mode == InsMode::Aligning` and `alignment_complete == false`. |
| T14 | `NavigationGrade_TighterAlignmentError` | Navigation grade: alignment heading error < Consumer grade alignment heading error (single run comparison). |
