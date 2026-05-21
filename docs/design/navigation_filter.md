# INS — Flight Code Algorithm

This document is the design authority for `NavigationFilter` within the `liteaerosim::estimation`
namespace. It specifies the flight code INS algorithm: an 18-state error-state Kalman
filter aiding a strapdown mechanization with GNSS, air data, and magnetometer. The
design targets deployment on ArduPilot-compatible flight hardware as a companion-computer
INS; its outputs are mapped to ArduPilot's external navigation interface
(see [ArduPilot Integration](#ardupilot-integration)). The same code runs unmodified in
HIL simulation.

`NavigationFilter` derives from `DynamicElement`. The `DynamicElement` lifecycle contract,
NVI pattern, and base class requirements are defined in
[`docs/architecture/dynamic_element.md`](dynamic_element.md).

> **Simulation use:** For Monte-Carlo and batch simulation where EKF computation cost is
> undesirable, use `SensorInsSimulation` instead. It produces identical `InsMeasurement`
> outputs via a truth-plus-error model at ~35× lower computational cost. See
> [`docs/architecture/sensor_ins_sim.md`](sensor_ins_sim.md).

---

## Scope

`NavigationFilter` models a strapdown inertial navigator with the following aiding sources:

| Aiding Source | Sensor | Measurements used |
| --- | --- | --- |
| GNSS | `SensorGnss` | Position (lat/lon, altitude) and NED velocity |
| Air data | `SensorAirData` | Barometric altitude and true airspeed (TAS) |
| Magnetometer | `SensorMag` | 3-axis body-frame magnetic field |

Each aiding source is optional at every `step()` call. The INS automatically selects
operating mode based on what is available.

**Outputs** (`InsMeasurement`): WGS84 latitude and longitude, inertially-smoothed
barometric altitude, NED velocity, attitude (quaternion and Euler angles), estimated wind
in NED, IMU bias estimates, 18-state error covariance, operating mode, and a back-computed
Kollsman pressure setting.

**Operating modes**: Aligning → FullyAided → DeadReckoning (GNSS lost). Transitions
between FullyAided and DeadReckoning are automatic.

`NavigationFilter` lives in the Domain Layer. It has no I/O, no unit conversions, and no display
logic.

---

## Architecture Overview

```text
                       ┌──────────────────────────────────────────────┐
   True specific       │ NavigationFilter                                     │
   force  ────────────►│  ┌──────────────┐    ┌──────────────────┐    │
                        │  │ IMU Error    │    │  Mechanization   │    │
   True angular        │  │ Model        ├───►│  (propagation)   │    │
   rate ─────────────► │  │ (bias + ARW) │    │  q, v, p         │    │
                        │  └──────────────┘    └────────┬─────────┘    │
                        │                               │              │
   GNSS ──────────────►│  ┌──────────────────────────► │  EKF         │  InsMeasurement
   AirData ───────────►│  │  Measurement               │  Update      ├──────────────►
   Mag ───────────────►│  │  Models                    │              │
                        │  └──────────────────────────► │              │
                        └──────────────────────────────────────────────┘
```

Processing each timestep:

1. Apply IMU error model (bias + noise) to true inputs → corrupted IMU measurements.
2. Run strapdown mechanization with corrupted IMU → propagate position, velocity, attitude.
3. Propagate EKF covariance (`P = Φ P Φᵀ + Q`).
4. For each available aiding measurement: compute innovation and apply EKF update.
5. Apply corrections to navigation state; output `InsMeasurement`.

---

## Data Structures

### `InsMode`

```cpp
// include/estimation/NavigationFilter.hpp
namespace liteaerosim::estimation {

enum class InsMode : int32_t {
    Uninitialized = 0,  // before initialize() is called
    Aligning      = 1,  // collecting data for initial attitude / position solution
    FullyAided    = 2,  // GNSS + baro + magnetometer all contributing
    DeadReckoning = 3,  // GNSS lost; baro + TAS + magnetometer only
};

} // namespace liteaerosim::sensor
```

---

### `InsMeasurement`

```cpp
// include/estimation/NavigationFilter.hpp
namespace liteaerosim::estimation {

struct InsMeasurement {
    // --- Navigation state ---
    double          latitude_rad;          // estimated WGS84 geodetic latitude (rad)
    double          longitude_rad;         // estimated WGS84 geodetic longitude (rad)
    float           altitude_baro_m;       // inertially-smoothed barometric altitude (m MSL); not WGS84
    Eigen::Vector3f velocity_ned_mps;      // estimated NED velocity (m/s)

    // --- Attitude ---
    Eigen::Quaternionf attitude_q;         // estimated attitude: body ← NED quaternion
    float           roll_rad;              // roll angle (rad); derived from attitude_q
    float           pitch_rad;             // pitch angle (rad); derived from attitude_q
    float           yaw_rad;              // yaw (true heading) angle (rad) [0, 2π); derived from attitude_q

    // --- Derived estimates ---
    Eigen::Vector3f wind_ned_mps;          // estimated ambient wind in NED frame (m/s)
    Eigen::Vector3f accel_bias_mps2;       // estimated accelerometer bias in body frame (m/s²)
    Eigen::Vector3f gyro_bias_rads;        // estimated gyroscope bias in body frame (rad/s)

    // --- Error covariance (diagonal of 18×18 P matrix) ---
    // One standard deviation per error state (see Error-State Vector section).
    // Full covariance available via serializeJson() / serializeProto() only.
    std::array<float, 18> covariance_diagonal;

    // --- Altimeter aiding output ---
    float           kollsman_pa;           // back-computed QNH from GNSS altitude aiding (Pa);
                                           // valid only when GNSS position is available;
                                           // set to 0 when GNSS unavailable.

    // --- Status ---
    InsMode         mode;
    bool            alignment_complete;    // true once the INS has a valid initial solution
};

} // namespace liteaerosim::sensor
```

---

### `InsConfig`

```cpp
// include/estimation/NavigationFilter.hpp
namespace liteaerosim::estimation {

struct InsConfig {
    // --- IMU noise model ---
    float accel_noise_mps2_per_sqrthz   = 0.f;   // accelerometer velocity random walk (m/s²/√Hz)
    float accel_bias_instability_mps2   = 0.f;   // in-run accel bias instability 1σ (m/s²)
    float accel_correlation_time_s      = 100.f; // accel Gauss-Markov correlation time τ_a (s)
    float gyro_noise_rad_per_sqrthz     = 0.f;   // gyroscope angle random walk (rad/s/√Hz)
    float gyro_bias_instability_rads    = 0.f;   // in-run gyro bias instability 1σ (rad/s)
    float gyro_correlation_time_s       = 100.f; // gyro Gauss-Markov correlation time τ_g (s)

    // --- Initial covariance (1σ values at start of aided navigation) ---
    float initial_position_uncertainty_m    = 5.f;     // position 1σ (applied isotropically to N, E, D)
    float initial_velocity_uncertainty_mps  = 0.1f;    // velocity 1σ per component (m/s)
    float initial_attitude_uncertainty_rad  = 0.017f;  // attitude 1σ per angle (~1 deg)
    float initial_bias_accel_uncertainty_mps2 = 0.05f; // accel bias 1σ (m/s²)
    float initial_bias_gyro_uncertainty_rads  = 0.001f;// gyro bias 1σ (rad/s)
    float initial_wind_uncertainty_mps      = 3.f;     // wind estimate 1σ per component (m/s)

    // --- Process noise (wind random walk) ---
    float wind_process_noise_mps_per_sqrthz = 0.1f;    // wind state diffusion (m/s/√Hz)

    // --- Measurement noise (overrides aiding sensor config when provided as aiding to INS) ---
    // Set to 0 to use the aiding sensor's own noise level as the EKF R matrix.
    // When > 0, these override.
    float gnss_horizontal_noise_m    = 0.f;   // GNSS horizontal position noise for EKF (m)
    float gnss_vertical_noise_m      = 0.f;   // GNSS vertical position noise for EKF (m)
    float gnss_velocity_noise_mps    = 0.f;   // GNSS velocity noise per component for EKF (m/s)
    float baro_noise_m               = 2.f;   // barometric altitude noise for EKF (m)
    float airspeed_noise_mps         = 0.5f;  // TAS noise for EKF (m/s)
    float mag_noise_nt               = 0.f;   // magnetometer noise for EKF (nT); 0 = use SensorMag config

    // --- Magnetic reference field in NED frame (nT) ---
    // Caller sets this to the expected local magnetic field for heading aiding.
    Eigen::Vector3f mag_reference_ned_nt = {20000.f, 0.f, -45000.f};

    // --- Magnetic declination (rad) ---
    float mag_declination_rad = 0.f;   // difference between true and magnetic north; positive = east

    // --- Alignment ---
    float alignment_time_s = 5.f;    // minimum static data duration before alignment is declared complete (s)
    float min_gnss_sog_for_heading_mps = 1.0f; // minimum GNSS SOG to use GNSS velocity for heading alignment (m/s)

    // --- Timing ---
    float    dt_s         = 0.01f;   // filter update timestep (s)
    uint32_t seed         = 0;       // RNG seed; 0 = non-deterministic
    int      schema_version = 1;
};

} // namespace liteaerosim::sensor
```

---

## Step Interface

```cpp
InsMeasurement step(const Eigen::Vector3f&  true_specific_force_body_mps2,
                    const Eigen::Vector3f&  true_angular_rate_body_rads,
                    const GnssMeasurement*  gnss,      // nullptr if unavailable
                    const AirDataMeasurement* air_data,// nullptr if unavailable
                    const MagMeasurement*   mag);      // nullptr if unavailable
```

**`true_specific_force_body_mps2`**: true specific force in body frame (m/s²). Specific
force is the total acceleration minus gravity: $\mathbf{f}^B = R_{B \leftarrow N}(\dot{\mathbf{v}}^N + \mathbf{g}^N)$ where $\mathbf{g}^N = [0, 0, g_0]^T$. For a stationary level aircraft: $\mathbf{f}^B = [0, 0, -g_0]^T$ (z-down body frame).

**`true_angular_rate_body_rads`**: true angular rate of body relative to inertial frame,
expressed in body frame (rad/s).

**Aiding pointers**: each may be `nullptr` independently. The INS applies only the
updates for which a non-null pointer is provided. Mode transitions are automatic.

---

## IMU Error Model

The INS internally corrupts the true inputs before running the mechanization:

$$
\mathbf{f}^B_{meas} = \mathbf{f}^B_{true} + \mathbf{b}_a + \mathbf{n}_a, \quad
\mathbf{n}_a \sim \mathcal{N}(0,\, \sigma_a^2 / dt)
$$

$$
\boldsymbol{\omega}^B_{meas} = \boldsymbol{\omega}^B_{true} + \mathbf{b}_g + \mathbf{n}_g, \quad
\mathbf{n}_g \sim \mathcal{N}(0,\, \sigma_g^2 / dt)
$$

where $\sigma_a$ = `accel_noise_mps2_per_sqrthz` and $\sigma_g$ = `gyro_noise_rad_per_sqrthz`
are the noise spectral densities. Division by $\sqrt{dt}$ converts PSD to discrete-time
variance.

Bias dynamics follow a first-order Gauss-Markov process:

$$
\dot{\mathbf{b}}_a = -\frac{1}{\tau_a}\mathbf{b}_a + \mathbf{w}_a, \quad
\mathbf{w}_a \sim \mathcal{N}\!\left(0,\, \frac{2\,\sigma_{ba}^2}{\tau_a \, dt}\right)
$$

$$
\dot{\mathbf{b}}_g = -\frac{1}{\tau_g}\mathbf{b}_g + \mathbf{w}_g, \quad
\mathbf{w}_g \sim \mathcal{N}\!\left(0,\, \frac{2\,\sigma_{bg}^2}{\tau_g \, dt}\right)
$$

where $\sigma_{ba}$ = `accel_bias_instability_mps2`, $\tau_a$ = `accel_correlation_time_s`,
and analogously for gyro. The discrete-time driving noise is derived from the steady-state
variance relation $\sigma_{bias}^2 = \sigma_{drive}^2 \cdot \tau/2$.

IMU noise is generated from the internal RNG. Twelve draws per `step()` call (3 axes ×
2 sensors × bias-drive + measurement noise); advance count increments by 12.

---

## Initialization and Alignment

### States and Modes

On `initialize(config)`, the INS enters `InsMode::Aligning`. Navigation state is zeroed.
The EKF is not active during alignment.

On `reset()`, the INS returns to `InsMode::Aligning` and clears all navigation state.

### Static Alignment

Static alignment uses the first `N_align = ceil(alignment_time_s / dt_s)` steps. During
alignment, the INS accumulates mean accelerometer and magnetometer readings:

**Roll and pitch** from the mean specific force vector $\bar{\mathbf{f}}$ (gravity sensing):

$$
\hat{\phi} = \text{atan2}(\bar{f}_y,\; \bar{f}_z)
$$

$$
\hat{\theta} = \text{atan2}\!\left(-\bar{f}_x,\; \sqrt{\bar{f}_y^2 + \bar{f}_z^2}\right)
$$

(body axes: $x$ forward, $y$ right, $z$ down; $\bar{f}_z \approx -g_0$ when level.)

**Heading** from the mean magnetometer body field $\bar{\mathbf{B}}^B$, tilt-compensated
using estimated roll and pitch:

$$
B_{h,x} = \bar{B}_x \cos\hat{\theta}
         + \bar{B}_y \sin\hat{\phi}\sin\hat{\theta}
         + \bar{B}_z \cos\hat{\phi}\sin\hat{\theta}
$$

$$
B_{h,y} = \bar{B}_y \cos\hat{\phi} - \bar{B}_z \sin\hat{\phi}
$$

$$
\hat{\psi}_{mag} = \text{atan2}(-B_{h,y},\; B_{h,x})
$$

$$
\hat{\psi} = \hat{\psi}_{mag} + \delta_{mag}
$$

where $\delta_{mag}$ = `mag_declination_rad`.

### GNSS Velocity Heading Alignment

If a valid GNSS measurement is provided during alignment with
`speed_over_ground_mps ≥ min_gnss_sog_for_heading_mps`, the INS uses the GNSS course
over ground as the initial heading instead of the magnetometer:

$$
\hat{\psi} = \text{course\_over\_ground\_rad (GNSS)}
$$

This enables faster, more accurate alignment during a rolling takeoff or immediately after
motion begins.

### Alignment Completion

Alignment is declared complete when either:

1. Static: `N_align` steps have accumulated and a mag measurement was available, or
2. GNSS velocity heading: roll and pitch are estimated and GNSS SOG exceeds threshold.

On completion, the INS:

- Builds initial quaternion $\hat{q}_0$ from $(\hat{\phi},\, \hat{\theta},\, \hat{\psi})$.
- Sets initial NED position from first GNSS fix (if available), else zeros with large uncertainty.
- Sets initial velocity to GNSS NED velocity (if available), else zero.
- Sets initial biases to zero.
- Sets $P_0 = \text{diag}(P_{pos},\, P_{vel},\, P_{att},\, P_{ba},\, P_{bg},\, P_{wind})$
  from the `initial_*_uncertainty` config values squared.
- Enters `InsMode::FullyAided` (GNSS available) or `InsMode::DeadReckoning` (GNSS absent).

---

## Propagation

### Mechanization (Nominal State)

Each `step()`, the nominal navigation state is propagated with the corrupted IMU:

$$
\dot{\hat{\mathbf{v}}}^N = R_{N \leftarrow B}\,\mathbf{f}^B_{meas} + \mathbf{g}^N, \qquad
\mathbf{g}^N = [0,\; 0,\; g_0]^T
$$

$$
\dot{\hat{\mathbf{p}}} = \hat{\mathbf{v}}^N \quad \text{(NED position in meters)}
$$

$$
\dot{\hat{q}} = \tfrac{1}{2}\,\hat{q} \otimes \begin{bmatrix}0 \\ \boldsymbol{\omega}^B_{meas}\end{bmatrix}
$$

Discrete integration uses the Euler method (first-order hold) at step `dt_s`.

Position in NED meters is maintained internally. Latitude, longitude, and altitude are
derived at output time using the WGS84 principal radii $R_N$ and $R_E$ at the reference
position (updated each GNSS fix).

### Error-State Vector

The 18-element error-state vector $\delta\mathbf{x}$ follows the EKF. Indices:

| Index | Error state | Units |
| --- | --- | --- |
| 0–2 | $\delta\mathbf{p}^N$ (N, E, D position error) | m |
| 3–5 | $\delta\mathbf{v}^N$ (N, E, D velocity error) | m/s |
| 6–8 | $\delta\boldsymbol{\psi}$ (attitude error angles) | rad |
| 9–11 | $\delta\mathbf{b}_a$ (accelerometer bias error) | m/s² |
| 12–14 | $\delta\mathbf{b}_g$ (gyroscope bias error) | rad/s |
| 15–17 | $\delta\mathbf{w}^N$ (wind velocity error, NED) | m/s |

### Linearized Error-State Dynamics (F Matrix)

The continuous-time F matrix (18×18) has the following non-zero blocks:

$$
F = \begin{bmatrix}
0        & I_3              & 0                               & 0        & 0        & 0   \\
0        & 0                & -R_{N \leftarrow B}[\mathbf{f}^B]_\times & R_{N \leftarrow B} & 0        & 0   \\
0        & 0                & -[\boldsymbol{\omega}^B]_\times  & 0        & -I_3     & 0   \\
0        & 0                & 0                               & -\beta_a I & 0      & 0   \\
0        & 0                & 0                               & 0        & -\beta_g I & 0 \\
0        & 0                & 0                               & 0        & 0        & 0
\end{bmatrix}
$$

where $[\cdot]_\times$ denotes the skew-symmetric (cross-product) matrix, $\beta_a = 1/\tau_a$,
$\beta_g = 1/\tau_g$.

The discrete-time state transition matrix is $\Phi \approx I + F\,dt$.

### Process Noise (Q Matrix)

Continuous-time process noise $Q_c$ (diagonal, 18×18):

| Block | Value |
| --- | --- |
| $\delta\mathbf{p}$ | 0 (driven only by $\delta\mathbf{v}$) |
| $\delta\mathbf{v}$ | $\sigma_a^2 \cdot I_3$ (accel measurement noise mapped through $R_{N \leftarrow B}$; approximated as isotropic) |
| $\delta\boldsymbol{\psi}$ | $\sigma_g^2 \cdot I_3$ (gyro measurement noise) |
| $\delta\mathbf{b}_a$ | $2\beta_a\sigma_{ba}^2 \cdot I_3$ |
| $\delta\mathbf{b}_g$ | $2\beta_g\sigma_{bg}^2 \cdot I_3$ |
| $\delta\mathbf{w}$ | $\sigma_w^2 \cdot I_3$ where $\sigma_w$ = `wind_process_noise_mps_per_sqrthz` |

Discrete-time: $Q_d \approx Q_c \cdot dt$.

Covariance propagation: $P \leftarrow \Phi P \Phi^T + Q_d$.

---

## EKF Measurement Models

All measurement updates follow the standard EKF form:

$$
K = P H^T (H P H^T + R)^{-1}, \quad
\delta\hat{\mathbf{x}} \mathrel{+}= K(\mathbf{z} - H\,\delta\hat{\mathbf{x}}), \quad
P \leftarrow (I - KH)P
$$

The correction $\delta\hat{\mathbf{x}}$ is applied to the nominal state and then reset to
zero (additive error-state EKF convention). The attitude correction $\delta\boldsymbol{\psi}$
is applied to the quaternion as a small rotation: $\hat{q} \leftarrow \hat{q} \otimes [1,\; \delta\boldsymbol{\psi}/2]^T$ (normalized after application).

### GNSS Position Update

Applied when `gnss != nullptr` and `gnss->fix_type == GnssFixType::Fix3D`.

Innovation (in NED meters relative to INS reference position):

$$
\mathbf{z}_{pos} = [\delta N_{GNSS},\; \delta E_{GNSS},\; \delta D_{GNSS}]^T
                 - [\hat{p}_N,\; \hat{p}_E,\; \hat{p}_D]^T
$$

where GNSS lat/lon are converted to NED meters using $R_N(\hat{\phi})$ and $R_E(\hat{\phi})$
at the current INS estimated latitude.

$$
H_{pos} = \begin{bmatrix} I_3 & 0 & 0 & 0 & 0 & 0 \end{bmatrix} \quad (3 \times 18)
$$

$$
R_{pos} = \text{diag}\!\left(\sigma_{hn}^2,\; \sigma_{he}^2,\; \sigma_v^2\right)
$$

using GNSS horizontal and vertical noise from config (or from `gnss->pdop_nd`-derived
values if config is zero).

### GNSS Velocity Update

Applied simultaneously with the position update when GNSS is valid.

$$
\mathbf{z}_{vel} = \mathbf{v}_{NED,GNSS} - \hat{\mathbf{v}}^N
$$

$$
H_{vel} = \begin{bmatrix} 0 & I_3 & 0 & 0 & 0 & 0 \end{bmatrix} \quad (3 \times 18)
$$

$$
R_{vel} = \sigma_{vel}^2 \cdot I_3
$$

### Barometric Altitude Update

Applied when `air_data != nullptr`. Uses `air_data->baro_altitude_m`.

In NED, altitude $h = -p_D$ (down is positive). Innovation:

$$
z_{baro} = h_{baro} - \hat{h}_{INS} = h_{baro} + \hat{p}_D
$$

$$
H_{baro} = \begin{bmatrix} 0 & 0 & -1 & 0 & \cdots & 0 \end{bmatrix} \quad (1 \times 18)
$$

$$
R_{baro} = \sigma_{baro}^2
$$

The INS `altitude_baro_m` output is $-\hat{p}_D$ after this EKF correction is applied;
it is therefore inertially smoothed baro altitude, not raw baro altitude.

### True Airspeed Update

Applied when `air_data != nullptr`. Uses `air_data->tas_mps`.

The estimated TAS is the magnitude of the airspeed vector in NED:

$$
\hat{\mathbf{V}}_a^N = \hat{\mathbf{v}}^N - \hat{\mathbf{w}}^N
$$

$$
\widehat{TAS} = \|\hat{\mathbf{V}}_a^N\|
$$

Innovation:

$$
z_{TAS} = TAS_{meas} - \widehat{TAS}
$$

Jacobian (linearized around current estimate):

$$
H_{TAS} = \begin{bmatrix} 0 & \dfrac{\hat{\mathbf{V}}_a^{N\,T}}{\widehat{TAS}} & 0 & 0 & 0 & -\dfrac{\hat{\mathbf{V}}_a^{N\,T}}{\widehat{TAS}} \end{bmatrix} \quad (1 \times 18)
$$

This update couples the velocity and wind error states: with GNSS velocity available,
$\delta\mathbf{v}$ is well-observed and the airspeed update drives convergence of
$\delta\mathbf{w}$ (wind estimation). In dead reckoning, it provides a direct velocity
magnitude constraint.

Update is skipped when $\widehat{TAS} < 1.0\,\text{m/s}$ to avoid division by zero.

$$
R_{TAS} = \sigma_{TAS}^2
$$

### Magnetometer Update

Applied when `mag != nullptr`. Uses `mag->field_body_nt`.

The predicted body-frame field from the INS reference and current attitude:

$$
\hat{\mathbf{B}}^B = R_{B \leftarrow N}(\hat{q})\;\mathbf{B}_{ref}^N
$$

Innovation (3-vector):

$$
\mathbf{z}_{mag} = \mathbf{B}_{meas}^B - \hat{\mathbf{B}}^B
$$

Jacobian with respect to attitude error $\delta\boldsymbol{\psi}$:

$$
H_{mag} = \begin{bmatrix} 0 & 0 & [\hat{\mathbf{B}}^B]_\times & 0 & 0 & 0 \end{bmatrix} \quad (3 \times 18)
$$

Full expression: $\partial \hat{B}^B / \partial \delta\psi = -[\hat{B}^B]_\times$ (from
the small-angle rotation identity $R(\delta\psi) \approx I + [\delta\psi]_\times$).

$$
R_{mag} = \sigma_{mag}^2 \cdot I_3
$$

where $\sigma_{mag}$ is taken from `mag_noise_nt` in config if nonzero, otherwise from
`SensorMag`'s own noise config.

---

## Kollsman Pressure Update

When GNSS position is valid, the INS back-computes the sea-level pressure that would
cause the barometric altimeter to read the GNSS altitude. Using the ISA tropopause
boundary formula (see `docs/algorithms/air_data.md §INS Aiding`):

$$
P_{Koll} = P_s^{meas} \cdot \left(\frac{T_0}{T_0 + L\,h_{geo}^{GNSS}}\right)^{-R_d L / g_0}
$$

where $P_s^{meas}$ is the current static pressure from `air_data->baro_altitude_m` (inverted
through the ISA formula), $h_{geo}^{GNSS}$ = GNSS altitude, $T_0 = 288.15\,K$,
$L = 0.0065\,K/m$, $R_d = 287.058\,J/(kg\cdot K)$, $g_0 = 9.80665\,m/s^2$.

The result is output as `InsMeasurement::kollsman_pa`. The caller may pass this to
`SensorAirData::setKollsman()` to correct the altimeter for actual surface pressure.
`kollsman_pa` is set to 0 when GNSS is unavailable.

---

## Operating Modes

### Mode Transitions

```text
Uninitialized ──initialize()──► Aligning ──alignment complete──► FullyAided
                                                                      │    ▲
                                                               GNSS lost│    │GNSS regained
                                                                        ▼    │
                                                                  DeadReckoning
```

| Mode | GNSS updates | Baro updates | TAS updates | Mag updates | Position drift |
| --- | --- | --- | --- | --- | --- |
| Aligning | no | no | no | no | — |
| FullyAided | yes | yes | yes | yes | bounded |
| DeadReckoning | no | yes | yes | yes | grows with time |

### Dead Reckoning Behavior

In `DeadReckoning`:

- GNSS position and velocity updates are suspended.
- Baro altitude, TAS, and magnetometer updates continue.
- The TAS update provides a velocity magnitude constraint; combined with magnetometer
  heading, horizontal velocity is partially observable.
- The wind estimate covariance grows (driven by process noise) because the velocity
  decomposition into airspeed + wind requires GNSS to resolve the ambiguity independently.
- Position error covariance grows over time; the covariance diagonal reflects this.

GNSS is considered regained when `gnss != nullptr` and
`gnss->fix_type == GnssFixType::Fix3D` on three consecutive steps. On regain, the INS
transitions back to `FullyAided` and applies a large-innovation GNSS position update
(without clamping) to re-anchor the position estimate.

---

## Class Interface

```cpp
// include/estimation/NavigationFilter.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <nlohmann/json.hpp>
#include "DynamicElement.hpp"
#include "sensor/SensorGnss.hpp"
#include "sensor/SensorAirData.hpp"
#include "sensor/SensorMag.hpp"

namespace liteaerosim::estimation {

class NavigationFilter : public liteaerosim::DynamicElement {
public:
    InsMeasurement step(
        const Eigen::Vector3f&    true_specific_force_body_mps2,
        const Eigen::Vector3f&    true_angular_rate_body_rads,
        const GnssMeasurement*    gnss,
        const AirDataMeasurement* air_data,
        const MagMeasurement*     mag);

    void serializeProto(liteaerosim::InsStateProto& proto) const;
    void deserializeProto(const liteaerosim::InsStateProto& proto);

protected:
    void onInitialize(const nlohmann::json& config) override;
    void onReset() override;
    nlohmann::json onSerializeJson() const override;
    void onDeserializeJson(const nlohmann::json& state) override;

private:
    struct RngState;
    InsConfig   config_;
    InsMode     mode_            = InsMode::Uninitialized;
    bool        alignment_complete_ = false;

    // Nominal navigation state
    double            ref_lat_rad_   = 0.0;  // reference latitude for NED origin
    double            ref_lon_rad_   = 0.0;  // reference longitude for NED origin
    Eigen::Vector3d   pos_ned_m_     = Eigen::Vector3d::Zero();  // NED position from reference (m)
    Eigen::Vector3f   vel_ned_mps_   = Eigen::Vector3f::Zero();
    Eigen::Quaternionf attitude_q_   = Eigen::Quaternionf::Identity();
    Eigen::Vector3f   accel_bias_    = Eigen::Vector3f::Zero();   // body frame (m/s²)
    Eigen::Vector3f   gyro_bias_     = Eigen::Vector3f::Zero();   // body frame (rad/s)
    Eigen::Vector3f   wind_ned_      = Eigen::Vector3f::Zero();   // NED (m/s)

    // EKF covariance (18×18, symmetric)
    Eigen::Matrix<float, 18, 18> P_;

    // Alignment accumulators
    int             align_count_       = 0;
    Eigen::Vector3f align_accel_sum_   = Eigen::Vector3f::Zero();
    Eigen::Vector3f align_mag_sum_     = Eigen::Vector3f::Zero();

    // GNSS regain counter
    int gnss_regain_count_ = 0;

    std::unique_ptr<RngState> rng_;

    // Internal helpers
    void propagate(const Eigen::Vector3f& f_meas, const Eigen::Vector3f& omega_meas);
    void updateGnss(const GnssMeasurement& gnss);
    void updateBaro(const AirDataMeasurement& air_data);
    void updateTas(const AirDataMeasurement& air_data);
    void updateMag(const MagMeasurement& mag);
    void applyCorrection(const Eigen::Matrix<float, 18, 1>& dx);
    InsMeasurement buildOutput() const;
};

} // namespace liteaerosim::estimation
```

---

## Serialization Contract

### JSON State Fields

| JSON key | Description |
| --- | --- |
| `"schema_version"` | Integer; must equal 1. Throws on mismatch. |
| `"mode"` | `InsMode` as integer. |
| `"alignment_complete"` | Boolean. |
| `"align_count"` | Alignment step counter. |
| `"align_accel_sum"` | 3-element array (m/s²). |
| `"align_mag_sum"` | 3-element array (nT). |
| `"ref_lat_rad"` | NED reference latitude (double). |
| `"ref_lon_rad"` | NED reference longitude (double). |
| `"pos_ned_m"` | 3-element array (m); NED position from reference. |
| `"vel_ned_mps"` | 3-element array (m/s). |
| `"attitude_q"` | 4-element array [qw, qx, qy, qz]. |
| `"accel_bias_mps2"` | 3-element array (m/s²). |
| `"gyro_bias_rads"` | 3-element array (rad/s). |
| `"wind_ned_mps"` | 3-element array (m/s). |
| `"covariance"` | 171-element array; upper triangle of the 18×18 symmetric P matrix, row-major. |
| `"gnss_regain_count"` | Integer (0–3). |
| `"rng_seed"` | RNG seed actually used. |
| `"rng_advance"` | Number of variate draws since seeding. |

Schema version: **1**.

---

## Proto Messages

```proto
// proto/liteaerosim.proto

message InsStateProto {
    int32  schema_version    = 1;
    int32  mode              = 2;    // InsMode cast to int32
    bool   alignment_complete = 3;
    int32  align_count       = 4;

    // Alignment accumulators
    float  align_accel_sum_x = 5;
    float  align_accel_sum_y = 6;
    float  align_accel_sum_z = 7;
    float  align_mag_sum_x   = 8;
    float  align_mag_sum_y   = 9;
    float  align_mag_sum_z   = 10;

    // Navigation state
    double ref_lat_rad       = 11;
    double ref_lon_rad       = 12;
    float  pos_ned_n_m       = 13;
    float  pos_ned_e_m       = 14;
    double pos_ned_d_m       = 15;   // double for altitude precision
    float  vel_ned_n_mps     = 16;
    float  vel_ned_e_mps     = 17;
    float  vel_ned_d_mps     = 18;
    float  att_qw            = 19;
    float  att_qx            = 20;
    float  att_qy            = 21;
    float  att_qz            = 22;
    float  accel_bias_x_mps2 = 23;
    float  accel_bias_y_mps2 = 24;
    float  accel_bias_z_mps2 = 25;
    float  gyro_bias_x_rads  = 26;
    float  gyro_bias_y_rads  = 27;
    float  gyro_bias_z_rads  = 28;
    float  wind_ned_n_mps    = 29;
    float  wind_ned_e_mps    = 30;
    float  wind_ned_d_mps    = 31;

    // EKF covariance — upper triangle, 171 elements
    repeated float covariance_upper = 32;

    int32  gnss_regain_count = 33;
    uint32 rng_seed          = 34;
    uint64 rng_advance       = 35;
}
```

---

## Computational Cost

### Memory Footprint

| Component | Size |
| --- | --- |
| Navigation state (lat/lon doubles, pos\_ned doubles, vel, quaternion, biases, wind) | ~104 bytes |
| EKF covariance matrix (18×18 `float`) | 1 296 bytes |
| Alignment accumulators (accel sum, mag sum, count) | ~28 bytes |
| `InsConfig` | ~120 bytes |
| `RngState` pimpl (`std::mt19937` engine) | ~2.5 KB |
| **Total active state (excl. RNG)** | **~1.6 KB** |

The entire working set fits within a typical 32 KB L1 data cache, keeping memory-bound
operations fast.

### Operations per `step()` Call

**Mandatory (every step):**

| Sub-task | Approximate FLOPs | Notes |
| --- | --- | --- |
| IMU error model (Gauss-Markov bias step) | ~20 | 6 multiply-add pairs |
| 12 Gaussian noise draws (accel + gyro, measurement + bias drive) | — | ~60–180 ns |
| Quaternion kinematic update + normalization | ~40 | $\dot{q} = \tfrac{1}{2}q \otimes \omega$, Euler step |
| Rotation matrix from quaternion | ~27 | For mechanization |
| Velocity integration ($R \mathbf{f} + \mathbf{g}$) | ~15 | |
| Position integration | 3 | |
| Covariance propagation $P \leftarrow \Phi P \Phi^T + Q$ | **~2 000** | Dominant; see note below |
| **Mandatory total (non-noise FLOPs)** | **~2 100** | |

> **Covariance propagation note:** The naive full 18³ computation costs ~11 664 FLOPs.
> Exploiting the sparsity of Φ (only five non-trivial off-diagonal block rows, each 3×18)
> reduces this to approximately 2 000 FLOPs. A naive dense implementation will be ~6×
> slower for this sub-task.

**Conditional (measurement updates):**

| Update | Observations | Approximate FLOPs | Frequency |
| --- | --- | --- | --- |
| GNSS position + velocity | 6 | ~1 200 | At GNSS update rate (e.g., 10 Hz) |
| Baro altitude (rank-1) | 1 | ~200 | Every step with valid air data |
| TAS (rank-1, with Jacobian) | 1 | ~220 | Every step with valid air data |
| Magnetometer (rank-3) | 3 | ~450 | Every step with valid mag |
| **All updates active** | — | **~2 100** | |

### Execution Budget at 100 Hz

| Scenario | FLOPs/step | Wall-clock estimate | % of 10 ms budget |
| --- | --- | --- | --- |
| No aiding (propagation only) | ~2 100 | ~2–5 μs | < 0.05% |
| Fully aided (all measurements) | ~4 200 | ~5–10 μs | < 0.1% |

Estimates assume ~500 MFLOPS scalar single-core throughput (conservative for a modern
application processor such as a Raspberry Pi CM4 or similar companion computer).
12 noise draws add ~60–180 ns independently.

### Scaling Considerations

- **State count:** covariance propagation and update scale as $O(N^2)$ in the number of
  states $N$. Extending from 18 to 21 states (e.g., adding barometric bias) increases
  covariance cost by $(21/18)^2 \approx 36\%$.
- **Observation count:** each rank-$m$ update adds $O(N^2 + m \cdot N)$ FLOPs for the
  covariance correction. Scalar updates (baro, TAS) are cheapest; 6-obs GNSS is the most
  expensive single update.
- **No terrain or scene complexity dependency.** All costs are determined solely by the
  state and observation dimensions.

### Comparison with Aircraft Dynamics

`Aircraft::step()` is intentionally lightweight — it is a trim-aero point-mass model, not
a 6-DOF moment integrator. Its sub-tasks and approximate costs are:

| `Aircraft::step()` sub-task | Approximate FLOPs | Notes |
| --- | --- | --- |
| Airspeed magnitude + dynamic pressure | ~10 | 3-vector subtract, norm, multiply |
| Envelope clamping | ~4 | 2 clamp operations |
| `LoadFactorAllocator::solve()` | ~55 | Newton iteration (2 steps, warm-started from previous α, β) |
| `LiftCurveModel::evaluate()` | ~5 | Piecewise-linear lookup |
| `AeroPerformance::compute()` | ~15 | CL, CD (polar), side-force coefficients → forces |
| `V_Propulsion::step()` (jet/EDF) | ~25 | Lag filter + atmosphere correction |
| Thrust → Wind-frame acceleration | ~35 | 4 trig + vector arithmetic |
| `KinematicState::step()` | ~90 | Quaternion kinematics, NED velocity/position integration, Euler angles |
| **Aircraft total** | **~240** | No RNG draws |

Side-by-side comparison at 100 Hz:

| Component | FLOPs/step | RNG draws/step | Wall-clock estimate | % of 10 ms budget |
| --- | --- | --- | --- | --- |
| `Aircraft::step()` | ~240 | 0 | ~0.5 μs | ~0.005% |
| `NavigationFilter::step()` — propagation only | ~2 100 | 12 | ~3–6 μs | ~0.03–0.06% |
| `NavigationFilter::step()` — fully aided | ~4 200 | 12 | ~5–10 μs | ~0.05–0.10% |
| **INS / Aircraft ratio** | **~9–18×** | | | |

**Conclusion:** `NavigationFilter` is 9–18× more expensive in raw FLOPs than `Aircraft::step()`,
but both are well within the execution budget of a modern companion computer at 100 Hz.
The INS consumes at most ~10 μs out of a 10 ms step period — under 0.1% of a single core.
No performance mitigation is required for a single-aircraft deployment.

The cost disparity is explained by a single sub-task: covariance propagation
($P \leftarrow \Phi P \Phi^T + Q$) accounts for ~2 000 of the INS's ~2 100 mandatory
FLOPs — roughly **8× the entire `Aircraft::step()` cost on its own**. This is the
unavoidable price of maintaining a consistent uncertainty estimate across 18 states.

Should a tightly resource-constrained flight computer (bare-metal MCU, sub-100 MHz)
require reduced cost, the following mitigations apply:

| Mitigation | Approximate saving | Trade-off |
| --- | --- | --- |
| Exploit Φ sparsity (already assumed above) | 6× over naive dense propagation | Implementation complexity |
| Reduced-rate covariance propagation (e.g., 10 Hz P, 100 Hz mechanics) | ~10× on propagation | Covariance lags state; acceptable for most applications |
| Frozen P during dead reckoning after convergence | Eliminates propagation cost when not needed | Covariance no longer reflects growing uncertainty |

---

## ArduPilot Integration

`NavigationFilter` runs on a companion computer alongside the ArduPilot autopilot and
delivers navigation state via MAVLink, bypassing ArduPilot's internal EKF entirely. The
same code path is used in HIL simulation with no modification.

### ArduPilot Configuration

Configure ArduPilot to accept external navigation data:

| Parameter | Value | Meaning |
| --- | --- | --- |
| `EK3_ENABLE` | `1` | Enable EKF3 |
| `EK3_SRC1_POSXY` | `6` | Horizontal position from external nav |
| `EK3_SRC1_VELXY` | `6` | Horizontal velocity from external nav |
| `EK3_SRC1_POSZ` | `6` | Vertical position from external nav |
| `EK3_SRC1_VELZ` | `6` | Vertical velocity from external nav |
| `EK3_SRC1_YAW` | `6` | Heading from external nav |
| `AHRS_EKF_TYPE` | `3` | Use EKF3 |
| `GPS_TYPE` | `0` | Disable internal GPS (INS provides position) |

With this configuration ArduPilot's EKF3 fuses only the external navigation source;
its internal IMU integration remains active but the external position/velocity/attitude
observations dominate.

### MAVLink Message Mapping

Two MAVLink 2 messages carry the `InsMeasurement` fields to ArduPilot. Both are sent
every `NavigationFilter` `step()` call (i.e., at the INS update rate).

#### `ODOMETRY` (message ID 331)

Primary message. Carries full pose, twist, and covariance at INS rate.

| MAVLink field | Source in `InsMeasurement` | Notes |
| --- | --- | --- |
| `time_usec` | system time at time of validity (µs) | Wall-clock or simulated time |
| `frame_id` | `MAV_FRAME_LOCAL_NED` = 8 | Position reference frame |
| `child_frame_id` | `MAV_FRAME_BODY_FRD` = 12 | Velocity reference frame |
| `x` | `cos(lat)·(lon−lon₀)·Rₑ` (m, East) | NED→ENU: x=E |
| `y` | `(lat−lat₀)·Rₙ` (m, North) | NED→ENU: y=N |
| `z` | `−altitude_baro_m` (m, Up) | NED→ENU: z=−D |
| `vx`, `vy`, `vz` | `velocity_ned_mps` reordered to ENU | |
| `q[4]` | `attitude_q` converted NED→ENU | Quaternion reorder: ArduPilot uses ENU body convention |
| `roll_rate`, `pitch_rate`, `yaw_rate` | Not populated (0) | INS does not estimate angular rate |
| `pose_covariance[0,7,14]` | `covariance_diagonal[0,1,2]` (position) | Diagonal only; off-diagonal set to 0 |
| `velocity_covariance[0,7,14]` | `covariance_diagonal[3,4,5]` (velocity) | |

#### `ATT_POS_MOCAP` (message ID 138) — fallback for older firmware

Used when the connected ArduPilot firmware predates `ODOMETRY` support (prior to 4.1).

| MAVLink field | Source |
| --- | --- |
| `time_usec` | system time at time of validity (µs) |
| `q[4]` | `attitude_q` converted to ENU |
| `x`, `y`, `z` | NED position converted to ENU (m) |
| `covariance` | `covariance_diagonal[0]` (horizontal 1σ²) |

#### `GPS_INPUT` (message ID 232) — GNSS pass-through

Sent when `InsMeasurement::mode == InsMode::FullyAided` and `kollsman_pa != 0`. Allows
ArduPilot's own EKF to cross-check the external nav with a synthetic GPS. Configure
`GPS_TYPE2 = 14` (MAVLink GPS) to receive this on the secondary GPS port.

| MAVLink field | Source |
| --- | --- |
| `lat`, `lon` | `latitude_rad`, `longitude_rad` (×10⁷ deg) |
| `alt` | `altitude_baro_m` × 1000 (mm) |
| `vn`, `ve`, `vd` | `velocity_ned_mps` (m/s) |
| `horiz_accuracy` | `covariance_diagonal[0]` (1σ, m) |
| `vert_accuracy` | `covariance_diagonal[2]` (1σ, m) |
| `speed_accuracy` | `covariance_diagonal[3]` (1σ, m/s) |
| `fix_type` | `3` (3D fix) when mode is `FullyAided`; `0` otherwise |
| `satellites_visible` | `12` (fixed; not tracked by INS) |

### Rate and Latency Requirements

| Requirement | Value | Rationale |
| --- | --- | --- |
| Minimum update rate | 25 Hz | ArduPilot EKF3 external nav timeout is 200 ms |
| Recommended update rate | 50–100 Hz | Matches INS step rate; reduces EKF3 prediction error |
| Maximum one-way latency | 40 ms | ArduPilot timestamps odometry; stale data triggers fallback |
| `ODOMETRY` timestamp | Estimated time of validity (not send time) | ArduPilot applies latency compensation when `EK3_DELAY_MS` is set |

In HIL simulation the MAVLink connection is loopback; latency is negligible. Set
`EK3_DELAY_MS = 0` for HIL.

### Wind Estimate Pass-Through

`InsMeasurement::wind_ned_mps` has no standard MAVLink message. In ArduPilot it can be
injected via `WIND` (message ID 168) for display only, or passed to the GCS. ArduPilot's
flight controller does not consume wind estimates from external sources for feedforward;
wind compensation must be implemented in the guidance layer.

### Dead Reckoning Behavior at the MAVLink Interface

During `InsMode::DeadReckoning`:

- `ODOMETRY` continues to be sent at the configured rate.
- The `pose_covariance` values reflect growing position uncertainty (from
  `covariance_diagonal[0–2]`), signaling to ArduPilot's EKF3 that the position
  measurement is unreliable.
- `GPS_INPUT` is suppressed (`fix_type = 0`).
- ArduPilot will eventually time out the external nav source and fall back to its own
  IMU-only dead reckoning unless `EK3_SRC1_POSXY` failsafes are configured.

---

## Test Requirements

All tests reside in `test/NavigationFilter_test.cpp`, test class `NavigationFilterTest`.

`InsMeasurement`, `InsConfig`, and `InsMode` are defined in
`include/estimation/NavigationFilter.hpp` within `namespace liteaerosim::estimation`.
`SensorInsSimulation` and any other class that uses these types includes from that path.

| ID | Test Name | Description |
| --- | --- | --- |
| T1 | `StaticAlignment_EstimatesRollPitch` | Stationary input `f = [0, 0, -g₀]` (level), mag = NED reference rotated to body, zero noise: after `alignment_time_s`, estimated roll and pitch are within 0.01 rad of true. |
| T2 | `StaticAlignment_EstimatesHeading` | Same as T1 with a 30° yaw offset: estimated yaw is within 1° of true after alignment. |
| T3 | `GnssVelocityAlignment_ConvergesWithMotion` | GNSS SOG > `min_gnss_sog_for_heading_mps`, no static phase: `alignment_complete` becomes true and yaw estimate is within 2° of GNSS COG. |
| T4 | `FullyAided_PositionBounded` | Zero IMU noise, GNSS noise 2 m horizontal: after 100 steps, estimated NED position error < 3× GNSS noise (filter converged). |
| T5 | `BaroUpdate_SmoothesBaro` | Baro noise 5 m; 200 steps: `altitude_baro_m` is smoother than raw baro (sample standard deviation of residual < 2 m). |
| T6 | `TasUpdate_DrivesWindEstimate` | Non-zero true wind `{5, 3, 0}` m/s, GNSS + TAS available, zero IMU noise: after 500 steps, estimated wind within 1 m/s of true. |
| T7 | `MagUpdate_CorrectsDrift` | Yaw error injected via initial bias; magnetometer aiding active: yaw error halved within 100 steps. |
| T8 | `DeadReckoning_PositionDrifts` | GNSS removed after 50 steps; 200 further steps: position error covariance diagonal 0–2 increases monotonically. |
| T9 | `DeadReckoning_AltitudeMaintained` | Baro aiding continues in dead reckoning: `altitude_baro_m` error remains < 10 m for 500 steps with 1 m baro noise. |
| T10 | `GnssRegain_ReanchorsPosition` | GNSS removed for 50 steps (1 m/s IMU drift injected), then restored: position error < 3 m within 10 steps of regain. |
| T11 | `KollsmanOutput_MatchesFormula` | Known GNSS altitude and baro reading: `kollsman_pa` matches the analytical back-computation formula within 1 Pa. |
| T12 | `KollsmanOutput_ZeroInDeadReckoning` | GNSS unavailable: `kollsman_pa == 0.f`. |
| T13 | `Reset_ReturnsToAligning` | After 200 steps, `reset()`: `mode == InsMode::Aligning` and `alignment_complete == false`. |
| T14 | `IdenticalSeeds_IdenticalOutputs` | Two instances with the same nonzero seed, same inputs: every field of `InsMeasurement` is bitwise-identical for N = 100 steps. |
| T15 | `JsonRoundTrip_PreservesFullState` | Serialize after 100 steps; deserialize into a new instance; next `step()` output is identical for all fields. |
| T16 | `ProtoRoundTrip_PreservesFullState` | Same as T15 using `serializeProto()` / `deserializeProto()`. |
| T17 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `schema_version != 1` throws `std::runtime_error`. |
| T18 | `CovarianceDiagonal_AllPositive` | After 10 steps in FullyAided mode: all 18 diagonal elements of P are positive. |
| T19 | `ZeroImuNoise_ZeroAidingNoise_ConvergesToTruth` | Zero IMU noise, zero GNSS noise, zero baro noise: after 200 steps, position error < 0.01 m, velocity error < 0.001 m/s, attitude error < 0.001 rad. |
