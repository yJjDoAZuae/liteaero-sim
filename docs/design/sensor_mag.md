# Magnetometer Sensor — Architecture and Interface Design

This document is the design authority for `SensorMag` within the `liteaerosim::sensor`
namespace. It specifies the triaxial magnetometer sensor class, all data structures, the
serialization contract, proto message definitions, and the full set of required tests that
drive TDD implementation.

`SensorMag` derives from `liteaerosim::DynamicElement`. The lifecycle contract, NVI
pattern, and base class requirements are defined in
[`docs/architecture/dynamic_element.md`](dynamic_element.md). Sensor-specific conventions
(serialization, RNG, naming, test requirements) are in
[`docs/architecture/sensor.md`](sensor.md).

---

## Scope

`SensorMag` models a triaxial magnetometer (fluxgate or MEMS type). It produces a
body-frame magnetic field measurement by applying hard-iron bias, soft-iron distortion,
and per-axis Gaussian noise to the true field rotated into the body frame.

The reference magnetic field in the NED frame is supplied by the caller at each `step()`
call. This decouples `SensorMag` from any particular field model (IGRF, tilted dipole,
or constant configured field); the caller is responsible for evaluating the field at the
current position and altitude. A constant NED reference field is sufficient for local-area
simulation.

`SensorMag` lives in the Domain Layer. It has no I/O, no unit conversions, and no display
logic. Units are nanotesla (nT) throughout.

---

## Use Case Decomposition

| ID | Use Case | Primary Actor | Description |
| --- | --- | --- | --- |
| UC-MAG1 | Advance one timestep | SimulationLoop | Calls `SensorMag::step(B_ned, q)`. Returns `MagMeasurement`. |
| UC-MAG2 | Initialize from JSON config | Scenario / Setup | Calls `DynamicElement::initialize(config)` with hard-iron bias, soft-iron matrix, noise, and seed. |
| UC-MAG3 | Reset between scenario runs | Scenario / Setup | Calls `DynamicElement::reset()` to re-seed the RNG; does not change calibration parameters. |
| UC-MAG4 | Serialize / deserialize | ConfigParser / SessionManager | Checkpoints the RNG state so the noise sequence is reproducible. |

---

## Data Structures

### `MagMeasurement`

```cpp
// include/sensor/SensorMag.hpp
namespace liteaerosim::sensor {

struct MagMeasurement {
    Eigen::Vector3f field_body_nt;  // 3-axis magnetic field in body frame (nT)
};

} // namespace liteaerosim::sensor
```

---

### `MagConfig`

```cpp
// include/sensor/SensorMag.hpp
namespace liteaerosim::sensor {

struct MagConfig {
    // Hard-iron bias: fixed additive offset in body frame (nT).
    // Arises from permanent magnets and DC currents on the airframe.
    Eigen::Vector3f hard_iron_bias_nt  = {0.f, 0.f, 0.f};

    // Soft-iron distortion: 3×3 matrix, stored row-major as a 9-element array.
    // Models scale-factor errors and cross-axis coupling from ferromagnetic materials.
    // Stored as M such that B_distorted = M * B_true_body.
    // Default is identity (no distortion).
    std::array<float, 9> soft_iron_matrix = {
        1.f, 0.f, 0.f,
        0.f, 1.f, 0.f,
        0.f, 0.f, 1.f
    };

    float    noise_nt     = 0.f;   // 1-sigma per-axis isotropic Gaussian noise (nT)
    uint32_t seed         = 0;     // RNG seed; 0 = non-deterministic (std::random_device)
    int      schema_version = 1;
};

} // namespace liteaerosim::sensor
```

---

## Measurement Model

The complete measurement model applied at each `step()` call:

$$
\mathbf{B}_{meas}^B = M \cdot R_{B \leftarrow N} \cdot \mathbf{B}_{ref}^N + \mathbf{b}_{hard} + \mathbf{n}
$$

where:

| Symbol | Meaning |
| --- | --- |
| $M$ | Soft-iron distortion matrix (3×3); stored in `soft_iron_matrix`; identity = no distortion |
| $R_{B \leftarrow N}$ | Rotation matrix from NED to body frame; derived from input quaternion $q_{B \leftarrow N}$ |
| $\mathbf{B}_{ref}^N$ | Reference magnetic field in NED frame (nT); supplied by caller |
| $\mathbf{b}_{hard}$ | Hard-iron bias vector in body frame (nT); from `hard_iron_bias_nt` |
| $\mathbf{n}$ | Per-axis independent Gaussian noise: $n_i \sim \mathcal{N}(0,\,\sigma^2)$; $\sigma$ = `noise_nt` |

The hard-iron bias and soft-iron matrix are fixed calibration parameters; they do not
evolve over time. Three noise draws are made per `step()` call (one per axis). The advance
count increments by 3 per step.

---

## Step Interface

```cpp
MagMeasurement step(const Eigen::Vector3f&    reference_field_ned_nt,
                    const Eigen::Quaternionf& q_body_from_ned);
```

`reference_field_ned_nt` is the local magnetic field in NED frame in nanotesla. A typical
value at mid-latitudes is approximately `{20 000, 2 000, −45 000}` nT (pointing mostly
downward in the Northern Hemisphere). The caller is responsible for supplying a
geographically correct value.

`q_body_from_ned` is the unit quaternion that transforms a vector from NED to body frame
(the same convention used throughout the sensor subsystem). It provides the rotation
$R_{B \leftarrow N}$ used in the model.

---

## Class Interface

```cpp
// include/sensor/SensorMag.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <nlohmann/json.hpp>
#include "DynamicElement.hpp"

namespace liteaerosim::sensor {

class SensorMag : public liteaerosim::DynamicElement {
public:
    MagMeasurement step(const Eigen::Vector3f&    reference_field_ned_nt,
                        const Eigen::Quaternionf& q_body_from_ned);

    void serializeProto(liteaerosim::MagStateProto& proto) const;
    void deserializeProto(const liteaerosim::MagStateProto& proto);

protected:
    void onInitialize(const nlohmann::json& config) override;
    void onReset() override;
    nlohmann::json onSerializeJson() const override;
    void onDeserializeJson(const nlohmann::json& state) override;

private:
    struct RngState;
    MagConfig                 config_;
    Eigen::Matrix3f           soft_iron_;  // soft_iron_matrix decoded into an Eigen matrix at init
    std::unique_ptr<RngState> rng_;
};

} // namespace liteaerosim::sensor
```

The `soft_iron_matrix` from config is decoded once in `onInitialize()` into `soft_iron_`
(an `Eigen::Matrix3f`) for efficient use in `step()`.

The `RngState` pimpl follows the same convention as other sensor classes: the destructor
is defined in the `.cpp` translation unit.

---

## Serialization Contract

### JSON State Fields

| JSON key | C++ member | Description |
| --- | --- | --- |
| `"schema_version"` | — | Integer; must equal 1. Throws `std::runtime_error` on mismatch. |
| `"rng_seed"` | `rng_.seed` | The seed actually used (from `std::random_device` if config seed was 0). |
| `"rng_advance"` | `rng_.advance_count` | Number of variate draws since seeding; 3 per `step()` call. |

Calibration parameters (`hard_iron_bias_nt`, `soft_iron_matrix`, `noise_nt`) are config,
not state, and are not re-serialized.

Schema version: **1**.

---

## Proto Messages

```proto
// proto/liteaerosim.proto

message MagConfig {
    int32 schema_version         = 1;
    float hard_iron_bias_x_nt    = 2;
    float hard_iron_bias_y_nt    = 3;
    float hard_iron_bias_z_nt    = 4;
    repeated float soft_iron_row_major = 5;  // 9 elements; row-major 3×3 matrix
    float noise_nt               = 6;
    uint32 seed                  = 7;
}

message MagStateProto {
    int32  schema_version = 1;
    uint32 rng_seed       = 2;
    uint64 rng_advance    = 3;
}
```

---

## Computational Cost

### Memory Footprint

| Component | Size |
| --- | --- |
| `MagConfig` (9-float matrix + 3-float bias + noise + seed + version) | ~60 bytes |
| Cached `soft_iron_` (`Eigen::Matrix3f`, decoded at init) | 36 bytes |
| `RngState` pimpl (`std::mt19937` engine) | ~2.5 KB |
| **Total active state (excl. RNG)** | ~100 bytes |

### Operations per `step()` Call

| Sub-task | Approximate FLOPs |
| --- | --- |
| Quaternion → 3×3 rotation matrix | ~27 |
| Rotate reference field to body frame (3×3 × vector) | ~15 |
| Soft-iron distortion (3×3 × vector) | ~15 |
| Add hard-iron bias | 3 |
| 3 Gaussian noise draws (one per axis) | dominant (~5–15 ns each) |
| **Total non-noise FLOPs** | **~60** |

### Dominant Cost and Scaling

Gaussian noise generation dominates wall-clock time (~15–45 ns for three draws). The
~60 non-noise FLOPs add another ~0.1 μs at typical scalar float throughput. At a 100 Hz
simulation rate (10 ms step budget) the total cost is under 0.01% of available
single-core budget.

No data-dependent branching or input-size scaling — every step executes the same fixed
sequence.

---

## Test Requirements

All tests reside in `test/SensorMag_test.cpp`, test class `SensorMagTest`.

| ID | Test Name | Description |
| --- | --- | --- |
| T1 | `ZeroDistortion_ZeroNoise_OutputEqualsRotatedField` | Identity soft-iron, zero hard-iron, zero noise: output equals `R(q) * B_NED` to within `float` rounding (< 0.01 nT). |
| T2 | `HardIronBias_AddedInBodyFrame` | Zero noise, identity soft-iron, `b_hard = {100, −50, 200}` nT: output equals rotated field plus bias exactly. |
| T3 | `SoftIronMatrix_AppliedBeforeBias` | Non-identity soft-iron M, zero hard-iron, zero noise: output equals `M * R(q) * B_NED` exactly. |
| T4 | `Noise_SampleStddev_MatchesConfig` | With `noise_nt = 50`, N = 2000 steps with fixed attitude and field: sample std of each axis residual is within 20% of 50 nT. |
| T5 | `IdenticalSeeds_IdenticalOutputs` | Two instances with the same nonzero seed, same config, same inputs: bitwise-identical output for N = 100 steps. |
| T6 | `Reset_ReseedsRng_SameSequence` | After N steps, `reset()` re-seeds the RNG; the same input sequence reproduces identical output as a freshly initialized instance. |
| T7 | `JsonRoundTrip_PreservesNoiseSequence` | Serialize after N = 50 steps; deserialize into a new instance; the next `step()` output is identical between original and restored instances. |
| T8 | `ProtoRoundTrip_PreservesNoiseSequence` | Same as T7 using `serializeProto()` / `deserializeProto()`. |
| T9 | `SchemaVersionMismatch_Throws` | `deserializeJson()` with `schema_version != 1` throws `std::runtime_error`. |
