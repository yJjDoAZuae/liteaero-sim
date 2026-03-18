#pragma once

#include "propulsion/Motor.hpp"

namespace liteaerosim::propulsion {

// Motor electromechanics parameters.
struct MotorElectricMotorParams {
    float kv_rps_per_v;         // Back-EMF / velocity constant (rad/s/V)
    float r_terminal_ohm;       // Phase winding resistance (Ω) — used only in batteryCurrent_a()
    float inertia_motor_kg_m2;  // Rotor moment of inertia, excluding propeller (kg·m²)
};

// ESC / drive electronics parameters.
struct MotorElectricEscParams {
    float supply_voltage_v;     // DC battery bus voltage (V)
    float i_max_a;              // ESC peak current rating (A)
    float esc_efficiency_nd;    // DC-to-3-phase power conversion efficiency [0, 1]
};

// BLDC motor and ESC model.  Stateless — no serialization required.
//
// No-load shaft speed:   Ω₀ = KV · δ_T · V_supply
// Maximum shaft speed:   Ω_max = KV · V_supply
// Maximum torque:        Q_max = I_max / KV
// Battery current:       I_bat = V_phase · min(I_motor, I_max) / (V_supply · η_ESC)
//   where  V_phase = δ_T · V_supply  and  I_motor = (V_phase − Ω/KV) / R
class MotorElectric : public Motor {
public:
    MotorElectric(const MotorElectricMotorParams& motor,
                  const MotorElectricEscParams&   esc);

    // Motor interface
    [[nodiscard]] float noLoadOmega_rps(float throttle_nd, float rho_kgm3) const override;
    [[nodiscard]] float maxOmega_rps()  const override;
    [[nodiscard]] float inertia_kg_m2() const override;

    // Battery current at the given operating point (A).
    // For energy / endurance accounting only; not called by PropulsionProp::step().
    [[nodiscard]] float batteryCurrent_a(float omega_rps, float throttle_nd) const;

private:
    float _kv_rps_per_v;
    float _r_terminal_ohm;
    float _inertia_motor_kg_m2;
    float _supply_voltage_v;
    float _i_max_a;
    float _esc_efficiency_nd;
};

} // namespace liteaerosim::propulsion
