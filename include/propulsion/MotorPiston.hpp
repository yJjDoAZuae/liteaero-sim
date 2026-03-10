#pragma once

#include "propulsion/V_Motor.hpp"

namespace liteaerosim::propulsion {

// Normally aspirated piston engine model.  Stateless — no serialization required.
//
// Available power:     P_avail(δ_T, ρ) = P_max · δ_T · (ρ/ρ_SL)^n_alt
// No-load shaft speed: Ω₀ = 2 · Ω_peak · δ_T · (ρ/ρ_SL)^(n_alt/2)
// Maximum shaft speed: Ω_max = 2 · Ω_peak
class MotorPiston : public V_Motor {
public:
    // power_max_w        — maximum shaft power at sea level (W)
    // peak_omega_rps     — shaft speed at maximum power (rad/s)
    // altitude_exponent  — density exponent for power lapse (≈1.0 normally aspirated)
    // inertia_kg_m2      — crankshaft + flywheel + propeller inertia (kg·m²)
    MotorPiston(float power_max_w,
                float peak_omega_rps,
                float altitude_exponent,
                float inertia_kg_m2);

    // V_Motor interface
    [[nodiscard]] float noLoadOmega_rps(float throttle_nd, float rho_kgm3) const override;
    [[nodiscard]] float maxOmega_rps()  const override;
    [[nodiscard]] float inertia_kg_m2() const override;

private:
    float _power_max_w;
    float _peak_omega_rps;
    float _altitude_exponent;
    float _inertia_kg_m2;
};

} // namespace liteaerosim::propulsion
