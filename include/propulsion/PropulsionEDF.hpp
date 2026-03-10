#pragma once

#include "propulsion/V_Propulsion.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <vector>

namespace liteaerosim::propulsion {

struct PropulsionEdfParams {
    float thrust_sl_n;          // Full-throttle static thrust at sea level (N)
    float fan_diameter_m;       // Fan disk diameter (m) — for actuator-disk power estimate
    float inlet_area_m2;        // Effective duct inlet capture area for ram drag (m²)
    float idle_fraction;        // Idle thrust as fraction of thrust_sl_n (—)
    float rotor_tau_s;          // First-order lag: throttle → thrust (s)
    float supply_voltage_v;     // DC battery bus voltage (V)
    float fan_efficiency_nd;    // Fan + duct aerodynamic efficiency (0.65–0.85)
    float esc_efficiency_nd;    // ESC DC-to-3-phase power conversion efficiency (0.90–0.98)
};

// Physics-based electric ducted fan model.
//
// Thrust lapse:  T_gross(δ_T, ρ) = δ_T · T_sl · (ρ/ρ_SL)   (density exponent fixed at 1)
// Ram drag:      F_ram = ρ · V² · A_inlet
// Rotor lag:     first-order IIR via FilterSS2Clip
// Battery current from actuator disk momentum theory (for endurance analysis).
class PropulsionEDF : public V_Propulsion {
public:
    PropulsionEDF(const PropulsionEdfParams& params, float dt_s);

    // V_Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;
    void reset() override;

    // Battery current at current thrust and flight condition (A).
    // For energy / endurance analysis only; not called by step().
    [[nodiscard]] float batteryCurrent_a(float tas_mps, float rho_kgm3) const;

    [[nodiscard]] nlohmann::json        serializeJson()                               const override;
    void                                deserializeJson(const nlohmann::json&         j)    override;
    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

private:
    [[nodiscard]] float thrustIdle(float rho_kgm3) const;

    PropulsionEdfParams                 _params;
    float                               _disk_area_m2;  // precomputed: π·(fan_diameter_m/2)²
    liteaerosim::control::FilterSS2Clip _spool_filter;
    float                               _thrust_n;
};

} // namespace liteaerosim::propulsion
