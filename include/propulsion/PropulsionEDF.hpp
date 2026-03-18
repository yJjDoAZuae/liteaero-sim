#pragma once

#include "propulsion/Propulsion.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <vector>

namespace liteaerosim::propulsion {

// Physics-based electric ducted fan model.
//
// Thrust lapse:  T_gross(δ_T, ρ) = δ_T · T_sl · (ρ/ρ_SL)   (density exponent fixed at 1)
// Ram drag:      F_ram = ρ · V² · A_inlet
// Rotor lag:     first-order IIR via FilterSS2Clip
//
// Config format (passed to initialize()):
//   {
//     "thrust_sl_n":       200,    // Full-throttle static thrust at sea level (N)
//     "fan_diameter_m":    0.12,   // Fan disk diameter (m)
//     "inlet_area_m2":     0.011,  // Effective duct inlet capture area (m²)
//     "idle_fraction":     0.05,   // Idle thrust as fraction of thrust_sl_n (—)
//     "rotor_tau_s":       0.1,    // First-order lag: throttle → thrust (s)
//     "supply_voltage_v":  22.2,   // DC battery bus voltage (V)
//     "fan_efficiency_nd": 0.75,   // Fan + duct aerodynamic efficiency (0.65–0.85)
//     "esc_efficiency_nd": 0.95,   // ESC DC-to-3-phase power conversion efficiency (0.90–0.98)
//     "dt_s":              0.01    // Fixed simulation timestep (s)
//   }
class PropulsionEDF : public Propulsion {
public:
    PropulsionEDF() = default;

    // Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;

    // Battery current at current thrust and flight condition (A).
    // For energy / endurance analysis only; not called by step().
    [[nodiscard]] float batteryCurrent_a(float tas_mps, float rho_kgm3) const;

    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                  override;
    nlohmann::json onSerializeJson()                    const override;
    void           onDeserializeJson(const nlohmann::json& j) override;
    int            schemaVersion()                      const override { return 1; }
    const char*    typeName()                           const override { return "PropulsionEDF"; }

private:
    [[nodiscard]] float thrustIdle(float rho_kgm3) const;

    float                               _thrust_sl_n        = 0.f;
    float                               _fan_diameter_m     = 0.f;
    float                               _inlet_area_m2      = 0.f;
    float                               _idle_fraction      = 0.f;
    float                               _rotor_tau_s        = 1.f;
    float                               _supply_voltage_v   = 0.f;
    float                               _fan_efficiency_nd  = 1.f;
    float                               _esc_efficiency_nd  = 1.f;
    float                               _disk_area_m2       = 0.f;  // precomputed: π·(fan_diameter_m/2)²
    liteaerosim::control::FilterSS2Clip _spool_filter;
    float                               _thrust_n           = 0.f;
};

} // namespace liteaerosim::propulsion
