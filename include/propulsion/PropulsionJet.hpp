#pragma once

#include "propulsion/Propulsion.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <vector>

namespace liteaerosim::propulsion {

// Physics-based jet engine model.
//
// Thrust lapse:  T_gross(ρ) = T_sl · (ρ/ρ_SL)^n,  n = 1/√(1+BPR)
// Ram drag:      F_ram = ρ · V² · A_inlet
// Spool lag:     first-order IIR via FilterSS2Clip
// Afterburner:   separate FilterSS2Clip, additive to dry thrust
//
// Config format (passed to initialize()):
//   {
//     "thrust_sl_n":    80000,   // Max dry thrust at sea level, full throttle (N)
//     "bypass_ratio":   0.5,     // BPR; 0 = turbojet
//     "inlet_area_m2":  0.30,    // Inlet capture area for ram drag (m²)
//     "idle_fraction":  0.05,    // Idle thrust as fraction of thrust_sl_n at sea level (—)
//     "spool_tau_s":    2.0,     // First-order spool lag time constant (s)
//     "ab_thrust_sl_n": 20000,   // Afterburner thrust increment at SL (N); 0 = no afterburner
//     "ab_spool_tau_s": 1.0,     // Afterburner light-off / blow-out time constant (s)
//     "dt_s":           0.01     // Fixed simulation timestep (s)
//   }
class PropulsionJet : public Propulsion {
public:
    PropulsionJet() = default;

    // Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;

    // Engage or disengage the afterburner. Disengaging causes natural blow-out.
    void setAfterburner(bool on);

    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                  override;
    nlohmann::json onSerializeJson()                    const override;
    void           onDeserializeJson(const nlohmann::json& j) override;
    int            schemaVersion()                      const override { return 1; }
    const char*    typeName()                           const override { return "PropulsionJet"; }

private:
    [[nodiscard]] float thrustIdle(float rho_kgm3) const;

    float                               _thrust_sl_n        = 0.f;
    float                               _bypass_ratio       = 0.f;
    float                               _inlet_area_m2      = 0.f;
    float                               _idle_fraction      = 0.f;
    float                               _spool_tau_s        = 1.f;
    float                               _ab_thrust_sl_n     = 0.f;
    float                               _ab_spool_tau_s     = 1.f;
    float                               _density_exponent   = 1.f;  // precomputed: 1/√(1+BPR)
    liteaerosim::control::FilterSS2Clip _spool_filter;
    liteaerosim::control::FilterSS2Clip _ab_filter;
    float                               _thrust_n           = 0.f;
    bool                                _ab_active          = false;
};

} // namespace liteaerosim::propulsion
