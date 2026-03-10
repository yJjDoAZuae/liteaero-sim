#pragma once

#include "propulsion/V_Propulsion.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <vector>

namespace liteaerosim::propulsion {

struct PropulsionJetParams {
    float thrust_sl_n;      // Max dry thrust at sea level, full throttle (N)
    float bypass_ratio;     // BPR; 0 = turbojet; drives density exponent n = 1/√(1+BPR)
    float inlet_area_m2;    // Inlet capture area for ram drag (m²)
    float idle_fraction;    // Idle thrust as fraction of thrust_sl_n at sea level (—)
    float spool_tau_s;      // First-order spool lag time constant (s)
    float ab_thrust_sl_n;   // Afterburner thrust increment at SL (N); 0 = no afterburner
    float ab_spool_tau_s;   // Afterburner light-off / blow-out time constant (s)
};

// Physics-based jet engine model.
//
// Thrust lapse:  T_gross(ρ) = T_sl · (ρ/ρ_SL)^n,  n = 1/√(1+BPR)
// Ram drag:      F_ram = ρ · V² · A_inlet
// Spool lag:     first-order IIR via FilterSS2Clip
// Afterburner:   separate FilterSS2Clip, additive to dry thrust
class PropulsionJet : public V_Propulsion {
public:
    PropulsionJet(const PropulsionJetParams& params, float dt_s);

    // V_Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;
    void reset() override;

    // Engage or disengage the afterburner.  Disengaging causes natural blow-out.
    void setAfterburner(bool on);

    [[nodiscard]] nlohmann::json        serializeJson()                               const override;
    void                                deserializeJson(const nlohmann::json&         j)    override;
    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

private:
    [[nodiscard]] float thrustIdle(float rho_kgm3) const;

    PropulsionJetParams                  _params;
    float                               _density_exponent;  // precomputed: 1/√(1+BPR)
    liteaerosim::control::FilterSS2Clip _spool_filter;
    liteaerosim::control::FilterSS2Clip _ab_filter;
    float                               _thrust_n;
    bool                                _ab_active;
};

} // namespace liteaerosim::propulsion
