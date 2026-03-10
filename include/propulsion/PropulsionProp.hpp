#pragma once

#include "propulsion/V_Propulsion.hpp"
#include "propulsion/PropellerAero.hpp"
#include "propulsion/V_Motor.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <memory>
#include <vector>

namespace liteaerosim::propulsion {

// Propeller propulsion model.  Owns a PropellerAero (value) and a V_Motor (polymorphic).
// Rotor speed dynamics are modelled by a first-order IIR FilterSS2Clip.
//
// Each step:
//   Omega_target = motor.noLoadOmega_rps(throttle_nd, rho_kgm3)
//   Omega_actual = _rotor_filter.step(Omega_target)   [clamped to [0, motor.maxOmega_rps()]]
//   thrust_n     = propeller.thrust_n(Omega_actual, tas_mps, rho_kgm3)
class PropulsionProp : public V_Propulsion {
public:
    // propeller   — geometry and coefficient model (value; copied)
    // motor       — power source (ownership transferred)
    // dt_s        — fixed simulation timestep (s)
    // rotor_tau_s — first-order rotor speed lag (s); typically 0.2–3 s
    PropulsionProp(PropellerAero              propeller,
                   std::unique_ptr<V_Motor>   motor,
                   float                      dt_s,
                   float                      rotor_tau_s);

    // V_Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;
    void reset() override;

    // Rotor speed from the most recent step() call (rad/s).  Zero before first step.
    [[nodiscard]] float omega_rps() const;

    [[nodiscard]] nlohmann::json        serializeJson()                               const override;
    void                                deserializeJson(const nlohmann::json&         j)    override;
    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

private:
    PropellerAero                       _propeller;
    std::unique_ptr<V_Motor>            _motor;
    liteaerosim::control::FilterSS2Clip _rotor_filter;
    float                               _thrust_n;
};

} // namespace liteaerosim::propulsion
