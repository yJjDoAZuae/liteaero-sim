#pragma once

#include "propulsion/Propulsion.hpp"
#include "propulsion/Motor.hpp"
#include "propulsion/PropellerAero.hpp"
#include "control/FilterSS2Clip.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace liteaerosim::propulsion {

// Propeller propulsion model.  Owns a PropellerAero (value) and a Motor (polymorphic).
// Rotor speed dynamics are modelled by a first-order IIR FilterSS2Clip.
//
// Each step:
//   Omega_target = motor.noLoadOmega_rps(throttle_nd, rho_kgm3)
//   Omega_actual = _rotor_filter.step(Omega_target)   [clamped to [0, motor.maxOmega_rps()]]
//   thrust_n     = propeller.thrust_n(Omega_actual, tas_mps, rho_kgm3)
//
// Config format (passed to initialize()):
//   {
//     "diameter_m":     0.5,    // Propeller diameter (m)
//     "pitch_m":        0.5,    // Geometric pitch (m) — advance per revolution at zero slip
//     "blade_count":    3,      // Number of blades
//     "blade_solidity": 0.15,   // σ = N_b · c_mean / (π · R)
//     "dt_s":           0.01,   // Fixed simulation timestep (s)
//     "rotor_tau_s":    0.3     // First-order rotor speed lag time constant (s)
//   }
class PropulsionProp : public Propulsion {
public:
    // motor — power source (ownership transferred at construction)
    explicit PropulsionProp(std::unique_ptr<Motor> motor);

    // Propulsion interface
    [[nodiscard]] float step(float throttle_nd, float tas_mps, float rho_kgm3) override;
    [[nodiscard]] float thrust_n() const override;

    // Rotor speed from the most recent step() call (rad/s).  Zero before first step.
    [[nodiscard]] float omega_rps() const;

    [[nodiscard]] std::vector<uint8_t>  serializeProto()                             const override;
    void                                deserializeProto(const std::vector<uint8_t>& b)    override;

protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                  override;
    nlohmann::json onSerializeJson()                    const override;
    void           onDeserializeJson(const nlohmann::json& j) override;
    int            schemaVersion()                      const override { return 1; }
    const char*    typeName()                           const override { return "PropulsionProp"; }

private:
    std::unique_ptr<Motor>              _motor;
    std::optional<PropellerAero>        _propeller;
    liteaerosim::control::FilterSS2Clip _rotor_filter;
    float                               _thrust_n           = 0.f;
};

} // namespace liteaerosim::propulsion
