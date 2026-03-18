#pragma once

#include "DynamicElement.hpp"
#include <cstdint>
#include <vector>

namespace liteaerosim::propulsion {

/// Abstract base for all propulsion models.
///
/// Derives from DynamicElement for unified lifecycle (initialize / reset /
/// serializeJson / deserializeJson). Adds the propulsion-specific step()
/// signature and thrust_n() accessor, which Aircraft calls through this base.
///
/// Proto serialization is included here because Aircraft dispatches
/// deserializeProto() generically through this base pointer. Each concrete
/// subclass serializes to and from its own proto message type.
class Propulsion : public liteaerosim::DynamicElement {
public:
    /// Advance propulsion state by one timestep and return thrust magnitude (N).
    ///   throttle_nd — normalized throttle demand [0, 1]
    ///   tas_mps     — true airspeed (m/s)
    ///   rho_kgm3    — ambient air density (kg/m³)
    [[nodiscard]] virtual float step(float throttle_nd,
                                     float tas_mps,
                                     float rho_kgm3) = 0;

    /// Thrust output from the most recent step() call (N). Zero before first step.
    [[nodiscard]] virtual float thrust_n() const = 0;

    /// Serialize warm-start state to protobuf bytes.
    [[nodiscard]] virtual std::vector<uint8_t> serializeProto()                             const = 0;

    /// Restore warm-start state from protobuf bytes.
    virtual void                               deserializeProto(const std::vector<uint8_t>& b)    = 0;
};

} // namespace liteaerosim::propulsion
