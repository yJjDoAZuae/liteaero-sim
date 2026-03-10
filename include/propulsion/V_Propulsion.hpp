#pragma once

#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

namespace liteaerosim::propulsion {

// Abstract propulsion interface.  Each concrete subclass encapsulates one
// propulsion technology (jet, EDF, propeller, …) and implements the full
// step / serialize / deserialize lifecycle.
class V_Propulsion {
public:
    virtual ~V_Propulsion() = default;

    // Advance propulsion state by one timestep and return thrust magnitude (N).
    //   throttle_nd — normalized throttle demand [0, 1]
    //   tas_mps     — true airspeed (m/s)
    //   rho_kgm3    — ambient air density (kg/m³)
    [[nodiscard]] virtual float step(float throttle_nd,
                                     float tas_mps,
                                     float rho_kgm3) = 0;

    // Thrust output from the most recent step() call (N).  Zero before first step.
    [[nodiscard]] virtual float thrust_n() const = 0;

    // Reset warm-start state.  thrust_n() returns 0 after reset().
    virtual void reset() = 0;

    // Serialize warm-start state only (not configuration parameters).
    // Snapshot includes "schema_version" (int) and "type" (string discriminator).
    [[nodiscard]] virtual nlohmann::json        serializeJson()                               const = 0;
    virtual void                                deserializeJson(const nlohmann::json&         j)    = 0;
    [[nodiscard]] virtual std::vector<uint8_t>  serializeProto()                             const = 0;
    virtual void                                deserializeProto(const std::vector<uint8_t>& b)    = 0;
};

} // namespace liteaerosim::propulsion
