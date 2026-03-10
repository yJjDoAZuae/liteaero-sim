// Structural and operational performance limits of the aircraft.

#pragma once
#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

namespace liteaerosim {

struct AirframePerformance {
    float g_max_nd    = 0.0f;  // maximum positive load factor (dimensionless)
    float g_min_nd    = 0.0f;  // maximum negative load factor (dimensionless, negative value)
    float tas_max_mps = 0.0f;  // never-exceed true airspeed (m/s)
    float mach_max_nd = 0.0f;  // never-exceed Mach number (dimensionless)

    nlohmann::json              serializeJson()                                      const;
    static AirframePerformance  deserializeJson(const nlohmann::json&        j);
    std::vector<uint8_t>        serializeProto()                                     const;
    static AirframePerformance  deserializeProto(const std::vector<uint8_t>& bytes);
};

}  // namespace liteaerosim
