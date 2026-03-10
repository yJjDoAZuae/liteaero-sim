// Inertial properties of the aircraft.

#pragma once
#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

namespace liteaerosim {

struct Inertia {
    float mass_kg  = 0.0f;  // total aircraft mass (kg)
    float Ixx_kgm2 = 0.0f;  // roll moment of inertia (kg·m²)
    float Iyy_kgm2 = 0.0f;  // pitch moment of inertia (kg·m²)
    float Izz_kgm2 = 0.0f;  // yaw moment of inertia (kg·m²)

    nlohmann::json  serializeJson()                                      const;
    static Inertia  deserializeJson(const nlohmann::json&        j);
    std::vector<uint8_t> serializeProto()                                const;
    static Inertia  deserializeProto(const std::vector<uint8_t>& bytes);
};

}  // namespace liteaerosim
