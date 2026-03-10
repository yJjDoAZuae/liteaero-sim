#include "airframe/Inertia.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

namespace liteaerosim {

nlohmann::json Inertia::serializeJson() const {
    return {
        {"schema_version", 1},
        {"type",           "Inertia"},
        {"mass_kg",        mass_kg},
        {"Ixx_kgm2",       Ixx_kgm2},
        {"Iyy_kgm2",       Iyy_kgm2},
        {"Izz_kgm2",       Izz_kgm2},
    };
}

Inertia Inertia::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("Inertia::deserializeJson: unsupported schema_version");
    }
    Inertia i;
    i.mass_kg  = j.at("mass_kg").get<float>();
    i.Ixx_kgm2 = j.at("Ixx_kgm2").get<float>();
    i.Iyy_kgm2 = j.at("Iyy_kgm2").get<float>();
    i.Izz_kgm2 = j.at("Izz_kgm2").get<float>();
    return i;
}

std::vector<uint8_t> Inertia::serializeProto() const {
    las_proto::InertiaParams proto;
    proto.set_schema_version(1);
    proto.set_mass_kg(mass_kg);
    proto.set_ixx_kgm2(Ixx_kgm2);
    proto.set_iyy_kgm2(Iyy_kgm2);
    proto.set_izz_kgm2(Izz_kgm2);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

Inertia Inertia::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::InertiaParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("Inertia::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error("Inertia::deserializeProto: unsupported schema_version");
    }
    Inertia i;
    i.mass_kg  = proto.mass_kg();
    i.Ixx_kgm2 = proto.ixx_kgm2();
    i.Iyy_kgm2 = proto.iyy_kgm2();
    i.Izz_kgm2 = proto.izz_kgm2();
    return i;
}

}  // namespace liteaerosim
