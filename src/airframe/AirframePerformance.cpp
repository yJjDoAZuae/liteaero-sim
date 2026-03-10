#include "airframe/AirframePerformance.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

namespace liteaerosim {

nlohmann::json AirframePerformance::serializeJson() const {
    return {
        {"schema_version", 1},
        {"type",           "AirframePerformance"},
        {"g_max_nd",       g_max_nd},
        {"g_min_nd",       g_min_nd},
        {"tas_max_mps",    tas_max_mps},
        {"mach_max_nd",    mach_max_nd},
    };
}

AirframePerformance AirframePerformance::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error(
            "AirframePerformance::deserializeJson: unsupported schema_version");
    }
    AirframePerformance a;
    a.g_max_nd    = j.at("g_max_nd").get<float>();
    a.g_min_nd    = j.at("g_min_nd").get<float>();
    a.tas_max_mps = j.at("tas_max_mps").get<float>();
    a.mach_max_nd = j.at("mach_max_nd").get<float>();
    return a;
}

std::vector<uint8_t> AirframePerformance::serializeProto() const {
    las_proto::AirframePerformanceParams proto;
    proto.set_schema_version(1);
    proto.set_g_max_nd(g_max_nd);
    proto.set_g_min_nd(g_min_nd);
    proto.set_tas_max_mps(tas_max_mps);
    proto.set_mach_max_nd(mach_max_nd);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

AirframePerformance AirframePerformance::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AirframePerformanceParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error(
            "AirframePerformance::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error(
            "AirframePerformance::deserializeProto: unsupported schema_version");
    }
    AirframePerformance a;
    a.g_max_nd    = proto.g_max_nd();
    a.g_min_nd    = proto.g_min_nd();
    a.tas_max_mps = proto.tas_max_mps();
    a.mach_max_nd = proto.mach_max_nd();
    return a;
}

}  // namespace liteaerosim
