#define _USE_MATH_DEFINES
#include "aerodynamics/AeroPerformance.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::aerodynamics {

AeroPerformance::AeroPerformance(const AeroPerformanceConfig& cfg)
    : _S(cfg.s_ref_m2), _ar(cfg.ar), _e(cfg.e), _cd0(cfg.cd0),
      _k(1.f / (static_cast<float>(M_PI) * cfg.e * cfg.ar)),
      _cl_y_beta(cfg.cl_y_beta),
      _cl_q_nd(cfg.cl_q_nd),
      _mac_m(cfg.mac_m),
      _cy_r_nd(cfg.cy_r_nd),
      _fin_arm_m(cfg.fin_arm_m)
{}

AeroForces AeroPerformance::compute(float /*alpha_rad*/, float beta_rad,
                                    float q_inf_pa, float cl) const
{
    const float cdi = _k * cl * cl;
    const float cd  = _cd0 + cdi;
    const float cy  = _cl_y_beta * beta_rad;
    const float qS  = q_inf_pa * _S;
    return {-qS * cd, qS * cy, -qS * cl};
}

// ── Serialization ──────────────────────────────────────────────────────────────

nlohmann::json AeroPerformance::serializeJson() const {
    return {
        {"schema_version", 2},
        {"type",           "AeroPerformance"},
        {"s_ref_m2",       _S},
        {"ar",             _ar},
        {"e",              _e},
        {"cd0",            _cd0},
        {"cl_y_beta",      _cl_y_beta},
        {"cl_q_nd",        _cl_q_nd},
        {"mac_m",          _mac_m},
        {"cy_r_nd",        _cy_r_nd},
        {"fin_arm_m",      _fin_arm_m},
    };
}

AeroPerformance AeroPerformance::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 2) {
        throw std::runtime_error("AeroPerformance::deserializeJson: unsupported schema_version");
    }
    AeroPerformanceConfig cfg;
    cfg.s_ref_m2  = j.at("s_ref_m2").get<float>();
    cfg.ar        = j.at("ar").get<float>();
    cfg.e         = j.at("e").get<float>();
    cfg.cd0       = j.at("cd0").get<float>();
    cfg.cl_y_beta = j.at("cl_y_beta").get<float>();
    cfg.cl_q_nd   = j.at("cl_q_nd").get<float>();
    cfg.mac_m     = j.at("mac_m").get<float>();
    cfg.cy_r_nd   = j.at("cy_r_nd").get<float>();
    cfg.fin_arm_m = j.at("fin_arm_m").get<float>();
    return AeroPerformance(cfg);
}

std::vector<uint8_t> AeroPerformance::serializeProto() const {
    las_proto::AeroPerformanceParams proto;
    proto.set_schema_version(2);
    proto.set_s_ref_m2(_S);
    proto.set_ar(_ar);
    proto.set_e(_e);
    proto.set_cd0(_cd0);
    proto.set_cl_y_beta(_cl_y_beta);
    proto.set_cl_q_nd(_cl_q_nd);
    proto.set_mac_m(_mac_m);
    proto.set_cy_r_nd(_cy_r_nd);
    proto.set_fin_arm_m(_fin_arm_m);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

AeroPerformance AeroPerformance::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AeroPerformanceParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("AeroPerformance::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 2) {
        throw std::runtime_error("AeroPerformance::deserializeProto: unsupported schema_version");
    }
    AeroPerformanceConfig cfg;
    cfg.s_ref_m2  = proto.s_ref_m2();
    cfg.ar        = proto.ar();
    cfg.e         = proto.e();
    cfg.cd0       = proto.cd0();
    cfg.cl_y_beta = proto.cl_y_beta();
    cfg.cl_q_nd   = proto.cl_q_nd();
    cfg.mac_m     = proto.mac_m();
    cfg.cy_r_nd   = proto.cy_r_nd();
    cfg.fin_arm_m = proto.fin_arm_m();
    return AeroPerformance(cfg);
}

} // namespace liteaerosim::aerodynamics
