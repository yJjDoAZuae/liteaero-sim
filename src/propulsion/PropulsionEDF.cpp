#define _USE_MATH_DEFINES
#include "propulsion/PropulsionEDF.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::propulsion {

static constexpr float kRhoSL_kgm3 = 1.225f;
static constexpr float kPi         = 3.14159265358979f;

// ── DynamicElement hooks ───────────────────────────────────────────────────────

void PropulsionEDF::onInitialize(const nlohmann::json& config) {
    _thrust_sl_n       = config.at("thrust_sl_n").get<float>();
    _fan_diameter_m    = config.at("fan_diameter_m").get<float>();
    _inlet_area_m2     = config.at("inlet_area_m2").get<float>();
    _idle_fraction     = config.at("idle_fraction").get<float>();
    _rotor_tau_s       = config.at("rotor_tau_s").get<float>();
    _supply_voltage_v  = config.at("supply_voltage_v").get<float>();
    _fan_efficiency_nd = config.at("fan_efficiency_nd").get<float>();
    _esc_efficiency_nd = config.at("esc_efficiency_nd").get<float>();
    const float dt_s   = config.at("dt_s").get<float>();

    _disk_area_m2 = kPi * (_fan_diameter_m * 0.5f) * (_fan_diameter_m * 0.5f);

    _spool_filter.setLowPassFirstIIR(dt_s, _rotor_tau_s);
    _spool_filter.valLimit.enableLower();
    _spool_filter.valLimit.enableUpper();

    _thrust_n = 0.f;
}

void PropulsionEDF::onReset() {
    _spool_filter.resetToOutput(0.f);
    _thrust_n = 0.f;
}

nlohmann::json PropulsionEDF::onSerializeJson() const {
    const Mat21 sx = _spool_filter.x();
    return nlohmann::json{
        {"thrust_n",    _thrust_n},
        {"spool_state", {sx(0, 0), sx(1, 0)}},
    };
}

void PropulsionEDF::onDeserializeJson(const nlohmann::json& j) {
    if (j.at("type").get<std::string>() != "PropulsionEDF")
        throw std::runtime_error("PropulsionEDF::onDeserializeJson: type mismatch");

    Mat21 sx;
    sx(0, 0) = j.at("spool_state")[0].get<float>();
    sx(1, 0) = j.at("spool_state")[1].get<float>();
    _spool_filter.valLimit.disableUpper();
    _spool_filter.resetState(sx);
    _spool_filter.valLimit.enableUpper();

    _thrust_n = j.at("thrust_n").get<float>();
}

// ── Propulsion interface ───────────────────────────────────────────────────────

float PropulsionEDF::thrustIdle(float rho_kgm3) const {
    return _idle_fraction * _thrust_sl_n * (rho_kgm3 / kRhoSL_kgm3);
}

float PropulsionEDF::step(float throttle_nd, float tas_mps, float rho_kgm3) {
    const float T_gross  = throttle_nd * _thrust_sl_n * (rho_kgm3 / kRhoSL_kgm3);
    const float F_ram    = rho_kgm3 * tas_mps * tas_mps * _inlet_area_m2;
    const float T_idle   = thrustIdle(rho_kgm3);
    const float T_avail  = std::max(T_idle, T_gross - F_ram);
    const float T_demand = throttle_nd * T_avail;

    _spool_filter.valLimit.setLower(T_idle);
    _spool_filter.valLimit.setUpper(T_avail);
    _thrust_n = _spool_filter.step(T_demand);
    return _thrust_n;
}

float PropulsionEDF::thrust_n() const { return _thrust_n; }

float PropulsionEDF::batteryCurrent_a(float tas_mps, float rho_kgm3) const {
    if (_thrust_n <= 0.f) return 0.f;
    const float half_v = tas_mps * 0.5f;
    const float v_i    = -half_v + std::sqrt(half_v * half_v
                          + _thrust_n / (2.f * rho_kgm3 * _disk_area_m2));
    const float P_fan  = _thrust_n * (tas_mps + v_i);
    return P_fan / (_fan_efficiency_nd * _esc_efficiency_nd * _supply_voltage_v);
}

// ── Proto serialization ────────────────────────────────────────────────────────

std::vector<uint8_t> PropulsionEDF::serializeProto() const {
    las_proto::PropulsionEdfState proto;
    proto.set_schema_version(1);
    proto.set_thrust_n(_thrust_n);
    const Mat21 sx = _spool_filter.x();
    proto.add_spool_state(sx(0, 0));
    proto.add_spool_state(sx(1, 0));
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void PropulsionEDF::deserializeProto(const std::vector<uint8_t>& b) {
    las_proto::PropulsionEdfState proto;
    if (!proto.ParseFromArray(b.data(), static_cast<int>(b.size())))
        throw std::runtime_error("PropulsionEDF::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("PropulsionEDF::deserializeProto: unsupported schema_version");

    Mat21 sx;
    sx(0, 0) = proto.spool_state_size() > 0 ? proto.spool_state(0) : 0.f;
    sx(1, 0) = proto.spool_state_size() > 1 ? proto.spool_state(1) : 0.f;
    _spool_filter.valLimit.disableUpper();
    _spool_filter.resetState(sx);
    _spool_filter.valLimit.enableUpper();

    _thrust_n = proto.thrust_n();
}

} // namespace liteaerosim::propulsion
