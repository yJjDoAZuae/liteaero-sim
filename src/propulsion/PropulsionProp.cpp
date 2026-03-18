#define _USE_MATH_DEFINES
#include "propulsion/PropulsionProp.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::propulsion {

// ── Construction ──────────────────────────────────────────────────────────────

PropulsionProp::PropulsionProp(std::unique_ptr<Motor> motor)
    : _motor(std::move(motor))
{}

// ── DynamicElement hooks ───────────────────────────────────────────────────────

void PropulsionProp::onInitialize(const nlohmann::json& config) {
    const float diameter_m     = config.at("diameter_m").get<float>();
    const float pitch_m        = config.at("pitch_m").get<float>();
    const int   blade_count    = config.at("blade_count").get<int>();
    const float blade_solidity = config.at("blade_solidity").get<float>();
    const float dt_s           = config.at("dt_s").get<float>();
    const float rotor_tau_s    = config.at("rotor_tau_s").get<float>();

    _propeller.emplace(diameter_m, pitch_m, blade_count, blade_solidity);

    _rotor_filter.setLowPassFirstIIR(dt_s, rotor_tau_s);
    _rotor_filter.valLimit.setLower(0.f);
    _rotor_filter.valLimit.enableLower();
    _rotor_filter.valLimit.setUpper(_motor->maxOmega_rps());
    _rotor_filter.valLimit.enableUpper();

    _thrust_n = 0.f;
}

void PropulsionProp::onReset() {
    _rotor_filter.resetToOutput(0.f);
    _thrust_n = 0.f;
}

nlohmann::json PropulsionProp::onSerializeJson() const {
    const Mat21 rx = _rotor_filter.x();
    return nlohmann::json{
        {"thrust_n",    _thrust_n},
        {"rotor_state", {rx(0, 0), rx(1, 0)}},
    };
}

void PropulsionProp::onDeserializeJson(const nlohmann::json& j) {
    if (j.at("type").get<std::string>() != "PropulsionProp")
        throw std::runtime_error("PropulsionProp::onDeserializeJson: type mismatch");

    Mat21 rx;
    rx(0, 0) = j.at("rotor_state")[0].get<float>();
    rx(1, 0) = j.at("rotor_state")[1].get<float>();
    _rotor_filter.valLimit.disableUpper();
    _rotor_filter.resetState(rx);
    _rotor_filter.valLimit.setUpper(_motor->maxOmega_rps());
    _rotor_filter.valLimit.enableUpper();

    _thrust_n = j.at("thrust_n").get<float>();
}

// ── Propulsion interface ───────────────────────────────────────────────────────

float PropulsionProp::step(float throttle_nd, float tas_mps, float rho_kgm3) {
    const float omega_target = _motor->noLoadOmega_rps(throttle_nd, rho_kgm3);

    _rotor_filter.valLimit.setUpper(_motor->maxOmega_rps());
    const float omega_actual = _rotor_filter.step(omega_target);

    _thrust_n = _propeller->thrust_n(omega_actual, tas_mps, rho_kgm3);
    return _thrust_n;
}

float PropulsionProp::thrust_n() const { return _thrust_n; }

float PropulsionProp::omega_rps() const { return _rotor_filter.out(); }

// ── Proto serialization ────────────────────────────────────────────────────────

std::vector<uint8_t> PropulsionProp::serializeProto() const {
    las_proto::PropulsionPropState proto;
    proto.set_schema_version(1);
    proto.set_thrust_n(_thrust_n);
    const Mat21 rx = _rotor_filter.x();
    proto.add_rotor_state(rx(0, 0));
    proto.add_rotor_state(rx(1, 0));
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void PropulsionProp::deserializeProto(const std::vector<uint8_t>& b) {
    las_proto::PropulsionPropState proto;
    if (!proto.ParseFromArray(b.data(), static_cast<int>(b.size())))
        throw std::runtime_error("PropulsionProp::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("PropulsionProp::deserializeProto: unsupported schema_version");

    Mat21 rx;
    rx(0, 0) = proto.rotor_state_size() > 0 ? proto.rotor_state(0) : 0.f;
    rx(1, 0) = proto.rotor_state_size() > 1 ? proto.rotor_state(1) : 0.f;
    _rotor_filter.valLimit.disableUpper();
    _rotor_filter.resetState(rx);
    _rotor_filter.valLimit.setUpper(_motor->maxOmega_rps());
    _rotor_filter.valLimit.enableUpper();

    _thrust_n = proto.thrust_n();
}

} // namespace liteaerosim::propulsion
