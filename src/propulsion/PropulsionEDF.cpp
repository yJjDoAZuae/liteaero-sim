#define _USE_MATH_DEFINES
#include "propulsion/PropulsionEDF.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::propulsion {

static constexpr float kRhoSL_kgm3 = 1.225f;
static constexpr float kPi         = 3.14159265358979f;

// ── Construction ──────────────────────────────────────────────────────────────

PropulsionEDF::PropulsionEDF(const PropulsionEdfParams& params, float dt_s)
    : _params(params)
    , _disk_area_m2(kPi * (params.fan_diameter_m * 0.5f) * (params.fan_diameter_m * 0.5f))
    , _thrust_n(0.f)
{
    _spool_filter.setLowPassFirstIIR(dt_s, params.rotor_tau_s);
    _spool_filter.valLimit.enableLower();  // lower = 0; per-step updated
    _spool_filter.valLimit.enableUpper();  // upper = 0; per-step updated
}

// ── V_Propulsion interface ────────────────────────────────────────────────────

float PropulsionEDF::thrustIdle(float rho_kgm3) const {
    return _params.idle_fraction * _params.thrust_sl_n * (rho_kgm3 / kRhoSL_kgm3);
}

float PropulsionEDF::step(float throttle_nd, float tas_mps, float rho_kgm3) {
    const float T_gross  = throttle_nd * _params.thrust_sl_n * (rho_kgm3 / kRhoSL_kgm3);
    const float F_ram    = rho_kgm3 * tas_mps * tas_mps * _params.inlet_area_m2;
    const float T_idle   = thrustIdle(rho_kgm3);
    const float T_avail  = std::max(T_idle, T_gross - F_ram);
    const float T_demand = throttle_nd * T_avail;

    _spool_filter.valLimit.setLower(T_idle);
    _spool_filter.valLimit.setUpper(T_avail);
    _thrust_n = _spool_filter.step(T_demand);
    return _thrust_n;
}

float PropulsionEDF::thrust_n() const { return _thrust_n; }

void PropulsionEDF::reset() {
    _spool_filter.resetOutput(0.f);
    _thrust_n = 0.f;
}

float PropulsionEDF::batteryCurrent_a(float tas_mps, float rho_kgm3) const {
    if (_thrust_n <= 0.f) return 0.f;
    // Actuator disk induced velocity.
    const float half_v   = tas_mps * 0.5f;
    const float v_i      = -half_v + std::sqrt(half_v * half_v
                            + _thrust_n / (2.f * rho_kgm3 * _disk_area_m2));
    const float P_fan    = _thrust_n * (tas_mps + v_i);
    return P_fan / (_params.fan_efficiency_nd * _params.esc_efficiency_nd
                    * _params.supply_voltage_v);
}

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json PropulsionEDF::serializeJson() const {
    // spool_state = FilterSS2Clip internal state vector x: [x[0], x[1]]
    const Mat21 sx = _spool_filter.x();
    return nlohmann::json{
        {"schema_version", 1},
        {"type",           "PropulsionEDF"},
        {"thrust_n",        _thrust_n},
        {"spool_state",    {sx(0, 0), sx(1, 0)}},
    };
}

void PropulsionEDF::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("PropulsionEDF::deserializeJson: unsupported schema_version");
    if (j.at("type").get<std::string>() != "PropulsionEDF")
        throw std::runtime_error("PropulsionEDF::deserializeJson: type mismatch");

    Mat21 sx;
    sx(0, 0) = j.at("spool_state")[0].get<float>();
    sx(1, 0) = j.at("spool_state")[1].get<float>();
    _spool_filter.valLimit.disableUpper();
    _spool_filter.resetState(sx);
    _spool_filter.valLimit.enableUpper();

    _thrust_n = j.at("thrust_n").get<float>();
}

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
