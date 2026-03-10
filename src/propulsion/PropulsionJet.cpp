#define _USE_MATH_DEFINES
#include "propulsion/PropulsionJet.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::propulsion {

static constexpr float kRhoSL_kgm3 = 1.225f;

// ── Construction ──────────────────────────────────────────────────────────────

PropulsionJet::PropulsionJet(const PropulsionJetParams& params, float dt_s)
    : _params(params)
    , _density_exponent(1.f / std::sqrt(1.f + params.bypass_ratio))
    , _thrust_n(0.f)
    , _ab_active(false)
{
    _spool_filter.setLowPassFirstIIR(dt_s, params.spool_tau_s);
    _spool_filter.valLimit.enableLower();  // lower = 0 (default); per-step updated
    _spool_filter.valLimit.enableUpper();  // upper = 0 (default); per-step updated

    const float ab_tau = params.ab_spool_tau_s > 0.f ? params.ab_spool_tau_s : params.spool_tau_s;
    _ab_filter.setLowPassFirstIIR(dt_s, ab_tau);
    _ab_filter.valLimit.setLower(0.f);
    _ab_filter.valLimit.enableLower();
    _ab_filter.valLimit.enableUpper();  // upper set per step
}

// ── V_Propulsion interface ────────────────────────────────────────────────────

float PropulsionJet::thrustIdle(float rho_kgm3) const {
    return _params.idle_fraction * _params.thrust_sl_n
           * std::pow(rho_kgm3 / kRhoSL_kgm3, _density_exponent);
}

float PropulsionJet::step(float throttle_nd, float tas_mps, float rho_kgm3) {
    // ── Dry spool ──
    const float T_gross  = _params.thrust_sl_n
                           * std::pow(rho_kgm3 / kRhoSL_kgm3, _density_exponent);
    const float F_ram    = rho_kgm3 * tas_mps * tas_mps * _params.inlet_area_m2;
    const float T_idle   = thrustIdle(rho_kgm3);
    const float T_avail  = std::max(T_idle, T_gross - F_ram);
    const float T_demand = throttle_nd * T_avail;

    _spool_filter.valLimit.setLower(T_idle);
    _spool_filter.valLimit.setUpper(T_avail);
    const float T_dry = _spool_filter.step(T_demand);

    // ── Afterburner ──
    const float T_ab_avail  = _params.ab_thrust_sl_n
                              * std::pow(rho_kgm3 / kRhoSL_kgm3, _density_exponent);
    const float T_ab_demand = _ab_active ? T_ab_avail : 0.f;
    _ab_filter.valLimit.setUpper(_ab_active ? T_ab_avail : 0.f);
    const float T_ab = _ab_filter.step(T_ab_demand);

    _thrust_n = T_dry + T_ab;
    return _thrust_n;
}

float PropulsionJet::thrust_n() const { return _thrust_n; }

void PropulsionJet::reset() {
    _spool_filter.resetOutput(0.f);
    _ab_filter.resetOutput(0.f);
    _thrust_n = 0.f;
    _ab_active = false;
}

void PropulsionJet::setAfterburner(bool on) { _ab_active = on; }

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json PropulsionJet::serializeJson() const {
    // spool_state = FilterSS2Clip internal state vector x: [x[0], x[1]]
    // ab_state    = FilterSS2Clip internal state vector x: [x[0], x[1]]
    const Mat21 sx = _spool_filter.x();
    const Mat21 ax = _ab_filter.x();
    return nlohmann::json{
        {"schema_version", 1},
        {"type",           "PropulsionJet"},
        {"thrust_n",        _thrust_n},
        {"ab_active",       _ab_active},
        {"spool_state",    {sx(0, 0), sx(1, 0)}},
        {"ab_state",       {ax(0, 0), ax(1, 0)}},
    };
}

void PropulsionJet::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("PropulsionJet::deserializeJson: unsupported schema_version");
    if (j.at("type").get<std::string>() != "PropulsionJet")
        throw std::runtime_error("PropulsionJet::deserializeJson: type mismatch");

    _ab_active = j.at("ab_active").get<bool>();

    Mat21 sx;
    sx(0, 0) = j.at("spool_state")[0].get<float>();
    sx(1, 0) = j.at("spool_state")[1].get<float>();
    _spool_filter.valLimit.disableUpper();
    _spool_filter.resetState(sx);
    _spool_filter.valLimit.enableUpper();  // upper will be updated on next step

    Mat21 ax;
    ax(0, 0) = j.at("ab_state")[0].get<float>();
    ax(1, 0) = j.at("ab_state")[1].get<float>();
    _ab_filter.valLimit.disableUpper();
    _ab_filter.resetState(ax);
    _ab_filter.valLimit.enableUpper();

    _thrust_n = j.at("thrust_n").get<float>();
}

std::vector<uint8_t> PropulsionJet::serializeProto() const {
    las_proto::PropulsionJetState proto;
    proto.set_schema_version(1);
    proto.set_thrust_n(_thrust_n);
    proto.set_ab_active(_ab_active);
    const Mat21 sx = _spool_filter.x();
    proto.add_spool_state(sx(0, 0));
    proto.add_spool_state(sx(1, 0));
    const Mat21 ax = _ab_filter.x();
    proto.add_ab_state(ax(0, 0));
    proto.add_ab_state(ax(1, 0));
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void PropulsionJet::deserializeProto(const std::vector<uint8_t>& b) {
    las_proto::PropulsionJetState proto;
    if (!proto.ParseFromArray(b.data(), static_cast<int>(b.size())))
        throw std::runtime_error("PropulsionJet::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("PropulsionJet::deserializeProto: unsupported schema_version");

    _ab_active = proto.ab_active();

    Mat21 sx;
    sx(0, 0) = proto.spool_state_size() > 0 ? proto.spool_state(0) : 0.f;
    sx(1, 0) = proto.spool_state_size() > 1 ? proto.spool_state(1) : 0.f;
    _spool_filter.valLimit.disableUpper();
    _spool_filter.resetState(sx);
    _spool_filter.valLimit.enableUpper();

    Mat21 ax;
    ax(0, 0) = proto.ab_state_size() > 0 ? proto.ab_state(0) : 0.f;
    ax(1, 0) = proto.ab_state_size() > 1 ? proto.ab_state(1) : 0.f;
    _ab_filter.valLimit.disableUpper();
    _ab_filter.resetState(ax);
    _ab_filter.valLimit.enableUpper();

    _thrust_n = proto.thrust_n();
}

} // namespace liteaerosim::propulsion
