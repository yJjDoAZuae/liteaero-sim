#include "Aircraft.hpp"
#include "liteaerosim.pb.h"
#include "navigation/WGS84.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionProp.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace liteaerosim {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Aircraft::Aircraft(std::unique_ptr<propulsion::Propulsion> propulsion)
    : _propulsion(std::move(propulsion)) {}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

void Aircraft::initialize(const nlohmann::json& config) {
    // 1. Inertia — read directly from config section (no schema_version in aircraft_config_v1)
    const auto& in_sec = config.at("inertia");
    _inertia.mass_kg   = in_sec.at("mass_kg").get<float>();
    _inertia.Ixx_kgm2  = in_sec.at("Ixx_kgm2").get<float>();
    _inertia.Iyy_kgm2  = in_sec.at("Iyy_kgm2").get<float>();
    _inertia.Izz_kgm2  = in_sec.at("Izz_kgm2").get<float>();

    // 2. Airframe performance — same pattern
    const auto& af_sec    = config.at("airframe");
    _airframe.g_max_nd    = af_sec.at("g_max_nd").get<float>();
    _airframe.g_min_nd    = af_sec.at("g_min_nd").get<float>();
    _airframe.tas_max_mps = af_sec.at("tas_max_mps").get<float>();
    _airframe.mach_max_nd = af_sec.at("mach_max_nd").get<float>();

    // 3. Lift curve
    const auto& lc = config.at("lift_curve");
    LiftCurveParams lcp{};
    lcp.cl_alpha              = lc.at("cl_alpha").get<float>();
    lcp.cl_max                = lc.at("cl_max").get<float>();
    lcp.cl_min                = lc.at("cl_min").get<float>();
    lcp.delta_alpha_stall     = lc.at("delta_alpha_stall").get<float>();
    lcp.delta_alpha_stall_neg = lc.at("delta_alpha_stall_neg").get<float>();
    lcp.cl_sep                = lc.at("cl_sep").get<float>();
    lcp.cl_sep_neg            = lc.at("cl_sep_neg").get<float>();
    _liftCurve.emplace(lcp);

    // 4. Aerodynamic performance
    const auto& ac = config.at("aircraft");
    aerodynamics::AeroPerformanceConfig aero_cfg;
    aero_cfg.s_ref_m2  = ac.at("S_ref_m2").get<float>();
    aero_cfg.ar        = ac.at("ar").get<float>();
    aero_cfg.e         = ac.at("e").get<float>();
    aero_cfg.cd0       = ac.at("cd0").get<float>();
    aero_cfg.cl_y_beta = ac.at("cl_y_beta").get<float>();
    const float S_ref_m2  = aero_cfg.s_ref_m2;
    const float cl_y_beta = aero_cfg.cl_y_beta;
    _aeroPerf.emplace(aero_cfg);

    // 5. Load factor allocator (references _liftCurve — must be emplaced after step 3)
    _allocator.emplace(*_liftCurve, S_ref_m2, cl_y_beta);

    // 6. Initial kinematic state from initial_state section
    const auto& is = config.at("initial_state");
    WGS84_Datum datum;
    datum.setLatitudeGeodetic_rad(is.at("latitude_rad").get<double>());
    datum.setLongitude_rad(is.at("longitude_rad").get<double>());
    datum.setHeight_WGS84_m(is.at("altitude_m").get<float>());

    const Eigen::Vector3f vel_NED{
        is.at("velocity_north_mps").get<float>(),
        is.at("velocity_east_mps").get<float>(),
        is.at("velocity_down_mps").get<float>()
    };

    _state = KinematicState(
        0.0,
        datum,
        vel_NED,
        Eigen::Vector3f::Zero(),          // acceleration_NED_mps
        Eigen::Quaternionf::Identity(),   // q_nb — level, heading north
        Eigen::Vector3f::Zero()           // rates_Body_rps
    );
    _initial_state = _state;
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void Aircraft::reset() {
    _state = _initial_state;
    _allocator->reset();
    _propulsion->reset();
}

// ---------------------------------------------------------------------------
// step()
// ---------------------------------------------------------------------------

void Aircraft::step(double time_sec,
                    const AircraftCommand& cmd,
                    const Eigen::Vector3f& wind_NED_mps,
                    float rho_kgm3) {
    // 1. True airspeed
    const float V_air = (_state.velocity_NED_mps() - wind_NED_mps).norm();

    // 2. Dynamic pressure
    const float q_inf = 0.5f * rho_kgm3 * V_air * V_air;

    // 3. Clamp load factors to airframe structural limits
    const float n_cmd   = std::clamp(cmd.n,   _airframe.g_min_nd, _airframe.g_max_nd);
    const float n_y_cmd = std::clamp(cmd.n_y, _airframe.g_min_nd, _airframe.g_max_nd);

    // 4. Solve for α and β
    LoadFactorInputs lfa_in;
    lfa_in.n        = n_cmd;
    lfa_in.n_y      = n_y_cmd;
    lfa_in.q_inf    = q_inf;
    lfa_in.thrust_n = _propulsion->thrust_n();   // previous-step thrust (0 on first call)
    lfa_in.mass_kg  = _inertia.mass_kg;
    lfa_in.n_dot    = cmd.n_dot;
    lfa_in.n_y_dot  = cmd.n_y_dot;
    const LoadFactorOutputs lfa_out = _allocator->solve(lfa_in);

    // 5. Lift coefficient
    const float cl = _liftCurve->evaluate(lfa_out.alpha_rad);

    // 6. Aerodynamic forces in Wind frame
    const aerodynamics::AeroForces F =
        _aeroPerf->compute(lfa_out.alpha_rad, lfa_out.beta_rad, q_inf, cl);

    // 7. Advance propulsion
    const float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3);

    // 8. Wind-frame acceleration.
    //    Thrust decomposition (Wind frame, X forward, Y right, Z down):
    //      Tx =  T·cos(α)·cos(β)
    //      Ty = -T·cos(α)·sin(β)
    //      Tz = -T·sin(α)
    //    Gravity is embedded in the load-factor constraint — must not be added here.
    const float m  = _inertia.mass_kg;
    const float ca = std::cos(lfa_out.alpha_rad);
    const float sa = std::sin(lfa_out.alpha_rad);
    const float cb = std::cos(lfa_out.beta_rad);
    const float sb = std::sin(lfa_out.beta_rad);

    const float ax = (T * ca * cb + F.x_n) / m;
    const float ay = (-T * ca * sb + F.y_n) / m;
    const float az = (-T * sa      + F.z_n) / m;

    // 9. Advance kinematic state
    _state.step(time_sec,
                Eigen::Vector3f{ax, ay, az},
                cmd.rollRate_Wind_rps,
                lfa_out.alpha_rad,
                lfa_out.beta_rad,
                lfa_out.alphaDot_rps,
                lfa_out.betaDot_rps,
                wind_NED_mps);
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

nlohmann::json Aircraft::serializeJson() const {
    nlohmann::json j;
    j["schema_version"]  = 1;
    j["type"]            = "Aircraft";
    j["kinematic_state"] = _state.serializeJson();
    j["initial_state"]   = _initial_state.serializeJson();
    j["airframe"]        = _airframe.serializeJson();
    j["inertia"]         = _inertia.serializeJson();
    if (_liftCurve)  j["lift_curve"]      = _liftCurve->serializeJson();
    if (_aeroPerf)   j["aero_performance"] = _aeroPerf->serializeJson();
    if (_allocator)  j["allocator"]        = _allocator->serializeJson();
    if (_propulsion) j["propulsion"]       = _propulsion->serializeJson();
    return j;
}

void Aircraft::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("Aircraft::deserializeJson: unsupported schema_version");
    if (j.at("type").get<std::string>() != "Aircraft")
        throw std::runtime_error("Aircraft::deserializeJson: unexpected type");

    _airframe = AirframePerformance::deserializeJson(j.at("airframe"));
    _inertia  = Inertia::deserializeJson(j.at("inertia"));

    _liftCurve.emplace(LiftCurveModel::deserializeJson(j.at("lift_curve")));
    _aeroPerf.emplace(aerodynamics::AeroPerformance::deserializeJson(j.at("aero_performance")));

    // Emplace allocator with placeholder config — deserializeJson overwrites _S and _cl_y_beta.
    // The reference to _liftCurve is set at construction and is the only field not restored.
    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeJson(j.at("allocator"));

    _state.deserializeJson(j.at("kinematic_state"));
    _initial_state.deserializeJson(j.at("initial_state"));

    if (_propulsion) {
        _propulsion->deserializeJson(j.at("propulsion"));
    }
}

// Helper: parse bytes into a proto sub-message of type T.
template <typename T>
static T parseSubMessage(const std::vector<uint8_t>& bytes) {
    T msg;
    msg.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()));
    return msg;
}

// Helper: serialize a proto sub-message to bytes.
template <typename T>
static std::vector<uint8_t> serializeSubMessage(const T& msg) {
    const std::string s = msg.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

std::vector<uint8_t> Aircraft::serializeProto() const {
    las_proto::AircraftState proto;
    proto.set_schema_version(1);

    *proto.mutable_kinematic_state()  = parseSubMessage<las_proto::KinematicState>(_state.serializeProto());
    *proto.mutable_initial_state()    = parseSubMessage<las_proto::KinematicState>(_initial_state.serializeProto());
    if (_allocator) *proto.mutable_allocator()         = parseSubMessage<las_proto::LoadFactorAllocatorState>(_allocator->serializeProto());
    if (_liftCurve) *proto.mutable_lift_curve()        = parseSubMessage<las_proto::LiftCurveParams>(_liftCurve->serializeProto());
    if (_aeroPerf)  *proto.mutable_aero_performance()  = parseSubMessage<las_proto::AeroPerformanceParams>(_aeroPerf->serializeProto());
    *proto.mutable_airframe() = parseSubMessage<las_proto::AirframePerformanceParams>(_airframe.serializeProto());
    *proto.mutable_inertia()  = parseSubMessage<las_proto::InertiaParams>(_inertia.serializeProto());

    if (_propulsion) {
        if (auto* jet = dynamic_cast<propulsion::PropulsionJet*>(_propulsion.get()))
            *proto.mutable_jet()  = parseSubMessage<las_proto::PropulsionJetState>(jet->serializeProto());
        else if (auto* edf = dynamic_cast<propulsion::PropulsionEDF*>(_propulsion.get()))
            *proto.mutable_edf()  = parseSubMessage<las_proto::PropulsionEdfState>(edf->serializeProto());
        else if (auto* prop = dynamic_cast<propulsion::PropulsionProp*>(_propulsion.get()))
            *proto.mutable_prop() = parseSubMessage<las_proto::PropulsionPropState>(prop->serializeProto());
    }

    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void Aircraft::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AircraftState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("Aircraft::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("Aircraft::deserializeProto: unsupported schema_version");

    _airframe = AirframePerformance::deserializeProto(serializeSubMessage(proto.airframe()));
    _inertia  = Inertia::deserializeProto(serializeSubMessage(proto.inertia()));

    _liftCurve.emplace(LiftCurveModel::deserializeProto(serializeSubMessage(proto.lift_curve())));
    _aeroPerf.emplace(aerodynamics::AeroPerformance::deserializeProto(serializeSubMessage(proto.aero_performance())));

    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeProto(serializeSubMessage(proto.allocator()));

    _state.deserializeProto(serializeSubMessage(proto.kinematic_state()));
    _initial_state.deserializeProto(serializeSubMessage(proto.initial_state()));

    if (_propulsion) {
        switch (proto.propulsion_case()) {
            case las_proto::AircraftState::kJet:
                _propulsion->deserializeProto(serializeSubMessage(proto.jet()));
                break;
            case las_proto::AircraftState::kEdf:
                _propulsion->deserializeProto(serializeSubMessage(proto.edf()));
                break;
            case las_proto::AircraftState::kProp:
                _propulsion->deserializeProto(serializeSubMessage(proto.prop()));
                break;
            default:
                break;
        }
    }
}

} // namespace liteaerosim
