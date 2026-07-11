#include "landing_gear/LandingGear.hpp"
#include "liteaerosim.pb.h"
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <stdexcept>
#include <vector>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

void LandingGear::initialize(const nlohmann::json& config) {
    _config.substeps = config.value("substeps", 4);
    if (_config.substeps < 1)
        throw std::invalid_argument("LandingGear::initialize: substeps must be >= 1");

    _config.wheel_units.clear();
    _wheel_units.clear();

    const auto& wu_arr = config.at("wheel_units");
    for (const auto& wu_json : wu_arr) {
        WheelUnitParams p;
        const auto& ap     = wu_json.at("attach_point_body_m");
        p.attach_point_body_m = {ap.at(0).get<float>(), ap.at(1).get<float>(), ap.at(2).get<float>()};
        const auto& ta     = wu_json.at("travel_axis_body");
        p.travel_axis_body = {ta.at(0).get<float>(), ta.at(1).get<float>(), ta.at(2).get<float>()};
        p.travel_axis_body.normalize();

        p.spring_stiffness_npm            = wu_json.at("spring_stiffness_npm").get<float>();
        p.damping_compression_nspm             = wu_json.at("damping_compression_nspm").get<float>();
        p.damping_extension_nspm               = wu_json.at("damping_extension_nspm").get<float>();
        p.orifice_damping_compression_ns2pm2   = wu_json.value("orifice_damping_compression_ns2pm2", 0.0f);
        p.orifice_damping_extension_ns2pm2     = wu_json.value("orifice_damping_extension_ns2pm2", 0.0f);
        p.spring_nonlinearity_nd               = wu_json.value("spring_nonlinearity_nd", 0.0f);
        p.preload_n                       = wu_json.value("preload_n", 0.0f);
        p.travel_max_m                    = wu_json.at("travel_max_m").get<float>();
        p.tyre_radius_m                   = wu_json.at("tyre_radius_m").get<float>();
        p.tyre_cornering_stiffness_npm    = wu_json.value("tyre_cornering_stiffness_npm", 0.0f);
        p.tyre_longitudinal_stiffness_npm = wu_json.value("tyre_longitudinal_stiffness_npm", 0.0f);
        p.rolling_resistance_nd           = wu_json.value("rolling_resistance_nd", 0.02f);
        p.max_brake_torque_nm             = wu_json.value("max_brake_torque_nm", 0.0f);
        p.is_steerable                    = wu_json.value("is_steerable", false);
        p.is_castering                    = wu_json.value("is_castering", false);
        p.has_brake                       = wu_json.value("has_brake", false);
        p.spindown_time_s                 = wu_json.value("spindown_time_s", 5.0f);
        p.spindown_reference_speed_mps    = wu_json.value("spindown_reference_speed_mps", 20.0f);

        _config.wheel_units.push_back(p);
        WheelUnit wu;
        wu.initialize(p);
        _wheel_units.push_back(std::move(wu));
    }

    // Worst-case reach: for any orientation, no contact point can be further
    // below the CG than attach_point.norm() + travel_max + tyre_radius.
    _max_reach_m = 0.f;
    for (const auto& p : _config.wheel_units) {
        const float reach = p.attach_point_body_m.norm() + p.travel_max_m + p.tyre_radius_m;
        _max_reach_m = std::max(_max_reach_m, reach);
    }

    _contact_forces = ContactForces{};
    _initialized    = true;
}

// ---------------------------------------------------------------------------

void LandingGear::reset() {
    for (auto& wu : _wheel_units) wu.reset();
    _contact_forces = ContactForces{};
}

// ---------------------------------------------------------------------------

ContactForces LandingGear::step(const liteaero::nav::KinematicStateSnapshot& snap,
                                 const liteaero::terrain::Terrain&            terrain,
                                 float nose_wheel_angle_rad,
                                 float brake_left_nd,
                                 float brake_right_nd,
                                 float outer_dt_s) {
    // Body-to-NED rotation matrix and its transpose (NED-to-body)
    const Eigen::Matrix3f R_nb = liteaero::nav::KinematicStateUtil::q_nb(snap).toRotationMatrix();
    const Eigen::Matrix3f R_bn = R_nb.transpose();

    // Surface normal in NED: pointing UP = [0, 0, -1] (NED z is down)
    const Eigen::Vector3f surface_normal_NED{0.f, 0.f, -1.f};
    const Eigen::Vector3f surface_normal_body = (R_bn * surface_normal_NED).normalized();

    // Aircraft state
    const float           h_ac   = snap.position.altitude_m;
    const Eigen::Vector3f v_body = R_bn * snap.velocity_ned_mps;
    const Eigen::Vector3f omega  = snap.rates_body_rps;

    // Single terrain query at CG; also used for AGL early-exit.
    const float terrain_h = terrain.elevation_m(snap.position.latitude_rad,
                                                 snap.position.longitude_rad);

    if (h_ac - terrain_h > _max_reach_m) {
        for (auto& wu : _wheel_units) wu.reset();
        _contact_forces = ContactForces{};
        return _contact_forces;
    }

    const float inner_dt_s = outer_dt_s / static_cast<float>(_config.substeps);
    const int   n          = static_cast<int>(_wheel_units.size());

    // Precompute per-wheel contact geometry at the undeflected tyre position.
    // Penetration = how deep the undeflected contact patch is into the terrain.
    // WheelUnit::step() clamps this to [0, travel_max] — no δ_prev term here.
    std::vector<float>           penetrations(n);
    std::vector<Eigen::Vector3f> contact_points(n);
    std::vector<Eigen::Vector3f> contact_vels(n);
    std::vector<float>           friction_mu(n);

    for (int i = 0; i < n; ++i) {
        const WheelUnitParams& p  = _config.wheel_units[i];

        // Contact point in body frame (undeflected strut position)
        const Eigen::Vector3f c_body = p.attach_point_body_m
                                     - p.tyre_radius_m * surface_normal_body;
        contact_points[i] = c_body;

        // Altitude of contact point: h_ac minus the NED-z component of the body offset
        const Eigen::Vector3f c_ned_offset = R_nb * c_body;
        const float h_contact = h_ac - c_ned_offset.z();

        penetrations[i] = terrain_h - h_contact;

        // Contact-patch velocity in body frame (includes rigid-body rotation)
        contact_vels[i] = v_body + omega.cross(c_body);

        // Friction: use longitudinal peak of the surface model
        friction_mu[i] = _surface_friction
                             .frictionCoefficients(Eigen::Vector3f::Zero())
                             .longitudinal_peak_nd;
    }

    // Fast-path: all wheels airborne, stopped, and struts neutral — nothing to integrate.
    // Strut check covers the first airborne step after liftoff (strut still deflected).
    {
        bool skip = true;
        for (int i = 0; i < n; ++i) {
            const StrutState ss = _wheel_units[i].strutState();
            if (penetrations[i] > 0.0f || ss.wheel_speed_rps > 0.0f
                    || ss.strut_deflection_m > 0.0f) {
                skip = false;
                break;
            }
        }
        if (skip) {
            _contact_forces = ContactForces{};
            return _contact_forces;
        }
    }

    // Inner substep loop — spring-damper + Pacejka run N_sub times at inner_dt_s
    std::vector<WheelContactForces> last_wcf(n);
    for (int sub = 0; sub < _config.substeps; ++sub) {
        for (int i = 0; i < n; ++i) {
            const WheelUnitParams& p = _config.wheel_units[i];

            // Assign brake demand by lateral (y) position: negative y = left side
            float brake = 0.0f;
            if (p.has_brake) {
                brake = (contact_points[i].y() <= 0.0f) ? brake_left_nd : brake_right_nd;
            }
            const float steer = p.is_steerable ? nose_wheel_angle_rad : 0.0f;

            last_wcf[i] = _wheel_units[i].step(
                penetrations[i],
                contact_points[i],
                contact_vels[i],
                surface_normal_body,
                steer,
                brake,
                friction_mu[i],
                inner_dt_s);
        }
    }

    // Accumulate total contact forces and moments
    ContactForces total;
    for (int i = 0; i < n; ++i) {
        total.force_body_n   += last_wcf[i].force_body_n;
        total.moment_body_nm += last_wcf[i].moment_body_nm;
        if (last_wcf[i].in_contact) total.weight_on_wheels = true;
    }
    _contact_forces = total;
    return total;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

nlohmann::json LandingGear::serializeJson() const {
    nlohmann::json j;
    j["schema_version"] = 1;
    j["substeps"]       = _config.substeps;
    nlohmann::json wu_arr = nlohmann::json::array();
    for (const auto& wu : _wheel_units) {
        wu_arr.push_back(wu.serializeJson());
    }
    j["wheel_units"] = wu_arr;
    return j;
}

void LandingGear::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("LandingGear::deserializeJson: unsupported schema_version");
    const auto& wu_arr = j.at("wheel_units");
    if (wu_arr.size() != _wheel_units.size())
        throw std::runtime_error("LandingGear::deserializeJson: wheel_units count mismatch");
    for (std::size_t i = 0; i < _wheel_units.size(); ++i) {
        _wheel_units[i].deserializeJson(wu_arr[i]);
    }
}

std::vector<uint8_t> LandingGear::serializeProto() const {
    las_proto::LandingGearState proto;
    proto.set_schema_version(1);
    for (const auto& wu : _wheel_units) {
        const StrutState ss = wu.strutState();
        auto* wup = proto.add_wheel_units();
        wup->set_strut_deflection_m(ss.strut_deflection_m);
        wup->set_strut_deflection_rate_mps(ss.strut_deflection_rate_mps);
        wup->set_wheel_speed_rps(ss.wheel_speed_rps);
    }
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void LandingGear::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::LandingGearState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("LandingGear::deserializeProto: failed to parse");
    if (proto.schema_version() != 1)
        throw std::runtime_error("LandingGear::deserializeProto: unsupported schema_version");
    if (static_cast<std::size_t>(proto.wheel_units_size()) != _wheel_units.size())
        throw std::runtime_error("LandingGear::deserializeProto: wheel_units count mismatch");
    for (int i = 0; i < proto.wheel_units_size(); ++i) {
        const auto& wup = proto.wheel_units(i);
        StrutState ss;
        ss.strut_deflection_m        = wup.strut_deflection_m();
        ss.strut_deflection_rate_mps = wup.strut_deflection_rate_mps();
        ss.wheel_speed_rps           = wup.wheel_speed_rps();
        _wheel_units[i].setStrutState(ss);
    }
}

}  // namespace liteaero::simulation
