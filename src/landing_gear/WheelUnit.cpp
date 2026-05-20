#include "landing_gear/WheelUnit.hpp"
#include "liteaerosim.pb.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Pacejka magic formula: F(s) = D * sin(C * atan(Bs - E*(Bs - atan(Bs))))
// ---------------------------------------------------------------------------
static float pacejka(float s, float B, float C, float D, float E) {
    const float Bs      = B * s;
    const float atan_Bs = std::atan(Bs);
    return D * std::sin(C * std::atan(Bs - E * (Bs - atan_Bs)));
}

// ---------------------------------------------------------------------------

void WheelUnit::initialize(const WheelUnitParams& params) {
    _params                    = params;
    _strut_deflection_m        = 0.0f;
    _strut_deflection_rate_mps = 0.0f;
    _wheel_speed_rps           = 0.0f;
}

void WheelUnit::reset() {
    _strut_deflection_m        = 0.0f;
    _strut_deflection_rate_mps = 0.0f;
    _wheel_speed_rps           = 0.0f;
}

// ---------------------------------------------------------------------------

WheelContactForces WheelUnit::step(float                         penetration_m,
                                    const Eigen::Vector3f&        contact_point_body_m,
                                    const Eigen::Vector3f&        contact_vel_body_mps,
                                    const Eigen::Vector3f&        surface_normal_body,
                                    float                         wheel_angle_rad,
                                    float                         brake_demand_nd,
                                    float                         friction_mu_nd,
                                    float                         dt_s) {
    if (penetration_m <= 0.0f) {
        _strut_deflection_m        = 0.0f;
        _strut_deflection_rate_mps = 0.0f;
        return {};
    }

    // 1. Quasi-static strut deflection.
    // delta = geometric penetration (vertical, in the terrain-normal direction).
    // delta_dot = rate of penetration increase = contact-patch velocity projected
    // onto the inward terrain normal (-surface_normal_body points into the terrain).
    // Using the surface normal rather than the travel axis eliminates the phantom
    // V_N·sin(α) term that appears when the aircraft is pitched: body-Z velocity
    // has a forward-flight component when α ≠ 0, but the terrain penetration only
    // changes due to the vertical (terrain-normal) velocity component.
    // For a level aircraft (α = 0) surface_normal_body = -travel_axis_body, so
    // this is identical to the previous formula.
    const float delta_new = std::clamp(penetration_m, 0.0f, _params.travel_max_m);
    const float delta_dot = -contact_vel_body_mps.dot(surface_normal_body);
    _strut_deflection_rate_mps = delta_dot;
    _strut_deflection_m        = delta_new;

    // 2. Normal (strut) force
    //    Nonlinear spring:   F_spring = k * δ * (1 + nl * (δ / δ_max)²)
    //    Asymmetric damping: viscous (linear) + orifice (quadratic, dominant at high speed)
    //      F_damp = b * δ_dot + c * |δ_dot| * δ_dot
    //    Compression (δ_dot ≥ 0): high b_c, high c_c — orifice nearly closed
    //    Extension   (δ_dot < 0): low  b_e, low  c_e — orifice open
    const float ratio    = ((_params.travel_max_m > 0.0f)
                               ? delta_new / _params.travel_max_m : 0.0f);
    const float F_spring = _params.spring_stiffness_npm * delta_new
                         * (1.0f + _params.spring_nonlinearity_nd * ratio * ratio)
                         + _params.preload_n;
    const bool  compress = (delta_dot >= 0.0f);
    const float b        = compress ? _params.damping_compression_nspm
                                    : _params.damping_extension_nspm;
    const float c        = compress ? _params.orifice_damping_compression_ns2pm2
                                    : _params.orifice_damping_extension_ns2pm2;
    const float F_damp   = b * delta_dot + c * std::abs(delta_dot) * delta_dot;
    const float F_z      = std::max(0.0f, F_spring + F_damp);

    // 3. Wheel heading in body frame
    //    Forward = body-x projected onto the ground plane, then rotated by steering angle.
    Eigen::Vector3f wheel_fwd{1.0f, 0.0f, 0.0f};
    wheel_fwd -= wheel_fwd.dot(surface_normal_body) * surface_normal_body;
    if (wheel_fwd.squaredNorm() < 1e-6f) {
        // Degenerate (aircraft near-vertical): fall back to body-y projected
        Eigen::Vector3f body_y{0.0f, 1.0f, 0.0f};
        wheel_fwd = body_y - body_y.dot(surface_normal_body) * surface_normal_body;
    }
    wheel_fwd.normalize();

    if (_params.is_steerable && std::abs(wheel_angle_rad) > 1e-6f) {
        const Eigen::AngleAxisf steer{wheel_angle_rad, surface_normal_body};
        wheel_fwd = steer * wheel_fwd;
    }

    // Right = forward × normal  (right-hand: fwd × up = right)
    const Eigen::Vector3f wheel_right = wheel_fwd.cross(surface_normal_body).normalized();

    // 4. Contact-patch velocity components
    constexpr float kVeps = 0.01f;  // m/s regularization
    const float V_cx = contact_vel_body_mps.dot(wheel_fwd);
    const float V_cy = contact_vel_body_mps.dot(wheel_right);

    // 5. Slip ratio (longitudinal)
    //    Reference speed: max(|V_cx|, |ω·r|) + ε so kappa stays in [-1, 1] when
    //    either speed approaches zero.  Using |V_cx| alone blows kappa up to
    //    thousands near standstill (V_cx → 0, ω > 0), creating a fictitious
    //    traction spike that injects energy and permanently accelerates the aircraft.
    const float wheel_speed_mps = _params.tyre_radius_m * std::abs(_wheel_speed_rps);
    const float V_ref  = std::max(std::abs(V_cx), wheel_speed_mps) + kVeps;
    const float kappa = (_params.tyre_radius_m * _wheel_speed_rps - V_cx) / V_ref;

    // 6. Slip angle (lateral)
    const float alpha_t = -std::atan2(V_cy, std::abs(V_cx) + kVeps);

    // 7. Pacejka tyre forces  (B, C, D=mu*Fz, E from arch doc Table 3d)
    float F_x = pacejka(kappa,   10.0f, 1.9f, friction_mu_nd * F_z,  0.97f);
    float F_y = pacejka(alpha_t,  8.0f, 1.3f, friction_mu_nd * F_z, -1.0f);

    // 8. Friction-circle saturation
    const float F_total = std::sqrt(F_x * F_x + F_y * F_y);
    const float F_limit = friction_mu_nd * F_z;
    if (F_total > F_limit && F_total > 0.0f) {
        const float scale = F_limit / F_total;
        F_x *= scale;
        F_y *= scale;
    }

    // 9. Wheel speed integration (Euler)
    const float r_w           = _params.tyre_radius_m;
    const float wheel_mass_kg = 0.3f * r_w;          // empirical: m_w ≈ 0.3 * r_w
    const float I_w           = wheel_mass_kg * r_w * r_w * 0.5f;

    // Brake torque: τ_brake = C_brake * b * ω_w  (arch doc §4)
    const float tau_brake = (_params.has_brake)
                                ? _params.max_brake_torque_nm * brake_demand_nd * _wheel_speed_rps
                                : 0.0f;

    // Rolling resistance torque with 0.01 rad/s deadband
    const float omega_sign = (_wheel_speed_rps >  0.01f) ?  1.0f
                           : (_wheel_speed_rps < -0.01f) ? -1.0f
                           :                               0.0f;
    const float tau_roll = _params.rolling_resistance_nd * r_w * F_z * omega_sign;

    if (I_w > 0.0f) {
        const float omega_dot = (-r_w * F_x - tau_brake - tau_roll) / I_w;
        const float omega_new = _wheel_speed_rps + omega_dot * dt_s;
        // Clamp: if the Euler step would cross the rolling condition in one substep,
        // stop at rolling speed.  The Pacejka stiffness (B = 10) makes explicit
        // Euler unstable at practical substep sizes — the wheel hunts across kappa = 0
        // every substep, creating a limit cycle that injects fictitious traction energy.
        const float omega_roll = V_cx / r_w;
        if ((_wheel_speed_rps - omega_roll) * (omega_new - omega_roll) < 0.f)
            _wheel_speed_rps = omega_roll;
        else
            _wheel_speed_rps = omega_new;
    }

    // 10. Force assembly in body frame
    const Eigen::Vector3f force = F_z * surface_normal_body
                                 + F_x * wheel_fwd
                                 + F_y * wheel_right;
    WheelContactForces result;
    result.force_body_n   = force;
    result.moment_body_nm = contact_point_body_m.cross(force);
    result.in_contact     = true;
    return result;
}

// ---------------------------------------------------------------------------

StrutState WheelUnit::strutState() const {
    return {_strut_deflection_m, _strut_deflection_rate_mps, _wheel_speed_rps};
}

void WheelUnit::setStrutState(const StrutState& s) {
    _strut_deflection_m        = s.strut_deflection_m;
    _strut_deflection_rate_mps = s.strut_deflection_rate_mps;
    _wheel_speed_rps           = s.wheel_speed_rps;
}

// ---------------------------------------------------------------------------

nlohmann::json WheelUnit::serializeJson() const {
    return {
        {"strut_deflection_m",        _strut_deflection_m},
        {"strut_deflection_rate_mps", _strut_deflection_rate_mps},
        {"wheel_speed_rps",           _wheel_speed_rps},
    };
}

void WheelUnit::deserializeJson(const nlohmann::json& j) {
    _strut_deflection_m        = j.at("strut_deflection_m").get<float>();
    _strut_deflection_rate_mps = j.at("strut_deflection_rate_mps").get<float>();
    _wheel_speed_rps           = j.at("wheel_speed_rps").get<float>();
}

std::vector<uint8_t> WheelUnit::serializeProto() const {
    las_proto::WheelUnitState proto;
    proto.set_strut_deflection_m(_strut_deflection_m);
    proto.set_strut_deflection_rate_mps(_strut_deflection_rate_mps);
    proto.set_wheel_speed_rps(_wheel_speed_rps);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void WheelUnit::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::WheelUnitState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("WheelUnit::deserializeProto: failed to parse");
    _strut_deflection_m        = proto.strut_deflection_m();
    _strut_deflection_rate_mps = proto.strut_deflection_rate_mps();
    _wheel_speed_rps           = proto.wheel_speed_rps();
}

}  // namespace liteaero::simulation
