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

    // Bearing drag coefficients — Coulomb + viscous (OQ-LG-6).
    // Equal-contribution at omega_ref: c_f = c_v * omega_ref.
    // Settling condition omega_w(T_sd) = 0 from omega_ref → c_v = I_w * ln2 / T_sd.
    // I_w = 0.15 * r_w^3 (empirical: m_w = 0.3*r_w, I_w = m_w*r_w^2/2)
    const float r_w       = params.tyre_radius_m;
    const float I_w       = 0.15f * r_w * r_w * r_w;
    const float T_sd      = params.spindown_time_s;
    const float omega_ref = (r_w > 0.0f) ? params.spindown_reference_speed_mps / r_w : 0.0f;
    if (omega_ref > 0.0f && T_sd > 0.0f && I_w > 0.0f) {
        _cv = I_w * std::log(2.0f) / T_sd;
        _cf = _cv * omega_ref;
    } else {
        _cf = 0.0f;
        _cv = 0.0f;
    }
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
        // Airborne bearing drag — Coulomb + viscous, Tustin predictor-corrector (OQ-LG-6).
        // Wheels only spin forward (omega >= 0); Coulomb term is always positive.
        const float r_w_ab  = _params.tyre_radius_m;
        const float I_w_ab  = 0.15f * r_w_ab * r_w_ab * r_w_ab;
        const float omega_k = _wheel_speed_rps;
        if (I_w_ab > 0.0f && (_cf != 0.0f || _cv != 0.0f) && omega_k > 0.0f) {
            const float drag_k    = _cf + _cv * omega_k;
            const float odot_k    = -drag_k / I_w_ab;
            const float omega_star = omega_k + odot_k * dt_s;
            if (omega_star <= 0.0f) {
                _wheel_speed_rps = 0.0f;
            } else {
                const float drag_star = _cf + _cv * omega_star;
                const float odot_star = -drag_star / I_w_ab;
                _wheel_speed_rps = omega_k + 0.5f * dt_s * (odot_k + odot_star);
            }
        }
        _diag = ContactDiag{};   // clear the contact-force breakdown when airborne
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
    const float r_w           = _params.tyre_radius_m;

    // Quasi-static omega for free-rolling: I_w = 0.15*r_w³ ≈ 0.0012 kg·m² gives a
    // Pacejka time constant of ~9 µs, three orders of magnitude shorter than dt = 20 ms.
    // When no brake torque is applied the wheel must be at pure rolling (omega = V_cx/r_w),
    // which makes kappa=0, F_x=0, and only F_rr decelerates the aircraft.
    //
    // The wheel must be allowed to spin in the SAME direction as the contact patch,
    // INCLUDING backward (omega < 0 when V_cx < 0).  A contact patch can move aft
    // transiently under any pitch rotation of a lever-arm wheel; clamping omega to >= 0
    // there would model the free wheel as LOCKED against a rearward-moving patch, giving
    // kappa = -V_cx/V_ref ≈ +1 and a large fictitious FORWARD traction (the tire would
    // "propel" the aircraft — non-physical). Matching omega = V_cx/r_w keeps kappa = 0 for
    // either sign, so a free wheel never produces longitudinal slip force.
    if (!_params.has_brake || brake_demand_nd < 1e-4f)
        _wheel_speed_rps = V_cx / r_w;

    const float wheel_speed_mps = r_w * std::abs(_wheel_speed_rps);
    const float V_ref  = std::max(std::abs(V_cx), wheel_speed_mps) + kVeps;
    const float kappa = (r_w * _wheel_speed_rps - V_cx) / V_ref;

    // 6. Slip angle (lateral)
    const float alpha_t = -std::atan2(V_cy, std::abs(V_cx) + kVeps);

    // 7. Pacejka tyre forces  (B, C, D=mu*Fz, E from arch doc Table 3d)
    // A magic castering (nose) wheel produces NO side force (F_y = 0) — it quasi-statically
    // aligns to its ground-relative velocity; directional control is the FBW yaw model, not
    // nosewheel cornering (landing_gear.md §3, OQ-BC-12 Alt B).
    float F_x = pacejka(kappa,   10.0f, 1.9f, friction_mu_nd * F_z,  0.97f);
    float F_y = _params.is_castering
                    ? 0.0f
                    : pacejka(alpha_t, 8.0f, 1.3f, friction_mu_nd * F_z, -1.0f);

    // 8. Friction-circle saturation
    const float F_total = std::sqrt(F_x * F_x + F_y * F_y);
    const float F_limit = friction_mu_nd * F_z;
    if (F_total > F_limit && F_total > 0.0f) {
        const float scale = F_limit / F_total;
        F_x *= scale;
        F_y *= scale;
    }

    // 8b. Low-speed lateral fade. The Pacejka side force is a sliding-friction force computed from the
    // slip ANGLE alpha_t = atan2(V_cy, |V_cx|); as the contact speed → 0 this becomes ill-conditioned —
    // a tiny residual V_cy at (near-)zero forward speed reads as a large slip angle, so a stationary
    // wheel would develop a large spurious lateral force. Fade F_y out below a low-speed scale so a
    // wheel at rest carries no slip-based side force (kVlat ≈ 0.5 m/s; full force by ~1 m/s, so rolling
    // and ground-steering behavior above walking pace is unaffected). F_x needs no such fade — a free
    // wheel has kappa = 0 (hence F_x = 0) at rest by construction.
    {
        constexpr float kVlat = 0.5f;   // m/s lateral-force low-speed fade scale
        const float V_c2 = V_cx * V_cx + V_cy * V_cy;
        F_y *= V_c2 / (V_c2 + kVlat * kVlat);
    }

    // 9. Wheel speed integration — Tustin predictor-corrector (OQ-LG-5)
    // I_w = m_w * r_w^2 / 2,  m_w ≈ 0.3 * r_w  →  I_w = 0.15 * r_w^3
    //
    // Rolling resistance is applied as a direct longitudinal force (step 10 below),
    // NOT as a torque here.  The wheel inertia I_w ≈ 0.0012 kg·m² gives a Pacejka
    // time constant of ~9 µs — orders of magnitude shorter than any practical dt.
    // Routing rolling resistance through the wheel ODE causes the omega integrator
    // to overshoot by ~3× per step and oscillate, producing a near-zero average
    // contact force instead of the intended rolling-resistance deceleration.
    const float I_w = 0.15f * r_w * r_w * r_w;

    if (I_w > 0.0f) {
        // --- torques at ω_k (wheels only spin forward: omega >= 0) ---
        const float tau_brake_k  = (_params.has_brake)
                                 ? _params.max_brake_torque_nm * brake_demand_nd * _wheel_speed_rps
                                 : 0.0f;
        const float odot_k       = (-r_w * F_x - tau_brake_k) / I_w;

        // --- Euler predictor ---
        const float omega_star = _wheel_speed_rps + odot_k * dt_s;

        // --- F_x at ω* with friction-circle saturation ---
        const float ws_star    = r_w * std::abs(omega_star);
        const float Vref_star  = std::max(std::abs(V_cx), ws_star) + kVeps;
        const float kappa_star = (r_w * omega_star - V_cx) / Vref_star;
        float F_x_star = pacejka(kappa_star, 10.0f, 1.9f, friction_mu_nd * F_z, 0.97f);
        float F_y_star = _params.is_castering
                             ? 0.0f
                             : pacejka(alpha_t, 8.0f, 1.3f, friction_mu_nd * F_z, -1.0f);
        const float Ftot_star = std::sqrt(F_x_star * F_x_star + F_y_star * F_y_star);
        if (Ftot_star > F_limit && Ftot_star > 0.0f)
            F_x_star *= F_limit / Ftot_star;

        // --- torques at ω* ---
        const float tau_brake_star = (_params.has_brake)
                                   ? _params.max_brake_torque_nm * brake_demand_nd * omega_star
                                   : 0.0f;
        const float odot_star      = (-r_w * F_x_star - tau_brake_star) / I_w;

        // --- Tustin (trapezoidal); clamp at zero — wheels do not spin backwards ---
        _wheel_speed_rps = std::max(0.0f, _wheel_speed_rps + 0.5f * dt_s * (odot_k + odot_star));
    }

    // Rolling resistance: direct backward force proportional to F_z (tire hysteresis loss).
    // Applied here rather than through the wheel ODE because I_w << (m_aircraft * dt²),
    // making the indirect path (torque → omega → kappa → Pacejka) numerically unstable.
    const float F_rr = (_params.rolling_resistance_nd > 0.0f && std::abs(V_cx) > kVeps)
                       ? -_params.rolling_resistance_nd * F_z * std::copysign(1.0f, V_cx)
                       : 0.0f;

    // 10. Force assembly in body frame
    const Eigen::Vector3f force = F_z * surface_normal_body
                                 + (F_x + F_rr) * wheel_fwd
                                 + F_y * wheel_right;
    WheelContactForces result;
    result.force_body_n   = force;
    result.moment_body_nm = contact_point_body_m.cross(force);
    result.in_contact     = true;

    // Cache the per-wheel contact-force breakdown for the diagnostic accessor.
    _diag.F_z = F_z;  _diag.F_x = F_x;  _diag.F_y = F_y;  _diag.F_rr = F_rr;
    _diag.kappa = kappa;  _diag.alpha_t = alpha_t;
    _diag.V_cx = V_cx;  _diag.V_cy = V_cy;  _diag.omega = _wheel_speed_rps;
    _diag.delta_dot = delta_dot;
    _diag.wheel_fwd = wheel_fwd;  _diag.surf_normal = surface_normal_body;
    _diag.force_body = force;
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
