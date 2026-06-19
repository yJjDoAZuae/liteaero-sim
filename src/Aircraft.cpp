#include "Aircraft.hpp"
#include <liteaero/control/filter_realizations.hpp>
#include "liteaerosim.pb.h"
#include "navigation/WGS84.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionProp.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <vector>

using liteaero::control::Mat21;

namespace liteaero::simulation {

// C² dynamic-pressure authority factor Φ(V) = smootherstep(clamp((V/V_ref)², 0, 1))
// (OQ-LG-19 force-channel fade and OQ-LG-21 attitude-reference blend). Zero (with zero 1st
// and 2nd derivatives) at V=0, unity (likewise) at V≥V_ref, monotone, no division/singularity.
static inline float phiAuthority(float V, float V_ref) {
    if (V_ref <= 0.0f) return 1.0f;
    float q = (V * V) / (V_ref * V_ref);
    q = std::clamp(q, 0.0f, 1.0f);
    return q * q * q * (q * (q * 6.0f - 15.0f) + 10.0f);   // 6q⁵ − 15q⁴ + 10q³
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Aircraft::Aircraft(std::unique_ptr<Propulsion> propulsion)
    : _propulsion(std::move(propulsion)) {}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

void Aircraft::initialize(const nlohmann::json& config, float outer_dt_s) {
    _outer_dt_s = outer_dt_s;
    // 1. Inertia — read directly from config section (no schema_version in aircraft_config_v1)
    const auto& in_sec = config.at("inertia");
    _inertia.mass_kg   = in_sec.at("mass_kg").get<float>();
    _inertia.Ixx_kgm2  = in_sec.at("Ixx_kgm2").get<float>();
    _inertia.Iyy_kgm2  = in_sec.at("Iyy_kgm2").get<float>();
    _inertia.Izz_kgm2  = in_sec.at("Izz_kgm2").get<float>();

    // 2. Airframe performance — same pattern
    const auto& af_sec      = config.at("airframe");
    _airframe.g_max_nd      = af_sec.at("g_max_nd").get<float>();
    _airframe.g_min_nd      = af_sec.at("g_min_nd").get<float>();
    _airframe.tas_max_mps   = af_sec.at("tas_max_mps").get<float>();
    _airframe.mach_max_nd   = af_sec.at("mach_max_nd").get<float>();

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
    const float alpha_max_rad = lc.at("alpha_max_rad").get<float>();
    const float alpha_min_rad = lc.at("alpha_min_rad").get<float>();

    // 4. Aerodynamic performance and command-derivative filter config
    const auto& ac = config.at("aircraft");
    AeroPerformanceConfig aero_cfg;
    aero_cfg.s_ref_m2  = ac.at("S_ref_m2").get<float>();
    aero_cfg.ar        = ac.at("ar").get<float>();
    aero_cfg.e         = ac.at("e").get<float>();
    aero_cfg.cd0       = ac.at("cd0").get<float>();
    aero_cfg.cl_y_beta = ac.at("cl_y_beta").get<float>();
    const float S_ref_m2  = aero_cfg.s_ref_m2;
    const float cl_y_beta = aero_cfg.cl_y_beta;
    _aeroPerf.emplace(aero_cfg);

    _cmd_filter_substeps  = ac.at("cmd_filter_substeps").get<int>();
    if (_cmd_filter_substeps < 1)
        throw std::invalid_argument("Aircraft::initialize: cmd_filter_substeps must be >= 1");
    _nz_wn_rad_s         = ac.at("nz_wn_rad_s").get<float>();
    _nz_zeta_nd          = ac.at("nz_zeta_nd").get<float>();
    _ny_wn_rad_s         = ac.at("ny_wn_rad_s").get<float>();
    _ny_zeta_nd          = ac.at("ny_zeta_nd").get<float>();
    _roll_rate_wn_rad_s  = ac.at("roll_rate_wn_rad_s").get<float>();
    _roll_rate_zeta_nd   = ac.at("roll_rate_zeta_nd").get<float>();
    _dtheta_zeta_nd        = ac.value("dtheta_zeta_nd", 0.7f);
    _dtheta_wn_pitch_rad_s = ac.value("dtheta_wn_pitch_rad_s", 3.0f);
    _dtheta_wn_roll_rad_s  = ac.value("dtheta_wn_roll_rad_s",  4.0f);
    _dtheta_wn_yaw_rad_s   = ac.value("dtheta_wn_yaw_rad_s",   2.0f);
    _dtheta_vref_mps       = ac.value("dtheta_vref_mps",       24.0f);
    // OQ-LG-21 attitude-reference velocity low-pass τ. Sets how heavily per-step gear-bounce
    // wobble is rejected at low speed while retaining the runway-slope/approach trend.
    _att_filt_tau_s        = ac.value("att_filt_tau_s",        0.7f);
    // H₁ load-handoff: its own (slower) parameters, NOT the FBW command filter (nz_wn).
    _nz_relax_wn_rad_s     = ac.value("nz_relax_wn_rad_s",     1.0f);
    _nz_relax_zeta_nd      = ac.value("nz_relax_zeta_nd",      0.8f);
    // Destancing LP shares the H₁ load-handoff timescale (same physical quantity: the steady
    // gear load), so τ_stance is derived, not configured.
    _dtheta_stance_tau_s   = (_nz_relax_wn_rad_s > 0.0f) ? 1.0f / _nz_relax_wn_rad_s : 1.0f;
    _cmd_filter_dt_s     = outer_dt_s / static_cast<float>(_cmd_filter_substeps);
    // Nyquist: wn * cmd_filter_dt_s must be < π for each axis.
    constexpr float kPi = 3.14159265f;
    if (_nz_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: nz_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    if (_ny_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: ny_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    if (_roll_rate_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: roll_rate_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    if (_nz_relax_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: nz_relax_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    _nz_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_wn_rad_s, _nz_zeta_nd, 0.f);
    _ny_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _ny_wn_rad_s, _ny_zeta_nd, 0.f);
    _roll_rate_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _roll_rate_wn_rad_s, _roll_rate_zeta_nd, 0.f);
    _nz_filter.resetToInput(1.f);
    // H₁: nz relaxation filter — FBW load-handoff timescale (own ωn/ζ, slower than H₂; OQ-LG-17).
    _nz_relax_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_relax_wn_rad_s, _nz_relax_zeta_nd, 0.f);
    _nz_relax_filter.resetToInput(0.f);
    // H₂: gear-MOMENT rotation-deviation filters — 2nd-order LP on M/I (finite DC = static
    // stance). ωn/ζ are physical rotational-mode parameters (config; OQ-LG-17). No fade — gear
    // torque persists at zero airspeed.
    _dtheta_pitch_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_pitch_rad_s, _dtheta_zeta_nd, 0.f);
    _dtheta_roll_filter .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_roll_rad_s,  _dtheta_zeta_nd, 0.f);
    _dtheta_yaw_filter  .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_yaw_rad_s,   _dtheta_zeta_nd, 0.f);
    _dtheta_pitch_filter.resetToInput(0.f);
    _dtheta_roll_filter .resetToInput(0.f);
    _dtheta_yaw_filter  .resetToInput(0.f);
    // Destancing low-pass for the gear vertical load (1st-order, τ = stance time constant).
    _fz_stance_filter.setLowPassFirstIIR(_cmd_filter_dt_s, _dtheta_stance_tau_s);
    _fz_stance_filter.resetToInput(0.f);
    // Force channel G(s) = -(s + 2ζωn)/(s² + 2ζωn s + ωn²), realized inline via the library's
    // general Tustin 2nd-order machinery (OQ-LG-20; no problem-specific design added to the
    // shared control library). num_s/den_s ordered [s², s¹, s⁰] per tustin_2_tf convention.
    {
        const float wn = _dtheta_wn_pitch_rad_s;
        const float z  = _dtheta_zeta_nd;
        liteaero::control::Vec3 num_s, den_s, num_z, den_z;
        num_s << 0.0f, -1.0f, -2.0f * z * wn;
        den_s << 1.0f, 2.0f * z * wn, wn * wn;
        liteaero::control::tustin_2_tf(num_s, den_s, _cmd_filter_dt_s, wn, num_z, den_z);
        liteaero::control::tf2ss(num_z, den_z, _force_phi, _force_gamma, _force_h, _force_j);
        _force_x.setZero();
    }

    // 5. Load factor allocator (references _liftCurve — must be emplaced after step 3)
    const auto& lfa_sec = config.at("load_factor_allocator");
    const float alpha_dot_max_rad_s = lfa_sec.value("alpha_dot_max_rad_s", 0.0f);
    _allocator.emplace(*_liftCurve, S_ref_m2, cl_y_beta,
                       alpha_min_rad, alpha_max_rad,
                       alpha_dot_max_rad_s);

    // 6. Initial kinematic state from initial_state section
    const auto& is = config.at("initial_state");
    WGS84_Datum datum;
    datum.setLatitudeGeodetic_rad(is.at("latitude_rad").get<double>());
    datum.setLongitude_rad(is.at("longitude_rad").get<double>());
    // initial_state.altitude_m is WGS84 ellipsoidal (OQ-LS-12 Option A).
    // SimRunner converts to MSL via the EGM2008 geoid before atmospheric
    // queries (LS-T6 / OQ-LS-14).
    datum.setHeight_WGS84_m(is.at("altitude_m").get<float>());

    const Eigen::Vector3f vel_NED{
        is.at("velocity_north_mps").get<float>(),
        is.at("velocity_east_mps").get<float>(),
        is.at("velocity_down_mps").get<float>()
    };

    // Optional Euler angles (ZYX: heading, pitch, roll) — all default to 0.
    const float heading_rad = is.value("heading_rad", 0.0f);
    const float pitch_rad   = is.value("pitch_rad",   0.0f);
    const float roll_rad    = is.value("roll_rad",    0.0f);
    const Eigen::Quaternionf q_nb =
        Eigen::AngleAxisf(heading_rad, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch_rad,   Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll_rad,    Eigen::Vector3f::UnitX());

    _state = KinematicState(
        0.0,
        datum,
        vel_NED,
        Eigen::Vector3f::Zero(),
        q_nb,
        Eigen::Vector3f::Zero()
    );
    _initial_state = _state;

    // 7. Landing gear (optional — only initialized when "landing_gear" section is present)
    if (config.contains("landing_gear")) {
        _landing_gear.initialize(config.at("landing_gear"));
        _has_landing_gear = true;
    } else {
        _has_landing_gear = false;
    }

    // 8. Body collider (optional — only initialized when "body_collider" section is present)
    if (config.contains("body_collider")) {
        _body_collider.initialize(config.at("body_collider"));
        _has_body_collider = true;
    } else {
        _has_body_collider = false;
    }
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void Aircraft::reset() {
    _state = _initial_state;
    _nz_filter.resetToInput(1.f);
    _ny_filter.resetToInput(0.f);
    _roll_rate_filter.resetToInput(0.f);
    _allocator->reset();
    _propulsion->reset();
    if (_has_landing_gear)  _landing_gear.reset();
    if (_has_body_collider) _body_collider.reset();
    _contact_forces = ContactForces{};
    _nz_relax_filter    .resetToInput(0.f);
    _dtheta_pitch_filter.resetToInput(0.f);
    _dtheta_roll_filter .resetToInput(0.f);
    _dtheta_yaw_filter  .resetToInput(0.f);
    _fz_stance_filter   .resetToInput(0.f);
    _force_x.setZero();
    _prev_dtheta_roll  = 0.f;
    _prev_dtheta_yaw   = 0.f;
    _v_filt_ned.setZero();
    _v_filt_init = false;
    _body_in_hard_contact = false;
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
    const float n_cmd   = std::clamp(cmd.n_z, _airframe.g_min_nd, _airframe.g_max_nd);
    const float n_y_cmd = std::clamp(cmd.n_y, _airframe.g_min_nd, _airframe.g_max_nd);

    // 4. Inner filter loop: 2nd-order LP command response for Nz, Ny, and roll rate.
    //    Runs _cmd_filter_substeps times at dt = outer_dt / substeps.
    float n_z_shaped       = 0.f;
    float n_y_shaped       = 0.f;
    float rollRate_filt_rps = 0.f;
    for (int i = 0; i < _cmd_filter_substeps; ++i) {
        n_z_shaped       = _nz_filter.step(n_cmd);
        n_y_shaped       = _ny_filter.step(n_y_cmd);
        rollRate_filt_rps = _roll_rate_filter.step(cmd.rollRate_Wind_rps);
    }
    // Analytical derivatives from filter state: ẏ[k] ≈ (H·(Φ·x[k]+Γ·u[k]) - y[k]) / dt_inner.
    // Uses only quantities already available after the substep loop — no additional lag.
    const float n_z_dot = (_nz_filter.h() * (_nz_filter.phi() * _nz_filter.x()
                            + _nz_filter.gamma() * n_cmd))(0, 0) / _cmd_filter_dt_s
                          - n_z_shaped / _cmd_filter_dt_s;
    const float n_y_dot = (_ny_filter.h() * (_ny_filter.phi() * _ny_filter.x()
                            + _ny_filter.gamma() * n_y_cmd))(0, 0) / _cmd_filter_dt_s
                          - n_y_shaped / _cmd_filter_dt_s;

    // Rotation matrices used both here and at step 10 (gear-to-wind transform).
    // _state is not mutated between steps 5 and 10.
    constexpr float kGravity_mps2 = 9.80665f;
    const Eigen::Matrix3f R_nb_mat = _state.q_nb().toRotationMatrix();
    const Eigen::Matrix3f R_nw_mat = _state.q_nw().toRotationMatrix();

    // 5a. Contact forces on the current pre-integration state.
    //     Computed HERE — before the LFA — so step 5b can use the same-step
    //     contact force for the n_z correction.  One-step lag (previous-step
    //     forces) caused n_z_shaped to remain at 1 on the first step of ground
    //     contact, allowing gear and full aerodynamic lift to both act
    //     simultaneously and send the aircraft airborne.  Same-step forces
    //     eliminate that lag: once the gear compresses, n_z_shaped drops to 0
    //     in the same step.
    // The gear uses the snapshot body rate directly. Since commitAttitude now derives that
    // rate from the committed (slope-aware, filtered-reference) attitude (OQ-LG-21), it is
    // consistent and free of the (v×a)/|V|² spike, so no separate gear-only override is needed.
    ContactForces contact_forces;
    if (_has_landing_gear && _terrain != nullptr) {
        contact_forces = _landing_gear.step(
            _state.snapshot(),
            *_terrain,
            0.0f,    // nose wheel steering angle (not yet wired to AircraftCommand)
            0.0f,    // brake left demand
            0.0f,    // brake right demand
            _outer_dt_s);
    }
    if (_has_body_collider && _terrain != nullptr) {
        const auto bc = _body_collider.step(_state.snapshot(), *_terrain);
        contact_forces.force_body_n   += bc.force_body_n;
        contact_forces.moment_body_nm += bc.moment_body_nm;
        contact_forces.weight_on_wheels |= bc.weight_on_wheels;
    }
    _contact_forces = contact_forces;
    // Propagate persistent body-contact flag: even when the penalty spring
    // sees pen=0 after the hard constraint corrected altitude, the aircraft is
    // still on terrain if _body_in_hard_contact is set.
    if (_body_in_hard_contact) {
        _contact_forces.weight_on_wheels = true;
    }

    // 5b. H₁: lagged n_z-command relaxation (apportionment, not suppression).
    //     The gear supplies part of the commanded load factor; H₁ (2nd-order LP,
    //     FBW Nz ωn/ζ, DC=1) smooths the gear's share and subtracts it from
    //     n_z_shaped so the wing targets only the residual not already carried by
    //     the ground. The total delivered load factor stays n_z_cmd, so full FBW
    //     command authority is preserved.
    //
    //     OQ-LG-22 (resolved, Alternative 4): the relaxation input is the ACTUAL
    //     ground reaction only, max(0, -F_z^B/(m g)). It is zero off-ground and falls
    //     continuously to zero as the gear unloads, so a go-around command above 1 g
    //     is delivered to the wing as the strut releases. The terrain hard constraint
    //     does NOT feed this: it is a deep-penetration safety net that applies no
    //     force and does not zero the strut reaction, which force_body_n already
    //     carries. (The earlier synthetic full-weight input under _body_in_hard_contact
    //     was removed — it over-credited the ground and, via a latched flag, capped a
    //     go-around at n_z_cmd-1 with zero contact force.)
    float nz_relax_dbg = 0.f;
    {
        float nz_gear_input = 0.f;
        if (contact_forces.weight_on_wheels) {
            nz_gear_input = std::max(0.f,
                -contact_forces.force_body_n.z() / (_inertia.mass_kg * kGravity_mps2));
        }
        nz_relax_dbg = _nz_relax_filter.step(nz_gear_input);
        n_z_shaped = std::max(0.0f, n_z_shaped - nz_relax_dbg);
    }

    // 5c. Δθ rotation-deviation state (OQ-LG-15/16/17/18/19/20).
    //     Two channels, both gear-derived and both with NO free integrator:
    //       Force (pitch): G(s)·u, where u = γ̇_arrest·Φ(V) is the destanced, dynamic-pressure-
    //         faded gear-induced flight-path rate. Zero in steady roll (destanced), bounded and
    //         →0 at low speed (Φ(V) fade). G(s) realized inline (OQ-LG-20).
    //       Moment (pitch/roll/yaw): H₂ low-pass on M/I (finite DC = static stance), no fade.
    //     Δθ_pitch = Δθ_force + Δθ_moment_pitch feeds α and the gear geometry.

    // Compute gear force in wind frame once; reused in step 9 to avoid double transform.
    const Eigen::Vector3f F_gear_wind =
        R_nw_mat.transpose() * (R_nb_mat * contact_forces.force_body_n);

    float dtheta_pitch = 0.f;
    float delta_rr_filt_rps  = 0.f;
    float delta_ay_filt_mps2 = 0.f;
    {
        // --- Force channel (pitch): destanced gear vertical load → faded FPA rate → G(s). ---
        // Gear contribution to NED-down acceleration (gear up → negative).
        const float fz_ned    = (R_nb_mat * contact_forces.force_body_n).z();
        const float fz_stance = _fz_stance_filter.step(fz_ned);      // steady support estimate
        const float a_arrest  = (fz_ned - fz_stance) / _inertia.mass_kg;  // arrest transient only

        const Eigen::Vector3f& vel_ned = _state.velocity_NED_mps();
        const float v_horiz = vel_ned.head<2>().norm();
        const float V_inertial = vel_ned.norm();
        const float V_safe = std::max(V_inertial, 1.0f);
        const float gamma  = std::atan2(-vel_ned.z(), std::max(v_horiz, 1e-3f));
        const float gamma_dot_arrest = -a_arrest * std::cos(gamma) / V_safe;

        // C² dynamic-pressure authority fade Φ(V) (OQ-LG-19, upgraded to the OQ-LG-21
        // smootherstep so both fades share one C² factor): removes the 1/V singularity by
        // emulating aero/FBW authority decay on rollout/takeoff.
        const float phi = phiAuthority(V_inertial, _dtheta_vref_mps);
        const float u_force = gamma_dot_arrest * phi;

        // G(s) inline two-state update: y = h·x + j·u; x = phi·x + gamma·u.
        const float dtheta_force = (_force_h * _force_x)(0, 0) + _force_j(0, 0) * u_force;
        _force_x = _force_phi * _force_x + _force_gamma * u_force;

        // --- Moment channels: H₂ low-pass on M/I, scaled by 1/ωn² (static deflection = M/(I ωn²)). ---
        const Eigen::Vector3f& M_body = contact_forces.moment_body_nm;
        const float inv_wn2_pitch = 1.0f / (_dtheta_wn_pitch_rad_s * _dtheta_wn_pitch_rad_s);
        const float inv_wn2_roll  = 1.0f / (_dtheta_wn_roll_rad_s  * _dtheta_wn_roll_rad_s);
        const float inv_wn2_yaw   = 1.0f / (_dtheta_wn_yaw_rad_s   * _dtheta_wn_yaw_rad_s);
        const float dtheta_moment_pitch =
            _dtheta_pitch_filter.step(M_body.y() / _inertia.Iyy_kgm2) * inv_wn2_pitch;
        const float dtheta_roll =
            _dtheta_roll_filter.step(M_body.x() / _inertia.Ixx_kgm2) * inv_wn2_roll;
        const float dtheta_yaw =
            _dtheta_yaw_filter.step(M_body.z() / _inertia.Izz_kgm2) * inv_wn2_yaw;

        dtheta_pitch = dtheta_force + dtheta_moment_pitch;

        // Roll: rate of change of deviation → roll rate contribution for commitAttitude.
        delta_rr_filt_rps  = (dtheta_roll - _prev_dtheta_roll) / _outer_dt_s;
        // Yaw: rate of change of deviation × velocity → lateral specific force.
        delta_ay_filt_mps2 = (dtheta_yaw  - _prev_dtheta_yaw)  / _outer_dt_s * V_safe;

        _prev_dtheta_roll = dtheta_roll;
        _prev_dtheta_yaw  = dtheta_yaw;
    }

    LoadFactorInputs lfa_in;
    lfa_in.n_z      = n_z_shaped;
    lfa_in.n_y      = n_y_shaped;
    lfa_in.q_inf    = q_inf;
    lfa_in.thrust_n = _propulsion->thrust_n();   // previous-step thrust (0 on first call)
    lfa_in.mass_kg  = _inertia.mass_kg;
    lfa_in.n_z_dot  = n_z_dot;
    lfa_in.n_y_dot  = n_y_dot;
    lfa_in.dt_s     = _outer_dt_s;
    const LoadFactorOutputs lfa_out = _allocator->solve(lfa_in);

    // 6. Lift coefficient (effective CL accounts for stall and recovery)
    const float cl = lfa_out.cl_eff;

    // 7. Aerodynamic forces in Wind frame
    const AeroForces F =
        _aeroPerf->compute(lfa_out.alpha_rad, lfa_out.beta_rad, q_inf, cl);

    // 8. Advance propulsion
    const float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3);

    // 9. Wind-frame specific force = aero + thrust + gravity.
    //    Thrust decomposition (Wind frame, X forward, Y right, Z down):
    //      Tx =  T·cos(α)·cos(β)
    //      Ty = -T·cos(α)·sin(β)
    //      Tz = -T·sin(α)
    //    Gravity in Wind frame: C_WN · g_NED = R_nw_mat^T · {0, 0, g}
    //    (R_nw_mat is already computed below for the landing gear transform.)
    const float m  = _inertia.mass_kg;
    // Body attitude uses alpha_cmd + Δθ_pitch so that gear geometry at the next
    // step sees the body's inertially-lagged orientation (one-step lag per design).
    const float alpha_body = lfa_out.alpha_rad + dtheta_pitch;
    const float ca = std::cos(alpha_body);
    const float sa = std::sin(alpha_body);
    const float cb = std::cos(lfa_out.beta_rad);
    const float sb = std::sin(lfa_out.beta_rad);

    // F_gear_wind was computed in step 5c; g_wind from NED gravity.
    const Eigen::Vector3f g_wind = R_nw_mat.transpose() * Eigen::Vector3f{0.f, 0.f, kGravity_mps2};

    // Clamp aerodynamic lift to the LFA's intended budget.
    //
    // Step 5 told the LFA to produce n_z_shaped worth of lift.  In steady flight
    // the LFA achieves exactly this.  But when the aircraft is in contact and
    // alpha is rate-limited, the wing can produce more lift than the LFA intended
    // (residual lift from the previous high-alpha state).  Clamping prevents that
    // excess from launching the aircraft off the ground.
    //
    // F.z_n is in wind-Z (positive = down); upward lift is negative.
    // -n_z_shaped * m * g is the most-upward value the LFA intended.
    // std::max(F.z_n, -(n_z_shaped * m * g)) keeps whichever is less upward,
    // i.e., it clamps the lift to at most the LFA's target.
    //
    // When n_z_shaped == 0 (contact forces cover the full commanded load),
    // the clamp reduces to max(F.z_n, 0) = 0 — zero aero lift, no launch.
    const float F_z_aero = std::max(F.z_n, -(n_z_shaped * m * kGravity_mps2));

    const float ax = (T * ca * cb + F.x_n + F_gear_wind.x()) / m + g_wind.x();
    const float ay = (-T * ca * sb + F.y_n + F_gear_wind.y()) / m + g_wind.y() + delta_ay_filt_mps2;
    const float az = (-T * sa      + F_z_aero + F_gear_wind.z()) / m + g_wind.z();

    // DBG-FULLSTOP: print forces for the first 5 s, and at late time (>87 s) every 0.1s.
    if (_has_landing_gear && (time_sec < 5.0 || time_sec > 87.0)) {
        const float alt_now = _state.positionDatum().height_WGS84_m();
        const float vD_now  = _state.velocity_NED_mps().z();
        const float vN_now  = _state.velocity_NED_mps().x();
        const bool  wow     = contact_forces.weight_on_wheels;
        static double s_next_print = 0.0;
        if (time_sec >= s_next_print) {
            const Eigen::Quaternionf q_nb_dbg(_state.q_nb());
            const Eigen::Matrix3f R_nb_dbg = q_nb_dbg.toRotationMatrix();
            const float pitch_deg = std::asin(-R_nb_dbg(0, 2)) * 57.2958f;
            printf("  [t=%.2f] alt=%.4f vN=%.4f vD=%.4f nzrelax=%.3f nzshp=%.3f"
                   " dthp=%.4f Fwz=%.0f Fwx=%.0f az=%.3f ax=%.4f pitch=%.2f wow=%d\n",
                   (float)time_sec, alt_now, vN_now, vD_now,
                   nz_relax_dbg,
                   n_z_shaped, dtheta_pitch,
                   F_gear_wind.z(), F_gear_wind.x(),
                   az, ax, pitch_deg, (int)wow);
            s_next_print = time_sec + 0.1;
        }
    }

    // 10. Advance position and velocity. Attitude is committed after the terrain
    //     constraint so that stepQnw always sees the truly final velocity.
    //     alpha_body = alpha_cmd + Δθ_pitch commits the body's inertially-lagged
    //     orientation so that gear geometry at the next step uses the deviated attitude.
    const float dt_s = static_cast<float>(time_sec - _state.time_sec());
    _state.stepPV(time_sec,
                  Eigen::Vector3f{ax, ay, az},
                  alpha_body,
                  lfa_out.beta_rad,
                  lfa_out.alphaDot_rps,
                  lfa_out.betaDot_rps,
                  wind_NED_mps);

    // 11. Post-integration terrain hard constraint.
    //     If the integrator left any corner below terrain, project altitude up
    //     by the deepest penetration and zero the downward velocity component.
    //     Running the body collider on the penetrated post-integration state
    //     captures the impact force and weight_on_wheels flag in _contact_forces
    //     for monitoring; applyTerrainHardConstraint is called after so the
    //     collider's pen_dot sees the impact velocity before it is zeroed.
    if (_has_body_collider && _terrain != nullptr) {
        // Signed clearance of the lowest body corner: < 0 penetrating, > 0 clear.
        // (maxCornerPenetration_m is clamped to >= 0 and so cannot detect separation;
        // the latch released on a negative penetration that never occurred — OQ-LG-22.)
        const float clearance = _body_collider.minCornerClearance_m(
            _state.snapshot(), *_terrain);
        const float pen = std::max(0.f, -clearance);   // deepest penetration (>= 0)
        constexpr float kBodySeparationMargin_m = 0.05f;
        if (pen > 0.f) {
            const auto bc_impact = _body_collider.step(_state.snapshot(), *_terrain);
            _contact_forces.force_body_n   += bc_impact.force_body_n;
            _contact_forces.moment_body_nm += bc_impact.moment_body_nm;
            _contact_forces.weight_on_wheels = true;
            _body_in_hard_contact = true;
            _state.applyTerrainHardConstraint(pen);
        } else if (clearance > kBodySeparationMargin_m) {
            // The body has genuinely separated from the terrain — release the
            // hard-contact latch so weight_on_wheels reporting is accurate.
            _body_in_hard_contact = false;
        }
        // clearance in [0, margin]: small hysteresis band — hold the flag as-is.
    }

    // 12. Commit attitude: q_nw sees the truly final velocity — after RK4 and
    //     after any terrain constraint modification.
    //
    //     OQ-LG-21: the velocity-derived attitude is referenced to a dynamic-pressure
    //     blend of the instantaneous velocity and a low-pass-filtered velocity. At flight
    //     speed Φ(V)→1 and the reference is the raw velocity (unchanged behavior); as speed
    //     decays toward stop Φ(V)→0 and the reference becomes the filtered velocity, which
    //     follows the runway slope/approach trend but rejects the per-step gear-bounce wobble
    //     that otherwise whips the zero-inertia attitude (and spikes the gear). The committed
    //     attitude then yields a consistent (near-zero at quiescence) body rate for the gear.
    const Eigen::Vector3f& v_final_ned = _state.velocity_NED_mps();
    if (!_v_filt_init) {
        _v_filt_ned  = v_final_ned;
        _v_filt_init = true;
    } else {
        const float tau   = std::max(_att_filt_tau_s, 1e-3f);
        const float alpha = 1.0f - std::exp(-dt_s / tau);
        _v_filt_ned += alpha * (v_final_ned - _v_filt_ned);
    }
    const float phi_att = phiAuthority(v_final_ned.norm(), _dtheta_vref_mps);
    const Eigen::Vector3f v_att_ref = phi_att * v_final_ned + (1.0f - phi_att) * _v_filt_ned;

    _state.commitAttitude(rollRate_filt_rps + delta_rr_filt_rps, dt_s, v_att_ref);
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
    j["cmd_filter_substeps"]  = _cmd_filter_substeps;
    j["cmd_filter_dt_s"]      = _cmd_filter_dt_s;
    j["nz_wn_rad_s"]          = _nz_wn_rad_s;
    j["nz_zeta_nd"]           = _nz_zeta_nd;
    j["ny_wn_rad_s"]          = _ny_wn_rad_s;
    j["ny_zeta_nd"]           = _ny_zeta_nd;
    j["roll_rate_wn_rad_s"]   = _roll_rate_wn_rad_s;
    j["roll_rate_zeta_nd"]    = _roll_rate_zeta_nd;
    j["nz_relax_filter"]      = _nz_relax_filter.serializeJson();
    j["dtheta_pitch_filter"]  = _dtheta_pitch_filter.serializeJson();
    j["dtheta_roll_filter"]   = _dtheta_roll_filter.serializeJson();
    j["dtheta_yaw_filter"]    = _dtheta_yaw_filter.serializeJson();
    j["fz_stance_filter"]     = _fz_stance_filter.serializeJson();
    j["force_x"]              = nlohmann::json::array({_force_x(0, 0), _force_x(1, 0)});
    j["prev_dtheta_roll"]     = _prev_dtheta_roll;
    j["prev_dtheta_yaw"]      = _prev_dtheta_yaw;
    j["att_filt_tau_s"]       = _att_filt_tau_s;
    j["v_filt_ned"]           = nlohmann::json::array({_v_filt_ned.x(), _v_filt_ned.y(), _v_filt_ned.z()});
    j["v_filt_init"]          = _v_filt_init;
    j["dtheta_wn_pitch_rad_s"] = _dtheta_wn_pitch_rad_s;
    j["dtheta_wn_roll_rad_s"]  = _dtheta_wn_roll_rad_s;
    j["dtheta_wn_yaw_rad_s"]   = _dtheta_wn_yaw_rad_s;
    j["dtheta_zeta_nd"]        = _dtheta_zeta_nd;
    j["dtheta_vref_mps"]       = _dtheta_vref_mps;
    j["nz_relax_wn_rad_s"]     = _nz_relax_wn_rad_s;
    j["nz_relax_zeta_nd"]      = _nz_relax_zeta_nd;
    j["dtheta_stance_tau_s"]   = _dtheta_stance_tau_s;   // derived (= 1/nz_relax_wn_rad_s)
    j["nz_filter"]             = _nz_filter.serializeJson();
    j["ny_filter"]            = _ny_filter.serializeJson();
    j["roll_rate_filter"]     = _roll_rate_filter.serializeJson();
    j["airframe"]        = _airframe.serializeJson();
    j["inertia"]         = _inertia.serializeJson();
    if (_liftCurve)       j["lift_curve"]       = _liftCurve->serializeJson();
    if (_aeroPerf)        j["aero_performance"] = _aeroPerf->serializeJson();
    if (_allocator)       j["allocator"]         = _allocator->serializeJson();
    if (_propulsion)      j["propulsion"]         = _propulsion->serializeJson();
    if (_has_landing_gear)  j["landing_gear_state"]   = _landing_gear.serializeJson();
    if (_has_body_collider) j["body_collider_config"]  = _body_collider.serializeJson();
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
    _aeroPerf.emplace(AeroPerformance::deserializeJson(j.at("aero_performance")));

    // Emplace allocator with placeholder config — deserializeJson overwrites all scalar fields.
    // The reference to _liftCurve is set at construction and is the only field not restored.
    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeJson(j.at("allocator"));

    _state.deserializeJson(j.at("kinematic_state"));
    _initial_state.deserializeJson(j.at("initial_state"));
    _cmd_filter_substeps = j.at("cmd_filter_substeps").get<int>();
    _cmd_filter_dt_s     = j.at("cmd_filter_dt_s").get<float>();
    _nz_wn_rad_s         = j.at("nz_wn_rad_s").get<float>();
    _nz_zeta_nd          = j.at("nz_zeta_nd").get<float>();
    _ny_wn_rad_s         = j.at("ny_wn_rad_s").get<float>();
    _ny_zeta_nd          = j.at("ny_zeta_nd").get<float>();
    _roll_rate_wn_rad_s  = j.at("roll_rate_wn_rad_s").get<float>();
    _roll_rate_zeta_nd   = j.at("roll_rate_zeta_nd").get<float>();
    _dtheta_wn_pitch_rad_s = j.value("dtheta_wn_pitch_rad_s", _dtheta_wn_pitch_rad_s);
    _dtheta_wn_roll_rad_s  = j.value("dtheta_wn_roll_rad_s",  _dtheta_wn_roll_rad_s);
    _dtheta_wn_yaw_rad_s   = j.value("dtheta_wn_yaw_rad_s",   _dtheta_wn_yaw_rad_s);
    _dtheta_zeta_nd        = j.value("dtheta_zeta_nd",        _dtheta_zeta_nd);
    _dtheta_vref_mps       = j.value("dtheta_vref_mps",       _dtheta_vref_mps);
    _nz_relax_wn_rad_s     = j.value("nz_relax_wn_rad_s",     _nz_relax_wn_rad_s);
    _nz_relax_zeta_nd      = j.value("nz_relax_zeta_nd",      _nz_relax_zeta_nd);
    _dtheta_stance_tau_s   = (_nz_relax_wn_rad_s > 0.0f) ? 1.0f / _nz_relax_wn_rad_s : 1.0f;
    _nz_relax_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_relax_wn_rad_s, _nz_relax_zeta_nd, 0.f);
    _dtheta_pitch_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_pitch_rad_s, _dtheta_zeta_nd, 0.f);
    _dtheta_roll_filter .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_roll_rad_s,  _dtheta_zeta_nd, 0.f);
    _dtheta_yaw_filter  .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_yaw_rad_s,   _dtheta_zeta_nd, 0.f);
    _fz_stance_filter.setLowPassFirstIIR(_cmd_filter_dt_s, _dtheta_stance_tau_s);
    if (j.contains("nz_relax_filter"))     _nz_relax_filter.deserializeJson(j.at("nz_relax_filter"));
    if (j.contains("dtheta_pitch_filter")) _dtheta_pitch_filter.deserializeJson(j.at("dtheta_pitch_filter"));
    if (j.contains("dtheta_roll_filter"))  _dtheta_roll_filter.deserializeJson(j.at("dtheta_roll_filter"));
    if (j.contains("dtheta_yaw_filter"))   _dtheta_yaw_filter.deserializeJson(j.at("dtheta_yaw_filter"));
    if (j.contains("fz_stance_filter"))    _fz_stance_filter.deserializeJson(j.at("fz_stance_filter"));
    // Rebuild the force-channel G(s) state space from restored params, then restore its 2-state.
    {
        const float wn = _dtheta_wn_pitch_rad_s, z = _dtheta_zeta_nd;
        liteaero::control::Vec3 num_s, den_s, num_z, den_z;
        num_s << 0.0f, -1.0f, -2.0f * z * wn;
        den_s << 1.0f, 2.0f * z * wn, wn * wn;
        liteaero::control::tustin_2_tf(num_s, den_s, _cmd_filter_dt_s, wn, num_z, den_z);
        liteaero::control::tf2ss(num_z, den_z, _force_phi, _force_gamma, _force_h, _force_j);
        _force_x.setZero();
        if (j.contains("force_x") && j.at("force_x").size() >= 2) {
            _force_x(0, 0) = j.at("force_x").at(0).get<float>();
            _force_x(1, 0) = j.at("force_x").at(1).get<float>();
        }
    }
    _prev_dtheta_roll  = j.value("prev_dtheta_roll",  0.f);
    _prev_dtheta_yaw   = j.value("prev_dtheta_yaw",   0.f);
    _att_filt_tau_s    = j.value("att_filt_tau_s", _att_filt_tau_s);
    if (j.contains("v_filt_ned") && j.at("v_filt_ned").size() >= 3) {
        _v_filt_ned.x() = j.at("v_filt_ned").at(0).get<float>();
        _v_filt_ned.y() = j.at("v_filt_ned").at(1).get<float>();
        _v_filt_ned.z() = j.at("v_filt_ned").at(2).get<float>();
    } else {
        _v_filt_ned.setZero();
    }
    _v_filt_init       = j.value("v_filt_init", false);
    _nz_filter.deserializeJson(j.at("nz_filter"));
    _ny_filter.deserializeJson(j.at("ny_filter"));
    _roll_rate_filter.deserializeJson(j.at("roll_rate_filter"));

    if (_propulsion) {
        _propulsion->deserializeJson(j.at("propulsion"));
    }
    if (_has_landing_gear && j.contains("landing_gear_state")) {
        _landing_gear.deserializeJson(j.at("landing_gear_state"));
    }
    if (_has_body_collider && j.contains("body_collider_config")) {
        _body_collider.deserializeJson(j.at("body_collider_config"));
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
    proto.set_cmd_filter_substeps(_cmd_filter_substeps);
    proto.set_cmd_filter_dt_s(_cmd_filter_dt_s);
    proto.set_nz_wn_rad_s(_nz_wn_rad_s);
    proto.set_nz_zeta_nd(_nz_zeta_nd);
    proto.set_ny_wn_rad_s(_ny_wn_rad_s);
    proto.set_ny_zeta_nd(_ny_zeta_nd);
    proto.set_roll_rate_wn_rad_s(_roll_rate_wn_rad_s);
    proto.set_roll_rate_zeta_nd(_roll_rate_zeta_nd);
    proto.set_nz_filter_x0(_nz_filter.x()(0, 0));
    proto.set_nz_filter_x1(_nz_filter.x()(1, 0));
    proto.set_ny_filter_x0(_ny_filter.x()(0, 0));
    proto.set_ny_filter_x1(_ny_filter.x()(1, 0));
    proto.set_roll_rate_filter_x0(_roll_rate_filter.x()(0, 0));
    proto.set_roll_rate_filter_x1(_roll_rate_filter.x()(1, 0));
    proto.set_nz_relax_filter_x0(_nz_relax_filter.x()(0,0));
    proto.set_nz_relax_filter_x1(_nz_relax_filter.x()(1,0));
    proto.set_dtheta_pitch_filter_x0(_dtheta_pitch_filter.x()(0,0));
    proto.set_dtheta_pitch_filter_x1(_dtheta_pitch_filter.x()(1,0));
    proto.set_dtheta_roll_filter_x0(_dtheta_roll_filter.x()(0,0));
    proto.set_dtheta_roll_filter_x1(_dtheta_roll_filter.x()(1,0));
    proto.set_dtheta_yaw_filter_x0(_dtheta_yaw_filter.x()(0,0));
    proto.set_dtheta_yaw_filter_x1(_dtheta_yaw_filter.x()(1,0));
    proto.set_force_x0(_force_x(0, 0));
    proto.set_force_x1(_force_x(1, 0));
    proto.set_fz_stance_filter_x0(_fz_stance_filter.x()(0, 0));
    proto.set_fz_stance_filter_x1(_fz_stance_filter.x()(1, 0));
    proto.set_dtheta_wn_pitch_rad_s(_dtheta_wn_pitch_rad_s);
    proto.set_dtheta_wn_roll_rad_s(_dtheta_wn_roll_rad_s);
    proto.set_dtheta_wn_yaw_rad_s(_dtheta_wn_yaw_rad_s);
    proto.set_dtheta_zeta_nd(_dtheta_zeta_nd);
    proto.set_dtheta_vref_mps(_dtheta_vref_mps);
    proto.set_dtheta_stance_tau_s(_dtheta_stance_tau_s);
    proto.set_nz_relax_wn_rad_s(_nz_relax_wn_rad_s);
    proto.set_nz_relax_zeta_nd(_nz_relax_zeta_nd);
    proto.set_att_filt_tau_s(_att_filt_tau_s);
    proto.set_v_filt_ned_n(_v_filt_ned.x());
    proto.set_v_filt_ned_e(_v_filt_ned.y());
    proto.set_v_filt_ned_d(_v_filt_ned.z());
    proto.set_v_filt_init(_v_filt_init);

    *proto.mutable_kinematic_state()  = parseSubMessage<las_proto::KinematicState>(_state.serializeProto());
    *proto.mutable_initial_state()    = parseSubMessage<las_proto::KinematicState>(_initial_state.serializeProto());
    if (_allocator) *proto.mutable_allocator()         = parseSubMessage<las_proto::LoadFactorAllocatorState>(_allocator->serializeProto());
    if (_liftCurve) *proto.mutable_lift_curve()        = parseSubMessage<las_proto::LiftCurveParams>(_liftCurve->serializeProto());
    if (_aeroPerf)  *proto.mutable_aero_performance()  = parseSubMessage<las_proto::AeroPerformanceParams>(_aeroPerf->serializeProto());
    *proto.mutable_airframe() = parseSubMessage<las_proto::AirframePerformanceParams>(_airframe.serializeProto());
    *proto.mutable_inertia()  = parseSubMessage<las_proto::InertiaParams>(_inertia.serializeProto());

    if (_propulsion) {
        if (auto* jet = dynamic_cast<PropulsionJet*>(_propulsion.get()))
            *proto.mutable_jet()  = parseSubMessage<las_proto::PropulsionJetState>(jet->serializeProto());
        else if (auto* edf = dynamic_cast<PropulsionEDF*>(_propulsion.get()))
            *proto.mutable_edf()  = parseSubMessage<las_proto::PropulsionEdfState>(edf->serializeProto());
        else if (auto* prop = dynamic_cast<PropulsionProp*>(_propulsion.get()))
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
    _aeroPerf.emplace(AeroPerformance::deserializeProto(serializeSubMessage(proto.aero_performance())));

    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeProto(serializeSubMessage(proto.allocator()));

    _state.deserializeProto(serializeSubMessage(proto.kinematic_state()));
    _initial_state.deserializeProto(serializeSubMessage(proto.initial_state()));
    _cmd_filter_substeps = proto.cmd_filter_substeps();
    _cmd_filter_dt_s     = proto.cmd_filter_dt_s();
    _nz_wn_rad_s         = proto.nz_wn_rad_s();
    _nz_zeta_nd          = proto.nz_zeta_nd();
    _ny_wn_rad_s         = proto.ny_wn_rad_s();
    _ny_zeta_nd          = proto.ny_zeta_nd();
    _roll_rate_wn_rad_s  = proto.roll_rate_wn_rad_s();
    _roll_rate_zeta_nd   = proto.roll_rate_zeta_nd();
    _dtheta_wn_pitch_rad_s = proto.dtheta_wn_pitch_rad_s();
    _dtheta_wn_roll_rad_s  = proto.dtheta_wn_roll_rad_s();
    _dtheta_wn_yaw_rad_s   = proto.dtheta_wn_yaw_rad_s();
    _dtheta_zeta_nd        = proto.dtheta_zeta_nd();
    _dtheta_vref_mps       = proto.dtheta_vref_mps();
    _nz_relax_wn_rad_s     = proto.nz_relax_wn_rad_s();
    _nz_relax_zeta_nd      = proto.nz_relax_zeta_nd();
    _dtheta_stance_tau_s   = (_nz_relax_wn_rad_s > 0.0f) ? 1.0f / _nz_relax_wn_rad_s : 1.0f;
    _nz_relax_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_relax_wn_rad_s, _nz_relax_zeta_nd, 0.f);
    _dtheta_pitch_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_pitch_rad_s, _dtheta_zeta_nd, 0.f);
    _dtheta_roll_filter .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_roll_rad_s,  _dtheta_zeta_nd, 0.f);
    _dtheta_yaw_filter  .setLowPassSecondIIR(_cmd_filter_dt_s, _dtheta_wn_yaw_rad_s,   _dtheta_zeta_nd, 0.f);
    _fz_stance_filter.setLowPassFirstIIR(_cmd_filter_dt_s, _dtheta_stance_tau_s);
    {
        Mat21 x;
        x(0,0) = proto.nz_relax_filter_x0();
        x(1,0) = proto.nz_relax_filter_x1();
        _nz_relax_filter.resetState(x);
        x(0,0) = proto.dtheta_pitch_filter_x0();
        x(1,0) = proto.dtheta_pitch_filter_x1();
        _dtheta_pitch_filter.resetState(x);
        x(0,0) = proto.dtheta_roll_filter_x0();
        x(1,0) = proto.dtheta_roll_filter_x1();
        _dtheta_roll_filter.resetState(x);
        x(0,0) = proto.dtheta_yaw_filter_x0();
        x(1,0) = proto.dtheta_yaw_filter_x1();
        _dtheta_yaw_filter.resetState(x);
        x(0,0) = proto.fz_stance_filter_x0();
        x(1,0) = proto.fz_stance_filter_x1();
        _fz_stance_filter.resetState(x);
    }
    // Rebuild force-channel G(s) from restored params, then restore its 2-state.
    {
        const float wn = _dtheta_wn_pitch_rad_s, z = _dtheta_zeta_nd;
        liteaero::control::Vec3 num_s, den_s, num_z, den_z;
        num_s << 0.0f, -1.0f, -2.0f * z * wn;
        den_s << 1.0f, 2.0f * z * wn, wn * wn;
        liteaero::control::tustin_2_tf(num_s, den_s, _cmd_filter_dt_s, wn, num_z, den_z);
        liteaero::control::tf2ss(num_z, den_z, _force_phi, _force_gamma, _force_h, _force_j);
        _force_x(0, 0) = proto.force_x0();
        _force_x(1, 0) = proto.force_x1();
    }
    _nz_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_wn_rad_s, _nz_zeta_nd, 0.f);
    _ny_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _ny_wn_rad_s, _ny_zeta_nd, 0.f);
    _roll_rate_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _roll_rate_wn_rad_s, _roll_rate_zeta_nd, 0.f);
    Mat21 nz_x;
    nz_x(0, 0) = proto.nz_filter_x0();
    nz_x(1, 0) = proto.nz_filter_x1();
    _nz_filter.resetState(nz_x);
    Mat21 ny_x;
    ny_x(0, 0) = proto.ny_filter_x0();
    ny_x(1, 0) = proto.ny_filter_x1();
    _ny_filter.resetState(ny_x);
    Mat21 rr_x;
    rr_x(0, 0) = proto.roll_rate_filter_x0();
    rr_x(1, 0) = proto.roll_rate_filter_x1();
    _roll_rate_filter.resetState(rr_x);

    _att_filt_tau_s = (proto.att_filt_tau_s() > 0.0f) ? proto.att_filt_tau_s() : _att_filt_tau_s;
    _v_filt_ned.x() = proto.v_filt_ned_n();
    _v_filt_ned.y() = proto.v_filt_ned_e();
    _v_filt_ned.z() = proto.v_filt_ned_d();
    _v_filt_init    = proto.v_filt_init();

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

// ---------------------------------------------------------------------------

float Aircraft::agl_m() const {
    if (_terrain == nullptr) return -1.f;
    const auto& pos = _state.positionDatum();
    return pos.height_WGS84_m() -
           _terrain->elevation_m(pos.latitudeGeodetic_rad(), pos.longitude_rad());
}

} // namespace liteaero::simulation
