#pragma once

#include "KinematicState.hpp"
#include "aerodynamics/AeroPerformance.hpp"
#include "aerodynamics/LiftCurveModel.hpp"
#include "aerodynamics/LoadFactorAllocator.hpp"
#include "airframe/AirframePerformance.hpp"
#include "airframe/Inertia.hpp"
#include "collision/BodyCollider.hpp"
#include "landing_gear/LandingGear.hpp"
#include <liteaero/control/FilterSS2Clip.hpp>
#include <liteaero/terrain/Terrain.hpp>
#include "propulsion/Propulsion.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

namespace liteaero::simulation {

// Inputs to a single Aircraft::step() call.
struct AircraftCommand {
    float n_z               = 1.f;   // commanded normal load factor (g)
    float n_y               = 0.f;   // commanded lateral load factor (g)
    float rollRate_Wind_rps = 0.f;   // commanded wind-frame roll rate (rad/s)
    float throttle_nd       = 0.f;   // normalized throttle [0, 1]
};

// Top-level aircraft physics model.
//
// Owns all aerodynamics and propulsion subcomponents and advances their state
// via a single step() call.  Lives in the Domain Layer — no I/O, no unit
// conversions, all values in SI units.
//
// Lifecycle:  Aircraft(propulsion) → initialize(config) → reset() → step(...)
//
// Aircraft is non-copyable and non-movable: LoadFactorAllocator holds a
// const reference to _liftCurve, which is stored inline.  Moving or copying
// Aircraft would invalidate that reference.
class Aircraft {
public:
    explicit Aircraft(std::unique_ptr<Propulsion> propulsion);

    // Aircraft is non-copyable, non-movable (LoadFactorAllocator holds &_liftCurve).
    Aircraft(const Aircraft&)            = delete;
    Aircraft& operator=(const Aircraft&) = delete;
    Aircraft(Aircraft&&)                 = delete;
    Aircraft& operator=(Aircraft&&)      = delete;

    // Initialize all subsystems from a validated aircraft_config_v1 JSON object.
    // outer_dt_s — integration timestep owned by the Simulation (s); stored for reference.
    // Throws std::invalid_argument if any required field is missing or out of range.
    void initialize(const nlohmann::json& config, float outer_dt_s);

    // Set the terrain model used by LandingGear each step.
    // Call before the first step(). Pass nullptr to disable landing gear contact forces.
    void setTerrain(const liteaero::terrain::Terrain* terrain) { _terrain = terrain; }

    // Reset all warm-start state to the initial conditions from the last initialize() call.
    void reset();

    // Advance the aircraft physics by one timestep.
    //   time_sec      — absolute simulation time (s); dt is derived from the previous state
    //   cmd           — commanded inputs from autopilot or test code
    //   wind_NED_mps  — ambient wind vector in NED frame (m/s)
    //   rho_kgm3      — local air density (kg/m³)
    void step(double time_sec,
              const AircraftCommand& cmd,
              const Eigen::Vector3f& wind_NED_mps,
              float rho_kgm3);

    // Current kinematic state (position, velocity, attitude, aerodynamic angles).
    const KinematicState& state() const { return _state; }

    // Combined contact forces (gear + body collider) from the most recent step().
    // Returns zero forces when no terrain is set or no contact occurred.
    const ContactForces& contactForces() const { return _contact_forces; }

    // True when any contact was detected on the most recent step().
    bool weightOnWheels() const { return _contact_forces.weight_on_wheels; }

    // Height above terrain at the current position (m).
    // Returns -1 if no terrain has been set via setTerrain().
    float agl_m() const;

    // Read-only access to the LandingGear subsystem (for Python instrumentation).
    const LandingGear& landingGear() const { return _landing_gear; }
    bool hasLandingGear() const { return _has_landing_gear; }

    // Serialize / deserialize warm-start state.
    // Note: deserializeJson() restores _propulsion state via _propulsion->deserializeJson()
    // but does not reconstruct the propulsion model itself — the correct Propulsion
    // subclass must have been injected at construction before calling deserializeJson().
    nlohmann::json       serializeJson() const;
    void                 deserializeJson(const nlohmann::json& j);
    std::vector<uint8_t> serializeProto() const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);

private:
    KinematicState                                    _state;
    KinematicState                                    _initial_state;
    std::optional<LiftCurveModel>                     _liftCurve;
    std::optional<LoadFactorAllocator>                _allocator;
    std::optional<AeroPerformance>                    _aeroPerf;
    AirframePerformance                               _airframe;
    Inertia                                           _inertia;
    std::unique_ptr<Propulsion>                       _propulsion;
    LandingGear                                       _landing_gear;
    BodyCollider                                      _body_collider;
    ContactForces                                     _contact_forces;
    const liteaero::terrain::Terrain*               _terrain            = nullptr;
    bool                                              _has_landing_gear   = false;
    bool                                              _has_body_collider  = false;

    // 2nd-order LP command response filters (Nz, Ny, roll rate).
    liteaero::control::FilterSS2Clip _nz_filter;
    liteaero::control::FilterSS2Clip _ny_filter;
    liteaero::control::FilterSS2Clip _roll_rate_filter;
    // H₁: lagged n_z-command relaxation from gear normal force load (LP, DC=1).
    // Parameterized from FBW Nz ωn/ζ (OQ-LG-17).
    liteaero::control::FilterSS2Clip _nz_relax_filter;
    // H₂: gear-MOMENT rotation-deviation filters (2nd-order LP on M/I, finite DC = static
    // stance). Pitch/roll/yaw. Force channel is realized separately (G(s) below). OQ-LG-19/20.
    liteaero::control::FilterSS2Clip _dtheta_pitch_filter;
    liteaero::control::FilterSS2Clip _dtheta_roll_filter;
    liteaero::control::FilterSS2Clip _dtheta_yaw_filter;
    // Force channel G(s) = -(s+2ζωn)/D, realized inline via tustin_2_tf/tf2ss (OQ-LG-20).
    // phi/gamma/h/j are set once in initialize(); _force_x is the 2-state (serialized).
    liteaero::control::Mat22 _force_phi   = liteaero::control::Mat22::Zero();
    liteaero::control::Mat21 _force_gamma = liteaero::control::Mat21::Zero();
    liteaero::control::Mat12 _force_h     = liteaero::control::Mat12::Zero();
    liteaero::control::Mat11 _force_j     = liteaero::control::Mat11::Zero();
    liteaero::control::Mat21 _force_x     = liteaero::control::Mat21::Zero();
    // Destancing low-pass for the gear vertical load (removes the steady gravity-balancing
    // support so the force channel is zero in steady roll). General 1st-order LP. OQ-LG-19.
    liteaero::control::FilterSS2Clip _fz_stance_filter;
    float  _prev_dtheta_roll      = 0.f;  // Δθ_roll from previous step (for rate computation)
    float  _prev_dtheta_yaw       = 0.f;  // Δθ_yaw from previous step (for rate computation)
    // H₂ physical rotational-mode frequencies/damping (config; OQ-LG-17 numeric values set at
    // implementation — supplied as physical constants, not derived from inertia alone, which is
    // dimensionally insufficient for a frequency).
    float  _dtheta_wn_pitch_rad_s = 3.0f;
    float  _dtheta_wn_roll_rad_s  = 4.0f;
    float  _dtheta_wn_yaw_rad_s   = 2.0f;
    float  _dtheta_zeta_nd        = 0.7f;
    float  _dtheta_vref_mps       = 24.0f; // V_ref for the V² authority fade Φ(V) (OQ-LG-19)
    // H₁ FBW load-handoff parameters (OQ-LG-17): distinct from the FBW *command* filter
    // (nz_wn) and slower than the body rotational mode H₂. The destancing low-pass shares
    // this timescale (both isolate the steady gear load), so it is NOT an independent knob:
    // _dtheta_stance_tau_s is derived as 1 / _nz_relax_wn_rad_s.
    float  _nz_relax_wn_rad_s     = 1.0f;  // FBW load-handoff natural frequency (rad/s)
    float  _nz_relax_zeta_nd      = 0.8f;  // FBW load-handoff damping ratio
    float  _dtheta_stance_tau_s   = 1.0f;  // destancing LP τ — DERIVED = 1/_nz_relax_wn_rad_s
    bool   _body_in_hard_contact  = false; // set by step-11 hard constraint; read in step-5b
    // OQ-LG-21: low-pass-filtered NED velocity used as the low-speed attitude reference, and
    // the dynamic-pressure blend toward it. Rejects gear-bounce wobble while retaining the
    // slope/approach trend, so the velocity-derived attitude is slope-correct and does not
    // whip at low horizontal speed. The committed-attitude rate (consistent) then feeds the
    // gear directly — superseding the earlier gear-only body-rate override.
    Eigen::Vector3f _v_filt_ned   = Eigen::Vector3f::Zero();
    bool            _v_filt_init  = false;
    float           _att_filt_tau_s = 0.7f;  // attitude-reference velocity low-pass τ (s)
    // OQ-LG-23: additive axial-acceleration settle/rotation term (§2b-ii). On the ground the
    // wing lift is biased by a single gain on the *steady* longitudinal-force deficit
    // ā_x = LP[(T − D_aero − D_wheel(V))/m] — not the raw (oscillatory) gear force — so the
    // aircraft settles onto the gear on landing and rotates on takeoff without a degenerate
    // float. Ground-faded by a stall-referenced speed smoothstep × WoW (separate landing/takeoff
    // transition speeds, shared width); the increment is low-pass filtered and clipped.
    float  _settle_gain_nd        = 25.0f;  // k_s: deficit (g) → load-factor increment gain
    float  _settle_clip_nd        = 1.0f;   // Δ_max: clip on the settle increment (g)
    float  _settle_tau_s          = 0.3f;   // low-pass τ on ā_x (s)
    float  _settle_wheel_rr_nd    = 0.05f;  // steady rolling-drag coeff for D_wheel(V) model
    float  _settle_vland_ratio    = 1.00f;  // landing transition speed / V_stall
    float  _settle_vtakeoff_ratio = 1.15f;  // takeoff transition speed / V_stall
    float  _settle_vwidth_ratio   = 0.50f;  // shared smoothstep width / V_stall
    float  _stall_speed_mps       = 1.0f;   // computed at init from W, S_ref, CL_max
    float  _settle_axbar          = 0.f;    // low-pass state of ā_x (serialized)
    float  _prev_aero_drag_n      = 0.f;    // previous-step aerodynamic drag magnitude (serialized)
    float                  _outer_dt_s           = 0.02f;  // integration timestep from Simulation
    int                    _cmd_filter_substeps   = 1;      // filter steps per Aircraft::step()
    float                  _cmd_filter_dt_s       = 0.02f;  // outer_dt_s / cmd_filter_substeps
    float                  _nz_wn_rad_s           = 10.f;
    float                  _nz_zeta_nd            = 0.7f;
    float                  _ny_wn_rad_s           = 10.f;
    float                  _ny_zeta_nd            = 0.7f;
    float                  _roll_rate_wn_rad_s    = 20.f;
    float                  _roll_rate_zeta_nd     = 0.7f;
};

} // namespace liteaero::simulation
