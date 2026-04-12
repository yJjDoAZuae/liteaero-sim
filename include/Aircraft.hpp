#pragma once

#include "KinematicState.hpp"
#include "aerodynamics/AeroPerformance.hpp"
#include "aerodynamics/LiftCurveModel.hpp"
#include "aerodynamics/LoadFactorAllocator.hpp"
#include "airframe/AirframePerformance.hpp"
#include "airframe/Inertia.hpp"
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

    // Landing gear contact forces from the most recent step().
    // Returns zero forces when landing gear is not configured or terrain is not set.
    const ContactForces& contactForces() const { return _landing_gear.contactForces(); }

    // True when any wheel unit was in contact with terrain on the most recent step().
    bool weightOnWheels() const { return _landing_gear.contactForces().weight_on_wheels; }

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
    const liteaero::terrain::Terrain*               _terrain          = nullptr;
    bool                                              _has_landing_gear = false;

    // 2nd-order LP command response filters (Nz, Ny, roll rate).
    liteaero::control::FilterSS2Clip _nz_filter;
    liteaero::control::FilterSS2Clip _ny_filter;
    liteaero::control::FilterSS2Clip _roll_rate_filter;
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
