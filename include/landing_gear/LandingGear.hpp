#pragma once

#include <landing_gear/ContactForces.hpp>
#include <landing_gear/SurfaceFrictionUniform.hpp>
#include <landing_gear/WheelUnit.hpp>
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <liteaero/terrain/Terrain.hpp>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

namespace liteaero::simulation {

struct LandingGearConfig {
    std::vector<WheelUnitParams> wheel_units;
    int substeps = 4;
};

class LandingGear {
public:
    // Initialize from the "landing_gear" JSON section of an aircraft config.
    // Throws std::invalid_argument if required fields are missing or invalid.
    void initialize(const nlohmann::json& config);

    void reset();

    // Compute contact forces for one outer timestep.
    //   snap             — current kinematic state snapshot
    //   terrain          — ground elevation source
    //   nose_wheel_angle_rad — steering angle for steerable wheels (rad)
    //   brake_left_nd    — left-side brake demand [0, 1]
    //   brake_right_nd   — right-side brake demand [0, 1]
    //   outer_dt_s       — outer simulation timestep (s)
    ContactForces step(const liteaero::nav::KinematicStateSnapshot& snap,
                       const liteaero::terrain::Terrain&            terrain,
                       float nose_wheel_angle_rad,
                       float brake_left_nd,
                       float brake_right_nd,
                       float outer_dt_s);

    const ContactForces& contactForces() const { return _contact_forces; }

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json&          j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    LandingGearConfig        _config;
    std::vector<WheelUnit>   _wheel_units;
    ContactForces            _contact_forces;
    SurfaceFrictionUniform   _surface_friction{SurfaceFrictionUniform::pavement()};
    bool                     _initialized = false;
};

}  // namespace liteaero::simulation
