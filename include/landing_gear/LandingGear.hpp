#pragma once

#include <physics/ContactForces.hpp>
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

    // OQ-AC-3 substep co-integration interface. The terrain tangent-plane approximation is computed
    // once per OUTER step by `beginContact`; `substepContact` then evaluates the strut/tire forces for
    // ONE inner step from the (possibly evolving) pose against that stored plane. `step()` is a
    // behavior-preserving wrapper: beginContact + N× substepContact on a fixed pose. The caller
    // (`Aircraft`) drives the substep loop and advances the roll bank between calls so the strut
    // damper reacts to the in-phase roll rate; no roll dynamics live here.
    struct ContactPlane {
        float           terrain_h_m = 0.f;
        Eigen::Vector3f surface_normal_ned{0.f, 0.f, -1.f};
        bool            active = false;   // false => fully airborne / nothing to integrate
    };
    ContactPlane  beginContact(const liteaero::nav::KinematicStateSnapshot& snap,
                               const liteaero::terrain::Terrain&            terrain);
    ContactForces substepContact(const liteaero::nav::KinematicStateSnapshot& snap,
                                 const ContactPlane&                          plane,
                                 float nose_wheel_angle_rad,
                                 float brake_left_nd,
                                 float brake_right_nd,
                                 float inner_dt_s);

    const ContactForces& contactForces() const { return _contact_forces; }

    // Read-only access for Python instrumentation (bind_aircraft.cpp).
    const std::vector<WheelUnit>&   wheelUnits() const { return _wheel_units; }
    const LandingGearConfig&        config()     const { return _config; }

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json&          j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    LandingGearConfig        _config;
    std::vector<WheelUnit>   _wheel_units;
    ContactForces            _contact_forces;
    SurfaceFrictionUniform   _surface_friction{SurfaceFrictionUniform::pavement()};
    float                    _max_reach_m  = 0.f;
    bool                     _initialized  = false;
};

}  // namespace liteaero::simulation
