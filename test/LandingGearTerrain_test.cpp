// Tests for LandingGear terrain elevation query — Step E.
// Design authority: docs/architecture/landing_gear.md

#include "landing_gear/LandingGear.hpp"
#include <liteaero/terrain/Terrain.hpp>
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <gtest/gtest.h>

using namespace liteaero::simulation;
using liteaero::nav::KinematicStateSnapshot;

class ElevatedTerrain : public liteaero::terrain::Terrain {
public:
    explicit ElevatedTerrain(float elev_m) : elev_(elev_m) {}
    [[nodiscard]] float elevation_m(double, double) const override { return elev_; }
private:
    float elev_;
};

static nlohmann::json makeSingleWheelConfig() {
    return nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [{
            "attach_point_body_m": [0.0, 0.0, 0.5],
            "travel_axis_body":    [0.0, 0.0, 1.0],
            "spring_stiffness_npm": 10000.0,
            "damper_coeff_nspm":    0.0,
            "preload_n":            0.0,
            "travel_max_m":         0.3,
            "tyre_radius_m":        0.2,
            "rolling_resistance_nd":0.0,
            "max_brake_torque_nm":  0.0,
            "is_steerable":         false,
            "has_brake":            false
        }]
    })");
}

static KinematicStateSnapshot makeSnap(float alt_m) {
    KinematicStateSnapshot s;
    s.position.altitude_m    = alt_m;
    s.position.latitude_rad  = 0.0;
    s.position.longitude_rad = 0.0;
    s.q_nw                   = Eigen::Quaternionf::Identity();
    s.alpha_rad              = 0.0f;
    s.beta_rad               = 0.0f;
    return s;
}

TEST(LandingGearTerrain, Contact_AtElevatedTerrain) {
    LandingGear gear;
    gear.initialize(makeSingleWheelConfig());

    // Terrain at 100 m; aircraft at 100.5 m → same 0.2 m penetration as sea-level
    ElevatedTerrain terrain{100.0f};
    const auto cf = gear.step(makeSnap(100.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_TRUE(cf.weight_on_wheels);
    EXPECT_LT(cf.force_body_n.z(), 0.0f);
}

TEST(LandingGearTerrain, ForceProportionalToPenetration) {
    // No damper → force = k * penetration (linear spring only).
    // k = 10 000 N/m.  Two independent gears:
    //   gear1: alt=0.6 → pen = 0 - (0.6 - 0.7) = 0.1 m → F = 1 000 N
    //   gear2: alt=0.5 → pen = 0 - (0.5 - 0.7) = 0.2 m → F = 2 000 N  (ratio = 2)
    LandingGear gear1;
    gear1.initialize(makeSingleWheelConfig());
    ElevatedTerrain terrain{0.0f};
    const auto cf1 = gear1.step(makeSnap(0.6f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    LandingGear gear2;
    gear2.initialize(makeSingleWheelConfig());
    const auto cf2 = gear2.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    const float F1 = cf1.force_body_n.norm();
    const float F2 = cf2.force_body_n.norm();
    ASSERT_GT(F1, 0.0f);
    EXPECT_NEAR(F2 / F1, 2.0f, 0.2f)
        << "Spring force should be proportional to penetration depth";
}

TEST(LandingGearTerrain, NoContact_WellAboveTerrain) {
    LandingGear gear;
    gear.initialize(makeSingleWheelConfig());

    ElevatedTerrain terrain{50.0f};
    // Aircraft at 5000 m — far above any terrain
    const auto cf = gear.step(makeSnap(5000.0f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_FALSE(cf.weight_on_wheels);
    EXPECT_FLOAT_EQ(cf.force_body_n.norm(), 0.0f);
}
