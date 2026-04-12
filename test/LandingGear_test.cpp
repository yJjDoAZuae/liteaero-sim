// Tests for LandingGear — Steps C and D (normal force and Pacejka tyre forces).
// Design authority: docs/architecture/landing_gear.md

#include "landing_gear/LandingGear.hpp"
#include <liteaero/terrain/Terrain.hpp>
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaero::simulation;
using liteaero::nav::KinematicStateSnapshot;

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

class FlatTestTerrain : public liteaero::terrain::Terrain {
public:
    explicit FlatTestTerrain(float elev_m = 0.0f) : elev_(elev_m) {}
    [[nodiscard]] float elevation_m(double, double) const override { return elev_; }
private:
    float elev_;
};

// Tricycle gear: nose at +2 m, mains at ±1 m lateral, all 0.5 m below CG, r=0.2 m.
// contact_altitude = aircraft_alt - 0.7 m (0.5 attach + 0.2 tyre radius).
static nlohmann::json makeTricycleConfig() {
    return nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [
            {
                "attach_point_body_m": [2.0, 0.0, 0.5],
                "travel_axis_body":    [0.0, 0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  0.0,
                "is_steerable":         false,
                "has_brake":            false
            },
            {
                "attach_point_body_m": [0.0, -1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  500.0,
                "is_steerable":         false,
                "has_brake":            true
            },
            {
                "attach_point_body_m": [0.0,  1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  500.0,
                "is_steerable":         false,
                "has_brake":            true
            }
        ]
    })");
}

static KinematicStateSnapshot makeSnap(float altitude_m,
                                        Eigen::Vector3f vel_ned_mps = Eigen::Vector3f::Zero()) {
    KinematicStateSnapshot s;
    s.position.altitude_m    = altitude_m;
    s.position.latitude_rad  = 0.0;
    s.position.longitude_rad = 0.0;
    s.velocity_ned_mps       = vel_ned_mps;
    s.q_nw                   = Eigen::Quaternionf::Identity();
    s.alpha_rad              = 0.0f;
    s.beta_rad               = 0.0f;
    return s;
}

// ---------------------------------------------------------------------------
// Step C — normal force tests
// ---------------------------------------------------------------------------

TEST(LandingGear, NoContact_ZeroForces) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    // Aircraft at 1000 m — all wheels airborne
    const auto cf = gear.step(makeSnap(1000.0f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_FLOAT_EQ(cf.force_body_n.x(),   0.0f);
    EXPECT_FLOAT_EQ(cf.force_body_n.y(),   0.0f);
    EXPECT_FLOAT_EQ(cf.force_body_n.z(),   0.0f);
    EXPECT_FALSE(cf.weight_on_wheels);
}

TEST(LandingGear, Compressed_PositiveNormalForce) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    // Aircraft at 0.5 m: contact_alt = 0.5 - 0.7 = -0.2 m → penetration = 0.2 m
    const auto cf = gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    // Ground pushes aircraft UP — in body frame (Z down) that is negative body-Z
    EXPECT_LT(cf.force_body_n.z(), 0.0f) << "Upward reaction force must be negative body-Z";
    EXPECT_TRUE(cf.weight_on_wheels);
}

TEST(LandingGear, SymmetricGear_NoRollMoment) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NEAR(cf.moment_body_nm.x(), 0.0f, 1e-2f)
        << "Symmetric main gear must produce zero roll moment";
}

TEST(LandingGear, NoseGear_PitchMoment) {
    // Single nose wheel at +2 m forward of CG
    auto config = nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [{
            "attach_point_body_m": [2.0, 0.0, 0.5],
            "travel_axis_body":    [0.0, 0.0, 1.0],
            "spring_stiffness_npm": 20000.0,
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
    LandingGear gear;
    gear.initialize(config);
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    // Nose at +x, force is upward (-Z body). Moment = c × f.
    // c ≈ [2, 0, ~0.7], f = [0, 0, -Fz]
    // M_y = c_z*f_x - c_x*f_z = 0 - 2*(-Fz) = +2*Fz > 0 (nose-up)
    EXPECT_GT(cf.moment_body_nm.y(), 0.0f)
        << "Nose gear contact must produce nose-up (positive body-Y) pitch moment";
}

TEST(LandingGear, WeightOnWheels_FalseWhenAirborne) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(1000.0f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);
    EXPECT_FALSE(cf.weight_on_wheels);
}

TEST(LandingGear, WeightOnWheels_TrueWhenContact) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);
    EXPECT_TRUE(cf.weight_on_wheels);
}

TEST(LandingGear, StrutLimit_ForceDoesNotExceedFullyCompressed) {
    // Single wheel: k = 10 000 N/m, travel_max = 0.1 m → max force = 1 000 N
    auto config = nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [{
            "attach_point_body_m": [0.0, 0.0, 0.5],
            "travel_axis_body":    [0.0, 0.0, 1.0],
            "spring_stiffness_npm": 10000.0,
            "damper_coeff_nspm":    0.0,
            "preload_n":            0.0,
            "travel_max_m":         0.1,
            "tyre_radius_m":        0.2,
            "rolling_resistance_nd":0.0,
            "max_brake_torque_nm":  0.0,
            "is_steerable":         false,
            "has_brake":            false
        }]
    })");
    LandingGear gear;
    gear.initialize(config);

    // Penetration >> travel_max: terrain at 1 m, aircraft at 0 m
    // contact_alt = 0 - 0.7 = -0.7 → pen = 1 - (-0.7) = 1.7 m >> 0.1 m limit
    FlatTestTerrain terrain{1.0f};
    const auto cf = gear.step(makeSnap(0.0f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    const float F_max = 10000.0f * 0.1f;  // k * travel_max
    EXPECT_LE(-cf.force_body_n.z(), F_max + 1.0f)
        << "Strut force must not exceed fully-compressed limit";
}

TEST(LandingGear, JsonRoundTrip_StrutState) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};
    gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    const auto j = gear.serializeJson();
    LandingGear gear2;
    gear2.initialize(makeTricycleConfig());
    gear2.deserializeJson(j);
    const auto j2 = gear2.serializeJson();

    EXPECT_FLOAT_EQ(j["wheel_units"][0]["strut_deflection_m"].get<float>(),
                    j2["wheel_units"][0]["strut_deflection_m"].get<float>());
}

TEST(LandingGear, ProtoRoundTrip_LandingGearState) {
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};
    gear.step(makeSnap(0.5f), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    const auto bytes = gear.serializeProto();
    LandingGear gear2;
    gear2.initialize(makeTricycleConfig());
    gear2.deserializeProto(bytes);

    EXPECT_FLOAT_EQ(gear.serializeJson()["wheel_units"][0]["strut_deflection_m"].get<float>(),
                    gear2.serializeJson()["wheel_units"][0]["strut_deflection_m"].get<float>());
}

// ---------------------------------------------------------------------------
// Step D — Pacejka tyre forces
// ---------------------------------------------------------------------------

TEST(LandingGear, TyreForce_ZeroSlip_ZeroLateral) {
    // Forward motion only, no sideslip → lateral tyre force should be near zero
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}),
                               terrain, 0.0f, 0.0f, 0.0f, 0.02f);
    EXPECT_NEAR(cf.force_body_n.y(), 0.0f, 5.0f)
        << "Pure forward motion should produce near-zero lateral force";
}

TEST(LandingGear, NoseWheelSteering_ProducesYawMoment) {
    auto config = nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [{
            "attach_point_body_m": [2.0, 0.0, 0.5],
            "travel_axis_body":    [0.0, 0.0, 1.0],
            "spring_stiffness_npm": 20000.0,
            "damper_coeff_nspm":    500.0,
            "preload_n":            0.0,
            "travel_max_m":         0.3,
            "tyre_radius_m":        0.2,
            "rolling_resistance_nd":0.02,
            "max_brake_torque_nm":  0.0,
            "is_steerable":         true,
            "has_brake":            false
        }]
    })");
    LandingGear gear;
    gear.initialize(config);
    FlatTestTerrain terrain{0.0f};

    const auto cf = gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}),
                               terrain, 0.2f, 0.0f, 0.0f, 0.02f);
    EXPECT_NE(cf.moment_body_nm.z(), 0.0f)
        << "Non-zero steering angle with forward velocity should produce yaw moment";
}

TEST(LandingGear, DifferentialBrake_ProducesYawMoment) {
    // Differential braking creates asymmetric longitudinal force (F_x) between left and
    // right main wheels, which produces a yaw moment (M_z).  To make the brake torque
    // (proportional to wheel speed) effective, wheel speeds are preset to free-roll via
    // serialization.  The brake is applied on the first of two steps so that the changed
    // wheel speed is reflected in the tyre force on the second step.
    LandingGear gear;
    gear.initialize(makeTricycleConfig());
    FlatTestTerrain terrain{0.0f};

    // One step to establish strut deflection at alt=0.5 m
    gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}), terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    // Override wheel speeds to free-roll: V_cx / r_w = 20 / 0.2 = 100 rad/s
    auto state_j = gear.serializeJson();
    for (auto& wu : state_j["wheel_units"])
        wu["wheel_speed_rps"] = 100.0f;

    // Symmetric run (no brake): apply for one step (updates omega), read on next step
    gear.deserializeJson(state_j);
    gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}), terrain, 0.0f, 0.0f, 0.0f, 0.02f);
    const auto cf_sym = gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}),
                                   terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    // Differential brake run: apply left brake on first step to change left-main wheel
    // speed, then read the resulting asymmetric F_x / M_z on the second step
    gear.deserializeJson(state_j);
    gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}), terrain, 0.0f, 1.0f, 0.0f, 0.02f);
    const auto cf_diff = gear.step(makeSnap(0.5f, {20.0f, 0.0f, 0.0f}),
                                    terrain, 0.0f, 0.0f, 0.0f, 0.02f);

    EXPECT_NE(cf_diff.moment_body_nm.z(), cf_sym.moment_body_nm.z())
        << "Differential braking must produce different yaw moment";
}
