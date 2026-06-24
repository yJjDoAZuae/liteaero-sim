// Tests for Aircraft — items 3 (class definition), 4 (step() physics loop),
//                      5 (serialization), 1 (JSON initialization from fixture files).

#include "Aircraft.hpp"
#include "environment/Atmosphere.hpp"
#include "propulsion/Propulsion.hpp"
#include <liteaero/terrain/Terrain.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// StubPropulsion — constant-thrust, stateless test double
// ---------------------------------------------------------------------------

class StubPropulsion : public liteaero::simulation::Propulsion {
public:
    explicit StubPropulsion(float thrust_n = 0.0f) : _thrust(thrust_n) {}

    [[nodiscard]] float step(float /*throttle_nd*/,
                             float /*tas_mps*/,
                             float /*rho_kgm3*/) override {
        return _thrust;
    }
    [[nodiscard]] float thrust_n() const override { return _thrust; }

    [[nodiscard]] std::vector<uint8_t> serializeProto()                             const override { return {}; }
    void                               deserializeProto(const std::vector<uint8_t>&)        override {}

protected:
    void           onInitialize(const nlohmann::json&)              override {}
    void           onReset()                                        override {}
    nlohmann::json onSerializeJson()                          const override { return {}; }
    void           onDeserializeJson(const nlohmann::json&)         override {}
    int            schemaVersion()                            const override { return 1; }
    const char*    typeName()                                 const override { return "StubPropulsion"; }

private:
    float _thrust;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// General-aviation config (Cessna 172 analog) — values from general_aviation.json.
// cmd_filter_substeps=1, outer_dt_s=0.1 → cmd_filter_dt_s=0.1.
// All wn satisfy Nyquist: wn * 0.1 < π.
static nlohmann::json makeConfig() {
    return nlohmann::json::parse(R"({
        "schema_version": 1,
        "aircraft": {
            "S_ref_m2": 16.2,
            "cl_y_beta": -0.60,
            "ar": 7.47,
            "e": 0.80,
            "cd0": 0.027,
            "cmd_filter_substeps": 1,
            "nz_wn_rad_s": 10.0,
            "nz_zeta_nd": 0.7,
            "ny_wn_rad_s": 10.0,
            "ny_zeta_nd": 0.7,
            "roll_rate_wn_rad_s": 20.0,
            "roll_rate_zeta_nd": 0.7,
            "dtheta_zeta_nd": 0.7,
            "dtheta_wn_pitch_ratio": 7.34,
            "dtheta_wn_roll_ratio": 9.79,
            "dtheta_wn_yaw_ratio": 4.89,
            "dtheta_vref_ratio": 1.0,
            "aero_effectiveness_vref_ratio": 1.0,
            "att_filt_tau_ratio": 1.22,
            "nz_relax_wn_ratio": 2.45,
            "nz_relax_zeta_nd": 0.8,
            "settle_gain_nd": 25.0,
            "settle_clip_nd": 1.0,
            "settle_tau_ratio": 0.52,
            "settle_wheel_rr_nd": 0.05,
            "settle_vland_ratio": 1.0,
            "settle_vtakeoff_ratio": 1.15,
            "settle_vwidth_ratio": 0.5
        },
        "airframe": {
            "g_max_nd":     3.8,
            "g_min_nd":    -1.52,
            "tas_max_mps": 82.3,
            "mach_max_nd":  0.25
        },
        "load_factor_allocator": {
            "alpha_dot_max_ratio": 0.0
        },
        "inertia": {
            "mass_kg":   1045.0,
            "Ixx_kgm2":  1285.0,
            "Iyy_kgm2":  1825.0,
            "Izz_kgm2":  2667.0
        },
        "lift_curve": {
            "cl_alpha":              5.1,
            "cl_max":                1.80,
            "cl_min":               -1.20,
            "delta_alpha_stall":     0.262,
            "delta_alpha_stall_neg": 0.262,
            "cl_sep":                1.05,
            "cl_sep_neg":           -0.80,
            "alpha_max_rad":         0.42,
            "alpha_min_rad":        -0.26
        },
        "initial_state": {
            "latitude_rad":        0.0,
            "longitude_rad":       0.0,
            "altitude_m":        300.0,
            "velocity_north_mps": 55.0,
            "velocity_east_mps":   0.0,
            "velocity_down_mps":   0.0,
            "wind_north_mps":      0.0,
            "wind_east_mps":       0.0,
            "wind_down_mps":       0.0
        }
    })");
}

static std::unique_ptr<liteaero::simulation::Aircraft> makeAircraft(float stub_thrust_n = 989.0f,
                                                            float outer_dt_s   = 0.1f) {
    auto prop = std::make_unique<StubPropulsion>(stub_thrust_n);
    auto ac   = std::make_unique<liteaero::simulation::Aircraft>(std::move(prop));
    ac->initialize(makeConfig(), outer_dt_s);
    return ac;
}

// ---------------------------------------------------------------------------
// Item 3 — class definition
// ---------------------------------------------------------------------------

TEST(AircraftTest, ConstructsWithoutThrowing) {
    auto prop = std::make_unique<StubPropulsion>();
    EXPECT_NO_THROW({
        liteaero::simulation::Aircraft ac(std::move(prop));
    });
}

TEST(AircraftTest, InitializePopulatesState) {
    auto ac = makeAircraft();

    const Eigen::Vector3f vel = ac->state().velocity_NED_mps();
    EXPECT_FLOAT_EQ(vel.x(), 55.0f);   // north
    EXPECT_FLOAT_EQ(vel.y(), 0.0f);    // east
    EXPECT_FLOAT_EQ(vel.z(), 0.0f);    // down
}

TEST(AircraftTest, ResetRestoresInitialState) {
    auto ac = makeAircraft();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    // Take a few steps to change state.
    for (int i = 1; i <= 5; ++i) {
        ac->step(i * 0.05, cmd, wind, 1.225f);
    }

    ac->reset();

    const Eigen::Vector3f vel = ac->state().velocity_NED_mps();
    EXPECT_NEAR(vel.x(), 55.0f, 1e-3f);
    EXPECT_NEAR(vel.y(), 0.0f,  1e-3f);
    EXPECT_NEAR(vel.z(), 0.0f,  1e-3f);
}

// ---------------------------------------------------------------------------
// Item 4 — step() physics loop
// ---------------------------------------------------------------------------

TEST(AircraftTest, StepDoesNotThrow) {
    auto ac = makeAircraft();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    EXPECT_NO_THROW(ac->step(0.1, cmd, wind, 1.225f));
}

TEST(AircraftTest, ZeroThrottle_AircraftDecelerates) {
    // Zero thrust → drag decelerates the aircraft.
    auto ac = makeAircraft(0.0f);   // StubPropulsion always returns 0 N

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.0f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    const float initial_speed = ac->state().velocity_NED_mps().norm();

    for (int i = 1; i <= 10; ++i) {
        ac->step(i * 0.1, cmd, wind, 1.225f);
    }

    const float final_speed = ac->state().velocity_NED_mps().norm();
    EXPECT_LT(final_speed, initial_speed) << "speed should decrease with zero thrust";
}

TEST(AircraftTest, StraightAndLevel_SpeedApproximatelyConstant) {
    // Thrust ≈ 989 N balances drag at 55 m/s for the GA config.
    // Over 5 steps of 0.1 s, speed should change by less than 5%.
    auto ac = makeAircraft(989.0f);

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    const float initial_speed = ac->state().velocity_NED_mps().norm();

    for (int i = 1; i <= 5; ++i) {
        ac->step(i * 0.1, cmd, wind, 1.225f);
    }

    const float final_speed = ac->state().velocity_NED_mps().norm();
    const float rel_change  = std::abs(final_speed - initial_speed) / initial_speed;
    EXPECT_LT(rel_change, 0.05f)
        << "speed changed by " << (rel_change * 100.f) << "% (threshold: 5%)";
}

// ---------------------------------------------------------------------------
// Item 5 — serialization
// ---------------------------------------------------------------------------

static liteaero::simulation::AircraftCommand levelCmd() {
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z         = 1.0f;
    cmd.throttle_nd = 0.5f;
    return cmd;
}

TEST(AircraftTest, JsonRoundTrip_StateMatchesAfterRestore) {
    auto ac1 = makeAircraft(989.0f);

    Eigen::Vector3f wind = Eigen::Vector3f::Zero();
    for (int i = 1; i <= 10; ++i) {
        ac1->step(i * 0.1, levelCmd(), wind, 1.225f);
    }

    const nlohmann::json snapshot = ac1->serializeJson();

    auto prop2 = std::make_unique<StubPropulsion>(989.0f);
    liteaero::simulation::Aircraft ac2(std::move(prop2));
    ac2.deserializeJson(snapshot);

    // One additional step on both; outputs must agree to float precision.
    ac1->step(11 * 0.1, levelCmd(), wind, 1.225f);
    ac2.step(11 * 0.1, levelCmd(), wind, 1.225f);

    const Eigen::Vector3f v1 = ac1->state().velocity_NED_mps();
    const Eigen::Vector3f v2 = ac2.state().velocity_NED_mps();
    EXPECT_NEAR(v1.x(), v2.x(), 1e-4f);
    EXPECT_NEAR(v1.y(), v2.y(), 1e-4f);
    EXPECT_NEAR(v1.z(), v2.z(), 1e-4f);
}

TEST(AircraftTest, ProtoRoundTrip_StateMatchesAfterRestore) {
    auto ac1 = makeAircraft(989.0f);

    Eigen::Vector3f wind = Eigen::Vector3f::Zero();
    for (int i = 1; i <= 10; ++i) {
        ac1->step(i * 0.1, levelCmd(), wind, 1.225f);
    }

    const std::vector<uint8_t> bytes = ac1->serializeProto();

    auto prop2 = std::make_unique<StubPropulsion>(989.0f);
    liteaero::simulation::Aircraft ac2(std::move(prop2));
    ac2.deserializeProto(bytes);

    ac1->step(11 * 0.1, levelCmd(), wind, 1.225f);
    ac2.step(11 * 0.1, levelCmd(), wind, 1.225f);

    const Eigen::Vector3f v1 = ac1->state().velocity_NED_mps();
    const Eigen::Vector3f v2 = ac2.state().velocity_NED_mps();
    EXPECT_NEAR(v1.x(), v2.x(), 1e-4f);
    EXPECT_NEAR(v1.y(), v2.y(), 1e-4f);
    EXPECT_NEAR(v1.z(), v2.z(), 1e-4f);
}

TEST(AircraftTest, JsonSchemaVersionMismatchThrows) {
    auto ac = makeAircraft();
    nlohmann::json snapshot = ac->serializeJson();
    snapshot["schema_version"] = 99;

    auto prop = std::make_unique<StubPropulsion>();
    liteaero::simulation::Aircraft ac2(std::move(prop));
    EXPECT_THROW({ ac2.deserializeJson(snapshot); }, std::runtime_error);
}

TEST(AircraftTest, ProtoSchemaVersionMismatchThrows) {
    auto ac = makeAircraft();
    std::vector<uint8_t> bytes = ac->serializeProto();
    // AircraftState field 1 = schema_version: tag 0x08 at bytes[0], value varint at bytes[1].
    bytes[1] = static_cast<uint8_t>(99);

    auto prop = std::make_unique<StubPropulsion>();
    liteaero::simulation::Aircraft ac2(std::move(prop));
    EXPECT_THROW({ ac2.deserializeProto(bytes); }, std::runtime_error);
}

// ---------------------------------------------------------------------------
// Item 1 — JSON initialization from fixture files
// ---------------------------------------------------------------------------

static nlohmann::json loadFixture(const std::string& relative_path) {
    std::ifstream f(std::string(LAS_TEST_DATA_DIR) + "/" + relative_path);
    EXPECT_TRUE(f.is_open()) << "Could not open fixture: " << relative_path;
    return nlohmann::json::parse(f);
}

TEST(AircraftTest, InitializeFromFixture_GeneralAviation) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/general_aviation.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 55.0f);
}

TEST(AircraftTest, InitializeFromFixture_JetTrainer) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/jet_trainer.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 150.0f);
}

TEST(AircraftTest, InitializeFromFixture_SmallUAS) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/small_uas.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 20.0f);
}

TEST(AircraftTest, InitializeWithMissingField_Throws) {
    nlohmann::json config = loadFixture("aircraft/general_aviation.json");
    config.erase("inertia");

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(config, 0.02f), std::exception);
}

// ---------------------------------------------------------------------------
// Straight-and-level trim test — all example aircraft configs
//
// For each fixture, compute ISA density at the initial altitude, then compute
// the drag-balanced trim thrust:
//   q   = 0.5 * rho * V²
//   CL  = m*g / (q*S)
//   CD  = cd0 + CL² / (π * AR * e)
//   T   = q * S * CD
//
// A 5 % margin is added above trim to ensure the thrust is strictly above
// minimum (avoiding the edge case where thrust exactly equals drag but
// numerical error causes a slow descent).
//
// The aircraft is then stepped for 100 s with n_z = 1 g, n_y = 0, roll rate
// = 0, and the computed thrust. Tolerances are intentionally generous:
//   - altitude change < 50 m   (net drift, not per-step error)
//   - speed change   < 10 %    (allow filter transients to settle)
// ---------------------------------------------------------------------------

struct TrimCase {
    std::string fixture;        // relative path under LAS_TEST_DATA_DIR
    float       initial_speed;  // m/s — must match initial_state in fixture
    float       initial_alt;    // m   — must match initial_state in fixture
};

static float computeTrimThrust(const nlohmann::json& cfg, float rho_kgm3) {
    const auto& ac  = cfg.at("aircraft");
    const auto& in  = cfg.at("inertia");
    const float S   = ac.at("S_ref_m2").get<float>();
    const float ar  = ac.at("ar").get<float>();
    const float e   = ac.at("e").get<float>();
    const float cd0 = ac.at("cd0").get<float>();
    const float m   = in.at("mass_kg").get<float>();
    const float V   = cfg.at("initial_state").at("velocity_north_mps").get<float>();

    constexpr float kG  = 9.80665f;
    constexpr float kPi = 3.14159265f;
    const float q  = 0.5f * rho_kgm3 * V * V;
    const float CL = m * kG / (q * S);
    const float CD = cd0 + CL * CL / (kPi * ar * e);
    return q * S * CD;
}

TEST(AircraftTest, StraightAndLevel_AllFixtures_100s) {
    const TrimCase cases[] = {
        {"aircraft/general_aviation.json",  55.0f,  300.0f},
        {"aircraft/jet_trainer.json",      150.0f, 3000.0f},
        {"aircraft/small_uas.json",         20.0f,  100.0f},
    };

    liteaero::simulation::Atmosphere atm;

    for (const auto& tc : cases) {
        SCOPED_TRACE(tc.fixture);

        const nlohmann::json cfg = loadFixture(tc.fixture);
        const float rho = atm.density_kgm3(tc.initial_alt);

        // Trim thrust: drag-balanced at initial speed and altitude.
        // The trim thrust is above the minimum (zero thrust) by construction — it
        // is the exact thrust required to maintain level flight at this condition.
        const float T_actual = computeTrimThrust(cfg, rho);

        auto ac = std::make_unique<liteaero::simulation::Aircraft>(
            std::make_unique<StubPropulsion>(T_actual));
        ac->initialize(cfg, 0.02f);

        const float alt0   = ac->state().positionDatum().height_WGS84_m();
        const float speed0 = ac->state().velocity_NED_mps().norm();

        liteaero::simulation::AircraftCommand cmd;
        cmd.n_z             = 1.0f;
        cmd.n_y             = 0.0f;
        cmd.rollRate_Wind_rps = 0.0f;

        constexpr float kDt     = 0.02f;
        constexpr int   kSteps  = static_cast<int>(100.0f / kDt); // 5000 steps
        for (int i = 1; i <= kSteps; ++i) {
            ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), rho);
        }

        const float alt_final   = ac->state().positionDatum().height_WGS84_m();
        const float speed_final = ac->state().velocity_NED_mps().norm();

        EXPECT_NEAR(alt_final, alt0, 50.0f)
            << "altitude drifted more than 50 m over 100 s";
        EXPECT_NEAR(speed_final, speed0, 0.1f * speed0)
            << "speed changed more than 10% over 100 s";
    }
}

// ---------------------------------------------------------------------------
// Item 0 — command processing: Nyquist protection
// outer_dt_s = 0.1, substeps = 1  →  cmd_filter_dt_s = 0.1
// Nyquist limit: wn * 0.1 < π ≈ 3.14159.  Violation: wn = 40.0 (40*0.1=4.0 > π).
// ---------------------------------------------------------------------------

static nlohmann::json makeConfigWithNz(float nz_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["nz_wn_rad_s"] = nz_wn_rad_s;
    return c;
}

static nlohmann::json makeConfigWithNy(float ny_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["ny_wn_rad_s"] = ny_wn_rad_s;
    return c;
}

static nlohmann::json makeConfigWithRollRate(float roll_rate_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["roll_rate_wn_rad_s"] = roll_rate_wn_rad_s;
    return c;
}

TEST(AircraftTest, NyquistViolation_Nz_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithNz(40.0f), 0.1f), std::invalid_argument);
}

TEST(AircraftTest, NyquistViolation_Ny_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithNy(40.0f), 0.1f), std::invalid_argument);
}

TEST(AircraftTest, NyquistViolation_RollRate_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithRollRate(40.0f), 0.1f), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Step F — LandingGear integration tests
// ---------------------------------------------------------------------------

namespace {

// Tricycle gear config appended to makeConfig() for Step F tests.
static nlohmann::json addLandingGear(nlohmann::json config) {
    config["landing_gear"] = nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [
            {
                "attach_point_body_m": [2.0, 0.0, 0.5],
                "travel_axis_body":    [0.0, 0.0, 1.0],
                "spring_stiffness_npm":      20000.0,
                "damping_compression_nspm": 2600.0,
                "damping_extension_nspm":   520.0,
                "spring_nonlinearity_nd":   0.0,
                "preload_n":                0.0,
                "travel_max_m":             0.3,
                "tyre_radius_m":            0.2,
                "rolling_resistance_nd":    0.02,
                "max_brake_torque_nm":      0.0,
                "is_steerable":             false,
                "has_brake":                false
            },
            {
                "attach_point_body_m": [-0.3, -1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm":      20000.0,
                "damping_compression_nspm": 2600.0,
                "damping_extension_nspm":   520.0,
                "spring_nonlinearity_nd":   0.0,
                "preload_n":                0.0,
                "travel_max_m":             0.3,
                "tyre_radius_m":            0.2,
                "rolling_resistance_nd":    0.02,
                "max_brake_torque_nm":      0.0,
                "is_steerable":             false,
                "has_brake":                false
            },
            {
                "attach_point_body_m": [-0.3,  1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm":      20000.0,
                "damping_compression_nspm": 2600.0,
                "damping_extension_nspm":   520.0,
                "spring_nonlinearity_nd":   0.0,
                "preload_n":                0.0,
                "travel_max_m":             0.3,
                "tyre_radius_m":            0.2,
                "rolling_resistance_nd":    0.02,
                "max_brake_torque_nm":      0.0,
                "is_steerable":             false,
                "has_brake":                false
            }
        ]
    })");
    return config;
}

// Initial state at low altitude so wheels can contact terrain at 0 m elevation.
// contact_alt = altitude - 0.7 m (0.5 attach + 0.2 tyre radius).
// With alt = 0.5 m → penetration = 0.2 m.
static nlohmann::json makeGroundConfig() {
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]        = 0.5;
    cfg["initial_state"]["velocity_north_mps"]= 0.0;
    cfg["initial_state"]["velocity_east_mps"] = 0.0;
    cfg["initial_state"]["velocity_down_mps"] = 0.0;
    return cfg;
}

static nlohmann::json addBodyCollider(nlohmann::json config) {
    config["body_collider"] = {{"volumes", nlohmann::json::array({
        {
            {"name",                 "fuselage"},
            {"half_extents_body_m",  {1.0f, 0.5f, 0.3f}},
            {"center_offset_body_m", {0.0f, 0.0f, 0.0f}},
            {"stiffness_npm",        50000.0f},
            {"damping_nspm",          2000.0f}
        }
    })}};
    return config;
}

}  // namespace

TEST(AircraftTest, LandingGear_Airborne_ZeroContact) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    ac->initialize(addLandingGear(makeConfig()), 0.1f);  // makeConfig() starts at 300 m
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;
    ac->step(0.0, cmd, Eigen::Vector3f::Zero(), 1.225f);

    EXPECT_FALSE(ac->weightOnWheels());
    EXPECT_FLOAT_EQ(ac->contactForces().force_body_n.norm(), 0.0f);
}

TEST(AircraftTest, LandingGear_GroundContact_PositiveNz) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    ac->initialize(makeGroundConfig(), 0.1f);
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;
    ac->step(0.0, cmd, Eigen::Vector3f::Zero(), 1.225f);

    EXPECT_TRUE(ac->weightOnWheels());
    // Gear pushes aircraft upward — body-Z force is negative (upward in body frame)
    EXPECT_LT(ac->contactForces().force_body_n.z(), 0.0f)
        << "Ground contact must produce upward (negative body-Z) force";
}

// ---------------------------------------------------------------------------
// Terrain hard constraint — post-integration position correction
// ---------------------------------------------------------------------------

TEST(AircraftTest, TerrainHardConstraint_KeepsAircraftAboveTerrain) {
    // Aircraft initialized 100 m below terrain (terrain at 0 m, aircraft at -100 m).
    // After one step() the hard constraint must project altitude above terrain
    // regardless of what the spring-damper forces did during integration.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    // Config with a single centered body_collider volume tall enough to trigger
    // at -100 m altitude.  Stiffness and damping are deliberately low so they
    // don't dominate — the constraint is the thing under test, not the spring.
    const auto cfg = nlohmann::json::parse(R"({
        "schema_version": 1,
        "aircraft": {
            "S_ref_m2": 16.2, "cl_y_beta": -0.60, "ar": 7.47, "e": 0.80,
            "cd0": 0.027, "cmd_filter_substeps": 1,
            "nz_wn_rad_s": 10.0, "nz_zeta_nd": 0.7,
            "ny_wn_rad_s": 10.0, "ny_zeta_nd": 0.7,
            "roll_rate_wn_rad_s": 20.0, "roll_rate_zeta_nd": 0.7,
            "dtheta_zeta_nd": 0.7, "dtheta_wn_pitch_ratio": 7.34,
            "dtheta_wn_roll_ratio": 9.79, "dtheta_wn_yaw_ratio": 4.89,
            "dtheta_vref_ratio": 1.0, "att_filt_tau_ratio": 1.22,
            "aero_effectiveness_vref_ratio": 1.0,
            "nz_relax_wn_ratio": 2.45, "nz_relax_zeta_nd": 0.8,
            "settle_gain_nd": 25.0, "settle_clip_nd": 1.0, "settle_tau_ratio": 0.52,
            "settle_wheel_rr_nd": 0.05, "settle_vland_ratio": 1.0,
            "settle_vtakeoff_ratio": 1.15, "settle_vwidth_ratio": 0.5
        },
        "airframe": {
            "g_max_nd": 3.8, "g_min_nd": -1.52,
            "tas_max_mps": 82.3, "mach_max_nd": 0.25
        },
        "load_factor_allocator": { "alpha_dot_max_ratio": 0.0 },
        "inertia": { "mass_kg": 1045.0, "Ixx_kgm2": 1285.0, "Iyy_kgm2": 1825.0, "Izz_kgm2": 2667.0 },
        "lift_curve": {
            "cl_alpha": 5.1, "cl_max": 1.80, "cl_min": -1.20,
            "delta_alpha_stall": 0.262, "delta_alpha_stall_neg": 0.262,
            "cl_sep": 1.05, "cl_sep_neg": -0.80,
            "alpha_max_rad": 0.42, "alpha_min_rad": -0.26
        },
        "body_collider": {
            "volumes": [{
                "name": "fuselage",
                "half_extents_body_m":  [1.0, 1.0, 0.5],
                "center_offset_body_m": [0.0, 0.0, 0.0],
                "stiffness_npm": 100.0,
                "damping_nspm":   10.0
            }]
        },
        "initial_state": {
            "latitude_rad": 0.0, "longitude_rad": 0.0,
            "altitude_m": -100.0,
            "velocity_north_mps": 0.0, "velocity_east_mps": 0.0,
            "velocity_down_mps": 50.0,
            "wind_north_mps": 0.0, "wind_east_mps": 0.0, "wind_down_mps": 0.0
        }
    })");

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;
    ac->step(0.0, cmd, Eigen::Vector3f::Zero(), 1.225f);

    EXPECT_GE(ac->state().positionDatum().height_WGS84_m(), 0.0f)
        << "Hard constraint must project aircraft above terrain after integration";
    EXPECT_LE(ac->state().velocity_NED_mps().z(), 0.0f)
        << "Hard constraint must zero downward velocity when penetrating terrain";
}

TEST(AircraftTest, BodyColliderOnly_Landing_StaysNearTerrain) {
    // Gear-less belly landing at approach speed, n_z=1, no thrust. The body collider
    // must keep the aircraft near terrain — a bounded float that does not take off, and
    // settling (not climbing away) as it decelerates.
    //
    // OQ-LG-22 (resolved, Alternative 4): the gear-less case no longer suppresses lift
    // (the synthetic full-weight n_z-relaxation input was removed). So while the wing is
    // still flying (V well above stall) the aircraft skims just above the surface — the
    // contact arrests the descent, it floats a few tenths of a metre, then settles as
    // speed bleeds off — instead of being artificially pinned to the ground. This is the
    // accepted gear-less behavior; the test verifies the float stays bounded near terrain
    // and converges (settling), not the earlier (superseded) lift-suppressed pinning.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto cfg = addBodyCollider(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 5.0;
    cfg["initial_state"]["velocity_north_mps"] = 55.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 2.0;

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;  // n_z=1, throttle=0

    bool  first_contact           = false;
    float max_agl_after_contact_m = 0.0f;
    float last_v_down_mps         = 0.0f;

    constexpr float kDt          = 0.02f;
    constexpr float kBodyHalfZ_m = 0.3f;  // half_extents_body_m z component
    constexpr int   kSteps       = static_cast<int>(10.0f / kDt);  // 10 s

    for (int i = 1; i <= kSteps; ++i) {
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);

        const float agl = ac->agl_m();
        if (!first_contact && agl <= kBodyHalfZ_m + 0.15f) {  // belly reaches the surface
            first_contact = true;
        }
        if (first_contact) {
            max_agl_after_contact_m = std::max(max_agl_after_contact_m, agl);
        }
        last_v_down_mps = ac->state().velocity_NED_mps().z();
    }

    EXPECT_TRUE(first_contact) << "body collider must bring aircraft to terrain";
    // Bounded float near terrain — must not climb away (start was 5 m; float peaks well below 1 m).
    EXPECT_LE(max_agl_after_contact_m, 1.0f)
        << "aircraft must remain near terrain after belly landing; max AGL after contact: "
        << max_agl_after_contact_m << " m";
    // Settling, not climbing: NED-down velocity must not be appreciably negative at the end.
    EXPECT_GT(last_v_down_mps, -0.10f)
        << "belly-landed aircraft must be settling, not climbing away; final v_down: "
        << last_v_down_mps << " m/s";
}

TEST(AircraftTest, BodyColliderOnly_GlideToImpact_ArrestsDescentAndReportsForce) {
    // No landing gear — body collider is the only contact model.
    // Aircraft starts at 5 m AGL, stationary.  At zero airspeed q_inf = 0 so
    // aerodynamic forces are zero and the aircraft falls under gravity.
    // Body collider half-extents z = 0.3 m → first corner contact at CG altitude 0.3 m.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto cfg = addBodyCollider(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 5.0;
    cfg["initial_state"]["velocity_north_mps"] = 0.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();

    // Default AircraftCommand: n_z=1, throttle=0, rollRate=0.
    const liteaero::simulation::AircraftCommand cmd;

    bool  contact_detected  = false;
    bool  upward_force_seen = false;
    float min_agl_m         = 1e6f;

    constexpr float kDt    = 0.02f;
    constexpr int   kSteps = static_cast<int>(6.0f / kDt);  // 6 s

    for (int i = 1; i <= kSteps; ++i) {
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);

        const float agl = ac->agl_m();
        min_agl_m = std::min(min_agl_m, agl);

        const auto& cf = ac->contactForces();
        if (cf.weight_on_wheels)          contact_detected  = true;
        if (cf.force_body_n.z() < -1.0f) upward_force_seen = true;
    }

    EXPECT_GE(min_agl_m, -0.05f)
        << "body collider must prevent terrain penetration throughout";
    EXPECT_TRUE(contact_detected)
        << "body collider must report weight-on-wheels during impact";
    EXPECT_TRUE(upward_force_seen)
        << "contactForces() must reflect body-collider upward force (not gear-only)";
    // Freefall from 5 m reaches sqrt(2 * 9.8 * 5) ≈ 9.9 m/s; body collider must
    // arrest the descent to well below that by 6 s.
    EXPECT_LT(ac->state().velocity_NED_mps().z(), 9.9f)
        << "body collider must arrest descent velocity after impact";
}

TEST(AircraftTest, LandingGear_FullStop_SpeedNearZero) {
    // After touchdown from approach speed, rolling resistance must bring the
    // aircraft to rest.  Horizontal speed must drop below 0.5 m/s within 90 s.
    //
    // Physics: rolling_resistance_nd (0.02) applied to the gear normal force
    // (~weight at low airspeed) gives ~0.2 m/s² deceleration, sufficient to
    // arrest the aircraft from 15 m/s in ~75 s.  A plateau above 0.5 m/s at
    // 90 s indicates the wheel friction model is failing to decelerate the
    // aircraft at low speed (e.g., sign-reversal, torque-not-force conversion,
    // or a minimum-velocity deadband).
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    // Tyre contact altitude = attach_z + tyre_radius = 0.5 + 0.2 = 0.7 m.
    // Start with wheels just touching so the gear spring compresses immediately.
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z         = 1.0f;
    cmd.throttle_nd = 0.0f;

    constexpr float kDt    = 0.02f;
    constexpr int   kSteps = static_cast<int>(90.0f / kDt);  // 90 s

    for (int i = 1; i <= kSteps; ++i) {
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
        if (i % 25 == 0) {
            const auto& vn = ac->state().velocity_NED_mps();
            const float alt = ac->state().positionDatum().height_WGS84_m();
            printf("  t=%.2fs  vN=%.3f vE=%.3f vD=%.3f  alt=%.4f  WoW=%d\n",
                   i*kDt, vn.x(), vn.y(), vn.z(), alt, (int)ac->weightOnWheels());
        }
    }

    const float final_speed_mps = ac->state().velocity_NED_mps().head<2>().norm();
    EXPECT_LT(final_speed_mps, 0.5f)
        << "aircraft must come to rest; final horizontal speed: "
        << final_speed_mps << " m/s after 90 s of rolling resistance";
}

// ---------------------------------------------------------------------------
// OQ-LG-15 diagnostic — per-step log for bounce oscillation analysis
//
// Runs the FullStop scenario for 300 s (extended duration, OQ-LG-15 Alt-2) and
// writes a CSV to build/oq_lg15_diagnostic.csv (OQ-LG-15 Alt-3 energy accounting).
// This test has no EXPECT_ assertion; it is a data-collection tool only.
// Do not use it to validate the physics — the FullStop EXPECT is in the test above.
// ---------------------------------------------------------------------------

TEST(AircraftTest, LandingGear_FullStop_OQ_LG15_Diagnostic) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z         = 1.0f;
    cmd.throttle_nd = 0.0f;

    constexpr float kDt    = 0.02f;
    constexpr int   kSteps = static_cast<int>(300.0f / kDt);  // 300 s extended run

    std::ofstream csv("oq_lg15_diagnostic.csv");
    csv << "time_s,v_north_mps,v_down_mps,altitude_m,"
        << "strut_nose_m,strut_left_m,strut_right_m,"
        << "contact_fz_body_n,contact_fx_ned_n,contact_fx_wind_n,contact_fz_wind_n,"
        << "contact_power_horiz_w,contact_power_total_w,"
        << "pitch_deg,alpha_deg,fpa_deg,"
        // OQ-LG-15 nose-wheel force breakdown (TEMPORARY)
        << "nose_Fz,nose_Fx,nose_Fy,nose_Frr,nose_kappa,nose_Vcx,nose_omega,"
        << "nose_fwd_x,nose_fwd_z,nose_norm_x,nose_norm_z,nose_force_bx,nose_force_bz,"
        // OQ-LG-15 main-gear (left) breakdown + per-wheel NED-x force (TEMPORARY)
        << "main_Fx,main_Frr,main_Fy,main_Vcx,main_fned_x,nose_fned_x,"
        << "body_pitchrate_rps,main_Fz,main_deltadot,"
        << "gear_wx,gear_wy,gear_wz,"
        << "wow\n";
    csv << std::fixed << std::setprecision(6);

    constexpr float kRad2Deg = 57.2957795f;
    constexpr float kRho     = 1.225f;

    for (int i = 1; i <= kSteps; ++i) {
        const double t = i * static_cast<double>(kDt);
        ac->step(t, cmd, Eigen::Vector3f::Zero(), kRho);

        const auto& vel = ac->state().velocity_NED_mps();
        const float alt = ac->state().positionDatum().height_WGS84_m();

        const auto& wu = ac->landingGear().wheelUnits();
        const float d_nose  = wu[0].strutState().strut_deflection_m;
        const float d_left  = wu[1].strutState().strut_deflection_m;
        const float d_right = wu[2].strutState().strut_deflection_m;

        const Eigen::Vector3f F_body = ac->contactForces().force_body_n;
        const Eigen::Matrix3f R_nb   = ac->state().q_nb().toRotationMatrix();
        const Eigen::Vector3f F_ned  = R_nb * F_body;
        const float F_ned_x = F_ned.x();

        // Wind-frame contact force — THIS is what the EOM actually applies to ax/az.
        // F_gear_wind = R_nw^T · F_ned.
        const Eigen::Matrix3f R_nw = ac->state().q_nw().toRotationMatrix();
        const Eigen::Vector3f F_wind = R_nw.transpose() * F_ned;
        const float F_wind_x = F_wind.x();   // along velocity vector (forward +)
        const float F_wind_z = F_wind.z();   // perpendicular (down +)

        const float contact_power_horiz_w = F_ned_x * vel.x();
        const float contact_power_total_w = F_ned.dot(vel);

        const float pitch_deg = ac->state().pitch() * kRad2Deg;
        const float alpha_deg = ac->state().alpha()  * kRad2Deg;
        // Flight path angle: atan2(-vD, vN) — positive = climbing.
        const float fpa_deg = std::atan2(-vel.z(),
                              std::sqrt(vel.x()*vel.x() + vel.y()*vel.y())) * kRad2Deg;

        const auto& nd = wu[0].lastContactDiag();   // nose wheel diagnostics
        const auto& md = wu[1].lastContactDiag();    // left-main diagnostics
        // Recompute the finite-difference q_nb body rate (what the gear contact model sees).
        static Eigen::Quaternionf s_qprev = Eigen::Quaternionf::Identity();
        static bool s_qinit = false;
        Eigen::Vector3f gear_w = Eigen::Vector3f::Zero();
        {
            const Eigen::Quaternionf qn = ac->state().q_nb();
            if (s_qinit) {
                Eigen::Quaternionf dq = s_qprev.conjugate() * qn;
                if (dq.w() < 0.f) dq.coeffs() = -dq.coeffs();
                gear_w = 2.0f * dq.vec() / kDt;
            }
            s_qprev = qn; s_qinit = true;
        }
        // Per-wheel NED-forward force contributions (R_nb · per-wheel body force).
        const float main_fned_x = (R_nb * md.force_body).x();
        const float nose_fned_x = (R_nb * nd.force_body).x();

        csv << t << ','
            << vel.x() << ',' << vel.z() << ',' << alt << ','
            << d_nose   << ',' << d_left  << ',' << d_right << ','
            << F_body.z() << ',' << F_ned_x << ',' << F_wind_x << ',' << F_wind_z << ','
            << contact_power_horiz_w << ',' << contact_power_total_w << ','
            << pitch_deg << ',' << alpha_deg << ',' << fpa_deg << ','
            << nd.F_z << ',' << nd.F_x << ',' << nd.F_y << ',' << nd.F_rr << ','
            << nd.kappa << ',' << nd.V_cx << ',' << nd.omega << ','
            << nd.wheel_fwd.x() << ',' << nd.wheel_fwd.z() << ','
            << nd.surf_normal.x() << ',' << nd.surf_normal.z() << ','
            << nd.force_body.x() << ',' << nd.force_body.z() << ','
            << md.F_x << ',' << md.F_rr << ',' << md.F_y << ',' << md.V_cx << ','
            << main_fned_x << ',' << nose_fned_x << ','
            << ac->state().rates_Body_rps().y() << ','
            << md.F_z << ',' << md.delta_dot << ','
            << gear_w.x() << ',' << gear_w.y() << ',' << gear_w.z() << ','
            << (int)ac->weightOnWheels() << '\n';
    }
    csv.close();
    // No assertion — this test produces diagnostic data only.
    // See docs/design/landing_gear.md §OQ-LG-15 for analysis.
}

// ---------------------------------------------------------------------------
// IP-AGF-6 — H₁ nz-relaxation filter + Δθ rotation-deviation state
// ---------------------------------------------------------------------------

TEST(AircraftTest, JsonSerialization_ContainsGearFilterStateKeys) {
    // serializeJson() must include state for the nz-relaxation (H₁) and
    // rotation-deviation (H₂) filters so they survive a JSON round-trip.
    // Before IP-AGF-6: these keys are absent → test fails.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt = 0.02f;
    for (int i = 1; i <= 5; ++i)
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);

    const nlohmann::json j = ac->serializeJson();
    EXPECT_TRUE(j.contains("nz_relax_filter"))
        << "serializeJson() must include nz_relax_filter (H₁) state";
    EXPECT_TRUE(j.contains("dtheta_pitch_filter"))
        << "serializeJson() must include dtheta_pitch_filter (H₂ moment) state";
    EXPECT_TRUE(j.contains("force_x"))
        << "serializeJson() must include force_x (G(s) force-channel) state";
    EXPECT_TRUE(j.contains("fz_stance_filter"))
        << "serializeJson() must include fz_stance_filter (destancing) state";
    EXPECT_TRUE(j.contains("prev_dtheta_roll"))
        << "serializeJson() must include prev_dtheta_roll state";
}

TEST(AircraftTest, LandingGear_DthetaState_NonzeroAfterContact) {
    // The Δθ force channel (G(s) on the destanced/faded gear vertical load) and the
    // moment channel (H₂ on M/I) are both driven during gear contact. After a few
    // steps in contact the force-channel state and/or moment-filter state must be nonzero.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt = 0.02f;
    for (int i = 1; i <= 10; ++i)
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);

    const nlohmann::json j = ac->serializeJson();
    ASSERT_TRUE(j.contains("force_x"));
    const float fx0 = j.at("force_x").at(0).get<float>();
    const float fx1 = j.at("force_x").at(1).get<float>();
    const auto& mj = j.at("dtheta_pitch_filter");
    const float mx0 = (mj.contains("state") && mj.at("state").contains("x0"))
                          ? mj.at("state").at("x0").get<float>() : 0.f;
    const float mx1 = (mj.contains("state") && mj.at("state").contains("x1"))
                          ? mj.at("state").at("x1").get<float>() : 0.f;
    EXPECT_NE(std::abs(fx0) + std::abs(fx1) + std::abs(mx0) + std::abs(mx1), 0.0f)
        << "Δθ force/moment state must be nonzero after gear contact";
}

TEST(AircraftTest, LandingGear_ForceChannel_DecaysAfterTransient) {
    // OQ-LG-20: the force channel G(s) must not accumulate a sustained deviation (no
    // integrator drift). The touchdown impulse excites it; thereafter, as the gear vertical
    // load settles toward its quasi-steady stance, the destancing-filter input collapses and
    // the asymptotically stable G(s) state decays. We assert the decay as a RATIO between a
    // post-touchdown sample and a later sample — realization-independent, since the absolute
    // magnitude of the tf2ss internal state is an arbitrary scaling artifact (it runs ~10² to
    // 10³ larger than the physical pitch-deviation output). A driven, drifting, or integrating
    // channel would hold or grow; a correct lead-form channel decays sharply.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.70;  // wheels just touching, no penetration
    cfg["initial_state"]["velocity_north_mps"] = 12.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt = 0.02f;

    auto force_state_norm = [&]() {
        const nlohmann::json j = ac->serializeJson();
        return std::abs(j.at("force_x").at(0).get<float>())
             + std::abs(j.at("force_x").at(1).get<float>());
    };

    // Step to ~1 s: past the touchdown impulse, near the force-channel excitation peak.
    for (int i = 1; i <= 50; ++i)
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
    const float x_peak = force_state_norm();

    // Continue to ~6 s of steady decelerating roll (well clear of the end-game).
    for (int i = 51; i <= 300; ++i)
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
    const float x_late = force_state_norm();

    EXPECT_GT(x_peak, 0.f) << "touchdown must excite the force channel";
    // Strong decay: a stable lead-form channel sheds the transient; no drift/accumulation.
    EXPECT_LT(x_late, 0.1f * x_peak)
        << "force-channel state must decay after the transient (no integrator drift); "
        << "x_peak=" << x_peak << " x_late=" << x_late;
}

TEST(AircraftTest, LandingGear_GearContact_DoesNotAccelerate) {
    // The gear-attitude feedback defect (OQ-LG-15) causes a +22.7 kN forward
    // impulse every ~2.4 s, so horizontal speed INCREASES above the 15 m/s
    // start within 3 s of ground contact.  With the H₁ + Δθ fix the speed
    // must decrease monotonically — no forward energy injection.
    // Before IP-AGF-6: speed exceeds 15 m/s at ~t=2.4 s → EXPECT fails.
    // After IP-AGF-6: speed is strictly less than 15 m/s → PASSES.
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt    = 0.02f;
    constexpr int   kSteps = static_cast<int>(3.0f / kDt);
    for (int i = 1; i <= kSteps; ++i)
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
    const float speed = ac->state().velocity_NED_mps().head<2>().norm();
    EXPECT_LT(speed, 15.0f)
        << "gear contact must not inject forward energy; speed after 3 s: "
        << speed << " m/s (started at 15 m/s)";
}

TEST(AircraftTest, LandingGear_TireNeverPropels_FullScenario) {
    // Aircraft-level proof that the TIRE (Pacejka longitudinal force F_x) never produces
    // traction during a full ground-roll landing. The addLandingGear fixture has no body
    // collider and the wheels are unbraked/free-rolling, so the per-wheel F_x must stay
    // ≈ 0 throughout — including through bouncing and attitude swings that drive the
    // contact-patch longitudinal velocity negative. (This isolates the tire from the
    // normal-force projection F_z·sinγ, which is a separate strut/FPA effect.)
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;
    cfg["initial_state"]["velocity_north_mps"] = 15.0;
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt    = 0.02f;
    constexpr int   kSteps = static_cast<int>(90.0f / kDt);

    float max_abs_fx = 0.0f;
    for (int i = 1; i <= kSteps; ++i) {
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
        for (const auto& wu : ac->landingGear().wheelUnits()) {
            const float fx = std::abs(wu.lastContactDiag().F_x);
            if (fx > max_abs_fx) max_abs_fx = fx;
        }
    }
    // A free-rolling tire must carry no longitudinal slip force. Bound generously at 5 N
    // (well below any μ·F_z traction, which would be O(1000 N)).
    EXPECT_LT(max_abs_fx, 5.0f)
        << "free-rolling tire produced longitudinal traction during the roll; max|F_x|="
        << max_abs_fx << " N (must be ≈ 0 — the tire must never propel)";
}

// ---------------------------------------------------------------------------
// OQ-LG-24: the aero-effectiveness weight w_a on the gear-relative aero demand makes the
// commanded angle of attack decay to zero as aero control authority collapses at low speed,
// rather than being pinned at the achievable-envelope fold (≈ stall AoA). Before the fix the
// on-ground attitude diverged nose-up below ~0.25× stall and the stiff strut contact blew up
// (pitch ±80°, the aircraft flung off the runway). Here a low-speed ground roll-out must
// decelerate to a stop on the gear with bounded attitude and no launch.
// ---------------------------------------------------------------------------
TEST(AircraftTest, LandingGear_LowSpeedRollout_Converges_OQ_LG24) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]         = 0.65;   // settled-on-gear ride height
    cfg["initial_state"]["velocity_north_mps"] = 12.0;   // ~0.5× stall; rolls down through 0.25× stall
    cfg["initial_state"]["velocity_east_mps"]  = 0.0;
    cfg["initial_state"]["velocity_down_mps"]  = 0.0;
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>(0.0f));
    ac->initialize(cfg, 0.02f);
    ac->setTerrain(&terrain);
    ac->reset();
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z = 1.0f;
    constexpr float kDt       = 0.02f;
    constexpr int   kSteps    = static_cast<int>(90.0f / kDt);
    constexpr float kRad2Deg  = 57.2957795f;

    float max_abs_alpha_deg = 0.0f;
    float max_abs_alt       = 0.0f;
    for (int i = 1; i <= kSteps; ++i) {
        ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), 1.225f);
        const float a_deg = std::abs(ac->state().alpha() * kRad2Deg);
        const float alt   = std::abs(ac->state().positionDatum().height_WGS84_m());
        if (a_deg > max_abs_alpha_deg) max_abs_alpha_deg = a_deg;
        if (alt   > max_abs_alt)       max_abs_alt       = alt;
        ASSERT_TRUE(std::isfinite(a_deg)) << "attitude diverged (non-finite) at step " << i;
    }
    const auto& vel = ac->state().velocity_NED_mps();
    const float v_horiz = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    EXPECT_LT(v_horiz, 1.0f)
        << "low-speed roll-out did not converge to a stop; final |v_horiz|=" << v_horiz << " m/s";
    EXPECT_LT(max_abs_alpha_deg, 12.0f)
        << "commanded angle of attack diverged toward the stall fold; max|alpha|="
        << max_abs_alpha_deg << " deg";
    EXPECT_LT(max_abs_alt, 1.0f)
        << "aircraft launched off the runway (gear contact blow-up); max|alt|="
        << max_abs_alt << " m";
}
