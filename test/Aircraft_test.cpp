// Tests for Aircraft — items 3 (class definition), 4 (step() physics loop),
//                      5 (serialization), 1 (JSON initialization from fixture files).

#include "Aircraft.hpp"
#include "propulsion/Propulsion.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// StubPropulsion — constant-thrust, stateless test double
// ---------------------------------------------------------------------------

class StubPropulsion : public liteaerosim::propulsion::Propulsion {
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
static nlohmann::json makeConfig() {
    return nlohmann::json::parse(R"({
        "schema_version": 1,
        "aircraft": {
            "S_ref_m2": 16.2,
            "cl_y_beta": -0.60,
            "ar": 7.47,
            "e": 0.80,
            "cd0": 0.027
        },
        "airframe": {
            "g_max_nd":    3.8,
            "g_min_nd":   -1.52,
            "tas_max_mps": 82.3,
            "mach_max_nd": 0.25
        },
        "inertia": {
            "mass_kg":   1045.0,
            "Ixx_kgm2":  1285.0,
            "Iyy_kgm2":  1825.0,
            "Izz_kgm2":  2667.0
        },
        "lift_curve": {
            "cl_alpha":             5.1,
            "cl_max":               1.80,
            "cl_min":              -1.20,
            "delta_alpha_stall":    0.262,
            "delta_alpha_stall_neg":0.262,
            "cl_sep":               1.05,
            "cl_sep_neg":          -0.80
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

static std::unique_ptr<liteaerosim::Aircraft> makeAircraft(float stub_thrust_n = 989.0f) {
    auto prop = std::make_unique<StubPropulsion>(stub_thrust_n);
    auto ac   = std::make_unique<liteaerosim::Aircraft>(std::move(prop));
    ac->initialize(makeConfig());
    return ac;
}

// ---------------------------------------------------------------------------
// Item 3 — class definition
// ---------------------------------------------------------------------------

TEST(AircraftTest, ConstructsWithoutThrowing) {
    auto prop = std::make_unique<StubPropulsion>();
    EXPECT_NO_THROW({
        liteaerosim::Aircraft ac(std::move(prop));
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

    liteaerosim::AircraftCommand cmd;
    cmd.n         = 1.0f;
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

    liteaerosim::AircraftCommand cmd;
    cmd.n         = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    EXPECT_NO_THROW(ac->step(0.1, cmd, wind, 1.225f));
}

TEST(AircraftTest, ZeroThrottle_AircraftDecelerates) {
    // Zero thrust → drag decelerates the aircraft.
    auto ac = makeAircraft(0.0f);   // StubPropulsion always returns 0 N

    liteaerosim::AircraftCommand cmd;
    cmd.n         = 1.0f;
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

    liteaerosim::AircraftCommand cmd;
    cmd.n         = 1.0f;
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

static liteaerosim::AircraftCommand levelCmd() {
    liteaerosim::AircraftCommand cmd;
    cmd.n           = 1.0f;
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
    liteaerosim::Aircraft ac2(std::move(prop2));
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
    liteaerosim::Aircraft ac2(std::move(prop2));
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
    liteaerosim::Aircraft ac2(std::move(prop));
    EXPECT_THROW({ ac2.deserializeJson(snapshot); }, std::runtime_error);
}

TEST(AircraftTest, ProtoSchemaVersionMismatchThrows) {
    auto ac = makeAircraft();
    std::vector<uint8_t> bytes = ac->serializeProto();
    // AircraftState field 1 = schema_version: tag 0x08 at bytes[0], value varint at bytes[1].
    bytes[1] = static_cast<uint8_t>(99);

    auto prop = std::make_unique<StubPropulsion>();
    liteaerosim::Aircraft ac2(std::move(prop));
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
    auto ac = std::make_unique<liteaerosim::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/general_aviation.json")));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 55.0f);
}

TEST(AircraftTest, InitializeFromFixture_JetTrainer) {
    auto ac = std::make_unique<liteaerosim::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/jet_trainer.json")));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 150.0f);
}

TEST(AircraftTest, InitializeFromFixture_SmallUAS) {
    auto ac = std::make_unique<liteaerosim::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/small_uas.json")));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 20.0f);
}

TEST(AircraftTest, InitializeWithMissingField_Throws) {
    nlohmann::json config = loadFixture("aircraft/general_aviation.json");
    config.erase("inertia");

    auto ac = std::make_unique<liteaerosim::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(config), std::exception);
}
