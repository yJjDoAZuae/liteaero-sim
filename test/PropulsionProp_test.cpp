#define _USE_MATH_DEFINES
#include "propulsion/PropulsionProp.hpp"
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include <cmath>
#include <memory>
#include <gtest/gtest.h>

using liteaerosim::propulsion::MotorElectric;
using liteaerosim::propulsion::MotorElectricEscParams;
using liteaerosim::propulsion::MotorElectricMotorParams;
using liteaerosim::propulsion::MotorPiston;
using liteaerosim::propulsion::PropulsionProp;

static constexpr float kRhoSL = 1.225f;

// Propeller config: 0.5 m diam, 0.5 m pitch, 3 blades, σ = 0.15
static nlohmann::json makePropConfig() {
    return nlohmann::json{
        {"diameter_m",     0.5f},
        {"pitch_m",        0.5f},
        {"blade_count",    3},
        {"blade_solidity", 0.15f},
        {"dt_s",           0.01f},
        {"rotor_tau_s",    0.3f},
    };
}

// Electric motor: KV=500, R=0.05, J=0.002; ESC: 22.2V, 60A, 0.95 efficiency
static std::unique_ptr<MotorElectric> makeElectricMotor() {
    MotorElectricMotorParams mpar{500.f, 0.05f, 0.002f};
    MotorElectricEscParams   epar{22.2f, 60.f, 0.95f};
    return std::make_unique<MotorElectric>(mpar, epar);
}

// Piston motor: 10 kW, 500 rad/s peak, alt_exp=1.0, J=0.05
static std::unique_ptr<MotorPiston> makePistonMotor() {
    return std::make_unique<MotorPiston>(10000.f, 500.f, 1.0f, 0.05f);
}

static void runToSteadyState(PropulsionProp& prop, float throttle, float tas = 0.f,
                              float rho = kRhoSL, int steps = 1000) {
    for (int i = 0; i < steps; ++i)
        prop.step(throttle, tas, rho);
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST(PropulsionProp, ZeroThrustBeforeFirstStep) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    EXPECT_FLOAT_EQ(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ZeroOmegaBeforeFirstStep) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    EXPECT_FLOAT_EQ(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, ElectricMotorThrustPositiveAtFullThrottle) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ElectricMotorOmegaPositiveAtFullThrottle) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, PistonMotorThrustPositive) {
    PropulsionProp prop(makePistonMotor());
    prop.initialize(makePropConfig());
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ThrustZeroAtZeroThrottle) {
    // At throttle = 0, noLoadOmega = 0 → filter → 0 → no thrust.
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    runToSteadyState(prop, 0.0f);
    EXPECT_NEAR(prop.thrust_n(), 0.f, 0.01f);
}

TEST(PropulsionProp, ResetGivesThrustZero) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    runToSteadyState(prop, 1.0f);
    prop.reset();
    EXPECT_FLOAT_EQ(prop.thrust_n(), 0.f);
    EXPECT_FLOAT_EQ(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, TypeMismatchThrowsOnDeserializeJson) {
    PropulsionProp prop(makeElectricMotor());
    prop.initialize(makePropConfig());
    nlohmann::json bad = {{"schema_version", 1}, {"type", "PropulsionJet"}, {"thrust_n", 0.f}};
    EXPECT_THROW(prop.deserializeJson(bad), std::runtime_error);
}

TEST(PropulsionProp, JsonRoundTrip) {
    PropulsionProp prop1(makeElectricMotor());
    prop1.initialize(makePropConfig());
    runToSteadyState(prop1, 1.0f);

    const nlohmann::json snap = prop1.serializeJson();

    PropulsionProp prop2(makeElectricMotor());
    prop2.initialize(makePropConfig());
    prop2.deserializeJson(snap);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, ProtoRoundTrip) {
    PropulsionProp prop1(makeElectricMotor());
    prop1.initialize(makePropConfig());
    runToSteadyState(prop1, 1.0f);

    const std::vector<uint8_t> bytes = prop1.serializeProto();

    PropulsionProp prop2(makeElectricMotor());
    prop2.initialize(makePropConfig());
    prop2.deserializeProto(bytes);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, JsonRoundTripMidTransient) {
    // Snapshot mid-transient (5 steps << tau_rotor=0.3s): x[0] differs from steady-state.
    PropulsionProp prop1(makeElectricMotor());
    prop1.initialize(makePropConfig());
    for (int i = 0; i < 5; ++i)
        (void)prop1.step(1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = prop1.serializeJson();

    PropulsionProp prop2(makeElectricMotor());
    prop2.initialize(makePropConfig());
    prop2.deserializeJson(snap);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, ProtoRoundTripMidTransient) {
    PropulsionProp prop1(makeElectricMotor());
    prop1.initialize(makePropConfig());
    for (int i = 0; i < 5; ++i)
        (void)prop1.step(1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = prop1.serializeProto();

    PropulsionProp prop2(makeElectricMotor());
    prop2.initialize(makePropConfig());
    prop2.deserializeProto(bytes);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}
