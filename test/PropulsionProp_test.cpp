#define _USE_MATH_DEFINES
#include "propulsion/PropulsionProp.hpp"
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include "propulsion/PropellerAero.hpp"
#include <cmath>
#include <memory>
#include <gtest/gtest.h>

using liteaerosim::propulsion::MotorElectric;
using liteaerosim::propulsion::MotorElectricEscParams;
using liteaerosim::propulsion::MotorElectricMotorParams;
using liteaerosim::propulsion::MotorPiston;
using liteaerosim::propulsion::PropellerAero;
using liteaerosim::propulsion::PropulsionProp;

static constexpr float kRhoSL = 1.225f;
static constexpr float kDt    = 0.01f;
static constexpr float kTau   = 0.3f;

// Propeller: 0.5 m diam, 0.5 m pitch, 3 blades, σ = 0.15
static PropellerAero makePropeller() {
    return PropellerAero(0.5f, 0.5f, 3, 0.15f);
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
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    EXPECT_FLOAT_EQ(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ZeroOmegaBeforeFirstStep) {
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    EXPECT_FLOAT_EQ(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, ElectricMotorThrustPositiveAtFullThrottle) {
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ElectricMotorOmegaPositiveAtFullThrottle) {
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, PistonMotorThrustPositive) {
    PropulsionProp prop(makePropeller(), makePistonMotor(), kDt, kTau);
    runToSteadyState(prop, 1.0f);
    EXPECT_GT(prop.thrust_n(), 0.f);
}

TEST(PropulsionProp, ThrustZeroAtZeroThrottle) {
    // At throttle = 0, noLoadOmega = 0 → filter → 0 → no thrust.
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop, 0.0f);
    EXPECT_NEAR(prop.thrust_n(), 0.f, 0.01f);
}

TEST(PropulsionProp, ResetGivesThrustZero) {
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop, 1.0f);
    prop.reset();
    EXPECT_FLOAT_EQ(prop.thrust_n(), 0.f);
    EXPECT_FLOAT_EQ(prop.omega_rps(), 0.f);
}

TEST(PropulsionProp, TypeMismatchThrowsOnDeserializeJson) {
    PropulsionProp prop(makePropeller(), makeElectricMotor(), kDt, kTau);
    nlohmann::json bad = {{"schema_version", 1}, {"type", "PropulsionJet"}, {"thrust_n", 0.f}};
    EXPECT_THROW(prop.deserializeJson(bad), std::runtime_error);
}

TEST(PropulsionProp, JsonRoundTrip) {
    PropulsionProp prop1(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop1, 1.0f);

    const nlohmann::json snap = prop1.serializeJson();

    PropulsionProp prop2(makePropeller(), makeElectricMotor(), kDt, kTau);
    prop2.deserializeJson(snap);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, ProtoRoundTrip) {
    PropulsionProp prop1(makePropeller(), makeElectricMotor(), kDt, kTau);
    runToSteadyState(prop1, 1.0f);

    const std::vector<uint8_t> bytes = prop1.serializeProto();

    PropulsionProp prop2(makePropeller(), makeElectricMotor(), kDt, kTau);
    prop2.deserializeProto(bytes);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, JsonRoundTripMidTransient) {
    // Snapshot mid-transient (5 steps << tau_rotor=0.3s): x[0] differs from steady-state.
    PropulsionProp prop1(makePropeller(), makeElectricMotor(), kDt, kTau);
    for (int i = 0; i < 5; ++i)
        (void)prop1.step(1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = prop1.serializeJson();

    PropulsionProp prop2(makePropeller(), makeElectricMotor(), kDt, kTau);
    prop2.deserializeJson(snap);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionProp, ProtoRoundTripMidTransient) {
    PropulsionProp prop1(makePropeller(), makeElectricMotor(), kDt, kTau);
    for (int i = 0; i < 5; ++i)
        (void)prop1.step(1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = prop1.serializeProto();

    PropulsionProp prop2(makePropeller(), makeElectricMotor(), kDt, kTau);
    prop2.deserializeProto(bytes);

    const float t1 = prop1.step(1.0f, 0.f, kRhoSL);
    const float t2 = prop2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}
