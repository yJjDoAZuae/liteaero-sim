#define _USE_MATH_DEFINES
#include "propulsion/PropulsionEDF.hpp"
#include <cmath>
#include <gtest/gtest.h>

using liteaerosim::propulsion::PropulsionEDF;
using liteaerosim::propulsion::PropulsionEdfParams;

static constexpr float kRhoSL = 1.225f;
static constexpr float kDt    = 0.01f;

static PropulsionEdfParams makeParams() {
    PropulsionEdfParams p;
    p.thrust_sl_n       = 200.f;
    p.fan_diameter_m    = 0.12f;
    p.inlet_area_m2     = 0.011f;
    p.idle_fraction     = 0.05f;
    p.rotor_tau_s       = 0.1f;
    p.supply_voltage_v  = 22.2f;
    p.fan_efficiency_nd = 0.75f;
    p.esc_efficiency_nd = 0.95f;
    return p;
}

static void runToSteadyState(PropulsionEDF& edf, float throttle, float tas = 0.f,
                              float rho = kRhoSL, int steps = 500) {
    for (int i = 0; i < steps; ++i)
        edf.step(throttle, tas, rho);
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST(PropulsionEDF, ZeroThrustBeforeFirstStep) {
    PropulsionEDF edf(makeParams(), kDt);
    EXPECT_FLOAT_EQ(edf.thrust_n(), 0.f);
}

TEST(PropulsionEDF, ThrustAtSeaLevelFullThrottleNoAirspeed) {
    // T_gross = 1.0 * 200 * 1 = 200N; F_ram = 0; T_avail = 200N.
    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 1.0f, 0.f, kRhoSL);
    EXPECT_NEAR(edf.thrust_n(), 200.f, 1.f);
}

TEST(PropulsionEDF, IdleFloorAtThrottleZero) {
    // T_idle = 0.05 * 200 = 10N.
    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 0.0f, 0.f, kRhoSL);
    EXPECT_NEAR(edf.thrust_n(), 10.f, 0.5f);
}

TEST(PropulsionEDF, AltitudeLapse) {
    // Density exponent is 1.0 for EDF.
    const float rho_alt  = 0.7f * kRhoSL;
    const float expected = 200.f * (rho_alt / kRhoSL);

    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 1.0f, 0.f, rho_alt);
    EXPECT_NEAR(edf.thrust_n(), expected, 1.f);
}

TEST(PropulsionEDF, RamDragReducesThrust) {
    // F_ram = rho * V^2 * A_inlet = 1.225 * 400 * 0.011 = 5.39N
    // T_avail = max(T_idle, 200 - 5.39) = 194.61N
    const float tas      = 20.f;
    const float F_ram    = kRhoSL * tas * tas * 0.011f;
    const float expected = std::max(0.05f * 200.f, 200.f - F_ram);

    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 1.0f, tas, kRhoSL);
    EXPECT_NEAR(edf.thrust_n(), expected, 1.f);
}

TEST(PropulsionEDF, ResetGivesThrustZero) {
    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 1.0f);
    edf.reset();
    EXPECT_FLOAT_EQ(edf.thrust_n(), 0.f);
}

TEST(PropulsionEDF, BatteryCurrentPositiveAtFullThrottle) {
    PropulsionEDF edf(makeParams(), kDt);
    runToSteadyState(edf, 1.0f, 0.f, kRhoSL);
    EXPECT_GT(edf.batteryCurrent_a(0.f, kRhoSL), 0.f);
}

TEST(PropulsionEDF, TypeMismatchThrowsOnDeserializeJson) {
    PropulsionEDF edf(makeParams(), kDt);
    nlohmann::json bad = {{"schema_version", 1}, {"type", "PropulsionJet"}, {"thrust_n", 0.f}};
    EXPECT_THROW(edf.deserializeJson(bad), std::runtime_error);
}

TEST(PropulsionEDF, JsonRoundTrip) {
    PropulsionEDF edf1(makeParams(), kDt);
    runToSteadyState(edf1, 1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = edf1.serializeJson();

    PropulsionEDF edf2(makeParams(), kDt);
    edf2.deserializeJson(snap);

    const float t1 = edf1.step(1.0f, 0.f, kRhoSL);
    const float t2 = edf2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionEDF, ProtoRoundTrip) {
    PropulsionEDF edf1(makeParams(), kDt);
    runToSteadyState(edf1, 1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = edf1.serializeProto();

    PropulsionEDF edf2(makeParams(), kDt);
    edf2.deserializeProto(bytes);

    const float t1 = edf1.step(1.0f, 0.f, kRhoSL);
    const float t2 = edf2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionEDF, JsonRoundTripMidTransient) {
    // Snapshot mid-transient (5 steps << tau_rotor=0.1s): x[0] differs from steady-state.
    PropulsionEDF edf1(makeParams(), kDt);
    for (int i = 0; i < 5; ++i)
        (void)edf1.step(1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = edf1.serializeJson();

    PropulsionEDF edf2(makeParams(), kDt);
    edf2.deserializeJson(snap);

    const float t1 = edf1.step(1.0f, 0.f, kRhoSL);
    const float t2 = edf2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionEDF, ProtoRoundTripMidTransient) {
    PropulsionEDF edf1(makeParams(), kDt);
    for (int i = 0; i < 5; ++i)
        (void)edf1.step(1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = edf1.serializeProto();

    PropulsionEDF edf2(makeParams(), kDt);
    edf2.deserializeProto(bytes);

    const float t1 = edf1.step(1.0f, 0.f, kRhoSL);
    const float t2 = edf2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}
