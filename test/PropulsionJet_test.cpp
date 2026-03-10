#define _USE_MATH_DEFINES
#include "propulsion/PropulsionJet.hpp"
#include <cmath>
#include <gtest/gtest.h>

using liteaerosim::propulsion::PropulsionJet;
using liteaerosim::propulsion::PropulsionJetParams;

// ── Shared test fixture ───────────────────────────────────────────────────────

static constexpr float kRhoSL = 1.225f;
static constexpr float kDt    = 0.01f;

static PropulsionJetParams makeParams() {
    PropulsionJetParams p;
    p.thrust_sl_n    = 80000.f;
    p.bypass_ratio   = 0.5f;    // n = 1/√1.5 ≈ 0.8165
    p.inlet_area_m2  = 0.30f;
    p.idle_fraction  = 0.05f;
    p.spool_tau_s    = 2.0f;
    p.ab_thrust_sl_n = 20000.f;
    p.ab_spool_tau_s = 1.0f;
    return p;
}

// Run the filter to approximate steady state.
// Default 3000 steps = 30 s = 15 × tau_spool (2 s) → > 99.9999 % converged.
static void runToSteadyState(PropulsionJet& jet, float throttle, float tas = 0.f,
                              float rho = kRhoSL, int steps = 3000) {
    for (int i = 0; i < steps; ++i)
        jet.step(throttle, tas, rho);
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST(PropulsionJet, ZeroThrustBeforeFirstStep) {
    PropulsionJet jet(makeParams(), kDt);
    EXPECT_FLOAT_EQ(jet.thrust_n(), 0.f);
}

TEST(PropulsionJet, ThrustAtSeaLevelFullThrottleNoAirspeed) {
    // At V=0, no ram drag.  T_avail = T_sl (full throttle), not limited.
    // After steady state, dry thrust ≈ T_sl.
    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 1.0f, 0.f, kRhoSL);
    EXPECT_NEAR(jet.thrust_n(), 80000.f, 100.f);
}

TEST(PropulsionJet, IdleFloorAtThrottleZero) {
    // Throttle 0 → T_demand = 0, but filter lower limit is T_idle.
    // T_idle = 0.05 * 80000 = 4000 N at SL.
    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 0.0f, 0.f, kRhoSL);
    EXPECT_NEAR(jet.thrust_n(), 4000.f, 50.f);
}

TEST(PropulsionJet, AltitudeLapse) {
    // At rho = 0.5 * rho_SL and full throttle (V=0):
    // n = 1/sqrt(1.5); T_gross = 80000 * 0.5^n
    const float n        = 1.f / std::sqrt(1.5f);
    const float rho_alt  = 0.5f * kRhoSL;
    const float expected = 80000.f * std::pow(0.5f, n);

    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 1.0f, 0.f, rho_alt);
    EXPECT_NEAR(jet.thrust_n(), expected, 200.f);
}

TEST(PropulsionJet, RamDragReducesThrust) {
    // At V=200 m/s, F_ram = rho * V^2 * A_inlet = 1.225 * 40000 * 0.3 = 14700 N.
    // T_avail = 80000 - 14700 = 65300 N (> T_idle, so not floored).
    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 1.0f, 200.f, kRhoSL);
    const float expected = 80000.f - kRhoSL * 200.f * 200.f * 0.30f;
    EXPECT_NEAR(jet.thrust_n(), expected, 200.f);
}

TEST(PropulsionJet, SpoolDynamicsBuildUp) {
    // After reset, thrust should build from 0 toward T_demand but not reach it in 1 step.
    PropulsionJet jet(makeParams(), kDt);
    (void)jet.step(1.0f, 0.f, kRhoSL);
    EXPECT_GT(jet.thrust_n(), 0.f);
    EXPECT_LT(jet.thrust_n(), 80000.f);
}

TEST(PropulsionJet, AfterburnerIncreasesThrust) {
    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 1.0f, 0.f, kRhoSL);
    const float dry_thrust = jet.thrust_n();

    jet.setAfterburner(true);
    runToSteadyState(jet, 1.0f, 0.f, kRhoSL);
    EXPECT_GT(jet.thrust_n(), dry_thrust + 15000.f);
}

TEST(PropulsionJet, AfterburnerBlowoutDecaysThrust) {
    PropulsionJet jet(makeParams(), kDt);
    jet.setAfterburner(true);
    runToSteadyState(jet, 1.0f, 0.f, kRhoSL);
    const float ab_thrust = jet.thrust_n();  // ≈ 80000 + 20000 = 100000 N

    jet.setAfterburner(false);
    // After 15 × tau_ab (1 s each) = 15 s = 1500 steps, AB contribution < 0.03% of 20000 N.
    runToSteadyState(jet, 1.0f, 0.f, kRhoSL, 1500);
    EXPECT_GT(ab_thrust - jet.thrust_n(), 15000.f);   // shed > 15 kN of AB thrust
}

TEST(PropulsionJet, ResetGivesThrustZero) {
    PropulsionJet jet(makeParams(), kDt);
    runToSteadyState(jet, 1.0f);
    jet.reset();
    EXPECT_FLOAT_EQ(jet.thrust_n(), 0.f);
}

TEST(PropulsionJet, TypeMismatchThrowsOnDeserializeJson) {
    PropulsionJet jet(makeParams(), kDt);
    nlohmann::json bad = {{"schema_version", 1}, {"type", "PropulsionEDF"}, {"thrust_n", 0.f}};
    EXPECT_THROW(jet.deserializeJson(bad), std::runtime_error);
}

TEST(PropulsionJet, JsonRoundTrip) {
    PropulsionJet jet1(makeParams(), kDt);
    runToSteadyState(jet1, 1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = jet1.serializeJson();

    PropulsionJet jet2(makeParams(), kDt);
    jet2.deserializeJson(snap);

    // One more step from both — results must match.
    const float t1 = jet1.step(1.0f, 0.f, kRhoSL);
    const float t2 = jet2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionJet, ProtoRoundTrip) {
    PropulsionJet jet1(makeParams(), kDt);
    runToSteadyState(jet1, 1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = jet1.serializeProto();

    PropulsionJet jet2(makeParams(), kDt);
    jet2.deserializeProto(bytes);

    const float t1 = jet1.step(1.0f, 0.f, kRhoSL);
    const float t2 = jet2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionJet, JsonRoundTripMidTransient) {
    // Snapshot mid-transient (5 steps << tau_spool=2s): x[0] differs from steady-state.
    PropulsionJet jet1(makeParams(), kDt);
    for (int i = 0; i < 5; ++i)
        (void)jet1.step(1.0f, 0.f, kRhoSL);

    const nlohmann::json snap = jet1.serializeJson();

    PropulsionJet jet2(makeParams(), kDt);
    jet2.deserializeJson(snap);

    const float t1 = jet1.step(1.0f, 0.f, kRhoSL);
    const float t2 = jet2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}

TEST(PropulsionJet, ProtoRoundTripMidTransient) {
    PropulsionJet jet1(makeParams(), kDt);
    for (int i = 0; i < 5; ++i)
        (void)jet1.step(1.0f, 0.f, kRhoSL);

    const std::vector<uint8_t> bytes = jet1.serializeProto();

    PropulsionJet jet2(makeParams(), kDt);
    jet2.deserializeProto(bytes);

    const float t1 = jet1.step(1.0f, 0.f, kRhoSL);
    const float t2 = jet2.step(1.0f, 0.f, kRhoSL);
    EXPECT_FLOAT_EQ(t1, t2);
}
