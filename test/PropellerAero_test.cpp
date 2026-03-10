#define _USE_MATH_DEFINES
#include "propulsion/PropellerAero.hpp"
#include <cmath>
#include <gtest/gtest.h>

using liteaerosim::propulsion::PropellerAero;

// ── Test propeller: 0.5 m diameter, 0.5 m pitch, 3 blades, σ = 0.15 ──────────
//
// Derived:
//   J_zero       = pitch / diam                          = 0.5 / 0.5 = 1.0
//   disk_area_m2 = π · 0.0625                            ≈ 0.19635 m²
//   θ₇₅          = atan(0.5 / (0.75π·0.5))               ≈ 0.40240 rad
//   CT0          = (0.15 · 2π / 8) · θ₇₅                ≈ 0.047426
//   CQ0          = 0.15 · 0.01 / 8                       = 0.0001875

static constexpr float kDiam      = 0.5f;
static constexpr float kPitch     = 0.5f;
static constexpr int   kBlades    = 3;
static constexpr float kSolidity  = 0.15f;
static constexpr float kRhoSL     = 1.225f;
static constexpr float kPi        = 3.14159265358979f;

static PropellerAero makeProp() {
    return PropellerAero(kDiam, kPitch, kBlades, kSolidity);
}

// ── Derived quantities ────────────────────────────────────────────────────────

TEST(PropellerAero, JZeroIsRatioOfPitchToDiameter) {
    const auto prop = makeProp();
    EXPECT_NEAR(prop.J_zero, kPitch / kDiam, 1e-6f);
}

TEST(PropellerAero, DiskAreaIsCorrect) {
    const auto prop = makeProp();
    const float expected = kPi * (kDiam * 0.5f) * (kDiam * 0.5f);
    EXPECT_NEAR(prop.disk_area_m2, expected, 1e-5f);
}

TEST(PropellerAero, CT0IsPositive) {
    EXPECT_GT(makeProp().CT0, 0.f);
}

TEST(PropellerAero, CQ0IsPositive) {
    EXPECT_GT(makeProp().CQ0, 0.f);
}

// ── advanceRatio ──────────────────────────────────────────────────────────────

TEST(PropellerAero, AdvanceRatioZeroAtZeroAirspeed) {
    const auto prop = makeProp();
    EXPECT_FLOAT_EQ(prop.advanceRatio(100.f, 0.f), 0.f);
}

TEST(PropellerAero, AdvanceRatioCorrectValue) {
    const auto prop = makeProp();
    // Omega = 2π·50 rad/s → n = 50 rev/s, J = 30 / (50·0.5) = 1.2 → clamped to J_zero=1.0
    const float Omega = 2.f * kPi * 50.f;
    EXPECT_NEAR(prop.advanceRatio(Omega, 30.f), 1.0f, 1e-4f);  // clamped
}

TEST(PropellerAero, AdvanceRatioClampedAtJZero) {
    const auto  prop  = makeProp();
    const float Omega = 2.f * kPi * 10.f;   // low RPM, high J
    EXPECT_LE(prop.advanceRatio(Omega, 100.f), prop.J_zero + 1e-5f);
}

TEST(PropellerAero, AdvanceRatioZeroForNonPositiveOmega) {
    const auto prop = makeProp();
    EXPECT_FLOAT_EQ(prop.advanceRatio(0.f, 20.f), 0.f);
    EXPECT_FLOAT_EQ(prop.advanceRatio(-1.f, 20.f), 0.f);
}

// ── Thrust coefficient ────────────────────────────────────────────────────────

TEST(PropellerAero, ThrustCoeffAtJZeroEqualsCT0) {
    const auto prop = makeProp();
    EXPECT_NEAR(prop.thrustCoeff(0.f), prop.CT0, 1e-6f);
}

TEST(PropellerAero, ThrustCoeffZeroAtJZero) {
    const auto prop = makeProp();
    EXPECT_NEAR(prop.thrustCoeff(prop.J_zero), 0.f, 1e-6f);
}

TEST(PropellerAero, ThrustCoeffParabolicShape) {
    const auto  prop = makeProp();
    const float J    = prop.J_zero * 0.5f;
    const float expected = prop.CT0 * 0.25f;  // (1 - 0.5)^2 = 0.25
    EXPECT_NEAR(prop.thrustCoeff(J), expected, 1e-6f);
}

// ── Torque coefficient ────────────────────────────────────────────────────────

TEST(PropellerAero, TorqueCoeffAtJZeroEqualsCQ0) {
    const auto prop = makeProp();
    EXPECT_NEAR(prop.torqueCoeff(0.f), prop.CQ0, 1e-6f);
}

TEST(PropellerAero, TorqueCoeffIncreasesWithJ) {
    const auto prop = makeProp();
    EXPECT_GT(prop.torqueCoeff(0.5f), prop.torqueCoeff(0.f));
}

// ── Dimensional thrust and torque ─────────────────────────────────────────────

TEST(PropellerAero, ThrustZeroAtZeroOmega) {
    const auto prop = makeProp();
    EXPECT_FLOAT_EQ(prop.thrust_n(0.f, 0.f, kRhoSL), 0.f);
}

TEST(PropellerAero, ThrustPositiveAtNonzeroOmega) {
    const auto  prop  = makeProp();
    const float Omega = 2.f * kPi * 100.f;  // 6000 rpm
    EXPECT_GT(prop.thrust_n(Omega, 0.f, kRhoSL), 0.f);
}

TEST(PropellerAero, ThrustDimensionalFormula) {
    const auto  prop  = makeProp();
    const float Omega = 2.f * kPi * 100.f;
    const float n     = 100.f;
    const float D4    = kDiam * kDiam * kDiam * kDiam;
    const float CT    = prop.CT0;  // J = 0 at V = 0
    const float expected = CT * kRhoSL * n * n * D4;
    EXPECT_NEAR(prop.thrust_n(Omega, 0.f, kRhoSL), expected, 0.01f);
}

TEST(PropellerAero, TorqueZeroAtZeroOmega) {
    const auto prop = makeProp();
    EXPECT_FLOAT_EQ(prop.torque_nm(0.f, 0.f, kRhoSL), 0.f);
}

TEST(PropellerAero, TorquePositiveAtNonzeroOmega) {
    const auto  prop  = makeProp();
    const float Omega = 2.f * kPi * 100.f;
    EXPECT_GT(prop.torque_nm(Omega, 0.f, kRhoSL), 0.f);
}
