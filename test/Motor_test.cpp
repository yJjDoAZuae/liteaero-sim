#define _USE_MATH_DEFINES
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include <cmath>
#include <gtest/gtest.h>

using liteaerosim::propulsion::MotorElectric;
using liteaerosim::propulsion::MotorElectricEscParams;
using liteaerosim::propulsion::MotorElectricMotorParams;
using liteaerosim::propulsion::MotorPiston;

static constexpr float kRhoSL = 1.225f;

// ── MotorElectric ─────────────────────────────────────────────────────────────
//
// Motor: KV = 1000 rad/s/V, R = 0.01 Ω, J = 0.001 kg·m²
// ESC:   V_supply = 11.1 V, I_max = 30 A, η_ESC = 0.95

static MotorElectric makeElectric() {
    MotorElectricMotorParams motor{1000.f, 0.01f, 0.001f};
    MotorElectricEscParams   esc{11.1f, 30.f, 0.95f};
    return MotorElectric(motor, esc);
}

TEST(MotorElectric, NoLoadOmegaFullThrottle) {
    // Ω₀ = KV · δ_T · V_supply = 1000 · 1.0 · 11.1 = 11100 rad/s
    EXPECT_NEAR(makeElectric().noLoadOmega_rps(1.0f, kRhoSL), 11100.f, 0.1f);
}

TEST(MotorElectric, NoLoadOmegaHalfThrottle) {
    EXPECT_NEAR(makeElectric().noLoadOmega_rps(0.5f, kRhoSL), 5550.f, 0.1f);
}

TEST(MotorElectric, NoLoadOmegaZeroAtZeroThrottle) {
    EXPECT_FLOAT_EQ(makeElectric().noLoadOmega_rps(0.f, kRhoSL), 0.f);
}

TEST(MotorElectric, NoLoadOmegaDensityIndependent) {
    const auto m = makeElectric();
    // Electric motor is density-independent.
    EXPECT_FLOAT_EQ(m.noLoadOmega_rps(1.0f, kRhoSL),
                    m.noLoadOmega_rps(1.0f, 0.5f * kRhoSL));
}

TEST(MotorElectric, MaxOmega) {
    // Ω_max = KV · V_supply = 11100 rad/s
    EXPECT_NEAR(makeElectric().maxOmega_rps(), 11100.f, 0.1f);
}

TEST(MotorElectric, Inertia) {
    EXPECT_FLOAT_EQ(makeElectric().inertia_kg_m2(), 0.001f);
}

TEST(MotorElectric, BatteryCurrentAtStallIsCurrentLimited) {
    // At omega = 0, full throttle: I_motor = (11.1 - 0) / 0.01 = 1110 A → clamped to 30A.
    // I_bat = 11.1 * 30 / (11.1 * 0.95) = 30 / 0.95 ≈ 31.58 A.
    const float expected = 30.f / 0.95f;
    EXPECT_NEAR(makeElectric().batteryCurrent_a(0.f, 1.0f), expected, 0.1f);
}

TEST(MotorElectric, BatteryCurrentZeroAtFreeRunning) {
    // At omega = Ω₀ = 11100 rad/s (back-EMF = supply): I_motor = 0.
    EXPECT_NEAR(makeElectric().batteryCurrent_a(11100.f, 1.0f), 0.f, 0.1f);
}

TEST(MotorElectric, BatteryCurrentZeroAtZeroThrottle) {
    // V_phase = 0 → no current.
    EXPECT_FLOAT_EQ(makeElectric().batteryCurrent_a(0.f, 0.f), 0.f);
}

// ── MotorPiston ───────────────────────────────────────────────────────────────
//
// P_max = 74600 W (100 hp), Ω_peak = 314.16 rad/s (3000 rpm), n_alt = 1.0, J = 0.05 kg·m²

static MotorPiston makePiston() {
    return MotorPiston(74600.f, 314.16f, 1.0f, 0.05f);
}

TEST(MotorPiston, MaxOmegaIsTwicePeakOmega) {
    EXPECT_NEAR(makePiston().maxOmega_rps(), 2.f * 314.16f, 0.01f);
}

TEST(MotorPiston, NoLoadOmegaFullThrottleSeaLevel) {
    // Ω₀ = 2 · Ω_peak · δ_T · (ρ/ρ_SL)^(n/2) = 2·314.16·1.0·1.0 = 628.32 rad/s
    EXPECT_NEAR(makePiston().noLoadOmega_rps(1.0f, kRhoSL), 628.32f, 0.1f);
}

TEST(MotorPiston, NoLoadOmegaHalfThrottle) {
    EXPECT_NEAR(makePiston().noLoadOmega_rps(0.5f, kRhoSL), 314.08f, 0.1f);
}

TEST(MotorPiston, NoLoadOmegaZeroAtZeroThrottle) {
    EXPECT_FLOAT_EQ(makePiston().noLoadOmega_rps(0.f, kRhoSL), 0.f);
}

TEST(MotorPiston, NoLoadOmegaLapsesWithDensity) {
    // At rho = 0.25 * rho_SL: (0.25)^0.5 = 0.5, so Ω₀ = 628.32 * 0.5 = 314.16
    const float omega_alt = makePiston().noLoadOmega_rps(1.0f, 0.25f * kRhoSL);
    EXPECT_NEAR(omega_alt, 628.32f * 0.5f, 0.2f);
}

TEST(MotorPiston, Inertia) {
    EXPECT_FLOAT_EQ(makePiston().inertia_kg_m2(), 0.05f);
}
