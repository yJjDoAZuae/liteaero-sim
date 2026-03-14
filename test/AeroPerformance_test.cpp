#define _USE_MATH_DEFINES
#include "aerodynamics/AeroPerformance.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>

using namespace liteaerosim::aerodynamics;

// Typical general-aviation parameters:
//   S = 16 m², AR = 7.2, e = 0.8, CD0 = 0.027, C_Yβ = -3.0 rad⁻¹
// Derived:
//   k = 1 / (π · 0.8 · 7.2) ≈ 0.05526
static AeroPerformance gaAero() {
    AeroPerformanceConfig cfg;
    cfg.s_ref_m2  = 16.f;
    cfg.ar        = 7.2f;
    cfg.e         = 0.8f;
    cfg.cd0       = 0.027f;
    cfg.cl_y_beta = -3.0f;
    return AeroPerformance(cfg);
}

static constexpr float kQ    = 1531.f;   // Pa (≈ 50 m/s at sea level)
static constexpr float kCL   = 0.40f;
static constexpr float kAlpha = 0.07f;   // rad (linear region)
static constexpr float kBeta  = 0.05f;   // rad

// ── Force correctness ─────────────────────────────────────────────────────────

TEST(AeroPerformanceTest, LevelFlightFzMatchesLift) {
    // Fz = -q · S · CL → -Fz = q · S · CL
    AeroPerformance aero = gaAero();
    AeroForces f = aero.compute(kAlpha, 0.f, kQ, kCL);
    EXPECT_NEAR(-f.z_n, kQ * 16.f * kCL, 1.f);
}

TEST(AeroPerformanceTest, ZeroAirspeedZeroForces) {
    AeroPerformance aero = gaAero();
    AeroForces f = aero.compute(kAlpha, kBeta, 0.f, kCL);
    EXPECT_NEAR(f.x_n, 0.f, 1e-6f);
    EXPECT_NEAR(f.y_n, 0.f, 1e-6f);
    EXPECT_NEAR(f.z_n, 0.f, 1e-6f);
}

TEST(AeroPerformanceTest, ZeroBetaZeroSideForce) {
    AeroPerformance aero = gaAero();
    AeroForces f = aero.compute(kAlpha, 0.f, kQ, kCL);
    EXPECT_NEAR(f.y_n, 0.f, 1e-5f);
}

TEST(AeroPerformanceTest, DragIncreasesWithCL) {
    // Induced drag term: CDi = k · CL²  →  higher CL → higher drag.
    AeroPerformance aero = gaAero();
    AeroForces f_lo = aero.compute(kAlpha, 0.f, kQ, 0.5f);
    AeroForces f_hi = aero.compute(kAlpha, 0.f, kQ, 1.0f);
    EXPECT_LT(f_hi.x_n, f_lo.x_n);  // more drag → more negative x_n
}

TEST(AeroPerformanceTest, DragIsNegativeX) {
    AeroPerformance aero = gaAero();
    AeroForces f = aero.compute(kAlpha, 0.f, kQ, kCL);
    EXPECT_LT(f.x_n, 0.f);
}

TEST(AeroPerformanceTest, LiftIsNegativeZ) {
    AeroPerformance aero = gaAero();
    AeroForces f = aero.compute(kAlpha, 0.f, kQ, kCL);
    EXPECT_LT(f.z_n, 0.f);
}

TEST(AeroPerformanceTest, SideForceMatchesAnalytic) {
    // CY = C_Yβ · β = -3.0 · 0.05 = -0.15
    // Fy = q · S · CY = 1531 · 16 · (-0.15) = -3674.4
    AeroPerformance aero = gaAero();
    const float expected_fy = kQ * 16.f * (-3.0f * kBeta);
    AeroForces f = aero.compute(kAlpha, kBeta, kQ, kCL);
    EXPECT_NEAR(f.y_n, expected_fy, 1.f);
}

TEST(AeroPerformanceTest, DragPolarAnalytic) {
    // k = 1/(π·e·AR) = 1/(π·0.8·7.2) ≈ 0.05526
    // CDi = k·CL² ≈ 0.05526·0.16 ≈ 0.008842
    // CD = 0.027 + 0.008842 ≈ 0.035842
    // Fx = -q·S·CD = -1531·16·0.035842 ≈ -877.9
    AeroPerformance aero = gaAero();
    const float k        = 1.f / (static_cast<float>(M_PI) * 0.8f * 7.2f);
    const float cdi      = k * kCL * kCL;
    const float cd       = 0.027f + cdi;
    const float expected = -kQ * 16.f * cd;
    AeroForces f = aero.compute(kAlpha, 0.f, kQ, kCL);
    EXPECT_NEAR(f.x_n, expected, 1.f);
}

// ── Serialization ─────────────────────────────────────────────────────────────

static const float kAlphaTest1 = 0.05f;
static const float kAlphaTest2 = 0.15f;
static const float kBetaTest   = 0.03f;

TEST(AeroPerformanceSerializationTest, JsonRoundTrip) {
    AeroPerformance original = gaAero();
    const nlohmann::json j = original.serializeJson();
    const AeroPerformance restored = AeroPerformance::deserializeJson(j);
    AeroForces f_orig    = original.compute(kAlphaTest1, kBetaTest, kQ, 0.4f);
    AeroForces f_restored = restored.compute(kAlphaTest1, kBetaTest, kQ, 0.4f);
    EXPECT_NEAR(f_restored.x_n, f_orig.x_n, 1e-3f);
    EXPECT_NEAR(f_restored.y_n, f_orig.y_n, 1e-3f);
    EXPECT_NEAR(f_restored.z_n, f_orig.z_n, 1e-3f);

    AeroForces f_orig2    = original.compute(kAlphaTest2, 0.f, kQ, 0.8f);
    AeroForces f_restored2 = restored.compute(kAlphaTest2, 0.f, kQ, 0.8f);
    EXPECT_NEAR(f_restored2.x_n, f_orig2.x_n, 1e-3f);
    EXPECT_NEAR(f_restored2.z_n, f_orig2.z_n, 1e-3f);
}

TEST(AeroPerformanceSerializationTest, JsonSchemaVersionMismatchThrows) {
    AeroPerformance aero = gaAero();
    nlohmann::json j = aero.serializeJson();
    j["schema_version"] = 99;
    EXPECT_THROW(AeroPerformance::deserializeJson(j), std::runtime_error);
}

TEST(AeroPerformanceSerializationTest, ProtoRoundTrip) {
    AeroPerformance original = gaAero();
    const std::vector<uint8_t> bytes = original.serializeProto();
    const AeroPerformance restored = AeroPerformance::deserializeProto(bytes);
    AeroForces f_orig    = original.compute(kAlphaTest1, kBetaTest, kQ, 0.4f);
    AeroForces f_restored = restored.compute(kAlphaTest1, kBetaTest, kQ, 0.4f);
    EXPECT_NEAR(f_restored.x_n, f_orig.x_n, 1e-2f);
    EXPECT_NEAR(f_restored.y_n, f_orig.y_n, 1e-2f);
    EXPECT_NEAR(f_restored.z_n, f_orig.z_n, 1e-2f);
}

TEST(AeroPerformanceSerializationTest, ProtoSchemaVersionMismatchThrows) {
    AeroPerformance aero = gaAero();
    std::vector<uint8_t> bytes = aero.serializeProto();
    for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] == 0x08) {
            bytes[i + 1] = 99;
            break;
        }
    }
    EXPECT_THROW(AeroPerformance::deserializeProto(bytes), std::runtime_error);
}
