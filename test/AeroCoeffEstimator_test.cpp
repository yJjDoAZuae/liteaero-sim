#define _USE_MATH_DEFINES
#include "aerodynamics/AeroCoeffEstimator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

using namespace liteaerosim::aerodynamics;

// ── Fixtures ──────────────────────────────────────────────────────────────────
//
// referenceUav() — rectangular planform UAV.  AR = 12.5, zero sweep throughout.
// Chosen for clean analytical checks: rectangular wings have tan(Λ_QC) = 0 exactly.
//
//   Wing:   b = 3.0 m, S = 0.72 m², Λ₀ = 0, λ = 1.0, x_LE = 0.50 m
//   H-tail: b = 1.0 m, S = 0.15 m², Λ₀ = 0, λ = 1.0, x_LE = 1.60 m
//   V-tail: b = 0.4 m, S = 0.08 m², Λ₀ = 0, λ = 1.0, x_LE = 1.65 m
//   Fuselage: l = 1.9 m, d = 0.12 m
//   Airfoil: t/c = 0.12, Cl_α_2D = 2π, Cl_max_2D = 1.4
//   CG: x = 0.72 m
//   M = 0, η = 0.9, cd_misc = 0.003
static AircraftGeometry referenceUav() {
    AircraftGeometry g;
    g.wing   = {3.0f, 0.72f, 0.0f, 1.0f, 0.50f};
    g.h_tail = {1.0f, 0.15f, 0.0f, 1.0f, 1.60f};
    g.v_tail = {0.4f, 0.08f, 0.0f, 1.0f, 1.65f};
    g.fuselage_length_m    = 1.9f;
    g.fuselage_diameter_m  = 0.12f;
    g.thickness_ratio_nd   = 0.12f;
    g.section_cl_alpha_rad = 2.0f * static_cast<float>(M_PI);
    g.section_cl_max_2d_nd = 1.4f;
    g.x_cg_m               = 0.72f;
    // mach_nd = 0, tail_efficiency_nd = 0.9, cd_misc_nd = 0.003 — from defaults
    return g;
}

// moderateArUav() — lower-AR UAV that produces C_Lq in the typical [3, 12] range.
//   Wing:   b = 2.0 m, S = 1.0 m², AR = 4.0, zero sweep, λ = 1.0, x_LE = 0.35 m
//   H-tail: b = 0.8 m, S = 0.10 m², AR = 6.4, zero sweep, λ = 1.0, x_LE = 1.20 m
//   V-tail: b = 0.3 m, S = 0.05 m², zero sweep, λ = 1.0, x_LE = 1.25 m
//   Fuselage: l = 1.4 m, d = 0.10 m
//   Airfoil: t/c = 0.12, Cl_α_2D = 2π, Cl_max_2D = 1.4
//   CG: x = 0.55 m
static AircraftGeometry moderateArUav() {
    AircraftGeometry g;
    g.wing   = {2.0f, 1.0f, 0.0f, 1.0f, 0.35f};
    g.h_tail = {0.8f, 0.10f, 0.0f, 1.0f, 1.20f};
    g.v_tail = {0.3f, 0.05f, 0.0f, 1.0f, 1.25f};
    g.fuselage_length_m    = 1.4f;
    g.fuselage_diameter_m  = 0.10f;
    g.thickness_ratio_nd   = 0.12f;
    g.section_cl_alpha_rad = 2.0f * static_cast<float>(M_PI);
    g.section_cl_max_2d_nd = 1.4f;
    g.x_cg_m               = 0.55f;
    return g;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// 1. AR = b² / S for the test wing.
TEST(AeroCoeffEstimatorTest, AspectRatio_MatchesBsquaredOverS) {
    const AircraftGeometry geom = referenceUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);
    const float expected_ar = geom.wing.span_m * geom.wing.span_m / geom.wing.area_m2;
    EXPECT_NEAR(aero.ar(), expected_ar, expected_ar * 0.001f);
}

// 2. MAC matches the closed-form trapezoidal formula.
TEST(AeroCoeffEstimatorTest, Mac_MatchesClosedForm) {
    const AircraftGeometry geom = referenceUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);
    // Rectangular wing (λ = 1): mac = c_root = 2S/(b(1+λ)) = 2·0.72/(3.0·2) = 0.24 m
    const float lambda  = geom.wing.taper_ratio_nd;
    const float c_root  = 2.f * geom.wing.area_m2 /
                          (geom.wing.span_m * (1.f + lambda));
    const float expected_mac = (2.f / 3.f) * c_root *
                                (1.f + lambda + lambda * lambda) / (1.f + lambda);
    EXPECT_NEAR(aero.macM(), expected_mac, expected_mac * 0.001f);
}

// 3. C_Lα for unswept wing at M = 0 matches the full Helmbold formula.
//    For zero sweep tan(Λ_c/2) = 0 and η = 1, so
//    C_Lα = 2π·AR / (2 + √(4 + AR²)).
TEST(AeroCoeffEstimatorTest, ClAlpha_UnsweptWing_MatchesHelmboldFormula) {
    // referenceUav has zero sweep and λ = 1 → tan(Λ_c/2) = 0 exactly.
    const AircraftGeometry geom = referenceUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);
    const float ar       = geom.wing.span_m * geom.wing.span_m / geom.wing.area_m2;
    const float expected = 2.f * static_cast<float>(M_PI) * ar /
                           (2.f + std::sqrt(4.f + ar * ar));
    EXPECT_NEAR(lcp.cl_alpha, expected, expected * 0.001f);
}

// 4. C_Lmax = Cl_max_2D · cos(Λ_QC) for a swept wing.
TEST(AeroCoeffEstimatorTest, ClMax_SweptWing_EqualsClMax2dTimesCosQcSweep) {
    AircraftGeometry geom = referenceUav();
    geom.wing.le_sweep_rad    = 0.2f;   // ~11.5°
    geom.wing.taper_ratio_nd  = 0.6f;
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);

    const float ar      = geom.wing.span_m * geom.wing.span_m / geom.wing.area_m2;
    const float tan_qc  = std::tan(0.2f) - (1.f / ar) * (1.f - 0.6f) / (1.f + 0.6f);
    const float expected_cl_max = 1.4f * std::cos(std::atan(tan_qc));
    EXPECT_NEAR(lcp.cl_max, expected_cl_max, 0.001f);
}

// 5. Oswald e — Hoerner formula for AR = 8, zero sweep.
TEST(AeroCoeffEstimatorTest, OswaldE_Hoerner_AR8_ZeroSweep) {
    // Use a separate rectangular wing fixture: b = 4.0 m, S = 2.0 m² → AR = 8.0
    AircraftGeometry geom = referenceUav();
    geom.wing = {4.0f, 2.0f, 0.0f, 1.0f, 0.50f};
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);

    // e0 = 1/(1+0.007·π·8); qc_sweep = 0 → e = e0 · cos(0 - 0.09)
    const float e0       = 1.f / (1.f + 0.007f * static_cast<float>(M_PI) * 8.f);
    const float expected = e0 * std::cos(-0.09f);
    EXPECT_NEAR(aero.e(), expected, expected * 0.001f);
}

// 6. CD0 component-buildup result is within a physically plausible range for a
//    clean small UAV (wide tolerance; exact value depends on Re reference condition).
TEST(AeroCoeffEstimatorTest, Cd0_ComponentBuildup_PhysicallyPlausible) {
    const AircraftGeometry geom = referenceUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);
    EXPECT_GT(aero.cd0(), 0.015f);
    EXPECT_LT(aero.cd0(), 0.060f);
}

// 7. C_Yβ is negative (stabilizing) and its magnitude increases when S_VT doubles.
TEST(AeroCoeffEstimatorTest, CyBeta_Negative_IncreasesInMagnitudeWithSvt) {
    AircraftGeometry geom = referenceUav();
    auto [aero1, lcp1] = AeroCoeffEstimator::estimate(geom);

    geom.v_tail.area_m2 *= 2.f;
    auto [aero2, lcp2] = AeroCoeffEstimator::estimate(geom);

    EXPECT_LT(aero1.clYBeta(), 0.f);
    EXPECT_LT(aero2.clYBeta(), aero1.clYBeta());  // more negative = larger magnitude
}

// 8. C_Lq for the moderate-AR UAV falls in the typical UAV range [3, 12] rad⁻¹.
TEST(AeroCoeffEstimatorTest, ClQ_ModerateAR_InRange3to12) {
    const AircraftGeometry geom = moderateArUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);
    EXPECT_GE(aero.clQNd(), 3.f);
    EXPECT_LE(aero.clQNd(), 12.f);
}

// 9. C_Yr is positive and increases when the vertical tail is moved further aft.
TEST(AeroCoeffEstimatorTest, CyR_Positive_IncreasesWithFinArm) {
    AircraftGeometry geom = referenceUav();
    auto [aero1, lcp1] = AeroCoeffEstimator::estimate(geom);

    geom.v_tail.x_le_root_m += 0.3f;  // longer moment arm
    auto [aero2, lcp2] = AeroCoeffEstimator::estimate(geom);

    EXPECT_GT(aero1.cyRNd(), 0.f);
    EXPECT_GT(aero2.cyRNd(), aero1.cyRNd());
}

// 10. estimate() produces an AeroPerformance that passes a JSON round-trip:
//     forces match and all four new fields are preserved.
TEST(AeroCoeffEstimatorTest, RoundTrip_EstimateProducesSerializableAeroPerformance) {
    const AircraftGeometry geom = referenceUav();
    auto [aero, lcp] = AeroCoeffEstimator::estimate(geom);

    const nlohmann::json j = aero.serializeJson();
    const AeroPerformance restored = AeroPerformance::deserializeJson(j);

    const AeroForces f1 = aero.compute(0.05f, 0.02f, 1000.f, 0.4f);
    const AeroForces f2 = restored.compute(0.05f, 0.02f, 1000.f, 0.4f);
    EXPECT_NEAR(f1.x_n, f2.x_n, 1e-3f);
    EXPECT_NEAR(f1.y_n, f2.y_n, 1e-3f);
    EXPECT_NEAR(f1.z_n, f2.z_n, 1e-3f);

    EXPECT_NEAR(aero.clQNd(),   restored.clQNd(),   1e-4f);
    EXPECT_NEAR(aero.macM(),    restored.macM(),    1e-4f);
    EXPECT_NEAR(aero.cyRNd(),   restored.cyRNd(),   1e-4f);
    EXPECT_NEAR(aero.finArmM(), restored.finArmM(), 1e-4f);
}

// 11. estimate() throws std::invalid_argument for degenerate geometry (zero area).
TEST(AeroCoeffEstimatorTest, InvalidGeometry_ZeroArea_Throws) {
    AircraftGeometry geom = referenceUav();
    geom.wing.area_m2 = 0.f;
    EXPECT_THROW({ (void)AeroCoeffEstimator::estimate(geom); }, std::invalid_argument);
}
