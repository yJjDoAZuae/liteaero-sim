#define _USE_MATH_DEFINES
#include "aerodynamics/LoadFactorAllocator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>

// Representative GA aircraft parameters (matching LiftCurveModel_test.cpp).
static LiftCurveParams gaLiftParams() {
    return {5.73f, 1.20f, -1.20f, 0.05f, 0.05f, 0.80f, -0.80f};
}

// Aircraft sizing: chosen so level-flight CL is comfortably within the linear
// region and the stall limit is above 1 g.
//   mass = 1000 kg, S = 16 m², q = 1531 Pa  →  qS ≈ 24 496 N
//   n_1g demand  = m·g = 9 806.65 N  →  CL ≈ 0.40  (linear region, α ≈ 7°)
//   n_max (T=0)  = cl_max·qS/mg ≈ 3.0 g  (stall ceiling with zero thrust)
static const float kMass = 1000.0f;    // kg
static const float kS    =   16.0f;    // m²
static const float kQ    = 1531.0f;    // Pa (≈ 50 m/s at sea level)
static const float kCYb  =   -3.0f;    // C_Yβ (rad⁻¹)
static const float kG    =    9.80665f; // m/s²

struct AllocatorFixture : ::testing::Test {
    LiftCurveModel     lift{gaLiftParams()};
    LoadFactorAllocator alloc{lift, kS, kCYb};
};

// ── Zero demand ───────────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, ZeroDemandZeroAngles) {
    // n=0, n_y=0, T=0 → f(0)=0 and g(0)=0 exactly; solver stays at α=β=0.
    LoadFactorInputs in{0.f, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alpha_rad, 0.f, 1e-5f);
    EXPECT_NEAR(out.beta_rad,  0.f, 1e-5f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::Linear);
}

// ── Linear region, analytic check ─────────────────────────────────────────────

TEST_F(AllocatorFixture, SmallNLinearRegionT0) {
    // With T=0, linear-region solution: α = n·m·g / (q·S·C_Lα).
    // Use n=1 (1 g).  α ≈ 9807 / (24496·5.73) ≈ 0.1711 rad < α_star ≈ 0.234 rad.
    const float n = 1.0f;
    const float alpha_expected = n * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);
    ASSERT_LT(alpha_expected, lift.alphaPeak()); // verify we're in pre-stall

    LoadFactorInputs in{n, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alpha_rad, alpha_expected, 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::Linear);
}

// ── Stall flag ────────────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, PositiveStallClampsAtPeak) {
    // n_max (T=0) ≈ cl_max·qS / (m·g) ≈ 3.0 g.  Use 20 % above ceiling.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    const float n_stall   = n_ceiling * 1.2f;

    LoadFactorInputs in{n_stall, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    // α clamped at the pre-stall peak — sits at the incipient/post boundary.
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallPositive);
}

TEST_F(AllocatorFixture, NegativeStallClampsAtTrough) {
    // Symmetric to positive stall: n more negative than the negative CL ceiling.
    const float n_ceiling_neg = gaLiftParams().cl_min * kQ * kS / (kMass * kG);
    const float n_stall_neg   = n_ceiling_neg * 1.2f; // 20 % more negative

    LoadFactorInputs in{n_stall_neg, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    // α clamped at the negative trough.
    EXPECT_NEAR(out.alpha_rad, lift.alphaTrough(), 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallNegative);
}

// ── Branch tracking ───────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, MonotonicNStaysPreStall) {
    // Step n up from 0 to just below the stall ceiling in 0.1-g increments.
    // Verify that α increases monotonically and stall is never flagged.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    for (float n = 0.1f; n < n_ceiling * 0.95f; n += 0.1f) {
        LoadFactorInputs in{n, 0.f, kQ, 0.f, kMass};
        auto out = alloc.solve(in);
        // α must stay in the pre-stall envelope (linear or incipient, not post-stall or separated).
        const auto seg = out.alpha_segment;
        EXPECT_TRUE(seg == LiftCurveSegment::Linear || seg == LiftCurveSegment::IncipientStallPositive)
            << "unexpected segment at n=" << n;
        EXPECT_GE(out.alpha_rad, alpha_prev) << "non-monotonic α at n=" << n;
        alpha_prev = out.alpha_rad;
    }
}

// ── Lateral, T=0 analytic check ───────────────────────────────────────────────

TEST_F(AllocatorFixture, LateralT0AnalyticCheck) {
    // With T=0, g(β) = q·S·C_Yβ·β − n_y·m·g = 0  →  β = n_y·m·g / (q·S·C_Yβ).
    const float n_y            = 0.2f; // 0.2 g lateral
    const float beta_expected  = n_y * kMass * kG / (kQ * kS * kCYb);

    LoadFactorInputs in{1.0f, n_y, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.beta_rad, beta_expected, 1e-4f);
}

// ── alphaDot: analytical via implicit function theorem ────────────────────────

TEST_F(AllocatorFixture, AlphaDotZeroForConstantN) {
    // n_z_dot=0 → alphaDot=0 at any operating point.
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    // n_z_dot defaults to 0
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, 0.f, 1e-6f);
}

TEST_F(AllocatorFixture, AlphaDotAnalyticLinearRegionT0) {
    // T=0, linear region: f'(α) = q·S·C_Lα  →  alphaDot = m·g·n_z_dot / (q·S·C_Lα).
    const float n_z_dot           = 0.2f;
    const float alphaDot_expected = kMass * kG * n_z_dot
                                    / (kQ * kS * gaLiftParams().cl_alpha);
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    in.n_z_dot = n_z_dot;
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, alphaDot_expected, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaDotZeroAtStall) {
    // Once α is clamped to the stall peak, further load-factor demand cannot
    // increase α — alphaDot should be zero.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    LoadFactorInputs in{n_ceiling * 1.2f, 0.f, kQ, 0.f, kMass};
    in.n_z_dot = 1.f; // non-zero demand rate
    auto out  = alloc.solve(in);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallPositive);
    EXPECT_NEAR(out.alphaDot_rps, 0.f, 1e-6f);
}

// ── betaDot: analytical via implicit function theorem ─────────────────────────

TEST_F(AllocatorFixture, BetaDotZeroForConstantNy) {
    // n_y_dot=0 → betaDot=0.
    LoadFactorInputs in{1.f, 0.2f, kQ, 0.f, kMass};
    // n_y_dot defaults to 0
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.betaDot_rps, 0.f, 1e-6f);
}

TEST_F(AllocatorFixture, BetaDotAnalyticT0) {
    // T=0: g'(β) = q·S·C_Yβ, dg/dα term = 0  →  betaDot = m·g·n_y_dot / (q·S·C_Yβ).
    const float n_y_dot           = 0.1f;
    const float betaDot_expected  = kMass * kG * n_y_dot / (kQ * kS * kCYb);
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    in.n_y_dot = n_y_dot;
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.betaDot_rps, betaDot_expected, 1e-5f);
}

// ── Serialization ─────────────────────────────────────────────────────────────

// Advance the allocator with several solve() calls to build warm-start state.
static void warmUpAllocator(LoadFactorAllocator& alloc) {
    alloc.solve({1.0f, 0.1f, kQ, 0.f, kMass});
    alloc.solve({1.5f, 0.2f, kQ, 0.f, kMass});
    alloc.solve({2.0f, 0.0f, kQ, 0.f, kMass});
}

TEST_F(AllocatorFixture, JsonRoundTrip) {
    warmUpAllocator(alloc);
    const nlohmann::json j = alloc.serializeJson();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeJson(j);

    // Both allocators should produce identical output on the next solve().
    LoadFactorInputs in{1.2f, 0.15f, kQ, 0.f, kMass};
    const LoadFactorOutputs outOriginal = alloc.solve(in);
    const LoadFactorOutputs outRestored = restored.solve(in);
    EXPECT_NEAR(outRestored.alpha_rad, outOriginal.alpha_rad, 1e-5f);
    EXPECT_NEAR(outRestored.beta_rad,  outOriginal.beta_rad,  1e-5f);
}

TEST_F(AllocatorFixture, JsonSchemaVersionMismatchThrows) {
    nlohmann::json j = alloc.serializeJson();
    j["schema_version"] = 99;
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    EXPECT_THROW(restored.deserializeJson(j), std::runtime_error);
}

TEST_F(AllocatorFixture, ProtoRoundTrip) {
    warmUpAllocator(alloc);
    const std::vector<uint8_t> bytes = alloc.serializeProto();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeProto(bytes);

    LoadFactorInputs in{1.2f, 0.15f, kQ, 0.f, kMass};
    const LoadFactorOutputs outOriginal = alloc.solve(in);
    const LoadFactorOutputs outRestored = restored.solve(in);
    EXPECT_NEAR(outRestored.alpha_rad, outOriginal.alpha_rad, 1e-4f);
    EXPECT_NEAR(outRestored.beta_rad,  outOriginal.beta_rad,  1e-4f);
}

// ── Positive-thrust α* ceiling ────────────────────────────────────────────────
//
// With positive thrust the maximum achievable Nz occurs at α* > alpha_peak,
// where f'(α) = qS·CL'(α) + T·cos(α) = 0 (the derivative zero-crossing).
// The solver must:
//   (a) converge to solutions at α > alpha_peak when Nz is in (Nz_peak, Nz_star], and
//   (b) clamp at α* (not alpha_peak) when Nz exceeds the ceiling.
//
// Helper: bisect f'(α) = 0 in [lo, hi] where f'(lo) > 0 and f'(hi) < 0.
static float bisectFprimeCrossing(const LiftCurveModel& lift,
                                   float qS, float T,
                                   float lo, float hi) {
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (qS * lift.derivative(mid) + T * std::cos(mid) > 0.f) lo = mid;
        else                                                        hi = mid;
    }
    return 0.5f * (lo + hi);
}

// Achievable Nz at a given α.
static float achievableNz(const LiftCurveModel& lift,
                           float alpha, float qS, float T, float mg) {
    return (qS * lift.evaluate(alpha) + T * std::sin(alpha)) / mg;
}

// Large positive thrust — shifts α* visibly above alpha_peak.
static constexpr float kLargeThrust = 50000.f; // N

// hi bound for bisectFprimeCrossing: 0.07 rad above alpha_peak puts us well
// inside the post-stall quadratic (below alpha_sep ≈ 0.318 rad for these params)
// where CL' < 0 and f'(hi) < 0 for kLargeThrust.
static float bisectHiBound(const LiftCurveModel& lift) {
    return lift.alphaPeak() + 0.07f;
}

TEST_F(AllocatorFixture, WithThrust_SolutionExistsAboveAlphaPeak) {
    // A commanded Nz between Nz(alpha_peak, T) and Nz(alpha*, T) has a valid
    // solution at α > alpha_peak.  The solver must not clamp prematurely.
    const float qS = kQ * kS;
    const float mg = kMass * kG;

    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));
    const float nz_at_peak = achievableNz(lift, lift.alphaPeak(), qS, kLargeThrust, mg);
    const float nz_at_star = achievableNz(lift, alpha_star,       qS, kLargeThrust, mg);

    ASSERT_GT(alpha_star, lift.alphaPeak()) << "test setup: α* must be above alpha_peak";
    ASSERT_GT(nz_at_star, nz_at_peak)       << "test setup: ceiling must exceed Nz at alpha_peak";

    // Command Nz midway between Nz(alpha_peak) and Nz(alpha*); solution at α > alpha_peak.
    const float n_cmd = 0.5f * (nz_at_peak + nz_at_star);

    // Warm-start from just below to help Newton approach from the right side.
    alloc.solve({n_cmd * 0.9f, 0.f, kQ, kLargeThrust, kMass});
    const auto out = alloc.solve({n_cmd, 0.f, kQ, kLargeThrust, kMass});

    // Equation residual must be near zero (good convergence).
    const float residual = qS * lift.evaluate(out.alpha_rad)
                           + kLargeThrust * std::sin(out.alpha_rad)
                           - n_cmd * mg;
    EXPECT_NEAR(residual, 0.f, 10.f)
        << "f(α) should be satisfied; premature clamp at alpha_peak leaves residual";

    // α must have advanced past alpha_peak toward the true solution.
    EXPECT_GT(out.alpha_rad, lift.alphaPeak())
        << "solver clamped prematurely at alpha_peak with positive thrust";
}

TEST_F(AllocatorFixture, WithThrust_ExcessNzClampsAtFprimeCrossing) {
    // When Nz is above the thrust-augmented ceiling, α must clamp at α*
    // (the f'=0 crossing), not at alpha_peak.
    const float qS = kQ * kS;

    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));

    const auto out = alloc.solve({20.f, 0.f, kQ, kLargeThrust, kMass});

    EXPECT_GT(out.alpha_rad, lift.alphaPeak())
        << "excess Nz with thrust must clamp above alpha_peak";
    EXPECT_NEAR(out.alpha_rad, alpha_star, 1e-3f)
        << "excess Nz with thrust must clamp at α* (f'=0), not alpha_peak";
}

TEST_F(AllocatorFixture, ZeroThrust_ExcessNzStillClampsAtAlphaPeak) {
    // With T=0, α* == alpha_peak (T·cos term vanishes). Existing behaviour must
    // be preserved: clamping at alpha_peak is correct.
    const auto out = alloc.solve({20.f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f);
}

TEST_F(AllocatorFixture, WithThrust_AlphaMonotonicAndReachesAlphaStar) {
    // Sweep Nz from 1 g through and past the thrust-augmented ceiling.
    // α must increase monotonically throughout and must reach α* (not stall
    // at alpha_peak) once Nz exceeds the ceiling.
    const float qS         = kQ * kS;
    const float mg         = kMass * kG;
    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));
    const float nz_ceiling = achievableNz(lift, alpha_star, qS, kLargeThrust, mg);

    float alpha_prev  = 0.f;
    float alpha_final = 0.f;
    for (int i = 0; i <= 100; ++i) {
        const float t   = static_cast<float>(i) / 100.f;
        const float n_z = 1.f + t * (nz_ceiling * 1.5f - 1.f);
        const auto  out = alloc.solve({n_z, 0.f, kQ, kLargeThrust, kMass});

        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α at step " << i << " (nz=" << n_z << ")";
        EXPECT_LE(out.alpha_rad, alpha_star + 1e-3f)
            << "α exceeded α* at step " << i;
        alpha_prev  = out.alpha_rad;
        alpha_final = out.alpha_rad;
    }
    // At 1.5× ceiling the solver must be clamped at α*, not stuck at alpha_peak.
    EXPECT_NEAR(alpha_final, alpha_star, 1e-3f)
        << "α must reach α* when Nz is above the thrust-augmented ceiling";
}

TEST_F(AllocatorFixture, ProtoSchemaVersionMismatchThrows) {
    const std::vector<uint8_t> bytes = alloc.serializeProto();
    // Corrupt: modify schema_version field (field 1, varint, tag = 0x08) to value 99.
    std::vector<uint8_t> bad = bytes;
    for (std::size_t i = 0; i + 1 < bad.size(); ++i) {
        if (bad[i] == 0x08) {
            bad[i + 1] = 99;
            break;
        }
    }
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    EXPECT_THROW(restored.deserializeProto(bad), std::runtime_error);
}

// ── Branch-continuation predictor ─────────────────────────────────────────────
//
// The predictor computes a first-order warm-start:
//   α₀ = α_prev + δn_z · m·g / f′(α_prev)
//   β₀ = β_prev + δn_y · m·g / g′(β_prev)
//
// In the linear lift region f is linear in α, so the predictor is exact: it
// places Newton's initial iterate directly at the solution.  The solver
// converges in a single iteration (i.e. the loop runs once and then checks
// that f(α₀) = 0, yielding |α_new − α₀| = 0 < kTol).
//
// Without the predictor the warm-start is α_prev (the previous solution), so
// a linear step of Δn = 0.1 g requires two loop iterations: one large Newton
// step to the solution, then a second iteration to confirm |Δ| < kTol.
//
// The three tests below will not compile until `iterations` is added to
// LoadFactorOutputs, and the convergence test will fail at runtime until the
// predictor is implemented.

TEST_F(AllocatorFixture, PredictorReducesIterationsOnLinearStep) {
    // Establish warm-start at n=1 g (linear region, α ≈ 0.0699 rad).
    alloc.solve({1.0f, 0.f, kQ, 0.f, kMass});

    // Small step to n=1.1 g — still fully in the linear region.
    // With the predictor, α₀ = α_prev + 0.1·m·g / (q·S·C_Lα) which is the
    // exact solution: f(α₀) = 0 → |α_new − α₀| = 0 → converged on iteration 1.
    const auto out = alloc.solve({1.1f, 0.f, kQ, 0.f, kMass});

    // Correct solution sanity check.
    const float alpha_expected = 1.1f * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);
    EXPECT_NEAR(out.alpha_rad, alpha_expected, 1e-5f);

    // With predictor: 1 iteration.  Without predictor: 2 iterations.
    EXPECT_EQ(out.iterations, 1)
        << "predictor must place the warm-start at the exact solution in the "
           "linear region, converging in 1 iteration";
}

TEST_F(AllocatorFixture, PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev) {
    // After a solve the allocator must persist n_z_prev and n_y_prev so that
    // the branch-continuation predictor state survives serialization.
    alloc.solve({1.5f, 0.1f, kQ, 0.f, kMass});
    const nlohmann::json j = alloc.serializeJson();

    ASSERT_TRUE(j.contains("n_z_prev_nd")) << "JSON missing n_z_prev_nd";
    ASSERT_TRUE(j.contains("n_y_prev_nd")) << "JSON missing n_y_prev_nd";
    EXPECT_NEAR(j["n_z_prev_nd"].get<float>(), 1.5f, 1e-4f);
    EXPECT_NEAR(j["n_y_prev_nd"].get<float>(), 0.1f, 1e-4f);

    // After round-trip, a subsequent linear step must also converge in 1 iteration.
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeJson(j);

    const auto outOrig = alloc.solve({1.6f, 0.1f, kQ, 0.f, kMass});
    const auto outRest = restored.solve({1.6f, 0.1f, kQ, 0.f, kMass});

    EXPECT_NEAR(outRest.alpha_rad, outOrig.alpha_rad, 1e-5f);
    EXPECT_EQ(outRest.iterations, 1)
        << "restored allocator must predict correctly — n_z_prev must be serialized";
}

TEST_F(AllocatorFixture, PredictorProtoRoundTrip_IncludesNzPrev) {
    // Proto serialization must also preserve n_z_prev / n_y_prev.
    alloc.solve({1.5f, 0.1f, kQ, 0.f, kMass});
    const std::vector<uint8_t> bytes = alloc.serializeProto();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeProto(bytes);

    const auto outOrig = alloc.solve({1.6f, 0.1f, kQ, 0.f, kMass});
    const auto outRest = restored.solve({1.6f, 0.1f, kQ, 0.f, kMass});

    EXPECT_NEAR(outRest.alpha_rad, outOrig.alpha_rad, 1e-5f);
    EXPECT_EQ(outRest.iterations, 1)
        << "restored allocator (proto) must predict correctly — n_z_prev must be serialized";
}

// ── Continuity and monotonicity across all lift-curve segments ─────────────────
//
// Derived values for gaLiftParams() (cl_alpha=5.73, cl_max=1.20, delta=0.05):
//   alpha_peak = cl_max/cl_alpha + delta/2  ≈  0.234 rad
//   alpha_star = alpha_peak − delta         ≈  0.184 rad   (Linear↔Incipient join)
//   n_ceiling  = cl_max·qS/mg              ≈  3.0 g
//   n_star     = cl_alpha·alpha_star·qS/mg ≈  2.64 g       (n at the join)
//   Symmetric negative values by construction.

TEST_F(AllocatorFixture, MonotonicNz_PositiveSweepThroughStall) {
    // Sweeps n from 0 to 120 % of the positive stall ceiling, covering the full
    // path: Linear → IncipientStall → clamp at alphaPeak.
    // Extends MonotonicNStaysPreStall (which stops at 95 %) to include the
    // incipient-stall transition and the clamp itself.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    LoadFactorOutputs last{};
    for (int i = 1; i <= 120; ++i) {
        const float n = n_ceiling * (static_cast<float>(i) / 100.f);
        last = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(last.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α at n = " << n << " g (step " << i << ")";
        alpha_prev = last.alpha_rad;
    }
    EXPECT_NEAR(last.alpha_rad, lift.alphaPeak(), 1e-4f)
        << "α must clamp at alphaPeak when Nz exceeds the stall ceiling";
}

TEST_F(AllocatorFixture, MonotonicNz_NegativeSweepThroughStall) {
    // Sweeps n from 0 to 120 % of the negative stall ceiling, covering the full
    // negative path: Linear → IncipientStallNegative → clamp at alphaTrough.
    const float n_ceiling_neg = gaLiftParams().cl_min * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    LoadFactorOutputs last{};
    for (int i = 1; i <= 120; ++i) {
        const float n = n_ceiling_neg * (static_cast<float>(i) / 100.f);
        last = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_LE(last.alpha_rad, alpha_prev + 1e-4f)
            << "non-monotonic α at n = " << n << " g (step " << i << ")";
        alpha_prev = last.alpha_rad;
    }
    EXPECT_NEAR(last.alpha_rad, lift.alphaTrough(), 1e-4f)
        << "α must clamp at alphaTrough when Nz is below the negative stall ceiling";
}

TEST_F(AllocatorFixture, SegmentBoundary_AlphaMonotonicAcrossLinearIncipientJoin) {
    // Fine-step (0.02 g) sweep from 2.5 g through the Linear→IncipientStall
    // join (n_star ≈ 2.64 g) to 2.85 g.  The C¹ join in C_L(α) implies a
    // continuous dα/dn — no jump in α should occur at the segment boundary.
    // Both the Linear and IncipientStallPositive segments must be observed.
    constexpr float kDn = 0.02f;

    // Establish warm-start just below the sweep range.
    alloc.solve({2.4f, 0.f, kQ, 0.f, kMass});
    float alpha_prev  = alloc.solve({2.5f, 0.f, kQ, 0.f, kMass}).alpha_rad;

    bool saw_linear    = false;
    bool saw_incipient = false;

    for (float n = 2.5f + kDn; n <= 2.85f + 1e-4f; n += kDn) {
        const auto out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α near the Linear→IncipientStall join at n = " << n;
        if (out.alpha_segment == LiftCurveSegment::Linear)                  saw_linear    = true;
        if (out.alpha_segment == LiftCurveSegment::IncipientStallPositive)  saw_incipient = true;
        alpha_prev = out.alpha_rad;
    }
    EXPECT_TRUE(saw_linear)    << "sweep must pass through the Linear segment";
    EXPECT_TRUE(saw_incipient) << "sweep must reach the IncipientStallPositive segment";
}

TEST_F(AllocatorFixture, StallRecovery_RequiresReset) {
    // After a stall call (_alpha_prev = alphaPeak), Newton starts at the fold
    // point where f'(alphaPeak) ≈ 0.  In floating-point arithmetic the residual
    // f'(alphaPeak) is nonzero but tiny, so the fold guard (kTol = 1e-6) does
    // not fire; Newton then takes a huge step to the wrong branch.
    //
    // Calling reset() clears the warm-start to zero.  The branch-continuation
    // predictor then places Newton directly at the correct linear-region
    // solution, converging in one iteration.
    //
    // This documents the correct usage: call reset() after any discontinuous
    // change in demand (e.g., re-engagement after a stall event).
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    const float n_1g      = 1.0f;
    const float alpha_1g  = n_1g * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);

    // Drive to stall; warm-start is now at alphaPeak.
    alloc.solve({n_ceiling * 1.5f, 0.f, kQ, 0.f, kMass});

    // Without reset(): stale warm-start at the fold point causes the solver to
    // converge to the wrong branch — result is far from the correct 1 g value.
    const auto out_wrong = alloc.solve({n_1g, 0.f, kQ, 0.f, kMass});
    EXPECT_GT(std::abs(out_wrong.alpha_rad - alpha_1g), 0.1f)
        << "without reset(), stale warm-start at alphaPeak must corrupt the solve";

    // After reset(): predictor finds the exact linear solution in one iteration.
    alloc.reset();
    const auto out_correct = alloc.solve({n_1g, 0.f, kQ, 0.f, kMass});
    EXPECT_EQ(out_correct.alpha_segment, LiftCurveSegment::Linear)
        << "after reset(), solver must return to the Linear segment";
    EXPECT_NEAR(out_correct.alpha_rad, alpha_1g, 1e-4f)
        << "after reset(), α must equal the linear-region solution";
}

// ── Black-box continuity: α must be monotone in Nz across the full domain ─────
//
// These tests treat the allocator as a black box.  They reference only the
// test-fixture sizing parameters (kMass, kS, kQ, kCYb, kLargeThrust) — never
// internal lift-curve quantities such as alphaPeak, alphaStar, or n_ceiling.
//
// Physical basis: for fixed T the normal-force equation f(α,n)=0 defines a
// unique monotone branch α(n).  A positive increment in Nz must produce a
// non-negative increment in α everywhere in the domain, including through the
// stall transition and the clamped plateau.  10 g safely exceeds the stall
// ceiling for both the zero-thrust and large-thrust cases.

TEST_F(AllocatorFixture, Alpha_IsMonotone_ZeroThrust_PositiveSweep) {
    // Sweeps Nz from 0 to +10 g in 0.02 g steps with warm-starting (T = 0).
    // Covers the pre-stall, stall-transition, and clamped-plateau regions.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(i) * 0.02f;   // 0.02 g … 10 g
        const auto  out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "α decreased on positive sweep at n = " << n << " g (T = 0)";
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, Alpha_IsMonotone_ZeroThrust_NegativeSweep) {
    // Sweeps Nz from 0 to −10 g in 0.02 g steps with warm-starting (T = 0).
    // Covers the negative pre-stall, stall-transition, and clamped-plateau regions.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(-i) * 0.02f;  // −0.02 g … −10 g
        const auto  out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_LE(out.alpha_rad, alpha_prev + 1e-4f)
            << "α increased on negative sweep at n = " << n << " g (T = 0)";
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, Alpha_IsMonotone_WithThrust_PositiveSweep) {
    // Sweeps Nz from 0 to +10 g with T = kLargeThrust.  Positive thrust raises
    // the stall ceiling above the zero-thrust alphaPeak; the test verifies that
    // α is monotone through this thrust-augmented ceiling without referencing
    // its location.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(i) * 0.02f;
        const auto  out = alloc.solve({n, 0.f, kQ, kLargeThrust, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "α decreased on positive sweep at n = " << n << " g (T = kLargeThrust)";
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, Alpha_Perturbation_IsMonotone_FullDomain) {
    // At each operating point on a uniform grid, verifies that a positive
    // perturbation δ in Nz produces a non-negative change in α.  Each point
    // uses a fresh allocator so the result is independent of the warm-start
    // path — a point-wise continuity check independent of traversal order.
    //
    // T = 0 case:    n₀ ∈ [−9 g, +9 g], step 0.5 g (37 points).
    // T > 0 case:    n₀ ∈ [  0 g, +9 g], step 0.5 g (19 points).
    //   (Large positive thrust makes very negative Nz infeasible; the positive
    //   half covers the thrust-augmented stall region without hitting that limit.)
    constexpr float kDelta = 0.05f;  // g

    LiftCurveModel scratch_lift(gaLiftParams());

    // T = 0: full signed domain.
    for (int i = -18; i <= 18; ++i) {
        const float n0 = static_cast<float>(i) * 0.5f;  // −9 g … +9 g

        LoadFactorAllocator lo(scratch_lift, kS, kCYb);
        LoadFactorAllocator hi(scratch_lift, kS, kCYb);

        const float alpha_lo = lo.solve({n0 - kDelta, 0.f, kQ, 0.f, kMass}).alpha_rad;
        const float alpha_hi = hi.solve({n0 + kDelta, 0.f, kQ, 0.f, kMass}).alpha_rad;

        EXPECT_LE(alpha_lo, alpha_hi + 1e-4f)
            << "T=0: α(n₀−δ) > α(n₀+δ) at n₀ = " << n0 << " g";
    }

    // T > 0: positive domain only.
    for (int i = 0; i <= 18; ++i) {
        const float n0 = static_cast<float>(i) * 0.5f;  // 0 g … +9 g

        LoadFactorAllocator lo(scratch_lift, kS, kCYb);
        LoadFactorAllocator hi(scratch_lift, kS, kCYb);

        const float alpha_lo = lo.solve({n0 - kDelta, 0.f, kQ, kLargeThrust, kMass}).alpha_rad;
        const float alpha_hi = hi.solve({n0 + kDelta, 0.f, kQ, kLargeThrust, kMass}).alpha_rad;

        EXPECT_LE(alpha_lo, alpha_hi + 1e-4f)
            << "T=kLargeThrust: α(n₀−δ) > α(n₀+δ) at n₀ = " << n0 << " g";
    }
}
