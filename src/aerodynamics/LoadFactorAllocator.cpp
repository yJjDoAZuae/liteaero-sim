#include "aerodynamics/LoadFactorAllocator.hpp"
#include <cmath>

LoadFactorAllocator::LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                         float S_ref_m2,
                                         float cl_y_beta)
    : _lift(liftCurve), _S(S_ref_m2), _cl_y_beta(cl_y_beta),
      _alpha_prev(0.0f), _beta_prev(0.0f) {}

void LoadFactorAllocator::reset(float alpha0_rad, float beta0_rad) {
    _alpha_prev = alpha0_rad;
    _beta_prev  = beta0_rad;
}

LoadFactorOutputs LoadFactorAllocator::solve(const LoadFactorInputs& in) {
    const float mg = in.mass_kg * kGravity;
    const float qS = in.q_inf * _S;
    const float T  = in.thrust_n;

    // ── α solver ─────────────────────────────────────────────────────────────
    // f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0
    // f'(α) = q·S·C_L'(α) + T·cos(α)
    float alpha = _alpha_prev;
    bool  stall = false;

    const float alpha_peak = _lift.alphaPeak();
    for (int i = 0; i < kMaxIter; ++i) {
        const float fval   = qS * _lift.evaluate(alpha)   + T * std::sin(alpha) - in.n * mg;
        const float fprime = qS * _lift.derivative(alpha) + T * std::cos(alpha);

        // Fold guard: derivative vanishes at the stall vertex (C_L'=0, T≈0).
        if (std::abs(fprime) < kTol) {
            stall = true;
            alpha = alpha_peak;
            break;
        }

        const float alpha_new = alpha - fval / fprime;

        // Overshoot guard: if Newton would cross the pre-stall peak, demand
        // exceeds the CL ceiling and no pre-stall solution exists.
        if (alpha_new > alpha_peak) {
            stall = true;
            alpha = alpha_peak;
            break;
        }

        const bool converged = std::abs(alpha_new - alpha) < kTol;
        alpha = alpha_new;
        if (converged) break;
    }

    _alpha_prev = alpha;

    // ── β solver ─────────────────────────────────────────────────────────────
    // g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
    // g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)   (always ≤ 0 → unique root)
    float beta     = _beta_prev;
    const float Tca = T * std::cos(alpha);

    for (int i = 0; i < kMaxIter; ++i) {
        const float gval   = qS * _cl_y_beta * beta - Tca * std::sin(beta) - in.n_y * mg;
        const float gprime = qS * _cl_y_beta - Tca * std::cos(beta);

        if (std::abs(gprime) < kTol) break; // degenerate (should not occur for typical params)

        const float beta_new = beta - gval / gprime;
        const bool  converged = std::abs(beta_new - beta) < kTol;
        beta = beta_new;
        if (converged) break;
    }

    _beta_prev = beta;

    return {alpha, beta, stall};
}
