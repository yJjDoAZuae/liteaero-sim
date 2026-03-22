#include "aerodynamics/LoadFactorAllocator.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

LoadFactorAllocator::LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                         float S_ref_m2,
                                         float cl_y_beta)
    : _lift(liftCurve), _S(S_ref_m2), _cl_y_beta(cl_y_beta),
      _alpha_prev(0.0f), _beta_prev(0.0f),
      _n_z_prev(0.0f), _n_y_prev(0.0f) {}

void LoadFactorAllocator::reset(float alpha0_rad, float beta0_rad) {
    _alpha_prev = alpha0_rad;
    _beta_prev  = beta0_rad;
    _n_z_prev   = 0.0f;
    _n_y_prev   = 0.0f;
}

LoadFactorOutputs LoadFactorAllocator::solve(const LoadFactorInputs& in) {
    const float mg = in.mass_kg * kGravity;
    const float qS = in.q_inf * _S;
    const float T  = in.thrust_n;

    // ── α solver ─────────────────────────────────────────────────────────────
    // f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0
    // f'(α) = q·S·C_L'(α) + T·cos(α)

    // Branch-continuation predictor: first-order warm-start via implicit function theorem.
    //   α₀ = α_prev + δn_z · m·g / f′(α_prev)
    // In the linear lift region this places Newton exactly at the solution, so
    // the solver converges in a single iteration.
    // The predictor is applied only when the raw prediction stays within the
    // parabolic domain [alphaSepNeg, alphaSep].  If the step would overshoot that
    // boundary (e.g., a cold-start excess-demand call) the warm-start falls back
    // to α_prev, keeping the overshoot guards operative.
    float alpha = _alpha_prev;
    {
        const float fprime_prev = qS * _lift.derivative(_alpha_prev) + T * std::cos(_alpha_prev);
        if (std::abs(fprime_prev) > kTol) {
            const float alpha_pred = _alpha_prev + (in.n_z - _n_z_prev) * mg / fprime_prev;
            if (alpha_pred >= _lift.alphaSepNeg() && alpha_pred <= _lift.alphaSep()) {
                alpha = alpha_pred;
            }
        }
    }

    bool  positive_stall = false;
    bool  negative_stall = false;
    int   alpha_iterations = 0;

    for (int i = 0; i < kMaxIter; ++i) {
        alpha_iterations = i + 1;
        const float fval   = qS * _lift.evaluate(alpha)   + T * std::sin(alpha) - in.n_z * mg;
        const float fprime = qS * _lift.derivative(alpha) + T * std::cos(alpha);

        // Fold guard: f'(α) ≈ 0 means we are at α* (the f'-zero crossing that
        // caps the achievable Nz envelope).  Stay at the current α — it already
        // is the best achievable value — rather than snapping to alpha_peak,
        // which would be wrong for T > 0 where α* > alpha_peak.
        if (std::abs(fprime) < kTol) {
            if (alpha >= 0.0f) {
                positive_stall = true;
            } else {
                negative_stall = true;
            }
            break;
        }

        const float alpha_new = alpha - fval / fprime;

        // Overshoot guards: if Newton would cross α* (the f'-zero crossing that caps
        // the achievable Nz envelope) there is no higher-|α| pre-stall solution.
        // Clamp the proposed step to the parabolic CL domain [alpha_sep_neg, alpha_sep]
        // before evaluating f' — in the flat separated plateau f' = T·cos(α), which
        // can stay positive well past alpha_sep and mislead Newton to escape entirely.
        if (alpha_new > alpha) {
            const float alpha_hi = std::min(alpha_new, _lift.alphaSep());
            const float fp_hi    = qS * _lift.derivative(alpha_hi) + T * std::cos(alpha_hi);
            if (fp_hi <= 0.0f) {
                float lo = alpha, hi = alpha_hi;
                for (int j = 0; j < 30; ++j) {
                    const float mid = 0.5f * (lo + hi);
                    if (qS * _lift.derivative(mid) + T * std::cos(mid) > 0.0f) lo = mid;
                    else                                                          hi = mid;
                }
                alpha = 0.5f * (lo + hi);
                positive_stall = true;
                break;
            }
        }
        if (alpha_new < alpha) {
            const float alpha_lo = std::max(alpha_new, _lift.alphaSepNeg());
            const float fp_lo    = qS * _lift.derivative(alpha_lo) + T * std::cos(alpha_lo);
            if (fp_lo <= 0.0f) {
                float lo = alpha_lo, hi = alpha;
                for (int j = 0; j < 30; ++j) {
                    const float mid = 0.5f * (lo + hi);
                    if (qS * _lift.derivative(mid) + T * std::cos(mid) > 0.0f) hi = mid;
                    else                                                          lo = mid;
                }
                alpha = 0.5f * (lo + hi);
                negative_stall = true;
                break;
            }
        }

        const bool converged = std::abs(alpha_new - alpha) < kTol;
        alpha = alpha_new;
        if (converged) break;
    }

    _alpha_prev = alpha;
    _n_z_prev   = in.n_z;

    // ── alphaDot: implicit function theorem on f(α, n) = 0 ───────────────────
    // df/dα · dα/dt + df/dn · dn/dt = 0
    // df/dn_z = -m·g  →  dα/dt = m·g · n_z_dot / f'(α)
    const float fprime_alpha = qS * _lift.derivative(alpha) + T * std::cos(alpha);
    float alphaDot = 0.f;
    if (!positive_stall && !negative_stall && std::abs(fprime_alpha) > kTol) {
        alphaDot = mg * in.n_z_dot / fprime_alpha;
    }

    // ── β solver ─────────────────────────────────────────────────────────────
    // g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
    // g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)   (always ≤ 0 → unique root)
    const float Tca = T * std::cos(alpha);

    // Branch-continuation predictor for β:
    //   β₀ = β_prev + δn_y · m·g / g′(β_prev)
    float beta = _beta_prev;
    {
        const float gprime_prev = qS * _cl_y_beta - Tca * std::cos(_beta_prev);
        if (std::abs(gprime_prev) > kTol) {
            beta = _beta_prev + (in.n_y - _n_y_prev) * mg / gprime_prev;
        }
    }

    for (int i = 0; i < kMaxIter; ++i) {
        const float gval   = qS * _cl_y_beta * beta - Tca * std::sin(beta) - in.n_y * mg;
        const float gprime = qS * _cl_y_beta - Tca * std::cos(beta);

        if (std::abs(gprime) < kTol) break; // degenerate (should not occur for typical params)

        const float beta_new  = beta - gval / gprime;
        const bool  converged = std::abs(beta_new - beta) < kTol;
        beta = beta_new;
        if (converged) break;
    }

    _beta_prev = beta;
    _n_y_prev  = in.n_y;

    // ── betaDot: implicit function theorem on g(β, n_y, α) = 0 ───────────────
    // dg/dβ · dβ/dt + dg/dα · dα/dt + dg/dn_y · dn_y/dt = 0
    // dg/dβ  = q·S·C_Yβ − T·cos(α)·cos(β)
    // dg/dα  = T·sin(α)·sin(β)
    // dg/dn_y = -m·g
    const float gprime_beta = qS * _cl_y_beta - Tca * std::cos(beta);
    const float dg_dalpha   = T * std::sin(alpha) * std::sin(beta);
    float betaDot = 0.f;
    if (std::abs(gprime_beta) > kTol) {
        betaDot = (mg * in.n_y_dot - dg_dalpha * alphaDot) / gprime_beta;
    }

    return {alpha, beta, _lift.classify(alpha), alphaDot, betaDot, alpha_iterations};
}

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json LoadFactorAllocator::serializeJson() const {
    return {
        {"schema_version",  1},
        {"type",            "LoadFactorAllocator"},
        {"s_ref_m2",        _S},
        {"cl_y_beta",       _cl_y_beta},
        {"alpha_prev_rad",  _alpha_prev},
        {"beta_prev_rad",   _beta_prev},
        {"n_z_prev_nd",     _n_z_prev},
        {"n_y_prev_nd",     _n_y_prev},
    };
}

void LoadFactorAllocator::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("LoadFactorAllocator::deserializeJson: unsupported schema_version");
    }
    _S          = j.at("s_ref_m2").get<float>();
    _cl_y_beta  = j.at("cl_y_beta").get<float>();
    _alpha_prev = j.at("alpha_prev_rad").get<float>();
    _beta_prev  = j.at("beta_prev_rad").get<float>();
    _n_z_prev   = j.at("n_z_prev_nd").get<float>();
    _n_y_prev   = j.at("n_y_prev_nd").get<float>();
}

std::vector<uint8_t> LoadFactorAllocator::serializeProto() const {
    las_proto::LoadFactorAllocatorState proto;
    proto.set_schema_version(1);
    proto.set_s_ref_m2(_S);
    proto.set_cl_y_beta(_cl_y_beta);
    proto.set_alpha_prev_rad(_alpha_prev);
    proto.set_beta_prev_rad(_beta_prev);
    proto.set_n_z_prev_nd(_n_z_prev);
    proto.set_n_y_prev_nd(_n_y_prev);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void LoadFactorAllocator::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::LoadFactorAllocatorState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("LoadFactorAllocator::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error("LoadFactorAllocator::deserializeProto: unsupported schema_version");
    }
    _S          = proto.s_ref_m2();
    _cl_y_beta  = proto.cl_y_beta();
    _alpha_prev = proto.alpha_prev_rad();
    _beta_prev  = proto.beta_prev_rad();
    _n_z_prev   = proto.n_z_prev_nd();
    _n_y_prev   = proto.n_y_prev_nd();
}
