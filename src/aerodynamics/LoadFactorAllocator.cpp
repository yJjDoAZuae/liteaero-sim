#include "aerodynamics/LoadFactorAllocator.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

LoadFactorAllocator::LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                         float S_ref_m2,
                                         float cl_y_beta,
                                         float alpha_min_rad,
                                         float alpha_max_rad,
                                         float alpha_dot_max_rad_s)
    : _lift(liftCurve), _S(S_ref_m2), _cl_y_beta(cl_y_beta),
      _alpha_min_rad(alpha_min_rad), _alpha_max_rad(alpha_max_rad),
      _alpha_dot_max_rad_s(alpha_dot_max_rad_s),
      _alpha_prev(0.0f), _beta_prev(0.0f),
      _n_z_prev(0.0f), _n_y_prev(0.0f),
      _stalled(false), _stalled_neg(false),
      _recovering(false), _recovering_neg(false),
      _cl_recovering(0.0f), _cl_recovering_neg(0.0f) {}

void LoadFactorAllocator::reset(float alpha0_rad, float beta0_rad) {
    _alpha_prev       = alpha0_rad;
    _beta_prev        = beta0_rad;
    _n_z_prev         = 0.0f;
    _n_y_prev         = 0.0f;
    _stalled          = false;
    _stalled_neg      = false;
    _recovering       = false;
    _recovering_neg   = false;
    _cl_recovering    = 0.0f;
    _cl_recovering_neg = 0.0f;
}

LoadFactorOutputs LoadFactorAllocator::solve(const LoadFactorInputs& in) {
    const float mg = in.mass_kg * kGravity;
    const float qS = in.q_inf * _S;
    const float T  = in.thrust_n;

    bool  positive_stall  = false;
    bool  negative_stall  = false;
    int   alpha_iterations = 0;
    float alpha_eq;

    // ── α solve ───────────────────────────────────────────────────────────────
    if (_stalled) {
        // Explicit solve: α_eq = arcsin((n·mg − qS·_cl_recovering) / T)
        // When T ≈ 0 there is no alpha solution; hold the previous alpha.
        if (T > kTol) {
            const float sin_a = (in.n_z * mg - qS * _cl_recovering) / T;
            alpha_eq = (std::abs(sin_a) <= 1.0f) ? std::asin(sin_a) : _alpha_prev;
        } else {
            alpha_eq = _alpha_prev;
        }
    } else if (_stalled_neg) {
        // Explicit solve using negative recovering CL
        if (T > kTol) {
            const float sin_a = (in.n_z * mg - qS * _cl_recovering_neg) / T;
            alpha_eq = (std::abs(sin_a) <= 1.0f) ? std::asin(sin_a) : _alpha_prev;
        } else {
            alpha_eq = _alpha_prev;
        }
    } else {
        // ── Newton solve ───────────────────────────────────────────────────────
        // Branch-continuation predictor: first-order warm-start via implicit function theorem.
        //   α₀ = α_prev + δn_z · m·g / f′(α_prev)
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

            // Box constraint: project the proposed step onto [alpha_min, alpha_max].
            const float alpha_raw = alpha - fval / fprime;
            const float alpha_new = std::clamp(alpha_raw, _alpha_min_rad, _alpha_max_rad);
            if (alpha_raw > _alpha_max_rad && _alpha_max_rad < _lift.alphaSep()) {
                const float f_at_max = qS * _lift.evaluate(_alpha_max_rad) + T * std::sin(_alpha_max_rad) - in.n_z * mg;
                if (f_at_max > 0.0f) {
                    alpha = _alpha_max_rad;
                    break;
                }
            }
            if (alpha_raw < _alpha_min_rad && _alpha_min_rad > _lift.alphaSepNeg()) {
                const float f_at_min = qS * _lift.evaluate(_alpha_min_rad) + T * std::sin(_alpha_min_rad) - in.n_z * mg;
                if (f_at_min < 0.0f) {
                    alpha = _alpha_min_rad;
                    break;
                }
            }

            // Overshoot guards: clamp the proposed step to the parabolic CL domain
            // before evaluating f' to avoid escaping into the flat separated plateau.
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
                    float fold = 0.5f * (lo + hi);
                    if (_alpha_max_rad > fold && _alpha_max_rad < kHalfPi) {
                        const float fp_at_max = qS * _lift.derivative(_alpha_max_rad) + T * std::cos(_alpha_max_rad);
                        if (fp_at_max > 0.0f) {
                            const float f_at_max = qS * _lift.evaluate(_alpha_max_rad) + T * std::sin(_alpha_max_rad) - in.n_z * mg;
                            if (f_at_max > 0.0f) fold = _alpha_max_rad;
                        }
                    }
                    alpha = std::clamp(fold, _alpha_min_rad, _alpha_max_rad);
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
                    float fold = 0.5f * (lo + hi);
                    if (_alpha_min_rad < fold && _alpha_min_rad > -kHalfPi) {
                        const float fp_at_min = qS * _lift.derivative(_alpha_min_rad) + T * std::cos(_alpha_min_rad);
                        if (fp_at_min < 0.0f) {
                            const float f_at_min = qS * _lift.evaluate(_alpha_min_rad) + T * std::sin(_alpha_min_rad) - in.n_z * mg;
                            if (f_at_min < 0.0f) fold = _alpha_min_rad;
                        }
                    }
                    alpha = std::clamp(fold, _alpha_min_rad, _alpha_max_rad);
                    negative_stall = true;
                    break;
                }
            }

            const bool converged = std::abs(alpha_new - alpha) < kTol;
            alpha = alpha_new;
            if (converged) break;
        }
        alpha_eq = alpha;
    }

    _n_z_prev = in.n_z;

    // ── Alpha rate bridge (stall-conditional) ─────────────────────────────────
    // The bridge applies only while _stalled or _stalled_neg is true (previous-step
    // value).  In non-stalled operation alpha_out == alpha_eq exactly.
    float alpha_out;
    if (_stalled || _stalled_neg) {
        const float max_step = _alpha_dot_max_rad_s * in.dt_s;
        alpha_out = _alpha_prev + std::clamp(alpha_eq - _alpha_prev, -max_step, +max_step);
    } else {
        alpha_out = alpha_eq;
    }

    // Stall state at step entry (previous-step value), captured before the in-step hysteresis
    // clear below. Recovery is armed off this so that the step on which a stall clears still
    // begins the rate-limited CL recovery (the exit fires before the recovery update).
    const bool entry_stalled     = _stalled;
    const bool entry_stalled_neg = _stalled_neg;

    // ── Hysteresis flag update ─────────────────────────────────────────────────
    // Uses _alpha_prev (pre-update) as the previous-step alpha.
    // Entry: alpha has passed the peak/trough vertex and is now reversing.
    if (alpha_out < _alpha_prev && _alpha_prev >= _lift.alphaPeak())   _stalled     = true;
    if (alpha_out > _alpha_prev && _alpha_prev <= _lift.alphaTrough()) _stalled_neg = true;
    // Exit: threshold (alphaStar) or snap-down (nominal meets plateau above alphaStar)
    if (alpha_out <= _lift.alphaStar() ||
        (alpha_out < _lift.alphaPeak() && _lift.evaluate(alpha_out) <= _lift.clSep()))
        _stalled = false;
    if (alpha_out >= _lift.alphaStarNeg() ||
        (alpha_out > _lift.alphaTrough() && _lift.evaluate(alpha_out) >= _lift.clSepNeg()))
        _stalled_neg = false;

    _alpha_prev = alpha_out;

    // ── alphaDot: implicit function theorem on f(α, n) = 0 ────────────────────
    // While stalled the Newton loop is bypassed; use the flat-CL IFT derivative.
    const float fprime_alpha = _stalled     ?  T * std::cos(alpha_out)
                             : _stalled_neg ? -T * std::cos(alpha_out)
                             : qS * _lift.derivative(alpha_out) + T * std::cos(alpha_out);
    float alphaDot = 0.f;
    if (!positive_stall && !negative_stall && std::abs(fprime_alpha) > kTol) {
        alphaDot = mg * in.n_z_dot / fprime_alpha;
    }

    // ── CL recovery update ─────────────────────────────────────────────────────
    // The CL rate limit models separated-flow reattachment and therefore applies ONLY while
    // recovering from a genuine stall — NOT in normal attached flight. In non-stalled,
    // non-recovering operation CL tracks the nominal model directly (the longitudinal response is
    // damped by the FBW n_z command filter, not by this limiter). A stall arms recovery; once CL
    // has caught up to nominal the recovery clears. (A previous version rate-limited the upward CL
    // return in ALL non-stalled flight, which both manufactured a zero-lift cold-start transient
    // and asymmetrically lagged CL under normal maneuvering.)
    const float cl_dot_max = _lift.clAlpha() * _alpha_dot_max_rad_s;
    const float cl_nom     = _lift.evaluate(alpha_out);

    if (_stalled) {
        // Hold plateau, snap instantly to nominal if nominal descends below clSep; arm recovery.
        _cl_recovering = std::min(_lift.clSep(), cl_nom);
        _recovering    = true;
    } else if ((entry_stalled || _recovering) && cl_dot_max > 0.0f && _cl_recovering < cl_nom) {
        // Post-stall recovery (the clearing step, via entry_stalled, or a continuing recovery):
        // rate-limit the upward return toward nominal.
        _cl_recovering = std::min(cl_nom, _cl_recovering + cl_dot_max * in.dt_s);
        _recovering    = (_cl_recovering < cl_nom);
    } else {
        // Normal attached flow: track nominal directly (no rate limit).
        _cl_recovering = cl_nom;
        _recovering    = false;
    }

    if (_stalled_neg) {
        // Hold plateau, snap instantly to nominal if nominal rises above clSepNeg; arm recovery.
        _cl_recovering_neg = std::max(_lift.clSepNeg(), cl_nom);
        _recovering_neg    = true;
    } else if ((entry_stalled_neg || _recovering_neg) && cl_dot_max > 0.0f && _cl_recovering_neg > cl_nom) {
        _cl_recovering_neg = std::max(cl_nom, _cl_recovering_neg - cl_dot_max * in.dt_s);
        _recovering_neg    = (_cl_recovering_neg > cl_nom);
    } else {
        _cl_recovering_neg = cl_nom;
        _recovering_neg    = false;
    }

    const float cl_eff       = _cl_recovering;
    const bool  cl_is_recov  = (cl_eff < cl_nom);
    const float n_z_realized = (qS * cl_eff + T * std::sin(alpha_out)) / mg;

    // ── β solver ──────────────────────────────────────────────────────────────
    // g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
    const float Tca = T * std::cos(alpha_out);

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

        if (std::abs(gprime) < kTol) break;

        const float beta_new  = beta - gval / gprime;
        const bool  converged = std::abs(beta_new - beta) < kTol;
        beta = beta_new;
        if (converged) break;
    }

    _beta_prev = beta;
    _n_y_prev  = in.n_y;

    // ── betaDot: implicit function theorem on g(β, n_y, α) = 0 ───────────────
    const float gprime_beta = qS * _cl_y_beta - Tca * std::cos(beta);
    const float dg_dalpha   = T * std::sin(alpha_out) * std::sin(beta);
    float betaDot = 0.f;
    if (std::abs(gprime_beta) > kTol) {
        betaDot = (mg * in.n_y_dot - dg_dalpha * alphaDot) / gprime_beta;
    }

    return {alpha_out, beta, _lift.classify(alpha_out), alphaDot, betaDot, alpha_iterations,
            n_z_realized, _stalled, _stalled_neg, cl_eff, cl_is_recov};
}

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json LoadFactorAllocator::serializeJson() const {
    return {
        {"schema_version",       1},
        {"type",                 "LoadFactorAllocator"},
        {"s_ref_m2",             _S},
        {"cl_y_beta",            _cl_y_beta},
        {"alpha_min_rad",        _alpha_min_rad},
        {"alpha_max_rad",        _alpha_max_rad},
        {"alpha_dot_max_rad_s",  _alpha_dot_max_rad_s},
        {"alpha_prev_rad",       _alpha_prev},
        {"beta_prev_rad",        _beta_prev},
        {"n_z_prev_nd",          _n_z_prev},
        {"n_y_prev_nd",          _n_y_prev},
        {"stalled",              _stalled},
        {"stalled_neg",          _stalled_neg},
        {"recovering",           _recovering},
        {"recovering_neg",       _recovering_neg},
        {"cl_recovering",        _cl_recovering},
        {"cl_recovering_neg",    _cl_recovering_neg},
    };
}

void LoadFactorAllocator::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("LoadFactorAllocator::deserializeJson: unsupported schema_version");
    }
    _S                   = j.at("s_ref_m2").get<float>();
    _cl_y_beta           = j.at("cl_y_beta").get<float>();
    _alpha_min_rad       = j.at("alpha_min_rad").get<float>();
    _alpha_max_rad       = j.at("alpha_max_rad").get<float>();
    _alpha_dot_max_rad_s = j.at("alpha_dot_max_rad_s").get<float>();
    _alpha_prev          = j.at("alpha_prev_rad").get<float>();
    _beta_prev           = j.at("beta_prev_rad").get<float>();
    _n_z_prev            = j.at("n_z_prev_nd").get<float>();
    _n_y_prev            = j.at("n_y_prev_nd").get<float>();
    _stalled             = j.at("stalled").get<bool>();
    _stalled_neg         = j.at("stalled_neg").get<bool>();
    _recovering          = j.value("recovering",     false);
    _recovering_neg      = j.value("recovering_neg", false);
    _cl_recovering       = j.at("cl_recovering").get<float>();
    _cl_recovering_neg   = j.at("cl_recovering_neg").get<float>();
}

std::vector<uint8_t> LoadFactorAllocator::serializeProto() const {
    las_proto::LoadFactorAllocatorState proto;
    proto.set_schema_version(1);
    proto.set_s_ref_m2(_S);
    proto.set_cl_y_beta(_cl_y_beta);
    proto.set_alpha_min_rad(_alpha_min_rad);
    proto.set_alpha_max_rad(_alpha_max_rad);
    proto.set_alpha_dot_max_rad_s(_alpha_dot_max_rad_s);
    proto.set_alpha_prev_rad(_alpha_prev);
    proto.set_beta_prev_rad(_beta_prev);
    proto.set_n_z_prev_nd(_n_z_prev);
    proto.set_n_y_prev_nd(_n_y_prev);
    proto.set_stalled(_stalled);
    proto.set_stalled_neg(_stalled_neg);
    proto.set_recovering(_recovering);
    proto.set_recovering_neg(_recovering_neg);
    proto.set_cl_recovering(_cl_recovering);
    proto.set_cl_recovering_neg(_cl_recovering_neg);
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
    _S                   = proto.s_ref_m2();
    _cl_y_beta           = proto.cl_y_beta();
    _alpha_min_rad       = proto.alpha_min_rad();
    _alpha_max_rad       = proto.alpha_max_rad();
    _alpha_dot_max_rad_s = proto.alpha_dot_max_rad_s();
    _alpha_prev          = proto.alpha_prev_rad();
    _beta_prev           = proto.beta_prev_rad();
    _n_z_prev            = proto.n_z_prev_nd();
    _n_y_prev            = proto.n_y_prev_nd();
    _stalled             = proto.stalled();
    _stalled_neg         = proto.stalled_neg();
    _recovering          = proto.recovering();
    _recovering_neg      = proto.recovering_neg();
    _cl_recovering       = proto.cl_recovering();
    _cl_recovering_neg   = proto.cl_recovering_neg();
}
