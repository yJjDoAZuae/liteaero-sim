#pragma once

#include "aerodynamics/LiftCurveModel.hpp"
#include <cstdint>
#include <vector>

// Inputs to the load-factor allocator for a single solve() call.
struct LoadFactorInputs {
    float n_z;            // commanded normal load factor (g)
    float n_y;            // commanded lateral load factor (g)
    float q_inf;          // dynamic pressure (Pa)
    float thrust_n;       // thrust magnitude (N)
    float mass_kg;        // aircraft mass (kg)
    float n_z_dot = 0.f;  // time derivative of n_z (1/s) — used to compute alphaDot
    float n_y_dot = 0.f;  // time derivative of n_y (1/s) — used to compute betaDot
    float dt_s    = 0.f;  // time step (s) — used by alpha rate bridge and CL recovery
};

// Outputs produced by the load-factor allocator.
struct LoadFactorOutputs {
    float            alpha_rad;                                // angle of attack (rad)
    float            beta_rad;                                 // sideslip angle (rad)
    LiftCurveSegment alpha_segment = LiftCurveSegment::Linear; // lift curve segment containing α
    float            alphaDot_rps  = 0.f;   // d(alpha)/dt (rad/s) — analytical, via implicit function theorem
    float            betaDot_rps   = 0.f;   // d(beta)/dt  (rad/s) — analytical, via implicit function theorem
    int              iterations    = 0;     // Newton iterations taken by the α solver (diagnostic)
    float            n_z_realized  = 0.f;   // load factor actually delivered (equals commanded when not stalled)
    bool             stalled       = false; // positive-side stall hysteresis flag
    bool             stalled_neg   = false; // negative-side stall hysteresis flag
    float            cl_eff        = 0.f;   // effective CL used (rate-limited during recovery, else nominal)
    bool             cl_recovering = false; // true while CL recovery bridge is active
};

// Implicit Newton solver that maps commanded load factors (n, n_y) to angle of
// attack and sideslip.
//
// Normal equation:   f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0
// Lateral equation:  g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
//
// α is solved first; β is solved using the converged α.  Branch continuity is
// maintained by warm-starting each Newton iteration from the previous solution.
class LoadFactorAllocator {
public:
    // liftCurve    — reference must outlive this object
    // S_ref_m2     — reference wing area (m²)
    // cl_y_beta    — lateral force coefficient slope C_Yβ (rad⁻¹, typically < 0)
    // alpha_min_rad / alpha_max_rad — hard physical alpha limits enforced as a box
    //   constraint at every Newton iterate.  Default values are ±π/2, which are
    //   non-binding for all practical configurations.
    explicit LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                 float S_ref_m2,
                                 float cl_y_beta,
                                 float alpha_min_rad       = -kHalfPi,
                                 float alpha_max_rad       =  kHalfPi,
                                 float alpha_dot_max_rad_s =  0.0f);

    LoadFactorOutputs solve(const LoadFactorInputs& in);

    // Reset warm-start state; call before a discontinuous change in demand.
    void reset(float alpha0_rad = 0.0f, float beta0_rad = 0.0f);

    // Serialization.  Note: _lift reference is NOT serialized.  Callers must
    // construct LoadFactorAllocator with the matching LiftCurveModel before
    // calling deserialize.
    nlohmann::json       serializeJson()                              const;
    void                 deserializeJson(const nlohmann::json&        j);

    std::vector<uint8_t> serializeProto()                            const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);

private:
    const LiftCurveModel& _lift;
    float _S;
    float _cl_y_beta;
    float _alpha_min_rad;
    float _alpha_max_rad;
    float _alpha_dot_max_rad_s;
    float _alpha_prev;
    float _beta_prev;
    float _n_z_prev;            // n_z from the previous solve() call — used by branch-continuation predictor
    float _n_y_prev;            // n_y from the previous solve() call — used by branch-continuation predictor
    bool  _stalled;             // positive-side stall hysteresis flag
    bool  _stalled_neg;         // negative-side stall hysteresis flag
    bool  _recovering;          // positive-side post-stall CL recovery in progress
    bool  _recovering_neg;      // negative-side post-stall CL recovery in progress
    float _cl_recovering;       // effective CL (positive side) — tracks nominal when not stalled
    float _cl_recovering_neg;   // effective CL (negative side) — tracks nominal when not stalled

    static constexpr int   kMaxIter = 20;
    static constexpr float kTol     = 1e-6f;
    static constexpr float kGravity = 9.80665f; // m/s²
    static constexpr float kHalfPi  = 1.5707963f; // π/2 — the default non-binding alpha limit
};
