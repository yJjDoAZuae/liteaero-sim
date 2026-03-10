#pragma once

namespace liteaerosim::propulsion {

// Plain value type: propeller geometry and non-dimensional aerodynamic coefficients.
// No virtual methods, no mutable state.  Construct once per aircraft configuration.
//
// Thrust / torque model (advance ratio J = V / (n·D), n in rev/s):
//   C_T(J) = CT0 · (1 − J/J₀)²
//   C_Q(J) = CQ0 + (CT0/J₀²) · J²
//   T = C_T · ρ · n² · D⁴
//   Q = C_Q · ρ · n² · D⁵
struct PropellerAero {
    // ── Geometry parameters (set by constructor) ─────────────────────────────
    float diameter_m;       // Propeller diameter (m)
    float pitch_m;          // Geometric pitch (m) — advance per revolution at zero slip
    int   blade_count;      // Number of blades
    float blade_solidity;   // σ = N_b · c_mean / (π · R), where R = D/2

    // ── Derived quantities (computed at construction) ─────────────────────────
    float disk_area_m2;     // π · (D/2)²
    float J_zero;           // Zero-thrust advance ratio = pitch_m / diameter_m
    float CT0;              // Static thrust coefficient
    float CQ0;              // Profile torque coefficient

    // Construct and compute all derived quantities.
    PropellerAero(float diameter_m, float pitch_m, int blade_count, float blade_solidity);

    // Advance ratio J, clamped to [0, J_zero].  Returns 0 when Omega_rps ≤ 0.
    [[nodiscard]] float advanceRatio(float Omega_rps, float tas_mps) const;

    // Non-dimensional coefficients.
    [[nodiscard]] float thrustCoeff(float J) const;   // C_T(J)
    [[nodiscard]] float torqueCoeff(float J) const;   // C_Q(J)

    // Dimensional thrust (N) and torque (N·m).  Returns 0 when Omega_rps ≤ 0.
    [[nodiscard]] float thrust_n (float Omega_rps, float tas_mps, float rho_kgm3) const;
    [[nodiscard]] float torque_nm(float Omega_rps, float tas_mps, float rho_kgm3) const;
};

} // namespace liteaerosim::propulsion
