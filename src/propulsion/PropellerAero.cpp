#define _USE_MATH_DEFINES
#include "propulsion/PropellerAero.hpp"
#include <cmath>
#include <algorithm>

namespace liteaerosim::propulsion {

// Blade-element theory constants.
static constexpr float kCLBlade = 6.28318530718f;  // 2π — thin-airfoil lift slope
static constexpr float kCDBlade = 0.01f;            // profile drag coefficient
static constexpr float kPi      = 3.14159265358979f;

// ── Constructor ───────────────────────────────────────────────────────────────

PropellerAero::PropellerAero(float diam, float pitch, int blades, float solidity)
    : diameter_m(diam)
    , pitch_m(pitch)
    , blade_count(blades)
    , blade_solidity(solidity)
{
    disk_area_m2 = kPi * (diam * 0.5f) * (diam * 0.5f);
    J_zero       = pitch / diam;
    const float theta75 = std::atan(pitch / (0.75f * kPi * diam));
    CT0 = (solidity * kCLBlade / 8.f) * theta75;
    CQ0 = solidity * kCDBlade / 8.f;
}

// ── Coefficient methods ───────────────────────────────────────────────────────

float PropellerAero::advanceRatio(float Omega_rps, float tas_mps) const {
    if (Omega_rps <= 0.f) return 0.f;
    const float n = Omega_rps / (2.f * kPi);          // rev/s
    const float J = tas_mps / (n * diameter_m);
    return std::min(J, J_zero);
}

float PropellerAero::thrustCoeff(float J) const {
    const float ratio = J / J_zero;
    return CT0 * (1.f - ratio) * (1.f - ratio);
}

float PropellerAero::torqueCoeff(float J) const {
    const float kCQ = CT0 / (J_zero * J_zero);
    return CQ0 + kCQ * J * J;
}

// ── Dimensional methods ───────────────────────────────────────────────────────

float PropellerAero::thrust_n(float Omega_rps, float tas_mps, float rho_kgm3) const {
    if (Omega_rps <= 0.f) return 0.f;
    const float n   = Omega_rps / (2.f * kPi);
    const float J   = advanceRatio(Omega_rps, tas_mps);
    const float CT  = thrustCoeff(J);
    const float D4  = diameter_m * diameter_m * diameter_m * diameter_m;
    return CT * rho_kgm3 * n * n * D4;
}

float PropellerAero::torque_nm(float Omega_rps, float tas_mps, float rho_kgm3) const {
    if (Omega_rps <= 0.f) return 0.f;
    const float n   = Omega_rps / (2.f * kPi);
    const float J   = advanceRatio(Omega_rps, tas_mps);
    const float CQ  = torqueCoeff(J);
    const float D5  = diameter_m * diameter_m * diameter_m * diameter_m * diameter_m;
    return CQ * rho_kgm3 * n * n * D5;
}

} // namespace liteaerosim::propulsion
