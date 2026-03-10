#define _USE_MATH_DEFINES
#include "propulsion/MotorPiston.hpp"
#include <cmath>

namespace liteaerosim::propulsion {

static constexpr float kRhoSL_kgm3 = 1.225f;

MotorPiston::MotorPiston(float power_max_w, float peak_omega_rps,
                         float altitude_exponent, float inertia_kg_m2)
    : _power_max_w(power_max_w)
    , _peak_omega_rps(peak_omega_rps)
    , _altitude_exponent(altitude_exponent)
    , _inertia_kg_m2(inertia_kg_m2)
{}

float MotorPiston::noLoadOmega_rps(float throttle_nd, float rho_kgm3) const {
    // Free-running speed where net torque to propeller is zero.
    const float density_ratio = rho_kgm3 / kRhoSL_kgm3;
    return 2.f * _peak_omega_rps * throttle_nd
           * std::pow(density_ratio, _altitude_exponent * 0.5f);
}

float MotorPiston::maxOmega_rps() const {
    return 2.f * _peak_omega_rps;
}

float MotorPiston::inertia_kg_m2() const {
    return _inertia_kg_m2;
}

} // namespace liteaerosim::propulsion
