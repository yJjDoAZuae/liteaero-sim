#include "propulsion/MotorElectric.hpp"
#include <algorithm>

namespace liteaerosim::propulsion {

MotorElectric::MotorElectric(const MotorElectricMotorParams& motor,
                              const MotorElectricEscParams&   esc)
    : _kv_rps_per_v(motor.kv_rps_per_v)
    , _r_terminal_ohm(motor.r_terminal_ohm)
    , _inertia_motor_kg_m2(motor.inertia_motor_kg_m2)
    , _supply_voltage_v(esc.supply_voltage_v)
    , _i_max_a(esc.i_max_a)
    , _esc_efficiency_nd(esc.esc_efficiency_nd)
{}

float MotorElectric::noLoadOmega_rps(float throttle_nd, float /*rho_kgm3*/) const {
    // Electric motor power is density-independent; rho_kgm3 is ignored.
    return _kv_rps_per_v * throttle_nd * _supply_voltage_v;
}

float MotorElectric::maxOmega_rps() const {
    return _kv_rps_per_v * _supply_voltage_v;
}

float MotorElectric::inertia_kg_m2() const {
    return _inertia_motor_kg_m2;
}

float MotorElectric::batteryCurrent_a(float omega_rps, float throttle_nd) const {
    const float V_phase  = throttle_nd * _supply_voltage_v;
    const float back_emf = omega_rps / _kv_rps_per_v;
    const float I_motor  = (V_phase - back_emf) / _r_terminal_ohm;
    const float I_clamped = std::min(I_motor, _i_max_a);
    if (I_clamped <= 0.f) return 0.f;
    return V_phase * I_clamped / (_supply_voltage_v * _esc_efficiency_nd);
}

} // namespace liteaerosim::propulsion
