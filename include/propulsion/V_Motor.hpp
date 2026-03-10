#pragma once

namespace liteaerosim::propulsion {

// Abstract motor interface.  Decouples the power source from propeller aerodynamics.
// PropulsionProp uses V_Motor to determine target shaft speed, speed ceiling, and
// rotor-system inertia.  All motor subclasses are stateless.
class V_Motor {
public:
    virtual ~V_Motor() = default;

    // Free-running (no-load) shaft speed for the given throttle and air density (rad/s).
    // PropulsionProp passes this as the target to the rotor FilterSS2Clip.
    //   throttle_nd — normalized throttle demand [0, 1]
    //   rho_kgm3    — ambient air density (for power-available correction)
    [[nodiscard]] virtual float noLoadOmega_rps(float throttle_nd,
                                                float rho_kgm3) const = 0;

    // Absolute maximum shaft speed (rad/s) — sets the FilterSS2Clip upper limit.
    [[nodiscard]] virtual float maxOmega_rps() const = 0;

    // Combined rotor + engine moment of inertia (kg·m²).
    [[nodiscard]] virtual float inertia_kg_m2() const = 0;
};

} // namespace liteaerosim::propulsion
