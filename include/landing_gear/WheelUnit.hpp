#pragma once

#include <landing_gear/StrutState.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

namespace liteaero::simulation {

struct WheelUnitParams {
    Eigen::Vector3f attach_point_body_m             = Eigen::Vector3f::Zero();
    Eigen::Vector3f travel_axis_body                = Eigen::Vector3f(0.f, 0.f, 1.f);
    float           spring_stiffness_npm            = 0.0f;
    float           damping_compression_nspm         = 0.0f;
    float           damping_extension_nspm           = 0.0f;
    float           orifice_damping_compression_ns2pm2 = 0.0f;
    float           orifice_damping_extension_ns2pm2   = 0.0f;
    float           spring_nonlinearity_nd           = 0.0f;
    float           preload_n                       = 0.0f;
    float           travel_max_m                    = 0.3f;
    float           tyre_radius_m                   = 0.2f;
    float           tyre_cornering_stiffness_npm    = 0.0f;
    float           tyre_longitudinal_stiffness_npm = 0.0f;
    float           rolling_resistance_nd           = 0.02f;
    float           max_brake_torque_nm             = 0.0f;
    bool            is_steerable                    = false;
    bool            has_brake                       = false;
    float           spindown_time_s                 = 5.0f;
    float           spindown_reference_speed_mps    = 20.0f;
};

struct WheelContactForces {
    Eigen::Vector3f force_body_n   = Eigen::Vector3f::Zero();
    Eigen::Vector3f moment_body_nm = Eigen::Vector3f::Zero();
    bool            in_contact     = false;
};

class WheelUnit {
public:
    void initialize(const WheelUnitParams& params);
    void reset();

    // Single substep update.
    // penetration_m     — terrain elevation minus contact-point altitude (m); > 0 = in contact
    // contact_point_body_m — contact patch position in body frame (m)
    // contact_vel_body_mps — velocity of contact patch in body frame (m/s)
    // surface_normal_body  — terrain surface normal in body frame (unit vector, pointing away from terrain)
    // wheel_angle_rad   — nose-wheel steering angle (applied only when is_steerable)
    // brake_demand_nd   — normalized brake demand [0, 1]
    // friction_mu_nd    — peak friction coefficient from surface model
    // dt_s              — substep timestep (s)
    WheelContactForces step(float                         penetration_m,
                            const Eigen::Vector3f&        contact_point_body_m,
                            const Eigen::Vector3f&        contact_vel_body_mps,
                            const Eigen::Vector3f&        surface_normal_body,
                            float                         wheel_angle_rad,
                            float                         brake_demand_nd,
                            float                         friction_mu_nd,
                            float                         dt_s);

    StrutState strutState() const;
    void       setStrutState(const StrutState& s);

    // OQ-LG-15 diagnostic — breakdown of the most recent step()'s contact force.
    // TEMPORARY: remove once the deep-penetration forward-force artifact is fixed.
    struct ContactDiag {
        float           F_z          = 0.f;  // normal force magnitude (N)
        float           F_x          = 0.f;  // Pacejka longitudinal force (N, +fwd)
        float           F_y          = 0.f;  // Pacejka lateral force (N)
        float           F_rr         = 0.f;  // rolling resistance (N, signed)
        float           kappa        = 0.f;  // longitudinal slip ratio
        float           alpha_t      = 0.f;  // slip angle (rad)
        float           V_cx         = 0.f;  // contact-patch fwd speed (m/s)
        float           V_cy         = 0.f;  // contact-patch lateral speed (m/s)
        float           omega        = 0.f;  // wheel spin (rad/s)
        float           delta_dot    = 0.f;  // strut closing rate (m/s)
        Eigen::Vector3f wheel_fwd    = Eigen::Vector3f::Zero();
        Eigen::Vector3f surf_normal  = Eigen::Vector3f::Zero();
        Eigen::Vector3f force_body   = Eigen::Vector3f::Zero();
    };
    const ContactDiag& lastContactDiag() const { return _diag; }

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json&          j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    WheelUnitParams _params;
    float           _strut_deflection_m        = 0.0f;
    float           _strut_deflection_rate_mps = 0.0f;
    float           _wheel_speed_rps           = 0.0f;
    float           _cf                        = 0.0f;
    float           _cv                        = 0.0f;
    ContactDiag     _diag;   // OQ-LG-15 diagnostic cache (TEMPORARY)
};

}  // namespace liteaero::simulation
