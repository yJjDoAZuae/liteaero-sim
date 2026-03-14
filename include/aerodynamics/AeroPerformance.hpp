#pragma once

#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

namespace liteaerosim::aerodynamics {

// Wind-frame aerodynamic forces (SI units: Newtons).
// Sign convention (NED-wind frame, X forward, Y right, Z down):
//   x_n — drag force: negative (opposes motion)
//   y_n — side force: positive right
//   z_n — lift force: negative (lift acts upward, opposing positive Z)
struct AeroForces {
    float x_n = 0.f;
    float y_n = 0.f;
    float z_n = 0.f;
};

// Configuration parameters for AeroPerformance. All fields are in SI units.
// The four rotational-coupling fields (cl_q_nd, mac_m, cy_r_nd, fin_arm_m) default
// to zero and are optional for models that do not use rotational turbulence coupling.
struct AeroPerformanceConfig {
    float s_ref_m2   = 0.f;  // reference wing area (m²)
    float ar         = 0.f;  // aspect ratio b²/S
    float e          = 0.f;  // Oswald efficiency factor (0 < e ≤ 1)
    float cd0        = 0.f;  // zero-lift drag coefficient
    float cl_y_beta  = 0.f;  // lateral force slope C_Yβ (rad⁻¹, typically < 0)
    float cl_q_nd    = 0.f;  // pitch-rate damping derivative C_Lq (rad⁻¹)
    float mac_m      = 0.f;  // mean aerodynamic chord (m)
    float cy_r_nd    = 0.f;  // yaw-rate lateral force derivative C_Yr (rad⁻¹)
    float fin_arm_m  = 0.f;  // vertical tail moment arm l_VT (m)
};

// Stateless aerodynamic force model for the simple drag-polar + lateral-force model.
//
// Normal force equation (z-axis, wind frame):
//   Fz = -q · S · CL          (lift upward = negative wind Z)
//
// Drag polar (x-axis, wind frame):
//   CDi = k · CL²             (k = 1 / (π · e · AR))
//   CD  = CD0 + CDi
//   Fx  = -q · S · CD         (drag opposes motion = negative wind X)
//
// Lateral force (y-axis, wind frame):
//   CY  = C_Yβ · β
//   Fy  =  q · S · CY
//
// Construct once per aircraft configuration.
class AeroPerformance {
public:
    explicit AeroPerformance(const AeroPerformanceConfig& cfg);

    // Compute aerodynamic forces in the Wind frame.
    //   alpha_rad — angle of attack (rad), from LoadFactorAllocator::solve()
    //   beta_rad  — sideslip angle (rad), from LoadFactorAllocator::solve()
    //   q_inf_pa  — dynamic pressure (Pa)
    //   cl        — lift coefficient from LiftCurveModel::evaluate(alpha_rad)
    AeroForces compute(float alpha_rad, float beta_rad, float q_inf_pa, float cl) const;

    // Accessors
    float ar()          const { return _ar; }
    float e()           const { return _e; }
    float cd0()         const { return _cd0; }
    float inducedDragK()const { return _k; }
    float clYBeta()     const { return _cl_y_beta; }
    float clQNd()       const { return _cl_q_nd; }
    float macM()        const { return _mac_m; }
    float cyRNd()       const { return _cy_r_nd; }
    float finArmM()     const { return _fin_arm_m; }

    // Serialization (stateless — serializes configuration parameters)
    nlohmann::json        serializeJson()                               const;
    static AeroPerformance deserializeJson(const nlohmann::json&        j);

    std::vector<uint8_t>  serializeProto()                             const;
    static AeroPerformance deserializeProto(const std::vector<uint8_t>& bytes);

private:
    float _S;           // reference wing area (m²)
    float _ar;          // aspect ratio
    float _e;           // Oswald efficiency
    float _cd0;         // zero-lift drag coefficient
    float _k;           // induced drag constant = 1 / (π · e · AR)
    float _cl_y_beta;   // lateral force slope (rad⁻¹)
    float _cl_q_nd;     // pitch-rate damping derivative C_Lq (rad⁻¹)
    float _mac_m;       // mean aerodynamic chord (m)
    float _cy_r_nd;     // yaw-rate lateral force derivative C_Yr (rad⁻¹)
    float _fin_arm_m;   // vertical tail moment arm (m)
};

} // namespace liteaerosim::aerodynamics
