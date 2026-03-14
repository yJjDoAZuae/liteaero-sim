#pragma once

namespace liteaerosim::aerodynamics {

// Dimensional description of a single lifting surface (wing, horizontal tail, or
// vertical tail). All quantities are in SI units.
struct SurfaceGeometry {
    float span_m;           // tip-to-tip span (or root-to-tip for vertical tail) [m]
    float area_m2;          // reference planform area [m²]
    float le_sweep_rad;     // leading-edge sweep angle [rad]
    float taper_ratio_nd;   // chord taper ratio c_tip / c_root [—]
    float x_le_root_m;      // body x-coordinate of the root leading edge [m]
};

// Full aircraft geometry description required to estimate all trim-aero-model
// coefficients via AeroCoeffEstimator. All quantities are in SI units.
// See docs/algorithms/aerodynamics.md for the derivation chain.
struct AircraftGeometry {
    SurfaceGeometry wing;
    SurfaceGeometry h_tail;
    SurfaceGeometry v_tail;
    float fuselage_length_m;            // total fuselage length [m]
    float fuselage_diameter_m;          // maximum fuselage diameter [m]
    float thickness_ratio_nd;           // wing section thickness-to-chord ratio t/c [—]
    float section_cl_alpha_rad;         // 2D section lift slope (≈ 2π rad⁻¹) [rad⁻¹]
    float section_cl_max_2d_nd;         // 2D section maximum lift coefficient [—]
    float x_cg_m;                       // body x-coordinate of center of gravity [m]
    float mach_nd          = 0.f;       // design Mach number (0 → incompressible) [—]
    float tail_efficiency_nd = 0.9f;    // dynamic pressure ratio η for both tails [—]
    float cd_misc_nd       = 0.003f;    // miscellaneous drag increment [—]
};

} // namespace liteaerosim::aerodynamics
