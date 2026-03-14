#define _USE_MATH_DEFINES
#include "aerodynamics/AeroCoeffEstimator.hpp"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::aerodynamics {

// ── Internal reference condition for C_D0 skin-friction Reynolds number ───────
// ISA sea-level, representative cruise speed.  Used consistently for all
// component chord Reynolds numbers so that relative drag contributions are
// correct even though absolute CD0 carries ±10% uncertainty.
static constexpr float kRhoSL   = 1.225f;    // kg/m³
static constexpr float kVRef    = 20.f;       // m/s
static constexpr float kMuSL    = 1.789e-5f;  // Pa·s

// Typical tail-section thickness ratio (Raymer: thin empennage ~0.09).
static constexpr float kTailTc  = 0.09f;

// ── Per-surface derived geometry ─────────────────────────────────────────────

struct SurfaceCache {
    float ar;           // aspect ratio
    float mac_m;        // mean aerodynamic chord (m)
    float y_bar_m;      // spanwise MAC station (m)
    float tan_le;       // tan(leading-edge sweep)
    float tan_qc;       // tan(quarter-chord sweep)
    float tan_hc;       // tan(half-chord sweep)
    float cl_alpha;     // 3D lift-curve slope (rad⁻¹)
    float x_ac_m;       // aerodynamic center body x (m)
};

static SurfaceCache computeSurface(const SurfaceGeometry& s, float mach) {
    const float ar     = s.span_m * s.span_m / s.area_m2;
    const float c_root = 2.f * s.area_m2 / (s.span_m * (1.f + s.taper_ratio_nd));
    const float mac    = (2.f / 3.f) * c_root *
                         (1.f + s.taper_ratio_nd + s.taper_ratio_nd * s.taper_ratio_nd) /
                         (1.f + s.taper_ratio_nd);
    const float y_bar  = (s.span_m / 6.f) *
                         (1.f + 2.f * s.taper_ratio_nd) / (1.f + s.taper_ratio_nd);
    const float tan_le = std::tan(s.le_sweep_rad);
    const float delta  = (1.f - s.taper_ratio_nd) / (1.f + s.taper_ratio_nd);
    const float tan_qc = tan_le - (1.f / ar) * delta;
    const float tan_hc = tan_le - (2.f / ar) * delta;

    // Helmbold / DATCOM §4.1.3.2 — Prandtl-Glauert factor η = 1 for M < 0.3.
    const float eta    = (mach < 0.3f) ? 1.f : std::sqrt(1.f - mach * mach);
    const float disc   = 4.f + ar * ar * (1.f + tan_hc * tan_hc) / (eta * eta);
    const float cl_a   = 2.f * static_cast<float>(M_PI) * ar /
                         (2.f + std::sqrt(disc));

    const float x_mac_le = s.x_le_root_m + y_bar * tan_le;
    const float x_ac     = x_mac_le + mac / 4.f;

    return {ar, mac, y_bar, tan_le, tan_qc, tan_hc, cl_a, x_ac};
}

// ── Skin friction (turbulent, Prandtl–Schlichting) ───────────────────────────

static float skinFriction(float chord_m) {
    const float re      = kRhoSL * kVRef * chord_m / kMuSL;
    const float log10re = std::log10(re);
    return 0.455f / std::pow(log10re, 2.58f);
}

// ── Form factor — lifting surface (Raymer Eq. 12.30) ─────────────────────────
// At M < 0.3 (incompressible) the compressibility term reduces to ~1.
// Use M_eff = max(M, 0.3) to avoid 0^0.18 → 0; the resulting overestimate
// is well within the ±10% CD0 tolerance.

static float formFactorSurface(float tc, float tan_hc, float mach) {
    const float m_eff   = std::max(mach, 0.3f);
    const float hc_ang  = std::atan(tan_hc);
    const float shape   = 1.f + (0.6f / 0.3f) * tc + 100.f * tc * tc * tc * tc;
    const float compress= 1.34f * std::pow(m_eff, 0.18f) *
                          std::pow(std::cos(hc_ang), 0.28f);
    return shape * compress;
}

// ── Form factor — fuselage (Raymer Eq. 12.31) ────────────────────────────────

static float formFactorFuselage(float fineness) {
    return 1.f + 60.f / (fineness * fineness * fineness) + fineness / 400.f;
}

// ── Wetted area — fuselage (Raymer Eq. 12.27) ────────────────────────────────

static float wettedAreaFuselage(float length_m, float diameter_m) {
    const float lam  = length_m / diameter_m;
    const float body = std::pow(1.f - 2.f / lam, 2.f / 3.f);
    const float tail = 1.f + 1.f / (lam * lam);
    return static_cast<float>(M_PI) * diameter_m * length_m * body * tail;
}

// ── Public entry point ────────────────────────────────────────────────────────

std::pair<AeroPerformance, LiftCurveParams>
AeroCoeffEstimator::estimate(const AircraftGeometry& geom) {
    // ── Input validation ─────────────────────────────────────────────────────
    if (geom.wing.area_m2 <= 0.f || geom.wing.span_m <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: wing area and span must be positive");
    if (geom.h_tail.area_m2 <= 0.f || geom.h_tail.span_m <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: h_tail area and span must be positive");
    if (geom.v_tail.area_m2 <= 0.f || geom.v_tail.span_m <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: v_tail area and span must be positive");
    if (geom.fuselage_length_m <= 0.f || geom.fuselage_diameter_m <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: fuselage dimensions must be positive");
    if (geom.section_cl_max_2d_nd <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: section_cl_max_2d_nd must be positive");
    if (geom.tail_efficiency_nd <= 0.f || geom.tail_efficiency_nd > 1.f)
        throw std::invalid_argument("AeroCoeffEstimator: tail_efficiency_nd must be in (0, 1]");
    if (geom.wing.taper_ratio_nd <= 0.f || geom.h_tail.taper_ratio_nd <= 0.f ||
        geom.v_tail.taper_ratio_nd <= 0.f)
        throw std::invalid_argument("AeroCoeffEstimator: taper ratios must be positive");

    // ── Part 1 — derived geometry ────────────────────────────────────────────
    const SurfaceCache wing = computeSurface(geom.wing,   geom.mach_nd);
    const SurfaceCache ht   = computeSurface(geom.h_tail, geom.mach_nd);
    const SurfaceCache vt   = computeSurface(geom.v_tail, geom.mach_nd);

    const float l_HT = ht.x_ac_m - geom.x_cg_m;
    const float l_VT = vt.x_ac_m - geom.x_cg_m;

    const float S    = geom.wing.area_m2;
    const float b    = geom.wing.span_m;
    const float eta  = geom.tail_efficiency_nd;

    // ── Part 2 — 3D lift-curve slope (wing only; from SurfaceCache) ──────────
    // cl_alpha for the wing is wing.cl_alpha.
    // cl_alpha for HT and VT are ht.cl_alpha and vt.cl_alpha respectively.

    // ── Part 3 — lift curve model parameters ─────────────────────────────────
    const float qc_sweep   = std::atan(wing.tan_qc);
    const float cl_max     = geom.section_cl_max_2d_nd * std::cos(qc_sweep);
    const float cl_min     = -cl_max;
    // Empirical stall region half-width (docs/algorithms/aerodynamics.md §3.3)
    const float da_stall   = 0.07f;
    const float cl_sep     =  0.6f * cl_max;
    const float cl_sep_neg =  0.6f * cl_min;

    // ── Part 4 — Oswald efficiency (Hoerner + sweep correction) ──────────────
    const float e0 = 1.f / (1.f + 0.007f * static_cast<float>(M_PI) * wing.ar);
    const float e  = e0 * std::cos(qc_sweep - 0.09f);

    // ── Part 5 — zero-lift drag C_D0 component buildup ───────────────────────
    const float M            = geom.mach_nd;
    const float tc           = geom.thickness_ratio_nd;
    const float lambda_f     = geom.fuselage_length_m / geom.fuselage_diameter_m;

    // Skin friction coefficients
    const float cf_wing = skinFriction(wing.mac_m);
    const float cf_ht   = skinFriction(ht.mac_m);
    const float cf_vt   = skinFriction(vt.mac_m);
    const float cf_fus  = skinFriction(geom.fuselage_length_m);

    // Form factors
    const float ff_wing = formFactorSurface(tc,          wing.tan_hc, M);
    const float ff_ht   = formFactorSurface(kTailTc,     ht.tan_hc,   M);
    const float ff_vt   = formFactorSurface(kTailTc,     vt.tan_hc,   M);
    const float ff_fus  = formFactorFuselage(lambda_f);

    // Wetted areas
    const float swet_wing = 2.003f * geom.wing.area_m2;
    const float swet_ht   = 2.003f * geom.h_tail.area_m2;
    const float swet_vt   = 2.003f * geom.v_tail.area_m2;
    const float swet_fus  = wettedAreaFuselage(geom.fuselage_length_m,
                                               geom.fuselage_diameter_m);

    // Interference factors: wing = 1.0, tails = 1.04, fuselage = 1.0
    const float drag_area = cf_wing * ff_wing * 1.00f * swet_wing
                          + cf_ht   * ff_ht   * 1.04f * swet_ht
                          + cf_vt   * ff_vt   * 1.04f * swet_vt
                          + cf_fus  * ff_fus  * 1.00f * swet_fus;
    const float cd0 = drag_area / S + geom.cd_misc_nd;

    // ── Part 6 — lateral force slope C_Yβ ────────────────────────────────────
    const float cy_beta = -eta * (geom.v_tail.area_m2 / S) * vt.cl_alpha;

    // ── Part 7 — pitch-damping derivative C_Lq ───────────────────────────────
    // Wing contribution — DATCOM §5.2.1.2 (handles nonzero sweep)
    const float cos_qc  = std::cos(qc_sweep);
    const float tan2_qc = wing.tan_qc * wing.tan_qc;
    const float taper_w = geom.wing.taper_ratio_nd;
    const float term1   = (wing.ar + 2.f * cos_qc) /
                          (2.f * (wing.ar + 4.f * cos_qc)) *
                          wing.ar / cos_qc;
    const float term2   = (1.f + 2.f * taper_w) /
                          (3.f * (1.f + taper_w)) * tan2_qc;
    const float clq_wing = wing.cl_alpha * (term1 + term2);

    // Tail contribution
    const float clq_tail = 2.f * eta * ht.cl_alpha *
                           (l_HT * geom.h_tail.area_m2) / (S * wing.mac_m);

    const float cl_q = clq_wing + clq_tail;

    // ── Part 8 — yaw-rate lateral derivative C_Yr ────────────────────────────
    const float cy_r = 2.f * eta * vt.cl_alpha *
                       (l_VT * geom.v_tail.area_m2) / (S * b);

    // ── Assemble outputs ─────────────────────────────────────────────────────
    AeroPerformanceConfig aero_cfg;
    aero_cfg.s_ref_m2  = S;
    aero_cfg.ar        = wing.ar;
    aero_cfg.e         = e;
    aero_cfg.cd0       = cd0;
    aero_cfg.cl_y_beta = cy_beta;
    aero_cfg.cl_q_nd   = cl_q;
    aero_cfg.mac_m     = wing.mac_m;
    aero_cfg.cy_r_nd   = cy_r;
    aero_cfg.fin_arm_m = l_VT;

    LiftCurveParams lcp;
    lcp.cl_alpha              = wing.cl_alpha;
    lcp.cl_max                = cl_max;
    lcp.cl_min                = cl_min;
    lcp.delta_alpha_stall     = da_stall;
    lcp.delta_alpha_stall_neg = da_stall;
    lcp.cl_sep                = cl_sep;
    lcp.cl_sep_neg            = cl_sep_neg;

    return {AeroPerformance(aero_cfg), lcp};
}

} // namespace liteaerosim::aerodynamics
