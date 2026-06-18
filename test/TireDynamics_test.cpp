// Tire-dynamics isolation tests.
//
// Physical invariant under test: a free-rolling (unbraked, undriven) tire can only
// produce rolling resistance and slip forces that OPPOSE the contact-patch motion.
// It must NEVER produce a net longitudinal traction that propels the vehicle. These
// tests prescribe the contact-patch motion directly (WheelUnit::step takes the contact
// velocity as an input), so the tire model is isolated from strut, attitude, aero, and
// collider dynamics.
//
// Convention: body z-down. Level terrain normal (pointing away from terrain = up) is
// (0,0,-1) in body frame, so wheel_fwd resolves to +x and the longitudinal contact
// force is force_body_n.x() = F_x (Pacejka) + F_rr (rolling resistance).

#include "landing_gear/WheelUnit.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iomanip>

using liteaero::simulation::WheelUnit;
using liteaero::simulation::WheelUnitParams;
using liteaero::simulation::WheelContactForces;

namespace {

WheelUnitParams makeFreeWheel() {
    WheelUnitParams p;
    p.attach_point_body_m            = {0.0f, 0.0f, 0.5f};
    p.travel_axis_body               = {0.0f, 0.0f, 1.0f};
    p.spring_stiffness_npm           = 20000.0f;
    p.damping_compression_nspm       = 0.0f;   // isolate the tire: no strut damping
    p.damping_extension_nspm         = 0.0f;
    p.spring_nonlinearity_nd         = 0.0f;
    p.preload_n                      = 0.0f;
    p.travel_max_m                   = 0.30f;
    p.tyre_radius_m                  = 0.2f;
    p.rolling_resistance_nd          = 0.02f;
    p.max_brake_torque_nm            = 0.0f;
    p.is_steerable                   = false;
    p.has_brake                      = false;  // free-rolling
    return p;
}

constexpr float kMu  = 0.8f;
constexpr float kDt  = 0.02f;
const Eigen::Vector3f kNormalUp{0.0f, 0.0f, -1.0f};       // level terrain normal (body, z-down)
const Eigen::Vector3f kContactPt{0.0f, 0.0f, 0.7f};       // contact patch below CG

}  // namespace

// ---------------------------------------------------------------------------
// Scenario A — single tire, prescribed contact motion. The defining test.
// A free wheel's longitudinal force must be rolling resistance only (|F_long| <=
// rr * F_z), and it must oppose the contact slip (F_long * V_cx <= 0). This must
// hold for BOTH signs of V_cx — a contact patch moving aft (V_cx < 0, as happens
// transiently under any pitch rotation of a lever-arm wheel) must not make the tire
// grip and push forward.
// ---------------------------------------------------------------------------
TEST(TireDynamics, FreeRoll_LongitudinalForceIsRollingResistanceOnly) {
    const float penetration = 0.10f;          // F_z = k*delta = 2000 N (no damping)
    const float F_z_expect  = 20000.0f * penetration;
    const float rr_bound    = 0.02f * F_z_expect * 1.25f;   // rolling-resistance envelope

    for (float V_cx : {-6.0f, -2.0f, -0.5f, -0.1f, 0.1f, 0.5f, 2.0f, 6.0f}) {
        WheelUnit w;
        w.initialize(makeFreeWheel());
        // Step a few times so the free-roll clamp/quasi-static settle.
        WheelContactForces cf;
        const Eigen::Vector3f contact_vel{V_cx, 0.0f, 0.0f};   // pure forward roll, no settling
        for (int i = 0; i < 5; ++i)
            cf = w.step(penetration, kContactPt, contact_vel, kNormalUp,
                        0.0f /*steer*/, 0.0f /*brake*/, kMu, kDt);

        const float F_long = cf.force_body_n.x();   // = F_x + F_rr (level geometry)

        EXPECT_LE(std::abs(F_long), rr_bound)
            << "free wheel produced longitudinal traction beyond rolling resistance at V_cx="
            << V_cx << ": F_long=" << F_long << " N (rr bound " << rr_bound << " N)";
        EXPECT_LE(F_long * V_cx, 1e-3f)
            << "free-wheel longitudinal force must oppose contact slip at V_cx="
            << V_cx << ": F_long=" << F_long << " N";
    }
}

// ---------------------------------------------------------------------------
// Scenario A2 — prescribed forward motion WITH vertical settling (descent).
// The vertical closing motion must not create longitudinal propulsion.
// ---------------------------------------------------------------------------
TEST(TireDynamics, FreeRoll_SettlingDoesNotPropel) {
    for (float V_cx : {0.5f, 3.0f, 8.0f}) {
        for (float V_z : {0.1f, 0.5f, 1.0f}) {   // body z-down: +V_z = descending
            WheelUnit w;
            w.initialize(makeFreeWheel());
            WheelContactForces cf;
            const Eigen::Vector3f contact_vel{V_cx, 0.0f, V_z};
            for (int i = 0; i < 5; ++i)
                cf = w.step(0.10f, kContactPt, contact_vel, kNormalUp,
                            0.0f, 0.0f, kMu, kDt);
            const float F_z = cf.force_body_n.dot(kNormalUp);   // upward reaction magnitude (>0)
            const float F_long = cf.force_body_n.x();
            // Forward motion (V_cx>0): longitudinal force must be retarding (<= 0).
            EXPECT_LE(F_long, 0.02f * F_z * 1.25f)
                << "tire propelled forward during settling at V_cx=" << V_cx
                << " V_z=" << V_z << ": F_long=" << F_long << " N";
        }
    }
}

// ---------------------------------------------------------------------------
// Scenario B — single tire under a sprung mass at constant forward speed, time-marched.
// A 1-DOF vertical mass sits on the strut; the contact rolls forward at constant speed.
// The longitudinal contact impulse over the run must be retarding (negative): the tire
// removes forward momentum, never adds it.
// ---------------------------------------------------------------------------
TEST(TireDynamics, SprungMass_ConstantForwardSpeed_NetRetarding) {
    WheelUnit w;
    w.initialize(makeFreeWheel());

    const float mass   = 300.0f;     // kg on the strut
    const float g      = 9.80665f;
    const float V_fwd  = 5.0f;       // constant prescribed forward speed
    float       z      = 0.0f;       // strut-top vertical position (penetration proxy)
    float       z_dot  = 0.0f;
    // Start near static deflection so it settles quickly: delta0 = mg/k
    z = mass * g / 20000.0f;

    double impulse_long = 0.0;
    for (int i = 0; i < 2000; ++i) {   // 40 s
        const float penetration = std::max(0.0f, z);
        const Eigen::Vector3f contact_vel{V_fwd, 0.0f, z_dot};
        const auto cf = w.step(penetration, kContactPt, contact_vel, kNormalUp,
                               0.0f, 0.0f, kMu, kDt);
        const float F_z_up = cf.force_body_n.dot(kNormalUp);    // upward reaction magnitude (>0)
        impulse_long += cf.force_body_n.x() * kDt;
        // 1-DOF vertical mass: m z_ddot = m g - F_z_up  (z = penetration, +down)
        const float z_ddot = g - F_z_up / mass;
        z_dot += z_ddot * kDt;
        z     += z_dot * kDt;
    }
    EXPECT_LT(impulse_long, 0.0)
        << "net longitudinal contact impulse must be retarding (tire cannot propel a "
           "constant-speed sprung mass); impulse=" << impulse_long << " N*s";
}

// ---------------------------------------------------------------------------
// Data emitter — free-roll longitudinal force vs contact speed, for documentation.
// Sweeps V_cx across both signs at a fixed normal load and writes a CSV. A correct
// free wheel shows F_x (Pacejka longitudinal) ≈ 0 everywhere and F_long = pure rolling
// resistance (a step that flips sign with V_cx). No assertion — data only.
// ---------------------------------------------------------------------------
TEST(TireDynamics, Emit_FreeRollSweepCsv) {
    const float penetration = 0.10f;
    std::ofstream csv("tire_freeroll_sweep.csv");
    csv << "v_cx_mps,F_x_pacejka_n,F_rr_n,F_long_n,F_z_n\n";
    csv << std::fixed << std::setprecision(5);
    for (int i = -160; i <= 160; ++i) {
        const float V_cx = i * 0.05f;
        WheelUnit w;
        w.initialize(makeFreeWheel());
        WheelContactForces cf;
        const Eigen::Vector3f contact_vel{V_cx, 0.0f, 0.0f};
        for (int k = 0; k < 5; ++k)
            cf = w.step(penetration, kContactPt, contact_vel, kNormalUp,
                        0.0f, 0.0f, kMu, kDt);
        const auto& d = w.lastContactDiag();
        const float F_z = cf.force_body_n.dot(kNormalUp);
        csv << V_cx << ',' << d.F_x << ',' << d.F_rr << ','
            << cf.force_body_n.x() << ',' << F_z << '\n';
    }
    csv.close();
    SUCCEED();
}

// ---------------------------------------------------------------------------
// Touchdown-impact isolation — gear model + parameterization sanity.
// A single wheel carrying its static share of the FullStop fixture mass (1045/3 kg)
// is dropped onto the runway at realistic sink rates with the FIXTURE strut params
// (k=20000 N/m, b_compression=2600). 1-DOF vertical, level, no attitude rotation,
// no aero. Reports the peak normal load in g. A reasonable approach (sink <= 3 m/s)
// must NOT produce a >5 g shock — if it does, the gear params/model are wrong.
// ---------------------------------------------------------------------------
TEST(TireDynamics, Touchdown_PeakG_ReasonableForApproachSinkRates) {
    auto fixtureWheel = []{
        WheelUnitParams p = makeFreeWheel();
        p.damping_compression_nspm = 2600.0f;   // FullStop fixture values
        p.damping_extension_nspm   = 520.0f;
        return p;
    };
    const float g    = 9.80665f;
    const float mass = 1045.0f / 3.0f;          // static share per wheel
    const float contact_alt = 0.7f;             // attach 0.5 + tyre 0.2

    for (float sink : {1.0f, 2.0f, 3.0f}) {
        WheelUnit w; w.initialize(fixtureWheel());
        float z      = contact_alt + 0.001f;    // just above the runway
        float z_dot  = -sink;                    // descending (altitude decreasing)
        float peakFz = 0.0f;
        for (int i = 0; i < 4000; ++i) {         // 80 s
            const float penetration = std::max(0.0f, contact_alt - z);
            // contact-patch velocity: forward 0, vertical = -z_dot (body z-down, descending=+)
            const Eigen::Vector3f contact_vel{0.0f, 0.0f, -z_dot};
            const auto cf = w.step(penetration, kContactPt, contact_vel, kNormalUp,
                                   0.0f, 0.0f, kMu, kDt);
            const float F_z_up = cf.force_body_n.dot(kNormalUp);   // upward (>0)
            if (F_z_up > peakFz) peakFz = F_z_up;
            const float z_ddot = -g + F_z_up / mass;   // z = altitude (up+); gravity pulls down
            z_dot += z_ddot * kDt;
            z     += z_dot * kDt;
        }
        const float peak_g = peakFz / (mass * g);
        std::printf("[touchdown] sink=%.1f m/s -> peak normal load = %.0f N = %.2f g\n",
                    sink, peakFz, peak_g);
        EXPECT_LT(peak_g, 5.0f)
            << "gear produced a " << peak_g << " g shock for a " << sink
            << " m/s touchdown — non-physical for a reasonable approach";
    }
}
