#pragma once

// KinematicState — simulation integrator and state accessor.
//
// Holds a liteaero::nav::KinematicStateSnapshot as its primary data member.
// Derived-quantity methods are one-line forwarders to liteaero::nav::KinematicStateUtil.
// The simulation engine (constructors and step()) lives here; all other logic is in
// liteaero-flight.

#include "navigation/WGS84.hpp"
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

// Convenience type aliases preserved for existing sim call sites.
using EulerAngles = Eigen::Vector3f;
using EulerRates  = Eigen::Vector3f;

class KinematicState
{

public:

    // ── Construction ─────────────────────────────────────────────────────────

    KinematicState() = default;

    // Constructor 1: supply q_nw, alpha, beta, and wind explicitly.
    KinematicState(double time_sec,
                   const WGS84_Datum& position_datum,
                   const Eigen::Vector3f& velocity_NED_mps,
                   const Eigen::Vector3f& acceleration_Wind_mps,
                   const Eigen::Quaternionf& q_nw,
                   float rollRate_Wind_rps,
                   float alpha_rad,
                   float beta_rad,
                   float alphaDot_rps,
                   float betaDot_rps,
                   const Eigen::Vector3f& wind_NED_mps);

    // Constructor 2: supply q_nb directly; alpha, beta, q_nw are derived. OQ-AC-4: q_nw (the wind
    // frame) and beta are derived from the AIRMASS velocity v_a = velocity_NED - wind, so q_nw aligns
    // with the relative wind (a crabbed q_nb over a steady crosswind yields beta = 0 with q_nw at the
    // airspeed azimuth — the crab is the azimuth offset to the ground track). wind defaults to zero.
    KinematicState(double time_sec,
                   const WGS84_Datum& position_datum,
                   const Eigen::Vector3f& velocity_NED_mps,
                   const Eigen::Vector3f& acceleration_NED_mps,
                   const Eigen::Quaternionf& q_nb,
                   const Eigen::Vector3f& rates_Body_rps,
                   const Eigen::Vector3f& wind_NED_mps = Eigen::Vector3f::Zero());

    ~KinematicState() = default;

    // ── Simulation engine ─────────────────────────────────────────────────────

    // Convenience wrapper: calls stepPV then commitAttitude immediately.
    // Use for callers that have no post-integration velocity modifications.
    void step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha_rad,
              float beta_rad,
              float alphaDot_rps,
              float betaDot_rps,
              const Eigen::Vector3f& wind_NED_mps);

    // Phase A of the split step: integrate position and velocity via RK4.
    // Stores v_prev internally; does NOT update q_nw or body rates.
    // Must be followed by commitAttitude() — with all velocity modifications
    // (terrain constraints, gear spring forces, etc.) applied between the two.
    void stepPV(double time_sec,
                Eigen::Vector3f acceleration_Wind_mps,
                float alpha_rad,
                float beta_rad,
                float alphaDot_rps,
                float betaDot_rps,
                const Eigen::Vector3f& wind_NED_mps);

    // Phase B of the split step: update q_nw and body rates using the stored
    // v_prev and the current (fully modified) velocity as v_new.
    // dt_s must equal the same timestep passed to stepPV.
    void commitAttitude(float rollRate_Wind_rps, float dt_s);

    // Overload for the load-factor Aircraft's low-speed attitude handling (OQ-LG-21).
    // Propagates q_nw toward an externally-supplied attitude-reference velocity
    // (a dynamic-pressure blend of the instantaneous and a low-pass-filtered velocity)
    // instead of the raw velocity, and derives the body angular rate from the resulting
    // committed attitude (consistent — near zero for a quiescent vehicle). The stored
    // velocity is unchanged; only the attitude/rate use the reference.
    // OQ-AC-2 Alt 4: max_curvature_per_m (= 1/R_min) speed-proportionally saturates the per-step
    // direction change of the attitude-reference velocity to theta_max = (V·max_curvature_per_m)·dt.
    // The saturated reference is fed forward and tracked strictly by q_nw, so the attitude rate is
    // bounded as V → 0 without ever breaking q_nw.x = v_hat. Pass ≤ 0 to disable the saturation.
    void commitAttitude(float rollRate_Wind_rps, float dt_s,
                        const Eigen::Vector3f& attitude_ref_velocity_ned_mps,
                        float max_curvature_per_m);

    // ── Snapshot access ───────────────────────────────────────────────────────

    const liteaero::nav::KinematicStateSnapshot& snapshot() const { return snapshot_; }

    // ── Direct field accessors (mirror existing API) ──────────────────────────

    double          time_sec()          const { return snapshot_.time_s; }
    WGS84_Datum     positionDatum()     const { return WGS84_Datum(snapshot_.position); }
    Eigen::Vector3f velocity_NED_mps()  const { return snapshot_.velocity_ned_mps; }
    Eigen::Vector3f acceleration_NED_mps() const { return snapshot_.acceleration_ned_mps2; }
    Eigen::Vector3f rates_Body_rps()    const { return snapshot_.rates_body_rps; }
    Eigen::Quaternionf q_nw()           const { return snapshot_.q_nw; }

    // ── Derived quantity forwarders ────────────────────────────────────────────

    Eigen::Quaternionf q_nb()  const { return liteaero::nav::KinematicStateUtil::q_nb(snapshot_); }
    Eigen::Quaternionf q_ns()  const { return liteaero::nav::KinematicStateUtil::q_ns(snapshot_); }
    Eigen::Quaternionf q_nl()  const { return liteaero::nav::KinematicStateUtil::q_nl(snapshot_); }
    Eigen::Quaternionf q_nv() const {
        Eigen::Quaternionf result = Eigen::Quaternionf::Identity();
        stepQnv(snapshot_.velocity_ned_mps, result);
        return result;
    }

    float alpha()              const { return snapshot_.alpha_rad; }
    float beta()               const { return snapshot_.beta_rad; }
    float alphaDot()           const { return snapshot_.alpha_dot_rad_s; }
    float betaDot()            const { return snapshot_.beta_dot_rad_s; }
    float rollRate_Wind_rps()  const { return snapshot_.roll_rate_wind_rad_s; }

    double latitudeRate_rps()  const { return liteaero::nav::WGS84::latitudeRate_rad_s(snapshot_.position, snapshot_.velocity_ned_mps(0)); }
    double longitudeRate_rps() const { return liteaero::nav::WGS84::longitudeRate_rad_s(snapshot_.position, snapshot_.velocity_ned_mps(1)); }

    Eigen::Vector3f velocity_Wind_mps()    const { return liteaero::nav::KinematicStateUtil::velocity_wind_mps(snapshot_); }
    Eigen::Vector3f velocity_Stab_mps()   const { return liteaero::nav::KinematicStateUtil::velocity_stab_mps(snapshot_); }
    Eigen::Vector3f velocity_Body_mps()   const { return liteaero::nav::KinematicStateUtil::velocity_body_mps(snapshot_); }
    Eigen::Vector3f acceleration_Wind_mps() const { return liteaero::nav::KinematicStateUtil::acceleration_wind_mps2(snapshot_); }
    Eigen::Vector3f acceleration_Body_mps() const { return liteaero::nav::KinematicStateUtil::acceleration_body_mps2(snapshot_); }

    Eigen::Vector3f eulers()       const;
    float roll()                   const { return liteaero::nav::KinematicStateUtil::roll_rad(snapshot_); }
    float pitch()                  const { return liteaero::nav::KinematicStateUtil::pitch_rad(snapshot_); }
    float heading()                const { return liteaero::nav::KinematicStateUtil::heading_rad(snapshot_); }
    float rollRate_rps()           const { return liteaero::nav::KinematicStateUtil::euler_rates_rad_s(snapshot_)(0); }
    float pitchRate_rps()          const { return liteaero::nav::KinematicStateUtil::euler_rates_rad_s(snapshot_)(1); }
    float headingRate_rps()        const { return liteaero::nav::KinematicStateUtil::euler_rates_rad_s(snapshot_)(2); }
    // OQ-AC-4 wind-triangle azimuths: air-track (from q_nw), ground-track (from v_g), and the crab.
    float crab_rad()               const { return liteaero::nav::KinematicStateUtil::crab_rad(snapshot_); }
    float airTrackAzimuth_rad()    const { return liteaero::nav::KinematicStateUtil::air_track_azimuth_rad(snapshot_); }
    float groundTrackAzimuth_rad() const { return liteaero::nav::KinematicStateUtil::ground_track_azimuth_rad(snapshot_); }

    liteaero::nav::PlaneOfMotion POM()        const { return liteaero::nav::KinematicStateUtil::plane_of_motion(snapshot_); }
    liteaero::nav::TurnCircle    turnCircle() const { return liteaero::nav::KinematicStateUtil::turn_circle(snapshot_); }

    float crab()     const { return liteaero::nav::KinematicStateUtil::crab_rad(snapshot_); }
    float crabRate() const { return liteaero::nav::KinematicStateUtil::crab_rate_rad_s(snapshot_); }

    static Eigen::Vector3f EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& rates);
    static EulerRates BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& rates);

    // ── Post-integration terrain hard constraint ──────────────────────────────

    // Shift altitude up by penetration_m and apply a restitution-consistent
    // correction to the NED-downward velocity component.  Called by
    // Aircraft::step() after integration whenever BodyCollider::minCornerClearance_m()
    // reports penetration.  The downward approach velocity v_D > 0 is mapped to
    // -restitution_nd * v_D, i.e. (1 + restitution_nd) * v_D is removed: at
    // restitution_nd = 0 (default) the velocity is fully arrested (inelastic);
    // for 0 < restitution_nd < 1 the body rebounds at that fraction of the
    // approach speed.  Upward velocity (negative NED-z) is preserved — the
    // aircraft is already leaving terrain and needs no correction.
    void applyTerrainHardConstraint(float penetration_m, float restitution_nd = 0.f);

    // ── Serialization ─────────────────────────────────────────────────────────

    nlohmann::json       serializeJson()                              const;
    void                 deserializeJson(const nlohmann::json& j);
    std::vector<uint8_t> serializeProto()                            const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);

protected:

    liteaero::nav::KinematicStateSnapshot snapshot_;
    Eigen::Vector3f velocity_prev_ned_mps_{}; // set by stepPV, consumed by commitAttitude
    // OQ-LG-21 attitude-reference-velocity overload state (transient; reinitializes on the
    // first call after construction/deserialize — one-step rate transient only).
    Eigen::Vector3f    att_ref_prev_ned_mps_{};                 // previous attitude-reference velocity
    Eigen::Quaternionf q_nb_prev_{Eigen::Quaternionf::Identity()}; // previous committed q_nb (for rate)
    bool               att_ref_init_ = false;

    // q_nw tracks the (reference) velocity strictly (q_nw.x = v_hat). The OQ-AC-2 speed-proportional
    // rate limit is applied upstream, by saturating the reference-velocity slew in commitAttitude.
    static void stepQnw(const Eigen::Vector3f& velocity_prev_NED_mps,
                        const Eigen::Vector3f& velocity_NED_mps,
                        float rollRate_Wind_rps,
                        float dt_s,
                        Eigen::Quaternionf& q_nw);

    static void stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv);
};
