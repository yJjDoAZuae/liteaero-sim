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

    // Constructor 2: supply q_nb directly; alpha, beta, q_nw are derived.
    KinematicState(double time_sec,
                   const WGS84_Datum& position_datum,
                   const Eigen::Vector3f& velocity_NED_mps,
                   const Eigen::Vector3f& acceleration_NED_mps,
                   const Eigen::Quaternionf& q_nb,
                   const Eigen::Vector3f& rates_Body_rps);

    ~KinematicState() = default;

    // ── Simulation engine ─────────────────────────────────────────────────────

    void step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha_rad,
              float beta_rad,
              float alphaDot_rps,
              float betaDot_rps,
              const Eigen::Vector3f& wind_NED_mps);

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
    Eigen::Quaternionf q_nv()  const { return Eigen::Quaternionf::Identity(); } // not implemented

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

    liteaero::nav::PlaneOfMotion POM()        const { return liteaero::nav::KinematicStateUtil::plane_of_motion(snapshot_); }
    liteaero::nav::TurnCircle    turnCircle() const { return liteaero::nav::KinematicStateUtil::turn_circle(snapshot_); }

    float crab()     const { return liteaero::nav::KinematicStateUtil::crab_rad(snapshot_); }
    float crabRate() const { return liteaero::nav::KinematicStateUtil::crab_rate_rad_s(snapshot_); }

    static Eigen::Vector3f EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& rates);
    static EulerRates BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& rates);

    // ── Serialization ─────────────────────────────────────────────────────────

    nlohmann::json       serializeJson()                              const;
    void                 deserializeJson(const nlohmann::json& j);
    std::vector<uint8_t> serializeProto()                            const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);

protected:

    liteaero::nav::KinematicStateSnapshot snapshot_;

    static void stepQnw(const Eigen::Vector3f& velocity_prev_NED_mps,
                        const Eigen::Vector3f& velocity_NED_mps,
                        float rollRate_Wind_rps,
                        float dt_s,
                        Eigen::Quaternionf& q_nw);

    static void stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv);
};
