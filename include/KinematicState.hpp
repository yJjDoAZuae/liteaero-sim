#pragma once

// Kinematic/temporal state and all quantities derived from kinematics

// Jerk and angular accelerations are not included, so trajectory torsion will not be calculated

#include "navigation/WGS84.hpp"
#include <Eigen/Dense>

typedef Eigen::Vector3f EulerAngles;
typedef Eigen::Vector3f EulerRates;

class PlaneOfMotion
{

public:
    Eigen::Quaternionf q_np; // POM to NED rotation
};

class TurnCircle
{

public:
    PlaneOfMotion pom;

    Eigen::Vector3f turnCenter_deltaNED_m;
};

class KinematicState
{

public:
    KinematicState() : _time_sec(0.0),
                       _positionDatum(),
                       _velocity_NED_mps(Eigen::Vector3f::Zero()),
                       _acceleration_NED_mps(Eigen::Vector3f::Zero()),
                       _q_nw(Eigen::Quaternionf::Identity()),
                       _q_nb(Eigen::Quaternionf::Identity()),
                       _rates_Body_rps(Eigen::Vector3f::Zero()),
                       _wind_NED_mps(Eigen::Vector3f::Zero()),
                       _alpha_rad(0.f),
                       _beta_rad(0.f),
                       _alphaDot_rps(0.f),
                       _betaDot_rps(0.f),
                       _rollRate_Wind_rps(0.f) {};

    KinematicState(double time_sec,
                   const WGS84_Datum &position_datum,
                   const Eigen::Vector3f &velocity_NED_mps,
                   const Eigen::Vector3f &acceleration_Wind_mps,
                   const Eigen::Quaternionf &q_nw,
                   float rollRate_Wind_rps,
                   float alpha,
                   float beta,
                   float alphaDot,
                   float betaDot,
                   float windSpeed_mps,
                   float windDirFrom_rad);

    // Constructor 2: accepts q_nb directly (e.g. from an Euler-angle specification).
    // _q_nw is derived exactly from velocity_NED_mps and q_nb.  With
    // q_wb = Ry(α)·Rz(−β) the body-frame airmass velocity satisfies
    //   u = V·cosα·cosβ,  v = V·cosα·sinβ,  w = V·sinα
    // which inverts to α = atan2(w, √(u²+v²)), β = atan2(v, u).
    // Then _q_nw = q_nb · Rz(β) · Ry(−α).  Below the small-V threshold α=β=0.
    KinematicState(double time_sec,
                   const WGS84_Datum &position_datum,
                   const Eigen::Vector3f &velocity_NED_mps,
                   const Eigen::Vector3f &acceleration_NED_mps,
                   const Eigen::Quaternionf &q_nb,
                   const Eigen::Vector3f &rates_Body_rps);

    ~KinematicState() {};

    // state update
    void step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha_rad,
              float beta_rad,
              float alphaDot_rps,
              float betaDot_rps,
              float windSpeed_mps,
              float windDirFrom_rad);

    // getters
    double time_sec() const { return _time_sec; }
    WGS84_Datum positionDatum() const { return _positionDatum; }
    Eigen::Vector3f velocity_NED_mps() const { return _velocity_NED_mps; }
    Eigen::Vector3f acceleration_NED_mps() const { return _acceleration_NED_mps; }
    Eigen::Quaternionf q_nb() const { return _q_nb; }   // Body to NED rotation
    Eigen::Quaternionf q_nw() const { return _q_nw; }   // Wind to NED rotation
    Eigen::Vector3f rates_Body_rps() const { return _rates_Body_rps; }

    // derived quantity methods
    double latitudeRate_rps() const;
    double longitudeRate_rps() const;
    Eigen::Vector3f velocity_Wind_mps() const;
    Eigen::Vector3f velocity_Stab_mps() const;
    Eigen::Vector3f velocity_Body_mps() const;
    Eigen::Vector3f acceleration_Body_mps() const;
    Eigen::Vector3f acceleration_Wind_mps() const;
    Eigen::Vector3f eulers() const;
    float roll() const;
    float pitch() const;
    float heading() const;
    float rollRate_rps() const; // roll Euler time derivative
    float pitchRate_rps() const; // pitch Euler time derivative
    float headingRate_rps() const; // heading Euler time derivative

    const PlaneOfMotion &POM() const;
    const TurnCircle &turnCircle() const;

    Eigen::Quaternionf q_nl() const; // Local Level to NED rotation
    Eigen::Quaternionf q_ns() const; // Stability to NED rotation
    Eigen::Quaternionf q_nv() const; // Velocity to NED rotation
    float alpha() const;
    float beta() const;
    float rollRate_Wind_rps() const; // roll rate of the Wind frame w.r.t. NED
    float alphaDot() const;
    float betaDot() const;
    float crab() const;
    float crabRate() const;

    static Eigen::Vector3f EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& rates);
    static EulerRates BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& rates);

protected:

    // time
    double _time_sec;

    // position
    WGS84_Datum _positionDatum;

    // velocity
    Eigen::Vector3f _velocity_NED_mps;

    // acceleration in NED frame
    Eigen::Vector3f _acceleration_NED_mps;

    // Wind-to-NED rotation (stored state)
    Eigen::Quaternionf _q_nw;

    // Body to NED rotation
    Eigen::Quaternionf _q_nb;

    // Body rates
    Eigen::Vector3f _rates_Body_rps;

    // Wind velocity in NED frame
    Eigen::Vector3f _wind_NED_mps;

    // Aerodynamic angles and rates — stored from step() / constructor parameters
    float _alpha_rad;
    float _beta_rad;
    float _alphaDot_rps;
    float _betaDot_rps;
    float _rollRate_Wind_rps;

    // Cached derived quantities (computed on demand by const methods)
    mutable PlaneOfMotion _pom;
    mutable TurnCircle    _turn_circle;

    static void stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv );

};
