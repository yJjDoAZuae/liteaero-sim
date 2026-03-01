
#include "KinematicState.hpp"
#include "math/math_util.hpp"

KinematicState::KinematicState(double time_sec,
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
               float windDirFrom_rad)
    : _time_sec(time_sec),
      _positionDatum(position_datum),
      _velocity_NED_mps(velocity_NED_mps),
      _acceleration_NED_mps(q_nw.toRotationMatrix() * acceleration_Wind_mps),
      _q_nw(q_nw),
      _q_nb(Eigen::Quaternionf::Identity()),
      _rates_Body_rps(Eigen::Vector3f::Zero()),
      _wind_NED_mps(-windSpeed_mps * Eigen::Vector3f(std::cos(windDirFrom_rad),
                                                      std::sin(windDirFrom_rad), 0.f))
{
    const float smallV(0.1);

    Eigen::Quaternionf local_q_nv = this->q_nv();
    stepQnv(_velocity_NED_mps, local_q_nv);

    // alpha, beta -> q_wb
    Eigen::Quaternionf q_wb(Eigen::AngleAxisf(alpha, Eigen::Vector3f(0, 1, 0)) * Eigen::AngleAxisf(-beta, Eigen::Vector3f(0, 0, 1)));

    _q_nb = _q_nw * q_wb;

    // alphaDot, betaDot -> omega_bw_b
    // Uses the Euler rate to body rate matrix
    Eigen::Vector3f omega_bw_b; // rotation rate of Body w.r.t. Wind expressed in the Body frame
    omega_bw_b << std::sin(alpha)*betaDot, alphaDot, -std::cos(alpha)*betaDot;

    Eigen::Quaternionf local_q_nvaero;
    local_q_nvaero.setIdentity();

    Eigen::Vector3f velocityWind_NED_mps(_velocity_NED_mps - _wind_NED_mps);

    stepQnv(velocityWind_NED_mps, local_q_nvaero);

    // accel, vel, -> omega_wn_n
    // this is path curvature induced rotation in the POM
    Eigen::Vector3f omega_wn_n(Eigen::Vector3f::Zero()); // rotation rate of Wind w.r.t. NED expressed in the NED frame

    float normV = _velocity_NED_mps.norm();
    if (normV > smallV)
    {
        Eigen::Vector3f kappa(_velocity_NED_mps.cross(_acceleration_NED_mps) / (normV * normV * normV));
        omega_wn_n = kappa * normV;
    }

    // sum angular rate contributions
    _rates_Body_rps = omega_bw_b + q_wb.toRotationMatrix().transpose() * (_q_nw.toRotationMatrix().transpose() * omega_wn_n);
}

double KinematicState::latitudeRate_rps() const
{
    return _positionDatum.latitudeRate(_velocity_NED_mps(0));
}

double KinematicState::longitudeRate_rps() const
{
    return _positionDatum.longitudeRate(_velocity_NED_mps(1));
}

Eigen::Vector3f KinematicState::velocity_Wind_mps() const
{
    return _q_nw.toRotationMatrix().transpose() * (_velocity_NED_mps - _wind_NED_mps);
}

Eigen::Vector3f KinematicState::velocity_Body_mps() const
{
    return _q_nb.toRotationMatrix().transpose() * _velocity_NED_mps;
}

Eigen::Vector3f KinematicState::acceleration_Wind_mps() const
{
    return _q_nw.toRotationMatrix().transpose() * _acceleration_NED_mps;
}

Eigen::Vector3f KinematicState::acceleration_Body_mps() const
{
    return _q_nb.toRotationMatrix().transpose() * _acceleration_NED_mps;
}

Eigen::Vector3f KinematicState::eulers() const
{
    return q_nb().toRotationMatrix().eulerAngles(3,2,1);
}

float KinematicState::roll() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(0);
}

float KinematicState::pitch() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(1);
}

float KinematicState::heading() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(2);
}

Eigen::Quaternionf KinematicState::q_nv() const
{
    return Eigen::Quaternionf::Identity();
}


float KinematicState::crab() const
{
    // tangent unit vector to ground-relative velocity
    Eigen::Vector3f Tvel_NED(_velocity_NED_mps.normalized());

    Eigen::Vector3f velocityAero_NED_mps(_velocity_NED_mps - _wind_NED_mps);

    // tangent unit vector to wind-relative velocity
    Eigen::Vector3f Twind_NED(velocityAero_NED_mps.normalized());

    return MathUtil::wrapToPi( atan2(Twind_NED(1), Twind_NED(0)) - atan2(Tvel_NED(1), Tvel_NED(0)));
}

float KinematicState::crabRate() const
{
    Eigen::Vector3f velocityAero_NED_mps(_velocity_NED_mps - _wind_NED_mps);

    // crabRate is crab angle derivative w.r.t. time
    // assume derivative of _wind_NED_mps = 0
    // d/dt velocityAero_NED_mps = _acceleration_NED_mps
    // https://en.wikipedia.org/wiki/Atan2
    float azVelDot = -_velocity_NED_mps(1)*_acceleration_NED_mps(0)/(_velocity_NED_mps(0)*_velocity_NED_mps(1) + _velocity_NED_mps(1)*_velocity_NED_mps(1))
                     + _velocity_NED_mps(0)*_acceleration_NED_mps(1)/(_velocity_NED_mps(0)*_velocity_NED_mps(1) + _velocity_NED_mps(1)*_velocity_NED_mps(1));
    float azWindDot = -velocityAero_NED_mps(1)*_acceleration_NED_mps(0)/(velocityAero_NED_mps(0)*velocityAero_NED_mps(1) + velocityAero_NED_mps(1)*velocityAero_NED_mps(1))
                     + velocityAero_NED_mps(0)*_acceleration_NED_mps(1)/(velocityAero_NED_mps(0)*velocityAero_NED_mps(1) + velocityAero_NED_mps(1)*velocityAero_NED_mps(1));

    return azWindDot - azVelDot;
}

// update the velocity frame based on the velocity vector
// velocity frame has its
// first axis aligned with the velocity vector
// second axis right perp in the local level frame
// third axis to complete the triad
void KinematicState::stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv )
{
    const float tol = 1e-6;

    Eigen::Vector3f T_hat = velocity_NED_mps.normalized();

    Eigen::Vector3f k_hat(0,0,1); // NED Down vector

    Eigen::Vector3f y_hat = k_hat.cross(T_hat);

    Eigen::Matrix3f CNV;

    if (y_hat.norm() < tol) {
        // if velocity is nearly straight up or down, then
        // make the wind axis azimuth irrotational
        // with respect to the previous orientation of the velocity frame
        // to avoid unnecessary discontinuity of the rotation
        CNV = q_nv.toRotationMatrix();

        y_hat = CNV.col(1);
        y_hat(2) = 0.0f;
        y_hat.normalize();
    } else {
        y_hat.normalize();
    }

    Eigen::Vector3f z_hat = T_hat.cross(y_hat).normalized();

    CNV << T_hat,y_hat,z_hat;

    q_nv = Eigen::Quaternionf(CNV);
}

void KinematicState::step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha,
              float beta,
              float alphaDot,
              float betaDot,
              float windSpeed_mps,
              float windDirFrom_rad)
{
    const float smallV(0.1);

    // update time
    float dt = float(time_sec - _time_sec);
    _time_sec = time_sec;

    // update stored wind
    _wind_NED_mps = -windSpeed_mps * Eigen::Vector3f(std::cos(windDirFrom_rad),
                                                      std::sin(windDirFrom_rad), 0.f);

    // save the previous frame rotations
    Eigen::Quaternionf local_q_nv(this->q_nv());
    Eigen::Quaternionf local_q_vw(this->q_nv().toRotationMatrix().transpose() * _q_nw.toRotationMatrix());
    Eigen::Quaternionf local_q_wb(_q_nw.toRotationMatrix().transpose() * _q_nb.toRotationMatrix());

    // get accelerations in the NED frame
    Eigen::Vector3f accel_NED = _q_nw.toRotationMatrix() * acceleration_Wind_mps;

    // previous wind-relative velocity expressed in NED
    Eigen::Vector3f velocityWind_NED = _velocity_NED_mps - _wind_NED_mps;

    // apply the accelerations in the wind-relative NED frame so that we
    // get the effects of heading rate in a steady wind
    velocityWind_NED += 0.5f * (_acceleration_NED_mps + accel_NED) * dt;

    Eigen::Vector3f velocity_NED_mps_prev = _velocity_NED_mps; // save to determine rotations

    // update the velocity vector based on acceleration
    _velocity_NED_mps = velocityWind_NED + _wind_NED_mps;

    // save the accel state value
    _acceleration_NED_mps = accel_NED;

    // update the velocity frame to align with the new velocity vector
    stepQnv(_velocity_NED_mps, local_q_nv);

    // path-curvature rotation: differential rotation from the velocity vector change
    Eigen::Quaternionf diff_rot_n;
    diff_rot_n.setFromTwoVectors(velocity_NED_mps_prev, _velocity_NED_mps);

    // roll rotation around Wind X axis
    Eigen::Quaternionf roll_delta(
        Eigen::AngleAxisf(rollRate_Wind_rps * dt, Eigen::Vector3f::UnitX()));

    // propagate _q_nw: path curvature rotates it in NED, roll rotates it in Wind
    _q_nw = (diff_rot_n * _q_nw * roll_delta).normalized();

    // alpha, beta -> q_wb
    Eigen::Quaternionf q_wb(Eigen::AngleAxisf(alpha, Eigen::Vector3f(0, 1, 0)) * Eigen::AngleAxisf(-beta, Eigen::Vector3f(0, 0, 1)));

    _q_nb = _q_nw * q_wb;

    // alphaDot, betaDot -> omega_bw_b
    Eigen::Vector3f omega_bw_b; // rotation rate of Body w.r.t. Wind expressed in the Body frame
    omega_bw_b << std::sin(alpha)*betaDot, alphaDot, -std::cos(alpha)*betaDot;

    // accel, vel, -> omega_wn_n
    // this is path curvature induced rotation in the POM
    Eigen::Vector3f omega_wn_n(Eigen::Vector3f::Zero()); // rotation rate of Wind w.r.t. NED expressed in the NED frame

    float normV = _velocity_NED_mps.norm();
    if (normV > smallV)
    {
        Eigen::Vector3f kappa(_velocity_NED_mps.cross(_acceleration_NED_mps) / (normV * normV * normV));
        omega_wn_n = kappa * normV;
    }

    // sum angular rate contributions
    _rates_Body_rps = omega_bw_b + q_wb.toRotationMatrix().transpose() * (_q_nw.toRotationMatrix().transpose() * omega_wn_n);
}

// https://stengel.mycpanel.princeton.edu/Quaternions.pdf
Eigen::Vector3f KinematicState::EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& ratesEuler)
{
    float sphi = sin(ang(0));
    float cphi = cos(ang(0));
    float stht = sin(ang(1));
    float ctht = cos(ang(1));

    Eigen::Matrix3f C;
    C << 1, 0, -stht, 0, cphi, sphi*ctht, 0, -sphi, cphi*ctht;

    return C*ratesEuler;
}

// https://stengel.mycpanel.princeton.edu/Quaternions.pdf
EulerRates KinematicState::BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& ratesBody)
{
    const float tol = 1e-6;

    float sphi = sin(ang(0));
    float cphi = cos(ang(0));
    float stht = sin(ang(1));
    float ctht = cos(ang(1));

    // check for gimbal lock
    if (fabs(ctht) > tol) {
        Eigen::Matrix3f C;
        C << 1, sphi*stht/ctht, cphi*stht/ctht, 0, cphi, -sphi, 0, sphi/ctht, cphi/ctht;
        return EulerRates(C*ratesBody);
    }

    return EulerRates::Zero();
}
