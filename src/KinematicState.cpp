
#include "KinematicState.hpp"
#include "math/math_util.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

namespace {

struct PVState {
    double lat_rad;
    double lon_rad;
    float  alt_m;
    float  vN_mps;
    float  vE_mps;
    float  vD_mps;
};

struct PVDerivative {
    double lat_dot_rps;
    double lon_dot_rps;
    float  alt_dot_mps;
    float  vN_dot_mps2;
    float  vE_dot_mps2;
    float  vD_dot_mps2;
};

static PVDerivative pvDerivative(const PVState& s, const Eigen::Vector3f& accel_NED) {
    WGS84_Datum d;
    d.setLatitudeGeodetic_rad(s.lat_rad);
    d.setLongitude_rad(s.lon_rad);
    d.setHeight_WGS84_m(s.alt_m);
    return PVDerivative{
        d.latitudeRate(s.vN_mps),
        d.longitudeRate(s.vE_mps),
        -s.vD_mps,
        accel_NED(0),
        accel_NED(1),
        accel_NED(2)
    };
}

static PVState pvAdvance(const PVState& s, const PVDerivative& d, float dt) {
    return PVState{
        s.lat_rad + d.lat_dot_rps * static_cast<double>(dt),
        s.lon_rad + d.lon_dot_rps * static_cast<double>(dt),
        s.alt_m   + d.alt_dot_mps * dt,
        s.vN_mps  + d.vN_dot_mps2 * dt,
        s.vE_mps  + d.vE_dot_mps2 * dt,
        s.vD_mps  + d.vD_dot_mps2 * dt
    };
}

static PVState pvRk4(const PVState& s0, const Eigen::Vector3f& accel_NED, float dt) {
    const float h2 = 0.5f * dt;
    const PVDerivative k1 = pvDerivative(s0,                    accel_NED);
    const PVDerivative k2 = pvDerivative(pvAdvance(s0, k1, h2), accel_NED);
    const PVDerivative k3 = pvDerivative(pvAdvance(s0, k2, h2), accel_NED);
    const PVDerivative k4 = pvDerivative(pvAdvance(s0, k3, dt), accel_NED);
    const PVDerivative d_avg{
        (k1.lat_dot_rps  + 2.0*k2.lat_dot_rps  + 2.0*k3.lat_dot_rps  + k4.lat_dot_rps)  / 6.0,
        (k1.lon_dot_rps  + 2.0*k2.lon_dot_rps  + 2.0*k3.lon_dot_rps  + k4.lon_dot_rps)  / 6.0,
        (k1.alt_dot_mps  + 2.f*k2.alt_dot_mps  + 2.f*k3.alt_dot_mps  + k4.alt_dot_mps)  / 6.f,
        (k1.vN_dot_mps2  + 2.f*k2.vN_dot_mps2  + 2.f*k3.vN_dot_mps2  + k4.vN_dot_mps2) / 6.f,
        (k1.vE_dot_mps2  + 2.f*k2.vE_dot_mps2  + 2.f*k3.vE_dot_mps2  + k4.vE_dot_mps2) / 6.f,
        (k1.vD_dot_mps2  + 2.f*k2.vD_dot_mps2  + 2.f*k3.vD_dot_mps2  + k4.vD_dot_mps2) / 6.f
    };
    return pvAdvance(s0, d_avg, dt);
}

} // namespace

KinematicState::KinematicState(double time_sec,
               const WGS84_Datum &position_datum,
               const Eigen::Vector3f &velocity_NED_mps,
               const Eigen::Vector3f &acceleration_NED_mps,
               const Eigen::Quaternionf &q_nb,
               const Eigen::Vector3f &rates_Body_rps)
    : _time_sec(time_sec),
      _positionDatum(position_datum),
      _velocity_NED_mps(velocity_NED_mps),
      _acceleration_NED_mps(acceleration_NED_mps),
      _q_nb(q_nb),
      _rates_Body_rps(rates_Body_rps),
      _wind_NED_mps(Eigen::Vector3f::Zero()),
      _alpha_rad(0.f),
      _beta_rad(0.f),
      _alphaDot_rps(0.f),
      _betaDot_rps(0.f),
      _rollRate_Wind_rps(0.f)
{
    // Derive alpha and beta exactly from the body-frame velocity projection.
    // With q_wb = Ry(alpha) * Rz(-beta), the body-frame airmass velocity is:
    //   u = V*cos(alpha)*cos(beta),  v = V*cos(alpha)*sin(beta),  w = V*sin(alpha)
    // Inverting: alpha = atan2(w, sqrt(u^2+v^2)),  beta = atan2(v, u)
    // Then _q_nw = q_nb * q_wb^{-1} = q_nb * Rz(beta) * Ry(-alpha).
    const float smallV = 0.1f;
    const Eigen::Vector3f v_body = q_nb.toRotationMatrix().transpose() * velocity_NED_mps;
    float alpha = 0.0f;
    float beta  = 0.0f;
    if (v_body.norm() > smallV) {
        const float u = v_body(0);
        const float v = v_body(1);
        const float w = v_body(2);
        alpha = std::atan2(w, std::sqrt(u*u + v*v));
        beta  = std::atan2(v, u);
    }
    _q_nw = (q_nb
             * Eigen::AngleAxisf( beta, Eigen::Vector3f::UnitZ())
             * Eigen::AngleAxisf(-alpha, Eigen::Vector3f::UnitY())).normalized();
}

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
               const Eigen::Vector3f &wind_NED_mps)
    : _time_sec(time_sec),
      _positionDatum(position_datum),
      _velocity_NED_mps(velocity_NED_mps),
      _acceleration_NED_mps(q_nw.toRotationMatrix() * acceleration_Wind_mps),
      _q_nw(q_nw),
      _q_nb(Eigen::Quaternionf::Identity()),
      _rates_Body_rps(Eigen::Vector3f::Zero()),
      _wind_NED_mps(wind_NED_mps),
      _alpha_rad(alpha),
      _beta_rad(beta),
      _alphaDot_rps(alphaDot),
      _betaDot_rps(betaDot),
      _rollRate_Wind_rps(rollRate_Wind_rps)
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
    // eulerAngles(2,1,0) gives ZYX decomposition: returns [yaw, pitch, roll].
    // Reorder to [roll, pitch, heading] so that eulers()(0)=φ, (1)=θ, (2)=ψ.
    const Eigen::Vector3f ypr = q_nb().toRotationMatrix().eulerAngles(2, 1, 0);
    return {ypr(2), ypr(1), ypr(0)};
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
              const Eigen::Vector3f &wind_NED_mps)
{
    const float smallV(0.1);

    // update time
    float dt = float(time_sec - _time_sec);
    _time_sec = time_sec;

    // store aerodynamic inputs
    _alpha_rad         = alpha;
    _beta_rad          = beta;
    _alphaDot_rps      = alphaDot;
    _betaDot_rps       = betaDot;
    _rollRate_Wind_rps = rollRate_Wind_rps;

    // update stored wind
    _wind_NED_mps = wind_NED_mps;

    // save the previous velocity frame for quaternion propagation
    Eigen::Quaternionf local_q_nv(this->q_nv());

    // rotate commanded acceleration from Wind frame to NED frame
    Eigen::Vector3f accel_NED = _q_nw.toRotationMatrix() * acceleration_Wind_mps;

    // RK4 integration of position and velocity
    // Ground velocity = airspeed + wind; since wind is constant, d(vGround)/dt = accel_NED.
    const PVState pv0{
        _positionDatum.latitudeGeodetic_rad(),
        _positionDatum.longitude_rad(),
        _positionDatum.height_WGS84_m(),
        _velocity_NED_mps(0),
        _velocity_NED_mps(1),
        _velocity_NED_mps(2)
    };
    const PVState pv1 = pvRk4(pv0, accel_NED, dt);

    const Eigen::Vector3f velocity_NED_mps_prev = _velocity_NED_mps;
    _velocity_NED_mps     = Eigen::Vector3f{pv1.vN_mps, pv1.vE_mps, pv1.vD_mps};
    _acceleration_NED_mps = accel_NED;

    _positionDatum.setLatitudeGeodetic_rad(pv1.lat_rad);
    _positionDatum.setLongitude_rad(pv1.lon_rad);
    _positionDatum.setHeight_WGS84_m(pv1.alt_m);

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

float KinematicState::alpha()              const { return _alpha_rad; }
float KinematicState::beta()               const { return _beta_rad; }
float KinematicState::alphaDot()           const { return _alphaDot_rps; }
float KinematicState::betaDot()            const { return _betaDot_rps; }
float KinematicState::rollRate_Wind_rps()  const { return _rollRate_Wind_rps; }

float KinematicState::rollRate_rps()    const { return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(0); }
float KinematicState::pitchRate_rps()   const { return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(1); }
float KinematicState::headingRate_rps() const { return BodyRatesToEulerRates(eulers(), _rates_Body_rps)(2); }

Eigen::Quaternionf KinematicState::q_ns() const
{
    return (_q_nw * Eigen::Quaternionf(
                Eigen::AngleAxisf(_alpha_rad, Eigen::Vector3f::UnitY()))).normalized();
}

Eigen::Vector3f KinematicState::velocity_Stab_mps() const
{
    return q_ns().toRotationMatrix().transpose() * _velocity_NED_mps;
}

Eigen::Quaternionf KinematicState::q_nl() const
{
    return Eigen::Quaternionf(_positionDatum.qne().cast<float>());
}

const PlaneOfMotion& KinematicState::POM() const
{
    static constexpr float kSmallV     = 0.1f;
    static constexpr float kSmallAperp = 1e-4f;

    const Eigen::Vector3f v = _velocity_NED_mps - _wind_NED_mps;
    const float V = v.norm();

    if (V < kSmallV) {
        _pom.q_np = Eigen::Quaternionf::Identity();
        return _pom;
    }

    const Eigen::Vector3f xhat   = v / V;
    const Eigen::Vector3f a_par  = _acceleration_NED_mps.dot(xhat) * xhat;
    const Eigen::Vector3f a_perp = _acceleration_NED_mps - a_par;

    if (a_perp.norm() < kSmallAperp) {
        _pom.q_np = Eigen::Quaternionf::Identity();
        return _pom;
    }

    const Eigen::Vector3f yhat = a_perp.normalized();
    const Eigen::Vector3f zhat = xhat.cross(yhat);

    Eigen::Matrix3f C_NP;
    C_NP.col(0) = xhat;
    C_NP.col(1) = yhat;
    C_NP.col(2) = zhat;
    _pom.q_np = Eigen::Quaternionf(C_NP);

    return _pom;
}

const TurnCircle& KinematicState::turnCircle() const
{
    static constexpr float kSmallV     = 0.1f;
    static constexpr float kSmallAperp = 1e-4f;

    _turn_circle.pom = POM();

    const Eigen::Vector3f v = _velocity_NED_mps - _wind_NED_mps;
    const float V = v.norm();

    if (V < kSmallV) {
        _turn_circle.turnCenter_deltaNED_m = Eigen::Vector3f::Zero();
        return _turn_circle;
    }

    const Eigen::Vector3f xhat   = v / V;
    const Eigen::Vector3f a_par  = _acceleration_NED_mps.dot(xhat) * xhat;
    const Eigen::Vector3f a_perp = _acceleration_NED_mps - a_par;

    if (a_perp.norm() < kSmallAperp) {
        _turn_circle.turnCenter_deltaNED_m = Eigen::Vector3f::Zero();
        return _turn_circle;
    }

    const float R = (V * V) / a_perp.norm();
    _turn_circle.turnCenter_deltaNED_m = R * a_perp.normalized();

    return _turn_circle;
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

// ── Serialization helpers ────────────────────────────────────────────────────

static nlohmann::json vec3ToJson(const Eigen::Vector3f& v) {
    return {{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}

static nlohmann::json quatToJson(const Eigen::Quaternionf& q) {
    return {{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
}

static Eigen::Vector3f vec3FromJson(const nlohmann::json& j) {
    return {j.at("x").get<float>(), j.at("y").get<float>(), j.at("z").get<float>()};
}

static Eigen::Quaternionf quatFromJson(const nlohmann::json& j) {
    return Eigen::Quaternionf(j.at("w").get<float>(), j.at("x").get<float>(),
                              j.at("y").get<float>(), j.at("z").get<float>());
}

// ── JSON ────────────────────────────────────────────────────────────────────

nlohmann::json KinematicState::serializeJson() const {
    return {
        {"schema_version",       1},
        {"type",                 "KinematicState"},
        {"time_sec",             _time_sec},
        {"position", {
            {"latitude_rad",  _positionDatum.latitudeGeodetic_rad()},
            {"longitude_rad", _positionDatum.longitude_rad()},
            {"altitude_m",    _positionDatum.height_WGS84_m()}
        }},
        {"velocity_NED_mps",     vec3ToJson(_velocity_NED_mps)},
        {"acceleration_NED_mps", vec3ToJson(_acceleration_NED_mps)},
        {"q_nw",                 quatToJson(_q_nw)},
        {"q_nb",                 quatToJson(_q_nb)},
        {"rates_body_rps",       vec3ToJson(_rates_Body_rps)},
        {"wind_NED_mps",         vec3ToJson(_wind_NED_mps)},
        {"alpha_rad",            _alpha_rad},
        {"beta_rad",             _beta_rad},
        {"alpha_dot_rps",        _alphaDot_rps},
        {"beta_dot_rps",         _betaDot_rps},
        {"roll_rate_wind_rps",   _rollRate_Wind_rps}
    };
}

void KinematicState::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("KinematicState::deserializeJson: unsupported schema_version");

    const auto& pos = j.at("position");
    _time_sec = j.at("time_sec").get<double>();
    _positionDatum.setLatitudeGeodetic_rad(pos.at("latitude_rad").get<double>());
    _positionDatum.setLongitude_rad(pos.at("longitude_rad").get<double>());
    _positionDatum.setHeight_WGS84_m(pos.at("altitude_m").get<float>());
    _velocity_NED_mps     = vec3FromJson(j.at("velocity_NED_mps"));
    _acceleration_NED_mps = vec3FromJson(j.at("acceleration_NED_mps"));
    _q_nw                 = quatFromJson(j.at("q_nw"));
    _q_nb                 = quatFromJson(j.at("q_nb"));
    _rates_Body_rps       = vec3FromJson(j.at("rates_body_rps"));
    _wind_NED_mps         = vec3FromJson(j.at("wind_NED_mps"));
    _alpha_rad            = j.at("alpha_rad").get<float>();
    _beta_rad             = j.at("beta_rad").get<float>();
    _alphaDot_rps         = j.at("alpha_dot_rps").get<float>();
    _betaDot_rps          = j.at("beta_dot_rps").get<float>();
    _rollRate_Wind_rps    = j.at("roll_rate_wind_rps").get<float>();
}

// ── Proto ────────────────────────────────────────────────────────────────────

static void fillVec3(las_proto::Vector3f* msg, const Eigen::Vector3f& v) {
    msg->set_x(v.x()); msg->set_y(v.y()); msg->set_z(v.z());
}

static void fillQuat(las_proto::Quaternionf* msg, const Eigen::Quaternionf& q) {
    msg->set_w(q.w()); msg->set_x(q.x()); msg->set_y(q.y()); msg->set_z(q.z());
}

std::vector<uint8_t> KinematicState::serializeProto() const {
    las_proto::KinematicState msg;
    msg.set_schema_version(1);
    msg.set_time_sec(_time_sec);

    auto* pos = msg.mutable_position();
    pos->set_latitude_rad(_positionDatum.latitudeGeodetic_rad());
    pos->set_longitude_rad(_positionDatum.longitude_rad());
    pos->set_altitude_m(_positionDatum.height_WGS84_m());

    fillVec3(msg.mutable_velocity_ned_mps(),     _velocity_NED_mps);
    fillVec3(msg.mutable_acceleration_ned_mps(), _acceleration_NED_mps);
    fillQuat(msg.mutable_q_nw(),                 _q_nw);
    fillQuat(msg.mutable_q_nb(),                 _q_nb);
    fillVec3(msg.mutable_rates_body_rps(),        _rates_Body_rps);
    fillVec3(msg.mutable_wind_ned_mps(),          _wind_NED_mps);

    msg.set_alpha_rad(_alpha_rad);
    msg.set_beta_rad(_beta_rad);
    msg.set_alpha_dot_rps(_alphaDot_rps);
    msg.set_beta_dot_rps(_betaDot_rps);
    msg.set_roll_rate_wind_rps(_rollRate_Wind_rps);

    std::string buf;
    msg.SerializeToString(&buf);
    return {buf.begin(), buf.end()};
}

void KinematicState::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::KinematicState msg;
    if (!msg.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("KinematicState::deserializeProto: parse failed");
    if (msg.schema_version() != 1)
        throw std::runtime_error("KinematicState::deserializeProto: unsupported schema_version");

    const auto& pos = msg.position();
    const auto& v   = msg.velocity_ned_mps();
    const auto& a   = msg.acceleration_ned_mps();
    const auto& qnw = msg.q_nw();
    const auto& qnb = msg.q_nb();
    const auto& r   = msg.rates_body_rps();
    const auto& w   = msg.wind_ned_mps();

    _time_sec = msg.time_sec();
    _positionDatum.setLatitudeGeodetic_rad(pos.latitude_rad());
    _positionDatum.setLongitude_rad(pos.longitude_rad());
    _positionDatum.setHeight_WGS84_m(pos.altitude_m());
    _velocity_NED_mps     = {v.x(), v.y(), v.z()};
    _acceleration_NED_mps = {a.x(), a.y(), a.z()};
    _q_nw                 = Eigen::Quaternionf(qnw.w(), qnw.x(), qnw.y(), qnw.z());
    _q_nb                 = Eigen::Quaternionf(qnb.w(), qnb.x(), qnb.y(), qnb.z());
    _rates_Body_rps       = {r.x(), r.y(), r.z()};
    _wind_NED_mps         = {w.x(), w.y(), w.z()};
    _alpha_rad            = msg.alpha_rad();
    _beta_rad             = msg.beta_rad();
    _alphaDot_rps         = msg.alpha_dot_rps();
    _betaDot_rps          = msg.beta_dot_rps();
    _rollRate_Wind_rps    = msg.roll_rate_wind_rps();
}
