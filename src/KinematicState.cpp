
#include "KinematicState.hpp"
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <liteaero/nav/WGS84.hpp>
#include <cmath>
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
    const liteaero::nav::GeodeticPosition p{ s.lat_rad, s.lon_rad, s.alt_m };
    return PVDerivative{
        liteaero::nav::WGS84::latitudeRate_rad_s(p, s.vN_mps),
        liteaero::nav::WGS84::longitudeRate_rad_s(p, s.vE_mps),
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

// ── Constructor 1: q_nw, alpha, beta, wind supplied explicitly ────────────────

KinematicState::KinematicState(double time_sec,
                               const WGS84_Datum& position_datum,
                               const Eigen::Vector3f& velocity_NED_mps,
                               const Eigen::Vector3f& acceleration_Wind_mps,
                               const Eigen::Quaternionf& q_nw,
                               float rollRate_Wind_rps,
                               float alpha_rad,
                               float beta_rad,
                               float alphaDot_rps,
                               float betaDot_rps,
                               const Eigen::Vector3f& wind_NED_mps)
{
    snapshot_.time_s               = time_sec;
    snapshot_.position             = position_datum.geodeticPosition();
    snapshot_.velocity_ned_mps     = velocity_NED_mps;
    snapshot_.acceleration_ned_mps2 = q_nw.toRotationMatrix() * acceleration_Wind_mps;
    snapshot_.q_nw                 = q_nw;
    snapshot_.alpha_rad            = alpha_rad;
    snapshot_.beta_rad             = beta_rad;
    snapshot_.alpha_dot_rad_s      = alphaDot_rps;
    snapshot_.beta_dot_rad_s       = betaDot_rps;
    snapshot_.roll_rate_wind_rad_s = rollRate_Wind_rps;
    snapshot_.wind_ned_mps         = wind_NED_mps;

    // q_wb = Ry(alpha) * Rz(-beta)
    const Eigen::Quaternionf q_wb(
        Eigen::AngleAxisf(alpha_rad, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(-beta_rad, Eigen::Vector3f::UnitZ()));

    // omega_bw_b: rotation rate of Body w.r.t. Wind expressed in Body frame
    Eigen::Vector3f omega_bw_b;
    omega_bw_b << std::sin(alpha_rad) * betaDot_rps,
                  alphaDot_rps,
                  -std::cos(alpha_rad) * betaDot_rps;

    // omega_wn_n: path-curvature rotation rate of Wind w.r.t. NED in NED frame
    Eigen::Vector3f omega_wn_n = Eigen::Vector3f::Zero();
    const float normV = velocity_NED_mps.norm();
    if (normV > 0.1f) {
        const Eigen::Vector3f kappa =
            velocity_NED_mps.cross(snapshot_.acceleration_ned_mps2) / (normV * normV * normV);
        omega_wn_n = kappa * normV;
    }

    snapshot_.rates_body_rps = omega_bw_b
        + q_wb.toRotationMatrix().transpose()
          * (q_nw.toRotationMatrix().transpose() * omega_wn_n);
}

// ── Constructor 2: q_nb supplied; alpha, beta, q_nw are derived ──────────────

KinematicState::KinematicState(double time_sec,
                               const WGS84_Datum& position_datum,
                               const Eigen::Vector3f& velocity_NED_mps,
                               const Eigen::Vector3f& acceleration_NED_mps,
                               const Eigen::Quaternionf& q_nb,
                               const Eigen::Vector3f& rates_Body_rps)
{
    snapshot_.time_s               = time_sec;
    snapshot_.position             = position_datum.geodeticPosition();
    snapshot_.velocity_ned_mps     = velocity_NED_mps;
    snapshot_.acceleration_ned_mps2 = acceleration_NED_mps;
    snapshot_.rates_body_rps       = rates_Body_rps;
    snapshot_.wind_ned_mps         = Eigen::Vector3f::Zero();
    snapshot_.alpha_dot_rad_s      = 0.0f;
    snapshot_.beta_dot_rad_s       = 0.0f;
    snapshot_.roll_rate_wind_rad_s = 0.0f;

    // Derive alpha and beta from the body-frame airmass velocity projection.
    // With q_wb = Ry(alpha) * Rz(-beta), the body-frame airmass velocity is:
    //   u = V·cos(α)·cos(β),  v = V·cos(α)·sin(β),  w = V·sin(α)
    // Inverting: α = atan2(w, sqrt(u²+v²)),  β = atan2(v, u)
    const Eigen::Vector3f v_body = q_nb.toRotationMatrix().transpose() * velocity_NED_mps;
    float alpha = 0.0f;
    float beta  = 0.0f;
    if (v_body.norm() > 0.1f) {
        const float u = v_body(0);
        const float v = v_body(1);
        const float w = v_body(2);
        alpha = std::atan2(w, std::sqrt(u*u + v*v));
        beta  = std::atan2(v, u);
    }
    snapshot_.alpha_rad = alpha;
    snapshot_.beta_rad  = beta;

    // q_nw = q_nb * Rz(beta) * Ry(-alpha)
    snapshot_.q_nw = (q_nb
        * Eigen::AngleAxisf( beta,  Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(-alpha, Eigen::Vector3f::UnitY())).normalized();
}

// ── Simulation engine ─────────────────────────────────────────────────────────

void KinematicState::step(double time_sec,
                          Eigen::Vector3f acceleration_Wind_mps,
                          float rollRate_Wind_rps,
                          float alpha_rad,
                          float beta_rad,
                          float alphaDot_rps,
                          float betaDot_rps,
                          const Eigen::Vector3f& wind_NED_mps)
{
    const float dt = static_cast<float>(time_sec - snapshot_.time_s);
    stepPV(time_sec, acceleration_Wind_mps, alpha_rad, beta_rad,
           alphaDot_rps, betaDot_rps, wind_NED_mps);
    commitAttitude(rollRate_Wind_rps, dt);
}

void KinematicState::stepPV(double time_sec,
                             Eigen::Vector3f acceleration_Wind_mps,
                             float alpha_rad,
                             float beta_rad,
                             float alphaDot_rps,
                             float betaDot_rps,
                             const Eigen::Vector3f& wind_NED_mps)
{
    const float dt = static_cast<float>(time_sec - snapshot_.time_s);

    // Capture v_prev before integration; commitAttitude uses it as the "from" vector.
    velocity_prev_ned_mps_ = snapshot_.velocity_ned_mps;

    snapshot_.time_s               = time_sec;
    snapshot_.alpha_rad            = alpha_rad;
    snapshot_.beta_rad             = beta_rad;
    snapshot_.alpha_dot_rad_s      = alphaDot_rps;
    snapshot_.beta_dot_rad_s       = betaDot_rps;
    snapshot_.wind_ned_mps         = wind_NED_mps;

    // Rotate commanded acceleration from Wind frame to NED frame
    const Eigen::Vector3f accel_NED = snapshot_.q_nw.toRotationMatrix() * acceleration_Wind_mps;

    // RK4 integration of position and velocity
    const PVState pv0{
        snapshot_.position.latitude_rad,
        snapshot_.position.longitude_rad,
        snapshot_.position.altitude_m,
        snapshot_.velocity_ned_mps(0),
        snapshot_.velocity_ned_mps(1),
        snapshot_.velocity_ned_mps(2)
    };
    const PVState pv1 = pvRk4(pv0, accel_NED, dt);

    snapshot_.velocity_ned_mps       = {pv1.vN_mps, pv1.vE_mps, pv1.vD_mps};
    snapshot_.acceleration_ned_mps2  = accel_NED;
    snapshot_.position.latitude_rad  = pv1.lat_rad;
    snapshot_.position.longitude_rad = pv1.lon_rad;
    snapshot_.position.altitude_m    = pv1.alt_m;
}

void KinematicState::commitAttitude(float rollRate_Wind_rps, float dt_s)
{
    snapshot_.roll_rate_wind_rad_s = rollRate_Wind_rps;

    // Propagate q_nw: path-curvature rotation + roll around Wind X axis.
    // velocity_prev_ned_mps_ was set by stepPV and is aligned with q_nw.x by construction.
    // snapshot_.velocity_ned_mps is the fully modified final velocity for this step.
    stepQnw(velocity_prev_ned_mps_, snapshot_.velocity_ned_mps,
            rollRate_Wind_rps, dt_s, snapshot_.q_nw);

    // q_wb = Ry(alpha) * Rz(-beta)
    const Eigen::Quaternionf q_wb(
        Eigen::AngleAxisf(snapshot_.alpha_rad, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(-snapshot_.beta_rad, Eigen::Vector3f::UnitZ()));

    // omega_bw_b: rotation rate of Body w.r.t. Wind expressed in Body frame
    Eigen::Vector3f omega_bw_b;
    omega_bw_b << std::sin(snapshot_.alpha_rad) * snapshot_.beta_dot_rad_s,
                  snapshot_.alpha_dot_rad_s,
                  -std::cos(snapshot_.alpha_rad) * snapshot_.beta_dot_rad_s;

    // omega_wn_n: path-curvature rotation rate of Wind w.r.t. NED in NED frame
    Eigen::Vector3f omega_wn_n = Eigen::Vector3f::Zero();
    const float normV = snapshot_.velocity_ned_mps.norm();
    if (normV > 0.1f) {
        const Eigen::Vector3f kappa =
            snapshot_.velocity_ned_mps.cross(snapshot_.acceleration_ned_mps2) /
            (normV * normV * normV);
        omega_wn_n = kappa * normV;
    }

    snapshot_.rates_body_rps = omega_bw_b
        + q_wb.toRotationMatrix().transpose()
          * (snapshot_.q_nw.toRotationMatrix().transpose() * omega_wn_n);
}

// OQ-LG-21: propagate the attitude toward an externally-supplied reference velocity
// (the dynamic-pressure blend of instantaneous and low-pass-filtered velocity) and derive
// the body rate from the resulting committed attitude (consistent, no (v×a)/|V|² spike).
void KinematicState::commitAttitude(float rollRate_Wind_rps, float dt_s,
                                    const Eigen::Vector3f& attitude_ref_velocity_ned_mps,
                                    float max_curvature_per_m)
{
    snapshot_.roll_rate_wind_rad_s = rollRate_Wind_rps;

    if (!att_ref_init_) {
        att_ref_prev_ned_mps_ = attitude_ref_velocity_ned_mps;
        q_nb_prev_            = liteaero::nav::KinematicStateUtil::q_nb(snapshot_);
        att_ref_init_         = true;
    }

    // OQ-AC-2 Alt 4: saturate the per-step direction change of the attitude-reference velocity to
    // theta_max = (V/R_min)·dt (R_min = 1/max_curvature_per_m). Rotate the stored previous reference
    // toward the new one by at most theta_max, keeping the current speed. Feeding this SATURATED
    // reference forward makes it slew toward — and converge to — the true velocity when the direction
    // is steady (no permanent lag), while bounding the rate as V → 0. q_nw then tracks the saturated
    // reference STRICTLY in stepQnw, so q_nw.x = v_hat always holds (no decouple, no crab launch).
    Eigen::Vector3f v_ref_sat = attitude_ref_velocity_ned_mps;
    if (max_curvature_per_m > 0.f && dt_s > 0.f) {
        const float speed_mps        = attitude_ref_velocity_ned_mps.norm();
        const float theta_max_rad    = max_curvature_per_m * speed_mps * dt_s;
        static constexpr float kMinNorm = 1e-4f;
        if (att_ref_prev_ned_mps_.norm() > kMinNorm && speed_mps > kMinNorm) {
            Eigen::Quaternionf q_dir;
            q_dir.setFromTwoVectors(att_ref_prev_ned_mps_, attitude_ref_velocity_ned_mps);
            const Eigen::AngleAxisf aa(q_dir);      // desired direction change (angle in [0, pi])
            if (aa.angle() > theta_max_rad) {
                v_ref_sat = Eigen::AngleAxisf(theta_max_rad, aa.axis())
                            * att_ref_prev_ned_mps_.normalized() * speed_mps;
            }
        }
    }

    // Propagate q_nw toward the saturated reference velocity; q_nw tracks it strictly.
    stepQnw(att_ref_prev_ned_mps_, v_ref_sat, rollRate_Wind_rps, dt_s, snapshot_.q_nw);
    att_ref_prev_ned_mps_ = v_ref_sat;   // feed the SATURATED reference forward (rate-limited slew)

    // Body rate = genuine rate of change of the committed attitude (consistent with it).
    const Eigen::Quaternionf q_nb_now = liteaero::nav::KinematicStateUtil::q_nb(snapshot_);
    if (dt_s > 0.f) {
        Eigen::Quaternionf dq = q_nb_prev_.conjugate() * q_nb_now;
        if (dq.w() < 0.f) dq.coeffs() = -dq.coeffs();   // shortest rotation
        snapshot_.rates_body_rps = (2.0f / dt_s) * dq.vec();
    } else {
        snapshot_.rates_body_rps = Eigen::Vector3f::Zero();
    }
    q_nb_prev_ = q_nb_now;
}

// ── Static simulation helpers ─────────────────────────────────────────────────

void KinematicState::stepQnw(const Eigen::Vector3f& velocity_prev_NED_mps,
                              const Eigen::Vector3f& velocity_NED_mps,
                              float rollRate_Wind_rps,
                              float dt_s,
                              Eigen::Quaternionf& q_nw)
{
    // Roll rotation around Wind X axis.
    const Eigen::Quaternionf roll_delta(
        Eigen::AngleAxisf(rollRate_Wind_rps * dt_s, Eigen::Vector3f::UnitX()));

    // Path-curvature rotation: the differential rotation from the velocity-vector change. q_nw tracks
    // the (reference) velocity STRICTLY — q_nw.x = v_hat — by construction. The OQ-AC-2 speed-proportional
    // rate limit is applied UPSTREAM, by saturating the reference-velocity slew in commitAttitude, NOT
    // here (a cap on the rotation itself would decouple q_nw from the velocity and launch a crabbed
    // touchdown). setFromTwoVectors normalizes internally, so guard only against an exact-zero norm — a
    // numerical epsilon, replacing the former discontinuous kMinSpeed cutoff.
    static constexpr float kMinNorm = 1e-4f;
    if (velocity_prev_NED_mps.norm() < kMinNorm || velocity_NED_mps.norm() < kMinNorm) {
        q_nw = (q_nw * roll_delta).normalized();
        return;
    }

    Eigen::Quaternionf diff_rot_n;
    diff_rot_n.setFromTwoVectors(velocity_prev_NED_mps, velocity_NED_mps);
    q_nw = (diff_rot_n * q_nw * roll_delta).normalized();
}

void KinematicState::stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv)
{
    const float tol = 1e-6f;

    const Eigen::Vector3f T_hat = velocity_NED_mps.normalized();
    const Eigen::Vector3f k_hat(0.f, 0.f, 1.f); // NED Down vector
    Eigen::Vector3f y_hat = k_hat.cross(T_hat);

    Eigen::Matrix3f CNV;
    if (y_hat.norm() < tol) {
        // Velocity is nearly straight up or down: preserve azimuth continuity
        CNV = q_nv.toRotationMatrix();
        y_hat = CNV.col(1);
        y_hat(2) = 0.0f;
        y_hat.normalize();
    } else {
        y_hat.normalize();
    }

    const Eigen::Vector3f z_hat = T_hat.cross(y_hat).normalized();
    CNV << T_hat, y_hat, z_hat;
    q_nv = Eigen::Quaternionf(CNV);
}

// ── Derived quantities ────────────────────────────────────────────────────────

Eigen::Vector3f KinematicState::eulers() const
{
    return {
        liteaero::nav::KinematicStateUtil::roll_rad(snapshot_),
        liteaero::nav::KinematicStateUtil::pitch_rad(snapshot_),
        liteaero::nav::KinematicStateUtil::heading_rad(snapshot_)
    };
}

Eigen::Vector3f KinematicState::EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& rates)
{
    return liteaero::nav::KinematicStateUtil::euler_rates_to_body_rates(ang, rates);
}

EulerRates KinematicState::BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& rates)
{
    return liteaero::nav::KinematicStateUtil::body_rates_to_euler_rates(ang, rates);
}

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json KinematicState::serializeJson() const
{
    return liteaero::nav::KinematicStateUtil::serializeJson(snapshot_);
}

void KinematicState::deserializeJson(const nlohmann::json& j)
{
    snapshot_ = liteaero::nav::KinematicStateUtil::deserializeJson(j);
}

std::vector<uint8_t> KinematicState::serializeProto() const
{
    return liteaero::nav::KinematicStateUtil::serializeProto(snapshot_);
}

void KinematicState::deserializeProto(const std::vector<uint8_t>& bytes)
{
    snapshot_ = liteaero::nav::KinematicStateUtil::deserializeProto(bytes);
}

void KinematicState::applyTerrainHardConstraint(float penetration_m, float restitution_nd)
{
    snapshot_.position.altitude_m += penetration_m;
    // Restitution-consistent velocity correction: remove (1 + e) of the downward
    // approach component, i.e. v_D -> -e * v_D.  e = 0 fully arrests (inelastic);
    // 0 < e < 1 rebounds at that fraction.  Upward velocity is left untouched.
    if (snapshot_.velocity_ned_mps.z() > 0.f)
        snapshot_.velocity_ned_mps.z() *= -restitution_nd;
}
