#define _USE_MATH_DEFINES
#include "KinematicState.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Helper: construct a state via Constructor 1 with commonly varied parameters.
static KinematicState makeState1(
    const Eigen::Vector3f&  vel_NED          = {50.f, 0.f, 0.f},
    const Eigen::Vector3f&  accel_Wind       = Eigen::Vector3f::Zero(),
    const Eigen::Quaternionf& q_nw           = Eigen::Quaternionf::Identity(),
    float rollRate_Wind_rps                  = 0.f,
    float alpha_rad                          = 0.f,
    float beta_rad                           = 0.f,
    float windSpeed_mps                      = 0.f,
    float windDirFrom_rad                    = 0.f)
{
    return KinematicState(0.0, WGS84_Datum(), vel_NED, accel_Wind,
                          q_nw, rollRate_Wind_rps, alpha_rad, beta_rad,
                          0.f, 0.f, windSpeed_mps, windDirFrom_rad);
}

static Eigen::Quaternionf makeQwb(float alpha_rad, float beta_rad) {
    return Eigen::Quaternionf(
        Eigen::AngleAxisf(alpha_rad,  Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(-beta_rad, Eigen::Vector3f::UnitZ()));
}

// ── Phase B: _q_nw storage and step() ────────────────────────────────────────

TEST(KinematicStateTest, QnwStoredFromConstructor) {
    // Constructor 1 should store the supplied q_nw exactly.
    Eigen::Quaternionf q_nw_init(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitY()));
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(), q_nw_init);
    EXPECT_NEAR((s.q_nw().toRotationMatrix() - q_nw_init.toRotationMatrix()).norm(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, QnwIsIdentityFromSimpleConstructor) {
    // Constructor 2 (takes q_nb directly) should default q_nw to Identity.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR((s.q_nw().toRotationMatrix() - Eigen::Matrix3f::Identity()).norm(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, QnwChangesAfterRollStep) {
    // With nonzero rollRate_Wind_rps and zero acceleration, _q_nw should rotate
    // by exactly rollRate * dt around the Wind X axis.
    KinematicState s = makeState1();          // Identity q_nw, V=[50,0,0], zero accel

    const float rollRate = 0.1f;
    const float dt       = 0.1f;
    s.step(dt, Eigen::Vector3f::Zero(), rollRate, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    // With zero acceleration diff_rot_n = Identity, so _q_nw = AngleAxisf(rollRate*dt, X).
    Eigen::AngleAxisf aa(s.q_nw());
    EXPECT_NEAR(aa.angle(), rollRate * dt, 1e-4f);
    EXPECT_NEAR(std::abs(aa.axis().x()), 1.f, 1e-4f);
}

TEST(KinematicStateTest, QnbEqualsQnwTimesQwb) {
    // After a step the body frame must satisfy q_nb = q_nw * q_wb(alpha, beta).
    KinematicState s = makeState1();

    const float alpha = 0.1f, beta = 0.05f;
    s.step(0.1, Eigen::Vector3f::Zero(), 0.f, alpha, beta, 0.f, 0.f, 0.f, 0.f);

    Eigen::Quaternionf expected_q_nb = s.q_nw() * makeQwb(alpha, beta);
    Eigen::Matrix3f diff = s.q_nb().toRotationMatrix() - expected_q_nb.toRotationMatrix();
    EXPECT_NEAR(diff.norm(), 0.f, 1e-4f);
}

// ── Phase C: derived quantity implementations ─────────────────────────────────

TEST(KinematicStateTest, VelocityWindEqualsGroundSpeedZeroWind) {
    // Zero wind, Identity q_nw: Wind-frame velocity equals NED velocity.
    KinematicState s = makeState1({50.f, 0.f, 0.f});
    EXPECT_NEAR(s.velocity_Wind_mps().x(), 50.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().y(),  0.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().z(),  0.f, 1e-4f);
}

TEST(KinematicStateTest, VelocityWindSubtractsWind) {
    // Wind FROM north (dir=0) at 10 m/s → wind velocity in NED = [-10, 0, 0].
    // Aircraft flying north at 50 m/s → airspeed = 60 m/s (headwind).
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf::Identity(), 0.f, 0.f, 0.f,
                                  10.f, 0.f);
    EXPECT_NEAR(s.velocity_Wind_mps().x(), 60.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().y(),  0.f, 1e-4f);
}

TEST(KinematicStateTest, VelocityBodyNonZeroAlpha) {
    // q_nb = pitch up by alpha.  velocity_Body_mps = C_NB^T * V_NED.
    const float alpha = 0.1f;
    Eigen::Quaternionf q_nb(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitY()));
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     q_nb,
                     Eigen::Vector3f::Zero());
    // C_NB^T * [50,0,0] = [50*cos(α), 0, 50*sin(α)]
    EXPECT_NEAR(s.velocity_Body_mps().x(), 50.f * std::cos(alpha), 1e-4f);
    EXPECT_NEAR(s.velocity_Body_mps().y(),  0.f,                   1e-4f);
    EXPECT_NEAR(s.velocity_Body_mps().z(), 50.f * std::sin(alpha), 1e-4f);
}

TEST(KinematicStateTest, AccelerationWindRotatesFromNED) {
    // q_nw = 90° around NED Z: Wind X maps to NED Y.
    // Constructor stores _accel_NED = C_NW * accel_Wind.
    // acceleration_Wind_mps() = C_WN * _accel_NED = accel_Wind (round-trip).
    // But the stored NED acceleration is [0, 10, 0], so the inverse rotation is tested.
    Eigen::Quaternionf q_nw_90z(Eigen::AngleAxisf(static_cast<float>(M_PI_2),
                                                    Eigen::Vector3f::UnitZ()));
    KinematicState s = makeState1({50.f, 0.f, 0.f},
                                  {10.f, 0.f, 0.f},   // accel_Wind = [10, 0, 0]
                                  q_nw_90z);

    // _accel_NED = C_NW * [10,0,0] = [0, 10, 0]
    EXPECT_NEAR(s.acceleration_NED_mps().x(),  0.f, 1e-4f);
    EXPECT_NEAR(s.acceleration_NED_mps().y(), 10.f, 1e-4f);

    // acceleration_Wind_mps() = C_WN * [0, 10, 0] = [10, 0, 0]
    EXPECT_NEAR(s.acceleration_Wind_mps().x(), 10.f, 1e-4f);
    EXPECT_NEAR(s.acceleration_Wind_mps().y(),  0.f, 1e-4f);
}

TEST(KinematicStateTest, AccelerationBodyRotatesFromNED) {
    // q_nb = 90° around Z: Body X maps to NED Y.
    // acceleration_Body_mps() = C_BN * _accel_NED.
    Eigen::Quaternionf q_nb_90z(Eigen::AngleAxisf(static_cast<float>(M_PI_2),
                                                    Eigen::Vector3f::UnitZ()));
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f(10.f, 0.f, 0.f),  // accel_NED
                     q_nb_90z,
                     Eigen::Vector3f::Zero());

    // C_NB^T * [10, 0, 0] = C_BN * [10,0,0]:
    // C_NB (90° around Z) = [[0,-1,0],[1,0,0],[0,0,1]]
    // C_NB^T = [[0,1,0],[-1,0,0],[0,0,1]]
    // C_NB^T * [10,0,0] = [0, -10, 0]
    EXPECT_NEAR(s.acceleration_Body_mps().x(),   0.f, 1e-4f);
    EXPECT_NEAR(s.acceleration_Body_mps().y(), -10.f, 1e-4f);
    EXPECT_NEAR(s.acceleration_Body_mps().z(),   0.f, 1e-4f);
}

TEST(KinematicStateTest, LatitudeRatePositiveForNorthwardVelocity) {
    // 50 m/s northward at lat=0, alt=0 → latitudeRate > 0, ≈ 50/R_N.
    KinematicState s = makeState1({50.f, 0.f, 0.f});
    EXPECT_GT(s.latitudeRate_rps(), 0.0);
    // Meridional radius at equator ≈ 6,335,439 m
    EXPECT_NEAR(s.latitudeRate_rps(), 50.0 / 6.3354e6, 1e-7);
}

TEST(KinematicStateTest, LongitudeRatePositiveForEastwardVelocity) {
    // 50 m/s eastward at lat=0, alt=0 → longitudeRate > 0, ≈ 50/R_E.
    KinematicState s = makeState1({0.f, 50.f, 0.f});
    EXPECT_GT(s.longitudeRate_rps(), 0.0);
    // Prime-vertical radius at equator ≈ 6,378,137 m
    EXPECT_NEAR(s.longitudeRate_rps(), 50.0 / 6.3781e6, 1e-7);
}

TEST(KinematicStateTest, ZeroVelocityZeroPositionRate) {
    KinematicState s = makeState1(Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.latitudeRate_rps(),  0.0, 1e-15);
    EXPECT_NEAR(s.longitudeRate_rps(), 0.0, 1e-15);
}
