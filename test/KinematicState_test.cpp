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
    // With q_nb = Identity and v = [50,0,0], alpha = 0, so q_nw = Identity.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR((s.q_nw().toRotationMatrix() - Eigen::Matrix3f::Identity()).norm(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, Constructor2_QnwAlignsVelocityWithWindX) {
    // Pitched-up q_nb, purely northward velocity (β=0).
    // Wind x-axis must align with v so that velocity_Wind_mps() = [50, 0, 0].
    const float pitch = 0.2f;
    Eigen::Quaternionf q_nb(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     q_nb,
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.velocity_Wind_mps().x(), 50.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().y(),  0.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().z(),  0.f, 1e-4f);
}

TEST(KinematicStateTest, Constructor2_QnwAlignsVelocityWithWindX_WithSideslip) {
    // Identity q_nb, velocity with a sideways component (nonzero β).
    // β is derived exactly from the body-frame velocity projection.
    // velocity_Wind_mps() must still be [V, 0, 0].
    const float beta0 = 0.1f;
    const float V     = 50.f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(V * std::cos(beta0), V * std::sin(beta0), 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.velocity_Wind_mps().x(), V,   1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().y(), 0.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Wind_mps().z(), 0.f, 1e-4f);
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

// ── Phase D: position integration in step() ───────────────────────────────────

TEST(KinematicStateTest, PositionIntegration_LatitudeIncreases) {
    // 50 m/s northward, dt = 1 s → latitude increases by ≈ 50/R_N.
    KinematicState s = makeState1({50.f, 0.f, 0.f});
    s.step(1.0, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_GT(s.positionDatum().latitudeGeodetic_rad(), 0.0);
    EXPECT_NEAR(s.positionDatum().latitudeGeodetic_rad(), 50.0 / 6.3354e6, 1e-9);
}

TEST(KinematicStateTest, PositionIntegration_ClimbIncreasesHeight) {
    // V_D = -10 m/s (climbing in NED convention), dt = 1 s → altitude ≈ +10 m.
    KinematicState s = makeState1({0.f, 0.f, -10.f});
    s.step(1.0, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_GT(s.positionDatum().height_WGS84_m(), 0.0f);
    EXPECT_NEAR(s.positionDatum().height_WGS84_m(), 10.0f, 0.1f);
}

TEST(KinematicStateTest, PositionIntegration_ZeroVelocityNoChange) {
    KinematicState s = makeState1(Eigen::Vector3f::Zero());
    s.step(1.0, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.positionDatum().latitudeGeodetic_rad(), 0.0, 1e-15);
    EXPECT_NEAR(s.positionDatum().longitude_rad(),        0.0, 1e-15);
    EXPECT_NEAR(s.positionDatum().height_WGS84_m(),       0.0f, 1e-10f);
}

// ── Phase E: eulers() / roll() / pitch() / heading() ─────────────────────────

TEST(KinematicStateTest, EulersZeroForIdentityQuaternion) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.roll(),    0.f, 1e-5f);
    EXPECT_NEAR(s.pitch(),   0.f, 1e-5f);
    EXPECT_NEAR(s.heading(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, EulersPureYaw) {
    // q_nb = Rz(psi): heading only, no roll or pitch
    const float psi = 1.2f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf(Eigen::AngleAxisf(psi, Eigen::Vector3f::UnitZ())),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.roll(),    0.f, 1e-5f);
    EXPECT_NEAR(s.pitch(),   0.f, 1e-5f);
    EXPECT_NEAR(s.heading(), psi, 1e-5f);
}

TEST(KinematicStateTest, EulersPurePitch) {
    // q_nb = Ry(theta): pitch only, no roll or heading
    const float theta = 0.3f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY())),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.roll(),    0.f,   1e-5f);
    EXPECT_NEAR(s.pitch(),   theta, 1e-5f);
    EXPECT_NEAR(s.heading(), 0.f,   1e-5f);
}

TEST(KinematicStateTest, EulersPureRoll) {
    // q_nb = Rx(phi): roll only, no pitch or heading
    const float phi = 0.5f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX())),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.roll(),    phi, 1e-5f);
    EXPECT_NEAR(s.pitch(),   0.f, 1e-5f);
    EXPECT_NEAR(s.heading(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, EulersCombinedZYX) {
    // q_nb = Rz(psi) * Ry(theta) * Rx(phi) — standard ZYX aerospace convention.
    // eulers() must recover all three angles.
    const float phi   = 0.3f;
    const float theta = 0.1f;
    const float psi   = 1.2f;
    Eigen::Quaternionf q_nb(
        Eigen::AngleAxisf(psi,   Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(phi,   Eigen::Vector3f::UnitX()));
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     q_nb,
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.roll(),    phi,   1e-4f);
    EXPECT_NEAR(s.pitch(),   theta, 1e-4f);
    EXPECT_NEAR(s.heading(), psi,   1e-4f);
}

// ── Phase F: alpha() and beta() ───────────────────────────────────────────────

TEST(KinematicStateTest, AlphaBetaZeroInitially) {
    // q_nb constructor: alpha/beta initialized to 0.f.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.alpha(), 0.f, 1e-6f);
    EXPECT_NEAR(s.beta(),  0.f, 1e-6f);
}

TEST(KinematicStateTest, AlphaBetaStoredFromConstructor) {
    // q_nw constructor: alpha and beta stored from explicit parameters.
    const float alpha_val = 0.2f, beta_val = 0.1f;
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf::Identity(), 0.f,
                                  alpha_val, beta_val);
    EXPECT_NEAR(s.alpha(), alpha_val, 1e-6f);
    EXPECT_NEAR(s.beta(),  beta_val,  1e-6f);
}

TEST(KinematicStateTest, AlphaBetaStoredFromStep) {
    // step() stores alpha and beta.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    const float alpha_val = 0.2f, beta_val = 0.1f;
    s.step(0.1, Eigen::Vector3f::Zero(), 0.f, alpha_val, beta_val, 0.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.alpha(), alpha_val, 1e-6f);
    EXPECT_NEAR(s.beta(),  beta_val,  1e-6f);
}

TEST(KinematicStateTest, AlphaBetaUpdatedEachStep) {
    // Consecutive step() calls: getter reflects the most recent.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    s.step(0.1, Eigen::Vector3f::Zero(), 0.f, 0.15f, 0.05f, 0.f, 0.f, 0.f, 0.f);
    s.step(0.2, Eigen::Vector3f::Zero(), 0.f, 0.25f, 0.08f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.alpha(), 0.25f, 1e-6f);
    EXPECT_NEAR(s.beta(),  0.08f, 1e-6f);
}

// ── Phase G: alphaDot() and betaDot() ─────────────────────────────────────────

TEST(KinematicStateTest, AlphaDotBetaDotZeroInitially) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.alphaDot(), 0.f, 1e-6f);
    EXPECT_NEAR(s.betaDot(),  0.f, 1e-6f);
}

TEST(KinematicStateTest, AlphaDotStoredFromConstructor) {
    // q_nw constructor: alphaDot and betaDot stored from parameters.
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     0.f, 0.1f, 0.f, 0.05f, 0.01f, 0.f, 0.f);
    EXPECT_NEAR(s.alphaDot(), 0.05f, 1e-6f);
    EXPECT_NEAR(s.betaDot(),  0.01f, 1e-6f);
}

TEST(KinematicStateTest, AlphaDotStoredFromStep) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    s.step(0.1, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.05f, 0.01f, 0.f, 0.f);
    EXPECT_NEAR(s.alphaDot(), 0.05f, 1e-6f);
    EXPECT_NEAR(s.betaDot(),  0.01f, 1e-6f);
}

TEST(KinematicStateTest, AlphaDotUpdatedEachStep) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    s.step(0.1, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.03f, 0.f, 0.f, 0.f);
    s.step(0.2, Eigen::Vector3f::Zero(), 0.f, 0.f, 0.f, 0.07f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.alphaDot(), 0.07f, 1e-6f);
}

// ── Phase H: Euler rate methods ───────────────────────────────────────────────

TEST(KinematicStateTest, EulerRatesAllZeroForZeroBodyRates) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.rollRate_rps(),    0.f, 1e-6f);
    EXPECT_NEAR(s.pitchRate_rps(),   0.f, 1e-6f);
    EXPECT_NEAR(s.headingRate_rps(), 0.f, 1e-6f);
}

TEST(KinematicStateTest, HeadingRateEqualsYawBodyRateAtLevelFlight) {
    // q_nb=I, [p,q,r]=[0,0,r] → headingRate=r, others≈0.
    const float r = 0.5f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f(0.f, 0.f, r));
    EXPECT_NEAR(s.headingRate_rps(), r,   1e-5f);
    EXPECT_NEAR(s.rollRate_rps(),    0.f, 1e-5f);
    EXPECT_NEAR(s.pitchRate_rps(),   0.f, 1e-5f);
}

TEST(KinematicStateTest, PitchRateEqualsBodyPitchAtLevelFlight) {
    // q_nb=I, [p,q,r]=[0,q,0] → pitchRate=q, others≈0.
    const float q_rate = 0.3f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f(0.f, q_rate, 0.f));
    EXPECT_NEAR(s.pitchRate_rps(),   q_rate, 1e-5f);
    EXPECT_NEAR(s.rollRate_rps(),    0.f,    1e-5f);
    EXPECT_NEAR(s.headingRate_rps(), 0.f,    1e-5f);
}

TEST(KinematicStateTest, RollRateEqualsBodyRollAtLevelFlight) {
    // q_nb=I, [p,q,r]=[p,0,0] → rollRate=p, others≈0.
    const float p = 0.2f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f(p, 0.f, 0.f));
    EXPECT_NEAR(s.rollRate_rps(),    p,   1e-5f);
    EXPECT_NEAR(s.pitchRate_rps(),   0.f, 1e-5f);
    EXPECT_NEAR(s.headingRate_rps(), 0.f, 1e-5f);
}

// ── Phase I: rollRate_Wind_rps() ──────────────────────────────────────────────

TEST(KinematicStateTest, RollRateWindZeroInitially) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.rollRate_Wind_rps(), 0.f, 1e-6f);
}

TEST(KinematicStateTest, RollRateWindStoredFromConstructor) {
    const float rollRate = 0.3f;
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf::Identity(), rollRate);
    EXPECT_NEAR(s.rollRate_Wind_rps(), rollRate, 1e-6f);
}

TEST(KinematicStateTest, RollRateWindStoredFromStep) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    const float rollRate = 0.3f;
    s.step(0.1, Eigen::Vector3f::Zero(), rollRate, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.rollRate_Wind_rps(), rollRate, 1e-6f);
}

TEST(KinematicStateTest, RollRateWindUpdatedEachStep) {
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(50.f, 0.f, 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    s.step(0.1, Eigen::Vector3f::Zero(), 0.15f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    s.step(0.2, Eigen::Vector3f::Zero(), 0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    EXPECT_NEAR(s.rollRate_Wind_rps(), 0.25f, 1e-6f);
}

// ── Phase J: q_ns() and velocity_Stab_mps() ───────────────────────────────────

TEST(KinematicStateTest, QnsEqualsQnwAtZeroAlpha) {
    // q_nw=I, alpha=0 → q_ns = I = q_nw.
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf::Identity(), 0.f, 0.f, 0.f);
    EXPECT_NEAR((s.q_ns().toRotationMatrix() - s.q_nw().toRotationMatrix()).norm(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, QnsRotatedByAlphaFromQnw) {
    // q_nw=I, alpha=α → q_ns = Ry(α).
    const float alpha_val = 0.2f;
    KinematicState s = makeState1({50.f, 0.f, 0.f}, Eigen::Vector3f::Zero(),
                                  Eigen::Quaternionf::Identity(), 0.f, alpha_val, 0.f);
    const Eigen::Matrix3f expected =
        Eigen::AngleAxisf(alpha_val, Eigen::Vector3f::UnitY()).toRotationMatrix();
    EXPECT_NEAR((s.q_ns().toRotationMatrix() - expected).norm(), 0.f, 1e-5f);
}

TEST(KinematicStateTest, VelocityStabXIsAirspeedAtZeroAeroAngles) {
    // q_nw=I, alpha=beta=0, no wind, v=[50,0,0] → velocity_Stab = [50,0,0].
    KinematicState s = makeState1({50.f, 0.f, 0.f});
    EXPECT_NEAR(s.velocity_Stab_mps().x(), 50.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Stab_mps().y(),  0.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Stab_mps().z(),  0.f, 1e-4f);
}

TEST(KinematicStateTest, VelocityStabYIsZeroWithNonzeroBeta) {
    // q_nb=I, v=[V*cos(β), V*sin(β), 0]: sideslip β encodes q_nw=Rz(β).
    // velocity_Stab.y() must be 0 and airspeed must be preserved in x.
    const float beta_val = 0.15f;
    const float V        = 50.f;
    KinematicState s(0.0, WGS84_Datum(),
                     Eigen::Vector3f(V * std::cos(beta_val), V * std::sin(beta_val), 0.f),
                     Eigen::Vector3f::Zero(),
                     Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero());
    EXPECT_NEAR(s.velocity_Stab_mps().y(), 0.f, 1e-4f);
    EXPECT_NEAR(s.velocity_Stab_mps().x(), V,   1e-4f);
}

// ── Phase K: q_nl() ───────────────────────────────────────────────────────────

TEST(KinematicStateTest, QnlIsUnitQuaternion) {
    KinematicState s = makeState1();
    EXPECT_NEAR(s.q_nl().norm(), 1.f, 1e-5f);
}

TEST(KinematicStateTest, QnlNorthVectorMapsToECEF) {
    // At lat=0, lon=0: q_nl (ECEF→NED), so q_nl.conjugate() maps NED→ECEF.
    // NED north [1,0,0] should map to ECEF (0,0,1) = toward North Pole.
    KinematicState s = makeState1();
    const Eigen::Vector3f north_ECEF = s.q_nl().conjugate() * Eigen::Vector3f::UnitX();
    EXPECT_NEAR(north_ECEF.x(), 0.f, 1e-4f);
    EXPECT_NEAR(north_ECEF.y(), 0.f, 1e-4f);
    EXPECT_NEAR(north_ECEF.z(), 1.f, 1e-4f);
}

// ── Phase L: POM() and turnCircle() ──────────────────────────────────────────

TEST(KinematicStateTest, POMIdentityForStraightFlight) {
    // Zero acceleration → straight flight → POM defaults to Identity.
    KinematicState s = makeState1({50.f, 0.f, 0.f});
    EXPECT_NEAR((s.POM().q_np.toRotationMatrix() - Eigen::Matrix3f::Identity()).norm(),
                0.f, 1e-5f);
}

TEST(KinematicStateTest, POMXAxisAlignedWithVelocity) {
    // v=[50,0,0], centripetal a_Wind=[0,5,0] → POM x-hat = [1,0,0].
    KinematicState s = makeState1({50.f, 0.f, 0.f}, {0.f, 5.f, 0.f});
    const Eigen::Vector3f x_pom = s.POM().q_np.toRotationMatrix().col(0);
    EXPECT_NEAR(x_pom.x(), 1.f, 1e-4f);
    EXPECT_NEAR(x_pom.y(), 0.f, 1e-4f);
    EXPECT_NEAR(x_pom.z(), 0.f, 1e-4f);
}

TEST(KinematicStateTest, POMYAxisTowardCurvatureCenter) {
    // v=[50,0,0], centripetal a_Wind=[0,5,0] → POM y-hat (toward center) = [0,1,0].
    KinematicState s = makeState1({50.f, 0.f, 0.f}, {0.f, 5.f, 0.f});
    const Eigen::Vector3f y_pom = s.POM().q_np.toRotationMatrix().col(1);
    EXPECT_NEAR(y_pom.x(), 0.f, 1e-4f);
    EXPECT_NEAR(y_pom.y(), 1.f, 1e-4f);
    EXPECT_NEAR(y_pom.z(), 0.f, 1e-4f);
}

TEST(KinematicStateTest, TurnCircleRadiusCorrect) {
    // V=50, a_perp=5 m/s² → R = V²/a_perp = 500 m.
    const float V = 50.f, a = 5.f;
    KinematicState s = makeState1({V, 0.f, 0.f}, {0.f, a, 0.f});
    EXPECT_NEAR(s.turnCircle().turnCenter_deltaNED_m.norm(), V * V / a, 1.f);
}

TEST(KinematicStateTest, TurnCircleCenterInCurvatureDirection) {
    // Center displacement must point in the a_perp direction = [0,1,0].
    KinematicState s = makeState1({50.f, 0.f, 0.f}, {0.f, 5.f, 0.f});
    const Eigen::Vector3f dir = s.turnCircle().turnCenter_deltaNED_m.normalized();
    EXPECT_NEAR(dir.x(), 0.f, 1e-4f);
    EXPECT_NEAR(dir.y(), 1.f, 1e-4f);
    EXPECT_NEAR(dir.z(), 0.f, 1e-4f);
}
