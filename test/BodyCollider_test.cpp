// BodyCollider unit tests.
// Design authority: docs/architecture/landing_gear.md §BodyCollider

#include "collision/BodyCollider.hpp"
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <liteaero/terrain/Terrain.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using namespace liteaero::simulation;
using liteaero::nav::KinematicStateSnapshot;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

class FlatTerrain : public liteaero::terrain::Terrain {
public:
    explicit FlatTerrain(float elev_m) : elev_(elev_m) {}
    [[nodiscard]] float elevation_m(double, double) const override { return elev_; }
private:
    float elev_;
};

static nlohmann::json makeConfig(
    float hx = 0.5f, float hy = 0.3f, float hz = 0.2f,
    float stiffness = 10000.f, float damping = 500.f)
{
    return {{"volumes", nlohmann::json::array({
        {
            {"name",                 "test"},
            {"half_extents_body_m",  {hx, hy, hz}},
            {"center_offset_body_m", {0.f, 0.f, 0.f}},
            {"stiffness_npm",        stiffness},
            {"damping_nspm",         damping}
        }
    })}};
}

static KinematicStateSnapshot makeSnap(
    float alt_m,
    Eigen::Quaternionf q_nb        = Eigen::Quaternionf::Identity(),
    Eigen::Vector3f    vel_ned_mps = Eigen::Vector3f::Zero(),
    Eigen::Vector3f    rates_rps   = Eigen::Vector3f::Zero())
{
    KinematicStateSnapshot s;
    s.position.altitude_m    = alt_m;
    s.position.latitude_rad  = 0.0;
    s.position.longitude_rad = 0.0;
    s.q_nw                   = q_nb;   // at zero velocity q_nw == q_nb
    s.alpha_rad              = 0.f;
    s.beta_rad               = 0.f;
    s.velocity_ned_mps       = vel_ned_mps;
    s.rates_body_rps         = rates_rps;
    return s;
}

// ---------------------------------------------------------------------------
// Basic contact detection
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, NoContact_WhenWellAboveTerrain) {
    BodyCollider c;
    c.initialize(makeConfig());
    FlatTerrain terrain{0.f};

    // Center at 10 m, half_z = 0.2 → bottom corners at 9.8 m > 0
    const auto cf = c.step(makeSnap(10.f), terrain);
    EXPECT_FALSE(cf.weight_on_wheels);
    EXPECT_FLOAT_EQ(cf.force_body_n.norm(), 0.f);
}

TEST(BodyColliderTest, NoContact_BottomCornersJustAboveTerrain) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Aircraft at alt = 0.21 m → bottom corners (body-z = +0.2) are at NED z offset +0.2
    // corner altitude = 0.21 - 0.20 = 0.01 m > terrain 0 m → no contact
    const auto cf = c.step(makeSnap(0.21f), terrain);
    EXPECT_FALSE(cf.weight_on_wheels);
    EXPECT_FLOAT_EQ(cf.force_body_n.norm(), 0.f);
}

TEST(BodyColliderTest, Contact_BottomCornersBelow) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Aircraft at alt = 0.1 m → bottom corners at altitude 0.1 - 0.2 = -0.1 m
    // penetration = 0 - (-0.1) = 0.1 m → should get upward force
    const auto cf = c.step(makeSnap(0.1f), terrain);
    EXPECT_TRUE(cf.weight_on_wheels);
    EXPECT_GT(cf.force_body_n.norm(), 0.f);
}

// ---------------------------------------------------------------------------
// Force direction — level aircraft
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ForceIsUpward_BodyFrame_LevelAircraft) {
    BodyCollider c;
    c.initialize(makeConfig());

    // Level aircraft (Identity q_nb), penetrating 0.1 m → force must be in -body-Z (upward)
    const auto cf = c.step(makeSnap(0.1f));
    // body-Z is down; upward = negative body-Z
    EXPECT_LT(cf.force_body_n.z(), 0.f);
    // No lateral or fore-aft force from a vertical terrain normal
    EXPECT_NEAR(cf.force_body_n.x(), 0.f, 1e-3f);
    EXPECT_NEAR(cf.force_body_n.y(), 0.f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Spring force proportional to penetration
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ForceProportionalToPenetration) {
    // With no damping, force is purely spring: F = k * pen * n_corners_in_contact.
    // Both cases hit the same 4 bottom corners with different penetration depths.
    BodyCollider c1, c2;
    c1.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 0.f));
    c2.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 0.f));

    const auto cf1 = c1.step(makeSnap(0.1f));   // pen = 0.1 m per 4 corners
    const auto cf2 = c2.step(makeSnap(0.05f));  // pen = 0.15 m per 4 corners

    // Force magnitude ratio should equal penetration ratio: 0.15/0.10 = 1.5
    const float F1 = -cf1.force_body_n.z();
    const float F2 = -cf2.force_body_n.z();
    ASSERT_GT(F1, 0.f);
    EXPECT_NEAR(F2 / F1, 1.5f, 0.05f);
}

// ---------------------------------------------------------------------------
// Damping adds when sinking, not when rising
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, DampingIncreasesForce_WhenSinkingIntoTerrain) {
    BodyCollider c;
    // Use non-zero damping
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 500.f));

    // Static case (zero velocity)
    const auto cf_static = c.step(makeSnap(0.1f));

    // Sinking at 1 m/s downward in NED (velocity_down = +1)
    const auto cf_sinking = c.step(
        makeSnap(0.1f, Eigen::Quaternionf::Identity(),
                 Eigen::Vector3f{0.f, 0.f, 1.f}));

    const float F_static  = -cf_static.force_body_n.z();
    const float F_sinking = -cf_sinking.force_body_n.z();
    EXPECT_GT(F_sinking, F_static) << "Damping should add force when sinking";
}

TEST(BodyColliderTest, TwoWayDamping_WhenRising_ForceReducedButNotNegative) {
    // Two-way damping: rising from terrain must reduce the contact force relative
    // to the static case (energy absorption), but the force must remain >= 0
    // (no suction pulling the aircraft back into the ground).
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 500.f));

    const auto cf_static = c.step(makeSnap(0.1f));
    const auto cf_rising = c.step(
        makeSnap(0.1f, Eigen::Quaternionf::Identity(),
                 Eigen::Vector3f{0.f, 0.f, -1.f}));

    const float F_static = -cf_static.force_body_n.z();
    const float F_rising = -cf_rising.force_body_n.z();
    EXPECT_LT(F_rising, F_static)
        << "Two-way damping must reduce contact force when aircraft is rising";
    EXPECT_GE(F_rising, 0.f)
        << "No suction: contact force must remain non-negative when rising";
}

// ---------------------------------------------------------------------------
// Inverted aircraft — protection for upside-down crash
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, Inverted_180DegRoll_UpperCornersContact) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 0.f));

    // Roll 180° around body-X (forward axis): body-Z now points NED-up.
    // Aircraft center at alt = 0.1 m.
    // "Top" corners in body frame (body-z = -0.2) now map to NED-down = +0.2.
    // Corner altitude = 0.1 - 0.2 = -0.1 → penetration = 0.1 m → contact.
    const Eigen::Quaternionf q_inverted(
        Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitX()));

    const auto cf = c.step(makeSnap(0.1f, q_inverted));
    EXPECT_TRUE(cf.weight_on_wheels);
    // Force must be upward in body — but inverted means body-Z points up, so
    // upward in world = positive body-Z in inverted aircraft.
    // Actually: force in body frame = R_bn * F_NED. F_NED is upward = [0,0,-1] * F_n.
    // For inverted R_nb: body-Z maps to NED-up, so R_bn.col(2) = [0,0,-1] in NED.
    // R_bn * [0,0,-F_n] = R_nb^T * [0,0,-F_n].
    // Inverted: R_nb = diag(1,-1,-1), so R_nb^T * [0,0,-F_n] = [0, 0, F_n].
    // So force is in +body-Z (which is upward in world when inverted).
    EXPECT_GT(cf.force_body_n.norm(), 0.f);
}

// ---------------------------------------------------------------------------
// Moment arm: off-center contact produces a moment
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, Moment_ProducedWhenContactIsOffCenter) {
    // Center offset pushes all corners forward: nose will be lower when pitched up.
    // Use center_offset_body_m = [0.3, 0, 0] (box shifted forward).
    nlohmann::json cfg = {{"volumes", nlohmann::json::array({
        {
            {"name",                 "test"},
            {"half_extents_body_m",  {0.5f, 0.3f, 0.2f}},
            {"center_offset_body_m", {0.3f, 0.f, 0.f}},
            {"stiffness_npm",        10000.f},
            {"damping_nspm",         0.f}
        }
    })}};
    BodyCollider c;
    c.initialize(cfg);

    // Level aircraft, center offset moves all corners forward.
    // Because box is shifted forward, all 8 corners have the same z penetration
    // but they're offset in x → moment_body_nm.y() should be non-zero (pitching moment).
    const auto cf = c.step(makeSnap(0.1f));
    EXPECT_GT(std::abs(cf.moment_body_nm.y()), 0.f)
        << "Forward-shifted box should produce a pitching moment";
}

// ---------------------------------------------------------------------------
// maxCornerPenetration_m — post-integration hard constraint query
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, MaxCornerPenetration_ZeroWhenAboveTerrain) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Bottom corners at 10 - 0.2 = 9.8 m above terrain — no penetration.
    EXPECT_FLOAT_EQ(c.maxCornerPenetration_m(makeSnap(10.f), terrain), 0.f);
}

TEST(BodyColliderTest, MaxCornerPenetration_MatchesDeepestCorner) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Level aircraft at alt = 0.1 m:
    //   Bottom corners (body-z = +0.2): altitude = 0.1 - 0.2 = -0.1 m → pen = 0.1 m
    //   Top corners (body-z = -0.2): altitude = 0.1 + 0.2 = 0.3 m → clear
    const float pen = c.maxCornerPenetration_m(makeSnap(0.1f), terrain);
    EXPECT_NEAR(pen, 0.1f, 1e-4f);
}

TEST(BodyColliderTest, MaxCornerPenetration_ZeroWhenJustAbove) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Bottom corners exactly at terrain level: pen = 0.
    EXPECT_FLOAT_EQ(c.maxCornerPenetration_m(makeSnap(0.2f), terrain), 0.f);
}

// ---------------------------------------------------------------------------
// minCornerClearance_m — signed clearance for hard-contact-latch release
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, MinCornerClearance_PositiveWhenAboveTerrain) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Bottom corners at 10 - 0.2 = 9.8 m above terrain — positive clearance.
    EXPECT_NEAR(c.minCornerClearance_m(makeSnap(10.f), terrain), 9.8f, 1e-3f);
}

TEST(BodyColliderTest, MinCornerClearance_NegativeWhenPenetrating) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Level aircraft at 0.1 m: bottom corners at -0.1 m → clearance = -0.1 m.
    // (maxCornerPenetration_m clamps this to +0.1; the signed clearance does not.)
    EXPECT_NEAR(c.minCornerClearance_m(makeSnap(0.1f), terrain), -0.1f, 1e-4f);
}

TEST(BodyColliderTest, MinCornerClearance_ZeroWhenJustTouching) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Bottom corners exactly at terrain level: clearance = 0.
    EXPECT_NEAR(c.minCornerClearance_m(makeSnap(0.2f), terrain), 0.f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Penetration cap — prevents explosive forces during extreme embedding
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, PenetrationCap_ForceIsBounded_UnderExtremeEmbedding) {
    // pen_cap = 2 * hz = 2 * 0.2 = 0.4 m
    // At alt = -10 m all 8 corners exceed the cap (pen ≈ 10 m each).
    // At alt = -100 m all 8 corners also exceed the cap (pen ≈ 100 m each).
    // With no damping and no velocity the force must be identical in both cases
    // because the cap clamps all effective penetrations to 0.4 m.
    // (Without the cap the forces would be 10× different.)
    BodyCollider c_deep, c_extreme;
    c_deep.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 0.f));
    c_extreme.initialize(makeConfig(0.5f, 0.3f, 0.2f, 10000.f, 0.f));

    const auto cf_deep    = c_deep.step(makeSnap(-10.0f));
    const auto cf_extreme = c_extreme.step(makeSnap(-100.0f));

    const float F_deep    = -cf_deep.force_body_n.z();
    const float F_extreme = -cf_extreme.force_body_n.z();
    ASSERT_GT(F_deep, 0.f) << "Contact must be detected at 10 m depth";
    EXPECT_NEAR(F_extreme, F_deep, 1.0f)
        << "Force must be equal at 10 m and 100 m depth — cap must clamp all corners";
}

// ---------------------------------------------------------------------------
// JSON round-trip
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, JsonRoundTrip) {
    BodyCollider original;
    original.initialize(makeConfig(0.4f, 0.25f, 0.15f, 8000.f, 400.f));

    const nlohmann::json j = original.serializeJson();
    ASSERT_EQ(j.at("schema_version").get<int>(), 1);
    ASSERT_EQ(j.at("volumes").size(), 1u);

    const auto& v0 = j.at("volumes").at(0);
    EXPECT_EQ(v0.at("name").get<std::string>(), "test");
    EXPECT_FLOAT_EQ(v0.at("stiffness_npm").get<float>(), 8000.f);
    EXPECT_FLOAT_EQ(v0.at("damping_nspm").get<float>(),  400.f);

    const auto& he = v0.at("half_extents_body_m");
    EXPECT_FLOAT_EQ(he.at(0).get<float>(), 0.4f);
    EXPECT_FLOAT_EQ(he.at(1).get<float>(), 0.25f);
    EXPECT_FLOAT_EQ(he.at(2).get<float>(), 0.15f);

    // Deserialize into a fresh instance and verify behaviour is identical.
    BodyCollider restored;
    restored.deserializeJson(j);
    FlatTerrain terrain{0.f};
    const auto cf_orig    = original.step(makeSnap(0.05f), terrain);
    const auto cf_restored = restored.step(makeSnap(0.05f), terrain);
    EXPECT_NEAR(cf_restored.force_body_n.z(), cf_orig.force_body_n.z(), 1e-3f);
}

// ---------------------------------------------------------------------------
// Proto round-trip
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ProtoRoundTrip) {
    BodyCollider original;
    original.initialize(makeConfig(0.4f, 0.25f, 0.15f, 8000.f, 400.f));

    const auto bytes = original.serializeProto();

    BodyCollider restored;
    restored.deserializeProto(bytes);

    FlatTerrain terrain{0.f};
    const auto cf_orig     = original.step(makeSnap(0.05f), terrain);
    const auto cf_restored = restored.step(makeSnap(0.05f), terrain);
    EXPECT_NEAR(cf_restored.force_body_n.z(), cf_orig.force_body_n.z(), 1e-3f);
}
