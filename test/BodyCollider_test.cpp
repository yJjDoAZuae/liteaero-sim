// BodyCollider unit tests.
// Design authority: docs/design/body_collider.md

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

static nlohmann::json makeConfig(float hx = 0.5f, float hy = 0.3f, float hz = 0.2f)
{
    return {{"volumes", nlohmann::json::array({
        {
            {"name",                 "test"},
            {"half_extents_body_m",  {hx, hy, hz}},
            {"center_offset_body_m", {0.f, 0.f, 0.f}}
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

// A snapshot penetrating terrain while sinking straight down at `sink_mps`.
// The §5a velocity-arrest force is produced only by an active sink rate, so the
// force tests supply one (a static penetration produces no force by design).
static KinematicStateSnapshot makeSinkingSnap(
    float alt_m, float sink_mps,
    Eigen::Quaternionf q_nb = Eigen::Quaternionf::Identity())
{
    return makeSnap(alt_m, q_nb, Eigen::Vector3f{0.f, 0.f, sink_mps});
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

TEST(BodyColliderTest, Contact_SetsWeightOnWheels_EvenWhenStatic) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f));
    FlatTerrain terrain{0.f};

    // Bottom corners penetrate 0.1 m. Weight-on-wheels is set on any penetration,
    // but the §5a velocity-arrest force is zero without a sink rate (no spring).
    const auto cf = c.step(makeSnap(0.1f), terrain);
    EXPECT_TRUE(cf.weight_on_wheels);
    EXPECT_NEAR(cf.force_body_n.norm(), 0.f, 1e-6f);
}

TEST(BodyColliderTest, Contact_ProducesUpwardForce_WhenSinking) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f), /*mass_kg=*/1000.f, /*dt_s=*/0.01f);
    FlatTerrain terrain{0.f};

    // Penetrating 0.1 m and sinking at 2 m/s → upward (-body-z) arrest force.
    const auto cf = c.step(makeSinkingSnap(0.1f, 2.f), terrain);
    EXPECT_TRUE(cf.weight_on_wheels);
    EXPECT_GT(cf.force_body_n.norm(), 0.f);
    EXPECT_LT(cf.force_body_n.z(), 0.f);
}

// ---------------------------------------------------------------------------
// Force direction — level aircraft
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ForceIsUpward_BodyFrame_LevelAircraft) {
    BodyCollider c;
    c.initialize(makeConfig(), 1000.f, 0.01f);

    // Level aircraft sinking at 2 m/s, penetrating 0.1 m → force in -body-Z (up).
    const auto cf = c.step(makeSinkingSnap(0.1f, 2.f));
    EXPECT_LT(cf.force_body_n.z(), 0.f);
    // No lateral or fore-aft force from a vertical terrain normal.
    EXPECT_NEAR(cf.force_body_n.x(), 0.f, 1e-3f);
    EXPECT_NEAR(cf.force_body_n.y(), 0.f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Velocity-arrest force law: F = c * delta * delta_dot (§5a)
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ForceScalesWithPenetrationAndSinkRate) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);

    // Proportional to penetration at a fixed sink rate (0.15 / 0.10 = 1.5).
    const auto cf1 = c.step(makeSinkingSnap(0.1f,  2.f));   // pen = 0.10 m
    const auto cf2 = c.step(makeSinkingSnap(0.05f, 2.f));   // pen = 0.15 m
    const float F1 = -cf1.force_body_n.z();
    const float F2 = -cf2.force_body_n.z();
    ASSERT_GT(F1, 0.f);
    EXPECT_NEAR(F2 / F1, 1.5f, 0.05f) << "F proportional to penetration at fixed sink rate";

    // Proportional to sink rate at fixed penetration (2.0 / 1.0 = 2.0).
    const auto cf_slow = c.step(makeSinkingSnap(0.1f, 1.f));
    const auto cf_fast = c.step(makeSinkingSnap(0.1f, 2.f));
    EXPECT_NEAR(-cf_fast.force_body_n.z() / -cf_slow.force_body_n.z(), 2.f, 0.05f)
        << "F proportional to sink rate at fixed penetration";
}

TEST(BodyColliderTest, ForcePresentOnlyWhenSinking) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);

    const auto cf_static  = c.step(makeSnap(0.1f));               // no sink rate
    const auto cf_sinking = c.step(makeSinkingSnap(0.1f, 1.f));   // sinking

    EXPECT_NEAR(-cf_static.force_body_n.z(),  0.f, 1e-6f) << "No static restoring force (§5a)";
    EXPECT_GT(  -cf_sinking.force_body_n.z(), 0.f)        << "Arrest force present when sinking";
}

TEST(BodyColliderTest, NoForce_WhenRising_NoSuction) {
    // The dissipative force opposes sinking only; the max(0) floor forbids suction,
    // so a rising corner produces zero force.
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);

    const auto cf_rising = c.step(makeSinkingSnap(0.1f, -1.f));  // rising at 1 m/s
    const float F_rising = -cf_rising.force_body_n.z();
    EXPECT_NEAR(F_rising, 0.f, 1e-6f) << "No suction / no force when rising";
    EXPECT_GE(F_rising, 0.f);
}

TEST(BodyColliderTest, ArrestDampingScalesWithMass) {
    // §5a: the derived damping b_corner = m / (n_corners * N_arr * dt) scales with
    // airframe mass, so a 2x heavier airframe gets 2x the arrest force.
    BodyCollider light, heavy;
    light.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);
    heavy.initialize(makeConfig(0.5f, 0.3f, 0.2f), 2000.f, 0.01f);

    const float F_light = -light.step(makeSinkingSnap(0.1f, 2.f)).force_body_n.z();
    const float F_heavy = -heavy.step(makeSinkingSnap(0.1f, 2.f)).force_body_n.z();
    ASSERT_GT(F_light, 0.f);
    EXPECT_NEAR(F_heavy / F_light, 2.f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Inverted aircraft — protection for upside-down crash
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, Inverted_180DegRoll_UpperCornersContact) {
    BodyCollider c;
    c.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);

    // Roll 180° around body-X (forward axis): body-Z now points NED-up.
    // Aircraft center at alt = 0.1 m.
    // "Top" corners in body frame (body-z = -0.2) now map to NED-down = +0.2.
    // Corner altitude = 0.1 - 0.2 = -0.1 → penetration = 0.1 m → contact.
    const Eigen::Quaternionf q_inverted(
        Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitX()));

    // Sinking at 2 m/s (NED-down) so the penetrating upper corners are arrested.
    const auto cf = c.step(makeSinkingSnap(0.1f, 2.f, q_inverted));
    EXPECT_TRUE(cf.weight_on_wheels);
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
            {"center_offset_body_m", {0.3f, 0.f, 0.f}}
        }
    })}};
    BodyCollider c;
    c.initialize(cfg, 1000.f, 0.01f);

    // Forward-shifted box, sinking: off-center contact → pitching moment.
    const auto cf = c.step(makeSinkingSnap(0.1f, 2.f));
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
    // Effective penetration is capped at 2*hz = 0.4 m, so two very different embed
    // depths give the same force at the same sink rate (without the cap they would
    // differ 10x).
    BodyCollider c_deep, c_extreme;
    c_deep.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);
    c_extreme.initialize(makeConfig(0.5f, 0.3f, 0.2f), 1000.f, 0.01f);

    const auto cf_deep    = c_deep.step(makeSinkingSnap(-10.0f, 2.f));
    const auto cf_extreme = c_extreme.step(makeSinkingSnap(-100.0f, 2.f));

    const float F_deep    = -cf_deep.force_body_n.z();
    const float F_extreme = -cf_extreme.force_body_n.z();
    ASSERT_GT(F_deep, 0.f) << "Contact must be detected at 10 m depth";
    EXPECT_NEAR(F_extreme, F_deep, 1e-3f)
        << "Force must be equal at 10 m and 100 m depth — cap must clamp all corners";
}

// ---------------------------------------------------------------------------
// JSON round-trip
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, JsonRoundTrip) {
    BodyCollider original;
    original.initialize(makeConfig(0.4f, 0.25f, 0.15f), 1000.f, 0.01f);

    const nlohmann::json j = original.serializeJson();
    ASSERT_EQ(j.at("schema_version").get<int>(), 1);
    ASSERT_EQ(j.at("volumes").size(), 1u);

    const auto& v0 = j.at("volumes").at(0);
    EXPECT_EQ(v0.at("name").get<std::string>(), "test");

    const auto& he = v0.at("half_extents_body_m");
    EXPECT_FLOAT_EQ(he.at(0).get<float>(), 0.4f);
    EXPECT_FLOAT_EQ(he.at(1).get<float>(), 0.25f);
    EXPECT_FLOAT_EQ(he.at(2).get<float>(), 0.15f);

    // Deserialize into a fresh instance and verify behaviour is identical (the
    // derived arrest damping is serialized so the restored force matches).
    BodyCollider restored;
    restored.deserializeJson(j);
    FlatTerrain terrain{0.f};
    const auto cf_orig     = original.step(makeSinkingSnap(0.05f, 2.f), terrain);
    const auto cf_restored = restored.step(makeSinkingSnap(0.05f, 2.f), terrain);
    EXPECT_NEAR(cf_restored.force_body_n.z(), cf_orig.force_body_n.z(), 1e-3f);
    EXPECT_LT(cf_orig.force_body_n.z(), 0.f);  // force is actually exercised
}

// ---------------------------------------------------------------------------
// Proto round-trip
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, ProtoRoundTrip) {
    BodyCollider original;
    original.initialize(makeConfig(0.4f, 0.25f, 0.15f), 1000.f, 0.01f);

    const auto bytes = original.serializeProto();

    BodyCollider restored;
    restored.deserializeProto(bytes);

    FlatTerrain terrain{0.f};
    const auto cf_orig     = original.step(makeSinkingSnap(0.05f, 2.f), terrain);
    const auto cf_restored = restored.step(makeSinkingSnap(0.05f, 2.f), terrain);
    EXPECT_NEAR(cf_restored.force_body_n.z(), cf_orig.force_body_n.z(), 1e-3f);
    EXPECT_LT(cf_orig.force_body_n.z(), 0.f);
}

// ---------------------------------------------------------------------------
// restitution_nd — the single §5b user-facing contact parameter (OQ-BC-5)
// ---------------------------------------------------------------------------

TEST(BodyColliderTest, RestitutionDefaultsToZero) {
    BodyCollider c;
    c.initialize(makeConfig());  // no restitution_nd in config
    EXPECT_FLOAT_EQ(c.restitution_nd(), 0.f);
}

TEST(BodyColliderTest, RestitutionParsedAndClamped) {
    auto cfg = makeConfig();

    cfg["restitution_nd"] = 0.3f;
    BodyCollider a; a.initialize(cfg);
    EXPECT_FLOAT_EQ(a.restitution_nd(), 0.3f);

    cfg["restitution_nd"] = -0.2f;        // below range -> clamp to 0
    BodyCollider b; b.initialize(cfg);
    EXPECT_FLOAT_EQ(b.restitution_nd(), 0.f);

    cfg["restitution_nd"] = 1.5f;         // >= 1 -> clamp just below 1
    BodyCollider d; d.initialize(cfg);
    EXPECT_LT(d.restitution_nd(), 1.f);
    EXPECT_GT(d.restitution_nd(), 0.99f);
}

TEST(BodyColliderTest, RestitutionRoundTrips) {
    auto cfg = makeConfig();
    cfg["restitution_nd"] = 0.25f;
    BodyCollider original; original.initialize(cfg);

    BodyCollider from_json; from_json.deserializeJson(original.serializeJson());
    EXPECT_FLOAT_EQ(from_json.restitution_nd(), 0.25f);

    BodyCollider from_proto; from_proto.deserializeProto(original.serializeProto());
    EXPECT_FLOAT_EQ(from_proto.restitution_nd(), 0.25f);
}
