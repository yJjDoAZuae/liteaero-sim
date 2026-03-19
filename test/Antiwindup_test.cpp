#include "control/Antiwindup.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace liteaerosim::control;
using Dir = AntiwindupConfig::Direction;

// ── NullDirection ────────────────────────────────────────────────────────────

TEST(AntiwindupTest, NullDirection_NeverActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction = Dir::Null;
    cfg.limit     = 0.0f;
    aw.configure(cfg);

    EXPECT_FALSE(aw.update(-1000.0f));
    EXPECT_FALSE(aw.update( 0.0f));
    EXPECT_FALSE(aw.update( 1000.0f));
    EXPECT_FALSE(aw.isActive());
}

// ── Positive direction ───────────────────────────────────────────────────────

TEST(AntiwindupTest, PositiveDirection_BelowLimit_NotActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    EXPECT_FALSE(aw.update(0.5f));
    EXPECT_FALSE(aw.isActiveUpper());
}

TEST(AntiwindupTest, PositiveDirection_AboveLimit_Active) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    EXPECT_TRUE(aw.update(1.5f));
    EXPECT_TRUE(aw.isActiveUpper());
    EXPECT_FALSE(aw.isActiveLower());
}

// ── Negative direction ───────────────────────────────────────────────────────

TEST(AntiwindupTest, NegativeDirection_AboveLimit_NotActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Negative;
    cfg.limit             = -1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    EXPECT_FALSE(aw.update(0.5f));
    EXPECT_FALSE(aw.isActiveLower());
}

TEST(AntiwindupTest, NegativeDirection_BelowLimit_Active) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Negative;
    cfg.limit             = -1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    EXPECT_TRUE(aw.update(-1.5f));
    EXPECT_TRUE(aw.isActiveLower());
    EXPECT_FALSE(aw.isActiveUpper());
}

// ── latch_on_direction = true ────────────────────────────────────────────────

TEST(AntiwindupTest, LatchOnDirection_Positive_RisingActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = true;
    aw.configure(cfg);

    aw.update(1.0f);   // prime prev_signal to 1.0
    EXPECT_TRUE(aw.update(1.5f));  // above limit and rising
    EXPECT_TRUE(aw.isActiveUpper());
}

TEST(AntiwindupTest, LatchOnDirection_Positive_FallingNotActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = true;
    aw.configure(cfg);

    aw.update(2.0f);   // prime prev_signal to 2.0 (above limit)
    EXPECT_FALSE(aw.update(1.5f));  // above limit but falling
    EXPECT_FALSE(aw.isActive());
}

TEST(AntiwindupTest, LatchOnDirection_Negative_FallingActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Negative;
    cfg.limit             = -1.0f;
    cfg.latch_on_direction = true;
    aw.configure(cfg);

    aw.update(-1.0f);   // prime prev_signal to -1.0
    EXPECT_TRUE(aw.update(-1.5f));  // below limit and falling
    EXPECT_TRUE(aw.isActiveLower());
}

TEST(AntiwindupTest, LatchOnDirection_Negative_RisingNotActive) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Negative;
    cfg.limit             = -1.0f;
    cfg.latch_on_direction = true;
    aw.configure(cfg);

    aw.update(-2.0f);   // prime prev_signal to -2.0 (below limit)
    EXPECT_FALSE(aw.update(-1.5f));  // below limit but rising
    EXPECT_FALSE(aw.isActive());
}

// ── reset() ──────────────────────────────────────────────────────────────────

TEST(AntiwindupTest, Reset_ClearsFlagsNotConfig) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    aw.update(2.0f);
    EXPECT_TRUE(aw.isActive());

    aw.reset();
    EXPECT_FALSE(aw.isActive());
    // Config preserved
    EXPECT_EQ(aw.direction(), Dir::Positive);
    EXPECT_FLOAT_EQ(aw.limit(), 1.0f);
    EXPECT_FALSE(aw.latchOnDirection());
}

// ── configure() ──────────────────────────────────────────────────────────────

TEST(AntiwindupTest, Configure_UpdatesConfig) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 1.0f;
    cfg.latch_on_direction = false;
    aw.configure(cfg);

    aw.update(2.0f);
    EXPECT_TRUE(aw.isActive());

    AntiwindupConfig cfg2;
    cfg2.direction         = Dir::Negative;
    cfg2.limit             = -0.5f;
    cfg2.latch_on_direction = true;
    aw.configure(cfg2);

    EXPECT_EQ(aw.direction(), Dir::Negative);
    EXPECT_FLOAT_EQ(aw.limit(), -0.5f);
    EXPECT_TRUE(aw.latchOnDirection());
}

// ── JSON round-trip ───────────────────────────────────────────────────────────

TEST(AntiwindupTest, JsonRoundTrip_PreservesStateAndConfig) {
    Antiwindup aw;
    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 0.5f;
    cfg.latch_on_direction = true;
    aw.configure(cfg);

    aw.update(0.3f);   // prime prev_signal; below limit, not active
    aw.update(0.8f);   // above limit and rising -> active

    EXPECT_TRUE(aw.isActive());

    nlohmann::json snap = aw.serializeJson();

    Antiwindup aw2;
    aw2.deserializeJson(snap);

    EXPECT_EQ(aw2.direction(), Dir::Positive);
    EXPECT_FLOAT_EQ(aw2.limit(), 0.5f);
    EXPECT_TRUE(aw2.latchOnDirection());
    EXPECT_TRUE(aw2.isActiveUpper());
    EXPECT_FALSE(aw2.isActiveLower());
}
