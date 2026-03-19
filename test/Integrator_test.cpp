#define _USE_MATH_DEFINES
#include "control/Integrator.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace liteaerosim::control;

TEST(IntegratorTest, Instantiation00) {
    Integrator I;
    EXPECT_EQ(I.in(), 0.0f);
    EXPECT_EQ(I.out(), 0.0f);
}

TEST(IntegratorTest, Step00) {
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});
    // Forward Euler: out += in_prev * dt
    EXPECT_FLOAT_EQ(I.step(1.0f), 0.0f);   // first step: out += 0 * dt = 0
    EXPECT_FLOAT_EQ(I.step(1.0f), 0.1f);   // second step: out += 1 * dt
    EXPECT_FLOAT_EQ(I.step(1.0f), 0.2f);
}

TEST(IntegratorTest, ResetTo00) {
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});
    I.step(1.0f);
    I.step(1.0f);
    I.resetTo(5.0f);
    EXPECT_FLOAT_EQ(I.out(), 5.0f);
}

TEST(IntegratorTest, JsonRoundTrip) {
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.05f}, {"method", 0}});
    I.step(2.0f);
    I.step(2.0f);
    I.step(2.0f);

    nlohmann::json snap = I.serializeJson();
    Integrator I2;
    I2.initialize({{"schema_version", 1}, {"dt_s", 0.05f}, {"method", 0}});
    I2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(I2.step(2.0f), I.step(2.0f));
}

TEST(IntegratorTest, SchemaVersionMismatch) {
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});
    nlohmann::json snap = I.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(I.deserializeJson(snap), std::runtime_error);
}

TEST(IntegratorTest, Integrator_FreezesWhenAwActive) {
    using Dir = liteaerosim::control::AntiwindupConfig::Direction;
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});

    AntiwindupConfig cfg;
    cfg.direction         = Dir::Positive;
    cfg.limit             = 0.5f;
    cfg.latch_on_direction = false;
    I.aw.emplace_back(cfg);

    // AW inactive — integrator advances
    I.aw[0].update(0.0f);
    float before = I.step(1.0f);
    float after  = I.step(1.0f);
    EXPECT_GT(after, before);

    // AW active (signal above limit) — integrator holds
    I.aw[0].update(1.0f);
    float frozen = I.out();
    I.step(1.0f);
    EXPECT_FLOAT_EQ(I.out(), frozen);
}

TEST(IntegratorTest, Integrator_JsonRoundTrip_IncludesAw) {
    using Dir = liteaerosim::control::AntiwindupConfig::Direction;
    Integrator I;
    I.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});

    AntiwindupConfig cfg;
    cfg.direction         = Dir::Negative;
    cfg.limit             = -1.0f;
    cfg.latch_on_direction = true;
    I.aw.emplace_back(cfg);
    I.aw[0].update(-2.0f);   // prime
    I.aw[0].update(-2.5f);   // active: below limit and falling

    nlohmann::json snap = I.serializeJson();

    Integrator I2;
    I2.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"method", 0}});
    I2.deserializeJson(snap);

    ASSERT_EQ(I2.aw.size(), 1u);
    EXPECT_EQ(I2.aw[0].direction(), Dir::Negative);
    EXPECT_FLOAT_EQ(I2.aw[0].limit(), -1.0f);
    EXPECT_TRUE(I2.aw[0].latchOnDirection());
    EXPECT_TRUE(I2.aw[0].isActiveLower());
}
