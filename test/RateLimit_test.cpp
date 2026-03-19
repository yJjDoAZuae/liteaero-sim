#define _USE_MATH_DEFINES
#include "control/RateLimit.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace liteaerosim::control;

TEST(RateLimitTest, Instantiation00) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.lowerLimit(), 0.0f);
    EXPECT_EQ(L.upperLimit(), 0.0f);

}

TEST(RateLimitTest, Enable00) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.enableLower();
    EXPECT_EQ(L.isLowerEnabled(), true);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.disableLower();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.enableUpper();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), true);

    L.disableUpper();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

}

TEST(RateLimitTest, Enable01) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.enable();
    EXPECT_EQ(L.isLowerEnabled(), true);
    EXPECT_EQ(L.isUpperEnabled(), true);

    L.disable();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

}

TEST(RateLimitTest, Step00) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.step(1.0f), 1.0f);

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

}

TEST(RateLimitTest, Step01) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    L.setDt(0.1);
    L.setLower(-1.0f);
    L.setUpper(1.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.step(1.0f), 1.0f);

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    L.enableUpper();

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disableUpper();

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.step(-1.0f), -1.0f);

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    L.enableLower();

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disableLower();

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

}

TEST(RateLimitTest, Step02) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    L.setDt(0.1);
    L.setLower(-1.0f);
    L.setUpper(1.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.step(-1.0f), -1.0f);

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    L.enableUpper();

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disableUpper();

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.step(1.0f), 1.0f);

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    L.enableLower();

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disableLower();

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

}

TEST(RateLimitTest, Step03) {

    RateLimit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    L.setDt(0.1);
    L.setLower(-1.0f);
    L.setUpper(1.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.enable();
    EXPECT_EQ(L.isLowerEnabled(), true);
    EXPECT_EQ(L.isUpperEnabled(), true);

    EXPECT_EQ(L.step(1.0f), 0.1f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), true);

    EXPECT_EQ(L.step(-1.0f), 0.0f);

    EXPECT_EQ(L.isLimitedLower(), true);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disable();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.isLimitedLower(), true);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.out(), 0.0f);

}

TEST(RateLimitTest, JsonRoundTrip) {
    RateLimit L;
    L.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"lower_limit", -5.0f},
                  {"upper_limit", 5.0f}, {"lower_enabled", true}, {"upper_enabled", true}});
    L.step(1.0f);
    L.step(1.0f);

    nlohmann::json snap = L.serializeJson();
    RateLimit L2;
    L2.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"lower_limit", -5.0f},
                   {"upper_limit", 5.0f}, {"lower_enabled", true}, {"upper_enabled", true}});
    L2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(L2.step(1.0f), L.step(1.0f));
    EXPECT_FLOAT_EQ(L2.dt(), L.dt());
    EXPECT_FLOAT_EQ(L2.lowerLimit(), L.lowerLimit());
    EXPECT_FLOAT_EQ(L2.upperLimit(), L.upperLimit());
}

TEST(RateLimitTest, SchemaVersionMismatch) {
    RateLimit L;
    L.initialize({{"schema_version", 1}});
    nlohmann::json snap = L.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(L.deserializeJson(snap), std::runtime_error);
}
