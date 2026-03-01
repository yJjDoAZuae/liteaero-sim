#define _USE_MATH_DEFINES
#include "control/RateLimit.hpp"
#include <gtest/gtest.h>

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

