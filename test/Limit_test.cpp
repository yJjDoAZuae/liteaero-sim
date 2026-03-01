#define _USE_MATH_DEFINES
#include "control/Limit.hpp"
#include <gtest/gtest.h>

using namespace liteaerosim::control;

TEST(LimitTest, Instantiation00) {

    Limit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.lowerLimit(), 0.0f);
    EXPECT_EQ(L.upperLimit(), 0.0f);

}

TEST(LimitTest, Enable00) {

    Limit L;

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

TEST(LimitTest, Enable01) {

    Limit L;

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

TEST(LimitTest, Step00) {

    Limit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.step(1.0f), 1.0f);

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

}

TEST(LimitTest, Step01) {

    Limit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.step(1.0f), 1.0f);

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 1.0f);

    L.enableUpper();

    EXPECT_EQ(L.in(), 1.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), true);

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
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLimitedLower(), true);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disableLower();

    EXPECT_EQ(L.in(), -1.0f);
    EXPECT_EQ(L.out(), -1.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

}

TEST(LimitTest, Step02) {

    Limit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

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

TEST(LimitTest, Step03) {

    Limit L;

    EXPECT_EQ(L.in(), 0.0f);
    EXPECT_EQ(L.out(), 0.0f);

    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    L.enable();
    EXPECT_EQ(L.isLowerEnabled(), true);
    EXPECT_EQ(L.isUpperEnabled(), true);

    EXPECT_EQ(L.step(1.0f), 0.0f);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), true);

    EXPECT_EQ(L.step(-1.0f), 0.0f);

    EXPECT_EQ(L.isLimitedLower(), true);
    EXPECT_EQ(L.isLimitedUpper(), false);

    L.disable();
    EXPECT_EQ(L.isLowerEnabled(), false);
    EXPECT_EQ(L.isUpperEnabled(), false);

    EXPECT_EQ(L.isLimitedLower(), false);
    EXPECT_EQ(L.isLimitedUpper(), false);

    EXPECT_EQ(L.out(), -1.0f);

}

