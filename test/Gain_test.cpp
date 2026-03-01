#define _USE_MATH_DEFINES
#include "control/Gain.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace liteaerosim::control;

TEST(GainTest, Instantiation00) {

    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
}

TEST(GainTest, Read00) {

    Gain<float, 3> K;
    std::ifstream fs;

    fs.open("tmp.json");

    EXPECT_EQ(K.K(), 0.0f);

    std::stringstream ss;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        // tab.readJSON(ss);
        EXPECT_EQ(K.readJSON(ss), 0);
    }

}

TEST(GainTest, Set00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
    K = 5;
    EXPECT_EQ(K.K(), 5.0f);

    K = -5;
    EXPECT_EQ(K.K(), -5.0f);
}

TEST(GainTest, Multiply00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
    K = 5;
    EXPECT_EQ(K.K(), 5.0f);

    EXPECT_EQ(K*10.0f, 50.0f);
    EXPECT_EQ(10.0f*K, 50.0f);
    EXPECT_EQ(K*10.0, 50.0f);
    EXPECT_EQ(10.0*K, 50.0f);
    EXPECT_EQ(K*10, 50.0f);
    EXPECT_EQ(10*K, 50.0f);
}

TEST(GainTest, Divide00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
    K = 5;
    EXPECT_EQ(K.K(), 5.0f);

    EXPECT_EQ(K/10.0f, 0.5f);
    EXPECT_EQ(10.0f/K, 2.0f);
    EXPECT_EQ(K/10.0, 0.5f);
    EXPECT_EQ(10.0/K, 2.0f);
    EXPECT_EQ(K/10, 0.5f);
    EXPECT_EQ(10/K, 2.0f);
}

TEST(GainTest, Add00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
    K = 5;
    EXPECT_EQ(K.K(), 5.0f);

    EXPECT_EQ(K+10.0f, 15.0f);
    EXPECT_EQ(10.0f+K, 15.0f);
    EXPECT_EQ(K+10.0, 15.0f);
    EXPECT_EQ(10.0+K, 15.0f);
    EXPECT_EQ(K+10, 15.0f);
    EXPECT_EQ(10+K, 15.0f);
}

TEST(GainTest, Subtract00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.K(), 0.0f);
    K = 5;
    EXPECT_EQ(K.K(), 5.0f);

    EXPECT_EQ(K-10.0f, -5.0f);
    EXPECT_EQ(10.0f-K, 5.0f);
    EXPECT_EQ(K-10.0, -5.0f);
    EXPECT_EQ(10.0-K, 5.0f);
    EXPECT_EQ(K-10, -5.0f);
    EXPECT_EQ(10-K, 5.0f);
}
