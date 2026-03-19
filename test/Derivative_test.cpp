#define _USE_MATH_DEFINES
#include "control/Derivative.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace liteaerosim::control;

TEST(DerivativeTest, Instantiation00) {
    Derivative D;
    EXPECT_EQ(D.in(), 0.0f);
    EXPECT_EQ(D.out(), 0.0f);
}

TEST(DerivativeTest, Step00) {
    // Forward Euler derivative: out = (in - in_prev) / tau  (simplified; tau clamped to 4*dt)
    Derivative D;
    D.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"tau_s", 0.5f}, {"method", 1}});
    EXPECT_FLOAT_EQ(D.step(0.0f), 0.0f);
    float out = D.step(1.0f);
    EXPECT_GT(out, 0.0f);  // positive derivative for rising input
}

TEST(DerivativeTest, ResetTo00) {
    Derivative D;
    D.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"tau_s", 0.5f}, {"method", 1}});
    D.step(0.0f);
    D.step(1.0f);
    D.resetTo(3.0f, 0.0f);
    EXPECT_FLOAT_EQ(D.in(), 3.0f);
    EXPECT_FLOAT_EQ(D.out(), 0.0f);
}

TEST(DerivativeTest, JsonRoundTrip) {
    Derivative D;
    D.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"tau_s", 0.5f}, {"method", 1}});
    D.step(0.0f);
    D.step(1.0f);
    D.step(1.5f);

    nlohmann::json snap = D.serializeJson();
    Derivative D2;
    D2.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"tau_s", 0.5f}, {"method", 1}});
    D2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(D2.step(2.0f), D.step(2.0f));
}

TEST(DerivativeTest, SchemaVersionMismatch) {
    Derivative D;
    D.initialize({{"schema_version", 1}, {"dt_s", 0.1f}, {"tau_s", 0.5f}, {"method", 1}});
    nlohmann::json snap = D.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(D.deserializeJson(snap), std::runtime_error);
}
