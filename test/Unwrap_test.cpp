#define _USE_MATH_DEFINES
#include "control/Unwrap.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <cmath>

using namespace liteaerosim::control;

TEST(UnwrapTest, Instantiation00) {

    Unwrap U;

    EXPECT_EQ(U.in(), 0.0f);
    EXPECT_EQ(U.out(), 0.0f);

}

TEST(UnwrapTest, JsonRoundTrip) {
    Unwrap U;
    U.initialize({{"schema_version", 1}});
    U.step(0.1f);
    U.step(0.2f);
    // Simulate wrap: step past pi, should unwrap to > pi
    U.step(static_cast<float>(M_PI) - 0.1f);
    U.step(-static_cast<float>(M_PI) + 0.1f);  // wraps; unwrapped output should be > pi

    nlohmann::json snap = U.serializeJson();
    Unwrap U2;
    U2.initialize({{"schema_version", 1}});
    U2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(U2.step(0.5f), U.step(0.5f));
}

TEST(UnwrapTest, SchemaVersionMismatch) {
    Unwrap U;
    U.initialize({{"schema_version", 1}});
    nlohmann::json snap = U.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(U.deserializeJson(snap), std::runtime_error);
}
