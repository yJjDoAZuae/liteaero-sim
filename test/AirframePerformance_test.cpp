#include "airframe/AirframePerformance.hpp"
#include "liteaerosim.pb.h"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using liteaerosim::AirframePerformance;

namespace {
AirframePerformance gaAirframe() {
    AirframePerformance a;
    a.g_max_nd    = 3.8f;
    a.g_min_nd    = -1.52f;
    a.tas_max_mps = 82.3f;
    a.mach_max_nd = 0.25f;
    return a;
}
}  // namespace

// ── Construction ──────────────────────────────────────────────────────────────

TEST(AirframePerformanceTest, DefaultConstructorZeroInitializes) {
    AirframePerformance a;
    EXPECT_FLOAT_EQ(a.g_max_nd,    0.0f);
    EXPECT_FLOAT_EQ(a.g_min_nd,    0.0f);
    EXPECT_FLOAT_EQ(a.tas_max_mps, 0.0f);
    EXPECT_FLOAT_EQ(a.mach_max_nd, 0.0f);
}

TEST(AirframePerformanceTest, FieldsAreReadable) {
    const AirframePerformance a = gaAirframe();
    EXPECT_FLOAT_EQ(a.g_max_nd,    3.8f);
    EXPECT_FLOAT_EQ(a.g_min_nd,   -1.52f);
    EXPECT_FLOAT_EQ(a.tas_max_mps, 82.3f);
    EXPECT_FLOAT_EQ(a.mach_max_nd, 0.25f);
}

// ── Serialization ─────────────────────────────────────────────────────────────

TEST(AirframePerformanceSerializationTest, JsonRoundTrip) {
    const AirframePerformance original = gaAirframe();
    const nlohmann::json j = original.serializeJson();
    const AirframePerformance restored = AirframePerformance::deserializeJson(j);
    EXPECT_FLOAT_EQ(restored.g_max_nd,    original.g_max_nd);
    EXPECT_FLOAT_EQ(restored.g_min_nd,    original.g_min_nd);
    EXPECT_FLOAT_EQ(restored.tas_max_mps, original.tas_max_mps);
    EXPECT_FLOAT_EQ(restored.mach_max_nd, original.mach_max_nd);
}

TEST(AirframePerformanceSerializationTest, JsonSchemaVersionMismatchThrows) {
    const AirframePerformance a = gaAirframe();
    nlohmann::json j = a.serializeJson();
    j["schema_version"] = 99;
    EXPECT_THROW(AirframePerformance::deserializeJson(j), std::runtime_error);
}

TEST(AirframePerformanceSerializationTest, ProtoRoundTrip) {
    const AirframePerformance original = gaAirframe();
    const std::vector<uint8_t> bytes = original.serializeProto();
    const AirframePerformance restored = AirframePerformance::deserializeProto(bytes);
    EXPECT_FLOAT_EQ(restored.g_max_nd,    original.g_max_nd);
    EXPECT_FLOAT_EQ(restored.g_min_nd,    original.g_min_nd);
    EXPECT_FLOAT_EQ(restored.tas_max_mps, original.tas_max_mps);
    EXPECT_FLOAT_EQ(restored.mach_max_nd, original.mach_max_nd);
}

TEST(AirframePerformanceSerializationTest, ProtoSchemaVersionMismatchThrows) {
    const AirframePerformance a = gaAirframe();
    std::vector<uint8_t> bytes = a.serializeProto();
    // Corrupt: modify schema_version field (field 1, varint, tag = 0x08) to value 99.
    for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] == 0x08) {
            bytes[i + 1] = 99;
            break;
        }
    }
    EXPECT_THROW(AirframePerformance::deserializeProto(bytes), std::runtime_error);
}
