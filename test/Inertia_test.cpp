#include "airframe/Inertia.hpp"
#include "liteaerosim.pb.h"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using liteaerosim::Inertia;

namespace {
Inertia gaInertia() {
    Inertia i;
    i.mass_kg  = 1045.0f;
    i.Ixx_kgm2 = 1285.0f;
    i.Iyy_kgm2 = 1825.0f;
    i.Izz_kgm2 = 2667.0f;
    return i;
}
}  // namespace

// ── Construction ──────────────────────────────────────────────────────────────

TEST(InertiaTest, DefaultConstructorZeroInitializes) {
    Inertia i;
    EXPECT_FLOAT_EQ(i.mass_kg,  0.0f);
    EXPECT_FLOAT_EQ(i.Ixx_kgm2, 0.0f);
    EXPECT_FLOAT_EQ(i.Iyy_kgm2, 0.0f);
    EXPECT_FLOAT_EQ(i.Izz_kgm2, 0.0f);
}

TEST(InertiaTest, FieldsAreReadable) {
    const Inertia i = gaInertia();
    EXPECT_FLOAT_EQ(i.mass_kg,   1045.0f);
    EXPECT_FLOAT_EQ(i.Ixx_kgm2,  1285.0f);
    EXPECT_FLOAT_EQ(i.Iyy_kgm2,  1825.0f);
    EXPECT_FLOAT_EQ(i.Izz_kgm2,  2667.0f);
}

// ── Serialization ─────────────────────────────────────────────────────────────

TEST(InertiaSerializationTest, JsonRoundTrip) {
    const Inertia original = gaInertia();
    const nlohmann::json j = original.serializeJson();
    const Inertia restored = Inertia::deserializeJson(j);
    EXPECT_FLOAT_EQ(restored.mass_kg,  original.mass_kg);
    EXPECT_FLOAT_EQ(restored.Ixx_kgm2, original.Ixx_kgm2);
    EXPECT_FLOAT_EQ(restored.Iyy_kgm2, original.Iyy_kgm2);
    EXPECT_FLOAT_EQ(restored.Izz_kgm2, original.Izz_kgm2);
}

TEST(InertiaSerializationTest, JsonSchemaVersionMismatchThrows) {
    const Inertia i = gaInertia();
    nlohmann::json j = i.serializeJson();
    j["schema_version"] = 99;
    EXPECT_THROW(Inertia::deserializeJson(j), std::runtime_error);
}

TEST(InertiaSerializationTest, ProtoRoundTrip) {
    const Inertia original = gaInertia();
    const std::vector<uint8_t> bytes = original.serializeProto();
    const Inertia restored = Inertia::deserializeProto(bytes);
    EXPECT_FLOAT_EQ(restored.mass_kg,  original.mass_kg);
    EXPECT_FLOAT_EQ(restored.Ixx_kgm2, original.Ixx_kgm2);
    EXPECT_FLOAT_EQ(restored.Iyy_kgm2, original.Iyy_kgm2);
    EXPECT_FLOAT_EQ(restored.Izz_kgm2, original.Izz_kgm2);
}

TEST(InertiaSerializationTest, ProtoSchemaVersionMismatchThrows) {
    const Inertia i = gaInertia();
    std::vector<uint8_t> bytes = i.serializeProto();
    // Corrupt: modify schema_version field (field 1, varint, tag = 0x08) to value 99.
    for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] == 0x08) {
            bytes[i + 1] = 99;
            break;
        }
    }
    EXPECT_THROW(Inertia::deserializeProto(bytes), std::runtime_error);
}
