// Tests for Logger, LogSource, and LogReader — roadmap item 1 (Logger subsystem).
// Covers: session lifecycle, round-trip value recovery, error on post-close log(),
// high-volume smoke test, multi-source registration, and source descriptor fidelity.

#include "logger/Logger.hpp"
#include "logger/LogReader.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace liteaerosim::logger;

namespace {

// Returns a unique temp-directory path for each test.
fs::path tempMcapPath(const std::string& stem) {
    return fs::temp_directory_path() / (stem + "_Logger_test.mcap");
}

}  // namespace

// ---------------------------------------------------------------------------
// T1: open creates a file; close flushes and closes without throwing.
// ---------------------------------------------------------------------------
TEST(LoggerTest, OpenAndClose_DoesNotThrow) {
    const auto path = tempMcapPath("T1");
    fs::remove(path);

    Logger logger;
    ASSERT_NO_THROW(logger.open(path));
    EXPECT_TRUE(logger.is_open());
    ASSERT_NO_THROW(logger.close());
    EXPECT_FALSE(logger.is_open());

    EXPECT_TRUE(fs::exists(path));
    fs::remove(path);
}

// ---------------------------------------------------------------------------
// T2: write two timesteps; LogReader recovers exact values for all channels.
// ---------------------------------------------------------------------------
TEST(LoggerTest, WriteAndRead_ValuesRoundTrip) {
    const auto path = tempMcapPath("T2");
    fs::remove(path);

    {
        Logger logger;
        logger.open(path);
        auto src = logger.addSource("state",
                                    {"vel_north_mps", "vel_east_mps", "altitude_m"},
                                    {"m/s", "m/s", "m"});
        src.log(0.0, {10.f, 0.f, 100.f});
        src.log(0.1, {10.5f, 0.1f, 99.8f});
        logger.close();
    }

    LogReader reader;
    reader.open(path);

    LogReader::Record r;
    int count = 0;
    while (reader.next(r)) {
        if (count == 0) {
            EXPECT_DOUBLE_EQ(r.time_s, 0.0);
            ASSERT_EQ(r.values.size(), 3u);
            EXPECT_FLOAT_EQ(r.values[0], 10.f);
            EXPECT_FLOAT_EQ(r.values[1],  0.f);
            EXPECT_FLOAT_EQ(r.values[2], 100.f);
        } else if (count == 1) {
            EXPECT_NEAR(r.time_s, 0.1, 1e-9);
            ASSERT_EQ(r.values.size(), 3u);
            EXPECT_FLOAT_EQ(r.values[0], 10.5f);
            EXPECT_FLOAT_EQ(r.values[1],  0.1f);
            EXPECT_FLOAT_EQ(r.values[2], 99.8f);
        }
        ++count;
    }
    EXPECT_EQ(count, 2);
    reader.close();
    fs::remove(path);
}

// ---------------------------------------------------------------------------
// T3: log() called after close() throws std::logic_error.
// ---------------------------------------------------------------------------
TEST(LoggerTest, LogAfterClose_ThrowsLogicError) {
    const auto path = tempMcapPath("T3");
    fs::remove(path);

    Logger logger;
    logger.open(path);
    auto src = logger.addSource("s", {"x"});
    logger.close();

    EXPECT_THROW(src.log(0.0, {1.f}), std::logic_error);
    fs::remove(path);
}

// ---------------------------------------------------------------------------
// T4: 10 000 consecutive log() calls — no crash, no OOM.
// ---------------------------------------------------------------------------
TEST(LoggerTest, HighVolume_NoMemoryGrowth) {
    const auto path = tempMcapPath("T4");
    fs::remove(path);

    Logger logger;
    logger.open(path);
    auto src = logger.addSource("telemetry",
                                {"a", "b", "c", "d", "e", "f", "g", "h"},
                                {"m/s", "m/s", "m/s", "m", "m", "m", "rad", "rad"});

    for (int i = 0; i < 10000; ++i) {
        const float t = static_cast<float>(i) * 0.01f;
        ASSERT_NO_THROW(src.log(static_cast<double>(i) * 0.01,
                                {t, t, t, t, t, t, t, t}));
    }
    ASSERT_NO_THROW(logger.close());
    fs::remove(path);
}

// ---------------------------------------------------------------------------
// T5: two sources registered — reader sees records from both.
// ---------------------------------------------------------------------------
TEST(LoggerTest, MultipleSources_AllChannelsPresent) {
    const auto path = tempMcapPath("T5");
    fs::remove(path);

    {
        Logger logger;
        logger.open(path);
        auto ac  = logger.addSource("aircraft", {"speed_mps"});
        auto ext = logger.addSource("radar",    {"range_m", "bearing_rad"});

        ac.log(0.0, {55.f});
        ext.log(0.0, {250.f, 0.5f});
        ac.log(0.1, {54.9f});
        logger.close();
    }

    LogReader reader;
    reader.open(path);

    int aircraft_count = 0;
    int radar_count    = 0;
    LogReader::Record r;
    while (reader.next(r)) {
        const auto& srcs = reader.sources();
        for (const auto& sd : srcs) {
            if (sd.channel_id == r.channel_id) {
                if (sd.name == "aircraft") ++aircraft_count;
                if (sd.name == "radar")    ++radar_count;
            }
        }
    }
    EXPECT_EQ(aircraft_count, 2);
    EXPECT_EQ(radar_count,    1);
    reader.close();
    fs::remove(path);
}

// ---------------------------------------------------------------------------
// T6: LogReader::sources() channel_names and channel_units match addSource().
// ---------------------------------------------------------------------------
TEST(LoggerTest, SourceDescriptors_MatchRegistration) {
    const auto path = tempMcapPath("T6");
    fs::remove(path);

    const std::vector<std::string> names = {"alpha_rad", "beta_rad", "mach_nd"};
    const std::vector<std::string> units = {"rad", "rad", "nd"};

    {
        Logger logger;
        logger.open(path);
        auto src = logger.addSource("aero", names, units);
        src.log(0.0, {0.05f, 0.01f, 0.3f});
        logger.close();
    }

    LogReader reader;
    reader.open(path);

    ASSERT_EQ(reader.sources().size(), 1u);
    const auto& sd = reader.sources().front();
    EXPECT_EQ(sd.name, "aero");
    EXPECT_EQ(sd.channel_names, names);
    EXPECT_EQ(sd.channel_units, units);
    reader.close();
    fs::remove(path);
}
