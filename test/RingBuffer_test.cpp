// Tests for ChannelRegistry and ChannelSubscriber.
// Design authority: docs/architecture/ring_buffer.md
// Roadmap item SB-2.

#include "runner/ChannelRegistry.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <thread>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, EmptyRegistryReturnsNoChannels)
{
    ChannelRegistry reg;
    EXPECT_TRUE(reg.available_channels().empty());
}

TEST(ChannelRegistryTest, RegisteredChannelAppearsInAvailableChannels)
{
    ChannelRegistry reg;
    reg.register_channel("kinematic/altitude_m", 50.0f, 10.0f);
    const auto names = reg.available_channels();
    ASSERT_EQ(names.size(), 1u);
    EXPECT_EQ(names[0], "kinematic/altitude_m");
}

TEST(ChannelRegistryTest, RegisterChannelIsIdempotent)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    reg.register_channel("ch/a", 50.0f, 10.0f);  // second call — no-op
    EXPECT_EQ(reg.available_channels().size(), 1u);
}

TEST(ChannelRegistryTest, MultipleChannelsAllAppear)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    reg.register_channel("ch/b", 50.0f, 10.0f);
    reg.register_channel("ch/c", 50.0f, 10.0f);
    const auto names = reg.available_channels();
    EXPECT_EQ(names.size(), 3u);
}

// ---------------------------------------------------------------------------
// Subscribe
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, SubscribeToUnregisteredChannelThrows)
{
    ChannelRegistry reg;
    EXPECT_THROW(reg.subscribe("nonexistent"), std::out_of_range);
}

TEST(ChannelRegistryTest, SubscribeReturnsNonNull)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    auto sub = reg.subscribe("ch/a");
    EXPECT_NE(sub, nullptr);
}

TEST(ChannelRegistryTest, SubscriberChannelNameMatchesRequest)
{
    ChannelRegistry reg;
    reg.register_channel("kinematic/altitude_m", 50.0f, 10.0f);
    auto sub = reg.subscribe("kinematic/altitude_m");
    EXPECT_EQ(sub->channel_name(), "kinematic/altitude_m");
}

// ---------------------------------------------------------------------------
// Write and drain
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, DrainBeforeAnyPublishReturnsEmpty)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    auto sub = reg.subscribe("ch/a");
    EXPECT_TRUE(sub->drain().empty());
}

TEST(ChannelRegistryTest, PublishThenDrainReturnsSample)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    auto sub = reg.subscribe("ch/a");

    reg.publish("ch/a", 1.0, 42.0f);
    const auto samples = sub->drain();

    ASSERT_EQ(samples.size(), 1u);
    EXPECT_DOUBLE_EQ(samples[0].time_s, 1.0);
    EXPECT_FLOAT_EQ(samples[0].value, 42.0f);
}

TEST(ChannelRegistryTest, SamplesReturnedInChronologicalOrder)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    auto sub = reg.subscribe("ch/a");

    reg.publish("ch/a", 0.02, 1.0f);
    reg.publish("ch/a", 0.04, 2.0f);
    reg.publish("ch/a", 0.06, 3.0f);

    const auto samples = sub->drain();
    ASSERT_EQ(samples.size(), 3u);
    EXPECT_FLOAT_EQ(samples[0].value, 1.0f);
    EXPECT_FLOAT_EQ(samples[1].value, 2.0f);
    EXPECT_FLOAT_EQ(samples[2].value, 3.0f);
}

TEST(ChannelRegistryTest, DrainResetsBuffer)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    auto sub = reg.subscribe("ch/a");

    reg.publish("ch/a", 1.0, 10.0f);
    sub->drain();  // first drain
    EXPECT_TRUE(sub->drain().empty());  // second drain — buffer reset
}

TEST(ChannelRegistryTest, PublishToUnregisteredChannelIsNoOp)
{
    ChannelRegistry reg;
    // No registration, no subscribers — should not throw.
    EXPECT_NO_THROW(reg.publish("nonexistent", 1.0, 42.0f));
}

TEST(ChannelRegistryTest, PublishWithNoSubscribersIsNoOp)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);
    // No subscriber attached — should not throw.
    EXPECT_NO_THROW(reg.publish("ch/a", 1.0, 42.0f));
}

// ---------------------------------------------------------------------------
// Buffer overflow — oldest sample overwritten
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, BufferOverflowRetainsNewestSamples)
{
    // capacity = ceil(10 Hz * 1 s) = 10 samples
    ChannelRegistry reg;
    reg.register_channel("ch/a", 10.0f, 1.0f);
    auto sub = reg.subscribe("ch/a");

    // Publish 15 samples — first 5 should be dropped.
    for (int i = 0; i < 15; ++i) {
        reg.publish("ch/a", static_cast<double>(i) * 0.1, static_cast<float>(i));
    }

    const auto samples = sub->drain();
    ASSERT_EQ(samples.size(), 10u);
    // Newest 10: values 5..14
    EXPECT_FLOAT_EQ(samples.front().value, 5.0f);
    EXPECT_FLOAT_EQ(samples.back().value, 14.0f);
}

// ---------------------------------------------------------------------------
// Late-join — no backfill (PP-F37)
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, LateJoinSubscriberStartsWithEmptyBuffer)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);

    // Publish before any subscriber exists.
    for (int i = 0; i < 5; ++i) {
        reg.publish("ch/a", static_cast<double>(i) * 0.02, static_cast<float>(i));
    }

    // Subscribe after publishing — must see empty buffer.
    auto sub = reg.subscribe("ch/a");
    EXPECT_TRUE(sub->drain().empty());

    // Subsequent publishes must be received.
    reg.publish("ch/a", 0.10, 99.0f);
    const auto samples = sub->drain();
    ASSERT_EQ(samples.size(), 1u);
    EXPECT_FLOAT_EQ(samples[0].value, 99.0f);
}

// ---------------------------------------------------------------------------
// Multiple subscribers (PP-F36)
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, MultipleSubscribersReceiveIndependentCopies)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);

    auto sub1 = reg.subscribe("ch/a");
    auto sub2 = reg.subscribe("ch/a");

    reg.publish("ch/a", 0.02, 7.0f);

    // sub1 drains — removes from sub1 buffer only.
    const auto s1 = sub1->drain();
    ASSERT_EQ(s1.size(), 1u);
    EXPECT_FLOAT_EQ(s1[0].value, 7.0f);

    // sub2 still has the sample.
    const auto s2 = sub2->drain();
    ASSERT_EQ(s2.size(), 1u);
    EXPECT_FLOAT_EQ(s2[0].value, 7.0f);
}

TEST(ChannelRegistryTest, SubscribersAccumulateIndependently)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);

    auto sub1 = reg.subscribe("ch/a");
    reg.publish("ch/a", 0.02, 1.0f);

    // sub2 joins later — starts empty.
    auto sub2 = reg.subscribe("ch/a");
    reg.publish("ch/a", 0.04, 2.0f);

    const auto s1 = sub1->drain();
    const auto s2 = sub2->drain();

    ASSERT_EQ(s1.size(), 2u);  // saw both publishes
    ASSERT_EQ(s2.size(), 1u);  // only saw publish after subscribe
    EXPECT_FLOAT_EQ(s2[0].value, 2.0f);
}

// ---------------------------------------------------------------------------
// Subscriber deallocation removes it from channel (PP-F36)
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, DestroyedSubscriberIsRemovedFromChannel)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 50.0f, 10.0f);

    {
        auto sub = reg.subscribe("ch/a");
        reg.publish("ch/a", 0.02, 1.0f);
        // sub goes out of scope here — destructor must unsubscribe cleanly.
    }

    // Publish after subscriber is gone — must not crash.
    EXPECT_NO_THROW(reg.publish("ch/a", 0.04, 2.0f));
}

// ---------------------------------------------------------------------------
// Thread safety — concurrent publish and drain
// ---------------------------------------------------------------------------

TEST(ChannelRegistryTest, ConcurrentPublishAndDrainDoNotDataRace)
{
    ChannelRegistry reg;
    reg.register_channel("ch/a", 1000.0f, 5.0f);  // 5000-sample buffer
    auto sub = reg.subscribe("ch/a");

    constexpr int kPublishes = 2000;
    std::atomic<int> drain_total{0};

    // Producer thread.
    std::thread producer([&] {
        for (int i = 0; i < kPublishes; ++i) {
            reg.publish("ch/a", static_cast<double>(i) * 0.001,
                        static_cast<float>(i));
        }
    });

    // Consumer thread drains repeatedly.
    std::thread consumer([&] {
        int received = 0;
        while (received < kPublishes) {
            auto samples = sub->drain();
            received += static_cast<int>(samples.size());
        }
        drain_total.store(received);
    });

    producer.join();
    consumer.join();

    EXPECT_GE(drain_total.load(), kPublishes);
}
