#pragma once

// ChannelRegistry — in-process ring-buffer pub/sub for real-time simulation output.
//
// Design authority: docs/architecture/ring_buffer.md
// Roadmap item SB-2.  Decisions DR-5 and DR-6 in post_processing.md.

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Sample
// ---------------------------------------------------------------------------

// One datum returned by ChannelSubscriber::drain().
struct Sample {
    double time_s;  // simulation time of this sample (s)
    float  value;   // channel value in SI units
};

// ---------------------------------------------------------------------------
// ChannelSubscriber
// ---------------------------------------------------------------------------

class ChannelRegistry;  // forward declaration

// RAII subscription token returned by ChannelRegistry::subscribe().
//
// Owns a per-subscriber circular buffer.  The simulation thread writes via
// ChannelRegistry::publish(); callers drain on their own schedule.
//
// Non-copyable.  May be held as a shared_ptr — the registry retains a raw
// pointer to this object (non-owning) for the lifetime of the subscription.
class ChannelSubscriber {
    friend class ChannelRegistry;
public:
    // Removes this subscriber from the channel registry.
    ~ChannelSubscriber();

    ChannelSubscriber(const ChannelSubscriber&)            = delete;
    ChannelSubscriber& operator=(const ChannelSubscriber&) = delete;

    const std::string& channel_name() const { return channel_name_; }

    // Return all samples buffered since the last drain(), in chronological order.
    // Resets the internal buffer to empty.  Safe to call from any thread.
    // Returns an empty vector if no new samples have arrived.
    std::vector<Sample> drain();

private:
    // Constructed only by ChannelRegistry::subscribe().
    ChannelSubscriber(std::string channel_name,
                      size_t      capacity,
                      ChannelRegistry* registry);

    // Write one sample into the circular buffer.
    // Caller must hold registry_.mutex_ (called from ChannelRegistry::publish()).
    void write(double time_s, float value);

    std::string      channel_name_;
    ChannelRegistry* registry_;      // non-owning; valid for the lifetime of the subscription

    std::vector<Sample> buffer_;     // circular buffer storage
    size_t              capacity_;   // buffer_.size()
    size_t              head_  = 0;  // next write position
    size_t              count_ = 0;  // number of valid samples currently buffered

    std::mutex mutex_;  // protects buffer_, head_, count_
};

// ---------------------------------------------------------------------------
// ChannelRegistry
// ---------------------------------------------------------------------------

// Producer-driven channel registry with subscription-driven buffer allocation.
//
// SimRunner owns a ChannelRegistry and registers all channels it produces at
// initialize() time.  Python subscribers call subscribe() to attach.  SimRunner
// calls publish() on every simulation tick; the call is a fast no-op when no
// subscriber is attached.
//
// Thread safety: see docs/architecture/ring_buffer.md §Threading Contract.
class ChannelRegistry {
public:
    // Register a channel.  Idempotent: calling again with the same name is a no-op.
    //   name            — channel identifier, e.g. "kinematic/altitude_m"
    //   sample_rate_hz  — expected production rate; used to compute buffer capacity
    //   depth_s         — seconds of history retained per subscriber buffer
    void register_channel(const std::string& name,
                          float sample_rate_hz,
                          float depth_s);

    // List all registered channel names regardless of subscriber count (PP-F38).
    std::vector<std::string> available_channels() const;

    // Attach a subscriber to a registered channel.
    // The subscriber's buffer starts empty (PP-F37 — no backfill).
    // Throws std::out_of_range if the channel has not been registered.
    std::shared_ptr<ChannelSubscriber> subscribe(const std::string& name);

    // Publish one sample.  Called by SimRunner from the simulation thread.
    // No-op when the channel has no active subscribers (PP-F34).
    // No-op when the channel name is not registered.
    void publish(const std::string& name, double time_s, float value);

    // Called by ~ChannelSubscriber only.  Do not call directly.
    void unsubscribe(const std::string& name, ChannelSubscriber* sub);

private:
    struct ChannelEntry {
        std::string                    name;
        float                          sample_rate_hz;
        float                          depth_s;
        std::vector<ChannelSubscriber*> subscribers;  // non-owning raw pointers
    };

    mutable std::mutex                      mutex_;
    std::vector<ChannelEntry>               entries_;
    std::unordered_map<std::string, size_t> index_;  // name → entries_ index; O(1) lookup
};

}  // namespace liteaero::simulation
