// ChannelRegistry — in-process ring-buffer pub/sub for real-time simulation output.
//
// Design authority: docs/architecture/ring_buffer.md

#include "runner/ChannelRegistry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// ChannelSubscriber
// ---------------------------------------------------------------------------

ChannelSubscriber::ChannelSubscriber(std::string  channel_name,
                                     size_t       capacity,
                                     ChannelRegistry* registry)
    : channel_name_(std::move(channel_name))
    , registry_(registry)
    , buffer_(capacity)
    , capacity_(capacity)
{}

ChannelSubscriber::~ChannelSubscriber()
{
    if (registry_) {
        registry_->unsubscribe(channel_name_, this);
    }
}

// ---------------------------------------------------------------------------

std::vector<Sample> ChannelSubscriber::drain()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (count_ == 0) {
        return {};
    }

    std::vector<Sample> result;
    result.reserve(count_);

    // Oldest sample is at (head_ - count_ + capacity_) % capacity_.
    size_t read_pos = (head_ + capacity_ - count_) % capacity_;
    for (size_t i = 0; i < count_; ++i) {
        result.push_back(buffer_[read_pos]);
        read_pos = (read_pos + 1) % capacity_;
    }

    head_  = 0;
    count_ = 0;
    return result;
}

// ---------------------------------------------------------------------------

void ChannelSubscriber::write(double time_s, float value)
{
    // Caller (ChannelRegistry::publish) holds registry_.mutex_.
    // Take our own mutex to serialise with drain().
    std::lock_guard<std::mutex> lock(mutex_);

    buffer_[head_] = {time_s, value};
    head_ = (head_ + 1) % capacity_;
    if (count_ < capacity_) {
        ++count_;
    }
    // If buffer is full, head_ has advanced past the oldest sample — it is
    // silently overwritten (overflow policy).
}

// ---------------------------------------------------------------------------
// ChannelRegistry
// ---------------------------------------------------------------------------

void ChannelRegistry::register_channel(const std::string& name,
                                       float sample_rate_hz,
                                       float depth_s)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (index_.count(name)) {
        return;  // idempotent
    }

    ChannelEntry entry;
    entry.name           = name;
    entry.sample_rate_hz = sample_rate_hz;
    entry.depth_s        = depth_s;

    index_[name] = entries_.size();
    entries_.push_back(std::move(entry));
}

// ---------------------------------------------------------------------------

std::vector<std::string> ChannelRegistry::available_channels() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> names;
    names.reserve(entries_.size());
    for (const auto& e : entries_) {
        names.push_back(e.name);
    }
    return names;
}

// ---------------------------------------------------------------------------

std::shared_ptr<ChannelSubscriber> ChannelRegistry::subscribe(const std::string& name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = index_.find(name);
    if (it == index_.end()) {
        throw std::out_of_range(
            "ChannelRegistry::subscribe: channel \"" + name + "\" is not registered");
    }

    ChannelEntry& entry = entries_[it->second];
    const size_t capacity = static_cast<size_t>(
        std::max(2.0f, std::ceil(entry.sample_rate_hz * entry.depth_s)));

    // Use a custom deleter so the shared_ptr does not delete via ChannelSubscriber*
    // (destructor calls unsubscribe, which takes the registry mutex — but we already
    // hold it here; this is only a risk if the caller immediately resets the shared_ptr
    // inside this lock, which the API contract forbids).
    auto sub = std::shared_ptr<ChannelSubscriber>(
        new ChannelSubscriber(name, capacity, this));

    entry.subscribers.push_back(sub.get());
    return sub;
}

// ---------------------------------------------------------------------------

void ChannelRegistry::publish(const std::string& name, double time_s, float value)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = index_.find(name);
    if (it == index_.end()) {
        return;  // unregistered channel — no-op
    }

    const ChannelEntry& entry = entries_[it->second];
    for (ChannelSubscriber* sub : entry.subscribers) {
        sub->write(time_s, value);
    }
}

// ---------------------------------------------------------------------------

void ChannelRegistry::unsubscribe(const std::string& name, ChannelSubscriber* sub)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = index_.find(name);
    if (it == index_.end()) {
        return;
    }

    ChannelEntry& entry = entries_[it->second];
    auto pos = std::find(entry.subscribers.begin(), entry.subscribers.end(), sub);
    if (pos != entry.subscribers.end()) {
        entry.subscribers.erase(pos);
    }
}

}  // namespace liteaero::simulation
