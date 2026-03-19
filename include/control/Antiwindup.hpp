#pragma once

#include <nlohmann/json.hpp>

namespace liteaerosim::control {

struct AntiwindupConfig {
    enum class Direction { Null = 0, Negative = -1, Positive = 1 };

    Direction direction         = Direction::Null;
    float     limit             = 0.0f;
    bool      latch_on_direction = false;
};

class Antiwindup {
public:
    Antiwindup() = default;
    explicit Antiwindup(const AntiwindupConfig& config);

    // Set or replace config after default construction.
    void configure(const AntiwindupConfig& config);

    // Update detector with the current value of the monitored signal.
    // Returns isActive().
    bool update(float signal);

    // Query saturation status (valid after the most recent update() call).
    bool isActive()      const { return is_active_lower_ || is_active_upper_; }
    bool isActiveLower() const { return is_active_lower_; }
    bool isActiveUpper() const { return is_active_upper_; }

    // Clear saturation flags and reset previous signal. Does not change config.
    void reset();

    // Config accessors.
    AntiwindupConfig::Direction direction()        const { return config_.direction; }
    float                       limit()            const { return config_.limit; }
    bool                        latchOnDirection() const { return config_.latch_on_direction; }

    // Serialization — embedded by owner (Integrator), not standalone.
    nlohmann::json serializeJson()                        const;
    void           deserializeJson(const nlohmann::json& state);

private:
    AntiwindupConfig config_;
    float prev_signal_     = 0.0f;
    bool  is_active_lower_ = false;
    bool  is_active_upper_ = false;
};

} // namespace liteaerosim::control
