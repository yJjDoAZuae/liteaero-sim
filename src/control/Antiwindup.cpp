#include "control/Antiwindup.hpp"

using namespace liteaerosim::control;

Antiwindup::Antiwindup(const AntiwindupConfig& config) : config_(config) {}

void Antiwindup::configure(const AntiwindupConfig& config)
{
    config_ = config;
}

bool Antiwindup::update(float signal)
{
    is_active_lower_ = false;
    is_active_upper_ = false;

    float diff = signal - prev_signal_;
    prev_signal_ = signal;

    switch (config_.direction) {
    case AntiwindupConfig::Direction::Null:
        break;

    case AntiwindupConfig::Direction::Negative:
        is_active_lower_ = (signal < config_.limit)
                           && (!config_.latch_on_direction || diff < 0.0f);
        break;

    case AntiwindupConfig::Direction::Positive:
        is_active_upper_ = (signal > config_.limit)
                           && (!config_.latch_on_direction || diff > 0.0f);
        break;
    }

    return isActive();
}

void Antiwindup::reset()
{
    prev_signal_     = 0.0f;
    is_active_lower_ = false;
    is_active_upper_ = false;
}

nlohmann::json Antiwindup::serializeJson() const
{
    return {
        {"direction",          static_cast<int>(config_.direction)},
        {"limit",              config_.limit},
        {"latch_on_direction", config_.latch_on_direction},
        {"prev_signal",        prev_signal_},
        {"active_lower",       is_active_lower_},
        {"active_upper",       is_active_upper_}
    };
}

void Antiwindup::deserializeJson(const nlohmann::json& state)
{
    config_.direction         = static_cast<AntiwindupConfig::Direction>(
                                    state.at("direction").get<int>());
    config_.limit             = state.at("limit").get<float>();
    config_.latch_on_direction = state.at("latch_on_direction").get<bool>();
    prev_signal_               = state.at("prev_signal").get<float>();
    is_active_lower_           = state.at("active_lower").get<bool>();
    is_active_upper_           = state.at("active_upper").get<bool>();
}
