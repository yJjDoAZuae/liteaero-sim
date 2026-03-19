#include "control/Limit.hpp"

using namespace liteaerosim::control;

float Limit::onStep(float u)
{
    float out = u;
    _limitedLower = false;
    _limitedUpper = false;

    if (isLowerEnabled() && out < lowerLimit()) {
        out = lowerLimit();
        _limitedLower = true;
    }

    if (isUpperEnabled() && out > upperLimit()) {
        out = upperLimit();
        _limitedUpper = true;
    }

    return out;
}

void Limit::onReset()
{
    _limitedLower = false;
    _limitedUpper = false;
}

void Limit::setLower(float lim)
{
    _lowerLimit = lim;
    _upperLimit = (_upperLimit >= lim) ? _upperLimit : lim;
    step(in_);
}

void Limit::setUpper(float lim)
{
    _upperLimit = lim;
    _lowerLimit = (_lowerLimit <= lim) ? _lowerLimit : lim;
    step(in_);
}

void Limit::onInitialize(const nlohmann::json& config)
{
    if (config.contains("lower_limit")) setLower(config.at("lower_limit").get<float>());
    if (config.contains("upper_limit")) setUpper(config.at("upper_limit").get<float>());
    if (config.value("lower_enabled", false)) enableLower();
    if (config.value("upper_enabled", false)) enableUpper();
}

nlohmann::json Limit::onSerializeJson() const
{
    return {
        {"in",             in_},
        {"out",            out_},
        {"lower_limit",    _lowerLimit},
        {"upper_limit",    _upperLimit},
        {"lower_enabled",  _enableLowerLimit},
        {"upper_enabled",  _enableUpperLimit}
    };
}

void Limit::onDeserializeJson(const nlohmann::json& state)
{
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    _lowerLimit       = state.at("lower_limit").get<float>();
    _upperLimit       = state.at("upper_limit").get<float>();
    _enableLowerLimit = state.at("lower_enabled").get<bool>();
    _enableUpperLimit = state.at("upper_enabled").get<bool>();
    // Recompute limited flags from restored state
    _limitedLower = _enableLowerLimit && (in_ < _lowerLimit);
    _limitedUpper = _enableUpperLimit && (in_ > _upperLimit);
}
