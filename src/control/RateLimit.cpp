#include "control/RateLimit.hpp"

using namespace liteaerosim::control;

void RateLimit::setLower(float lim)
{
    _lowerLimit = lim;
    _upperLimit = (_upperLimit >= lim) ? _upperLimit : lim;
}

void RateLimit::setUpper(float lim)
{
    _upperLimit = lim;
    _lowerLimit = (_lowerLimit <= lim) ? _lowerLimit : lim;
}

void RateLimit::resetTo(float u)
{
    in_  = u;
    out_ = u;
    _limitedLower = false;
    _limitedUpper = false;
}

float RateLimit::onStep(float u)
{
    _limitedLower = false;
    _limitedUpper = false;

    float delta = u - in_;
    float deltaLim = delta;

    if (isLowerEnabled() && delta < _dt * lowerLimit()) {
        deltaLim = _dt * lowerLimit();
        _limitedLower = true;
    }

    if (isUpperEnabled() && delta > _dt * upperLimit()) {
        deltaLim = _dt * upperLimit();
        _limitedUpper = true;
    }

    return out_ + deltaLim;
}

void RateLimit::onReset()
{
    _limitedLower = false;
    _limitedUpper = false;
}

void RateLimit::onInitialize(const nlohmann::json& config)
{
    if (config.contains("dt_s"))          setDt(config.at("dt_s").get<float>());
    if (config.contains("lower_limit"))   setLower(config.at("lower_limit").get<float>());
    if (config.contains("upper_limit"))   setUpper(config.at("upper_limit").get<float>());
    if (config.value("lower_enabled", false)) enableLower();
    if (config.value("upper_enabled", false)) enableUpper();
}

nlohmann::json RateLimit::onSerializeJson() const
{
    return {
        {"in",            in_},
        {"out",           out_},
        {"dt_s",          _dt},
        {"lower_limit",   _lowerLimit},
        {"upper_limit",   _upperLimit},
        {"lower_enabled", _enableLowerLimit},
        {"upper_enabled", _enableUpperLimit}
    };
}

void RateLimit::onDeserializeJson(const nlohmann::json& state)
{
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    _dt               = state.at("dt_s").get<float>();
    _lowerLimit       = state.at("lower_limit").get<float>();
    _upperLimit       = state.at("upper_limit").get<float>();
    _enableLowerLimit = state.at("lower_enabled").get<bool>();
    _enableUpperLimit = state.at("upper_enabled").get<bool>();
    _limitedLower     = false;
    _limitedUpper     = false;
}
