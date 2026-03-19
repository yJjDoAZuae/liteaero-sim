#pragma once

#include "control/LimitBase.hpp"

namespace liteaerosim::control {

class Limit : public LimitBase {
public:
    Limit() :
        _lowerLimit(0.0f),
        _upperLimit(0.0f),
        _limitedLower(false),
        _limitedUpper(false),
        _enableLowerLimit(false),
        _enableUpperLimit(false)
    {}

    void disableLower() override { _enableLowerLimit = false; step(in_); }
    void disableUpper() override { _enableUpperLimit = false; step(in_); }
    void enableLower()  override { _enableLowerLimit = true;  step(in_); }
    void enableUpper()  override { _enableUpperLimit = true;  step(in_); }

    void setLower(float lim) override;
    void setUpper(float lim) override;

    float lowerLimit()     const override { return _lowerLimit; }
    float upperLimit()     const override { return _upperLimit; }
    bool  isLimitedLower() const override { return _limitedLower; }
    bool  isLimitedUpper() const override { return _limitedUpper; }
    bool  isLowerEnabled() const override { return _enableLowerLimit; }
    bool  isUpperEnabled() const override { return _enableUpperLimit; }

protected:
    float onStep(float u) override;
    void  onReset() override;
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Limit"; }

private:
    float _lowerLimit;
    float _upperLimit;
    bool  _limitedLower;
    bool  _limitedUpper;
    bool  _enableLowerLimit;
    bool  _enableUpperLimit;
};

} // namespace liteaerosim::control
