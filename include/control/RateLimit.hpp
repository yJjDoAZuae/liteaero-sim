#pragma once

#include "control/LimitBase.hpp"

namespace liteaerosim::control {

class RateLimit : public LimitBase {
public:
    RateLimit() :
        _lowerLimit(0.0f),
        _upperLimit(0.0f),
        _limitedLower(false),
        _limitedUpper(false),
        _enableLowerLimit(false),
        _enableUpperLimit(false),
        _dt(1.0f)
    {}

    void disableLower() override { _enableLowerLimit = false; }
    void disableUpper() override { _enableUpperLimit = false; }
    void enableLower()  override { _enableLowerLimit = true; }
    void enableUpper()  override { _enableUpperLimit = true; }

    void setLower(float lim) override;
    void setUpper(float lim) override;
    void setDt(float dt) { _dt = (dt > 1e-6f) ? dt : 1.0f; }

    float lowerLimit()     const override { return _lowerLimit; }
    float upperLimit()     const override { return _upperLimit; }
    bool  isLimitedLower() const override { return _limitedLower; }
    bool  isLimitedUpper() const override { return _limitedUpper; }
    bool  isLowerEnabled() const override { return _enableLowerLimit; }
    bool  isUpperEnabled() const override { return _enableUpperLimit; }
    float dt()             const { return _dt; }

    /// Warm-start: set output (and input) to u without stepping.
    void resetTo(float u);

protected:
    float onStep(float u) override;
    void  onReset() override;
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "RateLimit"; }

private:
    float _lowerLimit;
    float _upperLimit;
    bool  _limitedLower;
    bool  _limitedUpper;
    bool  _enableLowerLimit;
    bool  _enableUpperLimit;
    float _dt;
};

} // namespace liteaerosim::control
