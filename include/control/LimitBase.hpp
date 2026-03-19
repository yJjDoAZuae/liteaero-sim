#pragma once

#include "SisoElement.hpp"

namespace liteaerosim::control {

class LimitBase : public liteaerosim::SisoElement {
public:
    virtual void disableLower() = 0;
    virtual void disableUpper() = 0;
    virtual void enableLower()  = 0;
    virtual void enableUpper()  = 0;
    void disable() { disableLower(); disableUpper(); }
    void enable()  { enableLower();  enableUpper(); }

    virtual void setLower(float lim) = 0;
    virtual void setUpper(float lim) = 0;
    void set(float lowerLim, float upperLim) { setLower(lowerLim); setUpper(upperLim); }

    virtual float lowerLimit()     const = 0;
    virtual float upperLimit()     const = 0;
    virtual bool  isLimitedLower() const = 0;
    virtual bool  isLimitedUpper() const = 0;
    bool          isLimited()      const { return isLimitedLower() || isLimitedUpper(); }
    virtual bool  isLowerEnabled() const = 0;
    virtual bool  isUpperEnabled() const = 0;

protected:
    // Default no-op lifecycle hooks — concrete subclasses override with real implementations.
    void           onInitialize(const nlohmann::json&)      override {}
    nlohmann::json onSerializeJson()                const   override { return {}; }
    void           onDeserializeJson(const nlohmann::json&) override {}
    int            schemaVersion()                  const   override { return 0; }
    const char*    typeName()                       const   override { return "LimitBase"; }
    float          onStep(float u)                          override { return u; }
};

} // namespace liteaerosim::control
