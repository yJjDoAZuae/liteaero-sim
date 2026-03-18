#pragma once

#include "SisoElement.hpp"

namespace liteaerosim::control {

/// Abstract base for standalone SISO limit/clamp elements.
///
/// Extends SisoElement with limit-specific query and control methods.
/// All concrete limiters (Limit, RateLimit, …) derive from this.
class LimitElement : public liteaerosim::SisoElement {
public:
    /// Enable both lower and upper limits.
    void enable() { enableLower(); enableUpper(); }

    /// Disable both lower and upper limits.
    void disable() { disableLower(); disableUpper(); }

    /// Set both limits in one call (lower must be ≤ upper).
    void set(float lower_limit, float upper_limit) {
        setLower(lower_limit);
        setUpper(upper_limit);
    }

    virtual void  enableLower()                  = 0;
    virtual void  enableUpper()                  = 0;
    virtual void  disableLower()                 = 0;
    virtual void  disableUpper()                 = 0;
    virtual void  setLower(float lower_limit)    = 0;
    virtual void  setUpper(float upper_limit)    = 0;

    virtual float lowerLimit()       const = 0;
    virtual float upperLimit()       const = 0;
    virtual bool  isLowerEnabled()   const = 0;
    virtual bool  isUpperEnabled()   const = 0;
    virtual bool  isLimitedLower()   const = 0;
    virtual bool  isLimitedUpper()   const = 0;

    /// True if either limit is active on the current output.
    bool isLimited() const { return isLimitedLower() || isLimitedUpper(); }
};

} // namespace liteaerosim::control
