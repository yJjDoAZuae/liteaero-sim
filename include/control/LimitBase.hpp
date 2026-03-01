

#pragma once

#include "SISOBlock.hpp"

namespace liteaerosim::control {

class LimitBase : public liteaerosim::SISOBlock {

    public:

        LimitBase() {}

        virtual ~LimitBase() override {}

        virtual float in() const = 0;
        virtual float out() const = 0;
        virtual operator float() const = 0;

        virtual void disable() final { disableLower(); disableUpper(); }
        virtual void enable() final { enableLower(); enableUpper(); }
        virtual void disableLower() = 0;
        virtual void disableUpper() = 0;
        virtual void enableLower() = 0;
        virtual void enableUpper() = 0;
        virtual void setLower(float lim) = 0;
        virtual void setUpper(float lim) = 0;
        virtual void set(float lowerLim, float upperLim) final { setLower(lowerLim); setUpper(upperLim); }

        virtual float lowerLimit() const = 0;
        virtual float upperLimit() const = 0;
        virtual bool isLimitedLower() const = 0;
        virtual bool isLimitedUpper() const = 0;
        virtual bool isLimited() const final { return isLimitedLower() || isLimitedUpper(); };
        virtual bool isLowerEnabled() const = 0;
        virtual bool isUpperEnabled() const = 0;

        virtual float step(float u) = 0;
};

}
