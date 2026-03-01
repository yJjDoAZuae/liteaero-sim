

#pragma once

#include "control/LimitBase.hpp"
#include "SISOBlock.hpp"

namespace liteaerosim::control {

class Limit : public LimitBase {

    public:

        Limit() : 
            _in(0),
            _out(0),
            _lowerLimit(0), 
            _upperLimit(0), 
            _limitedLower(false),
            _limitedUpper(false), 
            _enableLowerLimit(false), 
            _enableUpperLimit(false) 
            {}

        ~Limit() override {}

        float in() const override { return _in; }
        float out() const override { return _out; }
        operator float() const override { return out(); }

        // void disable() override { disableLower(); disableUpper(); }
        // void enable() override { enableLower(); enableUpper(); }
        void disableLower() override { _enableLowerLimit = false; step(_in); }
        void disableUpper() override { _enableUpperLimit = false; step(_in); }
        void enableLower() override { _enableLowerLimit = true; step(_in); }
        void enableUpper() override { _enableUpperLimit = true; step(_in); }
        void setLower(float lim) override;
        void setUpper(float lim) override;
        // void set(float lowerLim, float upperLim) { setLower(lowerLim); setUpper(upperLim); }

        float lowerLimit() const override { return _lowerLimit; };
        float upperLimit() const override { return _upperLimit; };
        bool isLimitedLower() const override { return _limitedLower; };
        bool isLimitedUpper() const override { return _limitedUpper; };
        // bool isLimited() const { return isLimitedLower() || isLimitedUpper(); };
        bool isLowerEnabled() const override { return _enableLowerLimit; }
        bool isUpperEnabled() const override { return _enableUpperLimit; }

        float step(float u) override;

    private:

        float _lowerLimit;
        float _upperLimit;
        bool _limitedLower;
        bool _limitedUpper;
        bool _enableLowerLimit;
        bool _enableUpperLimit;

        float _in;
        float _out;
};

}
