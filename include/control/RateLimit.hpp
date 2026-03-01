

#pragma once

#include "control/LimitBase.hpp"

namespace liteaerosim::control {

class RateLimit : public LimitBase {

    public:

        RateLimit() : 
            _in(0),
            _out(0),
            _lowerLimit(0), 
            _upperLimit(0), 
            _limitedLower(false),
            _limitedUpper(false), 
            _enableLowerLimit(false), 
            _enableUpperLimit(false),
            _dt(1.0f) {}

        RateLimit(const RateLimit& lim) : _dt(lim._dt) {}

        ~RateLimit() override {}

        float in() const override { return _in; }
        float out() const override { return _out; }
        operator float() const override { return out(); }

        // void disable() override { disableLower(); disableUpper(); }  // make sure we call the RateLimit version
        // void enable() override { enableLower(); enableUpper(); } // make sure we call the RateLimit version
        void disableLower() override { _enableLowerLimit = false; }
        void disableUpper() override { _enableUpperLimit = false; }
        void enableLower() override { _enableLowerLimit = true; }
        void enableUpper() override { _enableUpperLimit = true; }
        void setLower(float lim) override;
        void setUpper(float lim) override;
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };

        float lowerLimit() const override { return _lowerLimit; };
        float upperLimit() const override { return _upperLimit; };
        bool isLimitedLower() const override { return _limitedLower; };
        bool isLimitedUpper() const override { return _limitedUpper; };
        // bool isLimited() const { return isLimitedLower() || isLimitedUpper(); };
        bool isLowerEnabled() const override { return _enableLowerLimit; }
        bool isUpperEnabled() const override { return _enableUpperLimit; }

        void reset(float u);
        float step(float u) override;

        float dt() const {return _dt;}

    private:

        float _dt;

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
