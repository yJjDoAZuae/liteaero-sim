#pragma once

#include "control/control.hpp"
#include "SISOBlock.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

class Derivative : public liteaerosim::SISOBlock {

    public:

        Derivative() : 
                _in(0),
                _out(0),
                _dt(1.0f), 
                _Tau(0.0f), 
                _method(DiscretizationMethod::FwdEuler) 
            {}

        ~Derivative() override {}

        float in() const override { return _in; }
        float out() const override { return _out; }
        operator float() const override { return out(); }

        Limit limit;

        void reset(float u) 
        {
            reset(u, 0.0f);
        }
        
        void reset(float u, float uDot=0) 
        {
            _out = limit.step(uDot);
            _in = u;
        }

        float step(float u) override;
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };
        void setTau(float Tau) { _Tau = Tau; };
        float dt() const {return _dt;}
        float Tau() const {return _Tau;}
        void setMethod(DiscretizationMethod method) {_method = method;}

    protected:

        float _dt;
        float _Tau;
        DiscretizationMethod _method;

        float _in;
        float _out;

};

}
