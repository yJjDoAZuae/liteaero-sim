#pragma once

#include "control/control.hpp"
#include "SISOBlock.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

    class Integrator : public liteaerosim::SISOBlock {

    public:

        Integrator() : 
                _in(0), 
                _out(0), 
                _dt(1.0f), 
                _method(DiscretizationMethod::FwdEuler) 
            {}

        ~Integrator() override {}

        float in() const override { return _in; }
        float out() const override { return _out; }
        operator float() const override { return out(); }

        Limit limit;
        std::vector<Antiwindup> aw;

        void reset(float u);

        float step(float u) override;
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };
        float dt() const {return _dt;}
        void setMethod(DiscretizationMethod method) {_method = method;}

    protected:

        float _dt;
        DiscretizationMethod _method;

        float _in;
        float _out;

};

}
