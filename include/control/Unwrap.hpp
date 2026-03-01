

#pragma once

#include "SISOBlock.hpp"

namespace liteaerosim::control {

class Unwrap : public liteaerosim::SISOBlock {

    public:

        Unwrap() : _in(0), _out(0) {}

        ~Unwrap() override {}

        float in() const override { return _in; }
        float out() const override { return _out; }
        operator float() const override { return out(); }

        // unwrap relative to previous output value
        float step(float u) override;

        // unwrap relative to an externally provided reference
        float step(float u, float ref);

        void reset(float u) { _in = u; _out = u; };

    private:

        float _in;
        float _out;

};

}
