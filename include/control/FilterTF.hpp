
#pragma once

#include "SISOBlock.hpp"
#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>


namespace liteaerosim::control {


// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterTF : public Filter
{

public:
    FilterTF() :
         _in(0), 
        _out(0),
        _errorCode(0)
    {
        _num.resize(1);
        _den.resize(1);
        uBuff.resize(1);
        yBuff.resize(1);
        _num << 1;
        _den << 1;
        uBuff << 0;
        yBuff << 0;
    }

    FilterTF(const FilterTF &filt)
    {
        copy(filt);
    }

    ~FilterTF() override {}

    float in() const override { return _in; }
    float out() const override { return _out; }
    operator float() const override { return out(); }

    void copy(const FilterTF &filt);

    // IIR filter design
    void setButterworthIIR(char order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

    uint8_t order() const { return _den.rows() - 1; }

    // step the filter
    float step(float in) override;

    // reset the fiter based on inputs
    void resetToInput(float in_val);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetToOutput(float out_val);

    // dc gain value of the filter
    float dcGain() const;

    Vec3 num() const {return _num;}
    Vec3 den() const {return _den;}

    uint16_t errorCode() const override { return _errorCode; }

private:

    FiltVectorXf _num;
    FiltVectorXf _den;

    FiltVectorXf uBuff;
    FiltVectorXf yBuff;

    float _in;
    float _out;

    uint16_t _errorCode;
};

}
