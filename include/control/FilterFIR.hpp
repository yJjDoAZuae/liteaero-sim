
#pragma once

#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>


namespace liteaerosim::control {

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterFIR : public Filter
{

public:
    FilterFIR() : _in(0), _out(0), _errorCode(0)
    {
        num << 1;
        uBuff << 0;
    }

    FilterFIR(FilterFIR &filt)
    {
        copy(filt);
    }

    ~FilterFIR() override {}

    float in() const { return _in; }
    float out() const { return _out; }
    operator float() const { return out(); }

    void copy(FilterFIR &filt);

    uint8_t order() { return num.rows() - 1; }

    // FIR filter design
    void setAverageFIR(char order);        // equal weight moving average FIR filter design
    void setExpFIR(char order, float dt, float tau); // exponential decaying weight moving average FIR filter design

    // step the filter
    float step(float in) override;

    // reset the fiter based on inputs
    void resetToInput(float in_val);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetToOutput(float out_val);

    // dc gain value of the filter
    float dcGain();

    uint16_t errorCode() const override { return _errorCode; }

private:

    FiltVectorXf num;
    FiltVectorXf uBuff;

    float _in;
    float _out;

    uint16_t _errorCode;

};

}