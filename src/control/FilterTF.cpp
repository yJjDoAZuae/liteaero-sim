#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterTF.hpp"

static float dcTol = 1e-6;

using namespace liteaerosim::control;
using namespace liteaerosim;

// error_code values
// 0 : no error
// 1 : invalid vector dimension
// 2 : invalid timestep
// 3 : invalid denominator coefficients
// 4 : non-finite dc gain
// 5 : output initialization with zero dc gain

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterTF::copy(const FilterTF &filt)
{
    char n = (maxNumStates >= filt.order()) ? filt.order() : maxNumStates;

    _den = filt._den.head(n + 1);
    _num = filt._num.head(n + 1);
    uBuff = filt.uBuff.head(n + 1);
    yBuff = filt.yBuff.head(n + 1);

    // we ignore the first entry in den and set it to 1 in all cases
    _den(0) = 1.0f;

    _errorCode += filt.errorCode();
}

void FilterTF::setButterworthIIR(char order, float dt, float wn_rps)
{

    if (order > 10 || order > maxNumStates) {
        return;
    }

    FiltVectorXf num_s;
    FiltVectorXf den_s;

    _errorCode += butter(order, wn_rps, num_s, den_s);

    if (_errorCode != 0) {
        return;
    }

    _errorCode += tustin_n_tf(num_s, den_s, dt, wn_rps, _num, _den);

    uBuff.resize(order+1);
    yBuff.resize(order+1);

}

void FilterTF::resetInput(float in)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * in;
        yBuff << FiltVectorXf::Ones(order()+1) * in * dcGain;
    }

    _in = uBuff(0);
    _out = yBuff(0);
}

void FilterTF::resetOutput(float out)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * out / dcGain;
        yBuff << FiltVectorXf::Ones(order()+1) * out;
    }

    _in = uBuff(0);
    _out = yBuff(0);
}

float FilterTF::dcGain() const
{
    const float tol = 1e-6;

    // float dcGain = 1.0f;

    float numSum = _num.sum();
    float denSum = _den.sum();

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.
    if (fabs(denSum) < tol)
    {
        // non-finite DC gain
        // errorCode = 4;
        return 1.0f;
    }

    return numSum / denSum;
}

float FilterTF::step(float in)
{

    this->_in = in;
    this->_out = _num(0) * in;

    // NOTE: implicit state->a.k[0] == 1 because
    // we're assigning state->y.k[0] without a coefficient

    // update the output
    for (int k = 1; k < order() + 1 && k < kFilterMaxStates+1; k++)
    {
        // TRICKY: because we haven't rolled the buffers yet
        // the input and output buffer indices are one less
        // than the coefficient indices
        this->_out += _num(k) * uBuff(k - 1);
        this->_out -= _den(k) * yBuff(k - 1);
    }

    roll_buffer(yBuff, this->out());
    roll_buffer(uBuff, this->in());

    return this->out();
}
