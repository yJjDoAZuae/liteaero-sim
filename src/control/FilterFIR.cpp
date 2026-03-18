#include "control/FilterFIR.hpp"

static float dcTol = 1e-6;

using namespace liteaerosim::control;
using namespace liteaerosim;


void FilterFIR::setAverageFIR(char order)
{
    num = FiltVectorXf::Ones(order+1)*(1.0f/(order+1.0f));
    uBuff = FiltVectorXf::Zero(order+1);
}

void FilterFIR::setExpFIR(char order, float dt, float tau)
{
    num = FiltVectorXf::Zero(order+1);
    uBuff = FiltVectorXf::Zero(order+1);

    for (int k=0; k<this->order()+1; k++) {
        num(k) = exp(-k*dt/tau);
    }
    float sumNum = num.sum();
    num *= 1.0f/sumNum;  // normalize to unity dc gain
}


void FilterFIR::resetToInput(float in)
{
    uBuff.setZero();

    uBuff << FiltVectorXf::Ones(order()+1) * in;

    _in = uBuff(0);
    _out = _in*dcGain();
}

void FilterFIR::resetToOutput(float out)
{
    const float tol = 1e-6;

    uBuff.setZero();

    float dcGain = this->dcGain();

    if (fabs(dcGain) > tol) {
        uBuff << FiltVectorXf::Ones(order()+1) * out / dcGain;
        _in = uBuff(0);
        _out = _in/dcGain;
    } else {
        // zero DC gain
        _errorCode += FilterError::ZERO_DC_GAIN;
        _in = 0.0f;
        _out = 0.0f;
    }
}

float FilterFIR::dcGain()
{
    return num.sum();
}

float FilterFIR::step(float in)
{

    this->_in = in;
    this->_out = num(0)*in;

    // update the output
    for (int k = 1; k < order() + 1; k++)
    {
        // TRICKY: because we haven't rolled the buffer yet
        // the input buffer indices are one less
        // than the coefficient indices
        this->_out += num(k) * uBuff(k - 1);
    }

    roll_buffer(uBuff, this->out());

    return this->out();
}
