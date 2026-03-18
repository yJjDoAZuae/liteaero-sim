#define _USE_MATH_DEFINES

#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterTF2.hpp"

static float dcTol = 1e-6;

using namespace liteaerosim::control;
using namespace liteaerosim;

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterTF2::copy(const FilterTF2 &filt)
{
    _num << filt.num();
    _den << filt.den();
    uBuff << filt.uBuff;
    yBuff << filt.yBuff;

    _errorCode = filt.errorCode();
    
    _order = filt.order();
}

void FilterTF2::setLowPassFirstIIR(float dt, float tau)
{
    // HACK: zero order hold realization
    // ydot = -1/tau y + 1/tau u
    // yk+1 - yk = (-1/tau yk + 1/tau u) * dt
    // yk+1 = (1 - dt/tau) xk + dt/tau u

    // numz.k[0] = dt/tau;
    // numz.k[1] = 0.0f;
    // denz.k[0] = 1.0f;
    // denz.k[1] = -(1-dt/tau);

    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 0.0f;
    num_s(1) = 0.0f;
    num_s(2) = 1.0f / tau;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    _errorCode += tustin_1_tf(num_s, den_s, dt, 2.0f*M_PI / tau, _num, _den);

    _order = 1;

}

void FilterTF2::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero)
{
    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 0.0f;  // s^2
    num_s(1) = tau_zero * wn_rps * wn_rps; // s
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, _num, _den);

    _order = 2;
}

void FilterTF2::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num)
{
    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 1.0f;
    num_s(1) = 2.0f * zeta_num * wn_rps;
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;
    den_s(1) = 2.0f * zeta_den * wn_rps;
    den_s(2) = wn_rps * wn_rps;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, _num, _den);

    _order = 2;
}

void FilterTF2::setHighPassFirstIIR(float dt, float tau)
{
    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    _errorCode = tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, _num, _den);

    _order = 1;
}

void FilterTF2::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero)
{
    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 1.0f;  // s^2
    num_s(1) = c_zero * 2.0f * zeta * wn_rps;  // s
    num_s(2) = 0.0f;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, _num, _den);

    _order = 2;
}

void FilterTF2::setDerivIIR(float dt, float tau)
{
    Vec3 num_s;
    Vec3 den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    _errorCode += tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, _num, _den);

    _order = 1;
}

void FilterTF2::resetToInput(float in)
{
    const float tol = 1e-6;

    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << Vec3::Ones() * in;
        yBuff << Vec3::Ones() * in * dcGain;
    }

    _in = uBuff(0);
    _out = yBuff(0);
}

void FilterTF2::resetToOutput(float out)
{
    const float tol = 1e-6;

    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << Vec3::Ones() * out / dcGain;
        yBuff << Vec3::Ones() * out;
    }

    _in = uBuff(0);
    _out = yBuff(0);

}

float FilterTF2::dcGain() const
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

float FilterTF2::step(float in)
{

    this->_in = in;
    this->_out = _num(0) * in;

    // NOTE: implicit state->a.k[0] == 1 because
    // we're assigning state->y.k[0] without a coefficient

    // update the output
    for (int k = 1; k < order() + 1 && k < 3; k++)
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
