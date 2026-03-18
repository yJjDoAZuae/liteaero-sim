
#pragma once

#include "control/Filter.hpp"
#include "control/Limit.hpp"

namespace liteaerosim::control {

// template <char NUM_STATES=FILTER_MAX_STATES>

class FilterSS2Clip : public Filter {

public:

    FilterSS2Clip() :
         _in(0), 
        _out(0),
        _errorCode(0),
        _order(0),
        _dt(1.0f)
    {
        _Phi.setZero();
        _Gamma.setZero();
        _H.setZero();
        _J.setOnes();
        _x.setZero();
    }

    ~FilterSS2Clip() override {}

    float in()       const override { return _in; }
    float out()      const override { return _out; }
    operator float() const override { return _out; }

    Limit valLimit;
    Limit rateLimit;

    // step the filter
    float step(float in) override;

    void copy(FilterSS2Clip &filt);

    // IIR filter design
    void setLowPassFirstIIR(float dt, float tau);                  // first order low pass filter design
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);  // second order low pass filter design
    void setHighPassFirstIIR(float dt, float tau);                 // first order high pass filter design
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero); // second order high pass filter design
    void setDerivIIR(float dt, float tau);                         // first order derivative + low pass filter design
    void setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num);    // second order notch filter design

    // dc gain value of the filter
    float dcGain() const;

    Mat22 Phi() const {return _Phi;}
    Mat21 Gamma() const {return _Gamma;}
    Mat12 H() const {return _H;}
    Mat11 J() const {return _J;}
    Mat21 x() const {return _x;}

    // reset the fiter based on inputs
    void resetToInput(float in_val);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetToOutput(float out_val);

    // Directly restore the filter internal state vector.
    // Exact warm-start: restores x without the steady-state backsolve assumption.
    void resetState(const Mat21& x);

    uint8_t order() const override {return _order;}
    float dt() const {return _dt;}

    Mat22 controlGrammian() const;
    Mat22 observeGrammian() const;

    uint16_t errorCode() const override { return _errorCode; }

private:

    void backsolve1(float inPrev, float outPrev);
    void backsolve2(float inPrev, float outPrev);
    void backsolve(float inPrev, float outPrev);

    // 2nd order state space realization matrices
    Mat22 _Phi;
    Mat21 _Gamma;
    Mat12 _H;
    Mat11 _J;

    // 2nd order state vector
    Mat21 _x;

    uint8_t _order;
    float _dt;

    float _in;
    float _out;

    uint16_t _errorCode;
};

}
