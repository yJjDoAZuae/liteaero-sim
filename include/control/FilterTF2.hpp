
#pragma once

#include "SISOBlock.hpp"
#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // for Matrix::pow()

namespace liteaerosim::control {

// template <char NUM_STATES=FILTER_MAX_STATES>

class FilterTF2 : public Filter {

public:
    FilterTF2() :
         _in(0), 
        _out(0),
        _errorCode(0),
        _order(0)
    {
        _num << 1, 0, 0;
        _den << 1, 0, 0;
        uBuff.setZero();
        yBuff.setZero();
    }

    FilterTF2(const FilterTF2 &filt)
    {
        copy(filt);
    }

    ~FilterTF2() override {}

    float in() const override { return _in; }
    float out() const override { return _out; }
    operator float() const override { return out(); }

    void copy(const FilterTF2 &filt);

    // IIR filter design
    void setLowPassFirstIIR(float dt, float tau);                  // first order low pass filter design
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);  // second order low pass filter design
    void setHighPassFirstIIR(float dt, float tau);                 // first order high pass filter design
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero); // second order high pass filter design
    void setDerivIIR(float dt, float tau);                         // first order derivative + low pass filter design
    void setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num);    // second order notch filter design

    void setZPK(float dt, float z_re, float z_im, float p_re, float p_im, float K);

    // step the filter
    float step(float in) override;

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

    // dc gain value of the filter
    float dcGain() const;

    Vec3 num() const {return _num;}
    Vec3 den() const {return _den;}

    uint8_t order() const {return _order;}

    uint16_t errorCode() const override { return _errorCode; }

private:

    // 2nd order ARMA numerator and denominator vectors
    Vec3 _num;
    Vec3 _den;

    // 2nd order input and output buffers
    Vec3 uBuff;
    Vec3 yBuff;

    uint8_t _order;

    float _in;
    float _out;

    uint16_t _errorCode;
};

}
