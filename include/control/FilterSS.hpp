
#pragma once

#include "control/control.hpp"
#include "control/Filter.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // for Matrix::pow()

namespace liteaerosim::control {

class FilterSS2;  // forward declaration — full definition in FilterSS2.hpp

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterSS : public Filter
{

public:
    FilterSS() :
         _in(0), 
        _out(0),
        _errorCode(0)
    {
        _Phi.resize(0,0);
        _Gamma.resize(0,1);
        _H.resize(1,0);
        _J.setOnes();
        _x.resize(0,1);
    }


    FilterSS(FilterSS &filt)
    {
        copy(filt);
    }

    ~FilterSS() override;

    void copy(FilterSS &filt);
    void copy(liteaerosim::control::FilterSS2 &filt);

    float in() const override { return _in; }
    float out() const override { return _out; }
    operator float() const override { return out(); }

    // IIR filter design
    void setButterworthIIR(uint8_t order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

    uint8_t order() const;

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

    MatNN Phi() const {return _Phi;}
    MatN1 Gamma() const {return _Gamma;}
    Mat1N H() const {return _H;}
    Mat11 J() const {return _J;}
    MatN1 x() const {return _x;}

    MatNN controlGrammian() const;
    MatNN observeGrammian() const;

    uint16_t errorCode() const override { return _errorCode; }

private:

    void setDimension(uint8_t dim);

    // state space realization matrices
    MatNN _Phi;
    MatN1 _Gamma;
    Mat1N _H;
    Mat11 _J;

    // state vector
    MatN1 _x;

    float _in;
    float _out;

    uint16_t _errorCode;
};

}
