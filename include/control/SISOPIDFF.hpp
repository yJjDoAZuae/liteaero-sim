#pragma once

#include "control/control.hpp"
#include "control/Gain.hpp"
#include "control/FilterSS2Clip.hpp"
#include "control/Integrator.hpp"
#include "control/Derivative.hpp"
#include "control/Unwrap.hpp"

namespace liteaerosim::control {

class SISOPIDFF
{

public:
    FilterSS2Clip cmdSignal;
    FilterSS2Clip ffwdSignal;
    FilterSS2Clip measSignal;
    FilterSS2Clip measDotSignal;
    FilterSS2Clip errSignal; // for angular coordinates the errSignal.valLimit limits must be opposite sign if enabled
    FilterSS2Clip outSignal;

    Unwrap cmdUnwrap;
    Unwrap measUnwrap;

    Gain<float,3> Kp;
    Gain<float,3> Ki;
    Gain<float,3> Kd;
    Gain<float,3> Kff;

    Integrator I;
    Derivative D;

    bool unwrapInputs;

    SISOPIDFF() : unwrapInputs(false) {}

    SISOPIDFF(const SISOPIDFF &pid)
    {
        copy(pid);
    }

    void copy(const SISOPIDFF &pid);

    // step the PIDFF with internally calculated measurement derivative
    float step(float cmdIn, float measIn);

    // step the PIDFF with externally provided measurement derivative
    float step(float cmdIn, float measIn, float measDotIn);

    // reset the PIDFF based on cmd, meas, and output
    void reset(float cmdIn, float measIn, float outIn);

    // reset the PIDFF based on cmd, meas, measDot, and output
    void reset(float cmdIn, float measIn, float measDotIn, float outIn);

    float cmd() const { return cmdSignal.in(); }

    float meas() const { return measSignal.in(); }

    float measDot() const { return measDotSignal.in(); }

    float out() const { return outSignal.out(); }

    float err() const { return errSignal.in(); }

    // return the feedforward term
    float feedfwd() const { return Kff * ffwdSignal.out(); }

    // return the proportional term
    float prop() const { return Kp * errSignal.out(); }

    // return the derivative term
    float deriv() const { return Kd * measDotSignal.out(); }

    // return the integrator state (gain is upstream of the integrator)
    float integ() const {return I.out();}

};

}

