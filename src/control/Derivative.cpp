#include "control/Derivative.hpp"

using namespace liteaerosim::control;

float Derivative::step(float in)
{

    _Tau = (_Tau > 4.0f*_dt) ? _Tau : 4.0f*_dt;

    switch (_method) {

    case DiscretizationMethod::BackEuler:
        // Backward Euler
        _out = limit.step(_Tau/(_Tau+_dt)*_out + (in - _in) / (_Tau + _dt));
        break;

    case DiscretizationMethod::FwdEuler:
        // Forward Euler
        _out = limit.step(_out * (_Tau - _dt)/_Tau + (in - _in) /_Tau);
        break;

    case DiscretizationMethod::Bilinear:
        // Bilinear (Tustin)
        _out = limit.step(_out * (2.0f*_Tau - _dt)/(2.0f*_Tau + _dt) + 0.5*(in + _in) * 2.0f/(2.0f*_Tau + _dt));
        break;
    }

    _in = in;
    return _out;
}
