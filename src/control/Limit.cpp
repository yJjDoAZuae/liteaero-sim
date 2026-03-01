#include "control/Limit.hpp"

using namespace liteaerosim::control;

// void Limit::copy(const SISOLimit& lim) 
// {
//     // this is just a plain copy, so probably not necessary
//     _in = lim._in;
//     _lowerLimit = lim._lowerLimit;
//     _upperLimit = lim._upperLimit;
//     _limitedLower = lim._limitedLower;
//     _limitedLower = lim._limitedLower;
//     _enableLowerLimit = lim._enableLowerLimit;
//     _enableUpperLimit = lim._enableUpperLimit;
//     _out = lim._out;
// }

float Limit::step(float u)
{
    _in = u;

    _out = _in;
    _limitedLower = false;
    _limitedUpper = false;

    if (isLowerEnabled() && _out < lowerLimit())
    {
        _out = lowerLimit();
        _limitedLower = true;
    }

    if (isUpperEnabled() && _out > upperLimit())
    {
        _out = upperLimit();
        _limitedUpper = true;
    }

    return _out;
}

void Limit::setLower(float lim)
{
    _lowerLimit = lim;

    // ensure _lowerLimit <= _upperLimit
    _upperLimit = (_upperLimit >= lim) ? _upperLimit : lim;

    step(_in); // reevaluate the previous input with the new limits
}

void Limit::setUpper(float lim)
{
    _upperLimit = lim;

    // ensure _lowerLimit <= _upperLimit
    _lowerLimit = (_lowerLimit <= lim) ? _lowerLimit : lim;

    step(_in); // reevaluate the previous input with the new limits
}
