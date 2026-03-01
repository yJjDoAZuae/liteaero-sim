#include "control/RateLimit.hpp"

using namespace liteaerosim::control;

void RateLimit::setLower(float lim)
{
    _lowerLimit = lim;

    // ensure _lowerLimit <= _upperLimit
    _upperLimit = (_upperLimit >= lim) ? _upperLimit : lim;

    // if enabled we can't reevaluate the previous input with the new limits
    // because we don't know what the output was prior to the previous step
}

void RateLimit::setUpper(float lim)
{
    _upperLimit = lim;

    // ensure _lowerLimit <= _upperLimit
    _lowerLimit = (_lowerLimit <= lim) ? _lowerLimit : lim;

    // if enabled we can't reevaluate the previous input with the new limits
    // because we don't know what the output was prior to the previous step
}

void RateLimit::reset(float u)
{
    _in = u;
    _out = u;
    _limitedLower = false;
    _limitedUpper = false;
}

float RateLimit::step(float u)
{

    _limitedLower = false;
    _limitedUpper = false;

    float delUnlim = u - _in;

    _in = u;

    float delLim = delUnlim;

    if (isLowerEnabled() && delUnlim < _dt*lowerLimit())
    {
        delLim = _dt*lowerLimit();
        _limitedLower = true;
    }

    if (isUpperEnabled() && delUnlim > _dt*upperLimit())
    {
        delLim = _dt*upperLimit();
        _limitedUpper = true;
    }

    _out += delLim;

    return _out;
}
