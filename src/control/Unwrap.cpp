#include "control/Unwrap.hpp"
#include "math/math_util.hpp"

using namespace liteaerosim::control;

float Unwrap::step(float u) 
{
    return step(u, _out);
}

float Unwrap::step(float u, float ref) 
{
    _in = u;
    _out = ref + MathUtil::wrapToPi(_in - ref);;

    return _out;
}
