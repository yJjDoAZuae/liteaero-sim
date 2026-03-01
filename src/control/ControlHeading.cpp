#include "control/ControlHeading.hpp"

using namespace liteaerosim::control;

float ControlHeading::step(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    return pid.step(cmdIn, state.heading(), state.headingRate_rps());
}

void ControlHeading::reset(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    pid.reset(cmdIn, state.heading(), 0.0f);
}
