#include "control/ControlAltitude.hpp"

using namespace liteaerosim::control;

float ControlAltitude::step(float altitudeCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    return pid.step(altitudeCmdIn, state.positionDatum().height_WGS84_m(), -state.velocity_NED_mps()(2));
}

void ControlAltitude::reset(float altitudeCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    pid.reset(altitudeCmdIn, state.positionDatum().height_WGS84_m(), 0.0f);
}
