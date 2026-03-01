#include "control/ControlLoadFactor.hpp"
#include <Eigen/Dense>

using namespace liteaerosim::control;

static float constexpr g = 9.81;

float ControlLoadFactor::step(float loadFactorCmdIn, const KinematicState & state)
{
    return pid.step(loadFactorCmdIn, -state.acceleration_Wind_mps()(2)/g);
}

void ControlLoadFactor::reset(float loadFactorCmdIn, const KinematicState & state)
{
    pid.reset(loadFactorCmdIn, -state.acceleration_Wind_mps()(2)/g, 0.0f);
    pid.I.reset(0.0f);
}
