#include "control/ControlVerticalSpeed.hpp"
#include <cmath>

using namespace liteaerosim::control;

float velWind_horiz(const KinematicState & state)
{
    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    return hypot(velWind_NED_mps(0), velWind_NED_mps(1));
}

float ControlVerticalSpeed::step(float verticalSpeedCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = false;

    float gammaCmd = atan2(verticalSpeedCmdIn, velWind_horiz(state)); // positive up
    float gammaMeas = atan2(-state.velocity_NED_mps()(2), velWind_horiz(state));
    float gammaDot = state.pitchRate_rps() - state.alphaDot()*cos(state.roll());

    float gammaRateCmd = pid.step(gammaCmd, gammaMeas, gammaDot);

    return gammaRateCmd*state.velocity_Wind_mps().norm();
}

void ControlVerticalSpeed::reset(float verticalSpeedCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = false;

    float gammaCmd = atan2(verticalSpeedCmdIn, velWind_horiz(state)); // positive up
    float gammaMeas = atan2(-state.velocity_NED_mps()(2), velWind_horiz(state));
    pid.reset(gammaCmd, gammaMeas, 0.0f);
}
