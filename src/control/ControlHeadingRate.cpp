#include "control/ControlHeadingRate.hpp"
#include <Eigen/Dense>

using namespace liteaerosim::control;

float velWind_horiz(const KinematicState & state)
{
    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    return hypot(velWind_NED_mps(0), velWind_NED_mps(1));
}

float ControlHeadingRate::step(float headingRateCmdIn, const KinematicState & state)
{
    float ayCmd = velWind_horiz(state)*headingRateCmdIn;
    float ayMeas = state.headingRate_rps() * velWind_horiz(state);

    return pid.step(ayCmd, ayMeas);
}

void ControlHeadingRate::reset(float headingRateCmdIn, const KinematicState & state)
{
    float ayCmd = velWind_horiz(state)*headingRateCmdIn;
    float ayMeas = state.headingRate_rps() * velWind_horiz(state);

    pid.reset(ayCmd, ayMeas, 0.0f);
    pid.I.resetTo(0.0f);
}
