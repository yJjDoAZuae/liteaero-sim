#include "control/ControlRoll.hpp"

using namespace liteaerosim::control;

float ControlRoll::step(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;

    // get attitude Eulers of Wind frame
    Eigen::Vector3f eulersWind = state.q_nw().toRotationMatrix().eulerAngles(3,2,1);

    // get Body rates expressed in wind axes
    Eigen::Vector3f ratesWind = state.q_nw().toRotationMatrix().transpose() * state.q_nb().toRotationMatrix() * state.rates_Body_rps();

    // NOTE: roll and rollRate_rps aren't correct for velocity roll control
    // use roll Euler of Wind frame w.r.t. NED and the roll rate of the Wind frame
    return pid.step(cmdIn, eulersWind(0), state.rollRate_Wind_rps());
}

void ControlRoll::reset(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;

    // get attitude Eulers of Wind frame
    Eigen::Vector3f eulersWind = state.q_nw().toRotationMatrix().eulerAngles(3,2,1);
    
    pid.reset(cmdIn, eulersWind(0), 0.0f);
}
