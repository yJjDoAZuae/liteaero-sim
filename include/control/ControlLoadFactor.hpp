
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlLoadFactor : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float loadFactorCmdIn, const KinematicState & state);
    void reset(float loadFactorCmdIn, const KinematicState & state);

};

}
