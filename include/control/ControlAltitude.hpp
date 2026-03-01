
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlAltitude : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float altitudeCmdIn, const KinematicState & state);
    void reset(float altitudeCmdIn, const KinematicState & state);

};

}
