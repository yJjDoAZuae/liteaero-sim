
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

// input command is roll angle about velocity vector
// output command is roll rate about velocity vector

class ControlRoll : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float cmdIn, const KinematicState & state);
    void reset(float cmdIn, const KinematicState & state);

};

}
