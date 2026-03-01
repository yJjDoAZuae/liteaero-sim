
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

// input is a vertical speed command (positive local level up)
// output is a vertical acceleration command (positive local level up)
class ControlVerticalSpeed : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float verticalSpeedCmdIn, const KinematicState & state);
    void reset(float verticalSpeedCmdIn, const KinematicState & state);

};

}
