
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlHeadingRate : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float headingRateCmdIn, const KinematicState & state);
    void reset(float headingRateCmdIn, const KinematicState & state);

};

}
