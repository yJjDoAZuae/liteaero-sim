
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlHeading : public ControlLoop {

    void configure();
    void configure(json jsonConfig);
    float step(float cmdIn, const KinematicState & state);
    void reset(float cmdIn, const KinematicState & state);

};

}
