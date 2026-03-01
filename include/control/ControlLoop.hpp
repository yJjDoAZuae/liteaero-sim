
#include "control/SISOPIDFF.hpp"
#include "KinematicState.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace liteaerosim::control {

class ControlLoop {

public:

    SISOPIDFF pid;

    virtual void configure() = 0;
    virtual void configure(json jsonConfig) = 0;
    virtual float step(float cmdIn, const KinematicState & state) = 0;
    virtual void reset(float cmdIn, const KinematicState & state) = 0;

    float out() const { return pid.out(); }
    float cmd() const { return pid.cmd(); }
    float meas() const { return pid.meas(); }
    float err() const { return pid.err(); }
    float feedfwd() const { return pid.feedfwd(); }

};

}
