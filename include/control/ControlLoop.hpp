
#include <liteaero/control/SISOPIDFF.hpp>
#include "KinematicState.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace liteaerosim::control {

class ControlLoop {

public:

    virtual void configure() = 0;
    virtual void configure(json config) = 0;
    virtual float step(float command, const KinematicState& state) = 0;
    virtual void reset(float command, const KinematicState& state) = 0;

    float output()      const { return controller_.output(); }
    float command()     const { return controller_.command(); }
    float measurement() const { return controller_.measurement(); }
    float error()       const { return controller_.error(); }
    float feedForward() const { return controller_.feedForward(); }

protected:
    liteaero::control::SISOPIDFF controller_;

};

}
