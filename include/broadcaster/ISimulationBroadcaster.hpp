#pragma once

#include "SimulationFrame.hpp"

namespace liteaero::simulation {

// Pure interface for live simulation frame transport.
// SimRunner depends on this interface; implementations (e.g. UDP) live in
// the Interface Layer.  SimRunner does not take ownership of the broadcaster.
class ISimulationBroadcaster {
public:
    virtual ~ISimulationBroadcaster() = default;
    virtual void broadcast(const SimulationFrame& frame) = 0;
};

}  // namespace liteaero::simulation
