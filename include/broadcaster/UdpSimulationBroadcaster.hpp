#pragma once

#include "broadcaster/ISimulationBroadcaster.hpp"

#include <cstdint>

namespace liteaero::simulation {

// UDP implementation of ISimulationBroadcaster.
// Serializes SimulationFrame as a protobuf SimulationFrameProto message and
// sends it as a UDP datagram to 127.0.0.1:<port> (default 14560).
// The send is a non-blocking sendto(); dropped datagrams (e.g. no listener)
// are silently discarded.  Safe to call from any thread after construction.
class UdpSimulationBroadcaster : public ISimulationBroadcaster {
public:
    explicit UdpSimulationBroadcaster(uint16_t port = 14560);
    ~UdpSimulationBroadcaster() override;

    void broadcast(const SimulationFrame& frame) override;

private:
    // Opaque integer socket handle.  Stored as intptr_t so the header does
    // not need to include platform socket headers.
    intptr_t socket_fd_;
    uint16_t port_;
};

}  // namespace liteaero::simulation
