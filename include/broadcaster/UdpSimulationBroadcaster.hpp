#pragma once

#include "broadcaster/ISimulationBroadcaster.hpp"

#include <cstdint>

namespace liteaero::projection { class IViewerProjector; }

namespace liteaero::simulation {

// UDP implementation of ISimulationBroadcaster.
// Serializes SimulationFrame as a protobuf SimulationFrameProto message and
// sends it as a UDP datagram to 127.0.0.1:<port> (default 14560).
// The send is a non-blocking sendto(); dropped datagrams (e.g. no listener)
// are silently discarded.  Safe to call from any thread after construction.
//
// The optional projector (LS-T5) is invoked once per frame to populate the
// viewer_x/y/z_m proto fields.  When projector is null, viewer fields are
// zero and downstream receivers should treat them as absent.  The broadcaster
// does not own the projector — the caller is responsible for keeping it alive
// at least as long as the broadcaster.
class UdpSimulationBroadcaster : public ISimulationBroadcaster {
public:
    explicit UdpSimulationBroadcaster(
        uint16_t port = 14560,
        const liteaero::projection::IViewerProjector* projector = nullptr,
        bool verbose = false);
    ~UdpSimulationBroadcaster() override;

    void broadcast(const SimulationFrame& frame) override;

private:
    // Opaque integer socket handle.  Stored as intptr_t so the header does
    // not need to include platform socket headers.
    intptr_t                                       socket_fd_;
    uint16_t                                       port_;
    const liteaero::projection::IViewerProjector*  projector_;
    bool                                           verbose_            = false;
    unsigned int                                   broadcast_count_    = 0;
    bool                                           prev_agl_near_zero_ = false;
};

}  // namespace liteaero::simulation
