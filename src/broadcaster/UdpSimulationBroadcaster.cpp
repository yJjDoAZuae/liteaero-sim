#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include <algorithm>
#include "projection/IViewerProjector.hpp"
#include "liteaerosim.pb.h"

#include <cmath>
#include <cstdio>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
   using socket_t = SOCKET;
   static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
   static inline void close_sock(socket_t s) { closesocket(s); }
#else
#  include <sys/socket.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <fcntl.h>
   using socket_t = int;
   static constexpr socket_t kInvalidSocket = -1;
   static inline void close_sock(socket_t s) { close(s); }
#endif

#include <cstring>
#include <string>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

UdpSimulationBroadcaster::UdpSimulationBroadcaster(
    uint16_t port,
    const liteaero::projection::IViewerProjector* projector,
    bool verbose)
    : socket_fd_(static_cast<intptr_t>(kInvalidSocket))
    , port_(port)
    , projector_(projector)
    , verbose_(verbose)
{
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalidSocket) return;

    // Set socket to non-blocking so broadcast() never stalls the sim thread.
#ifdef _WIN32
    u_long nonblock = 1;
    ioctlsocket(s, FIONBIO, &nonblock);
#else
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif

    socket_fd_ = static_cast<intptr_t>(s);
}

// ---------------------------------------------------------------------------

UdpSimulationBroadcaster::~UdpSimulationBroadcaster()
{
    socket_t s = static_cast<socket_t>(socket_fd_);
    if (s != kInvalidSocket) {
        close_sock(s);
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

// ---------------------------------------------------------------------------

void UdpSimulationBroadcaster::broadcast(const SimulationFrame& frame)
{
    socket_t s = static_cast<socket_t>(socket_fd_);
    if (s == kInvalidSocket) return;

    las_proto::SimulationFrameProto proto;
    proto.set_timestamp_s(frame.timestamp_s);
    proto.set_latitude_rad(frame.latitude_rad);
    proto.set_longitude_rad(frame.longitude_rad);
    proto.set_height_wgs84_m(frame.height_wgs84_m);
    proto.set_q_w(frame.q_w);
    proto.set_q_x(frame.q_x);
    proto.set_q_y(frame.q_y);
    proto.set_q_z(frame.q_z);
    proto.set_velocity_north_mps(frame.velocity_north_mps);
    proto.set_velocity_east_mps(frame.velocity_east_mps);
    proto.set_velocity_down_mps(frame.velocity_down_mps);
    proto.set_airspeed_mps(frame.airspeed_mps);
    proto.set_agl_m(frame.agl_m);
    proto.set_height_msl_m(frame.height_msl_m);

    // Viewer-projected position (LS-T5 / OQ-LS-15).  When no projector is
    // configured, the fields remain zero; downstream receivers treat zero as
    // "absent" and fall back to their own coordinate handling.
    float vp_y = 0.0f;
    if (projector_ != nullptr) {
        const auto vp = projector_->project(frame.latitude_rad,
                                            frame.longitude_rad,
                                            frame.height_wgs84_m);
        proto.set_viewer_x_m(vp.x_m);
        proto.set_viewer_y_m(vp.y_m);
        proto.set_viewer_z_m(vp.z_m);
        vp_y = vp.y_m;
    }

    // Verbose diagnostic (opt-in via --verbose): per-frame pose trace, throttled to
    // the first few frames, every 25th frame, and any crossing into near-ground AGL.
    // Lets a developer confirm the broadcast pose (position + attitude) the viewer
    // receives. Silent unless verbose mode is enabled.
    ++broadcast_count_;
    if (verbose_) {
        const bool first_few = (broadcast_count_ <= 3);
        const bool agl_near_zero = (frame.agl_m >= 0.f && frame.agl_m < 1.0f);
        const bool agl_crossed = agl_near_zero && !prev_agl_near_zero_;
        prev_agl_near_zero_ = agl_near_zero;
        if (first_few || agl_crossed || (broadcast_count_ % 25u == 0u)) {
            // Euler angles from q_nb (body-to-NED, ZYX) + viewer x/z, for pose-tracking checks.
            const float qw = frame.q_w, qx = frame.q_x, qy = frame.q_y, qz = frame.q_z;
            const float roll  = std::atan2(2.f*(qw*qx+qy*qz), 1.f-2.f*(qx*qx+qy*qy)) * 57.2958f;
            const float pitch = std::asin (std::clamp(2.f*(qw*qy-qz*qx), -1.f, 1.f)) * 57.2958f;
            const float yaw   = std::atan2(2.f*(qw*qz+qx*qy), 1.f-2.f*(qy*qy+qz*qz)) * 57.2958f;
            std::printf(
                "[bc #%u] agl=%.2f vY=%.2f vX=%.2f vZ=%.2f as=%.2f  roll=%.1f pitch=%.1f yaw=%.1f  "
                "q=(%.3f,%.3f,%.3f,%.3f) %s\n",
                broadcast_count_, frame.agl_m, vp_y,
                (projector_ ? proto.viewer_x_m() : 0.f), (projector_ ? proto.viewer_z_m() : 0.f),
                frame.airspeed_mps, roll, pitch, yaw, qw, qx, qy, qz,
                std::isnan(frame.height_wgs84_m) ? "NaN!" : "");
            std::fflush(stdout);
        }
    }

    const std::string serialized = proto.SerializeAsString();

    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port_);
    dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    sendto(s,
           serialized.data(),
           static_cast<int>(serialized.size()),
           0,
           reinterpret_cast<const sockaddr*>(&dest),
           sizeof(dest));
    // Errors (EAGAIN, EWOULDBLOCK, no listener) are silently ignored —
    // UDP broadcast is fire-and-forget; the sim thread must not block.
}

}  // namespace liteaero::simulation
