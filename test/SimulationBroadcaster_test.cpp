// Tests for UdpSimulationBroadcaster and SimRunner broadcaster integration.
//
// Design authority: docs/architecture/live_sim_view.md
//
// Three test cases per design spec:
//   test_simulation_frame_size        — sizeof(SimulationFrame) sanity check
//   test_udp_broadcaster_sends_datagram — UdpSimulationBroadcaster sends a datagram
//                                         readable by a loopback UDP receiver
//   test_sim_runner_broadcasts_after_step — after a brief RealTime run the loopback
//                                           receiver has received >= 1 datagram

#include "SimulationFrame.hpp"
#include "broadcaster/ISimulationBroadcaster.hpp"
#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "runner/SimRunner.hpp"
#include "Aircraft.hpp"
#include "propulsion/Propulsion.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
   using socket_t = SOCKET;
   static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
   static inline void close_socket(socket_t s) { closesocket(s); }
#else
#  include <sys/socket.h>
#  include <arpa/inet.h>
#  include <unistd.h>
   using socket_t = int;
   static constexpr socket_t kInvalidSocket = -1;
   static inline void close_socket(socket_t s) { close(s); }
#endif

#include <array>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

namespace liteaero::simulation {
namespace {

// ---------------------------------------------------------------------------
// Zero-thrust propulsion stub — satisfies the Aircraft constructor requirement.
// ---------------------------------------------------------------------------
class StubPropulsion final : public Propulsion {
public:
    float step(float, float, float) override { return 0.0f; }
    float thrust_n() const override          { return 0.0f; }

    std::vector<uint8_t> serializeProto() const override           { return {}; }
    void                 deserializeProto(const std::vector<uint8_t>&) override {}

protected:
    void           onInitialize(const nlohmann::json&) override {}
    void           onReset() override {}
    nlohmann::json onSerializeJson() const override  { return {}; }
    void           onDeserializeJson(const nlohmann::json&) override {}
    int            schemaVersion() const override    { return 1; }
    const char*    typeName() const override         { return "StubPropulsion"; }
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Open a UDP socket bound to the loopback on the given port.
// Returns kInvalidSocket on failure.
static socket_t open_recv_socket(uint16_t port)
{
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalidSocket) return kInvalidSocket;

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close_socket(s);
        return kInvalidSocket;
    }

    // Set receive timeout to 500 ms so tests don't block forever.
#ifdef _WIN32
    DWORD timeout_ms = 500;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms));
#else
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
    return s;
}

// Minimal GA aircraft JSON config reused across tests.
static const std::string kGaJson = R"({
    "schema_version": 1,
    "aircraft": {
        "S_ref_m2": 16.2, "cl_y_beta": -0.60, "ar": 7.47, "e": 0.80,
        "cd0": 0.027, "cmd_filter_substeps": 1,
        "nz_wn_rad_s": 10.0, "nz_zeta_nd": 0.7,
        "ny_wn_rad_s": 10.0, "ny_zeta_nd": 0.7,
        "roll_rate_wn_rad_s": 20.0, "roll_rate_zeta_nd": 0.7
    },
    "airframe": { "g_max_nd": 3.8, "g_min_nd": -1.52,
                  "tas_max_mps": 82.3, "mach_max_nd": 0.25 },
    "inertia":  { "mass_kg": 1045.0, "Ixx_kgm2": 1285.0,
                  "Iyy_kgm2": 1825.0, "Izz_kgm2": 2667.0 },
    "lift_curve": {
        "cl_alpha": 5.1, "cl_max": 1.80, "cl_min": -1.20,
        "delta_alpha_stall": 0.262, "delta_alpha_stall_neg": 0.262,
        "cl_sep": 1.05, "cl_sep_neg": -0.80
    },
    "initial_state": {
        "latitude_rad": 0.0, "longitude_rad": 0.0, "altitude_m": 300.0,
        "velocity_north_mps": 55.0, "velocity_east_mps": 0.0,
        "velocity_down_mps": 0.0,
        "wind_north_mps": 0.0, "wind_east_mps": 0.0, "wind_down_mps": 0.0
    }
})";

// Use a high-numbered test port unlikely to conflict with system services.
static constexpr uint16_t kTestPort = 19560;

static std::unique_ptr<Aircraft> make_aircraft(float dt_s = 0.02f)
{
    auto ac = std::make_unique<Aircraft>(std::make_unique<StubPropulsion>());
    ac->initialize(nlohmann::json::parse(kGaJson), dt_s);
    return ac;
}

// ---------------------------------------------------------------------------
// test_simulation_frame_size
// ---------------------------------------------------------------------------

TEST(SimulationBroadcaster, test_simulation_frame_size)
{
    // SimulationFrame: timestamp_s, latitude_rad, longitude_rad (3 doubles = 24 B)
    //                  + height_wgs84_m, q_w/x/y/z, velocity N/E/D (8 floats = 32 B)
    //                  = 56 bytes (all members naturally aligned; no padding).
    static_assert(sizeof(double) == 8, "unexpected double size");
    static_assert(sizeof(float)  == 4, "unexpected float size");

    constexpr size_t expected = 3 * sizeof(double) + 8 * sizeof(float);  // 56
    EXPECT_EQ(sizeof(SimulationFrame), expected);

    // Verify field offsets are consistent with the protobuf schema field order.
    SimulationFrame f{};
    EXPECT_EQ(static_cast<void*>(&f.timestamp_s),        static_cast<void*>(&f));
    EXPECT_GT(reinterpret_cast<uintptr_t>(&f.q_w),
              reinterpret_cast<uintptr_t>(&f.height_wgs84_m));
}

// ---------------------------------------------------------------------------
// test_udp_broadcaster_sends_datagram
// ---------------------------------------------------------------------------

TEST(SimulationBroadcaster, test_udp_broadcaster_sends_datagram)
{
    socket_t recv_sock = open_recv_socket(kTestPort);
    ASSERT_NE(recv_sock, kInvalidSocket) << "Could not bind loopback receiver socket";

    {
        UdpSimulationBroadcaster broadcaster(kTestPort);
        SimulationFrame frame{};
        frame.timestamp_s        = 1.0;
        frame.latitude_rad       = 0.1;
        frame.longitude_rad      = 0.2;
        frame.height_wgs84_m     = 300.0f;
        frame.q_w                = 1.0f;
        frame.q_x = frame.q_y = frame.q_z = 0.0f;
        frame.velocity_north_mps = 55.0f;
        frame.velocity_east_mps  = 0.0f;
        frame.velocity_down_mps  = 0.0f;

        broadcaster.broadcast(frame);
    }

    std::array<char, 4096> buf{};
    const int received = static_cast<int>(
        recv(recv_sock, buf.data(), static_cast<int>(buf.size()), 0));
    close_socket(recv_sock);

    EXPECT_GT(received, 0) << "No datagram received on loopback port " << kTestPort;
}

// ---------------------------------------------------------------------------
// test_sim_runner_broadcasts_after_step
// ---------------------------------------------------------------------------

TEST(SimulationBroadcaster, test_sim_runner_broadcasts_after_step)
{
    // Use a different port to avoid interference from the previous test.
    static constexpr uint16_t kRunnerTestPort = 19561;

    socket_t recv_sock = open_recv_socket(kRunnerTestPort);
    ASSERT_NE(recv_sock, kInvalidSocket) << "Could not bind loopback receiver socket";

    auto aircraft = make_aircraft();

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::RealTime;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 30.0;

    SimRunner runner;
    runner.initialize(cfg, *aircraft);

    UdpSimulationBroadcaster broadcaster(kRunnerTestPort);
    runner.set_broadcaster(&broadcaster);

    runner.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    runner.stop();

    // Drain any datagrams received in the 150 ms window.
    int datagram_count = 0;
    std::array<char, 4096> buf{};
    while (true) {
        const int r = static_cast<int>(
            recv(recv_sock, buf.data(), static_cast<int>(buf.size()), 0));
        if (r <= 0) break;
        ++datagram_count;
    }
    close_socket(recv_sock);

    EXPECT_GE(datagram_count, 1)
        << "SimRunner did not broadcast any datagrams in 150 ms";
}

}  // namespace
}  // namespace liteaero::simulation
