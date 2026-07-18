// SimulationReceiver.cpp — GDExtension UDP receiver implementation.
//
// Design authority: docs/architecture/godot_plugin.md §SimulationReceiver Class
//
// Receives SimulationFrameProto datagrams from live_sim and drives the parent
// Vehicle node transform each render frame via linear/slerp interpolation between
// the two most recently received sim frames.
//
// Coordinate mapping (design authority: live_sim_view.md §Coordinate System):
//   ENU -> Godot:  X = East, Y = Up, Z = -North
//   NED -> Godot rotation quaternion: Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5)

#include "SimulationReceiver.hpp"
#include "liteaerosim.pb.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/classes/canvas_layer.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cmath>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define closesocket    close
#endif

using namespace godot;

namespace {
    constexpr int k_recv_buf_size = 512;
} // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

SimulationReceiver::SimulationReceiver() = default;

SimulationReceiver::~SimulationReceiver() {
    _close_socket();
}

// ---------------------------------------------------------------------------
// ClassDB binding
// ---------------------------------------------------------------------------

void SimulationReceiver::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_broadcast_port", "port"),
                         &SimulationReceiver::set_broadcast_port);
    ClassDB::bind_method(D_METHOD("get_broadcast_port"),
                         &SimulationReceiver::get_broadcast_port);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "broadcast_port"),
                 "set_broadcast_port", "get_broadcast_port");

    ClassDB::bind_method(D_METHOD("set_max_datagrams_per_frame", "n"),
                         &SimulationReceiver::set_max_datagrams_per_frame);
    ClassDB::bind_method(D_METHOD("get_max_datagrams_per_frame"),
                         &SimulationReceiver::get_max_datagrams_per_frame);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_datagrams_per_frame"),
                 "set_max_datagrams_per_frame", "get_max_datagrams_per_frame");

    // LS-T8: world-origin awareness moved to the simulation side
    // (GodotEnuProjector); the receiver is no longer aware of the origin.
}

// ---------------------------------------------------------------------------
// Property accessors
// ---------------------------------------------------------------------------

void SimulationReceiver::set_broadcast_port(int port) { broadcast_port_ = port; }
int  SimulationReceiver::get_broadcast_port() const   { return broadcast_port_; }

void SimulationReceiver::set_max_datagrams_per_frame(int n) { max_datagrams_per_frame_ = n; }
int  SimulationReceiver::get_max_datagrams_per_frame() const { return max_datagrams_per_frame_; }

// ---------------------------------------------------------------------------
// Godot lifecycle
// ---------------------------------------------------------------------------

void SimulationReceiver::_ready() {
    if (Engine::get_singleton()->is_editor_hint())
        return;
    _open_socket();
    _create_hud();
}

void SimulationReceiver::_process(double /*delta*/) {
    if (Engine::get_singleton()->is_editor_hint())
        return;
    if (socket_fd_ < 0)
        return;

    // Drain queued datagrams — each decoded frame shifts curr→prev and
    // stores the new frame in curr, so after the loop frame_prev_ and
    // frame_curr_ are the two most recent sim frames.
    uint8_t buf[k_recv_buf_size];
    int datagrams = 0;
    while (datagrams < max_datagrams_per_frame_) {
        int n = static_cast<int>(
            recvfrom(socket_fd_, reinterpret_cast<char*>(buf),
                     static_cast<int>(sizeof(buf)), 0, nullptr, nullptr));
        if (n <= 0)
            break;
        _decode_frame(buf, n);
        ++datagrams;
    }

    if (!frame_curr_.valid)
        return;

    // Interpolate between frame_prev_ and frame_curr_ based on current
    // wall time.  t=0 → frame_prev_, t=1 → frame_curr_, t>1 extrapolates
    // (clamped to 1 to avoid runaway when no new packets arrive).
    double t = 1.0;
    if (frame_prev_.valid) {
        const double dt_frames = frame_curr_.wall_time_s - frame_prev_.wall_time_s;
        if (dt_frames > 1e-6) {
            const double now_s = static_cast<double>(
                Time::get_singleton()->get_ticks_usec()) * 1e-6;
            t = (now_s - frame_prev_.wall_time_s) / dt_frames;
            // Clamp: never extrapolate past curr, allow up to 1 frame behind.
            t = Math::clamp(t, 0.0, 1.0);
        }
    }

    const Vector3    pos = frame_prev_.valid
        ? frame_prev_.position.lerp(frame_curr_.position, static_cast<float>(t))
        : frame_curr_.position;
    const Quaternion rot = frame_prev_.valid
        ? frame_prev_.rotation.slerp(frame_curr_.rotation, static_cast<float>(t))
        : frame_curr_.rotation;

    get_parent()->set("position",   pos);
    get_parent()->set("quaternion", rot);
    _update_hud();
}

// ---------------------------------------------------------------------------
// Socket helpers
// ---------------------------------------------------------------------------

void SimulationReceiver::_open_socket() {
    socket_fd_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (socket_fd_ == INVALID_SOCKET) {
        ERR_PRINT("SimulationReceiver: socket() failed");
        socket_fd_ = -1;
        return;
    }

    // Allow immediate rebind after Godot restarts (avoids EADDRINUSE / Error 10048).
    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    // Bind to loopback so only local traffic is received.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(broadcast_port_));
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        ERR_PRINT(String("SimulationReceiver: bind() failed on port ") +
                  String::num_int64(broadcast_port_));
        closesocket(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    // Set non-blocking so _process() returns immediately when no data arrives.
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(socket_fd_, FIONBIO, &mode);
#else
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
#endif
}

void SimulationReceiver::_close_socket() {
    if (socket_fd_ >= 0) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
    }
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------

void SimulationReceiver::_create_hud() {
    // Build the canvas + label fully off-tree first; then defer the single
    // root-attach call so it lands after the scene-tree initialization pass
    // finishes (Godot rejects add_child while the parent is mid-setup —
    // "Parent node is busy setting up children").
    CanvasLayer* canvas = memnew(CanvasLayer);
    canvas->set_name("HudOverlay");

    Label* label = memnew(Label);
    label->set_name("HudLabel");
    label->add_theme_font_size_override("font_size", 20);
    label->add_theme_color_override("font_color", Color(0.0f, 1.0f, 0.2f, 1.0f));
    // Anchor to top-right corner.
    label->set_anchor(SIDE_LEFT,   1.0f);
    label->set_anchor(SIDE_RIGHT,  1.0f);
    label->set_anchor(SIDE_TOP,    0.0f);
    label->set_anchor(SIDE_BOTTOM, 0.0f);
    label->set_offset(SIDE_LEFT,  -230.0f);
    label->set_offset(SIDE_RIGHT, -10.0f);
    label->set_offset(SIDE_TOP,    20.0f);
    label->set_offset(SIDE_BOTTOM, 140.0f);
    label->set_horizontal_alignment(HORIZONTAL_ALIGNMENT_RIGHT);
    canvas->add_child(label);
    hud_label_ = label;

    // Deferred: runs at the end of the current frame, when the root window's
    // child-setup pass is complete and add_child is again allowed.
    get_tree()->get_root()->call_deferred("add_child", canvas);
}

void SimulationReceiver::_update_hud() {
    if (hud_label_ == nullptr)
        return;

    const int spd_kt = static_cast<int>(latest_airspeed_mps_ * 1.94384f);
    // ALT is MSL (orthometric) — what pilots read off charts and altimeters.
    // The simulation populates height_msl_m via the EGM2008 geoid; it falls
    // back to h_WGS84 when no geoid grid is available.
    const int alt_ft = static_cast<int>(latest_height_msl_m_ * 3.28084f);

    String text = String("SPD  ") + String::num_int64(spd_kt) + " kt\n";
    text       += String("ALT  ") + String::num_int64(alt_ft) + " ft\n";
    if (latest_agl_m_ >= 0.f) {
        const int agl_ft = static_cast<int>(latest_agl_m_ * 3.28084f);
        text += String("AGL  ") + String::num_int64(agl_ft) + " ft\n";
    } else {
        text += String("AGL  ---\n");
    }
    // Vertical speed (ft/s, climb positive) from the NED down-velocity (proto field 11).
    const int vs_fps = static_cast<int>(Math::round(-latest_velocity_down_mps_ * 3.28084f));
    text += String("V/S  ") + (vs_fps >= 0 ? String("+") : String("")) +
            String::num_int64(vs_fps) + " fps";
    hud_label_->set_text(text);
}

// ---------------------------------------------------------------------------
// Frame decoding
// ---------------------------------------------------------------------------

void SimulationReceiver::_decode_frame(const uint8_t* data, int size) {
    las_proto::SimulationFrameProto frame;
    if (!frame.ParseFromArray(data, size)) {
        UtilityFunctions::print("SimulationReceiver: proto parse failed, size=", size);
        return;
    }

    // LS-T8: read the simulation-side viewer-projected position directly
    // (curvature-aware, computed by liteaero::projection::GodotEnuProjector
    // in the broadcaster).  This eliminates the visual/sim AGL mismatch that
    // came from this receiver applying a flat-Earth approximation while the
    // terrain GLB was placed via ECEF -> ENU.  See live_sim_view.md Issue 7.
    const Vector3 pos(frame.viewer_x_m(), frame.viewer_y_m(), frame.viewer_z_m());

    // Body-to-NED quaternion -> body-to-Godot quaternion.
    // NED->Godot frame rotation: Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5)
    const Quaternion r_ned_to_godot(0.5f, 0.5f, -0.5f, 0.5f);
    Quaternion q_b2n(frame.q_x(), frame.q_y(), frame.q_z(), frame.q_w());
    q_b2n = q_b2n.normalized();
    const Quaternion rot = (r_ned_to_godot * q_b2n).normalized();

    // Store latest HUD data from this received frame.
    latest_height_wgs84_m_    = frame.height_wgs84_m();
    latest_height_msl_m_      = frame.height_msl_m();
    latest_airspeed_mps_      = frame.airspeed_mps();
    latest_agl_m_             = frame.agl_m();
    latest_velocity_down_mps_ = frame.velocity_down_mps();

    // Shift curr -> prev, store new frame in curr.
    frame_prev_ = frame_curr_;
    frame_curr_.position    = pos;
    frame_curr_.rotation    = rot;
    frame_curr_.wall_time_s = static_cast<double>(
        Time::get_singleton()->get_ticks_usec()) * 1e-6;
    frame_curr_.valid       = true;
}
