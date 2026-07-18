#pragma once
// SimulationReceiver.hpp — GDExtension Node3D subclass that receives
// SimulationFrameProto UDP datagrams and drives the Vehicle node transform.
//
// Design authority: docs/architecture/godot_plugin.md §SimulationReceiver Class

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <cstdint>

namespace godot {

class Label;

class SimulationReceiver : public Node3D {
    GDCLASS(SimulationReceiver, Node3D)

public:
    SimulationReceiver();
    ~SimulationReceiver() override;

    void _ready() override;
    void _process(double delta) override;

    // Inspector-exported properties.
    void set_broadcast_port(int port);
    int  get_broadcast_port() const;

    void set_max_datagrams_per_frame(int n);
    int  get_max_datagrams_per_frame() const;

protected:
    static void _bind_methods();

private:
    void _open_socket();
    void _close_socket();
    void _decode_frame(const uint8_t* data, int size);
    void _create_hud();
    void _update_hud();

    // A decoded sim frame in Godot world space.
    struct GodotFrame {
        Vector3    position;
        Quaternion rotation;
        double     wall_time_s = 0.0;  // Time::get_ticks_usec() / 1e6 at receipt
        bool       valid       = false;
    };

    int broadcast_port_          = 14560;
    int max_datagrams_per_frame_ = 64;

    // Two most recently received frames for interpolation.
    // frame_prev_ is one sim step behind frame_curr_.
    GodotFrame frame_prev_;
    GodotFrame frame_curr_;

    // Latest received HUD data (not interpolated — display raw sim values).
    float latest_height_wgs84_m_    = 0.f;
    float latest_height_msl_m_      = 0.f;
    float latest_airspeed_mps_      = 0.f;
    float latest_agl_m_             = -1.f;  // -1 = no terrain
    float latest_velocity_down_mps_ = 0.f;   // NED down; HUD V/S = -this

    // HUD overlay label created programmatically in _ready().
    Label* hud_label_ = nullptr;

    // POSIX / WinSock UDP socket file descriptor; -1 when not open.
    int socket_fd_ = -1;
};

} // namespace godot
