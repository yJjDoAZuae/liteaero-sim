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

    // Called by TerrainLoader._ready() before any UDP packet arrives.
    // Sets the geodetic world origin used to convert absolute geodetic
    // coordinates in each datagram to a local ENU offset.
    void set_world_origin(double lat_rad, double lon_rad, double h_m);

protected:
    static void _bind_methods();

private:
    void _open_socket();
    void _close_socket();
    void _apply_frame(const uint8_t* data, int size);

    int broadcast_port_          = 14560;
    int max_datagrams_per_frame_ = 10;

    double world_origin_lat_rad_ = 0.0;
    double world_origin_lon_rad_ = 0.0;
    double world_origin_h_m_     = 0.0;
    bool   world_origin_set_     = false;

    // POSIX / WinSock UDP socket file descriptor; -1 when not open.
    int socket_fd_ = -1;
};

} // namespace godot
