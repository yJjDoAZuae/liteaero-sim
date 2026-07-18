# Godot Plugin — GDExtension Design Specification

Design authority for the Godot 4 plugin that receives `SimulationFrameProto` UDP
datagrams and drives the Godot scene. Closely related to
[`live_sim_view.md`](live_sim_view.md), which specifies the C++ simulation
broadcast path and overall live-sim architecture.

---

## Current State

A **GDScript placeholder** exists at `godot/addons/liteaero_sim/`:

| File | Status |
| --- | --- |
| `SimulationReceiver.gd` | Functional GDScript placeholder — hand-rolled proto3 wire-format parser |
| `TerrainLoader.gd` | Implemented in GDScript — no C++ needed (pure Godot API calls) |

`SimulationReceiver.gd` is self-documented as a placeholder. It works for
development but the hand-rolled binary parser is fragile: any addition to
`SimulationFrameProto` (e.g., new fields, nested messages) requires manual
parser updates and is undetected at compile time.

The GDExtension C++ implementation replaces `SimulationReceiver.gd` with a
class compiled against the protobuf C++ runtime, eliminating the fragile
binary parser. `TerrainLoader.gd` is pure Godot API and remains GDScript (see
[OQ-GP-1](#oq-gp-1--terrain-loader-language)).

---

## Motivation for GDExtension

| Concern | GDScript placeholder | GDExtension C++ |
| --- | --- | --- |
| Protobuf parsing | Hand-rolled wire-format decoder | `SimulationFrameProto::ParseFromArray()` — standard, type-safe |
| Schema evolution | Parser breaks silently on new fields | Unknown fields ignored by protobuf runtime; new required fields caught at compile time |
| Performance | GDScript overhead at 400 Hz datagram rate | Near-zero overhead; all parsing in native C++ |
| Type safety | `Dictionary` with untyped `float` values | Typed proto message struct |
| Maintainability | Parser must be updated in sync with `liteaerosim.proto` | `#include "liteaerosim.pb.h"` — automatically in sync |

---

## File Structure

```text
godot/
├── addons/
│   └── liteaero_sim/
│       ├── liteaero_sim.gdextension       ← plugin manifest (Godot reads this)
│       ├── SimulationReceiver.gd           ← replaced by C++ once GDExtension is built
│       ├── TerrainLoader.gd               ← remains GDScript (no C++ needed)
│       └── src/
│           ├── CMakeLists.txt             ← GDExtension shared library build
│           ├── register_types.hpp
│           ├── register_types.cpp         ← GDExtensionBool entry point + ClassDB registration
│           ├── SimulationReceiver.hpp
│           └── SimulationReceiver.cpp
└── scenes/
    └── World.tscn
```

Once the GDExtension is built and the shared library placed at the path declared
in `liteaero_sim.gdextension`, Godot loads it at startup and makes
`SimulationReceiver` available as a native node type. `SimulationReceiver.gd` is
then no longer attached to the scene node (the C++ type is used directly).

---

## Dependencies

| Dependency | Version | License | Integration method |
| --- | --- | --- | --- |
| [godot-cpp](https://github.com/godotengine/godot-cpp) | Must match Godot editor (4.6) | MIT | FetchContent (CMake) |
| protobuf | 3.21.12 | BSD-3-Clause | Conan — already in `conanfile.txt` |

`godot-cpp` provides the C++ bindings for the GDExtension API. It must be built
for the same Godot version as the editor being used. The version is determined by
the `config/features` line in `godot/project.godot` (currently `"4.6"`).

`godot-cpp` is fetched via CMake `FetchContent` using a tag that matches the
Godot editor version. It is not available in ConanCenter.

---

## Build System

### `godot/addons/liteaero_sim/src/CMakeLists.txt`

The build file is implemented at
[`godot/addons/liteaero_sim/src/CMakeLists.txt`](../../godot/addons/liteaero_sim/src/CMakeLists.txt).
It is the authoritative source; the sections below describe its structure.

**Enabled from the root** via:

```cmake
# Root CMakeLists.txt
option(LITEAERO_SIM_BUILD_GODOT_PLUGIN
    "Build the GDExtension C++ plugin for Godot 4" OFF)

if(LITEAERO_SIM_BUILD_GODOT_PLUGIN)
    add_subdirectory(godot/addons/liteaero_sim/src)
endif()
```

**Version parsing** — derives the `godot-cpp` git tag from
`godot/project.godot` automatically (see [OQ-GP-2](#oq-gp-2--godot-cpp-version-tracking)
for the full failure-mode table).

**Library target** — `liteaero_sim_gdext` shared library, linking `godot-cpp`
and `protobuf::libprotobuf`, with static winpthread on MinGW, output to
`godot/addons/liteaero_sim/bin/` (excluded from version control).

**GP-1 gate** — if the C++ source files (`register_types.cpp`,
`SimulationReceiver.cpp`) are not present, CMake halts with a clear message
pointing to the GP-1 roadmap item before any library target is defined.

---

## `.gdextension` Manifest

**File:** `godot/addons/liteaero_sim/liteaero_sim.gdextension`

```ini
[configuration]
entry_symbol = "liteaero_sim_init"
compatibility_minimum = "4.1"

[libraries]
windows.debug.x86_64   = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
windows.release.x86_64 = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
```

The manifest path is relative to the Godot project root (`godot/`), so
`res://addons/liteaero_sim/bin/` maps to
`godot/addons/liteaero_sim/bin/` on disk.

---

## `register_types` Entry Point

**Files:** `register_types.hpp`, `register_types.cpp`

```cpp
// register_types.hpp
#pragma once
#include <godot_cpp/core/class_db.hpp>

void initialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
void uninitialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
```

```cpp
// register_types.cpp
#include "register_types.hpp"
#include "SimulationReceiver.hpp"
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
        return;
    ClassDB::register_class<SimulationReceiver>();
}

void uninitialize_liteaero_sim(ModuleInitializationLevel p_level) {
    (void)p_level;
}

extern "C" GDExtensionBool GDE_EXPORT liteaero_sim_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    const GDExtensionClassLibraryPtr p_library,
    GDExtensionInitialization *r_initialization)
{
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_liteaero_sim);
    init_obj.register_terminator(uninitialize_liteaero_sim);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
```

---

## `SimulationReceiver` Class

Post-LS-T8 the receiver is a thin pass-through: it parses
`SimulationFrameProto`, reads `viewer_x_m / viewer_y_m / viewer_z_m`
(fields 14–16) directly into `Vehicle.position`, and applies the
body-to-Godot quaternion.  The world origin and curvature-aware ECEF→ENU
math live simulation-side in
[`liteaero::projection::GodotEnuProjector`](../../include/projection/GodotEnuProjector.hpp);
the receiver is unaware of the origin.

### Header — `SimulationReceiver.hpp`

```cpp
#pragma once
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

    // Exported properties — visible in the Godot Inspector.
    void set_broadcast_port(int port);
    int  get_broadcast_port() const;

    void set_max_datagrams_per_frame(int n);
    int  get_max_datagrams_per_frame() const;

    // Read-only properties — HUD display values updated each received frame.
    double get_height_msl_m() const;
    double get_agl_m() const;

protected:
    static void _bind_methods();

private:
    void _open_socket();
    void _close_socket();
    void _apply_frame(const uint8_t* data, int size);

    int    broadcast_port_          = 14560;
    int    max_datagrams_per_frame_ = 10;

    int    socket_fd_               = -1;   // POSIX / WinSock UDP socket, non-blocking
    double latest_height_msl_m_     = 0.0;
    double latest_agl_m_            = 0.0;
};

} // namespace godot
```

### `_bind_methods()` — Inspector properties and scripting API

```cpp
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

    ClassDB::bind_method(D_METHOD("get_height_msl_m"),
                         &SimulationReceiver::get_height_msl_m);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height_msl_m"),
                 "", "get_height_msl_m");

    ClassDB::bind_method(D_METHOD("get_agl_m"),
                         &SimulationReceiver::get_agl_m);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "agl_m"),
                 "", "get_agl_m");
}
```

### `_ready()` — Socket initialization

Open a non-blocking UDP socket and bind to `127.0.0.1:<broadcast_port>`.

```cpp
void SimulationReceiver::_ready() {
    // GDExtension _ready() is called in the Godot editor as well as at runtime.
    // Opening a socket in the editor causes the game's bind() to silently fail.
    if (Engine::get_singleton()->is_editor_hint()) return;
    _open_socket();
}
```

The `is_editor_hint()` guard is mandatory. Without it, GDExtension `_ready()` runs
when the scene is opened in the editor, binding the socket there. When the game then
starts, the second `bind()` silently fails and no datagrams are received.

The same guard must be applied to `_process()`:

```cpp
void SimulationReceiver::_process(double /*delta*/) {
    if (Engine::get_singleton()->is_editor_hint()) return;
    if (socket_fd_ < 0) return;
    // ... polling loop ...
}
```

Socket implementation uses `socket()` / `bind()` / `ioctlsocket()` (Windows) or
`fcntl()` (POSIX) to set `O_NONBLOCK`. Must handle `WSAEWOULDBLOCK` /
`EAGAIN` gracefully in `recvfrom()`.

### `_process()` — Datagram poll and transform update

```cpp
void SimulationReceiver::_process(double /*delta*/) {
    if (socket_fd_ < 0) return;

    uint8_t buf[512];
    int frames = 0;
    while (frames < max_datagrams_per_frame_) {
        int n = recvfrom(socket_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) break;
        _apply_frame(buf, n);
        ++frames;
    }
}
```

### `_apply_frame()` — Protobuf deserialization and transform application

```cpp
void SimulationReceiver::_apply_frame(const uint8_t* data, int size) {
    liteaero::simulation::SimulationFrameProto frame;
    if (!frame.ParseFromArray(data, size)) return;

    // Viewer position computed simulation-side by GodotEnuProjector (ENU from world
    // origin).  Godot axes: X=East, Y=Up, Z=-North — matches viewer_x/y/z_m convention.
    get_parent()->set("position", Vector3(
        static_cast<float>(frame.viewer_x_m()),
        static_cast<float>(frame.viewer_y_m()),
        static_cast<float>(frame.viewer_z_m())));

    latest_height_msl_m_ = frame.height_msl_m();
    latest_agl_m_        = frame.agl_m();

    // Body-to-NED quaternion -> body-to-Godot quaternion.
    // NED->Godot rotation matrix (from live_sim_view.md §Coordinate System):
    //   [ 0  1  0 ]
    //   [ 0  0 -1 ]
    //   [-1  0  0 ]
    // Godot Quaternion constructor is (x, y, z, w) — not (w, x, y, z).
    // Quaternion for this matrix: x=0.5, y=0.5, z=-0.5, w=0.5.
    Quaternion r_ned_to_godot(0.5f, 0.5f, -0.5f, 0.5f);
    // Proto fields q_x/q_y/q_z/q_w map to Godot (x, y, z, w) — not (w, x, y, z).
    Quaternion q_b2n(frame.q_x(), frame.q_y(), frame.q_z(), frame.q_w());
    q_b2n = q_b2n.normalized();
    get_parent()->set("quaternion", (r_ned_to_godot * q_b2n).normalized());
}
```

---

## TerrainLoader Integration

`TerrainLoader.gd` performs three tasks in `_ready()`: load terrain, load the
aircraft mesh, and apply appearance materials. All mesh loading is driven by
`terrain_config.json`, whose path is passed via Godot command-line user args
(`OS.get_cmdline_user_args()`).

### Scene Lighting

`World.tscn` uses a `WorldEnvironment` node with ambient light plus two
`DirectionalLight3D` nodes:

| Light | Shadow | Energy | Purpose |
| --- | --- | --- | --- |
| Primary (sun) | Yes | 1.5 | Casts aircraft shadow toward camera |
| Fill | No | 1.5 | Illuminates aft/underside faces for mesh readability |

The shadow light is positioned so its shadow falls in front of the aircraft in the
camera frame, not behind it. `directional_shadow_max_distance = 100.0` and
`directional_shadow_mode = 0` (orthogonal) ensure the shadow is visible at typical
low-altitude test altitudes.

### Appearance — Aircraft, Terrain, Sky

`TerrainLoader` applies three independent, runtime-adjustable colour grades — to the
aircraft mesh, the terrain, and the sky background. Their values come from the **viewer
appearance config** (below), so the look is data-driven rather than hard-coded, and no
terrain build is required to change it.

**Aircraft mesh** — a `StandardMaterial3D` (`_aircraft_material`) whose `albedo_color` is
modulated from the base colour `#4A7FC1` (PP-F25) by `aircraft_brightness`,
`aircraft_contrast`, `aircraft_saturation`, and `aircraft_transparency`
(`_update_aircraft_color()`), applied via `material_override` with `cast_shadow = ON`.

**Terrain** — each terrain tile carries its own glTF albedo texture, so a single shared
material cannot grade them. Instead every tile `MeshInstance3D` is wrapped in a
`ShaderMaterial` `material_override` running
[`addons/liteaero_sim/terrain_grade.gdshader`](../../godot/addons/liteaero_sim/terrain_grade.gdshader):
the tile's own texture is bound to the `albedo_tex` uniform and the grade is a per-channel
**affine** transform — `out = albedo * gain + offset` — followed by `contrast` and
`saturation`. The additive `offset` (unlike a multiplicative tint) can introduce colour the
imagery lacks. The shader writes graded `ALBEDO` (a lit `spatial` shader with `cull_back`),
so scene lighting and shadows are unchanged from the source `StandardMaterial3D`. Wrapping
happens as each chunk loads (`_apply_terrain_grade()`), so streamed-in tiles are graded too;
a value change re-pushes the uniforms to every wrapped tile (`_update_terrain_grade()`).

**Sky** — the sky dome colours are set **absolutely** from `sky_top_color` (zenith) and
`sky_horizon_color` on the scene `ProceduralSkyMaterial` (`_update_sky()`); the ground half
keeps its scene baseline (captured once in `_capture_sky()`). `sky_brightness` and
`sky_saturation` are then applied over all four as overall multipliers. Unlike the terrain
tint (a multiplicative filter over imagery), the sky colours are direct — set the sky to any
colour, including hues the default gradient lacks.

Grade operation order: terrain — affine (`albedo * gain + offset`) → contrast (scale about
mid-grey 0.5) → saturation (BT.709 luminance lerp); aircraft — brightness → contrast →
saturation (StandardMaterial3D albedo modulation); sky — absolute colour → brightness →
saturation.

| Property | Range | Default | Target |
| --- | --- | --- | --- |
| `terrain_gain` | Color | white [1,1,1] | terrain — affine multiplicative factor |
| `terrain_offset` | Color | black [0,0,0] | terrain — affine additive offset (may be negative) |
| `terrain_contrast` / `terrain_saturation` | 0.0–3.0 | 1.0 | terrain |
| `sky_top_color` / `sky_horizon_color` | Color | Godot sky defaults | sky (absolute) |
| `sky_brightness` / `sky_saturation` | 0.0–3.0 | 1.0 | sky |
| `aircraft_brightness` / `aircraft_contrast` / `aircraft_saturation` | 0.0–2.0 | 1.0 | aircraft mesh |
| `aircraft_transparency` | 0.0–1.0 | 1.0 | aircraft mesh |

These are `@export` properties on `TerrainLoader` (so they are also editable in the editor
Inspector during Play), but their runtime values come from the viewer config.

### Viewer Appearance Config

Appearance values live in a standalone JSON config — [`configs/viewer.json`](../../configs/viewer.json)
— read by `TerrainLoader._apply_viewer_config()` at startup. The path is passed as the
`--viewer-config <path>` Godot user arg (added by `run_sim.sh` alongside `--terrain`); if
the arg is absent or the file is missing/malformed, the exported defaults are used and the
viewer still runs. The config is **independent of the terrain build** — edit it and relaunch
to change the look; a terrain rebuild never overwrites it.

```json
{
    "schema_version": 1,
    "terrain": { "gain": [1.0, 1.0, 1.0], "offset": [0.0, 0.0, 0.0], "contrast": 1.0, "saturation": 1.0 },
    "sky":     { "top_color": [0.385, 0.454, 0.55], "horizon_color": [0.6463, 0.6558, 0.6708], "brightness": 1.0, "saturation": 1.0 }
}
```

Colors/vectors are `[r, g, b]`. Terrain `gain`/`offset` are the affine factors (`offset` may be
negative); `sky` `top_color`/`horizon_color` are absolute. Scalars are multipliers with
1.0 = source. Values are applied before terrain loads, so streamed tiles are wrapped with the
configured grade.

### HUD

`SimulationReceiver` draws a corner HUD updated per received frame: airspeed (`SPD`, kt),
MSL altitude (`ALT`, ft), height above terrain (`AGL`, ft), and vertical speed (`V/S`,
ft/s, climb positive). Vertical speed is read from the telemetry `velocity_down_mps`
(proto field 11); it requires no simulation rebuild as the broadcaster already sends it.

### Aircraft mesh loading

The aircraft mesh path is stored in `terrain_config.json` as `aircraft_mesh_path`
(a `res://` path). `TerrainLoader.gd` reads this field and instantiates the mesh
as a child of the `Vehicle` node at runtime. No static `AircraftMesh` node exists
in `World.tscn` — the mesh is fully programmatic.

#### Coordinate frame correction

Aircraft GLBs are generated in the **layout frame**: +X = aft (increasing fuselage
station), +Y = right (starboard buttline), +Z = up (waterline). Godot's GLTF
importer reads vertex positions as-is without applying any axis conversion.

`TerrainLoader` applies `mesh_node.rotation_degrees = Vector3(0, 180, 0)`.
This is a pure Ry(180°) rotation, which maps layout→body (NED body): +X(aft)→-X(fwd),
+Y unchanged (right stays right), +Z(up)→-Z(down). The resulting rotation matrix is:

$$R_\text{correction} = R_y(180°) = \begin{bmatrix} -1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

This rotation is applied to all layout-frame aircraft meshes. It is correct for both
static display and live simulation:

| Layout mesh axis | After Ry(180°) | Physical meaning |
| --- | --- | --- |
| +X (aft) | −X | Forward (nose direction) |
| +Y (right) | +Y | Starboard (unchanged) |
| +Z (up) | −Z | Down (NED body convention) |

The corrected mesh local frame (+X=fwd, +Y=right, +Z=down) matches the NED body frame
used by `q_b2n`, so no residual attitude error occurs during live simulation.

**Pitfall — smooth shading:** Godot's GLTF importer shares vertices across faces and
interpolates normals, producing rounded edges even on prismatic geometry. Aircraft GLBs
must embed unshared vertices with explicit per-face normals to prevent this. In trimesh:

```python
flat_verts = mesh.vertices[mesh.faces].reshape(-1, 3)
flat_faces = np.arange(len(flat_verts), dtype=np.int32).reshape(-1, 3)
face_normals = np.repeat(mesh.face_normals, 3, axis=0)
mesh = trimesh.Trimesh(vertices=flat_verts, faces=flat_faces,
                       vertex_normals=face_normals, process=False)
```

If a mesh is updated, delete `godot/.godot/imported/` cached `.scn` files — Godot
will not re-import from a stale import cache.

#### Wingspan scaling

`TerrainLoader` scales the mesh to real-world dimensions using `aircraft_wingspan_m`
from `terrain_config.json`. The wingspan spans local Y (the right-wing axis as stored
in the GLB); `aabb.size.y` (from the first `MeshInstance3D` child) gives the mesh's
unscaled span. The uniform scale factor `target_wingspan_m / aabb.size.y` is applied
before the AABB is used for positioning.

`aircraft_wingspan_m` is written by `build_terrain.py` as `sqrt(S_ref_m2 * ar)` from
the aircraft config's `aircraft` section. If the field is absent, no scaling is
applied and the mesh is displayed at whatever size the GLB was generated with.

#### terrain_config.json aircraft fields

| Field | Type | Written by | Description |
| --- | --- | --- | --- |
| `aircraft_mesh_path` | string | `build_terrain.py` | `res://` path to the aircraft GLB |
| `aircraft_wingspan_m` | float | `build_terrain.py` | Real wingspan in metres: `sqrt(S_ref_m2 * ar)` |

`aircraft_mesh_path` is derived from `visualization.mesh_res_path` in the aircraft
config (see [§Aircraft Config Visualization Section](#aircraft-config-visualization-section)).
`aircraft_mesh_path` is the only field in `terrain_config.json` that is safe to edit
by hand without a terrain rebuild — the world origin and GLB path must not be changed.

If either aircraft field is absent (e.g., generated by an older `build_terrain.py`),
`TerrainLoader` warns and continues — the simulation runs without a visible mesh or
with an unscaled mesh respectively.

### Aircraft config visualization section

The `visualization` section of the aircraft config JSON specifies the Godot mesh
asset to use for this aircraft category:

```json
"visualization": {
    "mesh_res_path": "res://assets/aircraft_lp.glb"
}
```

| Field | Type | Description |
| --- | --- | --- |
| `mesh_res_path` | string | Godot `res://` path to the aircraft mesh GLB. Must be a file present in the Godot project (i.e., under `godot/`). |

`build_terrain.py` reads `visualization.mesh_res_path` and writes it to
`terrain_config.json` as `aircraft_mesh_path`. If the field is absent from the
aircraft config, `build_terrain.py` uses the default `"res://assets/aircraft_lp.glb"`.

To swap the aircraft mesh for a test case without rebuilding terrain, edit
`data/terrain/<name>/terrain_config.json` directly and change `aircraft_mesh_path`.
This is the only field in `terrain_config.json` that is safe to edit by hand —
the world origin, GLB path, and las_terrain path must not be changed without a
terrain rebuild.

---

## WinSock Initialization (Windows)

On Windows, `WSAStartup()` must be called once before any socket operations.
The correct location is the GDExtension initialization function:

```cpp
void initialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
#ifdef _WIN32
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
    ClassDB::register_class<SimulationReceiver>();
}

void uninitialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
#ifdef _WIN32
    WSACleanup();
#endif
}
```

---

## Coordinate System

The broadcast convention and coordinate frames are specified in
[`live_sim_view.md §Coordinate System`](live_sim_view.md#coordinate-system).

After LS-T8, `SimulationReceiver` does not compute ENU offsets from geodetic
coordinates.  That computation is performed simulation-side by
[`GodotEnuProjector`](../../include/projection/GodotEnuProjector.hpp), which
broadcasts `viewer_x_m / viewer_y_m / viewer_z_m` as pre-computed Godot-space
ENU offsets from the terrain world origin.  The receiver applies them directly:

**Viewer position:** $\text{pos} = (\texttt{viewer\_x\_m},\ \texttt{viewer\_y\_m},\ \texttt{viewer\_z\_m})$

**ENU→Godot axes:** X = East, Y = Up, Z = −North

**NED → Godot frame rotation quaternion:** $q_r = (x=0.5,\ y=0.5,\ z=-0.5,\ w=0.5)$ (Godot constructor order: `Quaternion(x, y, z, w)`)

**Vehicle quaternion:** $q_\text{Godot} = q_r \otimes q_{b2n}$ (normalized)

The aircraft mesh correction rotation is a fixed `rotation_degrees = Vector3(0, 180, 0)`
applied by `TerrainLoader` to the aircraft mesh node — not applied by `SimulationReceiver`.

---

## Open Questions

### OQ-GP-1 — Terrain loader language

**Resolved — Option A (keep GDScript).**

`TerrainLoader` remains GDScript. The operations it performs (JSON read,
`ResourceLoader`, scene-tree traversal) are pure Godot API; there is no
correctness or performance argument for C++.

After LS-T8, `TerrainLoader` no longer calls `set_world_origin()` on
`SimulationReceiver`.  The cross-language API boundary between `TerrainLoader`
and the C++ `SimulationReceiver` has been removed, eliminating the previously
acknowledged maintainability concern.  Option A is confirmed as the appropriate
long-term choice unless the plugin grows beyond its current two components.

---

### OQ-GP-2 — godot-cpp version tracking

**Resolved — Option B (auto-parse `project.godot`).**

The `godot-cpp` GIT_TAG must match the Godot editor version exactly.
A mismatch causes Godot to print "GDExtension API mismatch" at startup and
disable the plugin entirely — a failure that is confusing without knowing to
look for it.

#### Options

##### Option A — Manual GIT_TAG in CMakeLists.txt

The developer edits the `GIT_TAG` line in `CMakeLists.txt` when upgrading the
Godot editor.

- Pro: No tooling. Simple to understand.
- Con: Relies entirely on developer discipline. No signal from the build system
  that the tag is stale. The mismatch symptom is a Godot runtime error, not a
  build error — meaning the mistake is not caught until the scene is opened.
- Con: Two places to update when the editor version changes: `project.godot`
  (done by the Godot editor automatically) and `CMakeLists.txt` (done manually).

##### Option B — Parse `project.godot` in CMake to derive the tag (recommended)

`godot/project.godot` already contains the authoritative version in the line:

```ini
config/features=PackedStringArray("4.6", "Forward Plus")
```

CMake reads this file, extracts the version string, and constructs the
`GIT_TAG` automatically:

The implemented CMake code handles five failure modes with informative
`FATAL_ERROR` messages:

| # | Failure mode | Detection | Message guidance |
| --- | --- | --- | --- |
| 1 | `project.godot` not found | `if(NOT EXISTS ...)` | Path to fix in CMakeLists |
| 2 | `config/features=` line missing | `file(STRINGS ... REGEX)` empty | Re-open project in Godot editor |
| 3 | No version token in features line | regex match fails | Open in stable Godot 4 |
| 4 | Non-stable version (rc/beta/dev) | digits-and-dots check | Switch to stable release |
| 5 | FetchContent fetch fails | pre-flight STATUS messages | Tag URL + network guidance |

For failure mode 5, CMake cannot validate that the derived tag exists before
attempting the fetch. Three STATUS messages are printed immediately before
`FetchContent_MakeAvailable` so that a git error ("revision not found",
"could not resolve host") is accompanied by the derived tag name, a link to
the godot-cpp tags page, and a note about the network-access requirement.

- Pro: Single source of truth. Upgrading the editor (which rewrites
  `project.godot`) automatically causes the next CMake configure to fetch
  the matching `godot-cpp` version. No manual step.
- Pro: Mismatch is structurally impossible — the version in `project.godot` and
  the version used to build the plugin are the same by construction.
- Con: Assumes the `godot-N.M-stable` tag naming convention holds. This is true
  for all stable releases. Release candidates (e.g., `4.6-rc1`) are rejected
  at failure mode 4; the project tracks stable releases only.
- Patch releases (e.g., `4.6.1`) produce tag `godot-4.6.1-stable`, which is a
  valid godot-cpp tag and works correctly.

##### Option C — Explicit version file (`godot/godot_version.cmake`)

A separate `set(GODOT_EDITOR_VERSION "4.6")` file is included by CMakeLists.
Both files must be updated when the editor changes.

- Same manual discipline requirement as Option A, but with a cleaner separation.
- Still two places to update. No improvement over Option A in practice.

#### Recommendation

**Option B.** The version is already authoritatively declared in
`project.godot` by the Godot editor itself. Parsing it in CMake is five lines
of regex and eliminates the only manual synchronization step. The `*-rc*`
exception is not a concern for a project tracking stable releases.

---

### OQ-GP-3 — GDExtension build integration with main CMakeLists

**Resolved — Option B (`add_subdirectory` from root CMakeLists).**

The root `CMakeLists.txt` gains an option and a conditional `add_subdirectory`:

```cmake
option(LITEAERO_SIM_BUILD_GODOT_PLUGIN
    "Build the GDExtension C++ plugin for Godot 4" OFF)

if(LITEAERO_SIM_BUILD_GODOT_PLUGIN)
    add_subdirectory(godot/addons/liteaero_sim/src)
endif()
```

Defaulting to `OFF` because `godot-cpp` pulls in a large FetchContent dependency
that is irrelevant to C++ unit tests and Python-only workflows. Developers doing
Godot work set `-DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON` explicitly, consistent with
how `-DLITEAERO_SIM_BUILD_PYTHON_BINDINGS` was structured before it was made the
default.

The `godot/addons/liteaero_sim/bin/` output directory (where the `.dll` lands) is
added to `.gitignore`. `docs/installation/README.md` documents the configure flag.

---

## Decision Records

### GP-DR-1 — GDExtension over GDScript for `SimulationReceiver`

**Decision:** `SimulationReceiver` is implemented as a GDExtension C++ class.

**Rationale:** The primary function of `SimulationReceiver` is protobuf
deserialization. Doing this in GDScript requires a hand-rolled wire-format
decoder that must be maintained in sync with `liteaerosim.proto`. The C++
runtime handles all parsing including future field additions and is the correct
tool for the job. The GDScript placeholder serves until the GDExtension build is
added to the project.

### GP-DR-2 — `TerrainLoader` remains GDScript

**Decision:** `TerrainLoader` remains GDScript (OQ-GP-1 Option A).

**Rationale:** `TerrainLoader` performs no operations that require C++ — it reads
a JSON file, loads a GLB resource, and traverses the scene tree. All of these
are idiomatic GDScript operations. Moving it to C++ adds no correctness or
performance benefit.

After LS-T8, `TerrainLoader` no longer calls `set_world_origin()`, removing
the cross-language API boundary that was the previously acknowledged risk.

### GP-DR-3 — godot-cpp version parsed from `project.godot`

**Decision:** The `godot-cpp` FetchContent `GIT_TAG` is derived automatically
by parsing `godot/project.godot` in CMake (OQ-GP-2 Option B).

**Rationale:** `project.godot` is the authoritative Godot editor version source,
updated automatically by the editor on every upgrade. Parsing it eliminates the
only manual synchronization step between the editor version and the godot-cpp
build dependency.

### GP-DR-4 — GDExtension built via root CMakeLists `add_subdirectory`

**Decision:** The plugin is built under `LITEAERO_SIM_BUILD_GODOT_PLUGIN=OFF`
(default), enabled explicitly by developers doing Godot work (OQ-GP-3 Option B).

**Rationale:** Consistent with the project's pattern for optional build targets
(Python bindings). A single CMake configure covers the full project when the flag
is set; the large `godot-cpp` FetchContent is not pulled into CI or C++-only
builds.

---

## Roadmap

The GDExtension implementation is roadmap item **GP-1**. Prerequisite: protobuf
generated sources are available in the main build tree (already satisfied).
Predecessor: GDScript placeholder (already functional — no blocking dependency
on GP-1 for early live-sim testing).

GP-1 is not blocking for the first live-sim run. The GDScript placeholder is
sufficient for development. GP-1 should be implemented before the plugin is
considered production-quality.
