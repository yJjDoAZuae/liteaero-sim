# Live Simulation Viewer — Architecture and Design

Design authority for the live simulation pipeline: the C++ `SimulationFrame` broadcast
path and the Godot 4 visualization scene that receives it.

Implemented by roadmap item LS-1. Blocking dependencies: SB-1 (Aircraft / SimRunner
Python bindings — required for `SimSession` scripted input path), MI-1 (`JoystickInput`,
`ScriptedInput`, `AircraftCommand`).

**SB-2 (ring buffer) is not a blocking dependency.** The MVP live sim rendering path is
`SimRunner → SimulationFrame → UDP → Godot 4`. The ring buffer is not in that path.
SB-2 is useful for Python-side logging and post-run analysis but is not required for LS-1.

**PP-2 is not a blocking dependency.** The Vispy post-processing infrastructure is not
used for live rendering.

---

## Purpose

The live simulation viewer closes the loop between the C++ simulation engine and a
real-time 3D display. `SimRunner` drives `Aircraft::step()` in `RealTime` mode; after
each step it populates a `SimulationFrame` value object and broadcasts it via UDP to
a fixed local port; a **Godot 4** scene receives the datagrams and updates the vehicle
actor transform and ribbon trail each render frame.

Godot 4 is the selected real-time renderer — see
[`docs/architecture/system/future/decisions.md`](system/future/decisions.md) decision
row 26 and [`docs/architecture/terrain.md §Game Engine Integration`](terrain.md#game-engine-integration).
Rationale: MIT license, native glTF 2.0 / GLB import, Vulkan-based renderer, C++
integration via GDExtension.

This is the first item in the roadmap where the full loop closes:

```text
Operator / Script
    | AircraftCommand (JoystickInput or ScriptedInput, via Python SimSession)
    v
SimRunner::runLoop()  (C++ background thread, RealTime mode)
    | Aircraft::step()  ->  KinematicState
    | populate SimulationFrame
    v
ISimulationBroadcaster::broadcast()
    | UDP datagram  ->  fixed local port (default 14560)
    v
Godot 4 GDExtension plugin
    | convert geodetic -> ENU offset from world origin
    | apply body-to-Godot rotation from quaternion
    v
Godot 3D scene  (terrain GLB tiles, aircraft mesh GLB, ribbon trail, HUD overlay)
```

---

## Use Case Decomposition

| ID | Use Case | Primary Actor | Mechanism |
| --- | --- | --- | --- |
| UC-LS1 | Run live simulation with joystick FBW input | Operator | CLI launcher; `JoystickInput` → `SimSession` → `SimRunner` |
| UC-LS2 | Run live simulation with scripted input | Developer | Notebook / CLI; `ScriptedInput.push()` from Python |
| UC-LS3 | Observe 3D trajectory in real time | Developer / Operator | Godot scene trailing camera tracks aircraft |
| UC-LS4 | Change camera mode during flight | Operator | Godot scene UI |
| UC-LS5 | Stop the simulation and close the window | Operator | Close Godot window; `SimSession.stop()` in Python |
| UC-LS6 | Launch from a Jupyter notebook | Developer | `SimSession` in notebook cell; Godot window launched separately |

---

## Architectural Issues and Required Corrections

The following C++ gaps block a correct live simulation implementation.

### Issue 1 — `SimulationFrame` and `ISimulationBroadcaster` are not yet implemented

`terrain.md §Live Simulation Streaming` specifies the `SimulationFrame` value object
and the `ISimulationBroadcaster` interface but neither is implemented. The required
new files are:

- `include/SimulationFrame.hpp` — plain value struct, no I/O
- `include/broadcaster/ISimulationBroadcaster.hpp` — pure interface
- `include/broadcaster/UdpSimulationBroadcaster.hpp`
- `src/broadcaster/UdpSimulationBroadcaster.cpp`

### Issue 2 — `SimRunner` does not call a broadcaster

`SimRunner::runLoop()` calls `aircraft_.step()` but does not populate or broadcast a
`SimulationFrame`. Required change: add `SimRunner::set_broadcaster(ISimulationBroadcaster*)`;
call `broadcaster_->broadcast(frame)` after each `step()` if a broadcaster is set.

### Issue 3 — `SimRunner.set_broadcaster()` is not bound to Python

Once Issue 2 is resolved in C++, `bind_runner.cpp` must expose
`SimRunner.set_broadcaster(broadcaster)` so Python can wire `UdpSimulationBroadcaster`
into the session.

### Issue 4 — `SimRunner.setManualInput()` is not exposed to Python; `JoystickInput` is only partially bound

`SimRunner::setManualInput(ManualInput*)` is not bound in `bind_runner.cpp`. `JoystickInput`
is bound only for its static `enumerate_devices()` method. See OQ-LS-5.

### Issue 5 — GDExtension C++ plugin not yet implemented (GDScript placeholder in use)

The Godot project (`godot/`) and a functional GDScript placeholder exist.
`SimulationReceiver.gd` implements the full UDP receive and coordinate transform
pipeline in GDScript with a hand-rolled proto3 wire-format parser. This is
sufficient for development but the hand-rolled parser is fragile — see the
detailed design specification in
[`docs/architecture/godot_plugin.md`](godot_plugin.md) for the GDExtension C++
implementation that replaces it. The GDScript placeholder is not blocking for
the first live-sim run.

### Issue 6 — Terrain DEM heights are orthometric (EGM96) but consumed as WGS84 ellipsoidal

**Status:** Open — root cause of "aircraft reacts at higher AGL than rendered
terrain shows" symptoms reported during live-sim flight tests.

The terrain build pipeline downloads Copernicus GLO-30 DEM via the Sentinel Hub
Process API. The evalscript in
[`python/tools/terrain/download.py`](../../python/tools/terrain/download.py)
documents the returned values explicitly:

```text
DEM — FLOAT32 elevation GeoTIFF (metres, MSL-adjusted via EGM96)
```

Copernicus GLO-30 publishes heights against the **EGM2008 geoid** (orthometric,
mean-sea-level). The Sentinel Hub `dem_2018` instance returns values referenced to
EGM96. Either way, the source values are **orthometric**, not ellipsoidal. This
contradicts the comment in
[`python/tools/terrain/geoid_correct.py`](../../python/tools/terrain/geoid_correct.py)
that claims "Copernicus DEM is already ellipsoidal."

Downstream the pipeline labels these heights as `height_wgs84_m`:

| Stage | File | Field name |
| --- | --- | --- |
| Triangulation | `triangulate.py` | `centroid_height_m` (passed as ellipsoidal to ECEF/ENU math) |
| Serialization | `las_terrain.py` | `centroid.height_wgs84_m` (metadata JSON) |
| C++ tile | `TerrainTile::centroid().height_wgs84_m` | (proto field name) |
| Elevation query | `TerrainMesh::elevation_m()` | returns `centroid.height_wgs84_m + interpolated_up_offset` |

The `apply_geoid_correction()` helper exists in `geoid_correct.py` and converts
orthometric → ellipsoidal heights via PROJ EPSG:9518 → EPSG:4979. **It is never
called.** A grep of `apply_geoid_correction` finds the definition, its tests, and
documentation references — but no invocation in `build_terrain.py` or any other
build-pipeline module.

`Aircraft.cpp:108` reads `initial_state.altitude_m` and stores it directly as
`setHeight_WGS84_m()`. The simulation's `KinematicState`, `SimulationFrame`, and
`SimulationFrameProto` all carry `height_wgs84_m` as ellipsoidal. So:

- **Aircraft state** is treated as **WGS84 ellipsoidal** throughout the simulation.
- **Terrain elevations** in `TerrainMesh` are **orthometric (EGM96)** but the field
  names assert ellipsoidal.

The geoid undulation $N$ at KSBA is approximately **−33 m** (geoid below ellipsoid).
The two datums therefore disagree by ~33 m at that location. Whether the
disagreement manifests as a visible AGL error depends on what the user types into
`initial_state.altitude_m`:

- If the user types the **chart MSL** elevation (typical aviator practice — KSBA
  is published as 13 ft MSL ≈ 4 m), the value is stored as `h_WGS84 = 4 m`, and the
  terrain DEM at that location reads ≈ 4 m orthometric (also stored as
  `h_WGS84 = 4 m`). The two values match numerically by accident; AGL = 0 on the
  runway. The labels are wrong, but the simulation works.
- If the user types the **true WGS84 ellipsoidal** height (≈ −29 m at KSBA), the
  aircraft is initialized 33 m below the terrain — `agl_m()` returns −33 m and the
  landing gear penetrates from the start.

**Required fix:** call `apply_geoid_correction()` in `build_terrain.py` between
the `mosaic_dem` step and the `triangulate` step, producing an ellipsoidal DEM.
After the fix, terrain heights truly are WGS84 ellipsoidal and the field names are
honest. The user must then supply `initial_state.altitude_m` as a WGS84 ellipsoidal
value — for KSBA this is ≈ −29 m, not the chart MSL value. A documentation note
or an `initial_state.altitude_msl_m` config field with automatic geoid lookup
would prevent surprises.

### Issue 7 — Earth curvature treated inconsistently between aircraft position and terrain placement

**Status:** Open — secondary contributor to visual/sim AGL mismatch, magnitude grows
with horizontal distance from world origin.

Terrain GLB tile placement uses **full ECEF→ENU math** in
[`python/tools/terrain/export_gltf.py`](../../python/tools/terrain/export_gltf.py)
(`_enu_offset_between` goes through ECEF). Per-vertex up-offsets within each tile
are likewise computed via ECEF in `triangulate.py:_geodetic_to_enu_vec`. Both are
curvature-aware: at 25 km horizontal distance the ECEF "up" component drops by
$d^2/(2R) \approx 49\ \text{m}$ relative to a flat-Earth approximation.

The aircraft side, by contrast, uses a **flat-Earth tangent-plane approximation**.
[`SimulationReceiver.cpp:_decode_frame`](../../godot/addons/liteaero_sim/src/SimulationReceiver.cpp)
and the GDScript placeholder both compute:

```cpp
double dlat    = lat_rad - world_origin_lat_rad_;
double dlon    = lon_rad - world_origin_lon_rad_;
double north_m = dlat * k_earth_radius_m;
double east_m  = dlon * k_earth_radius_m * std::cos(world_origin_lat_rad_);
double up_m    = h_WGS84_m - world_origin_h_m_;     // pure subtraction, no curvature
```

The aircraft `up_m` is a flat vertical subtraction; no curvature correction is
applied. So the rendered aircraft Y position sits on the **tangent plane at the
world origin**, while the rendered terrain sits on the **WGS84 ellipsoid** (via the
ECEF math in the GLB). The two surfaces diverge with horizontal distance from
origin:

| Distance from world origin | Curvature drop $d^2/(2R)$ |
| --- | --- |
| 5 km | 2 m |
| 10 km | 8 m |
| 25 km | 49 m |
| 50 km | 196 m |
| 100 km | 785 m |

The aircraft is rendered **above** where the terrain actually sits (the rendered
ground appears to drop away as the aircraft flies away from origin), while the
simulation's `Aircraft::agl_m()` is curvature-correct because it never goes
through `world_origin` — it queries `terrain.elevation_m()` at the aircraft's own
lat/lon and subtracts from `h_WGS84`. Result: at 25 km from origin the simulation
says `AGL = 0` (gear-contact triggered) when the visual scene shows the aircraft
~49 m above the rendered terrain. This matches the symptom report.

`TerrainMesh::elevation_m()` itself uses tile-local tangent-plane math
(`return centroid.height_wgs84_m + u*v0.up_m + v*v1.up_m + w*v2.up_m`) which
combines a tile-centroid-frame ENU offset with the centroid's absolute height.
This produces a small in-tile curvature error (~13 m at the corner of a 25 km
tile) but does not contribute the dominant 49 m+ visual mismatch — that comes
entirely from the aircraft side.

**Required fix:** make `SimulationReceiver` compute the aircraft Godot position via
the same ECEF→ENU transform used by `export_gltf.py`. The transform is exactly
`_enu_offset_between(aircraft_geodetic, world_origin_geodetic)` followed by the
glTF axis permutation `(east, up, −north)`. After the fix, the aircraft and
terrain share the same tangent-plane treatment and visual AGL matches simulation
AGL at all distances. The reference implementation already exists in
[`src/environment/TerrainMesh.cpp`](../../src/environment/TerrainMesh.cpp)
(`geodeticToEcef` + `ecefOffsetToEnu`).

---

## C++ Component Design

### `SimulationFrame`

**File:** `include/SimulationFrame.hpp`

Plain value struct in the Domain Layer. No I/O, no serialization machinery. Defined in
[`terrain.md §Live Simulation Streaming`](terrain.md#live-simulation-streaming).

```cpp
// include/SimulationFrame.hpp  (Domain Layer value object — no I/O)
namespace liteaero::simulation {

struct SimulationFrame {
    double timestamp_s;          // simulation time (s)
    double latitude_rad;         // vehicle geodetic latitude, WGS84 (rad)
    double longitude_rad;        // vehicle geodetic longitude, WGS84 (rad)
    float  height_wgs84_m;       // vehicle ellipsoidal altitude (m)
    float  q_w, q_x, q_y, q_z; // body-to-NED attitude quaternion
    float  velocity_north_mps;
    float  velocity_east_mps;
    float  velocity_down_mps;
};

} // namespace liteaero::simulation
```

### `ISimulationBroadcaster`

**File:** `include/broadcaster/ISimulationBroadcaster.hpp`

Pure interface in the Interface Layer. `SimRunner` depends on this interface, not on
any transport implementation.

```cpp
namespace liteaero::simulation {

class ISimulationBroadcaster {
public:
    virtual ~ISimulationBroadcaster() = default;
    virtual void broadcast(const SimulationFrame& frame) = 0;
};

} // namespace liteaero::simulation
```

### `UdpSimulationBroadcaster`

**File:** `include/broadcaster/UdpSimulationBroadcaster.hpp`,
`src/broadcaster/UdpSimulationBroadcaster.cpp`

UDP implementation of `ISimulationBroadcaster`. Serializes `SimulationFrame` as a
protobuf message and sends it as a UDP datagram to `127.0.0.1:<port>` (default port
14560). Uses a new `SimulationFrameProto` message added to `liteaerosim.proto`
(resolved by OQ-LS-7 Option B).

```cpp
namespace liteaero::simulation {

class UdpSimulationBroadcaster : public ISimulationBroadcaster {
public:
    explicit UdpSimulationBroadcaster(uint16_t port = 14560);
    ~UdpSimulationBroadcaster() override;

    void broadcast(const SimulationFrame& frame) override;

private:
    int socket_fd_;
    uint16_t port_;
};

} // namespace liteaero::simulation
```

### `SimRunner` modifications

Two additions to `SimRunner`:

1. `set_broadcaster(ISimulationBroadcaster* broadcaster)` — stores a non-owning pointer;
   pass `nullptr` to disable broadcasting. Called before `start()`.
2. After each `Aircraft::step()` call in `runLoop()`, if `broadcaster_ != nullptr`:
   populate a `SimulationFrame` from the current `KinematicState` and call
   `broadcaster_->broadcast(frame)`.

`SimRunner` does not own the broadcaster. The caller (Python session or C++ main) is
responsible for broadcaster lifetime; the broadcaster must outlive the `SimRunner` run.

---

## Python `SimSession` Design

**File:** `python/tools/live_sim_session.py`

`SimSession` owns the C++ objects for the lifetime of the simulation. It constructs
`Aircraft`, `SimRunner`, and `UdpSimulationBroadcaster` via pybind11 bindings, wires
them together, and starts the run. It does not own a display window.

```python
class SimSession:
    def __init__(
        self,
        aircraft_config: str,           # JSON string or file path
        mode: str = "realtime",
        dt_s: float = 0.02,
        duration_s: float = 0.0,        # 0 = run until stop()
        broadcast_port: int = 14560,
    ) -> None: ...
    # Constructs Aircraft, SimRunner, UdpSimulationBroadcaster.
    # Calls runner.set_broadcaster(broadcaster).

    def start(self) -> None: ...        # calls runner.start(); returns immediately
    def stop(self) -> None: ...         # calls runner.stop(); blocks until stopped
    def is_running(self) -> bool: ...

    def set_scripted_input(self, scripted: ScriptedInput) -> None: ...
    def set_joystick_input(self, joystick_config: str) -> None: ...

    @property
    def scripted_input(self) -> ScriptedInput: ...
```

`drain_kinematic()` is no longer required on `SimSession` for rendering — the Godot
scene receives state via UDP. It may still be exposed for logging and test purposes (the
ring buffer is still populated by `SimRunner`).

**Lifetime contract:** `SimSession` keeps `Aircraft`, `SimRunner`, and
`UdpSimulationBroadcaster` alive for its entire lifetime. `stop()` must be called before
`SimSession` is released.

---

## Godot Plugin Design

**Repository location:** `godot/` (Godot 4 project root)

The Godot plugin consists of two components:

| Component | Language | Status |
| --- | --- | --- |
| `SimulationReceiver` | GDScript placeholder → GDExtension C++ | Placeholder functional; GDExtension specified but not yet built |
| `TerrainLoader` | GDScript | Implemented |

Full design specification, file structure, build system, class API, and open
questions are in [`docs/architecture/godot_plugin.md`](godot_plugin.md).

### Scene hierarchy

```text
World (Node3D)
├── WorldEnvironment (ambient light, energy=0.1)
├── TerrainLoader (Node — TerrainLoader.gd)
│     instantiates terrain GLB tiles as children at runtime
├── DirectionalLight3D  (primary sun — shadow on, energy=1.5)
├── DirectionalLight3D  (fill — shadow off, energy=1.5, illuminates aft faces)
├── Vehicle (Node3D)
│   ├── SimulationReceiver (Node3D — GDExtension C++)
│   └── AircraftMesh (instanced by TerrainLoader from mesh_res_path GLB)
└── Camera3D
```

### `SimulationReceiver` behaviour per `_process()` frame

1. Read all pending UDP datagrams (non-blocking socket; up to `max_datagrams_per_frame`).
2. For the latest datagram: deserialize `SimulationFrameProto` (via protobuf C++ runtime
   in GDExtension; via hand-rolled parser in GDScript placeholder).
3. Compute ENU offset from world origin (set by `TerrainLoader._ready()`):

   ```text
   east_m  = (lon - lon_0) * R_earth * cos(lat_0)
   north_m = (lat - lat_0) * R_earth
   up_m    = height_wgs84_m - h_0
   ```

   Godot position: `Vector3(east_m, up_m, -north_m)`.
4. Convert body-to-NED quaternion to body-to-Godot quaternion using the
   NED→Godot rotation quaternion $(x=0.5,\ y=0.5,\ z=-0.5,\ w=0.5)$
   (Godot `Quaternion` constructor order is `(x, y, z, w)`).
5. Set `Vehicle.position` and `Vehicle.quaternion`.

### World origin

`TerrainLoader._ready()` reads `terrain_config.json` and sets the world origin
on `SimulationReceiver` before any UDP packet arrives. If terrain is not loaded,
`SimulationReceiver` logs an error on the first received frame.

---

## Coordinate System

The Godot scene uses ENU coordinates as specified in
[`terrain.md §Coordinate Frame Mapping`](terrain.md#coordinate-frame-mapping):

$$\text{Godot/glTF}(X,\ Y,\ Z) = \text{ENU}(+\text{East},\ +\text{Up},\ -\text{North})$$

| Frame | East | Up | −North | Notes |
| --- | --- | --- | --- | --- |
| ENU (terrain local grid) | +X | +Z | −Y | Domain Layer storage |
| glTF 2.0 canonical | +X | +Y | +Z | GLB export format |
| Godot 4 | +X | +Y | +Z | Same as glTF — no rotation needed for terrain |

### NED-to-Godot frame rotation

`SimulationFrame` carries a body-to-NED quaternion `q_b2n`. To drive the Godot vehicle
transform, the GDExtension plugin converts it to body-to-Godot:

```text
R_NED_to_Godot:
    NED +North -> Godot -Z    (North = -Z_Godot)
    NED +East  -> Godot +X    (East  = +X_Godot)
    NED +Down  -> Godot -Y    (Down  = -Y_Godot)
```

As a rotation matrix acting on NED column vectors:

```text
       [  0   1   0  ]
R =    [  0   0  -1  ]
       [ -1   0   0  ]
```

The `Vehicle` node's Godot quaternion is set by `SimulationReceiver`:

$$q_\text{Godot} = R_\text{NED\to Godot} \otimes q_{b2n}$$

The aircraft mesh is a child `AircraftMesh` node of `Vehicle` with a fixed local
rotation `R_\text{correction}` (see §Aircraft Mesh Coordinate Frame below).
The mesh correction is **not** folded into `q_Godot` — it is a static node rotation
applied once at scene setup by `TerrainLoader`.

### Aircraft Mesh Coordinate Frame

#### Three coordinate frames

There are three distinct frames that must be kept clearly separate:

**1 — Mesh layout frame** (vertices as stored in aircraft GLBs, generated from layout
drawings with fuselage station / buttline / waterline axes):

| Mesh layout axis | Physical direction |
| --- | --- |
| +X | Aft (increasing fuselage station) |
| +Y | Starboard (right buttline) |
| +Z | Up (waterline) |

Godot's GLTF importer reads vertex positions as-is; no axis conversion is applied.

**2 — NED body frame** (physics convention used by `SimulationFrame.q_b2n`):

| NED body axis | Physical direction |
| --- | --- |
| +X | Nose (forward / North in level flight) |
| +Y | Starboard (East in level flight) |
| +Z | Down |

At level, wings-level flight the body frame is aligned with NED:
body +X = North, body +Y = East, body +Z = Down.

**3 — Godot world frame:**

| Godot axis | Physical / ENU direction |
| --- | --- |
| +X | East |
| +Y | Up |
| +Z | −North |

#### Layout-to-body frame correction

`TerrainLoader` applies `mesh_node.rotation_degrees = Vector3(0, 180, 0)` to all
aircraft meshes. This is a pure Ry(180°) rotation:

$$R_\text{correction} = R_y(180°) = \begin{bmatrix} -1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

This maps layout axes to NED body axes:

| Layout mesh axis | After Ry(180°) | Physical meaning |
| --- | --- | --- |
| +X (aft) | −X | Forward (nose) |
| +Y (right) | +Y | Starboard (unchanged) |
| +Z (up) | −Z | Down (NED body convention) |

The corrected mesh frame (+X=fwd, +Y=right, +Z=down) is exactly the NED body frame,
so `q_b2n` from the simulation drives the mesh orientation correctly with no residual
attitude error.

#### Live simulation rotation chain

During simulation, `SimulationReceiver` sets:

$$q_\text{Vehicle} = R_\text{NED\to Godot} \otimes q_{b2n}$$

The `AircraftMesh` global rotation is:

$$R_\text{mesh,global} = q_\text{Vehicle} \cdot R_y(180°) = R_\text{NED\to Godot} \cdot q_{b2n} \cdot R_y(180°)$$

At level flight ($q_{b2n}$ = identity), the mesh +X axis (fwd in layout) maps to Godot
−Z (−North in world frame).
All three rotation axes (roll, pitch, yaw) have been verified with ±30° perturbations
and a compound 75°roll + 30°pitch test case using the `test_prism.glb` asset with
embossed axis labels.

---

## Implementation Pitfalls

The following issues were discovered during initial implementation and are documented
here to prevent regression.

### Godot `Quaternion` constructor argument order

Godot's `Quaternion` constructor takes `(x, y, z, w)` — **not** `(w, x, y, z)`.

```cpp
// WRONG — passes proto q_w as Godot x, producing identity mapped to (x=1,y=0,z=0,w=0):
Quaternion q_b2n(frame.q_w(), frame.q_x(), frame.q_y(), frame.q_z());

// CORRECT:
Quaternion q_b2n(frame.q_x(), frame.q_y(), frame.q_z(), frame.q_w());
```

The NED→Godot quaternion is similarly specified in Godot's (x, y, z, w) order:

```cpp
// x=0.5, y=0.5, z=-0.5, w=0.5  (NOT w=0.5, x=0.5, y=0.5, z=-0.5)
Quaternion r_ned_to_godot(0.5f, 0.5f, -0.5f, 0.5f);
```

### `is_editor_hint()` guard in GDExtension `_ready()` and `_process()`

GDExtension `_ready()` is called when the scene is opened in the Godot editor, not
only at runtime. If `SimulationReceiver::_ready()` opens a UDP socket without an
editor guard, the socket is bound in the editor session. When the game starts, the
second `bind()` silently fails and no datagrams are received.

Both `_ready()` and `_process()` must begin with:

```cpp
if (Engine::get_singleton()->is_editor_hint()) return;
```

### Initial attitude in aircraft config

`Aircraft.cpp` reads the following optional fields from `initial_state` in the
aircraft config JSON:

| Field | Unit | Default | Description |
| --- | --- | --- | --- |
| `roll_rad` | rad | 0.0 | Initial bank angle (positive = right wing down) |
| `pitch_rad` | rad | 0.0 | Initial pitch angle (positive = nose up) |
| `heading_rad` | rad | 0.0 | Initial heading (0 = North, positive = clockwise) |

The initial attitude quaternion is `q_nb = Rz(heading) · Ry(pitch) · Rx(roll)`
(ZYX Euler, standard aerospace convention). Example:

```json
"initial_state": {
    "roll_rad": 0.5236,
    "pitch_rad": 0.0,
    "heading_rad": 0.0
}
```

Test configs `test_axis_prism_roll30.json`, `test_axis_prism_pitch30.json`,
`test_axis_prism_yaw30.json`, and `test_axis_prism_roll75_pitch30.json` demonstrate
these fields. Landing gear is omitted from orientation test configs to prevent gear
spring forces from correcting the initial attitude.

---

## Launch Modes

### CLI — `python/tools/live_sim.py`

The Python CLI starts the simulation and UDP broadcaster. The Godot scene is launched
separately (either from the Godot editor or as a pre-built executable). See OQ-LS-8.

```bash
# Start simulation with scripted input (broadcasts to port 14560):
python tools/live_sim.py \
    --config aircraft_configs/ga_aircraft.json \
    --scripted \
    --dt 0.02

# With joystick:
python tools/live_sim.py \
    --config aircraft_configs/ga_aircraft.json \
    --joystick gx12_config.json
```

**Arguments:**

| Argument | Type | Description |
| --- | --- | --- |
| `--config` | path | Aircraft JSON config (required) |
| `--scripted` | flag | Use `ScriptedInput` with neutral commands (default if `--joystick` absent) |
| `--joystick` | path | `JoystickInput` JSON config; enables joystick mode |
| `--dt` | float | Simulation timestep in seconds (default `0.02`) |
| `--port` | int | UDP broadcast port (default `14560`) |

### Notebook — `python/live_sim_demo.ipynb`

```python
# Cell 1 — create and start session
from live_sim_session import SimSession

session = SimSession("aircraft_configs/ga_aircraft.json")
session.start()
# Godot scene already open and listening on port 14560

# Cell 2 — inject scripted commands
from liteaero_sim_py import AircraftCommand, ScriptedInput
scripted = ScriptedInput()
session.set_scripted_input(scripted)
scripted.push(AircraftCommand(n_z=1.0, throttle_nd=0.5))

# Cell 3 — stop
session.stop()
```

---

## Threading Model

```text
Python main thread (notebook / CLI)
└── SimSession: constructs C++ objects, calls runner.start()

C++ background thread (spawned by SimRunner.start())
└── runLoop():
        Aircraft::step()
        → populate SimulationFrame from KinematicState
        → ISimulationBroadcaster::broadcast()  (UDP send, non-blocking)
        → ChannelRegistry::publish()  (ring buffer, for logging)

Godot main thread (separate process)
└── _process() each render frame:
        SimulationReceiver::poll_socket()
        → update Vehicle transform
        → update RibbonTrail
        → update HudOverlay
```

The UDP send in `UdpSimulationBroadcaster::broadcast()` is a non-blocking `sendto()`
on a pre-opened socket. It does not block the simulation thread. Dropped datagrams
(e.g., when Godot is not running) are silently discarded.

---

## Test Strategy

**C++ tests** (`test/SimulationBroadcaster_test.cpp`):

| Test | Pass criterion |
| --- | --- |
| `test_simulation_frame_size` | `sizeof(SimulationFrame)` matches expected byte count |
| `test_udp_broadcaster_sends_datagram` | `UdpSimulationBroadcaster` sends a datagram readable by a loopback UDP receiver |
| `test_sim_runner_broadcasts_after_step` | After `runner.start()` + 0.1 s + `runner.stop()`, test UDP receiver has received ≥ 1 datagram |

**Python tests** (`python/test/test_live_sim_session.py`):

| Test | Pass criterion |
| --- | --- |
| `test_sim_session_initializes` | `SimSession(config)` constructs without error; `is_running()` is False |
| `test_sim_session_start_stop` | `start()` + `stop()` completes without deadlock; `is_running()` is False after stop |
| `test_sim_session_broadcasts` | After `start()` + 0.1 s sleep + `stop()`, a loopback UDP socket on port 14560 has received ≥ 1 datagram |
| `test_sim_session_datagram_size` | Received datagram size matches `sizeof(SimulationFrame)` |
| `test_scripted_input_wiring` | `set_scripted_input()` + `push(AircraftCommand(...))` does not raise |
| `test_cli_argument_parsing` | `argparse` namespace contains expected attributes for all argument combinations |

---

## File Map

| File | Role |
| --- | --- |
| `include/SimulationFrame.hpp` | `SimulationFrame` value struct |
| `include/broadcaster/ISimulationBroadcaster.hpp` | Pure broadcaster interface |
| `include/broadcaster/UdpSimulationBroadcaster.hpp` | UDP implementation header |
| `src/broadcaster/UdpSimulationBroadcaster.cpp` | UDP implementation |
| `test/SimulationBroadcaster_test.cpp` | C++ broadcaster tests |
| `python/tools/live_sim_session.py` | `SimSession` — Python scripted/notebook launcher |
| `python/tools/live_sim.py` | Python CLI launcher (scripted input only; no joystick) |
| `tools/live_sim.cpp` | C++ joystick + terrain launcher; loads `TerrainMesh` from `las_terrain_path` in `terrain_config.json`; calls `aircraft.setTerrain()` |
| `python/live_sim_demo.ipynb` | Notebook demo |
| `python/test/test_live_sim_session.py` | Python session tests |
| `python/assets/aircraft_lp.glb` | Low-poly aircraft mesh (323 triangles) |
| `godot/` | Godot 4 project root |
| `godot/addons/liteaero_sim/` | GDExtension plugin source |

---

## Open Questions

### OQ-LS-5 — `JoystickInput` wiring approach

**Resolved — Option A.**

`JoystickInput` is wired to `SimRunner` directly in C++. A `live_sim` C++ executable
(`src/tools/live_sim.cpp`) owns `Aircraft`, `SimRunner`, `JoystickInput`, and
`UdpSimulationBroadcaster`; wires them together; and blocks on the run loop. No Python
binding for `JoystickInput` is required or added. Python `SimSession` handles scripted
and notebook use only.

**Required new file:** `src/tools/live_sim.cpp` with a corresponding CMake executable
target. `JoystickInput` config path and device index are CLI arguments to the binary.

---

### OQ-LS-6 — Float precision of ring buffer position channels

**Deferred to SB-3 (ring buffer redesign).**

The `SB-2` ring buffer `Sample::value` is scalar `float`, which is inadequate not just
for geodetic position precision but as a general design — it cannot represent vectors or
quaternions atomically. The appropriate fix is a redesign of the channel type system
(roadmap item SB-3), not a patch to `SimRunner`. This question is not blocking for
LS-1: Godot receives `double` geodetic coordinates directly in `SimulationFrame` and
does not use the ring buffer.

---

### OQ-LS-7 — UDP datagram serialization format for `SimulationFrame`

**Resolved — Option B.**

`UdpSimulationBroadcaster` serializes `SimulationFrame` as a protobuf message.
A new `SimulationFrameProto` message is added to `liteaerosim.proto`:

```proto
message SimulationFrameProto {
    double timestamp_s         = 1;
    double latitude_rad        = 2;
    double longitude_rad       = 3;
    float  height_wgs84_m      = 4;
    float  q_w                 = 5;  // body-to-NED attitude quaternion
    float  q_x                 = 6;
    float  q_y                 = 7;
    float  q_z                 = 8;
    float  velocity_north_mps  = 9;
    float  velocity_east_mps   = 10;
    float  velocity_down_mps   = 11;
}
```

This extends the `TrajectoryFrame` schema (fields 1–8) with velocity fields (9–11).
The schema is self-describing, version-tolerant, and endian-safe. Protobuf serialization
overhead at 400 Hz is negligible (~microseconds per frame, < 20 KB/s). The Godot
GDExtension plugin receives and deserializes the message using the C++ protobuf runtime
(or the GDScript protobuf addon).

---

### OQ-LS-8 — Godot scene launch mechanism

**Resolved — Option C.**

Godot runs independently; Python does not manage the Godot process. The developer
opens the Godot scene once per session (from the editor or a pre-built export). The
Python CLI and C++ `live_sim` binary broadcast UDP regardless of whether Godot is
running — datagrams are silently dropped if no listener is present. This matches the
fire-and-forget UDP model naturally and requires no process management code.

---

### OQ-LS-9 — Aircraft mesh coordinate frame correction in Godot

**Resolved — layout-frame meshes with `rotation_degrees = Vector3(0, 180, 0)`.**

Aircraft meshes are generated in the **layout frame** (+X=aft, +Y=right, +Z=up).
`TerrainLoader` applies `mesh_node.rotation_degrees = Vector3(0, 180, 0)` — a pure
Ry(180°) — which maps the layout frame to the NED body frame (+X=fwd, +Y=right,
+Z=down). The corrected mesh frame matches `q_b2n` exactly, producing correct
attitude display for both static and live-simulation cases.

This supersedes the earlier empirical `Vector3(-90, 0, 90)` approach, which was
derived for a mesh assumed to be in a different convention and produced incorrect
live-simulation attitude. The correct rotation has been verified with roll, pitch,
yaw, and compound rotation test cases using `test_prism.glb`.

---

### OQ-LS-11 — Mesh Z-axis sign mismatch and live-simulation attitude error

**Resolved — root cause eliminated by adopting layout-frame mesh convention.**

The original problem arose because meshes were assumed to be in a body-like frame
(+X=nose, +Y=right, +Z=up) which differs from the NED body frame (+Z=down) in Z
sign, with no proper rotation able to reconcile them.

**Resolution:** All aircraft meshes are now defined in the **layout frame**
(+X=aft, +Y=right, +Z=up). The layout-to-body correction `Ry(180°)` maps this to
exactly the NED body frame (+X=fwd, +Y=right, +Z=down). There is no axis mismatch
and no residual attitude error in live simulation.

This question is closed. See OQ-LS-9 for the resolved correction rotation.

---

### OQ-LS-10 — Terrain wiring in `live_sim.cpp` for landing gear contact

**Resolved — Option B.**

> **Naming note:** The terrain abstract base class in `liteaero-flight` is currently
> named `V_Terrain` (`liteaero/terrain/V_Terrain.hpp`). That name violates the
> project's `PascalCase`-no-prefix naming convention. It is used below as the current
> code name only; a rename cleanup is required (see `liteaero-flight` roadmap).

`LandingGear::step()` requires a const reference to the terrain abstract base, passed
through `Aircraft::setTerrain()`. Without this call, `Aircraft::step()` skips the gear
contact block (`_terrain == nullptr` guard in `Aircraft.cpp`), producing incorrect
physics for any ground-start scenario. Terrain wiring must occur in C++ — not via
`SimSession` — because `Aircraft::setTerrain()` is not bound in the Python pybind11
layer.

**Option B — `TerrainMesh` from `.las_terrain` file.**

`live_sim.cpp` reads `terrain_config.json` (written by `build_terrain.py`) to locate
the `.las_terrain` binary; loads `TerrainMesh` from it; calls
`aircraft.setTerrain(terrain_mesh)`. If `terrain_config.json` is absent or the
`.las_terrain` file cannot be loaded, the binary exits with an error — there is no
silent fallback to `FlatTerrain`. A terrain build is a required precondition for
running `live_sim.exe`.

`live_sim.cpp` constructs the `TerrainMesh` before calling `runner.initialize()`, calls
`aircraft.setTerrain(terrain_mesh)`, and keeps the mesh alive for the duration of the
run. The mesh load may add several seconds of startup time for large datasets.

---

## Decision Records

### LS-DR-1 — Renderer for live simulation

**Decision:** Godot 4 (MIT license, Vulkan, native glTF import, GDExtension C++ API).

Decided during terrain mesh implementation (Step 11/12); documented in
[`terrain.md §Game Engine Integration`](terrain.md#game-engine-integration) and
[`decisions.md` row 26](system/future/decisions.md). The live simulation viewer is a
Godot scene receiving UDP `SimulationFrame` datagrams from the C++ `SimRunner`.
Vispy is used only for post-processing (`TrajectoryView`); it has no role in the live
viewer.

### LS-DR-2 — Transport mechanism and serialization format

**Decision:** UDP unicast to `127.0.0.1:14560` (default port configurable);
serialized as protobuf `SimulationFrameProto` (OQ-LS-7 Option B).

Fire-and-forget; dropped datagrams are silently discarded. The simulation thread is
never blocked by the renderer. Consistent with the transport model specified in
`terrain.md §Live Simulation Streaming`. Protobuf is used (not raw binary) for
endian-safety, version tolerance, and consistency with the rest of the project's
serialization strategy.

### LS-DR-3 — `SimulationFrame` field set

**Decision:** Use the field set specified in `terrain.md §Live Simulation Streaming`:
position (geodetic double), attitude (body-to-NED quaternion float), velocity NED
(float). This is sufficient for Godot position + orientation + velocity HUD display.

### LS-DR-4 — Python does not own the Godot window

**Decision:** `SimSession` is a pure simulation controller (starts/stops `SimRunner` +
broadcaster). It does not launch, monitor, or close the Godot process. The Godot window
is managed independently by the developer (OQ-LS-8 Option C).

### LS-DR-5 — `JoystickInput` wiring approach

**Decision:** Option A — C++ `live_sim` executable owns `JoystickInput` directly.

No Python binding for `JoystickInput` is required. `src/tools/live_sim.cpp` is the
operator-facing joystick launcher; Python `SimSession` handles scripted and notebook
use only. Clean separation: no SDL lifecycle management from Python.

### LS-DR-6 — Ring buffer drain rate

**Decision:** No longer relevant for live rendering — the ring buffer is not the
rendering data path. The ring buffer is populated at `1/dt_s` Hz by `SimRunner` for
logging and post-processing consumers. Ring buffer drain rate for those consumers is
determined by their own polling interval.

---

### OQ-LS-12 — Vertical datum convention for `initial_state.altitude_m`

**Resolved — Option A.** Aircraft state altitude is always WGS84 ellipsoidal.
`initial_state.altitude_m` is interpreted as WGS84 ellipsoidal height; the
canonical internal convention is unchanged from current behavior. Any MSL
height needed for instrumentation, charts, or atmosphere is produced by an
explicit conversion using the EGM2008 geoid undulation at the aircraft's
lat/lon (see OQ-LS-13 and OQ-LS-14). No `altitude_msl_m` config field is
introduced — config authors must supply ellipsoidal values. Update
[`Aircraft.cpp:108`](../../src/Aircraft.cpp) inline documentation and the
aircraft-config schema docs to state this convention explicitly.

**Background and rejected options retained below.**

After the geoid-correction step is added to `build_terrain.py`, the
`.las_terrain` file will hold true WGS84 ellipsoidal heights and the
`height_wgs84_m` field name will finally be honest. The aircraft side already
treats `initial_state.altitude_m` as ellipsoidal
([`Aircraft.cpp:108`](../../src/Aircraft.cpp)). The accidental numeric agreement
between chart-MSL altitudes and orthometric DEM heights goes away once the DEM
is corrected, so the user-facing convention must be settled.

**Option A — `altitude_m` means WGS84 ellipsoidal (no compatibility shims).**

The aircraft config simply requires the user to supply an ellipsoidal value.
Chart-MSL elevations must be converted by the user (e.g., via PROJ or a
pre-flight planning tool). Atmospheric model implications: see OQ-LS-14.

Pros: simplest implementation; matches the C++ field naming exactly; no
runtime geoid lookup; consistent with the project rule "no backward
compatibility, no schema iteration."

Cons: aviator-unfriendly. Pilots and chart consumers think in MSL; nobody
publishes ellipsoidal field elevations. Anyone building a config from real
airport data must do an external lookup (~33 m offset at KSBA, +47 m at KDEN,
−10 m at LFPG). Easy to get wrong silently.

**Option B — Add `altitude_msl_m` config field with one-time geoid lookup.**

Aircraft config accepts EITHER `altitude_m` (ellipsoidal, as today) OR
`altitude_msl_m` (orthometric / chart MSL). If the latter, `Aircraft.cpp`
performs a single PROJ geoid lookup at initialization and converts to
ellipsoidal before storing. Both fields must not be present simultaneously —
that is a config error.

Pros: aviator-friendly path is available; ellipsoidal path remains for
machine-generated configs; conversion is a one-shot at init (no per-frame
cost); the stored `KinematicState.position.altitude_m` continues to be true
WGS84 ellipsoidal so all internal math is unchanged.

Cons: introduces a PROJ dependency in the C++ aircraft initialization path
(currently PROJ is Python-only), or requires a custom EGM2008 lookup table
shipped with the build. Adds a config field and the rule that they're
mutually exclusive.

**Option C — Make `altitude_m` mean MSL throughout the simulation.**

Rename the field-but-not-yet — the simulation switches to MSL/orthometric
internally. Terrain queries return MSL. ECEF transforms add a per-call geoid
undulation lookup to convert to ellipsoidal where ECEF math is required (e.g.,
visualization-side ECEF→ENU).

Pros: most aviator-friendly. Atmosphere model gets the "right" input directly.

Cons: large refactor — `WGS84_Datum`, `KinematicState.position.altitude_m`,
proto field `height_wgs84_m`, and many call sites all need renaming. ECEF
math (e.g., `geodeticToEcef`) requires ellipsoidal; would now need a per-call
geoid lookup, paid every iteration. Increases the chance of subtle errors at
the MSL↔ellipsoidal boundary. Inconsistent with the WGS84-as-canonical
convention adopted across navigation, sensors, and proto serialization.

**Recommendation:** Option B. The aircraft config is the boundary where users
hand-author values, and that boundary should accept the convention humans use
(MSL). Internal sim state remains ellipsoidal — no per-frame geoid lookup, no
refactor of the existing WGS84 plumbing. Cost is one PROJ lookup at init plus
the EGM grid file.

---

### OQ-LS-13 — Geoid model and source-specific correction

**Resolved — per-source geoid for DEM correction; EGM2008 for in-simulation
runtime conversions.**

`build_terrain.py` selects the geoid model based on `--dem-source` (Option A
below): Copernicus GLO-30 → EGM2008, NASADEM → EGM96, SRTM → EGM96. Each
source is corrected with its native geoid so the conversion is exact within
the geoid model's own accuracy.

For all in-simulation runtime conversions — for example, the atmosphere
model's WGS84 → MSL conversion (OQ-LS-14) and any future instrumentation
that displays MSL altitude — the simulation uses **EGM2008 universally**.
This decouples runtime conversion from whatever geoid the underlying DEM
source happened to use; the simulation's internal geoid reference is fixed
and well-defined regardless of terrain provenance. EGM2008 is also the
highest-accuracy modern geoid widely available, and the residual difference
versus EGM96 (< 1 m almost everywhere) is below the simulation's noise floor.

The docstring in [`download.py`](../../python/tools/terrain/download.py)
that incorrectly claims "MSL-adjusted via EGM96" for Copernicus must be
corrected to "EGM2008" as part of this work item.

**Background and rejected options retained below.**

`apply_geoid_correction()` accepts `geoid="egm2008"` or `geoid="egm96"`. The
correct choice depends on the DEM source's published vertical datum, which
differs across the three sources currently supported by `--dem-source`:

| Source | Native vertical datum |
| --- | --- |
| Copernicus DEM GLO-30 (default) | EGM2008 (per Copernicus documentation) |
| NASADEM | EGM96 |
| SRTM v3 | EGM96 |

The docstring in [`download.py`](../../python/tools/terrain/download.py)
asserts "MSL-adjusted via EGM96" for Copernicus — this contradicts the
Copernicus product specification, which states heights are referenced to
EGM2008. The docstring is incorrect (likely copied from an SRTM-era
predecessor) and should be updated regardless of which option below is chosen.

**Option A — Per-source geoid model, hard-coded.**

`build_terrain.py` selects the geoid model based on `--dem-source`:

- `copernicus_dem_glo30` → `egm2008`
- `nasadem` → `egm96`
- `srtm` → `egm96`

Pros: matches each source's native datum; the conversion is exact (within the
geoid model's own accuracy).

Cons: tightly couples the build to current source assumptions; if Copernicus
ever republishes against a newer geoid (e.g., EGM2020 if/when it appears) the
table must be updated.

**Option B — Single project-wide geoid (EGM2008).**

Always use EGM2008. For sources that are natively EGM96, the conversion
introduces a small residual error (EGM2008 vs EGM96 differ by < 1 m almost
everywhere, < 5 m worst case at high latitudes).

Pros: simplest; only one PROJ grid file required; consistent across all
datasets in the project.

Cons: the EGM96-native sources are not exactly corrected — their published
"MSL=0" surface differs from EGM2008's by up to ~1 m. For a flight simulator
this is well below visual or physical resolution.

**Recommendation:** Option A. Per-source correctness is cheap (one extra
column in a small lookup table) and removes a known source of residual error.
The aircraft initialization geoid (OQ-LS-12 Option B) should also use EGM2008
for parity with Copernicus, the default DEM source.

---

### OQ-LS-14 — Atmospheric model height reference

**Resolved — Option B.** Convert `h_WGS84` → `h_MSL` inside `Aircraft::step()`
before each atmospheric query, using the EGM2008 geoid undulation at the
aircraft's current lat/lon (per OQ-LS-13). The conversion is physically
correct and removes a known systematic bias from atmospheric density,
pressure, and temperature outputs — and from any sensor or instrumentation
model that consumes them.

**Implementation approach:**

- Add a geoid-undulation lookup helper to the simulation domain (e.g.,
  `liteaero::geodesy::Egm2008Geoid::undulation(lat_rad, lon_rad)`). The
  underlying grid is the EGM2008 PROJ grid file (see OQ-LS-17 for grid
  distribution).
- `Aircraft::step()` computes
  `h_msl_m = state.position.height_WGS84_m() - undulation(lat, lon)`
  before invoking `Atmosphere::state(h_msl_m)`.
- Per-frame lookup cost is one bilinear interpolation on a small grid;
  negligible at simulation rates. If profiling later identifies hot-path
  cost, cache the value across consecutive frames whenever the aircraft
  moves less than a configurable horizontal threshold (the geoid is smooth;
  N varies < 0.1 m over typical between-frame motions).
- Existing `Atmosphere` API does not change — it continues to accept an
  altitude argument; the call site interprets the value as MSL.
- No protobuf or wire-format change is required.

**Background and rejected options retained below.**

`Atmosphere::density_kgm3(altitude_m)` and related queries take "altitude" as
input ([`Atmosphere.cpp`](../../src/environment/Atmosphere.cpp)). The ISA
standard is keyed to **MSL geopotential height**, not WGS84 ellipsoidal. The
simulation currently passes `KinematicState.position.altitude_m` (= h_WGS84)
into atmosphere queries.

The error is the geoid undulation N at the aircraft's lat/lon. At KSBA
N ≈ −33 m, giving a ~3 Pa pressure error and ~0.004 kg/m³ density error
(≈ 0.3 % at sea level). For a low-altitude trainer scenario this is
negligible — well below other ISA approximations. For high-altitude jet
operations it remains < 0.5 %.

**Decision pathway:**

- **Option A — Accept the error**, document it as a known approximation in
  `Atmosphere.cpp`. Recommended for the current implementation phase.
- **Option B — Convert h_WGS84 → h_MSL inside `Aircraft::step()`** before each
  atmospheric query, via per-frame PROJ geoid lookup. Adds runtime cost and
  PROJ dependency to the simulation hot path.
- **Option C — Pre-compute geoid undulation once per aircraft initialization**
  (assumes flight stays in a region small enough that N is approximately
  constant — generally true for general-aviation flights of < 100 km radius).

**Recommendation:** Option A for now (defer to a future open question that
considers atmosphere accuracy more broadly). Issues 6 and 7 do not depend on
this resolution.

---

### OQ-LS-15 — Where to perform ECEF→ENU for aircraft Godot position

**Resolved — Option B with strict architectural separation.** The broadcaster
side pre-computes the viewer-local position so the wire format carries
viewer-ready coordinates, and the receiver becomes a thin pass-through.
However, the projection logic must **not** be embedded into core simulation
domain code (`Aircraft`, `KinematicState`, `TerrainMesh`, `SimRunner`). It
lives in a dedicated projection module that the live-sim launcher composes
into the broadcaster. Other viewer back-ends (Unity, Unreal, web-based, etc.)
plug in by providing a different projector implementation; the simulation
domain is unaware of any of them.

**New module: `liteaero::projection`**

A new module is added under `include/projection/` and `src/projection/`
containing a viewer-projection abstraction:

```cpp
// include/projection/IViewerProjector.hpp  (Interface Layer)
namespace liteaero::projection {

struct ViewerOrigin {
    double lat_rad;
    double lon_rad;
    double height_wgs84_m;     // ellipsoidal
};

struct ViewerPosition {
    float x_m;
    float y_m;
    float z_m;
};

class IViewerProjector {
public:
    virtual ~IViewerProjector() = default;

    // Project a geodetic position (WGS84 ellipsoidal) into the viewer's
    // local 3D coordinate system relative to the configured world origin.
    virtual ViewerPosition project(double lat_rad,
                                   double lon_rad,
                                   float  height_wgs84_m) const = 0;
};

} // namespace
```

The Godot/glTF concrete implementation (`GodotEnuProjector`) performs full
ECEF→ENU using the WGS84 ellipsoid at the world origin's tangent plane,
then applies the glTF axis permutation `(x = east, y = up, z = -north)`.
Implementation reuses geodesy helpers — `geodeticToEcef` / `ecefOffsetToEnu`
from [`TerrainMesh.cpp`](../../src/environment/TerrainMesh.cpp) — extracted
to a shared `liteaero::geodesy` module so that both the terrain domain and
the projection module call identical math without one depending on the
other.

**Wire format change.**

`SimulationFrameProto` is extended with viewer-projected position fields:

```proto
// Already present (kept):
double timestamp_s        = 1;
double latitude_rad       = 2;
double longitude_rad      = 3;
float  height_wgs84_m     = 4;
// ... q_w/x/y/z, velocity_north/east/down_mps, airspeed_mps, agl_m
// New (added by this work item):
float  viewer_x_m         = 14;
float  viewer_y_m         = 15;
float  viewer_z_m         = 16;
```

The original geodetic fields are retained — they are the canonical state and
remain useful for non-Godot consumers. The viewer fields are an additional
convenience for the configured viewer back-end. Multiple back-ends with
different axis conventions cannot be served by the same broadcast in this
form (a single broadcaster has a single configured projector); a future
multi-viewer scenario would call for either separate broadcasters per viewer
or a more abstract handshake.

**Wiring.**

[`tools/live_sim.cpp`](../../tools/live_sim.cpp) reads
`world_origin_lat_rad / lon_rad / height_m` from the same `terrain_config.json`
it already consumes for the `.las_terrain` path (OQ-LS-10), constructs a
`GodotEnuProjector` with that origin, and passes it to the broadcaster:

```cpp
auto projector  = std::make_unique<projection::GodotEnuProjector>(origin);
auto broadcaster = std::make_unique<UdpSimulationBroadcaster>(port,
                                                              projector.get());
runner.set_broadcaster(broadcaster.get());
```

`UdpSimulationBroadcaster` invokes `projector->project(...)` once per frame
and writes the result into `viewer_x/y/z_m` before sending. The simulation
domain (`Aircraft`, `SimRunner`, `TerrainMesh`) never sees the projector or
the world origin.

**Receiver changes.**

`SimulationReceiver` (both C++ GDExtension and GDScript placeholder) reads
`viewer_x/y/z_m` directly into `Vehicle.position`. The flat-Earth
`east_m / up_m / north_m` computation in `_decode_frame()` is removed.
`set_world_origin()` is no longer needed by the receiver — only the
projector on the simulation side requires the origin. `TerrainLoader.gd`
stops calling `set_world_origin()` on the receiver. (It still reads the
origin for its own use, e.g., diagnostic prints, but no longer pushes it
across the scene tree.)

**Architectural compliance check.**

- Domain Layer (Aircraft, KinematicState, TerrainMesh, SimRunner): unchanged.
  No knowledge of viewer or world origin.
- Interface Layer: `liteaero::projection` (new); `UdpSimulationBroadcaster`
  composes a projector via interface pointer.
- Application Layer: `tools/live_sim.cpp` instantiates the concrete projector
  and wires it.
- Future viewers add a new concrete projector class; nothing else changes.

**Background and rejected options retained below.**

The terrain GLB places tiles via curvature-aware ECEF→ENU. The aircraft
position must be computed via the **same** transform for visual/sim AGL
parity. Three places this can happen:

**Option A — `SimulationReceiver` Godot-side (port the math to C++/GDScript).**

The GDExtension receiver computes ECEF→ENU itself, mirroring the helpers in
`triangulate.py:_geodetic_to_enu_vec` and `TerrainMesh.cpp:geodeticToEcef` +
`ecefOffsetToEnu`. The protocol on the wire is unchanged:
`SimulationFrameProto` continues to carry geodetic position. The world origin
arrives via `set_world_origin()` from `TerrainLoader._ready()`, exactly as
today.

Pros: the simulation is unaware of the Godot world origin (clean separation
of concerns); the wire format does not change; the same math already exists in
[`TerrainMesh.cpp`](../../src/environment/TerrainMesh.cpp) and can be lifted
verbatim into the receiver.

Cons: math now lives in two places (sim's `TerrainMesh` and Godot's
`SimulationReceiver`). Unit tests for the receiver are awkward (Godot
test harness).

**Option B — Broadcaster pre-computes ENU offsets and sends them in the
proto.**

`SimulationFrameProto` is extended with `east_m`, `up_m`, `north_m` fields
relative to the world origin. The C++ broadcaster computes these via
`TerrainMesh::geodeticToEcef` + ENU rotation. `SimulationReceiver` simply
copies them into the Godot position.

Pros: ECEF math lives in one place (the simulation side, in C++); Godot
receiver is trivial; can be unit-tested with the existing C++ test harness.

Cons: the simulation now needs to know the world origin. Currently the
simulation has no awareness of `world_origin_lat/lon/h_m` — it only knows the
aircraft's geodetic position. Wiring the world origin into `SimRunner` /
`UdpSimulationBroadcaster` couples the simulation to the rendering layer's
choice of origin, violating LS-DR-4's spirit (sim controller doesn't know
about display concerns).

**Option C — Hybrid: send both forms and let receiver pick.**

Wire carries geodetic AND ENU; receiver uses ENU. Worst of both worlds — wire
overhead plus the coupling concern.

**Recommendation:** Option A. Keep the wire format and simulation/render
boundary as they are; port the existing C++ ECEF transform from
`TerrainMesh.cpp` into `SimulationReceiver.cpp` (and update the GDScript
placeholder to match). Unit tests for the receiver math can live in a
standalone C++ test file that includes only `SimulationReceiver`'s helpers
(extracted to a free function).

---

### OQ-LS-16 — In-tile curvature correction in `TerrainMesh::elevation_m()`

**Resolved — Option A, defer.** With OQ-LS-15 resolved, visual / simulation
AGL parity is established at the broadcast boundary: the projector and the
terrain GLB both use the same ECEF-based projection from the same world
origin, so the rendered aircraft sits at exactly the same Y as the rendered
terrain whenever the simulation reports `AGL = 0`. The remaining in-tile
tangent-plane approximation in `TerrainMesh::elevation_m()` is purely a
sim-internal quantity — it affects what the simulation reports as AGL and
when landing-gear contact triggers, but it does **not** create a sim-vs-render
mismatch any more.

The residual error (~12 m worst-case at the corner of a 25 km LOD 4 tile,
< 1 m within the home-tile interior of typical AGL queries) is acceptable
for the current implementation phase. Document the approximation in the
`elevation_m()` docstring and revisit if/when a use case requires
sub-meter AGL accuracy at large tile distances.

**Background and rejected options retained below.**

`TerrainMesh::elevation_m()` returns
`centroid.height_wgs84_m + barycentric(up_offsets)`, treating the tile as a
flat tangent plane at its centroid. The tile-vertex up_offsets in
`.las_terrain` were computed via ECEF (`triangulate.py:_geodetic_to_enu_vec`),
so they are themselves curvature-aware relative to the centroid — but adding
them to `centroid.height_wgs84_m` to produce an "absolute height" implicitly
flattens the tile.

For a 25 km tile at LOD 4, vertices at the corner are ~12.5 km from the
centroid; the tangent-plane projection differs from the true ellipsoidal
height by $d^2/(2R) \approx 12\ \text{m}$. Worst case is at the tile corner;
typical query points fall well inside.

**Options:**

- **A — Defer.** Document the approximation in `elevation_m()` and accept
  ~12 m worst-case error within a tile. Acceptable because (i) AGL queries
  near the runway are inside the home-tile's interior where error is < 1 m,
  and (ii) larger tiles only appear at LOD ≥ 4 (cruise altitude), where AGL
  precision matters less.
- **B — Project query through ECEF.** Convert query lat/lon/(centroid_h) to
  ECEF, find the containing facet in 2D ENU, then convert the interpolated
  facet point back to geodetic to recover the true ellipsoidal height. Adds
  one ECEF→geodetic call per query.

**Recommendation:** A. The visual mismatch reported by the user is dominated
by the world-origin curvature (Issue 7 / OQ-LS-15), not by in-tile error.
Once OQ-LS-15 is resolved, the residual in-tile error becomes a separate,
much smaller issue.

---

### OQ-LS-17 — Geoid grid file distribution

**Resolved — Option A, rely on PROJ network access.** First terrain build on
a given machine downloads the EGM2008 / EGM96 grid files via PROJ to
`$PROJ_USER_WRITABLE_DIRECTORY` and caches them; subsequent builds and
in-simulation lookups (per OQ-LS-14) hit the cache without network access.
Document the first-run network requirement in
[`docs/installation/README.md`](../installation/README.md) and in
`build_terrain.py --help`. C++ in-simulation use of the same grids
(per OQ-LS-14) reads them from the PROJ cache directory — the simulation
does not perform its own download; if the cache is empty when the
simulation starts, a clear error message tells the operator to run
`build_terrain.py` first.

**Background and rejected options retained below.**

`geoid_correct.py` uses `pyproj` with PROJ network access enabled
(`pyproj.network.set_network_enabled(True)`). On first use, PROJ downloads the
EGM2008 / EGM96 grid files to `$PROJ_USER_WRITABLE_DIRECTORY` and caches them.

Once Issue 6 fix is in place, `build_terrain.py` becomes a hard consumer of
this network call on first use per machine. Aircraft initialization (OQ-LS-12
Option B) becomes a per-startup consumer.

**Options:**

- **A — Rely on PROJ network access.** First-run requires internet; cached
  thereafter. Document in [`installation/README.md`](../installation/README.md).
- **B — Bundle the grids with the repo.** EGM2008 1° grid is ~3 MB; EGM96
  15-minute grid is ~2 MB. Add to a `python/data/proj_grids/` directory and
  set `PROJ_DATA` accordingly.
- **C — Pre-build per-region.** Compute the geoid undulation once per terrain
  build and store it in the dataset metadata; `Aircraft` reads from there.

**Recommendation:** B for offline-friendliness. Both grids are small, MIT- /
public-domain-licensed, and stable. This also removes the runtime PROJ network
dependency from `Aircraft` initialization (OQ-LS-12 Option B).

---

## Open Questions — Status Summary

| ID | Question | Status | Blocking |
| --- | --- | --- | --- |
| OQ-LS-1 | `LiveSimView` class relationship (Vispy) | Superseded — Vispy not used for live rendering; Godot 4 is the renderer (LS-DR-1) | N/A |
| OQ-LS-2 | Incremental ribbon trail strategy (Vispy) | Superseded — ribbon trail is a Godot scene concern | N/A |
| OQ-LS-3 | Shadow rendering approach (Vispy) | Superseded — shadow is a Godot scene concern | N/A |
| OQ-LS-4 | Aircraft mesh asset sourcing | Resolved — `python/assets/aircraft_lp.glb` committed (323 triangles) | No |
| OQ-LS-5 | `JoystickInput` wiring approach | Resolved → Option A (C++ `live_sim` binary owns `JoystickInput` directly; no Python binding) | No |
| OQ-LS-6 | Float precision for ring buffer position | Deferred to SB-3 (ring buffer redesign) — not a SimRunner patch | No |
| OQ-LS-7 | UDP datagram serialization format | Resolved → Option B (protobuf `SimulationFrameProto`) | No |
| OQ-LS-8 | Godot scene launch mechanism | Resolved → Option C (developer opens Godot manually; Python/C++ broadcast regardless) | No |
| OQ-LS-9 | Aircraft mesh coordinate frame correction | Resolved — layout-frame meshes + `rotation_degrees = Vector3(0, 180, 0)` correct for both static and live simulation | No |
| OQ-LS-10 | Terrain wiring in `live_sim.cpp` for landing gear contact | Resolved → Option B (`TerrainMesh` from `.las_terrain`; error if absent — no `FlatTerrain` fallback) | No |
| OQ-LS-11 | Mesh Z-axis sign mismatch: live-simulation attitude error | Resolved — eliminated by layout-frame mesh convention; see OQ-LS-9 | No |
| OQ-LS-12 | Vertical datum convention for `initial_state.altitude_m` | Resolved — Option A (always WGS84 ellipsoidal; MSL produced by EGM2008 conversion when needed) | No |
| OQ-LS-13 | Geoid model and source-specific correction | Resolved — per-source geoid for DEM correction; EGM2008 universally for in-simulation runtime conversions | No |
| OQ-LS-14 | Atmospheric model height reference | Resolved — Option B (convert h_WGS84 → h_MSL inside `Aircraft::step()` via EGM2008) | No |
| OQ-LS-15 | Where to perform ECEF→ENU for aircraft Godot position | Resolved — Option B with strict architectural separation (new `liteaero::projection` module; broadcaster composes a projector; domain layer untouched) | No |
| OQ-LS-16 | In-tile curvature correction in `TerrainMesh::elevation_m()` | Resolved — Option A, defer (no sim-vs-render mismatch after OQ-LS-15; ~12 m worst-case sim-internal error acceptable) | No |
| OQ-LS-17 | Geoid grid file distribution | Resolved — Option A (rely on PROJ network access; cached after first build) | No |
