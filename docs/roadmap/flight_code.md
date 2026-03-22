# LiteAero Flight Component — Roadmap

LiteAero Flight is a separable software system. It is not a subsystem of LiteAero Sim. It will
reside in a separate `liteaero-flight` repository. Until that repository is created, stub headers
for LiteAero Flight elements live in the LiteAero Sim repository as temporary placeholders and will
be relocated at the repo split milestone. Architecture and component boundaries are defined in
`docs/architecture/system/future/`.

**Item process.** All items follow a documentation-first process:

1. Architecture and design documents are produced and reviewed.
2. Authorization to begin implementation is granted.
3. Implementation follows TDD — a failing test is written before every production code change.
4. All implementation items include simulation scenarios and automated tests sufficient for
   integration testing, demonstration, and investigation of algorithmic alternatives.

---

## Current State

Stub headers in LiteAero Sim (to be relocated to `liteaero-flight` at repo split):

| Element | Stub location | Design authority |
| --- | --- | --- |
| `Autopilot` | `include/control/Autopilot.hpp` | — |
| `PathGuidance` | `include/guidance/PathGuidance.hpp` | — |
| `VerticalGuidance` | `include/guidance/VerticalGuidance.hpp` | — |
| `ParkTracking` | `include/guidance/ParkTracking.hpp` | — |
| `V_PathSegment` | `include/path/V_PathSegment.hpp` | — |
| `PathSegmentHelix` | `include/path/PathSegmentHelix.hpp` | — |
| `Path` | `include/path/Path.hpp` | — |
| `NavigationFilter` | — (no LiteAero Sim stub; created in `liteaero-flight` at Step 9) | [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | — (no LiteAero Sim stub; created in `liteaero-flight` at Step 9) | [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | — (no LiteAero Sim stub; created in `liteaero-flight` at Step 9) | [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |

Infrastructure to be migrated from LiteAero Sim at repo split:

- `DynamicElement`, `SisoElement`, `Filter` hierarchy, `Integrator`, `Derivative`,
  `RateLimit`, `Limit`, `SISOPIDFF` → `liteaero::control`
- `ILogger` and logging sinks → `liteaero::log`
- `TerrainVertex`, `TerrainFacet`, `TerrainLod`, `TerrainTile`, `GeodeticPoint`,
  `GeodeticAABB`, `LocalAABB`, `V_Terrain` → `liteaero::terrain`
- Shared interface types (`AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`,
  sensor measurement structs) → `liteaero-flight` (target name TBD)

---

## FC-1. Repository Setup

Create the `liteaero-flight` repository. Set up CMake build system with one target per subsystem,
matching the C++ namespace structure:

| CMake target | C++ namespace | Contents |
| --- | --- | --- |
| `liteaero::log` | `liteaero::log` | `ILogger`, logging sinks |
| `liteaero::control` | `liteaero::control` | `DynamicElement`, `SisoElement`, `Filter` hierarchy, `Integrator`, `Derivative`, `RateLimit`, `Limit`, `SISOPIDFF` |
| `liteaero::terrain` | `liteaero::terrain` | Terrain mesh types, `V_Terrain` |
| Shared interface target (name TBD) | — | `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, sensor measurement structs |
| `liteaero::nav` | `liteaero::nav` | Navigation filter, wind/flow-angle estimators |
| `liteaero::guidance` | `liteaero::guidance` | Path guidance, vertical guidance, park tracking |
| `liteaero::autopilot` | `liteaero::autopilot` | Autopilot inner loop |
| `liteaero::perception` | `liteaero::perception` | Vision navigator, lidar terrain estimator |
| `liteaero::mission_autonomy` | `liteaero::mission_autonomy` | Link budget estimator |

Migrate infrastructure and shared interface types from LiteAero Sim. Update LiteAero Sim
`CMakeLists.txt` to take a versioned dependency on `liteaero-flight`. Apply `liteaero::` namespace
to all migrated code. Verify all LiteAero Sim tests still pass after migration.

**Prerequisite:** Repo split milestone (see project roadmap `README.md`).

---

## FC-2. Gain Scheduling — Design

`Gain<T, NumAxes>` currently holds template parameters for value type and scheduling axis
count, but the scheduling logic is unimplemented. This item defines the full gain scheduling
architecture. Gain scheduling is a general-purpose library capability used beyond PID loops —
it may parameterize model coefficients, limits, or other algorithm inputs. Fundamentally, a
gain is an object that produces a value parameterized by the current aircraft state. Within a
PID it is a multiplicative coefficient, but it may also enter an algorithm as an additive
term or in other ways.

### Scope to Define

- **Lookup methods** — interpolation strategies (rectilinear table bilinear,
  nearest-neighbor, polynomial fit) and how they are selected.
- **Axis dimensions** — how `NumAxes` maps to physical scheduling variables (airspeed,
  altitude, angle of attack); how axes are labeled and units enforced.
- **Lookup domain constraint functions** — how constraint functions ensure lookup occurs
  only within the valid domain of the scheduling inputs.
- **Lookup parameterization** — which flight condition and aircraft configuration values
  are available as scheduling axes.
- **Runtime update** — how a scheduled gain is evaluated at each step given the current
  scheduling variable values; whether evaluation is synchronous with `SISOPIDFF::step()`
  or driven externally.
- **Serialization** — how the gain table is stored and restored (JSON + proto).
- **Initialization** — whether the gain table is loaded from a file, embedded in config
  JSON, or populated programmatically.

### Deliverables

Design authority document at `docs/architecture/gain_scheduling.md`.
Do not implement in this item.

---

## FC-3. Gain Scheduling — Implementation

Implement the design produced in FC-2. Follow TDD: write failing tests before production code.

---

## FC-4. Autopilot Gain Design — Python Tooling

Python workflow that derives autopilot control gains from the aircraft model. Prerequisite
for FC-5 (`Autopilot`) — the C++ implementation is parameterized by gains computed here.

Scope to be defined when this item is scheduled. Expected to use Python control-system
libraries (`python-control`, `scipy.signal`) applied to linearized models extracted from
`Aircraft` trim and `AeroCoeffEstimator` outputs.

---

## FC-5. Autopilot — Inner Loop Knobs-Mode Tracking

`Autopilot` implements the inner closed-loop layer. It tracks pilot-style set point commands
corresponding to "knobs" modes — altitude hold, vertical speed hold, heading hold, and roll
attitude hold. Guidance (FC-7) is the outer loop that wraps around it and supplies set point
commands. Control gains are derived by the Python gain design workflow (FC-4). The autopilot
must support simulation use cases (reset and initialization to arbitrary conditions for batch
testing) as well as deployment as flight software.

Namespace: `liteaero::autopilot`. Derives from `DynamicElement`.

### Interface Sketch — Autopilot

*To be defined during design. Inputs will include `KinematicStateSnapshot` (or
`NavigationState`), `AtmosphericState`, and a set point struct (target altitude, target
vertical speed, target heading, target roll attitude). Output is `AircraftCommand`.*

### Tests

- Altitude hold: starting from a displaced altitude, output drives altitude error to zero
  within the expected settling time.
- Heading hold: starting from a heading offset, output drives heading error to zero without
  overshoot beyond a specified bound.
- Roll attitude hold: commanded roll angle is tracked with correct steady-state and transient.
- Vertical speed hold: commanded climb rate is tracked correctly.
- JSON and proto round-trips preserve filter states.
- Reset and re-initialization to arbitrary conditions produces correct initial output.

---

## FC-6. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

Namespace: `liteaero::guidance`. A `Path` is an ordered sequence of `V_PathSegment` objects.
Each segment exposes a cross-track error, along-track distance, and desired heading at a
query position. The initial concrete segment type is `PathSegmentHelix` (a straight line is a
degenerate helix with infinite radius).

### Interface Sketch — Path

```cpp
namespace liteaero::guidance {

struct PathQuery {
    Eigen::Vector3f position_NED_m;
    float heading_rad;
};

struct PathResponse {
    float crosstrack_error_m;     // positive = right of path
    float along_track_m;          // distance from segment start
    float desired_heading_rad;    // commanded heading at query point
    float desired_altitude_m;
    bool  segment_complete;       // true when along_track_m >= segment_length_m
};

class V_PathSegment {
public:
    virtual PathResponse query(const PathQuery& q) const = 0;
    virtual float length_m() const = 0;
    virtual ~V_PathSegment() = default;
};

} // namespace liteaero::guidance
```

### Tests — `PathSegmentHelix`

- On a straight segment (infinite radius), cross-track error equals the perpendicular
  distance from the line.
- At the midpoint of a circular arc, `desired_heading_rad` is tangent to the arc.
- `segment_complete` is false before the end and true after `along_track_m >= length_m`.

### Tests — `Path`

- A path with two segments advances to the second segment when the first is complete.
- `Path::query()` delegates to the active segment.

---

## FC-7. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

Guidance is the outer loop that wraps around `Autopilot`. It converts path and altitude
errors into set point commands (target altitude, vertical speed, heading, roll attitude) that
are fed to `Autopilot::step()`. Each guidance class derives from `DynamicElement` and
implements the full component lifecycle. Namespace: `liteaero::guidance`.

### `PathGuidance` — Lateral Path Tracking

Implements a nonlinear guidance law (L1 or similar) that commands a target heading or roll
attitude set point to null cross-track error against a path segment.

### `VerticalGuidance` — Altitude / Climb-Rate Hold

Commands target altitude or vertical speed set points to track a target altitude or climb
rate profile.

### `ParkTracking` — Loiter / Station-Keep

Commands the aircraft to orbit a fixed ground point at a specified radius and altitude by
producing heading and altitude set point commands to `Autopilot`.

### Tests — `PathGuidance`

- With zero cross-track error and correct heading, commanded heading set point matches
  current heading (no corrective input).
- With a large cross-track error, the commanded heading correction is bounded within a
  specified maximum bank angle equivalent.

### Tests — `VerticalGuidance`

- With aircraft at target altitude, commanded altitude set point matches current altitude
  (no corrective input).
- With aircraft below target altitude, commanded vertical speed set point is positive.

### Tests — Serialization

- JSON and proto round-trips preserve filter state; next `step()` output matches between
  original and restored instances.

---

## FC-8. Estimation Subsystem

Namespace: `liteaero::nav`. Each class derives from `DynamicElement` with the full component
lifecycle. Design authorities listed below.

| Class | Depends on | Design authority |
| --- | --- | --- |
| `NavigationFilter` | `SensorGnss`, `SensorAirData`, `SensorMag` | [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `NavigationFilter` or `SensorInsSimulation`, `SensorAirData` | [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `WindEstimator`, `SensorAirData` | [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |

---

## FC-9. Perception and Mission Autonomy — Proposed

The following elements are proposed and not yet designed. Architecture placeholders are
defined in `docs/architecture/system/future/element_registry.md`. Design items for each
will be scheduled when the prerequisite terrain and sensor models are available.

| Element | Namespace | Responsibility |
| --- | --- | --- |
| `VisionNavigator` | `liteaero::perception` | Position/attitude estimation from synthetic or real imagery against terrain model; fuses measurements into `NavigationFilter` |
| `LidarTerrainEstimator` | `liteaero::perception` | Terrain-relative navigation from lidar point cloud against terrain model |
| `LinkBudgetEstimator` | `liteaero::mission_autonomy` | RF link quality assessment via line-of-sight terrain occlusion; informs waypoint planning |
