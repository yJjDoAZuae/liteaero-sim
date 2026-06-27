# Implementation Plans — Master Index

All structured implementation plans are listed here. Every plan must be registered in this
index before any of its work items are begun. Cross-plan dependency analysis starts from
this file.

Plans are created and maintained using the [`/impl` skill](../../.claude/commands/impl.md).

---

| Plan file | Scope | Status |
| --- | --- | --- |
| [landing_gear_dynamics.md](landing_gear_dynamics.md) | Consolidated landing-gear plan: wheel dynamics (OQ-LG-5/6), gear→aircraft force-&-moment coupling (OQ-LG-9–23), and pending debt (OQ-LG-24 effectiveness weight, OQ-LG-15 diagnostic cleanup). SD-1 smoothness reconciliations excluded (undecided — OQ-LG-25) | Active |
| [body_collider_dynamics.md](body_collider_dynamics.md) | Body-collider §5 improvements: §5a/§5b/§5d done (velocity-arrest contact, restitution-consistent constraint, Coulomb + viscous friction); §5c dedicated $\Delta\theta$ rotational channel fully designed (OQ-BC-3/6/7 resolved), IP-BC-8/9 todo | Active |

---

## Notes

### Superseded legacy files

The following files predate the `/impl` skill and do not conform to the canonical plan
format. All work items they described are complete. They are retained for historical
reference and must not be consulted as descriptions of current work.

**In `docs/implementation/`:**

- [`aircraft_serialization.md`](aircraft_serialization.md) — listed gaps in
  `Aircraft::deserializeJson()`. All resolved: `AircraftState` proto exists; JSON and
  proto round-trips implemented and tested; `schema_version` and `type` checks in place.
- [`equations_of_motion.md`](equations_of_motion.md) — covered `KinematicState`,
  `LiftCurveModel`, and `LoadFactorAllocator`. All three fully implemented with tests and
  proto messages.

**In `docs/roadmap/` (implementation records, not implementation plans):**

- [`terrain-implementation-plan.md`](../roadmap/terrain-implementation-plan.md) — all 22
  implementation steps are ✅ complete. One pending performance item (WORK-TB-1:
  parallelize the terrain build triangulation loop with `ProcessPoolExecutor`) is not yet
  implemented and not yet blocking any other work. One defect (DEFECT-TB-1: tile export
  uses centroid-distance filtering instead of bbox intersection) is resolved with a
  workaround; the correct fix is not yet implemented.
- [`liteaero-flight-migration-plan.md`](../roadmap/liteaero-flight-migration-plan.md) —
  all steps complete; liteaero-flight repo created and liteaero-sim migrated.

### Subsystems with no pending implementation plans

As of 2026-05-20 the following subsystems are fully implemented, tested, and serialized,
and require no implementation plan:

- `KinematicState` / RK4 EOM integration
- `LoadFactorAllocator`, `LiftCurveModel`, `AeroPerformance`, `AeroCoeffEstimator` (stateless)
- All propulsion types (`PropulsionJet`, `PropulsionEDF`, `PropulsionProp`)
- `Aircraft` (full lifecycle, JSON + proto serialization, all subsystem integration)
- `Atmosphere`, `Wind`, `Turbulence`, `Gust`
- `TerrainMesh` / LOD pipeline; Python terrain ingestion pipeline
- `SensorAirData`
- `JoystickInput`, `KeyboardInput`, `ScriptedInput`, `ManualInput`
- `SimRunner`, `ChannelRegistry`
- `UdpSimulationBroadcaster`, `GodotEnuProjector`
- `tools/live_sim.cpp` — C++ joystick + terrain launcher (401 lines)
- GDExtension C++ plugin — `SimulationReceiver.cpp` (296 lines, compiled `.dll` in place);
  `TerrainLoader.gd` (GDScript; OQ-GP-1 resolved to stay GDScript)
- `liteaero_sim_py` pybind11 module — `Aircraft`, `KinematicState`, `SimRunner`,
  `RunnerConfig`, `ExecutionMode`, `AircraftCommand`, `ScriptedInput`,
  `JoystickInput.enumerate_devices()`, `ChannelRegistry`, `ChannelSubscriber`, `Sample`
- Python post-processing tools — `FlightLogReader`, `ModeEventSeries`,
  `TimeHistoryFigure`, `RibbonTrail`, `HudOverlay`, `TrajectoryView`, `terrain_paths`
  (224+ tests in `python/test/`)

### Pending roadmap items with no implementation plan

The following roadmap items ([`docs/roadmap/aircraft.md`](../roadmap/aircraft.md)) are
pending and have sufficient design authority but no implementation plan yet. Create a plan
with `/impl new` before beginning implementation.

- **Item 7 — LandingGear Python Bindings and Scenario Tests** — `bind_landing_gear.cpp`;
  `LandingGear`, `WheelUnit`, `StrutState`, `ContactForces` pybind11 bindings; rewrite
  `touchdown_animation.py`; four pytest scenario tests. Design authority:
  [`landing_gear.md §Steps G–H`](../design/landing_gear.md),
  [`python_bindings.md §Landing Gear`](../design/python_bindings.md). No blocking
  OQs.
- **Arch-1 — `liteaero::` Namespace Migration** — single-step migration of all LiteAero
  Sim code to `liteaero::simulation`; coordinated with liteaero-flight repo split
  milestone. Design authority:
  [`decisions.md §Namespace adoption timing`](../architecture/system/future/decisions.md).
  No blocking OQs.

### Roadmap items pending design before implementation plans can be created

The following roadmap items need design documents written (and any OQs in those documents
resolved) before an implementation plan can be created:

- **SB-3 — Ring Buffer Redesign** — type policy decision (scalar float→double vs.
  typed `Channel<T>`) must be documented in revised
  [`ring_buffer.md`](../design/ring_buffer.md) before any implementation work.
- **Item 1 — Sensor Models (implementable subset)** — `SensorMag`, `SensorGnss`,
  `SensorLaserAlt`, `SensorRadAlt` each require a design document before implementation.
- **Items 2, 3, 4, 5, 6** — design documents required; see roadmap for blocking chain.
- **Log-1 — Logging Subsystem Architecture** — design document
  (`docs/design/logging_subsystem.md`) must be produced before implementation.
- **TB-1 — Terrain Build Tool** — blocked on OQ-TB-1 and OQ-TB-2 in
  [`terrain_build.md`](../design/terrain_build.md).
- **LG-2 — Runway Geometry Extension** — blocked on OQ-LG-3 in
  [`landing_gear.md`](../design/landing_gear.md).

The following sensor headers are empty stubs (0 bytes). Design documents are needed before
any implementation plan can be created:

- `SensorINS` — design doc: `docs/design/sensor_ins_sim.md` (exists; unreviewed)
- `SensorRadAlt` — design doc: `docs/design/sensor_laser_alt.md` (exists; unreviewed)
- `SensorAA`, `SensorAAR` — no design doc
- `SensorForwardTerrainProfile`, `SensorTrackEstimator` — no design doc
- `SensorGnss`, `SensorLaserAlt`, `SensorMag` — no header, no design doc (planned in
  roadmap Item 1)
