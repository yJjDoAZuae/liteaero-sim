# LiteAero Sim — Documentation

## Document Taxonomy

LiteAero Sim uses six document types organized in a defined folder hierarchy. The type
determines the canonical format, the maintenance skill, and the inter-referencing rules.
Using the correct type and following the referencing rules is a project requirement
(CLAUDE.md §Rules 15–21).

| Type | Folder | Role | Skill |
| --- | --- | --- | --- |
| **Architecture** | [`docs/architecture/`](architecture/overview.md) | System topology, layer model, subsystem registry, cross-subsystem coordination contracts | [`/arch`](../.claude/commands/arch.md) |
| **Design** | [`docs/design/`](design/) | Per-subsystem authority: class hierarchy, models, interface, OQs, serialization, test strategy | [`/design`](../.claude/commands/design.md) |
| **Algorithm** | [`docs/algorithms/`](algorithms/) | Mathematical derivations, discretization methods, numerical properties — no code, no OQs | [`/algo`](../.claude/commands/algo.md) |
| **Schema** | [`docs/schemas/`](schemas/) | Serialization format specs: JSON field tables, proto messages, constraints, validation examples | [`/schema`](../.claude/commands/schema.md) |
| **Roadmap** | [`docs/roadmap/`](roadmap/) | What capabilities to build and why; delivered vs. pending; item-level blocking dependencies | [`/roadmap`](../.claude/commands/roadmap.md) |
| **Implementation plan** | [`docs/implementation/`](implementation/) | How to build it: atomic code-level work items with dependency order and status tracking | [`/impl`](../.claude/commands/impl.md) |

Open questions (`/oq`) are not a document type — they live inside architecture and design
documents using the format defined in [`.claude/commands/oq.md`](../.claude/commands/oq.md).

---

## How the types inter-reference each other

```text
Architecture (docs/architecture/)
  │ subsystem registry → links to
  ▼
Design (docs/design/)                     ◄── Roadmap "Design authority:" field
  │ model sections → cites                    (docs/roadmap/)
  ▼                                               │ spawns
Algorithm (docs/algorithms/)              Implementation plan (docs/implementation/)
  (derivations; no back-reference)                │ work item "Design refs:" → cites
  │                                               │         design doc sections
Design serialization section → cites      ◄───────┘
  ▼                                       │ blocked (OQ-XX-N) → references
Schema (docs/schemas/)                    │         OQs in design/architecture docs
  (field specs; links back to design)
```

**The key rule:** information flows from design authority toward planning documents, not
the reverse. A design document does not list which implementation plans reference it.

---

## Document Index

### Architecture

| Document | Contents |
| --- | --- |
| [`architecture/overview.md`](architecture/overview.md) | Layer model, subsystem map and registry, coordinate frames, data flow, component lifecycle |
| [`architecture/system/future/`](architecture/system/future/) | Target state registry: requirements, use cases, element registry, dataflow registry, diagrams, ICDs |

### Design

Design documents live in `docs/design/` (see [`docs/design/README.md`](design/README.md)).

| Document | Subsystem |
| --- | --- |
| [`design/dynamic_element.md`](design/dynamic_element.md) | `DynamicElement` / `SisoElement` lifecycle contract; Filter and Propulsion hierarchies |
| [`design/aircraft.md`](design/aircraft.md) | `Aircraft` — integration loop, use cases, serialization |
| [`design/landing_gear.md`](design/landing_gear.md) | `LandingGear` — contact forces, wheel dynamics, serialization |
| [`design/propulsion.md`](design/propulsion.md) | `V_Propulsion`, `PropulsionJet`, `PropulsionProp`, `PropulsionEDF` |
| [`design/aero_coeff_estimator.md`](design/aero_coeff_estimator.md) | `AeroCoeffEstimator`, `AeroPerformance` |
| [`design/aero_coefficient_model.md`](design/aero_coefficient_model.md) | Aerodynamic coefficient model |
| [`design/sensor.md`](design/sensor.md) | Sensor base interface and hierarchy |
| [`design/sensor_air_data.md`](design/sensor_air_data.md) | `SensorAirData` |
| [`design/sensor_ins_sim.md`](design/sensor_ins_sim.md) | `SensorINS` (design doc; unreviewed) |
| [`design/sensor_laser_alt.md`](design/sensor_laser_alt.md) | `SensorRadAlt` (design doc; unreviewed) |
| [`design/sensor_gnss.md`](design/sensor_gnss.md) | `SensorGnss` |
| [`design/sensor_mag.md`](design/sensor_mag.md) | `SensorMag` |
| [`design/sim_runner.md`](design/sim_runner.md) | `SimRunner`, `RunnerConfig`, `ExecutionMode` |
| [`design/terrain.md`](design/terrain.md) | `V_Terrain`, `TerrainMesh`, height query interface |
| [`design/terrain_build.md`](design/terrain_build.md) | Python terrain ingestion pipeline |
| [`design/godot_plugin.md`](design/godot_plugin.md) | GDExtension C++ plugin — `SimulationReceiver`, build system |
| [`design/live_sim_view.md`](design/live_sim_view.md) | Live simulation viewer — UDP broadcast path, SimSession, Godot scene |
| [`design/python_bindings.md`](design/python_bindings.md) | pybind11 module — binding strategy, exposed classes |
| [`design/post_processing.md`](design/post_processing.md) | Python post-processing tools — `FlightLogReader`, `TimeHistoryFigure`, etc. |
| [`design/ring_buffer.md`](design/ring_buffer.md) | Ring buffer — `ChannelRegistry`, `ChannelSubscriber` |
| [`design/logger.md`](design/logger.md) | Logger subsystem |
| [`design/environment.md`](design/environment.md) | `Atmosphere`, `Wind`, `Turbulence`, `Gust` |
| [`design/manual_input.md`](design/manual_input.md) | `ManualInput`, `JoystickInput`, `KeyboardInput` |
| [`design/navigation_filter.md`](design/navigation_filter.md) | Navigation filter |
| [`design/flow_angles_estimator.md`](design/flow_angles_estimator.md) | `FlowAnglesEstimator` |
| [`design/wind_estimator.md`](design/wind_estimator.md) | `WindEstimator` |
| [`design/antiwindup.md`](design/antiwindup.md) | Anti-windup strategy |
| [`design/propulsion_coeff_estimator.md`](design/propulsion_coeff_estimator.md) | `PropulsionCoeffEstimator` |

### Algorithms

| Document | Contents |
| --- | --- |
| [`algorithms/filters.md`](algorithms/filters.md) | Tustin discretization with frequency prewarping; first-order and second-order filter coefficients |
| [`algorithms/aerodynamics.md`](algorithms/aerodynamics.md) | Aerodynamic coefficient models |
| [`algorithms/equations_of_motion.md`](algorithms/equations_of_motion.md) | RK4 equations-of-motion integration |
| [`algorithms/ground_directional_dynamics.md`](algorithms/ground_directional_dynamics.md) | On-ground lateral–directional dynamics: velocity-slaved heading, gear hold fraction, crab/sideslip, reduction to the flight model |
| [`algorithms/integration.md`](algorithms/integration.md) | Numerical integration methods |
| [`algorithms/air_data.md`](algorithms/air_data.md) | Air data computation — indicated airspeed, Mach, AGL |
| [`algorithms/screen_space_lod_selection.md`](algorithms/screen_space_lod_selection.md) | Screen-space error → required LOD at any slant range; adequacy range, hysteresis dead-band |
| [`algorithms/lod_culling_geometry.md`](algorithms/lod_culling_geometry.md) | Distance-to-centroid LOD culling: selection error, tile-size bound, transition alignment, tile-count scaling |

### Schemas

| Document | Contents |
| --- | --- |
| [`schemas/aircraft_config_v1.md`](schemas/aircraft_config_v1.md) | `aircraft_config_v1` JSON schema — all sections and field constraints; maps to `Aircraft::initialize()` |

### Roadmap

| Document | Scope |
| --- | --- |
| [`roadmap/aircraft.md`](roadmap/aircraft.md) | Aircraft simulation subsystem: landing gear, sensors, propulsion, post-processing, architecture migration |
| [`roadmap/flight_code.md`](roadmap/flight_code.md) | Flight code (liteaero-flight) roadmap |

### Implementation Plans

| Document | Scope | Status |
| --- | --- | --- |
| [`implementation/PLANS.md`](implementation/PLANS.md) | Master index of all implementation plans | Index |
| [`implementation/landing_gear_dynamics.md`](implementation/landing_gear_dynamics.md) | Wheel kappa fix, rolling-condition clamp, Tustin ODE, airborne bearing drag | Active |

### Guidelines and Standards

| Document | Contents |
| --- | --- |
| [`guidelines/general.md`](guidelines/general.md) | TDD, naming standards, SI units, serialization, architecture |
| [`guidelines/cpp.md`](guidelines/cpp.md) | C++ conventions, tooling, testing with gtest, CMake |
| [`guidelines/python.md`](guidelines/python.md) | Python conventions, type hints, testing with pytest, tooling |
| [`testing/strategy.md`](testing/strategy.md) | TDD strategy, required test categories, coverage, known failures |
| [`examples/siso_elements.md`](examples/siso_elements.md) | Usage examples for filters, integrators, PID, serialization, logging |
| [`installation/README.md`](installation/README.md) | Build from source, toolchain setup, first run |
| [`dependencies/README.md`](dependencies/README.md) | License policy, dependency registry, Conan + FetchContent patterns |
