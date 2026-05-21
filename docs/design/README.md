# Subsystem Design Documents

This folder is the canonical home for all subsystem design authority documents in
LiteAero Sim. A design authority document is the single source of truth for a
subsystem's interface, models, serialization format, test requirements, and open design
questions.

## Role

Design documents answer *how does this subsystem work?* They specify:

- What the subsystem does (use cases and actors)
- How it is structured (class hierarchy and interface)
- What algorithms and models it implements (citing `docs/algorithms/` for derivations)
- What data it serializes and in what format (citing `docs/schemas/` for complex schemas)
- What design choices remain open (open questions that block implementation)
- How it is tested (unit, integration, scenario, and serialization tests)

Design documents do not answer *what to build next* (see [`docs/roadmap/`](../roadmap/))
or *exactly which code changes are needed in what order* (see
[`docs/implementation/`](../implementation/)).

## Relation to other document types

| Other type | Relationship |
| --- | --- |
| [`docs/architecture/`](../architecture/) | Architecture documents describe the system at the layer above subsystems. The subsystem registry in `overview.md` links to design documents here. |
| [`docs/algorithms/`](../algorithms/) | Design document model sections cite algorithm documents for mathematical derivations — they reference, do not reproduce. |
| [`docs/schemas/`](../schemas/) | Design document serialization sections cite schema documents when field tables are complex enough to warrant a standalone specification. |
| [`docs/roadmap/`](../roadmap/) | Roadmap pending items list a design document as their `**Design authority:**`. The design document does not link back to the roadmap item. |
| [`docs/implementation/`](../implementation/) | Implementation plan work items cite specific sections of design documents in their `Design refs` column. Open questions in design documents (`OQ-*`) appear in `blocked (OQ-*)` status cells in implementation plans. |

## Maintenance

Design documents are created and audited with the [`/design`](../../.claude/commands/design.md) skill:

- `/design new <name>` — scaffold a new design document from the canonical template
- `/design check <file>` — audit for completeness and format consistency
- `/design update <file>` — update after implementation changes

Open questions within design documents are managed with the [`/oq`](../../.claude/commands/oq.md) skill.

## Migration note

Prior to 2026-05-21, subsystem design authority documents were stored in
`docs/architecture/`. Migration to `docs/design/` was completed on 2026-05-21.

## Current documents

| Document | Subsystem |
| --- | --- |
| [`dynamic_element.md`](dynamic_element.md) | `DynamicElement` / `SisoElement` lifecycle contract; Filter and Propulsion hierarchies |
| [`aircraft.md`](aircraft.md) | `Aircraft` — integration loop, use cases, serialization |
| [`landing_gear.md`](landing_gear.md) | `LandingGear` — contact forces, wheel dynamics, serialization |
| [`propulsion.md`](propulsion.md) | `V_Propulsion`, `PropulsionJet`, `PropulsionProp`, `PropulsionEDF` |
| [`aero_coeff_estimator.md`](aero_coeff_estimator.md) | `AeroCoeffEstimator`, `AeroPerformance` |
| [`aero_coefficient_model.md`](aero_coefficient_model.md) | Aerodynamic coefficient model |
| [`sensor.md`](sensor.md) | Sensor base interface and hierarchy |
| [`sensor_air_data.md`](sensor_air_data.md) | `SensorAirData` |
| [`sensor_ins_sim.md`](sensor_ins_sim.md) | `SensorINS` (design doc; unreviewed) |
| [`sensor_laser_alt.md`](sensor_laser_alt.md) | `SensorRadAlt` (design doc; unreviewed) |
| [`sensor_gnss.md`](sensor_gnss.md) | `SensorGnss` |
| [`sensor_mag.md`](sensor_mag.md) | `SensorMag` |
| [`sim_runner.md`](sim_runner.md) | `SimRunner`, `RunnerConfig`, `ExecutionMode` |
| [`terrain.md`](terrain.md) | `V_Terrain`, `TerrainMesh`, height query interface |
| [`terrain_build.md`](terrain_build.md) | Python terrain ingestion pipeline |
| [`godot_plugin.md`](godot_plugin.md) | GDExtension C++ plugin — `SimulationReceiver`, build system |
| [`live_sim_view.md`](live_sim_view.md) | Live simulation viewer — UDP broadcast path, SimSession, Godot scene |
| [`python_bindings.md`](python_bindings.md) | pybind11 module — binding strategy, exposed classes |
| [`post_processing.md`](post_processing.md) | Python post-processing tools — `FlightLogReader`, `TimeHistoryFigure`, etc. |
| [`ring_buffer.md`](ring_buffer.md) | Ring buffer — `ChannelRegistry`, `ChannelSubscriber` |
| [`logger.md`](logger.md) | Logger subsystem |
| [`environment.md`](environment.md) | `Atmosphere`, `Wind`, `Turbulence`, `Gust` |
| [`manual_input.md`](manual_input.md) | `ManualInput`, `JoystickInput`, `KeyboardInput` |
| [`navigation_filter.md`](navigation_filter.md) | Navigation filter |
| [`flow_angles_estimator.md`](flow_angles_estimator.md) | `FlowAnglesEstimator` |
| [`wind_estimator.md`](wind_estimator.md) | `WindEstimator` |
| [`antiwindup.md`](antiwindup.md) | Anti-windup strategy |
| [`propulsion_coeff_estimator.md`](propulsion_coeff_estimator.md) | `PropulsionCoeffEstimator` |
