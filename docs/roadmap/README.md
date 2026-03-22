# LiteAero — Project Roadmap

Project-level view of all active and planned development work. Subsystem roadmaps provide
detailed items within each component boundary.

---

## Workstreams

| Workstream | Component | Roadmap | Status |
| --- | --- | --- | --- |
| Simulation plant — aircraft physics, environment, terrain, sensors, external interfaces, sim runner | LiteAero Sim | [aircraft.md](aircraft.md) | Active |
| Flight code — autopilot, guidance, path, navigation, estimation, gain scheduling | LiteAero Flight | [flight_code.md](flight_code.md) | Planned — `liteaero-flight` repo not yet created |

---

## Cross-Cutting Milestones

| Milestone | Dependency | Status |
| --- | --- | --- |
| **System architecture definition** | All subsequent development | ✅ Complete — see `docs/architecture/system/future/` |
| **Aerodynamic coefficient design study** — define how aero coefficients will be obtained; run design process through several aircraft configurations to determine `BodyAxisCoeffModel` format | `Aircraft6DOF` / `BodyAxisCoeffModel` design (OQ-16c) | Not yet scheduled |
| **`liteaero-flight` repository creation** — create repo; set up CMake build system with one target per subsystem (`liteaero::log`, `liteaero::control`, `liteaero::terrain`, `liteaero::nav`, `liteaero::guidance`, `liteaero::autopilot`, `liteaero::perception`, `liteaero::mission_autonomy`); migrate shared interface types | LiteAero Flight development | Planned |
| **Repo split and namespace migration** — migrate `DynamicElement`/`SisoElement`/Filter hierarchy → `liteaero::control`; `ILogger` → `liteaero::log`; terrain mesh types / `V_Terrain` → `liteaero::terrain`; shared interface types → `liteaero-flight`; apply `liteaero::` namespace to all code; update LiteAero Sim to take a versioned dependency on `liteaero-flight` | LiteAero Flight development; SITL verification pipeline | Planned — deferred to repo split milestone; all target namespaces are final |

---

## Integration and Operations

Items that span both LiteAero Sim and LiteAero Flight, or that represent end-to-end capability
milestones. Detailed design for each item will be documented before implementation begins.

| # | Item | Depends on |
| --- | --- | --- |
| I-1 | **Containerized SITL (UC-2b)** — `liteaero-sim` and `liteaero-flight` Docker containers on the same host; ICD-8 network transport; QGroundControl connection; demonstrated closed-loop flight | Repo split; SimRunner (LAS-13); LiteAero Flight autopilot + guidance + navigation; `ArduPilotInterface` or `PX4Interface` or LiteAero Sim Autopilot |
| I-2 | **ArduPilot SITL integration** — `ArduPilotInterface` adapter; demonstrated closed-loop simulation with ArduPilot firmware as autopilot baseline via the ArduPilot SITL protocol | `ArduPilotInterface` (LAS-ext-1) |
| I-3 | **PX4 SITL integration** — `PX4Interface` adapter; demonstrated closed-loop simulation with PX4 firmware as autopilot baseline via the PX4 SITL bridge | `PX4Interface` (LAS-ext-2) |
| I-4 | **Airfield traffic pattern operations (UC-5)** — autotakeoff and autolanding in a standard traffic pattern; RC transmitter operator interface; go-around handling; see `docs/architecture/system/future/use_cases.md §UC-5` | LandingGear implementation (LAS-4a); closed-loop autopilot + guidance; traffic pattern design document |
| I-5 | **Airfield ground operations (UC-6)** — full ground sequence from power-up through taxi; runway survey; systems tests; landing rollout and back-taxi; see `docs/architecture/system/future/use_cases.md §UC-6` | LandingGear implementation (LAS-4a); autopilot ground mode; GPS/EKF alignment |
| I-6 | **HITL — Unmodified ArduPilot + companion computer (UC-3a)** — LiteAero Sim on desktop; ArduPilot autopilot board; companion computer running LiteAero Flight navigation and guidance components over MAVLink | I-2; LiteAero Flight navigation + guidance |
| I-7 | **HITL — Unmodified PX4 + companion computer (UC-3b)** — same structure as I-6 with PX4 replacing ArduPilot | I-3; LiteAero Flight navigation + guidance |

---

## Document Index

| Document | Contents |
| --- | --- |
| [aircraft.md](aircraft.md) | LiteAero Sim simulation plant — Aircraft, environment, terrain, sensors, sim runner, external interfaces, visualization |
| [flight_code.md](flight_code.md) | LiteAero Flight component — autopilot, guidance, path, navigation, estimation, gain scheduling |
| [terrain-implementation-plan.md](terrain-implementation-plan.md) | Terrain subsystem implementation record (complete) |
| [liteaero-flight-migration-plan.md](liteaero-flight-migration-plan.md) | LiteAero Flight repository creation and repo split — implementation plan |
