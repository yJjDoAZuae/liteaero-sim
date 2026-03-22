# Architectural Decisions and Open Questions — Future State

---

## Decisions Made

| Decision | Choice | Rationale |
| --- | --- | --- |
| LiteAeroSim scope | Simulation plant only | Separates flight-code reuse from simulation-specific code; enables flight software deployment without simulation dependencies |
| FlightCode as separate component | Yes — not a LiteAeroSim subsystem | Flight code must meet flight-software standards; including it in a simulation library creates wrong-level dependencies |
| MAVLink as baseline external protocol | Yes, with custom extension where needed | ArduPilot and PX4 compatibility; QGroundControl integration; extensible |
| Autopilot / Navigation boundary | Navigation is outside Autopilot boundary | Separation of estimation and control; enables navigation filter replacement |
| Perception functions | Accommodated in architecture, not yet designed | Identifies kinds of perception capabilities; sets requirements for future integration without redesign |
| Docker containerization for SITL verification | LiteAeroSim and FlightCode in separate containers for verification venues; direct-call permitted for development only | Ensures the flight code under test executes in the same runtime environment as the active flight software load; the container boundary provides hard enforcement of the ICD-8 architectural boundary with no shared memory or in-process calls possible |
| FlightCode repository structure | Separate `flightcode` repository; shared interface data types (AircraftCommand, NavigationState, sensor measurement structs) live in `flightcode`; LiteAeroSim takes a versioned dependency on `flightcode` | Repo boundary structurally enforces the one-way dependency rule (simulation may depend on flight code; flight code must not depend on simulation); shared types belong in `flightcode` because flight code defines and owns them |
| C++ namespace structure | `avraero` top-level organizational namespace; second-level namespaces: `avraero::perception`, `avraero::mission_autonomy`, `avraero::nav`, `avraero::guidance`, `avraero::autopilot`, `avraero::simulation` | Org-scoped top level avoids collision in any multi-library build; subsystem namespaces follow the architectural boundary structure; `avraero::simulation` applies to LiteAeroSim code, the remaining namespaces to FlightCode |
| CMake build target structure | One CMake library target per subsystem, matching the C++ namespace: `avraero::nav`, `avraero::guidance`, `avraero::autopilot`, `avraero::perception`, `avraero::mission_autonomy`, `avraero::simulation`; shared interface types (AircraftCommand, NavigationState, sensor measurement structs) require a separate target, name TBD (see OQ-12) | Enables deployment targets to link only the subsystems they need; provides hard build-system enforcement of subsystem boundaries; consistent with the C++ namespace structure |
| SITL boundary enforcement model | Two SITL environments: (1) development SITL — single process, direct function call, no boundary enforcement, for rapid iteration; (2) integration SITL — containerized (UC-2b), strict ICD-8 boundary enforcement, required gate before HITL | Eliminates the class of errors that appear at HITL but are not caught in single-process development SITL; transport, serialization, and timing errors that would only manifest at the container or hardware boundary are exposed at integration SITL instead |
| Control element infrastructure ownership | `DynamicElement`, `SisoElement`, `Filter` hierarchy, `Integrator`, `Derivative`, `RateLimit`, `Limit`, `SISOPIDFF`, and related control elements belong in `flightcode`, not `liteaerosim` | These are general-purpose discrete-time control and DSP infrastructure with no dependency on any simulation concept; their current location in the LiteAeroSim repository is an artifact of development order; both simulation and flight code link them from `flightcode` |
| Logging interface ownership and deployment model | `ILogger` interface belongs in `flightcode`; flight code provides its own logger implementation (`FlightLogger`) for HITL and real-flight deployments; the simulation `Logger` implements `ILogger` and is injected into flight code components during SITL | Flight code components log independently of simulation in HITL and real-flight contexts; dependency inversion through `ILogger` makes flight code components agnostic to which logger is injected |
| Terrain type split | Common terrain mesh elements (`TerrainVertex`, `TerrainFacet`, `FacetColor`, `TerrainLod`, `TerrainTile`, `GeodeticPoint`, `GeodeticAABB`, `LocalAABB`) and the `V_Terrain` query interface belong in `flightcode`; simulation-specific terrain machinery (`TerrainMesh`, `LodSelector`, `FlatTerrain`, `TerrainCell`, `MeshQualityVerifier`, terrain file I/O) stays in `avraero::simulation` | Perception, relative navigation, and guidance depend on terrain data at runtime; they program against `V_Terrain` and the mesh element types without knowing the simulation terrain implementation |

---

## Open Questions Requiring Resolution

Questions are grouped by when they must be resolved. Pre-design questions block the start of
the software design phase. Design-phase questions must be resolved during that phase before
the affected component can be designed.

### Pre-Design

| ID | Question |
| --- | --- |
| OQ-12 | Which Tier 3 types does flight code actually need at runtime? Settled shared types: `AircraftCommand`, `KinematicState` (or snapshot — see OQ-15), `AirDataMeasurement`, `AtmosphericState`, `EnvironmentState`, `TurbulenceVelocity`, `WGS84_Datum`, `GeodeticPoint`, and the common terrain mesh types (`TerrainVertex`, `TerrainFacet`, `FacetColor`, `TerrainLod`, `TerrainTile`, `GeodeticAABB`, `LocalAABB`, `V_Terrain`). Tier 3 candidates still requiring a decision: `AirframePerformance` (g limits and Vne for envelope protection — likely yes), `Inertia` (mass and moments — depends on autopilot design), `LiftCurveParams`/`LiftCurveModel` (flight code unlikely to run a lift curve solver — likely no), `AeroPerformanceConfig` (drag polar — likely no), `LoadFactorInputs`/`LoadFactorOutputs` (flight code issues load factor commands without needing to invert the lift curve — likely no). |
| OQ-13 | What is the plan for adopting the `avraero::simulation` namespace in the existing LiteAeroSim codebase? When does this refactor occur relative to other milestones, and is it done in one step or subsystem by subsystem? |
| OQ-14 | What is the namespace and CMake target for cross-cutting infrastructure in `flightcode`? Four categories of infrastructure are cross-cutting across all functional subsystems and have no clean home in the six decided namespaces: (1) `ILogger` and logging sinks; (2) `DynamicElement`/`SisoElement` lifecycle; (3) `Filter`, `Integrator`, and control element hierarchy; (4) common terrain mesh types (`TerrainVertex`, `TerrainTile`, `V_Terrain`, etc.). Options: single `avraero::common` namespace and `avraero::common` CMake target; or two namespaces (`avraero::control` + `avraero::terrain`) with separate CMake targets. |
| OQ-15 | How should `KinematicState` be presented to flight code across ICD-8? Option A: share the full class — flight code receives a const reference; simpler but exposes integrator internals and serialization machinery to flight code. Option B: define a lightweight `KinematicStateSnapshot` plain struct that `KinematicState` populates each step — clean value type, no dependency on the stepper or WGS84 transformation library; the natural counterpart to the planned `NavigationState` output of `NavigationFilter`. |

### Design Phase

| ID | Question |
| --- | --- |
| OQ-4 | What is the minimum viable MAVLink message set for the ICD-10 QGroundControl interface? Where does the current MAVLink standard fall short? |
| OQ-5 | What is the preferred game engine for real-time visualization (ICD-11)? What transport protocol does it support? |
| OQ-6 | What specific kinds of perception functions should the architecture accommodate in ICD-9 and the Navigation component design (OQ-6a: image-based navigation; OQ-6b: radar-based track estimation; OQ-6c: other inference-based functions)? |
| OQ-7 | For the `LandingGear` component: should it integrate with `Aircraft::step()` directly, or should the `SimRunner` collect landing gear forces separately and pass them to `Aircraft` together with aerodynamic and propulsion forces? |
| OQ-8 | For HITL configurations (ICD-8 over network): what is the maximum acceptable latency for the command path, and what is the timestep rate of the simulation loop? |
| OQ-9 | For UC-3a/UC-3b (unmodified autopilot + companion computer): what is the complete set of MAVLink messages required for the companion computer ↔ autopilot board interface (set-point commands, state feedback, mode control)? Where does the standard MAVLink offboard mode message set fall short? |
| OQ-10 | For UC-3c/UC-3d (modified autopilot + companion computer): what interface does the modified ArduPilot or PX4 firmware expose to companion-computer components? Is this a new ICD in this document set, or is it defined in the firmware fork? |
| OQ-11 | For modified autopilot variants (UC-3c/UC-3d): which ArduPilot and PX4 extension points are used for custom control loop, navigation, and guidance backends (e.g., ArduPilot scripting, PX4 modules)? What are the build and deployment constraints for the modified firmware? |
