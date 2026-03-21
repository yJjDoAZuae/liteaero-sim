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

---

## Open Questions Requiring Resolution

These questions must be answered before the software design phase begins.

| ID | Question |
| --- | --- |
| OQ-1 | What is the repository structure for FlightCode? Separate repository, monorepo subdirectory, or git submodule within LiteAeroSim? |
| OQ-2 | What is the C++ namespace and build target name for FlightCode components? |
| OQ-3 | For co-resident SITL (same process): is the ICD-8 boundary a direct function call, or should a thin adapter enforce the architectural boundary (preventing accidental coupling)? |
| OQ-4 | What is the minimum viable MAVLink message set for the ICD-10 QGroundControl interface? Where does the current MAVLink standard fall short? |
| OQ-5 | What is the preferred game engine for real-time visualization (ICD-11)? What transport protocol does it support? |
| OQ-6 | What specific kinds of perception functions should the architecture accommodate in ICD-9 and the Navigation component design (OQ-6a: image-based navigation; OQ-6b: radar-based track estimation; OQ-6c: other inference-based functions)? |
| OQ-7 | For the `LandingGear` component: should it integrate with `Aircraft::step()` directly, or should the `SimRunner` collect landing gear forces separately and pass them to `Aircraft` together with aerodynamic and propulsion forces? |
| OQ-8 | For HITL configurations (ICD-8 over network): what is the maximum acceptable latency for the command path, and what is the timestep rate of the simulation loop? |
