# Originating Requirements — Future State

These requirements are drawn from the roadmap and from the answers to architectural
uncertainty questions recorded during roadmap development.

---

## Simulation Requirements

| ID | Requirement |
| --- | --- |
| SR-1 through SR-12 | All present-state simulation requirements carry forward unchanged (see `present/requirements.md`). |
| SR-13 | LiteAeroSim is the simulation plant. It does not contain autopilot, guidance, path management, or navigation functions. These are flight code components that are architecturally separate. |
| SR-14 | The simulation must be able to operate with no flight code present (simulation-only mode for batch processing, Monte Carlo, and algorithmic development). |
| SR-15 | The simulation must be able to accept commands from and provide outputs to a co-resident flight code component without modification to the simulation internals. |
| SR-16 | The simulation must be able to interface with flight code running on a separate compute node (hardware-in-the-loop, remote SITL). |
| SR-17 | The simulation must support a landing gear dynamic model that produces ground contact forces and moments in the body frame, integrated with the terrain model. |

---

## Flight Code Requirements

| ID | Requirement |
| --- | --- |
| FC-1 | Autopilot, guidance, path management, and navigation are flight code components. They are designed to flight software standards and are not simulation-specific. |
| FC-2 | Flight code components must support simulation use cases: reset and initialization to arbitrary conditions, deterministic replay, and state serialization. |
| FC-3 | The autopilot must not include estimation functions within its architectural boundary. Estimation belongs to the navigation component. |
| FC-4 | Flight code components must support integration with ArduPilot and PX4, using MAVLink where sufficient and custom interfaces where MAVLink does not meet requirements. |
| FC-5 | Flight code components may be collocated with the simulation (same process or same host) or deployed on separate compute resources (companion computer, flight hardware). The architecture must accommodate all of these configurations. |
| FC-6 | The architecture must accommodate future perception functions (image-based navigation, inference-based state estimation) without requiring a redesign. The architecture definition identifies the kinds of perception functions anticipated and sets requirements for future expansion. |

---

## Integration and External Interface Requirements

| ID | Requirement |
| --- | --- |
| EI-1 | The system must support connection to a game engine for real-time and scaled-real-time 3D visualization. |
| EI-2 | The system must support pilot-in-the-loop control input via joystick and RC transmitter (USB). |
| EI-3 | The system must support connection to QGroundControl for mission planning and telemetry display. |
| EI-4 | The system must support ArduPilot hardware-in-the-loop (HITL) and software-in-the-loop (SITL) simulation modes. |
| EI-5 | The system must support PX4 HITL and SITL simulation modes. |
| EI-6 | The simulation loop timing must support real-time, scaled real-time, and full-rate batch execution modes. |
| EI-7 | The system must support traffic pattern operations at FAA non-towered airfields and AMA club fields, including off-nominal handling (aborted takeoff, aborted landing, go-around, expedited approach). |
| EI-8 | The system must support full ground operations: from power-up through GPS/EKF alignment, runway survey, takeoff, landing rollout, and taxi. |
