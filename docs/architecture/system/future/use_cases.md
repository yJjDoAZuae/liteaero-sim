# Use Cases — Future State

---

## UC-1 — Batch Simulation (Simulation-Only)

**Actor:** Engineer or automated test harness

**Configuration:** LiteAeroSim only; no flight code.

**Steps:**

1. Load simulation configuration.
2. Initialize all simulation components.
3. Drive `Aircraft` with a scripted `AircraftCommand` sequence (open-loop or tabulated).
4. Run N timesteps at full rate (no wall-clock pacing).
5. Log output; analyze results offline.

**Use:** Algorithm development, aero model validation, Monte Carlo, CI regression.

---

## UC-2 — Software-in-the-Loop Simulation (SITL)

**Actor:** Engineer

**Configuration:** LiteAeroSim + FlightCode on the same host. Two deployment variants apply
depending on the purpose of the session.

### UC-2a — Development SITL

**Configuration:** LiteAeroSim and FlightCode co-resident (same process or separate processes
on the same host, no container isolation).

**Steps:**

1. Start LiteAeroSim simulation runner in real-time or scaled-real-time mode.
2. Start flight code components (Autopilot, Navigation) in the same process or as a local co-process.
3. Flight code receives sensor measurements from LiteAeroSim and produces `AircraftCommand`.
4. Simulation runner feeds command to `Aircraft::step()` and returns updated state to flight code.
5. Operator monitors via QGroundControl connection.

**Use:** Day-to-day closed-loop autopilot development; gain tuning; rapid iteration.

### UC-2b — Containerized SITL (Verification Venue)

**Configuration:** LiteAeroSim and FlightCode run in separate Docker containers on the same
host. The `flightcode` container image is identical to the active flight software load image;
it contains no simulation-specific code.

**Steps:**

1. Start the `liteaerosim` container; simulation runner enters real-time mode and listens on
   the ICD-8 network port.
2. Start the `flightcode` container; flight code initializes and connects to the `liteaerosim`
   ICD-8 port.
3. Flight code receives sensor measurements over the container network and produces
   `AircraftCommand` responses.
4. Simulation runner feeds commands to `Aircraft::step()` and returns updated state.
5. Operator monitors via QGroundControl connection to the `liteaerosim` container.

**Use:** SITL verification; integration test gate; ensures flight code executes in the same
runtime environment as an active flight software load. The container boundary enforces the
ICD-8 architectural boundary — no shared memory or direct function calls are possible.

**Constraint:** This deployment variant is required for any simulation result that is used
as formal verification evidence for the flight software.

### Autopilot baseline variants (applies to UC-2a and UC-2b)

All SITL use cases support multiple autopilot baselines. The simulation plant and the
containerization policy are the same regardless of which baseline is in use.

| Variant | Autopilot source | Notes |
| --- | --- | --- |
| LiteAeroSim Autopilot | `Autopilot`, `PathGuidance`, `NavigationFilter` components of this library | Not yet implemented; the target for in-house development |
| ArduPilot SITL | ArduPilot firmware running in software simulation mode | Interfaces via ArduPilot SITL protocol; sensor and command encoding per ArduPilot convention |
| PX4 SITL | PX4 firmware running in software simulation mode | Interfaces via PX4 SITL bridge; sensor and command encoding per PX4 convention |

---

## UC-3 — Hardware-in-the-Loop Simulation (HITL)

**Actor:** Test engineer

**Hardware topology:** LiteAeroSim runs on a desktop host. Flight hardware comprises an
autopilot board and a companion computer connected by a local network or serial link.

- **Autopilot board** — runs ArduPilot or PX4 firmware; responsible for inner-loop attitude
  and rate control.
- **Companion computer** — runs navigation (EKF, sensor fusion) and guidance (path tracking,
  outer-loop set points); communicates with the autopilot board over MAVLink.
- **Desktop host** — LiteAeroSim generates simulated sensor outputs and receives actuator
  commands over a network link to the autopilot board.

Four sub-variants apply depending on the autopilot firmware and the degree of modification.

### UC-3a — HITL, Unmodified ArduPilot + Companion Computer

**Configuration:** Unmodified ArduPilot firmware on the autopilot board. LiteAeroSim's
`NavigationFilter` and guidance components run on the companion computer.

**Steps:**

1. LiteAeroSim streams simulated sensor measurements to the ArduPilot board at sensor rate.
2. ArduPilot runs its stock control loops and forwards navigation inputs to the companion computer.
3. Companion computer runs `NavigationFilter`, `PathGuidance`, and `VerticalGuidance`; sends
   set-point commands to ArduPilot via MAVLink.
4. ArduPilot produces actuator commands; LiteAeroSim feeds them to `Aircraft::step()`.

**Use:** Integration testing of navigation and guidance components on flight hardware with
unmodified ArduPilot as the inner-loop controller.

### UC-3b — HITL, Unmodified PX4 + Companion Computer

**Configuration:** Unmodified PX4 firmware on the autopilot board. Companion computer as
in UC-3a.

**Steps:** Same structure as UC-3a with PX4 SITL bridge replacing the ArduPilot interface.

**Use:** Same as UC-3a; PX4 as the inner-loop controller.

### UC-3c — HITL, Modified ArduPilot + Companion Computer

**Configuration:** ArduPilot firmware modified to incorporate custom control loop, navigation,
and guidance implementations. Companion computer provides the modified navigation and
guidance components; the modified ArduPilot firmware incorporates custom inner-loop control
backends. See UC-4 for integration patterns.

**Steps:** Same structure as UC-3a; the modified ArduPilot firmware exposes custom control
interfaces consumed by the companion computer.

**Use:** Integration testing of custom control and navigation algorithms on flight hardware
with ArduPilot as the firmware host.

### UC-3d — HITL, Modified PX4 + Companion Computer

**Configuration:** PX4 firmware modified to incorporate custom control loop, navigation, and
guidance implementations. Companion computer as in UC-3c. See UC-4 for integration patterns.

**Steps:** Same structure as UC-3b with PX4 replacing ArduPilot.

**Use:** Same as UC-3c; PX4 as the firmware host.

---

## UC-4 — ArduPilot / PX4 Autopilot Integration Patterns

**Actor:** Engineer

**Scope:** Describes the integration patterns used in UC-3c and UC-3d (modified autopilot
firmware). This is not a standalone use case; it provides the design detail behind the
"modified" HITL sub-variants.

**Pattern A — Companion-computer offboard control:**

Flight code components (`NavigationFilter`, `PathGuidance`, `VerticalGuidance`) run on
the companion computer. The autopilot firmware runs stock (or lightly modified) inner-loop
control; the companion computer sends set-point commands via MAVLink offboard mode or
custom message extensions. This is the approach used in UC-3a and UC-3b.

**Pattern B — Modified firmware with custom backends:**

The autopilot firmware (ArduPilot or PX4) is modified to incorporate custom control loops
and navigation backends, replacing or augmenting the stock algorithms at the firmware level.
The companion computer runs guidance and higher-level navigation; the autopilot firmware
exposes custom interfaces to receive set points and report state. This is the approach used
in UC-3c and UC-3d.

**Pattern B integration constraints:**

- Firmware modifications must be tracked in a version-controlled fork of the ArduPilot or
  PX4 repository with traceability to the upstream baseline.
- Custom control and navigation backends must expose reset and initialization interfaces
  consistent with the LiteAeroSim component lifecycle.
- The interface between custom firmware backends and companion-computer components is a
  separately defined ICD (not yet assigned a number in this document set).

---

## UC-5 — Traffic Pattern Operations

**Actor:** Operator (RC pilot) or automated mission

**Preconditions:** Closed-loop autopilot operational (UC-2 or UC-3); landing gear model implemented (for ground phases).

**Steps:**

1. Operator initiates autotakeoff or traffic pattern entry.
2. Flight code executes the pattern sequence: departure, crosswind, downwind, base, final.
3. Operator may intervene via RC transmitter switch inputs (abort, go-around, sequencing).
4. Flight code handles off-nominal cases per defined behavior.

---

## UC-6 — Ground Operations

**Actor:** Operator

**Preconditions:** Landing gear model implemented; ground operations design complete.

**Steps:**

1. Power-up; GPS acquisition; EKF alignment.
2. Systems test on test stand; pre-takeoff test.
3. Taxi sequence; runway survey; takeoff roll with abort capability.
4. Landing rollout; back-taxi; taxi to parking.

---

## UC-7 — Post-Flight Analysis

**Actor:** Engineer (offline)

**Steps:** As in present-state UC-6; extended to cover flight code state logs (autopilot
commands, navigation state) in addition to simulation state.
