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

## UC-2 — Software-in-the-Loop Simulation (Co-Resident SITL)

**Actor:** Engineer

**Configuration:** LiteAeroSim + FlightCode co-resident on same host (same or separate processes).

**Steps:**

1. Start LiteAeroSim simulation runner in real-time or scaled-real-time mode.
2. Start flight code components (Autopilot, Navigation) in the same process or as a local co-process.
3. Flight code receives sensor measurements from LiteAeroSim and produces `AircraftCommand`.
4. Simulation runner feeds command to `Aircraft::step()` and returns updated state to flight code.
5. Operator monitors via QGroundControl connection.

**Use:** Closed-loop autopilot development and testing; gain validation; traffic pattern development.

---

## UC-3 — Hardware-in-the-Loop Simulation (HITL)

**Actor:** Test engineer

**Configuration:** LiteAeroSim on desktop; flight code on flight hardware (companion computer or autopilot board).

**Steps:**

1. LiteAeroSim runs on desktop; flight hardware is connected via Ethernet or serial link.
2. Sensor measurements are streamed from LiteAeroSim to flight hardware at sensor update rate.
3. Flight hardware runs flight code; produces actuator commands.
4. Actuator commands are received by LiteAeroSim and fed to `Aircraft::step()`.

**Use:** Flight software integration testing on actual hardware; hardware timing and latency characterization.

---

## UC-4 — ArduPilot / PX4 Integration

**Actor:** Engineer

**Configuration:** LiteAeroSim + ArduPilot or PX4 (co-resident SITL or HITL).

**Integration options (any of the following):**

- Full replacement of ArduPilot/PX4 autopilot internals with the LiteAeroSim flight code components.
- Partial override: flight code runs as a companion-computer or offboard node, interfacing to ArduPilot/PX4 via MAVLink or custom protocol.
- Mode sequencing via Lua scripting within ArduPilot/PX4; flight code provides outer-loop set points.

**Steps:**

1. Configure the integration mode.
2. LiteAeroSim provides plant dynamics; the autopilot source (ArduPilot/PX4 internal, custom flight code, or hybrid) closes the control loop.
3. QGroundControl connects for telemetry and mission management.

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
