# System Architecture Overview

## Layer Model

The simulation is structured in four strict layers. Dependencies flow downward only — upper layers may use lower layers, never the reverse.

```mermaid
flowchart TB
    subgraph IL["Interface Layer"]
        direction LR
        CFG["Config Parser<br/>(unit conversion in)"]
        DISP["Display / Export<br/>(unit conversion out)"]
    end

    subgraph AL["Application Layer"]
        direction LR
        SCEN["Scenario"]
        SESS["Session Manager"]
        LOG["Logger"]
    end

    subgraph DL["Domain Layer"]
        direction TB
        subgraph Control
            CTRL["ControlLoop<br/>(Autopilot)"]
            SISO["SisoElement<br/>(Filters, PID, Limiters)"]
        end
        subgraph Guidance
            GUID["PathGuidance"]
            PARK["ParkTracking"]
        end
        subgraph Physics
            KIN["KinematicState"]
            ATMO["Atmosphere"]
            PROP["Propulsion"]
            AERO["Aerodynamics"]
        end
        subgraph Sensors
            INS["SensorINS"]
            AIRD["SensorAirData"]
            RALT["SensorRadAlt"]
        end
        subgraph Path
            PDEF["Path"]
            PSEG["PathSegment<br/>(Helix, Trochoid, …)"]
        end
    end

    subgraph INFRA["Infrastructure Layer"]
        direction LR
        MATH["math_util"]
        UNITS["unit_conversion"]
        SER["serialization helpers"]
        JSON["nlohmann/json"]
        EIGEN["Eigen3"]
        TROCH["trochoids"]
    end

    IL --> AL
    AL --> DL
    DL --> INFRA
```

**Rules:**

- All values crossing layer boundaries are in SI units (m, rad, s, kg, N).
- Unit conversion happens exclusively in the Interface Layer.
- The Domain Layer has no I/O, no file access, no display logic.

---

## Component Lifecycle

Every dynamic simulation element follows this lifecycle. The interface is defined by `DynamicElement` — see [DynamicElement Design](dynamic_element.md) for the full specification.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Uninitialized

    Uninitialized --> Ready : initialize(config)

    Ready --> Ready : reset()
    Ready --> Stepping : step(u)
    Stepping --> Ready : return output

    Ready --> Serialized : serialize()
    Serialized --> Ready : deserialize(state)

    Ready --> [*]
```

| Method | Responsibility |
| --- | --- |
| `initialize(config)` | Parse parameters, allocate resources, configure internal structure |
| `reset()` | Return to initial post-initialize conditions; called between simulation runs |
| `step(u)` | Advance one timestep (fixed at initialize time); return scalar output |
| `serialize()` | Return a complete SI-unit JSON snapshot of internal state |
| `deserialize(state)` | Restore internal state from a snapshot |

---

## Subsystem Map

```mermaid
flowchart LR
    subgraph Autopilot
        AP["Autopilot"]
        CR["ControlRoll"]
        CH["ControlHeading"]
        CA["ControlAltitude"]
        CV["ControlVerticalSpeed"]
        CLF["ControlLoadFactor"]
        CHR["ControlHeadingRate"]
    end

    subgraph ControlPrimitives["Control Primitives"]
        PID["SISOPIDFF"]
        FSS["FilterSS2<br/>(signal conditioning)"]
        CLIP["FilterSS2Clip<br/>(w/ limits)"]
        INT["Integrator"]
        DRV["Derivative"]
        LIM["Limit"]
        RLIM["RateLimit"]
        GAIN["Gain<br/>(scheduling planned)"]
    end

    subgraph GuidanceSubsystem["Guidance"]
        VGUID["VerticalGuidance"]
        PGUID["PathGuidance"]
        PARK2["ParkTracking"]
    end

    subgraph PathSubsystem["Path"]
        PATH["Path"]
        HELIX["PathSegmentHelix"]
        TROC["PathSegmentTrochoid<br/>(proposed)"]
    end

    subgraph Environment
        ATM["Atmosphere<br/>(ISA + lapse)"]
        WIND["Wind"]
        GUST["Gust"]
        TERR["Terrain"]
    end

    KIN2["KinematicState"] --> AP
    AP --> CR & CH & CA & CV & CLF & CHR
    CR & CH & CA & CV & CLF & CHR --> PID
    PID --> FSS & CLIP & INT & DRV & LIM & RLIM & GAIN
    PGUID & VGUID --> AP
    PATH --> PGUID
    HELIX & TROC --> PATH
    ATM & WIND & GUST --> KIN2
    TERR --> RALT2["SensorRadAlt"]
```

---

## Coordinate Frames and Sign Conventions

| Frame | Description | Usage |
| --- | --- | --- |
| NED | North–East–Down, Earth-fixed | Navigation, waypoints, wind |
| Body | $x$ forward, $y$ right, $z$ down | Aerodynamic forces, angular rates |
| Wind | $x$ along velocity vector | Aerodynamic angles ($\alpha$, $\beta$) |

Euler angles follow the **3-2-1** (yaw–pitch–roll) sequence:

$$
\mathbf{R}_{B}^{N} = R_1(\phi)\, R_2(\theta)\, R_3(\psi)
$$

Angular rates in the body frame relate to Euler rates by:

$$
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
=
\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
$$

---

## Data Flow — Closed-Loop Step

The following sequence shows one simulation step through the full stack.

```mermaid
sequenceDiagram
    participant Session
    participant KinematicState
    participant Sensors
    participant Autopilot
    participant ControlLoop
    participant Propulsion
    participant Aerodynamics

    Session->>KinematicState: step(dt_s)
    Session->>Sensors: step(kinematicState, dt_s)
    Sensors-->>Session: measurements

    Session->>Autopilot: step(cmd, measurements, dt_s)
    Autopilot->>ControlLoop: step(cmd, state, dt_s)
    ControlLoop-->>Autopilot: actuator commands

    Session->>Aerodynamics: step(state, actuators, atmosphere, dt_s)
    Aerodynamics-->>Session: forces_n, moments_nm

    Session->>Propulsion: step(throttle, atmosphere, dt_s)
    Propulsion-->>Session: thrust_n

    Session->>KinematicState: applyForces(forces_n, moments_nm, thrust_n)
```

---

## Key Architectural Decisions

| Decision | Choice | Rationale |
| --- | --- | --- |
| Lifecycle interface | Non-Virtual Interface (NVI) | Base enforces cross-cutting concerns (logging, schema validation) once |
| State snapshot format | JSON (nlohmann/json) | Human-readable, schema-versioned, language-agnostic |
| SI unit enforcement | All internal values in SI | Eliminates unit-conversion bugs in computation code |
| Filter discretization | Tustin (bilinear) with prewarping | Preserves frequency-domain behavior at design frequency |
| Path transitions | Dubins / Trochoid | Time-optimal under curvature constraint and wind |
| Linear algebra | Eigen3 | De-facto standard; zero-cost abstractions; fixed-size matrices |
| Build system | CMake FetchContent | Reproducible, no separate package manager required |
