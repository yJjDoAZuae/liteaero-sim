# System Diagrams — Future State

---

## System Context

```mermaid
flowchart LR
    subgraph LAS["LiteAeroSim (Simulation Plant)"]
        direction TB
        ENV3["Environment"]
        PHYS3["Aircraft Physics"]
        SENS3["Sensors"]
        TERR3["Terrain"]
        LG["LandingGear"]
        LOG3["Logger"]
        RUNNER["SimRunner"]
    end

    subgraph FC["FlightCode (separate component)"]
        direction TB
        AP2["Autopilot"]
        GUID3["Guidance"]
        PATH3["Path"]
        NAV3["Navigation"]
    end

    subgraph EXT2["External"]
        QGC["QGroundControl"]
        GE["Game Engine"]
        MI["Manual Input"]
        APX["ArduPilot / PX4"]
    end

    RUNNER -- "AircraftCommand" --> PHYS3
    PHYS3 -- "KinematicState" --> SENS3
    SENS3 -- "measurements" --> NAV3
    NAV3 -- "NavigationState" --> AP2
    AP2 -- "AircraftCommand" --> RUNNER
    GUID3 -- "SetPoint" --> AP2
    PATH3 -- "PathResponse" --> GUID3
    LG -- "GroundContactForces" --> PHYS3
    RUNNER -- "SimulationState" --> GE
    RUNNER -- "SimulationState" --> APX
    APX -- "AircraftCommand" --> RUNNER
    MI -- "AircraftCommand" --> RUNNER
    QGC <--> AP2
    RUNNER -- "all state" --> LOG3
```

---

## Closed-Loop Step Sequence (SITL)

```mermaid
sequenceDiagram
    participant Runner as SimRunner
    participant Env as Environment
    participant AC as Aircraft
    participant LG as LandingGear
    participant Sens as Sensors
    participant Nav as NavigationFilter
    participant Guid as Guidance
    participant AP as Autopilot
    participant Log as Logger

    Runner->>Env: step()
    Env-->>Runner: EnvironmentState

    Runner->>LG: step(KinematicState, TerrainQuery)
    LG-->>Runner: GroundContactForces

    Runner->>AC: step(AircraftCommand, EnvironmentState, GroundContactForces)
    AC-->>Runner: KinematicState

    Runner->>Sens: step(KinematicState, AtmosphericState)
    Sens-->>Runner: measurements[]

    Runner->>Nav: step(measurements[])
    Nav-->>Runner: NavigationState

    Runner->>Guid: step(NavigationState, Path)
    Guid-->>Runner: SetPoint

    Runner->>AP: step(NavigationState, SetPoint)
    AP-->>Runner: AircraftCommand

    Runner->>Log: write(KinematicState, measurements[], NavigationState, AircraftCommand)
```

---

## Deployment Configurations

```mermaid
flowchart LR
    subgraph A["Config A: Batch / Open-Loop"]
        LAS_A["LiteAeroSim"] --> SCRIPT["Scripted command"]
        SCRIPT --> LAS_A
    end

    subgraph B["Config B: SITL (co-resident)"]
        LAS_B["LiteAeroSim"] <--> FC_B["FlightCode"]
        FC_B <--> QGC_B["QGroundControl"]
    end

    subgraph C["Config C: HITL"]
        LAS_C["LiteAeroSim (desktop)"] <-->|"sensor stream / actuator cmd"| HW["Flight Hardware (FlightCode)"]
        HW <--> QGC_C["QGroundControl"]
    end

    subgraph D["Config D: ArduPilot/PX4 SITL"]
        LAS_D["LiteAeroSim"] <-->|"MAVLink / SITL bridge"| APX_D["ArduPilot / PX4"]
        APX_D <--> QGC_D["QGroundControl"]
        APX_D <-->|"optional"| FC_D["FlightCode (companion/offboard)"]
    end
```

---

## LiteAeroSim Internal Layer Architecture

```mermaid
flowchart TB
    subgraph IL2["Interface Layer"]
        CFG2["Config Parser (unit conversion in)"]
        DISP2["Display / Export (unit conversion out)"]
        EI2["External Interface Adapters (MAVLink, SITL bridge, visualization)"]
    end

    subgraph AL2["Application Layer"]
        RUNNER2["SimRunner (real-time / batch)"]
        LOG4["Logger"]
        MI2["ManualInput"]
    end

    subgraph DL2["Domain Layer"]
        PHYS4["Physics (Aircraft, KinematicState, Aerodynamics, Propulsion, LandingGear)"]
        ENV4["Environment (Atmosphere, Wind, Turbulence, Gust, Terrain)"]
        SENS4["Sensors"]
    end

    subgraph INFRA2["Infrastructure Layer"]
        MATH3["math utilities / Eigen3"]
        SER3["serialization / nlohmann/json / protobuf"]
    end

    IL2 --> AL2
    AL2 --> DL2
    DL2 --> INFRA2
```

**Note:** FlightCode (Autopilot, Guidance, Path, Navigation) is outside this layer diagram.
It communicates with LiteAeroSim only through the Interface Layer adapters (ICD-8, ICD-9).
