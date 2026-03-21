# System Diagrams — Present State

---

## System Context

```mermaid
flowchart LR
    EXT["External Caller<br/>(test harness / future<br/>simulation runner)"]

    subgraph LAS["LiteAeroSim Library"]
        direction TB
        ENV["Environment<br/>(Atmosphere, Wind,<br/>Turbulence, Gust)"]
        PHYS["Aircraft Physics<br/>(6-DOF)"]
        SENS["Sensors<br/>(SensorAirData + stubs)"]
        TERR["Terrain<br/>(FlatTerrain / TerrainMesh)"]
        LOG["Logger"]
    end

    EXT -- "AircraftCommand" --> PHYS
    ENV -- "EnvironmentState" --> PHYS
    PHYS -- "KinematicState" --> SENS
    ENV -- "AtmosphericState" --> SENS
    TERR -- "height, normal" --> SENS
    PHYS -- "KinematicState" --> LOG
    SENS -- "measurements" --> LOG
```

---

## Per-Step Data Flow (Single Simulation Step)

```mermaid
sequenceDiagram
    participant Loop as Simulation Loop
    participant Atm as Atmosphere
    participant Wind as Wind/Turbulence/Gust
    participant AC as Aircraft
    participant SAD as SensorAirData
    participant Log as Logger

    Loop->>Atm: step(altitude_m)
    Atm-->>Loop: AtmosphericState

    Loop->>Wind: step(altitude_m, Va_mps)
    Wind-->>Loop: EnvironmentState (wind + turbulence + gust)

    Loop->>AC: step(AircraftCommand, EnvironmentState)
    AC-->>Loop: KinematicState

    Loop->>SAD: step(airspeed_body_mps, AtmosphericState)
    SAD-->>Loop: AirDataMeasurement

    Loop->>Log: write(KinematicState, AirDataMeasurement, ...)
```

---

## Layer Architecture

```mermaid
flowchart TB
    subgraph IL["Interface Layer (not yet implemented)"]
        CFG["Config Parser"]
        DISP["Display / Export"]
    end

    subgraph AL["Application Layer (Logger implemented; runner not yet)"]
        LOG2["Logger / LogSource / LogReader"]
        RUNNER["SimRunner (not yet implemented)"]
    end

    subgraph DL["Domain Layer"]
        direction LR
        PHYS2["Physics<br/>(Aircraft, KinematicState,<br/>Aerodynamics, Propulsion)"]
        ENV2["Environment<br/>(Atmosphere, Wind,<br/>Turbulence, Gust, Terrain)"]
        SENS2["Sensors<br/>(SensorAirData + stubs)"]
        CTRL2["Control Library<br/>(SISOPIDFF, filters,<br/>integrators, limiters)"]
        NAV2["Navigation<br/>(stubs only)"]
        FC["Flight Code Stubs<br/>(Autopilot, Guidance, Path)<br/>— to be relocated"]
    end

    subgraph INFRA["Infrastructure Layer"]
        MATH2["math utilities"]
        SER2["Serialization helpers"]
        EIGEN2["Eigen3"]
        JSON2["nlohmann/json"]
        PROTO2["protobuf"]
    end

    IL --> AL
    AL --> DL
    DL --> INFRA
```

---

## Component Lifecycle State Machine

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Uninitialized
    Uninitialized --> Ready : initialize(config)
    Ready --> Ready : reset()
    Ready --> Stepping : step(inputs)
    Stepping --> Ready : return outputs
    Ready --> Serialized : serializeJson() / serializeProto()
    Serialized --> Ready : deserializeJson() / deserializeProto()
    Ready --> [*]
```
