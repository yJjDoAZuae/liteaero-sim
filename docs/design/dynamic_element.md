# DynamicElement — Unified Base Class for Dynamic Components

`DynamicElement` and `SisoElement` now reside in **`liteaero-flight`**
(`liteaero::control` namespace, `liteaero::control` CMake target).
The canonical design authority for those two classes is
[liteaero-flight/docs/architecture/dynamic_element.md](../../liteaero-flight/docs/architecture/dynamic_element.md).

This document covers:

- The `DynamicElement` / `SisoElement` interface as it applies to LiteAero Sim components
- The `Filter` abstract base and its concrete implementations (still in LiteAero Sim)
- The `Propulsion` hierarchy (still in LiteAero Sim)
- Control sub-elements (`Limit`, `RateLimit`, `Integrator`, `Derivative`, `Unwrap`, `SISOPIDFF`)
- The complete class hierarchy
- The serialization contract
- The logging interface

---

## Motivation

LiteAero Sim contains several families of components that share an identical behavioral
contract: they maintain internal state, receive time-stepped inputs, and produce
time-correlated outputs whose characteristics are determined by their dynamic properties.
A unified root base class allows cross-cutting concerns (logging, telemetry, watchdog,
replay) to be added once and generic tooling (scenario replay, Monte-Carlo harness,
snapshot comparison) to hold a heterogeneous collection of dynamic components through a
common base pointer.

---

## Naming

The root base class is named **`DynamicElement`**.

The name is drawn from the same usage as "circuit element," "finite element," and "boundary
element" — a fundamental building block in a larger interconnected system. "Dynamic"
distinguishes it from static configuration structs and data-only value objects.

Categorical identity (sensor, estimator, propulsion) is expressed through namespace
(`liteaerosim::sensor`, `liteaerosim::estimation`, `liteaerosim::propulsion`) rather than
through thin base classes.

---

## `DynamicElement` Interface

`DynamicElement` lives in `liteaero-flight`. LiteAero Sim components consume it via the
`liteaero::control` CMake target.

```cpp
// liteaero-flight/include/liteaero/control/DynamicElement.hpp
#pragma once
#include <liteaero/log/ILogger.hpp>
#include <nlohmann/json.hpp>

namespace liteaero::control {

class DynamicElement {
public:
    virtual ~DynamicElement() = default;

    void initialize(const nlohmann::json& config);
    virtual void reset();
    [[nodiscard]] nlohmann::json serializeJson() const;
    void deserializeJson(const nlohmann::json& state);
    void attachLogger(liteaero::log::ILogger* logger) noexcept;

protected:
    virtual void           onInitialize(const nlohmann::json& config) = 0;
    virtual void           onReset()                                   = 0;
    virtual nlohmann::json onSerializeJson()                   const   = 0;
    virtual void           onDeserializeJson(const nlohmann::json&)    = 0;
    virtual void           onLog(liteaero::log::ILogger& /*logger*/) const {}
    virtual int            schemaVersion()                     const   = 0;
    virtual const char*    typeName()                          const   = 0;

    liteaero::log::ILogger* logger_ = nullptr;

private:
    void validateSchema(const nlohmann::json& state) const;
};

} // namespace liteaero::control
```

`initialize()` and `deserializeJson()` validate `"schema_version"` before forwarding to
the hook; they throw `std::runtime_error` on mismatch. `schemaVersion()` and `typeName()`
are pure-virtual so every concrete class is required to declare them explicitly. The base
injects both into every serialized snapshot so that diagnostic tools can identify and
version-check any snapshot without knowing the concrete type.

Proto serialization is not declared on this base because proto message types are
component-specific. Each concrete class declares `serializeProto()` /
`deserializeProto()` with the appropriate message type.

---

## `SisoElement`

`SisoElement` is the abstract base for all single-input, single-output dynamic elements.
It derives from `DynamicElement` and adds the SISO step interface. It lives in
`liteaero-flight` alongside `DynamicElement`.

```cpp
// liteaero-flight/include/liteaero/control/SisoElement.hpp
namespace liteaero::control {

class SisoElement : public DynamicElement {
public:
    [[nodiscard]] float in()  const { return in_; }
    [[nodiscard]] float out() const { return out_; }
    operator float()          const { return out_; }

    float step(float u);   // NVI entry point — calls onStep(), then onLog()
    void  reset() override; // zeros in_ and out_, then calls onReset()

protected:
    float in_  = 0.0f;
    float out_ = 0.0f;

    virtual float onStep(float u) = 0;
    void onReset() override {}
};

} // namespace liteaero::control
```

The public `step(float u)` is the NVI entry point: it records `in_` and `out_`, calls
`onStep()` for the subclass-defined update, then calls `onLog()` if a logger is attached.
`reset()` zeros `in_` and `out_` before calling `onReset()`.

Every SISO element in the codebase — filters, limiters, integrators, derivatives, unwrap
— derives from `SisoElement`. This gives every element a uniform lifecycle and makes it
holdable through a `DynamicElement` base pointer.

---

## `Filter` (under `SisoElement`)

`Filter` extends `SisoElement` with filter-specific API: `order()`, `dcGain()`,
`resetToInput()`, `resetToOutput()`, `errorCode()`. `Filter` is a pure abstract base —
it declares no lifecycle hook defaults. Every concrete filter must implement the full
`DynamicElement` lifecycle: `onInitialize`, `onSerializeJson`, `onDeserializeJson`,
`schemaVersion`, `typeName`, and `onStep`.

`Filter` currently lives in LiteAero Sim (`liteaerosim::control` namespace,
`include/control/Filter.hpp`) and will be migrated to `liteaero::control` in Step 4.

### Filter implementations

| Class | Description |
| --- | --- |
| `FilterSS2` | Second-order state-space filter — full lifecycle |
| `FilterSS2Clip` | `FilterSS2` with value and rate limits — full lifecycle; NVI via `onStep()` |
| `FilterTF` | Transfer-function form (variable order) — full lifecycle; NVI via `onStep()` |
| `FilterTF2` | Second-order transfer-function form — full lifecycle; NVI via `onStep()` |
| `FilterFIR` | Finite impulse response filter — full lifecycle; NVI via `onStep()` |
| `FilterSS` | General state-space filter (arbitrary order) — full lifecycle; NVI via `onStep()` |

---

## `Propulsion` (under `DynamicElement`)

`Propulsion` derives from `liteaero::control::DynamicElement` directly and adds the
propulsion-specific `step(throttle, tas, rho)` signature, `thrust_n()` accessor, and
proto serialization methods. All concrete propulsion models (`PropulsionJet`, `PropulsionEDF`,
`PropulsionProp`, `MotorElectric`, `MotorPiston`) derive from `Propulsion` and implement
the full lifecycle.

`Motor` is a stateless abstract interface — it does not derive from `DynamicElement`.

---

## Control Sub-elements

`Limit`, `RateLimit`, `Integrator`, `Derivative`, and `Unwrap` all derive from
`liteaero::control::SisoElement`. Each carries the full `DynamicElement` lifecycle:
`initialize()`, `reset()`, `serializeJson()`, `deserializeJson()`. These elements are used
both standalone and embedded within larger elements such as `SISOPIDFF`; both use cases
benefit from a uniform lifecycle.

`SISOPIDFF` derives from `liteaero::control::DynamicElement` and implements the full
lifecycle. Its `onSerializeJson()` assembles a composite snapshot by delegating to each
embedded member (`FilterSS2Clip` × 6, `Integrator`, `Derivative`, `Unwrap` × 2). This
makes the full mid-flight PID state saveable and restorable as a unit.

`Gain` is a template and does not derive from `SisoElement`. Its scalar value is
serialized directly (as a JSON number) within the `SISOPIDFF` snapshot.

These sub-elements currently live in LiteAero Sim (`liteaerosim::control` namespace) and
will be migrated to `liteaero::control` in Step 5.

---

## Complete Class Hierarchy

### DynamicElement tree

```text
liteaero::control::DynamicElement                     [liteaero-flight]
│
├── liteaero::control::SisoElement                    [liteaero-flight]
│   ├── liteaerosim::control::Filter  (pure abstract) [liteaero-sim — migrates Step 4]
│   │   ├── FilterSS2
│   │   ├── FilterSS2Clip
│   │   ├── FilterTF
│   │   ├── FilterTF2
│   │   ├── FilterFIR
│   │   └── FilterSS
│   ├── liteaerosim::control::Limit                   [liteaero-sim — migrates Step 5]
│   ├── liteaerosim::control::RateLimit               [liteaero-sim — migrates Step 5]
│   ├── liteaerosim::control::Integrator              [liteaero-sim — migrates Step 5]
│   ├── liteaerosim::control::Derivative              [liteaero-sim — migrates Step 5]
│   └── liteaerosim::control::Unwrap                  [liteaero-sim — migrates Step 5]
│
├── liteaerosim::control::SISOPIDFF                   [liteaero-sim — migrates Step 5]
│
├── liteaerosim::propulsion::Propulsion               [liteaero-sim]
│   ├── PropulsionJet
│   ├── PropulsionEDF
│   ├── PropulsionProp
│   ├── MotorElectric
│   └── MotorPiston
│
├── liteaerosim::sensor::SensorAirData    (stub)
├── liteaerosim::sensor::SensorGnss       (stub)
├── liteaerosim::sensor::SensorLaserAlt   (stub)
├── liteaerosim::sensor::SensorMag        (stub)
├── liteaerosim::sensor::SensorRadAlt     (stub)
├── liteaerosim::sensor::SensorAA         (stub)
├── liteaerosim::sensor::SensorAAR        (stub)
├── liteaerosim::sensor::SensorTrackEstimator          (stub)
├── liteaerosim::sensor::SensorForwardTerrainProfile   (stub)
├── liteaerosim::sensor::SensorInsSimulation           (stub)
│
├── liteaerosim::estimation::NavigationFilter    (stub)
├── liteaerosim::estimation::WindEstimator       (stub)
└── liteaerosim::estimation::FlowAnglesEstimator (stub)
```

---

## Serialization Contract

Every `DynamicElement` subclass serializes a complete, self-describing JSON snapshot.

### Rules

| Rule | Detail |
| --- | --- |
| All values in SI units | `"wn_rad_s"` not `"wn_hz"`; `"dt_s"` not `"dt_ms"` |
| Field names encode units | `"altitude_m"`, `"roll_rate_rad_s"`, `"thrust_n"` |
| `schema_version` always present | Integer; base class injects it; `onSerializeJson()` must not duplicate it |
| `type` always present | String from `typeName()`; base class injects it; `onSerializeJson()` must not duplicate it |
| Round-trip lossless | `deserializeJson(serializeJson())` must yield identical state |
| Schema version checked on load | Base class validates; throws `std::runtime_error` on mismatch |

### Example snapshot

```json
{
    "schema_version": 1,
    "type": "FilterSS2",
    "in": 0.0,
    "out": 0.0,
    "state": { "x0": 0.0, "x1": 0.0 },
    "params": {
        "design":    "low_pass_second",
        "dt_s":      0.01,
        "wn_rad_s":  6.2832,
        "zeta":      0.7071,
        "tau_zero_s": 0.0
    }
}
```

---

## Logging Interface

Logging is injected via a pointer to `liteaero::log::ILogger`. The base class calls
`onLog()` at the end of every `step()` when a logger is attached. Elements are not
required to implement `onLog()` — the default is a no-op.

```cpp
// liteaero-flight/include/liteaero/log/ILogger.hpp
namespace liteaero::log {

class ILogger {
public:
    virtual ~ILogger() = default;
    virtual void log(std::string_view channel, float value_si) = 0;
    virtual void log(std::string_view channel, const nlohmann::json& snapshot) = 0;
};

} // namespace liteaero::log
```

Loggers are attached at scenario setup time, not in constructors.

---

## Schema Version Convention

Each concrete class supplies its own `kSchemaVersion_` constant and implements
`schemaVersion()` to return it. Schema version is always `1` during initial development;
it will be incremented when fields change after the project transitions to a maintenance
phase.

---

## Files

| File | Contents | Location |
| --- | --- | --- |
| `include/liteaero/control/DynamicElement.hpp` | Root abstract base | liteaero-flight |
| `src/control/DynamicElement.cpp` | `initialize`, `reset`, `serializeJson`, `deserializeJson`, `attachLogger` | liteaero-flight |
| `include/liteaero/control/SisoElement.hpp` | SISO NVI wrapper over `DynamicElement` | liteaero-flight |
| `src/control/SisoElement.cpp` | `step`, `reset` | liteaero-flight |
| `include/control/Filter.hpp` | Abstract filter base over `SisoElement` | liteaero-sim |
