# Antiwindup — Architecture and Interface Design

This document is the design authority for `Antiwindup`, the saturation detector used to
freeze the `Integrator` when downstream limits are active and continued integration would
worsen the saturation.

---

## Purpose

An integrating controller accumulates error into a state variable indefinitely. When a
downstream actuator or output limit is active, the integrator continues to wind up even
though it is having no effect on the plant — and when the saturation clears, the wound-up
state causes overshoot and slow recovery. This is *integrator windup*.

`Antiwindup` is a single-signal saturation detector. Each instance monitors one scalar
signal, compares it against a configured threshold, and exposes a boolean `isActive()`
flag. The owning `Integrator` polls all its attached `Antiwindup` instances on every step
and holds its state (does not integrate) whenever any instance is active.

---

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| Not a `DynamicElement` | `Antiwindup` is a stateless comparator; its only runtime state is two booleans that reset to `false`. The `DynamicElement` lifecycle (`initialize`, `schemaVersion`, `typeName`) is disproportionate overhead for this role. The owning `Integrator` handles the lifecycle and embeds AW state in its own serialization. |
| Not a `SisoElement` | Its output is boolean, not float. `SisoElement::out()` and `operator float()` would be meaningless or misleading. |
| Config struct, not public fields | `AntiwindupConfig` is set once at initialization and does not change during stepping. Making config mutable at any time creates correctness hazards. |
| `update(float)` replaces `operator=` | `operator=` reads as assignment and obscures the step-time semantics. `update(signal)` is self-documenting. |
| `enum class Direction` | Replaces the C-style `AW_Direction` typedef enum. Scoped enum prevents accidental implicit conversion and name collisions. |
| `latch_on_direction` freeze criterion | When `true`, the detector is active only when the signal is beyond the threshold *and* moving further beyond it (rate has the saturating sign). This allows the integrator to unwind naturally when the plant begins recovering, rather than remaining frozen throughout the saturated period. |
| Embedded serialization only | `serializeJson()`/`deserializeJson()` exist so the owning `Integrator` can include each `Antiwindup` in its own snapshot. There is no standalone schema version check — `Integrator` validates its own schema, which covers the embedded AW state. |
| `name` field removed | The `name` field in the old design had no defined semantics. Logging channel naming is the owner's responsibility. |

---

## Behavior

`Antiwindup::update(signal)` evaluates two conditions:

**Saturation condition** — the signal has crossed the threshold in the configured direction:

| `Direction` | Saturated lower | Saturated upper |
|-------------|-----------------|-----------------|
| `Negative`  | `signal < limit` | — |
| `Positive`  | — | `signal > limit` |
| `Null`      | — | — |

**Windup condition** (only checked when `latch_on_direction = true`) — the signal is moving
further into saturation on this step (rate has the saturating sign):

| `Direction` | Windup condition |
|-------------|-----------------|
| `Negative`  | `signal < signal_prev` |
| `Positive`  | `signal > signal_prev` |

`isActive()` returns `true` when both conditions are met (saturation confirmed; if
`latch_on_direction` is `false`, the windup condition is skipped and saturation alone is
sufficient).

`isActiveLower()` and `isActiveUpper()` distinguish which limit is saturated, which allows
callers to apply asymmetric freeze logic in the future if needed.

---

## Interface

```cpp
// include/control/Antiwindup.hpp

namespace liteaerosim::control {

struct AntiwindupConfig {
    enum class Direction { Null = 0, Negative = -1, Positive = 1 };

    Direction direction        = Direction::Null;
    float     limit            = 0.0f;
    bool      latch_on_direction = false;
};

class Antiwindup {
public:
    explicit Antiwindup() = default;
    explicit Antiwindup(const AntiwindupConfig& config);

    // Set config after default construction.
    void configure(const AntiwindupConfig& config);

    // Update detector with the current value of the monitored signal.
    // Returns isActive().
    bool update(float signal);

    // Query saturation status (valid after the most recent update() call).
    bool isActive()      const { return isActiveLower() || isActiveUpper(); }
    bool isActiveLower() const { return is_active_lower_; }
    bool isActiveUpper() const { return is_active_upper_; }

    // Clear saturation flags. Does not change config.
    void reset();

    // Config accessors.
    AntiwindupConfig::Direction direction()      const { return config_.direction; }
    float                       limit()          const { return config_.limit; }
    bool                        latchOnDirection() const { return config_.latch_on_direction; }

    // Serialization — embedded by owner, not standalone.
    nlohmann::json serializeJson()                        const;
    void           deserializeJson(const nlohmann::json& state);

private:
    AntiwindupConfig config_;
    float  prev_signal_     = 0.0f;
    bool   is_active_lower_ = false;
    bool   is_active_upper_ = false;
};

} // namespace liteaerosim::control
```

---

## Usage Pattern

`Antiwindup` instances are embedded in `Integrator::aw` as a `std::vector<Antiwindup>`.
The typical setup for a PID with one downstream saturating limit:

```cpp
// Scenario setup (once, after initialize())
AntiwindupConfig aw_cfg;
aw_cfg.direction          = AntiwindupConfig::Direction::Positive;
aw_cfg.limit              = max_output;
aw_cfg.latch_on_direction = true;

integrator.aw.emplace_back(aw_cfg);

// Per step (called by SISOPIDFF before Integrator::step())
integrator.aw[0].update(pid.out());   // monitor the PID output signal

integrator.step(Ki * error);          // Integrator polls aw internally
```

`Integrator::onStep()` iterates `aw` and OR-reduces `isActive()`:

```cpp
bool aw_active = false;
for (auto& detector : aw) {
    aw_active |= detector.isActive();
}
if (!aw_active) {
    // integrate normally
} else {
    next = out_;  // hold current state
}
```

---

## Serialization

`Antiwindup` does not carry a `schema_version` field in its own JSON fragment — the owning
`Integrator` controls schema versioning. The embedded format is:

```json
{
    "direction":         1,
    "limit":             1.0,
    "latch_on_direction": true,
    "prev_signal":       0.0,
    "active_lower":      false,
    "active_upper":      false
}
```

`Integrator::onSerializeJson()` writes an array under the key `"antiwindup"`:

```json
{
    "schema_version": 1,
    "type": "Integrator",
    "in": 0.0,
    "out": 0.0,
    "dt_s": 0.01,
    "method": 0,
    "antiwindup": [
        { "direction": 1, "limit": 1.0, "latch_on_direction": true,
          "prev_signal": 0.0, "active_lower": false, "active_upper": false }
    ]
}
```

Round-trip for `Integrator` covers the embedded `Antiwindup` state.

---

## Not-Yet-Implemented

The following are known gaps relative to the described design:

- `AntiwindupConfig` struct does not yet exist; config is currently held as loose public
  fields on the class.
- `update(float)` does not yet exist; the current interface uses `operator=(float)`.
- `configure(const AntiwindupConfig&)` does not yet exist.
- `reset()` does not yet exist; `is_active_lower_` and `is_active_upper_` (currently
  `_activeLower`, `_activeUpper`) are uninitialized at construction.
- `serializeJson()` / `deserializeJson()` do not yet exist.
- `Integrator::onSerializeJson()` does not yet include `Antiwindup` state.
- `enum class Direction` does not yet exist; the current code uses a C-style
  `typedef enum AW_Direction`.

---

## Files

| File | Contents |
|------|----------|
| `include/control/Antiwindup.hpp` | `AntiwindupConfig` struct; `Antiwindup` class |
| `src/control/Antiwindup.cpp` | `update()`, `reset()`, `configure()`, `serializeJson()`, `deserializeJson()` |
| `test/Antiwindup_test.cpp` | Unit tests — see Test Requirements below |

---

## Test Requirements

| Test | Description |
|------|-------------|
| `NullDirection_NeverActive` | `Direction::Null`: `update()` always returns `false` regardless of signal value |
| `PositiveDirection_BelowLimit_NotActive` | `Direction::Positive`, signal < limit: not active |
| `PositiveDirection_AboveLimit_Active` | `Direction::Positive`, signal > limit, `latch_on_direction=false`: active |
| `NegativeDirection_AboveLimit_NotActive` | `Direction::Negative`, signal > limit: not active |
| `NegativeDirection_BelowLimit_Active` | `Direction::Negative`, signal < limit, `latch_on_direction=false`: active |
| `LatchOnDirection_Positive_RisingActive` | `latch_on_direction=true`, signal above limit and rising: `isActiveUpper()=true` |
| `LatchOnDirection_Positive_FallingNotActive` | `latch_on_direction=true`, signal above limit but falling: not active |
| `LatchOnDirection_Negative_FallingActive` | `latch_on_direction=true`, signal below limit and falling: `isActiveLower()=true` |
| `LatchOnDirection_Negative_RisingNotActive` | `latch_on_direction=true`, signal below limit but rising: not active |
| `Reset_ClearsFlagsNotConfig` | After `update()` activates, `reset()` clears `isActive()` but direction/limit unchanged |
| `Configure_UpdatesConfig` | `configure()` replaces all config fields; subsequent `update()` uses new config |
| `JsonRoundTrip_PreservesStateAndConfig` | Serialize after activation; deserialize; `isActive()`, `prev_signal_`, config all match |
| `Integrator_FreezesWhenAwActive` | `Integrator` with one `Antiwindup`: when `isActive()`, `step()` holds `out()` constant |
| `Integrator_JsonRoundTrip_IncludesAw` | `Integrator` serialization round-trip preserves embedded `Antiwindup` config and state |
