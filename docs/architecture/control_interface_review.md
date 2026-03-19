# Control Subsystem — Interface Design Review

This document identifies interface design pattern errors in the control subsystem
(`include/control/`, `src/control/`) and proposes concrete refactoring for each.
Issues are ordered by severity.

---

## Summary Table

| # | Issue | Severity | Files |
|---|-------|----------|-------|
| 1 | Tier-1 filters bypass NVI and shadow base members | Critical | `FilterSS2Clip`, `FilterTF`, `FilterTF2`, `FilterFIR`, `FilterSS` |
| 2 | `Filter` provides no-op lifecycle defaults | Critical (enables #1) | `Filter.hpp` |
| 3 | `SISOPIDFF` has no `DynamicElement` lifecycle | Significant | `SISOPIDFF.hpp/cpp` |
| 4 | `Gain` template is prematurely generic and inconsistent | Moderate | `Gain.hpp` |
| 5 | `RateLimit` derives from `LimitBase` despite semantic mismatch | Moderate | `RateLimit.hpp`, `LimitBase.hpp` |
| 6 | `Unwrap::step(float, float)` bypasses NVI | Moderate | `Unwrap.hpp/cpp` |
| 7 | Public embedded `Limit` members on `FilterSS2Clip`, `Integrator`, `Derivative` | Minor | Multiple |
| 8 | C-style enums in the control namespace | Minor | `control.hpp` |
| 9 | Naming convention inconsistency in legacy filters | Minor | `FilterSS2Clip`, `FilterTF`, etc. |

---

## Issue 1 — Tier-1 Filters Bypass NVI and Shadow Base Members

**Severity: Critical**

### Problem

`FilterSS2Clip`, `FilterTF`, `FilterTF2`, `FilterFIR`, and `FilterSS` all derive from
`Filter : SisoElement`, but they break the NVI contract in three compounding ways:

**a) Override `step()` directly instead of implementing `onStep()`.**

`SisoElement::step(float u)` is the NVI entry point. It records `in_`/`out_`, calls
`onStep()`, then calls `onLog()`. When a subclass overrides `step()` directly, it bypasses
`onLog()` (logging is silently skipped), and `in_`/`out_` in `SisoElement` are never
updated.

**b) Declare their own `_in`/`_out` members that shadow `SisoElement::in_`/`out_`.**

These filters carry a duplicate pair of state variables. Every instance wastes memory
on `in_` and `out_` from `SisoElement` that are always zero and never used. The
shadowing also means that reading `in()` or `out()` through a base pointer returns a
different value than reading it through the concrete type.

**c) Shadow `in()`, `out()`, and `operator float()` with non-virtual methods.**

The base-class versions return `in_`/`out_` (always zero for these filters). The
concrete-class versions return `_in`/`_out` (correct). The shadowing is invisible at
the concrete type, but calling through a `SisoElement*` or `Filter*` silently returns
the wrong value:

```cpp
FilterSS2Clip f;
f.setLowPassFirstIIR(0.01f, 0.1f);
f.step(1.0f);

float v1 = f.out();              // 0.something  — correct (shadow method)
float v2 = static_cast<SisoElement&>(f).out();   // 0.0f — WRONG (base method)
float v3 = static_cast<float>(f);                // 0.something — correct (shadow)
SisoElement* p = &f;
float v4 = p->out();             // 0.0f — WRONG (base method, no override)
```

### Consequence

These filters cannot be used correctly through any base pointer. The `DynamicElement`
lifecycle contract — `step()`, `serializeJson()`, `deserializeJson()` usable through a
base pointer — is broken for the entire `Filter` subtree except `FilterSS2`.

### Recommended Fix

Migrate each filter to the NVI pattern:

1. Remove the filter's own `_in`/`_out` members; use `in_`/`out_` from `SisoElement`.
2. Remove the filter's own `in()`, `out()`, `operator float()` shadow methods.
3. Rename `step(float in)` → `onStep(float u)` so the NVI chain is used.
4. Implement `onInitialize`, `onSerializeJson`, `onDeserializeJson`, `schemaVersion`,
   `typeName` (see Issue 2 for why these need to be real).

`FilterSS2` already demonstrates the correct pattern and serves as the template for
migration. `FilterSS2Clip` is the highest priority because it is used extensively in
`SISOPIDFF`; see Issue 3.

---

## Issue 2 — `Filter` Provides No-Op Lifecycle Defaults

**Severity: Critical (enables Issue 1)**

### Problem

`Filter` provides default implementations for every `DynamicElement` lifecycle hook:

```cpp
void           onInitialize(const nlohmann::json&)      override {}
nlohmann::json onSerializeJson()               const    override { return {}; }
void           onDeserializeJson(const nlohmann::json&) override {}
int            schemaVersion()                 const    override { return 0; }
const char*    typeName()                      const    override { return "Filter"; }
float          onStep(float u)                          override { return u; }
```

These defaults exist solely to allow the Tier-1 filters (Issue 1) to compile without
implementing the lifecycle. The consequences:

- `serializeJson()` on a `FilterSS2Clip*` (through the base) returns `{}` — an empty
  JSON object with no filter state.
- `deserializeJson({})` silently succeeds and restores nothing.
- `schemaVersion() == 0` passes schema checks for any snapshot (0 never matches a
  real schema version).
- `typeName() == "Filter"` is wrong for every concrete type.
- `onStep(u) { return u; }` is a pass-through — any filter that forgets to override
  both `step()` and `onStep()` silently becomes a wire.

The no-op defaults normalize and hide a broken design. They are not a temporary
transition aid — they are the permanent mechanism that makes Issue 1 compile.

### Recommended Fix

Once the Tier-1 filters are migrated (Issue 1), remove the no-op defaults from
`Filter`. The correct posture is:

```cpp
class Filter : public liteaerosim::SisoElement {
public:
    virtual uint8_t  order()               const = 0;
    virtual float    dcGain()              const = 0;
    virtual void     resetToInput(float)         = 0;
    virtual void     resetToOutput(float)        = 0;
    virtual uint16_t errorCode()           const = 0;
    // No default lifecycle hooks — concrete subclasses must implement them.
};
```

During migration, filters can keep temporary stubs, but those stubs live in the
concrete class, not in the abstract base. The base should never provide a pass-through
`onStep()`.

---

## Issue 3 — `SISOPIDFF` Has No `DynamicElement` Lifecycle

**Severity: Significant**

### Problem

`SISOPIDFF` is the primary control law. It is a plain aggregate struct: no base class,
no `initialize()`, no serialization. Its six `FilterSS2Clip` members, `Integrator`,
`Derivative`, and two `Unwrap` members each have a `DynamicElement` lifecycle, but
`SISOPIDFF` neither coordinates their lifecycle nor exposes one of its own.

Consequences:

- Mid-flight PID state (filter states, integrator state, error signal state) cannot be
  saved or restored as a unit.
- A collection of control loops cannot be driven through a `DynamicElement*` interface
  (e.g., a watchdog or replay harness).
- `SISOPIDFF` has a `reset(...)` method, but its signature is hand-rolled and does not
  conform to `DynamicElement::reset()`.

### Recommended Fix

Make `SISOPIDFF` derive from `DynamicElement` and implement the lifecycle:

```cpp
class SISOPIDFF : public liteaerosim::DynamicElement {
public:
    float step(float cmd, float meas);
    float step(float cmd, float meas, float meas_dot);
    void  reset(float cmd, float meas, float out);
    void  reset(float cmd, float meas, float meas_dot, float out);
    // ...
protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                  override;
    nlohmann::json onSerializeJson()                  const   override;
    void           onDeserializeJson(const nlohmann::json&)   override;
    int            schemaVersion()                    const   override { return 1; }
    const char*    typeName()                         const   override { return "SISOPIDFF"; }
};
```

`onSerializeJson()` delegates to each member's `serializeJson()` and assembles a
composite snapshot. `onDeserializeJson()` restores each member. `onReset()` calls the
existing `reset()` overload with zero values.

`SISOPIDFF::step()` is not the NVI `step(float u)` from `SisoElement` — it takes two
or three arguments. `SISOPIDFF` should therefore derive from `DynamicElement` directly,
not from `SisoElement`. The SISOPIDFF output is exposed via `out()` as a non-virtual
accessor, not through the SISO interface.

Note: this fix is blocked by Issue 1. `SISOPIDFF::onSerializeJson()` must delegate to
its `FilterSS2Clip` members' `serializeJson()`. Until those members have real
serialization, the composite snapshot is meaningless.

---

## Issue 4 — `Gain` Template Is Prematurely Generic and Inconsistent

**Severity: Moderate**

### Problem

`Gain<T, NumAxes>` is parameterized on value type `T` and scheduling axis count
`NumAxes`. In all current uses it is instantiated as `Gain<float, 3>`.

**a) Scheduling stubs mislead.** `schedule()`, `readJSON()`, and `readFile()` are
stubs. `readJSON()` parses JSON and discards the result. `readFile()` opens a file and
calls `readJSON()` on it. Neither updates any state. The `RectilinearTable<T, T,
NumAxes>` member is never populated or used. The code implies scheduling exists when
it does not, which is more confusing than having no scheduling code at all.

**b) API is inconsistent with the rest of the codebase.** `Gain` uses:
- `operator=(T K)` to set the gain value — reads as assignment, not configuration
- `operator double()` to read the gain — inconsistent with `operator float()` everywhere else
- `K()` accessor — ambiguous name; not self-documenting

**c) `_K` is `protected`.** There is no subclass mechanism, so this provides no benefit
and widens the encapsulation boundary unnecessarily.

### Status

Gain scheduling is a planned feature. The template structure (`T`, `NumAxes`) and the
`RectilinearTable` placeholder anticipate it. The design — scheduling axes, lookup
methods, serialization format — is not yet specified.

**This issue is deferred pending the gain scheduling design** (see roadmap). The correct
action is not to flatten `Gain` to a non-template class, but to:

1. Remove the misleading stub methods (`schedule()`, `readJSON()`, `readFile()`) and
   the unpopulated `RectilinearTable` member until the design is ready.
2. Fix the API inconsistencies in isolation: `operator=(T)` → `set(T)`,
   `operator double()` → `operator float()`, `K()` → `value()`, `_K` → `value_`.

These two cleanup steps are low-risk and do not constrain the future scheduling design.
The template parameters `T` and `NumAxes` are retained as-is.

---

## Issue 5 — `RateLimit` Derives from `LimitBase` Despite Semantic Mismatch

**Severity: Moderate**

### Problem

`LimitBase` defines an interface for **value saturation**: `setLower(float lim)` sets a
value bound, and `isLimitedLower()` reports whether the signal is currently clamped to
that bound.

`RateLimit` uses the same interface, but its bounds are **rate bounds** (units/second),
not value bounds. Its semantics are: "limit how fast the output can change, not how far
it can go." The `dt` timestep parameter — which is essential to rate limiting — has no
place on `LimitBase`, so callers holding a `LimitBase&` to a `RateLimit` cannot
configure `dt` through the base interface.

The shared base API has the right shape (lower/upper/enable/disable) but the wrong
units. A caller reading `lowerLimit()` on a `LimitBase&` cannot tell whether it is
reading a value bound or a rate bound.

### Recommended Fix

Remove `RateLimit` from the `LimitBase` hierarchy. With `RateLimit` gone, `LimitBase`
has exactly one concrete subclass (`Limit`) and no caller holds a `LimitBase*`. Delete
`LimitBase` entirely; derive both `Limit` and `RateLimit` from `SisoElement` directly:

```
SisoElement
├── Limit       (value saturation)
├── RateLimit   (rate saturation)
├── Integrator
├── Derivative
└── Unwrap
```

`Limit`'s `setLower`/`setUpper`/`enable`/`disable` interface is unchanged — those
methods stay on `Limit` where they belong. `RateLimit`'s public interface is also
unchanged. `LimitBase.hpp` is deleted.

---

## Issue 6 — `Unwrap::step(float, float)` Bypasses NVI

**Severity: Moderate**

### Problem

`Unwrap` exposes two public `step` overloads:

```cpp
using SisoElement::step;            // NVI: step(float u) — wraps onStep()
float step(float u, float ref);     // direct: bypasses NVI entirely
```

The two-arg overload directly writes `in_` and `out_` without going through
`SisoElement::step()`. The consequences:

- `onLog()` is never called for two-arg steps. If a logger is attached to `Unwrap`,
  the log is incomplete — only single-arg steps are recorded.
- `in_` is written by the two-arg overload but the base NVI's "prev = in_" convention
  is bypassed, meaning that after a two-arg step, a subsequent single-arg step computes
  delta from the last two-arg output rather than from the last NVI input.

The two-arg overload exists because `Unwrap` needs an external reference signal that
cannot be passed through the single-arg NVI interface. This is a legitimate need, but
the current mechanism leaks implementation.

### Recommended Fix

The cleanest fix is to keep the NVI chain but pass the reference differently. Options:

**Option A — Reference as a stored field:**

```cpp
class Unwrap : public SisoElement {
public:
    void setReference(float ref) { ref_ = ref; }
    // single-arg step uses stored ref_
protected:
    float onStep(float u) override;  // uses ref_ internally
private:
    float ref_ = 0.0f;
};
```

Caller sets `ref_` before calling `step(u)`. Logging works. The stored reference is
included in serialization.

**Option B — Keep two-arg overload but route through NVI:**

Make `step(float u, float ref)` call `SisoElement::step(u)` with `ref_` pre-set.
This preserves the two-arg API while going through the NVI chain.

Option A is preferred because it makes the reference a named, serializable field rather
than an invisible side-channel argument.

---

## Issue 7 — Public Embedded `Limit` Members

**Severity: Minor**

### Problem

`FilterSS2Clip::valLimit`, `FilterSS2Clip::rateLimit`, `Integrator::limit`, and
`Derivative::limit` are all `public`. This allows callers to mutate limit state between
steps — changing bounds, enabling/disabling — without the owner being aware of the
change. No invariants can be maintained by the owning class.

Additionally, public member subobjects are serialized by the owner but configured by
the caller, creating a split-ownership situation.

### Recommended Fix

The appropriate visibility depends on the intended usage:

- If callers must configure limits before each step (e.g., `FilterSS2Clip` in
  SISOPIDFF), expose typed configuration methods on the owner class, not the raw member.
- If limits are configured once at initialization, include them in `initialize()` config
  JSON and remove the public member.

The exact API is a design decision for each class, but the public raw member is the
wrong default. A minimal change is to make the members `private` and provide forwarding
methods for the operations actually used by callers.

---

## Issue 8 — C-Style Enums in the Control Namespace

**Severity: Minor**

### Problem

`control.hpp` declares:

```cpp
enum FilterError : uint16_t { NONE = 0, INVALID_DIMENSION = 1, ... };
enum DiscretizationMethod    { FwdEuler = 0, BackEuler = 1, ... };
```

Both are C-style unscoped enums. Their enumerators are injected into
`liteaerosim::control`, so `liteaerosim::control::NONE` conflicts with any other
`NONE` in the namespace. `FwdEuler`, `BackEuler`, etc. are similarly unscoped.

### Recommended Fix

```cpp
enum class FilterError : uint16_t {
    None = 0, InvalidDimension = 1, InvalidTimestep = 2,
    Unstable = 4, InfiniteDcGain = 8, ZeroDcGain = 16, InvalidPolynomial = 32
};

enum class DiscretizationMethod {
    ForwardEuler = 0, BackwardEuler = 1, Bilinear = 2, Prewarp = 3, PoleZeroMatch = 4
};
```

Enumerator names are also renamed to avoid `SCREAMING_CASE` (which the CLAUDE.md
convention reserves for compile-time constants, not enum values) and to be
self-documenting (`ForwardEuler` not `FwdEuler`).

---

## Issue 9 — Naming Convention Violations (Comprehensive Inventory)

**Severity: Minor**

The CLAUDE.md convention is: `snake_case_` members (trailing underscore), `camelCase`
methods, `PascalCase` classes, unit suffixes in names when not obvious (`altitude_m`,
`dt_s`), no abbreviations, boolean members named as predicates (`is_active_`, not
`active_`). The following violations are spread across the control subsystem.

---

### 9a — `_camelCase` Members (Leading Underscore, Wrong)

CLAUDE.md mandates `snake_case_` (trailing underscore). These classes use the opposite
convention (`_camelCase` with a leading underscore):

| Class | Wrong name | Correct name |
|-------|-----------|--------------|
| `FilterSS2Clip` | `_in` | `in_` |
| `FilterSS2Clip` | `_out` | `out_` |
| `FilterSS2Clip` | `_Phi` | `phi_` |
| `FilterSS2Clip` | `_Gamma` | `gamma_` |
| `FilterSS2Clip` | `_H` | `h_` |
| `FilterSS2Clip` | `_J` | `j_` |
| `FilterSS2Clip` | `_x` | `x_` |
| `FilterSS2Clip` | `_order` | `order_` |
| `FilterSS2Clip` | `_dt` | `dt_s_` |
| `FilterSS2Clip` | `_errorCode` | `error_code_` |
| `Limit` / `LimitBase` | `_lowerLimit` | `lower_limit_` |
| `Limit` / `LimitBase` | `_upperLimit` | `upper_limit_` |
| `Limit` / `LimitBase` | `_limitedLower` | `limited_lower_` |
| `Limit` / `LimitBase` | `_limitedUpper` | `limited_upper_` |
| `Limit` / `LimitBase` | `_enableLowerLimit` | `lower_enabled_` |
| `Limit` / `LimitBase` | `_enableUpperLimit` | `upper_enabled_` |
| `RateLimit` | `_lowerLimit` | `lower_limit_` |
| `RateLimit` | `_upperLimit` | `upper_limit_` |
| `RateLimit` | `_limitedLower` | `limited_lower_` |
| `RateLimit` | `_limitedUpper` | `limited_upper_` |
| `RateLimit` | `_enableLowerLimit` | `lower_enabled_` |
| `RateLimit` | `_enableUpperLimit` | `upper_enabled_` |
| `RateLimit` | `_dt` | `dt_s_` |
| `Integrator` | `_dt` | `dt_s_` |
| `Integrator` | `_method` | `method_` |
| `Derivative` | `_dt` | `dt_s_` |
| `Derivative` | `_Tau` | `tau_s_` |
| `Derivative` | `_method` | `method_` |
| `FilterTF` | `_num` | `num_` |
| `FilterTF` | `_den` | `den_` |
| `FilterTF` | `uBuff` | `u_buff_` |
| `FilterTF` | `yBuff` | `y_buff_` |
| `FilterTF` | `_in` | `in_` |
| `FilterTF` | `_out` | `out_` |
| `FilterTF` | `_errorCode` | `error_code_` |
| `FilterTF2` | `_num` | `num_` |
| `FilterTF2` | `_den` | `den_` |
| `FilterTF2` | `uBuff` | `u_buff_` |
| `FilterTF2` | `yBuff` | `y_buff_` |
| `FilterTF2` | `_order` | `order_` |
| `FilterTF2` | `_in` | `in_` |
| `FilterTF2` | `_out` | `out_` |
| `FilterTF2` | `_errorCode` | `error_code_` |
| `FilterFIR` | `num` | `num_` |
| `FilterFIR` | `uBuff` | `u_buff_` |
| `FilterFIR` | `_in` | `in_` |
| `FilterFIR` | `_out` | `out_` |
| `FilterFIR` | `_errorCode` | `error_code_` |
| `FilterSS` | `_Phi` | `phi_` |
| `FilterSS` | `_Gamma` | `gamma_` |
| `FilterSS` | `_H` | `h_` |
| `FilterSS` | `_J` | `j_` |
| `FilterSS` | `_x` | `x_` |
| `FilterSS` | `_in` | `in_` |
| `FilterSS` | `_out` | `out_` |
| `FilterSS` | `_errorCode` | `error_code_` |
| `Gain` | `_K` | `value_` |
| `RectilinearTable` (protected) | `valid` | `valid_` |
| `RectilinearTable` (protected) | `value` | `value_` |
| `RectilinearTable` (protected) | `axis` | `axis_` |

Note: `uBuff`/`yBuff` in `FilterTF`/`FilterTF2` and `num`/`uBuff` in `FilterFIR`
are inconsistent with both naming conventions: no leading underscore and no trailing
underscore. `FilterSS2` already uses the correct `snake_case_` convention and serves
as the reference.

---

### 9b — Missing Unit Suffixes in Names

Names that involve physical quantities with non-obvious units must carry a suffix.

| Location | Wrong name | Correct name | Reason |
|----------|-----------|--------------|--------|
| `Derivative` member / accessor | `_Tau` / `Tau()` | `tau_s_` / `tau_s()` | time constant in seconds |
| `Derivative` JSON key | `"tau"` | `"tau_s"` | serialized field must match name convention |
| `RateLimit` member / accessor | `_dt` / `dt()` | `dt_s_` / `dt_s()` | timestep in seconds |
| `Integrator` member / accessor | `_dt` / `dt()` | `dt_s_` / `dt_s()` | timestep in seconds |
| `FilterSS2Clip` member / param | `_dt` / `dt` param | `dt_s_` / `dt_s` param | timestep in seconds |
| `FilterSS2Clip` `setLowPassFirstIIR` param | `wn` | `wn_rad_s` | `FilterSS2` already uses `wn_rad_s` — inconsistency |
| `FilterSS2Clip` `setLowPassSecondIIR` param | `wn` | `wn_rad_s` | same |
| `Gain` accessor | `K()` | `value()` | `K` is a bare symbol with no unit or semantic meaning |

---

### 9c — Abbreviations in Names

CLAUDE.md forbids abbreviations. Abbreviated names found in the control subsystem:

| Location | Abbreviated name | Full name |
|----------|-----------------|-----------|
| `DiscretizationMethod` enumerator | `FwdEuler` | `ForwardEuler` |
| `DiscretizationMethod` enumerator | `BackEuler` | `BackwardEuler` |
| `DiscretizationMethod` enumerator | `PZMatch` | `PoleZeroMatch` |
| `FilterSS2Clip` private method | `backsolve()` | `backSolve()` (also wrong case — see 9d) |
| `FilterSS2Clip` private method | `backsolve1()` | `backSolve1()` |
| `FilterSS2Clip` private method | `backsolve2()` | `backSolve2()` |
| `SISOPIDFF` member | `Kp` | `proportional_gain_` (see 9e) |
| `SISOPIDFF` member | `Ki` | `integral_gain_` |
| `SISOPIDFF` member | `Kd` | `derivative_gain_` |
| `SISOPIDFF` member | `Kff` | `feedforward_gain_` |
| `SISOPIDFF` member | `I` (Integrator) | `integrator_` |
| `SISOPIDFF` member | `D` (Derivative) | `derivative_` |
| `SISOPIDFF` member | `ffwdSignal` | `feedforward_signal_` |
| `SISOPIDFF` member | `cmdSignal` | `command_signal_` |
| `SISOPIDFF` member | `measSignal` | `measurement_signal_` |
| `SISOPIDFF` member | `measDotSignal` | `measurement_derivative_signal_` |
| `SISOPIDFF` member | `errSignal` | `error_signal_` |
| `SISOPIDFF` member | `outSignal` | `output_signal_` |
| `SISOPIDFF` member | `cmdUnwrap` | `command_unwrap_` |
| `SISOPIDFF` member | `measUnwrap` | `measurement_unwrap_` |
| `SISOPIDFF` method | `feedfwd()` | `feedForward()` |
| `SISOPIDFF` method | `cmd()` | `command()` |
| `SISOPIDFF` method | `meas()` | `measurement()` |
| `SISOPIDFF` method | `measDot()` | `measurementDerivative()` |
| `SISOPIDFF` method | `err()` | `error()` |
| `SISOPIDFF` method | `prop()` | `proportional()` |
| `SISOPIDFF` method | `deriv()` | `derivativeTerm()` |
| `SISOPIDFF` method | `integ()` | `integral()` |
| `ControlLoop` member | `pid` | `controller_` (when made private per Issue 3) |
| `ControlLoop` method | `feedfwd()` | `feedForward()` |
| `ControlLoop` method | `cmd()` | `command()` |
| `ControlLoop` method | `meas()` | `measurement()` |
| `ControlLoop` method | `err()` | `error()` |

Note: `DiscretizationMethod` enumerator renames also appear under Issue 8 (enum class
promotion). The abbreviation fix should be made at the same time.

---

### 9d — Wrong Case on Private Method Names

Private methods must be `camelCase`. `FilterSS2Clip` uses all-lowercase for its private
helper methods:

| Wrong name | Correct name |
|-----------|--------------|
| `backsolve()` | `backSolve()` |
| `backsolve1()` | `backSolve1()` |
| `backsolve2()` | `backSolve2()` |

---

### 9e — Member and Method Naming in `SISOPIDFF` and `ControlLoop`

`SISOPIDFF` and `ControlLoop` expose sub-elements and accessors with names that violate
every applicable convention. Setting aside the broader visibility issue (Issue 7):

**Members — `SISOPIDFF`:**

| Wrong name | Type | Correct name | Violations |
|-----------|------|--------------|------------|
| `Kp` | `Gain<float,3>` | `proportional_gain_` | abbreviation, missing trailing underscore |
| `Ki` | `Gain<float,3>` | `integral_gain_` | abbreviation, missing trailing underscore |
| `Kd` | `Gain<float,3>` | `derivative_gain_` | abbreviation, missing trailing underscore |
| `Kff` | `Gain<float,3>` | `feedforward_gain_` | abbreviation, missing trailing underscore |
| `I` | `Integrator` | `integrator_` | single-letter name, missing trailing underscore |
| `D` | `Derivative` | `derivative_` | single-letter name, missing trailing underscore |
| `unwrapInputs` | `bool` | `unwrap_inputs_` | not `snake_case_`; not a predicate (`is_` / `has_`) |
| `cmdSignal` | `FilterSS2Clip` | `command_signal_` | camelCase, abbreviation, no trailing underscore |
| `ffwdSignal` | `FilterSS2Clip` | `feedforward_signal_` | camelCase, abbreviation, no trailing underscore |
| `measSignal` | `FilterSS2Clip` | `measurement_signal_` | camelCase, abbreviation, no trailing underscore |
| `measDotSignal` | `FilterSS2Clip` | `measurement_derivative_signal_` | camelCase, abbreviation |
| `errSignal` | `FilterSS2Clip` | `error_signal_` | camelCase, abbreviation, no trailing underscore |
| `outSignal` | `FilterSS2Clip` | `output_signal_` | camelCase, abbreviation, no trailing underscore |
| `cmdUnwrap` | `Unwrap` | `command_unwrap_` | camelCase, abbreviation, no trailing underscore |
| `measUnwrap` | `Unwrap` | `measurement_unwrap_` | camelCase, abbreviation, no trailing underscore |

**Members — `ControlLoop`:**

| Wrong name | Type | Correct name | Violations |
|-----------|------|--------------|------------|
| `pid` | `SISOPIDFF` | `controller_` (when private) | abbreviation |

**Accessor methods — `SISOPIDFF` and `ControlLoop`:**

All accessor methods use abbreviations. Public method names must be self-documenting:

| Wrong name | Correct name |
|-----------|--------------|
| `cmd()` | `command()` |
| `meas()` | `measurement()` |
| `measDot()` | `measurementDerivative()` |
| `err()` | `error()` |
| `feedfwd()` | `feedForward()` |
| `prop()` | `proportional()` |
| `deriv()` | `derivativeTerm()` |
| `integ()` | `integral()` |

---

### 9f — Dead Code, Non-Const Parameters, and Missing `const` Qualifiers

| File | Location | Issue |
|------|----------|-------|
| `FilterSS2Clip.cpp` | `float delU = in - inPrev;` in `step()` | computed but never used; dead code |
| `FilterSS2Clip.cpp` | `static float dcTol = 1e-6;` | mutable static — should be `static constexpr float DC_TOLERANCE = 1.0e-6f;` |
| `FilterSS2Clip.cpp` | `copy(FilterSS2Clip &filt)` parameter | should be `const FilterSS2Clip&` |
| `FilterFIR.hpp` | `FilterFIR(FilterFIR &filt)` copy constructor | parameter should be `const FilterFIR&` |
| `FilterFIR.hpp` | `copy(FilterFIR &filt)` | parameter should be `const FilterFIR&` |
| `FilterFIR.hpp` | `uint8_t order()` | missing `const` qualifier |
| `FilterFIR.hpp` | `float dcGain()` | missing `const` qualifier |
| `FilterSS.hpp` | `FilterSS(FilterSS &filt)` copy constructor | parameter should be `const FilterSS&` |
| `FilterSS.hpp` | `copy(FilterSS &filt)` | parameter should be `const FilterSS&` |
| `FilterSS.hpp` | `copy(FilterSS2 &filt)` | parameter should be `const FilterSS2&` |

`dcTol` violates the `SCREAMING_CASE` convention for compile-time constants and should
be `constexpr` to make clear it cannot change at runtime.

---

### 9g — camelCase Method Parameters

Method parameters must be `snake_case` (no trailing underscore). The following methods
use camelCase parameter names:

| Class | Method | Wrong params | Correct params |
|-------|--------|-------------|----------------|
| `SISOPIDFF` | `step(float, float)` | `cmdIn`, `measIn` | `command`, `measurement` |
| `SISOPIDFF` | `step(float, float, float)` | `cmdIn`, `measIn`, `measDotIn` | `command`, `measurement`, `measurement_derivative` |
| `SISOPIDFF` | `reset(float, float, float)` | `cmdIn`, `measIn`, `outIn` | `command`, `measurement`, `output` |
| `SISOPIDFF` | `reset(float, float, float, float)` | `cmdIn`, `measIn`, `measDotIn`, `outIn` | `command`, `measurement`, `measurement_derivative`, `output` |
| `ControlLoop` | `configure(json)` | `jsonConfig` | `config` |
| `ControlLoop` | `step(float, const KinematicState&)` | `cmdIn` | `command` |
| `ControlLoop` | `reset(float, const KinematicState&)` | `cmdIn` | `command` |
| `ControlHeadingRate` | `step` / `reset` | `headingRateCmdIn` | `heading_rate_command` |
| `ControlLoadFactor` | `step` / `reset` | `loadFactorCmdIn` | `load_factor_command` |

---

### Recommended Fix

Fold renames into the migration pass that already touches each class:

| Sub-issue | When to apply |
|-----------|--------------|
| 9a — `FilterSS2Clip` members | Item D (NVI migration) |
| 9a — `FilterTF2` members | Item E (NVI migration) |
| 9a — `FilterTF`, `FilterFIR`, `FilterSS` members | Item F (NVI migration / consolidation) |
| 9a — `Limit` / `RateLimit` members | Item B (hierarchy fix) |
| 9a — `Integrator` / `Derivative` members | Item I (`SISOPIDFF` lifecycle) |
| 9a — `Gain` member | Item C (API cleanup) |
| 9a — `RectilinearTable` protected members | Gain scheduling design item |
| 9b — unit suffixes | Same pass as 9a for each class |
| 9c — enumerator abbreviations | Item A (enum class) |
| 9c — `backSolve` methods | Item D |
| 9c/9e — `SISOPIDFF` / `ControlLoop` member + method abbreviations | Item I |
| 9d — `backSolve` case | Item D |
| 9e — all `SISOPIDFF` / `ControlLoop` members and accessors | Item I |
| 9f — `FilterSS2Clip` dead code, `dcTol`, const-ref | Fix immediately (zero-risk) |
| 9f — `FilterFIR` / `FilterSS` non-const params, missing `const` | Items E–F |
| 9g — camelCase method parameters | Same pass as each class's migration |

---

## Recommended Implementation Order

Dependencies between issues constrain the order:

```
Issue 2 (Filter no-ops)  ←blocks— Issue 1 (NVI migration)  ←blocks— Issue 3 (SISOPIDFF lifecycle)
Issue 8 (enum class)     — independent, low risk
Issue 5 (RateLimit base) — independent, moderate scope
Issue 4 (Gain)           — partial: API cleanup now; full redesign deferred to gain scheduling roadmap item
Issue 6 (Unwrap NVI)     — independent
Issue 7 (public Limit)   — coordinate with Issue 3 (SISOPIDFF restructuring touches limits)
Issue 9 (naming)         — fold into Issue 1 migration
```

Suggested roadmap items, in order:

| Item | Scope |
|------|-------|
| A | `DiscretizationMethod` and `FilterError` → `enum class` (Issue 8) |
| B | `Limit` / `RateLimit` hierarchy: delete `LimitBase`; both derive from `SisoElement` directly (Issue 5) |
| C | `Gain` API cleanup: remove stubs, fix `operator=` → `set()`, `operator double()` → `operator float()`, `K()` → `value()`, `_K` → `value_`; template parameters retained (Issue 4 partial) |
| D | `FilterSS2Clip` NVI migration: `onStep()`, real lifecycle, `snake_case_` members (Issues 1, 2, 9) |
| E | `FilterTF2` NVI migration (Issues 1, 2, 9); evaluate consolidation with `FilterSS2` |
| F | `FilterTF`, `FilterFIR`, `FilterSS` — evaluate for consolidation / removal or NVI migration |
| G | `Filter` no-op defaults removed (Issue 2) — after D–F |
| H | `Unwrap` reference field design (Issue 6) |
| I | `SISOPIDFF` → `DynamicElement` lifecycle + serialization (Issue 3) — after D |
| J | `FilterSS2Clip` / `Integrator` / `Derivative` limit visibility (Issue 7) — fold into I |

Items A–C are self-contained and can proceed in any order. Item D is the critical path
prerequisite for G and I.
