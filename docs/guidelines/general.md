# General Coding and Development Guidelines

## Development Philosophy

### Test-Driven Development (TDD)

All new functionality must follow the **Red-Green-Refactor** cycle:

1. **Red** — Write a failing test that defines the desired behavior.
2. **Green** — Write the minimal code required to pass the test.
3. **Refactor** — Clean up the implementation without changing behavior, keeping all tests green.

Rules:
- No production code is written without a failing test that motivates it.
- Each test covers exactly one behavior or requirement.
- Tests are committed alongside (or before) the code they cover.
- Tests must be fast, isolated, repeatable, and self-validating.
- Aim for a test pyramid: many unit tests, fewer integration tests, few end-to-end tests.

### SOLID Principles

| Principle | Summary |
|---|---|
| Single Responsibility | A class or module has one reason to change. |
| Open/Closed | Open for extension, closed for modification. |
| Liskov Substitution | Subtypes must be substitutable for their base types. |
| Interface Segregation | Prefer narrow, focused interfaces over wide ones. |
| Dependency Inversion | Depend on abstractions, not concretions. |

---

## Naming Standards

Clear, unambiguous naming is mandatory. Names must communicate intent without requiring a comment.

### General Rules

- Names must be descriptive and self-documenting.
- Abbreviations are forbidden unless universally understood in the domain (e.g., `pid`, `imu`, `gps`).
- Boolean names must read as a predicate: `is_converged`, `has_waypoint`, `can_engage`.
- Names must not encode type information (no Hungarian notation): prefer `altitude` over `dAltitude`.
- Acronyms are treated as words: `ImuSensor`, not `IMUSensor`; `pid_controller`, not `PID_controller`.

### Naming by Category

| Category | Convention | Example |
|---|---|---|
| Classes / Types | `PascalCase` | `KinematicState`, `AutopilotMode` |
| Functions / Methods | `camelCase` (C++) or `snake_case` (Python) | `computeLoadFactor()` / `compute_load_factor()` |
| Local variables | `snake_case` | `roll_rate`, `target_altitude` |
| Member variables | `snake_case` with trailing `_` (C++) | `roll_rate_`, `mass_kg_` |
| Constants | `SCREAMING_SNAKE_CASE` | `MAX_BANK_ANGLE_RAD`, `GRAVITY_MPS2` |
| Namespaces / Packages | `snake_case`, lowercase | `control`, `guidance`, `propulsion` |
| Files | `PascalCase` for classes, `snake_case` for others | `KinematicState.cpp`, `unit_conversion.cpp` |
| Test files | Mirror the file under test with `_test` suffix | `KinematicState_test.cpp`, `test_kinematic_state.py` |

---

## SI Units — The Project Standard

### Mandate

**All internally stored values use SI base units: meters (m), radians (rad), and seconds (s).**

This applies to every variable, field, parameter, function argument, and return value throughout the codebase.

| Quantity | Unit | Symbol |
|---|---|---|
| Length / distance | meter | m |
| Angle | radian | rad |
| Time | second | s |
| Mass | kilogram | kg |
| Speed | meter per second | m/s |
| Acceleration | meter per second squared | m/s² |
| Angular rate | radian per second | rad/s |
| Force | newton | N |
| Pressure | pascal | Pa |
| Temperature | kelvin | K |
| Density | kilogram per cubic meter | kg/m³ |

### Enforcement

- Variable names **should** encode units when not obvious from context: `altitude_m`, `bank_angle_rad`, `thrust_n`.
- Unit conversion functions must live in a dedicated `unit_conversion` module/header, never inline in computation code.
- Conversions to other units (degrees, feet, knots, etc.) occur **only** at the outermost interface layer:
  - Display / HUD rendering
  - Human-edited configuration file parsing (YAML, JSON, etc.)
  - External data bus outputs (e.g., flight data recorder export)
- Internal computation functions must never accept or return non-SI values.

### Conversion Module Pattern

```
units::deg_to_rad(degrees)    // input boundary: config file parser
units::rad_to_deg(radians)    // output boundary: display layer only
units::ft_to_m(feet)
units::kts_to_mps(knots)
```

---

## Serialization and Deserialization

### Mandate

Every dynamic element (any object with internal state that evolves over time) must implement serialization and deserialization of its full internal state as a standard interface.

### Purpose

- Enables save/restore of simulation state (pause, rewind, replay).
- Enables logging and post-flight analysis.
- Enables reproducible testing with fixed initial conditions.
- Enables inter-process or inter-language data exchange.

### Rules

- Serialized format is **JSON** by default for human-readable interchange; binary formats are acceptable for performance-critical logging.
- Serialized representations use SI units — never convert to other units in serialized output.
- Every serializable class must implement:
  - `serialize()` — returns a complete snapshot of internal state.
  - `deserialize(data)` — restores state from a snapshot; validates schema version.
- Serialization must be round-trip lossless (serialize then deserialize must yield identical state).
- A schema version field is included in all serialized output to support forward compatibility.
- Serialization interfaces are covered by dedicated round-trip unit tests.

---

## Architectural Design Patterns

### Separation of Concerns

The simulation is structured in layers. Each layer has a single well-defined responsibility:

```
┌─────────────────────────────────────────┐
│  Interface Layer (I/O, display, config)  │  ← Unit conversion lives here
├─────────────────────────────────────────┤
│  Application Layer (scenario, session)   │
├─────────────────────────────────────────┤
│  Domain Layer (physics, guidance, ctrl)  │  ← All SI, no I/O
├─────────────────────────────────────────┤
│  Infrastructure (math, serialization)    │
└─────────────────────────────────────────┘
```

### Component Model

- Simulation elements are **components** with a uniform lifecycle interface:
  - `initialize(config)` — set up from configuration; `config` includes `dt_s` for elements that need it.
  - `reset()` — return to initial conditions.
  - `step(u)` — advance state by one timestep; timestep is fixed at `initialize()` time.
  - `serialize()` / `deserialize(data)` — state snapshot.
- Components are composed, not inherited, wherever possible.

### Dependency Injection

- Components receive their dependencies (sensors, models, data buses) via constructor or factory — never via global state or singletons.
- This makes unit testing straightforward: inject mock dependencies.

### Observer / Event Pattern

- Use observer/callback patterns for decoupled communication between subsystems (e.g., mode changes, fault annunciation).
- Avoid tight coupling between producers and consumers of events.

---

## External Dependencies and Licensing

### License Policy

**Default to permissive open-source libraries.** A permissive license places minimal restrictions on use, distribution, and linking, preserving maximum flexibility for the project.

Acceptable licenses (preferred first):

| License | Notes |
|---|---|
| MIT | Preferred. |
| BSD-2/3-Clause, Clear BSD | Preferred. |
| Apache 2.0 | Preferred. Includes explicit patent grant. |
| Boost Software License 1.0 | Preferred. No binary attribution required. |
| ISC | Preferred. Functionally equivalent to MIT. |
| LGPL (any) | Acceptable only with dynamic linking. Never link statically against LGPL code. |
| GPL (any) | Avoid. Copyleft propagates to the linked binary. |
| Proprietary | Avoid unless no open-source alternative exists and explicit approval is obtained. |

### Dependency Selection Criteria

When evaluating a new dependency, consider in order:
1. **License** — must be permissive (see above).
2. **Maintenance** — active development or stable/complete.
3. **Minimal footprint** — prefer focused, single-purpose libraries over large frameworks.
4. **Source availability** — prefer source-distributed libraries over binary blobs.
5. **CMake support** — prefer libraries that integrate cleanly with CMake.

All dependencies must be documented with their version, license, and integration method. See the C++ and Python guidelines for language-specific dependency management.

---

## Code Review Standards

- All code is reviewed before merging to `main`.
- Tests must pass before review begins.
- Review checklist:
  - [ ] Tests present and meaningful
  - [ ] Naming follows standards
  - [ ] SI units used throughout internal code
  - [ ] No unnecessary coupling
  - [ ] Serialization implemented for new stateful components
  - [ ] No hardcoded magic numbers (use named constants)
  - [ ] New dependency license is permissive and recorded in the dependency registry
  - [ ] No backward-compatibility shims, deprecated aliases, or forwarding wrappers (all call sites updated directly)

---

## Version Control

- Commit messages use imperative mood: "Add roll rate filter" not "Added roll rate filter".
- Each commit is self-contained and leaves the build passing.
- Feature branches are short-lived; integrate frequently.
- Do not commit commented-out code. Delete unused code; version control preserves history.
