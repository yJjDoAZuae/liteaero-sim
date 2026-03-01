# LiteAeroSim — Claude Code Guidelines

This file provides Claude Code with the project's development standards. Read the linked documents before writing or modifying any code.

---

## Coding Guidelines

| Document | Scope |
|---|---|
| [docs/guidelines/general.md](docs/guidelines/general.md) | TDD, naming standards, SI units, serialization, architecture |
| [docs/guidelines/cpp.md](docs/guidelines/cpp.md) | C++ conventions, tooling, testing with gtest, CMake |
| [docs/guidelines/python.md](docs/guidelines/python.md) | Python conventions, type hints, testing with pytest, tooling |

**Always consult the relevant guideline document before writing code.** When guidelines conflict with an existing pattern in the codebase, flag it and ask rather than silently adopting the worse pattern.

---

## Quick Reference

### Non-Negotiable Rules

1. **TDD** — Write a failing test before writing production code.
2. **SI units** — All stored values are meters, radians, seconds, kilograms, newtons. No exceptions inside the domain layer.
3. **Unit conversion** — Only at the outermost interface (display, config file parsing). Never inside computation code.
4. **Serialization** — Every stateful dynamic component implements `serialize()` and `deserialize()`. Round-trip test is required.
5. **Naming** — Names are self-documenting. Abbreviations and Hungarian notation are forbidden. Encode units in names when not obvious from context.
6. **No backward compatibility** — This codebase is not legacy code and must not be treated as such. Do not add forwarding shims, deprecated aliases, compatibility wrappers, or any other construct whose sole purpose is to preserve an old interface. When an API changes, update all call sites directly. When code is removed, remove it completely.
7. **American English** — Use American spellings in all comments, documentation, identifiers, and string literals (e.g. "color" not "colour", "serialize" not "serialise").
8. **Documentation reflects the present** — Keep all documentation accurate to the current state of the code. Remove or update any section that describes a previous design or a state that no longer exists. Do not leave historical notes inline; version control preserves history. Proposed extensions to the architecture (new subsystems, planned classes, future algorithms) may be documented, but must be clearly labeled as proposed or not yet implemented — never described as if they already exist.

### Architecture at a Glance

```
Interface Layer      ← unit conversions, config parsing, display
Application Layer    ← scenario setup, session management
Domain Layer         ← physics, guidance, control, propulsion (pure SI, no I/O)
Infrastructure       ← math utilities, serialization helpers, unit conversion module
```

### Component Lifecycle

Every dynamic element implements: `initialize(config)` → `reset()` → `step(u)` → `serialize()` / `deserialize(state)`

### Project Language Mix

- **C++17** — core simulation engine (physics, control, guidance, sensors, propulsion)
- **Python 3.10+** — analysis, tooling, and scripting interfaces

---

## Project Documentation

Full project documentation lives in [docs/](docs/README.md).

| Document | Contents |
|---|---|
| [docs/architecture/overview.md](docs/architecture/overview.md) | Layer model, subsystem map, coordinate frames, data flow |
| [docs/architecture/dynamic_block.md](docs/architecture/dynamic_block.md) | **Design authority** for all SISO dynamic elements — `DynamicBlock` NVI pattern, serialization contract, logging interface, migration strategy |
| [docs/algorithms/filters.md](docs/algorithms/filters.md) | Filter discretization, Tustin bilinear prewarping, control algorithms |
| [docs/dependencies/README.md](docs/dependencies/README.md) | License policy, dependency registry, FetchContent patterns |
| [docs/installation/README.md](docs/installation/README.md) | Build from source, toolchain setup, first run |
| [docs/testing/strategy.md](docs/testing/strategy.md) | TDD strategy, required test categories, coverage, known failures |
| [docs/examples/siso_elements.md](docs/examples/siso_elements.md) | Usage examples for filters, integrators, PID, serialization, logging |
| [docs/guidelines/](docs/guidelines/) | Coding standards — general, C++, Python |

**Before implementing any new dynamic element**, read [docs/architecture/dynamic_block.md](docs/architecture/dynamic_block.md).

---

## Project Structure

```
include/             C++ public headers (mirrored by subsystem)
src/                 C++ implementation files
test/                C++ unit tests (Google Test)
extern/              Git submodules (source libraries without CMake support)
python/              Python source and notebooks
docs/
  architecture/      System design documents
  algorithms/        Algorithm design and math
  guidelines/        Coding standards (this project's law)
  examples/          Usage examples
  dependencies/      External library registry
  installation/      Build and setup instructions
  testing/           Test strategy and coverage requirements
cmake/               CMake helpers
```

---

## External Dependencies

**License preference:** MIT → BSD-2/3/Clear BSD → Apache 2.0 → Boost → ISC. Avoid GPL. LGPL only with dynamic linking.

**C++ integration method (in priority order):**

| Situation | Method |
|---|---|
| Source available, CMake supported | `FetchContent` in `cmake/Dependencies.cmake` — pin to a tag/SHA |
| Source available, no CMake | Git submodule under `extern/`, write a CMake wrapper |
| Binary-only (no source available) | Vendor under `libs/` with a CMake `IMPORTED` target — last resort |

**Current dependencies:**

| Library | Version | License | Method |
|---|---|---|---|
| googletest | v1.14.0 | BSD-3-Clause | FetchContent |
| nlohmann_json | v3.11.3 | MIT | FetchContent |
| trochoids | 38d23eb | Clear BSD | FetchContent (source from castacks/trochoids; bypasses catkin CMakeLists) |

---

## Key Conventions at a Glance

| Topic | Rule |
|---|---|
| C++ naming | `PascalCase` classes, `camelCase` methods, `snake_case_` members, `SCREAMING` constants |
| Python naming | `PascalCase` classes, `snake_case` functions/variables, `SCREAMING` constants |
| Booleans | Named as predicates: `is_converged`, `has_waypoint` |
| Units in names | `altitude_m`, `roll_rate_rad_s`, `thrust_n` when not obvious |
| Test naming | Mirror source file with `_test` suffix (C++) or `test_` prefix (Python) |
| Serialized fields | SI unit suffix in field name: `"altitude_m"`, `"roll_rate_rad_s"` |
| Schema version | Field `"schema_version"` (int) in every serialized object |
| New dependency | Check license first; use FetchContent if source is available |
| Angular velocity notation | $\boldsymbol{\omega}_{A/B}^C$: rotation of frame $A$ relative to frame $B$, expressed in frame $C$ |
