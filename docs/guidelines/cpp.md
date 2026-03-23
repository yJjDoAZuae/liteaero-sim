# C++ Coding Guidelines

Refer to [general.md](general.md) for project-wide standards on TDD, naming, SI units, serialization, and architecture. This document covers C++-specific conventions.

---

## Language Standard

- **C++17** minimum. Use C++20 features where the toolchain supports them (concepts, ranges, `std::span`).
- Compile with warnings enabled and treated as errors: `-Wall -Wextra -Wpedantic -Werror`.
- Enable sanitizers in debug/test builds: `-fsanitize=address,undefined`.

---

## Naming Conventions (C++)

| Category | Convention | Example |
| --- | --- | --- |
| Classes / Structs | `PascalCase` | `KinematicState`, `RollController` |
| Abstract base classes | `PascalCase`, no prefix | `DynamicElement`, `SisoElement` |
| Methods | `camelCase` | `computeLoadFactor()`, `step()` |
| Private / protected members | `snake_case_` (trailing underscore) | `roll_rate_rad_s_`, `mass_kg_` |
| Public struct fields | `snake_case` (no trailing underscore) | `altitude_m`, `roll_rad` |
| Method parameters | `snake_case` (no trailing underscore) | `dt_s`, `load_factor` |
| Local variables | `snake_case` | `dt_s`, `lift_n` |
| Constants / `constexpr` | `SCREAMING_SNAKE_CASE` | `GRAVITY_MPS2`, `MAX_BANK_RAD` |
| Enums | `enum class`, `PascalCase` type, `PascalCase` values | `AutopilotMode::LateralNav` |
| Namespaces | `snake_case`, lowercase | `namespace control`, `namespace guidance` |
| Template parameters | `PascalCase` | `template <typename StateType>` |
| Macros | `SCREAMING_SNAKE_CASE` with project prefix | `LAS_ASSERT(...)` |

The trailing underscore on private/protected members is the primary visual signal that a
name is instance state, not a local or parameter. It must be applied consistently: every
`private` and `protected` data member gets it; public struct fields and all function
parameters do not.

```cpp
struct WindConfig {
    float speed_mps     = 0.0f;   // public field — no trailing underscore
    float altitude_m    = 0.0f;
};

class Integrator : public SisoElement {
public:
    void resetTo(float value);    // parameter — no trailing underscore
private:
    float dt_s_   = 0.0f;        // private member — trailing underscore
    float output_ = 0.0f;
};
```

### Unit Encoding in Names

When units are not obvious from context, encode them in the variable name:

```cpp
double altitude_m_;          // member: altitude in meters
double roll_rate_rad_s_;     // member: roll rate in rad/s
double thrust_n_;            // member: thrust in newtons
double bank_angle_rad;       // local: bank angle in radians
constexpr double GRAVITY_MPS2 = 9.80665;
```

---

## File and Directory Structure

```text
include/
  <subsystem>/
    ClassName.hpp          // public interface
src/
  <subsystem>/
    ClassName.cpp          // implementation
test/
  <subsystem>/
    ClassName_test.cpp     // unit tests
docs/
  guidelines/
```

- One class per header/source pair.
- Headers use `#pragma once`.
- Implementation files include their own header first, then standard library, then third-party, then project headers — each group separated by a blank line.

```cpp
#pragma once

// ClassName.hpp
#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

#include "ISerializable.hpp"
```

---

## Object Lifecycle Interface

Every dynamic simulation component implements this interface:

```cpp
namespace liteaerosim {

class IComponent {
public:
    virtual ~IComponent() = default;

    /// Initialize from configuration. Called once before first use.
    virtual void initialize(const nlohmann::json& config) = 0;

    /// Restore component to initial post-initialize conditions.
    virtual void reset() = 0;

    /// Advance internal state by dt seconds (SI: seconds).
    /// For higher-level components (KinematicState, Autopilot, Session) dt_s
    /// is passed at runtime. SISO dynamic elements use DynamicBlock instead,
    /// where dt_s is a fixed configuration parameter and step(float u) takes
    /// only the input signal — see docs/architecture/dynamic_block.md.
    virtual void step(double dt_s) = 0;

    /// Return a complete snapshot of internal state (SI units throughout).
    virtual nlohmann::json serialize() const = 0;

    /// Restore internal state from a snapshot produced by serialize().
    virtual void deserialize(const nlohmann::json& state) = 0;
};

} // namespace liteaerosim
```

---

## Memory Management

- Use **RAII** for all resource management. Never use raw `new`/`delete`.
- Prefer **value semantics** and stack allocation for small, fixed-size objects.
- Use `std::unique_ptr` for single ownership; `std::shared_ptr` only when shared ownership is genuinely required.
- Prefer `std::vector`, `std::array`, and standard containers over raw arrays.
- Avoid `std::shared_ptr` cycles; use `std::weak_ptr` to break them.

---

## Type Safety and Modern C++

- Prefer `enum class` over plain `enum` for all enumerations.
- Use `constexpr` for all compile-time constants instead of `#define`.
- Use `[[nodiscard]]` on functions whose return value must not be discarded.
- Use `explicit` on single-argument constructors and conversion operators.
- Prefer `auto` for type deduction where the type is obvious from context; avoid it when it obscures the type.
- Use structured bindings and range-for where they improve clarity.
- Avoid raw pointers in interfaces; use references or smart pointers.

```cpp
// Good
[[nodiscard]] double computeLoadFactor(double lift_n, double weight_n) const;

// Bad
double computeLoadFactor(double lift, double weight);  // units unclear
```

---

## SI Units Enforcement

- All function parameters and return values use SI units.
- Unit conversions are isolated in `include/units/UnitConversion.hpp`.
- Never call unit conversion functions inside physics or control computation code.

```cpp
// Domain code: pure SI
double computeBankAngle(double lateral_accel_mps2, double speed_mps) const;

// Interface/config code: convert at the boundary
double bank_rad = units::deg_to_rad(config.at("bank_angle_deg").get<double>());
```

---

## Serialization

Use [nlohmann/json](https://github.com/nlohmann/json) as the standard JSON library.

### Pattern

```cpp
nlohmann::json KinematicState::serialize() const {
    return {
        {"schema_version", 1},
        {"altitude_m", altitude_m_},
        {"speed_mps", speed_mps_},
        {"roll_rad", roll_rad_},
        {"pitch_rad", pitch_rad_},
        {"yaw_rad", yaw_rad_}
    };
}

void KinematicState::deserialize(const nlohmann::json& j) {
    const int version = j.at("schema_version").get<int>();
    if (version != 1) {
        throw std::runtime_error("KinematicState: unsupported schema version");
    }
    altitude_m_ = j.at("altitude_m").get<double>();
    speed_mps_  = j.at("speed_mps").get<double>();
    roll_rad_   = j.at("roll_rad").get<double>();
    pitch_rad_  = j.at("pitch_rad").get<double>();
    yaw_rad_    = j.at("yaw_rad").get<double>();
}
```

### Rules

- All serialized field names use SI unit suffixes: `"altitude_m"`, `"roll_rate_rad_s"`.
- Schema version is always field `"schema_version"` (integer).
- Round-trip test is mandatory for every serializable class.

---

## Testing (C++)

### Framework

Use **Google Test (gtest)** with **Google Mock (gmock)** for mocking dependencies.

### Test File Structure

```cpp
// test/control/RollController_test.cpp
#include <gtest/gtest.h>
#include "control/RollController.hpp"

namespace control {
namespace {

class RollControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_.initialize(default_config_);
    }

    RollController controller_;
    nlohmann::json default_config_ = { /* ... */ };
};

TEST_F(RollControllerTest, ZeroErrorProducesZeroCommand) {
    controller_.step(0.01);
    EXPECT_NEAR(controller_.rollRateCommand(), 0.0, 1e-9);
}

TEST_F(RollControllerTest, SerializeDeserializeRoundTrip) {
    controller_.step(0.05);
    const auto snapshot = controller_.serialize();
    RollController restored;
    restored.initialize(default_config_);
    restored.deserialize(snapshot);
    EXPECT_EQ(controller_.serialize(), restored.serialize());
}

} // namespace
} // namespace control
```

### Rules — Testing

- Test names follow `MethodName_ConditionUnderTest_ExpectedBehavior` or `GivenX_WhenY_ThenZ`.
- Tests are independent; no shared mutable state between test cases.
- Use `EXPECT_NEAR` (not `EXPECT_EQ`) for floating-point comparisons; choose an appropriate tolerance.
- Mock only external dependencies (sensors, I/O); do not mock the class under test.
- Every serializable class has a round-trip test.

---

## Build System

- Use **CMake** (3.20+).
- Tests are built and run with `ctest` or `cmake --build . --target test`.
- Test targets are defined in `test/CMakeLists.txt`.
- Enable sanitizers in the `Debug` configuration.

```cmake
# CMakePresets.json or CMakeLists.txt
target_compile_options(my_target PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
)
```

---

## External Dependency Management

### License Policy

**Prefer permissive open-source licenses.** Acceptable licenses, in order of preference:

| License | Notes |
| --- | --- |
| MIT | Preferred. Maximum compatibility. |
| BSD-2-Clause / BSD-3-Clause / Clear BSD | Preferred. Minimal restrictions. |
| Apache 2.0 | Preferred. Includes patent grant. |
| Boost Software License 1.0 | Preferred. No attribution required in binaries. |
| ISC | Preferred. Equivalent to MIT in practice. |
| LGPL v2.1 / v3 | Acceptable **only** with dynamic linking. Avoid static linking. |
| GPL (any version) | **Avoid.** Copyleft propagates to the entire linked binary. |
| Proprietary / Commercial | **Avoid** unless there is no open-source alternative and explicit approval is obtained. |

Always record the license of every dependency in the table below and in the dependency's own entry in `CMakeLists.txt` as a comment.

### Decision Tree for Adding a New Dependency

```text
Is the library in ConanCenter?
├── YES → Add to conanfile.txt; use find_package() in CMakeLists.txt  (preferred)
└── NO  → Is source available?
          ├── YES → FetchContent pattern 1b (manual target, bypass upstream CMake)
          └── NO  → Binary-only vendor in libs/<name>/                (last resort)
```

### Tier 1 — Conan (Preferred)

Use Conan for all libraries available in ConanCenter. Declare them in `conanfile.txt` and
locate them in CMake with `find_package()` after running `conan install`.

```ini
# conanfile.txt
[requires]
eigen/3.4.0
nlohmann_json/3.12.0
gtest/1.14.0
protobuf/3.21.12

[generators]
CMakeDeps
CMakeToolchain
```

```cmake
# CMakeLists.txt — after conan_toolchain.cmake is loaded
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_package(protobuf REQUIRED CONFIG)
```

Run `conan install` before configuring CMake (see `docs/installation/README.md`).

### Tier 2 — FetchContent (Not in ConanCenter)

Use FetchContent **only** for packages that are not available in ConanCenter. All current
FetchContent deps (trochoids, mcap, tinygltf) have incompatible upstream build systems and
are integrated with pattern 1b — download source only, define a manual CMake target.

**Pattern 1b — source with incompatible build system:**

```cmake
include(FetchContent)

FetchContent_Declare(
    trochoids                                       # Clear BSD license — AirLab / CMU 2023
    GIT_REPOSITORY https://github.com/castacks/trochoids.git
    GIT_TAG        38d23eb3346737fe9d6e9ff57c742113e29dfe4f
    # GIT_SHALLOW not used with a SHA — use a tag if one becomes available
)
FetchContent_GetProperties(trochoids)
if(NOT trochoids_POPULATED)
    FetchContent_Populate(trochoids)
    add_library(trochoids STATIC
        ${trochoids_SOURCE_DIR}/src/trochoids.cpp
        ${trochoids_SOURCE_DIR}/src/trochoid_utils.cpp
        ${trochoids_SOURCE_DIR}/src/DubinsStateSpace.cpp
    )
    target_include_directories(trochoids SYSTEM PUBLIC
        ${trochoids_SOURCE_DIR}/include
    )
    target_compile_features(trochoids PUBLIC cxx_std_17)
endif()
```

Rules:

- Always pin to a specific **tag or commit SHA** — never `main` or `master`.
- Use `GIT_SHALLOW TRUE` when pinning to a tag; omit it when pinning to a bare SHA.
- Record the library name, version/SHA, and license in a comment next to the `FetchContent_Declare` call.

### Tier 3 — Git Submodules

Use git submodules when:

- The library does not support CMake natively and requires a wrapper.
- You need the source to be present in the repository (e.g., for offline builds).
- The library is internal or not publicly hosted.

```bash
git submodule add https://github.com/org/library.git extern/library
git submodule update --init --recursive
```

Place submodules under `extern/` (not `libs/`). Add a `CMakeLists.txt` wrapper that creates a proper CMake target.

### Tier 3 — Vendored Binaries (`libs/`)

Reserve `libs/` **exclusively for binary-only libraries** where source is genuinely unavailable and there is no open-source alternative. This is a last resort.

Rules for vendored binaries:

- Include the library's `LICENSE` file alongside the binary.
- Create a CMake `IMPORTED` target so consumers link cleanly:

```cmake
# cmake/ImportedLibs.cmake

# example_lib — (Proprietary, Vendor Inc.)
# Binary-only distribution. Source not available. Platform: linux-x86_64, GCC 11, C++17.
add_library(example_lib STATIC IMPORTED)
set_target_properties(example_lib PROPERTIES
    IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/example_lib/libexample.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/libs/example_lib/include"
)
```

- Document the source URL (if known), version/commit, and license in the CMake wrapper comment.
- **Always note the platform** the binary was compiled for (architecture, OS, compiler, C++ standard). A `.a` is not portable across platforms.
- If source becomes available, migrate to FetchContent and remove the binary from the repo.

### Dependency Registry

Maintain this table as a comment header in the root `CMakeLists.txt` dependency block:

```text
# Dependency       | Version/Commit                           | License      | Method
# ----------------|------------------------------------------|--------------|----------------------
# eigen            | 3.4.0                                    | MPL-2        | Conan (find_package)
# nlohmann_json    | 3.12.0                                   | MIT          | Conan (find_package)
# gtest            | 1.14.0                                   | BSD-3-Clause | Conan (find_package)
# protobuf         | 3.21.12                                  | BSD-3-Clause | Conan (find_package)
# trochoids        | 38d23eb3346737fe9d6e9ff57c742113e29dfe4f | Clear BSD    | FetchContent (1b)
# mcap             | releases/cpp/v1.4.0                      | MIT          | FetchContent (1b)
# tinygltf         | v2.9.3                                   | MIT          | FetchContent (1b)
```

---

## Error Handling

- Use exceptions for programming errors and unrecoverable state violations (`std::logic_error`, `std::runtime_error`).
- Use return codes or `std::optional`/`std::expected` for expected failure modes (e.g., waypoint not found).
- Never use exceptions for normal control flow.
- Assert preconditions at function entry in debug builds using `LAS_ASSERT` or `assert`.
- Do not `catch (...)` silently; always log or re-throw.

---

## Code Style

- Indentation: **4 spaces** (no tabs).
- Brace style: **K&R** (opening brace on same line for functions and control structures).
- Line length: **120 characters** maximum.
- Use `clang-format` with the project's `.clang-format` file for automated formatting.
- Use `clang-tidy` for static analysis.

### clang-format baseline

```yaml
# .clang-format
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 120
AccessModifierOffset: -4
```
