# Python Coding Guidelines

Refer to [general.md](general.md) for project-wide standards on TDD, naming, SI units, serialization, and architecture. This document covers Python-specific conventions.

---

## Language and Tooling

- **Python 3.10+** minimum (use `match` statements, `X | Y` union types).
- **Type hints are mandatory** on all public functions and class attributes.
- Use `mypy` for static type checking (strict mode).
- Use `ruff` for linting and `black` for formatting.
- Use `pytest` as the test framework.
- Use `pytest-cov` for coverage reporting.

---

## Naming Conventions (Python)

| Category | Convention | Example |
| --- | --- | --- |
| Classes | `PascalCase` | `KinematicState`, `RollController` |
| Functions / Methods | `snake_case` | `compute_load_factor()`, `step()` |
| Variables | `snake_case` | `roll_rate_rad_s`, `altitude_m` |
| Constants | `SCREAMING_SNAKE_CASE` | `GRAVITY_MPS2`, `MAX_BANK_RAD` |
| Private attributes | `_snake_case` (single leading underscore) | `self._roll_rate_rad_s` |
| Abstract base classes | `PascalCase` | `Component`, `ISerializable` |
| Modules / Packages | `snake_case`, lowercase | `kinematic_state.py`, `control/` |
| Type aliases | `PascalCase` | `StateDict = dict[str, float]` |

### Unit Encoding in Names

When units are not obvious from context, encode them in the variable name:

```python
altitude_m: float           # altitude in meters
roll_rate_rad_s: float      # roll rate in rad/s
thrust_n: float             # thrust in newtons
GRAVITY_MPS2: float = 9.80665
```

---

## Package and Module Structure

```text
src/
  las/                      # top-level package
    __init__.py
    units/
      __init__.py
      conversion.py         # unit conversion functions (SI boundary)
    control/
      __init__.py
      roll_controller.py
    guidance/
      __init__.py
    kinematic_state.py
test/
  control/
    test_roll_controller.py
  test_kinematic_state.py
```

---

## Object Lifecycle Interface

Every dynamic simulation component implements this abstract base class:

```python
from abc import ABC, abstractmethod
from typing import Any

class Component(ABC):
    """Base interface for all dynamic simulation components."""

    @abstractmethod
    def initialize(self, config: dict[str, Any]) -> None:
        """Set up component from configuration dict. Called once before use."""

    @abstractmethod
    def reset(self) -> None:
        """Restore component to initial post-initialize conditions."""

    @abstractmethod
    def step(self, dt_s: float) -> None:
        """Advance internal state by dt_s seconds (SI: seconds)."""

    @abstractmethod
    def serialize(self) -> dict[str, Any]:
        """Return a complete snapshot of internal state. All values in SI units."""

    @abstractmethod
    def deserialize(self, state: dict[str, Any]) -> None:
        """Restore internal state from a snapshot produced by serialize()."""
```

---

## Type Hints

- All public function signatures must have complete type hints.
- Use `from __future__ import annotations` for forward references.
- Use `dataclasses` or `pydantic` for data-heavy configuration objects.
- Prefer `Sequence` over `list` and `Mapping` over `dict` in function signatures where read-only access is sufficient.

```python
from __future__ import annotations
from dataclasses import dataclass, field

@dataclass
class KinematicState:
    altitude_m: float = 0.0
    speed_mps: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
```

---

## SI Units Enforcement

- All function parameters and return values use SI units.
- Unit conversions are isolated in `las/units/conversion.py`.
- Never call unit conversion functions inside physics or control computation code.

```python
# las/units/conversion.py — the SI boundary
import math

def deg_to_rad(degrees: float) -> float:
    return math.radians(degrees)

def rad_to_deg(radians: float) -> float:
    return math.degrees(radians)

def ft_to_m(feet: float) -> float:
    return feet * 0.3048

def m_to_ft(meters: float) -> float:
    return meters / 0.3048

def kts_to_mps(knots: float) -> float:
    return knots * 0.514444

def mps_to_kts(mps: float) -> float:
    return mps / 0.514444
```

```python
# Config parser (outermost interface) — convert here and only here
from las.units.conversion import deg_to_rad

bank_angle_rad = deg_to_rad(config["bank_angle_deg"])
```

---

## Serialization

Use the standard library `json` module for JSON serialization. All serialized output must be in SI units.

### Pattern

```python
import json
from typing import Any

SCHEMA_VERSION = 1

class KinematicState(Component):

    def serialize(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "altitude_m": self._altitude_m,
            "speed_mps": self._speed_mps,
            "roll_rad": self._roll_rad,
            "pitch_rad": self._pitch_rad,
            "yaw_rad": self._yaw_rad,
        }

    def deserialize(self, state: dict[str, Any]) -> None:
        version = state["schema_version"]
        if version != SCHEMA_VERSION:
            raise ValueError(f"KinematicState: unsupported schema version {version}")
        self._altitude_m = state["altitude_m"]
        self._speed_mps  = state["speed_mps"]
        self._roll_rad   = state["roll_rad"]
        self._pitch_rad  = state["pitch_rad"]
        self._yaw_rad    = state["yaw_rad"]

    def to_json(self) -> str:
        return json.dumps(self.serialize(), indent=2)

    @classmethod
    def from_json(cls, json_str: str) -> "KinematicState":
        obj = cls()
        obj.deserialize(json.loads(json_str))
        return obj
```

### Rules

- All serialized field names use SI unit suffixes: `"altitude_m"`, `"roll_rate_rad_s"`.
- Schema version is always field `"schema_version"` (integer).
- Round-trip test is mandatory for every serializable class.

---

## Testing (Python)

### Framework

Use **pytest** with **pytest-cov** for coverage and **unittest.mock** for mocking.

### Test File Structure

```python
# test/control/test_roll_controller.py

import pytest
from las.control.roll_controller import RollController

SCHEMA_VERSION = 1

@pytest.fixture
def default_config() -> dict:
    return {
        "kp": 1.0,
        "ki": 0.1,
        "kd": 0.05,
    }

@pytest.fixture
def controller(default_config) -> RollController:
    ctrl = RollController()
    ctrl.initialize(default_config)
    return ctrl


class TestRollController:

    def test_zero_error_produces_zero_command(self, controller: RollController) -> None:
        controller.step(0.01)
        assert controller.roll_rate_command_rad_s == pytest.approx(0.0, abs=1e-9)

    def test_serialize_deserialize_round_trip(self, controller: RollController, default_config) -> None:
        controller.step(0.05)
        snapshot = controller.serialize()

        restored = RollController()
        restored.initialize(default_config)
        restored.deserialize(snapshot)

        assert controller.serialize() == restored.serialize()

    def test_deserialize_rejects_unknown_schema_version(self, controller: RollController) -> None:
        bad_state = controller.serialize()
        bad_state["schema_version"] = 999
        with pytest.raises(ValueError, match="schema version"):
            controller.deserialize(bad_state)
```

### Rules — Testing

- Test names: `test_<method>_<condition>_<expected>` or descriptive intent names.
- Use `pytest.approx` for all floating-point comparisons; set an appropriate tolerance.
- Tests are independent; fixtures handle setup.
- Mock only external dependencies (`unittest.mock.patch`, `MagicMock`); do not mock the class under test.
- Every serializable class has a round-trip test and a version rejection test.
- Run tests with `pytest --cov=las --cov-report=term-missing`.

---

## Code Style

- Indentation: **4 spaces** (no tabs).
- Line length: **100 characters** maximum.
- Use `black` for automated formatting, `ruff` for linting.
- Use `isort` (or ruff's import sorter) to order imports: standard library → third-party → local.

```toml
# pyproject.toml
[tool.black]
line-length = 100

[tool.ruff]
line-length = 100
select = ["E", "F", "W", "I", "N", "UP"]

[tool.mypy]
strict = true
python_version = "3.10"

[tool.pytest.ini_options]
testpaths = ["test"]
```

---

## Docstrings

Public modules, classes, and functions must have docstrings. Use **Google style**:

```python
def compute_load_factor(lift_n: float, weight_n: float) -> float:
    """Compute the load factor (g-loading).

    Args:
        lift_n: Aerodynamic lift force in newtons.
        weight_n: Aircraft weight in newtons.

    Returns:
        Dimensionless load factor (1.0 = 1g).

    Raises:
        ValueError: If weight_n is zero or negative.
    """
    if weight_n <= 0.0:
        raise ValueError(f"weight_n must be positive, got {weight_n}")
    return lift_n / weight_n
```

---

## Error Handling

- Raise specific exceptions (`ValueError`, `TypeError`, `RuntimeError`) with descriptive messages.
- Use `ValueError` for bad input data (e.g., unsupported schema version, out-of-range SI value).
- Use `RuntimeError` for illegal state transitions.
- Do not silence exceptions with bare `except:` clauses.
- Validate inputs at public API boundaries; trust internal calls within the module.

---

## Notebooks (Jupyter)

Notebooks in `python/` are for analysis and visualization only — not for production code.

- All production logic lives in `src/las/`, not in notebooks.
- Notebooks import from `las` like any other consumer.
- Notebooks are not subject to the same test coverage requirements but must follow naming and SI unit conventions.
- Cell outputs must be cleared before committing (use `nbstripout`).
