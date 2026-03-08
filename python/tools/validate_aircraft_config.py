"""Validates an aircraft configuration JSON file against the v1 schema.

Usage:
    python validate_aircraft_config.py <file.json>

Exit code 0 on success; non-zero with a human-readable error message on failure.
"""

import json
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Schema version
# ---------------------------------------------------------------------------

SCHEMA_VERSION = 1

# ---------------------------------------------------------------------------
# Required fields per section
# All numeric values accept int | float (JSON may parse whole numbers as int).
# ---------------------------------------------------------------------------

_AIRCRAFT_FIELDS: list[str] = ["S_ref_m2", "mass_kg", "cl_y_beta"]

_LIFT_CURVE_FIELDS: list[str] = [
    "cl_alpha",
    "cl_max",
    "cl_min",
    "delta_alpha_stall",
    "delta_alpha_stall_neg",
    "cl_sep",
    "cl_sep_neg",
]

_INITIAL_STATE_FIELDS: list[str] = [
    "latitude_rad",
    "longitude_rad",
    "altitude_m",
    "velocity_north_mps",
    "velocity_east_mps",
    "velocity_down_mps",
    "wind_north_mps",
    "wind_east_mps",
    "wind_down_mps",
]


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _is_numeric(value: object) -> bool:
    """Return True iff value is int or float but not bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_fields(section: dict, fields: list[str], section_name: str) -> None:
    """Raise ValueError if any required field is missing or has a non-numeric type.

    Args:
        section: The JSON object for this section.
        fields: List of required field names.
        section_name: Human-readable section name used in error messages.

    Raises:
        ValueError: On missing field or wrong type.
    """
    for field in fields:
        if field not in section:
            raise ValueError(
                f"Missing required field '{field}' in '{section_name}' section."
            )
        if not _is_numeric(section[field]):
            raise ValueError(
                f"Field '{field}' in '{section_name}' must be a number, "
                f"got {type(section[field]).__name__!r}."
            )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def validate(config: dict) -> None:
    """Validate an aircraft configuration against the v1 schema.

    Checks schema version, required fields, field types, numeric ranges, and
    cross-field invariants.

    Args:
        config: Parsed JSON configuration as a Python dict.

    Raises:
        ValueError: With a descriptive message on any schema violation.
    """
    # -- schema_version ---------------------------------------------------
    if "schema_version" not in config:
        raise ValueError("Missing top-level field 'schema_version'.")
    if config["schema_version"] != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported schema_version {config['schema_version']!r}; "
            f"expected {SCHEMA_VERSION}."
        )

    # -- top-level sections -----------------------------------------------
    for section_name in ("aircraft", "lift_curve", "initial_state"):
        if section_name not in config:
            raise ValueError(f"Missing top-level section '{section_name}'.")
        if not isinstance(config[section_name], dict):
            raise ValueError(f"Section '{section_name}' must be a JSON object.")

    aircraft = config["aircraft"]
    lift_curve = config["lift_curve"]
    initial_state = config["initial_state"]

    # -- required fields and types ----------------------------------------
    _check_fields(aircraft, _AIRCRAFT_FIELDS, "aircraft")
    _check_fields(lift_curve, _LIFT_CURVE_FIELDS, "lift_curve")
    _check_fields(initial_state, _INITIAL_STATE_FIELDS, "initial_state")

    # -- range checks: aircraft -------------------------------------------
    if aircraft["mass_kg"] <= 0:
        raise ValueError(
            f"'mass_kg' must be > 0, got {aircraft['mass_kg']}."
        )
    if aircraft["S_ref_m2"] <= 0:
        raise ValueError(
            f"'S_ref_m2' must be > 0, got {aircraft['S_ref_m2']}."
        )
    if aircraft["cl_y_beta"] >= 0:
        raise ValueError(
            f"'cl_y_beta' must be < 0, got {aircraft['cl_y_beta']}."
        )

    # -- range checks: lift_curve -----------------------------------------
    if lift_curve["cl_min"] >= 0:
        raise ValueError(
            f"'cl_min' must be < 0, got {lift_curve['cl_min']}."
        )
    if lift_curve["delta_alpha_stall"] <= 0:
        raise ValueError(
            f"'delta_alpha_stall' must be > 0, got {lift_curve['delta_alpha_stall']}."
        )
    if lift_curve["delta_alpha_stall_neg"] <= 0:
        raise ValueError(
            f"'delta_alpha_stall_neg' must be > 0, got {lift_curve['delta_alpha_stall_neg']}."
        )

    # -- cross-field invariants -------------------------------------------
    if lift_curve["cl_sep"] > lift_curve["cl_max"]:
        raise ValueError(
            f"'cl_sep' ({lift_curve['cl_sep']}) must be ≤ 'cl_max' ({lift_curve['cl_max']})."
        )
    if lift_curve["cl_sep_neg"] < lift_curve["cl_min"]:
        raise ValueError(
            f"'cl_sep_neg' ({lift_curve['cl_sep_neg']}) must be ≥ "
            f"'cl_min' ({lift_curve['cl_min']})."
        )


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """Parse command-line arguments and run validation.

    Usage:
        python validate_aircraft_config.py <file.json>
    """
    if len(sys.argv) != 2:
        print(
            "Usage: python validate_aircraft_config.py <file.json>",
            file=sys.stderr,
        )
        sys.exit(1)

    path = Path(sys.argv[1])
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
        validate(config)
        print(f"OK: {path}")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
