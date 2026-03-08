"""Tests for validate_aircraft_config.py."""

import copy

import pytest

from validate_aircraft_config import validate

# ---------------------------------------------------------------------------
# Shared fixture — deep-copied per test so mutations do not bleed across tests
# ---------------------------------------------------------------------------

VALID_CONFIG: dict = {
    "schema_version": 1,
    "aircraft": {
        "S_ref_m2": 16.2,
        "mass_kg": 1045.0,
        "cl_y_beta": -0.60,
    },
    "lift_curve": {
        "cl_alpha": 5.1,
        "cl_max": 1.80,
        "cl_min": -1.20,
        "delta_alpha_stall": 0.262,
        "delta_alpha_stall_neg": 0.262,
        "cl_sep": 1.05,
        "cl_sep_neg": -0.80,
    },
    "initial_state": {
        "latitude_rad": 0.0,
        "longitude_rad": 0.0,
        "altitude_m": 300.0,
        "velocity_north_mps": 55.0,
        "velocity_east_mps": 0.0,
        "velocity_down_mps": 0.0,
        "wind_north_mps": 0.0,
        "wind_east_mps": 0.0,
        "wind_down_mps": 0.0,
    },
}


def valid() -> dict:
    """Return a deep copy of VALID_CONFIG."""
    return copy.deepcopy(VALID_CONFIG)


# ---------------------------------------------------------------------------
# Valid config
# ---------------------------------------------------------------------------


def test_valid_config_passes() -> None:
    """A fully valid configuration must not raise."""
    validate(valid())


# ---------------------------------------------------------------------------
# Schema version
# ---------------------------------------------------------------------------


def test_missing_schema_version_raises() -> None:
    cfg = valid()
    del cfg["schema_version"]
    with pytest.raises(ValueError, match="schema_version"):
        validate(cfg)


def test_wrong_schema_version_raises() -> None:
    cfg = valid()
    cfg["schema_version"] = 99
    with pytest.raises(ValueError, match="schema_version"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing top-level sections
# ---------------------------------------------------------------------------


def test_missing_aircraft_section_raises() -> None:
    cfg = valid()
    del cfg["aircraft"]
    with pytest.raises(ValueError, match="aircraft"):
        validate(cfg)


def test_missing_lift_curve_section_raises() -> None:
    cfg = valid()
    del cfg["lift_curve"]
    with pytest.raises(ValueError, match="lift_curve"):
        validate(cfg)


def test_missing_initial_state_section_raises() -> None:
    cfg = valid()
    del cfg["initial_state"]
    with pytest.raises(ValueError, match="initial_state"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing required fields — aircraft
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("field", ["S_ref_m2", "mass_kg", "cl_y_beta"])
def test_missing_aircraft_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["aircraft"][field]
    with pytest.raises(ValueError, match=field):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing required fields — lift_curve
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "field",
    [
        "cl_alpha",
        "cl_max",
        "cl_min",
        "delta_alpha_stall",
        "delta_alpha_stall_neg",
        "cl_sep",
        "cl_sep_neg",
    ],
)
def test_missing_lift_curve_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["lift_curve"][field]
    with pytest.raises(ValueError, match=field):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing required fields — initial_state
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "field",
    [
        "latitude_rad",
        "longitude_rad",
        "altitude_m",
        "velocity_north_mps",
        "velocity_east_mps",
        "velocity_down_mps",
        "wind_north_mps",
        "wind_east_mps",
        "wind_down_mps",
    ],
)
def test_missing_initial_state_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["initial_state"][field]
    with pytest.raises(ValueError, match=field):
        validate(cfg)


# ---------------------------------------------------------------------------
# Wrong field types
# ---------------------------------------------------------------------------


def test_wrong_type_aircraft_field_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["mass_kg"] = "1045"  # string instead of number
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


def test_wrong_type_lift_curve_field_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_max"] = "1.8"  # string instead of number
    with pytest.raises(ValueError, match="cl_max"):
        validate(cfg)


def test_bool_type_rejected() -> None:
    """Booleans must not be accepted as numeric values even though bool is a subtype of int."""
    cfg = valid()
    cfg["aircraft"]["mass_kg"] = True
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Integer values accepted (JSON may parse whole numbers as int)
# ---------------------------------------------------------------------------


def test_integer_values_accepted() -> None:
    cfg = valid()
    cfg["aircraft"]["mass_kg"] = 1045  # int, no decimal point
    cfg["lift_curve"]["cl_max"] = 2  # int
    validate(cfg)  # must not raise


# ---------------------------------------------------------------------------
# Range / invariant violations
# ---------------------------------------------------------------------------


def test_mass_kg_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["mass_kg"] = 0.0
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


def test_mass_kg_negative_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["mass_kg"] = -1.0
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


def test_S_ref_m2_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["S_ref_m2"] = 0.0
    with pytest.raises(ValueError, match="S_ref_m2"):
        validate(cfg)


def test_cl_y_beta_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["cl_y_beta"] = 0.0
    with pytest.raises(ValueError, match="cl_y_beta"):
        validate(cfg)


def test_cl_y_beta_positive_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["cl_y_beta"] = 0.5
    with pytest.raises(ValueError, match="cl_y_beta"):
        validate(cfg)


def test_cl_min_zero_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_min"] = 0.0
    with pytest.raises(ValueError, match="cl_min"):
        validate(cfg)


def test_cl_min_positive_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_min"] = 0.3
    with pytest.raises(ValueError, match="cl_min"):
        validate(cfg)


def test_delta_alpha_stall_zero_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["delta_alpha_stall"] = 0.0
    with pytest.raises(ValueError, match="delta_alpha_stall"):
        validate(cfg)


def test_delta_alpha_stall_neg_zero_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["delta_alpha_stall_neg"] = 0.0
    with pytest.raises(ValueError, match="delta_alpha_stall_neg"):
        validate(cfg)


def test_cl_sep_exceeds_cl_max_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_sep"] = 2.0  # cl_max = 1.80
    with pytest.raises(ValueError, match="cl_sep"):
        validate(cfg)


def test_cl_sep_neg_below_cl_min_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_sep_neg"] = -2.0  # cl_min = -1.20
    with pytest.raises(ValueError, match="cl_sep_neg"):
        validate(cfg)
