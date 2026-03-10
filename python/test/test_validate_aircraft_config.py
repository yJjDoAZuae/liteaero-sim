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
        "cl_y_beta": -0.60,
        "ar": 7.47,
        "e": 0.80,
        "cd0": 0.027,
    },
    "airframe": {
        "g_max_nd": 3.8,
        "g_min_nd": -1.52,
        "tas_max_mps": 82.3,
        "mach_max_nd": 0.25,
    },
    "inertia": {
        "mass_kg": 1045.0,
        "Ixx_kgm2": 1285.0,
        "Iyy_kgm2": 1825.0,
        "Izz_kgm2": 2667.0,
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


def test_missing_airframe_section_raises() -> None:
    cfg = valid()
    del cfg["airframe"]
    with pytest.raises(ValueError, match="airframe"):
        validate(cfg)


def test_missing_inertia_section_raises() -> None:
    cfg = valid()
    del cfg["inertia"]
    with pytest.raises(ValueError, match="inertia"):
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


@pytest.mark.parametrize("field", ["S_ref_m2", "cl_y_beta", "ar", "e", "cd0"])
def test_missing_aircraft_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["aircraft"][field]
    with pytest.raises(ValueError, match=field):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing required fields — airframe
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("field", ["g_max_nd", "g_min_nd", "tas_max_mps", "mach_max_nd"])
def test_missing_airframe_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["airframe"][field]
    with pytest.raises(ValueError, match=field):
        validate(cfg)


# ---------------------------------------------------------------------------
# Missing required fields — inertia
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("field", ["mass_kg", "Ixx_kgm2", "Iyy_kgm2", "Izz_kgm2"])
def test_missing_inertia_field_raises(field: str) -> None:
    cfg = valid()
    del cfg["inertia"][field]
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
    cfg["aircraft"]["S_ref_m2"] = "16.2"  # string instead of number
    with pytest.raises(ValueError, match="S_ref_m2"):
        validate(cfg)


def test_wrong_type_lift_curve_field_raises() -> None:
    cfg = valid()
    cfg["lift_curve"]["cl_max"] = "1.8"  # string instead of number
    with pytest.raises(ValueError, match="cl_max"):
        validate(cfg)


def test_bool_type_rejected() -> None:
    """Booleans must not be accepted as numeric values even though bool is a subtype of int."""
    cfg = valid()
    cfg["inertia"]["mass_kg"] = True
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Integer values accepted (JSON may parse whole numbers as int)
# ---------------------------------------------------------------------------


def test_integer_values_accepted() -> None:
    cfg = valid()
    cfg["inertia"]["mass_kg"] = 1045  # int, no decimal point
    cfg["lift_curve"]["cl_max"] = 2  # int
    validate(cfg)  # must not raise


# ---------------------------------------------------------------------------
# Range / invariant violations — aircraft
# ---------------------------------------------------------------------------


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


def test_ar_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["ar"] = 0.0
    with pytest.raises(ValueError, match="ar"):
        validate(cfg)


def test_ar_negative_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["ar"] = -1.0
    with pytest.raises(ValueError, match="ar"):
        validate(cfg)


def test_e_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["e"] = 0.0
    with pytest.raises(ValueError, match="'e'"):
        validate(cfg)


def test_e_greater_than_one_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["e"] = 1.1
    with pytest.raises(ValueError, match="'e'"):
        validate(cfg)


def test_cd0_zero_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["cd0"] = 0.0
    with pytest.raises(ValueError, match="cd0"):
        validate(cfg)


def test_cd0_negative_raises() -> None:
    cfg = valid()
    cfg["aircraft"]["cd0"] = -0.01
    with pytest.raises(ValueError, match="cd0"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Range / invariant violations — airframe
# ---------------------------------------------------------------------------


def test_g_max_nd_zero_raises() -> None:
    cfg = valid()
    cfg["airframe"]["g_max_nd"] = 0.0
    with pytest.raises(ValueError, match="g_max_nd"):
        validate(cfg)


def test_g_max_nd_negative_raises() -> None:
    cfg = valid()
    cfg["airframe"]["g_max_nd"] = -1.0
    with pytest.raises(ValueError, match="g_max_nd"):
        validate(cfg)


def test_g_min_nd_zero_raises() -> None:
    cfg = valid()
    cfg["airframe"]["g_min_nd"] = 0.0
    with pytest.raises(ValueError, match="g_min_nd"):
        validate(cfg)


def test_g_min_nd_positive_raises() -> None:
    cfg = valid()
    cfg["airframe"]["g_min_nd"] = 1.0
    with pytest.raises(ValueError, match="g_min_nd"):
        validate(cfg)


def test_tas_max_mps_zero_raises() -> None:
    cfg = valid()
    cfg["airframe"]["tas_max_mps"] = 0.0
    with pytest.raises(ValueError, match="tas_max_mps"):
        validate(cfg)


def test_mach_max_nd_zero_raises() -> None:
    cfg = valid()
    cfg["airframe"]["mach_max_nd"] = 0.0
    with pytest.raises(ValueError, match="mach_max_nd"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Range / invariant violations — inertia
# ---------------------------------------------------------------------------


def test_mass_kg_zero_raises() -> None:
    cfg = valid()
    cfg["inertia"]["mass_kg"] = 0.0
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


def test_mass_kg_negative_raises() -> None:
    cfg = valid()
    cfg["inertia"]["mass_kg"] = -1.0
    with pytest.raises(ValueError, match="mass_kg"):
        validate(cfg)


def test_Ixx_kgm2_zero_raises() -> None:
    cfg = valid()
    cfg["inertia"]["Ixx_kgm2"] = 0.0
    with pytest.raises(ValueError, match="Ixx_kgm2"):
        validate(cfg)


def test_Iyy_kgm2_zero_raises() -> None:
    cfg = valid()
    cfg["inertia"]["Iyy_kgm2"] = 0.0
    with pytest.raises(ValueError, match="Iyy_kgm2"):
        validate(cfg)


def test_Izz_kgm2_zero_raises() -> None:
    cfg = valid()
    cfg["inertia"]["Izz_kgm2"] = 0.0
    with pytest.raises(ValueError, match="Izz_kgm2"):
        validate(cfg)


# ---------------------------------------------------------------------------
# Range / invariant violations — lift_curve
# ---------------------------------------------------------------------------


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
