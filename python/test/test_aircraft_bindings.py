"""Tests for liteaero_sim_py Aircraft and KinematicState bindings.

Design authority: docs/architecture/python_bindings.md

The module is an optional build target (LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON).
These tests are skipped automatically when the module is not built.
"""
import json
import math
import os
import tempfile

import pytest

liteaero_sim_py = pytest.importorskip(
    "liteaero_sim_py",
    reason="liteaero_sim_py not built (set LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON)",
)

# ---------------------------------------------------------------------------
# Shared fixture config — general-aviation aircraft, no propulsion section
# (zero-thrust stub is used when "propulsion" is absent).
# ---------------------------------------------------------------------------

_GA_CONFIG = {
    "schema_version": 1,
    "aircraft": {
        "S_ref_m2": 16.2,
        "cl_y_beta": -0.60,
        "ar": 7.47,
        "e": 0.80,
        "cd0": 0.027,
        "cmd_filter_substeps": 1,
        "nz_wn_rad_s": 10.0,
        "nz_zeta_nd": 0.7,
        "ny_wn_rad_s": 10.0,
        "ny_zeta_nd": 0.7,
        "roll_rate_wn_rad_s": 20.0,
        "roll_rate_zeta_nd": 0.7,
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

_GA_JSON = json.dumps(_GA_CONFIG)


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


def test_aircraft_construct_from_json_string():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    assert ac is not None


def test_aircraft_construct_from_file_path():
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False
    ) as f:
        json.dump(_GA_CONFIG, f)
        path = f.name
    try:
        ac = liteaero_sim_py.Aircraft(path)
        assert ac is not None
    finally:
        os.unlink(path)


def test_aircraft_construct_with_explicit_dt_s():
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.05)
    assert ac is not None


def test_aircraft_invalid_config_raises():
    with pytest.raises(Exception):
        liteaero_sim_py.Aircraft("{}")


# ---------------------------------------------------------------------------
# KinematicState attributes after initialization
# ---------------------------------------------------------------------------


def test_aircraft_initial_altitude():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    s = ac.state()
    assert s.altitude_m == pytest.approx(300.0, abs=1.0)


def test_aircraft_initial_airspeed():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    s = ac.state()
    # v_north = 55 m/s, no wind → airspeed ≈ 55 m/s
    assert s.airspeed_m_s == pytest.approx(55.0, abs=1.0)


def test_aircraft_initial_position_geodetic():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    s = ac.state()
    assert s.latitude_rad == pytest.approx(0.0, abs=1e-6)
    assert s.longitude_rad == pytest.approx(0.0, abs=1e-6)


def test_aircraft_kinematic_state_attributes_exist():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    s = ac.state()
    # All specified attributes must be accessible and return finite floats.
    for attr in (
        "time_s",
        "latitude_rad",
        "longitude_rad",
        "altitude_m",
        "velocity_north_mps",
        "velocity_east_mps",
        "velocity_down_mps",
        "heading_rad",
        "pitch_rad",
        "roll_rad",
        "alpha_rad",
        "beta_rad",
        "airspeed_m_s",
        "roll_rate_rad_s",
    ):
        val = getattr(s, attr)
        assert math.isfinite(val), f"KinematicState.{attr} is not finite: {val}"


# ---------------------------------------------------------------------------
# step()
# ---------------------------------------------------------------------------


def test_aircraft_step_does_not_raise():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    cmd = liteaero_sim_py.AircraftCommand(n_z=1.0, throttle_nd=0.5)
    ac.step(cmd)


def test_aircraft_step_with_explicit_dt_and_rho():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    cmd = liteaero_sim_py.AircraftCommand(n_z=1.0, throttle_nd=0.0)
    ac.step(cmd, dt_s=0.02, rho_kgm3=1.225)


def test_aircraft_step_advances_state():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    cmd = liteaero_sim_py.AircraftCommand(n_z=1.0, throttle_nd=0.0)
    alt_before = ac.state().altitude_m
    for _ in range(50):
        ac.step(cmd)
    # Altitude must have changed (zero thrust → descending)
    assert ac.state().altitude_m != pytest.approx(alt_before, abs=0.01)


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------


def test_aircraft_reset_restores_initial_state():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    cmd = liteaero_sim_py.AircraftCommand(n_z=1.0, throttle_nd=0.0)
    alt_initial = ac.state().altitude_m

    for _ in range(50):
        ac.step(cmd)
    assert ac.state().altitude_m != pytest.approx(alt_initial, abs=0.01)

    ac.reset()
    assert ac.state().altitude_m == pytest.approx(alt_initial, abs=0.5)
