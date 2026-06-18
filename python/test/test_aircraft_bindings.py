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
# Shared fixture config — general-aviation aircraft with an explicit zero-thrust
# ("none") propulsion model. The "propulsion" section is required; "none"
# selects the zero-thrust stub for an unpowered airframe.
# ---------------------------------------------------------------------------

_GA_CONFIG = {
    "schema_version": 1,
    "propulsion": {"type": "none"},
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
    "load_factor_allocator": {
        "alpha_dot_max_rad_s": 0.0,
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
        "alpha_max_rad": 0.42,
        "alpha_min_rad": -0.26,
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


def test_aircraft_missing_propulsion_section_raises():
    # The "propulsion" section is required. Omitting it used to silently select a
    # zero-thrust stub, so a config that forgot its engine produced an aircraft that
    # could never accelerate or take off, with no error. Omission is now an error;
    # an unpowered airframe must opt in explicitly via "type": "none".
    cfg = {k: v for k, v in _GA_CONFIG.items() if k != "propulsion"}
    with pytest.raises(Exception, match="propulsion"):
        liteaero_sim_py.Aircraft(json.dumps(cfg))


def test_aircraft_explicit_none_propulsion_zero_thrust():
    # "type": "none" is the explicit opt-in to a zero-thrust airframe: it must
    # construct without error and never produce thrust regardless of throttle.
    cfg = dict(_GA_CONFIG)
    cfg["propulsion"] = {"type": "none"}
    ac = liteaero_sim_py.Aircraft(json.dumps(cfg))
    cmd = liteaero_sim_py.AircraftCommand()
    cmd.throttle_nd = 1.0
    v0 = ac.state().velocity_north_mps
    for _ in range(50):  # 1 s of full throttle
        ac.step(cmd, dt_s=0.02, rho_kgm3=1.225)
    # With no engine, full throttle adds no thrust: airspeed only decays (drag).
    assert ac.state().velocity_north_mps <= v0 + 1e-3


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
    speed_before = ac.state().airspeed_m_s
    for _ in range(50):
        ac.step(cmd)
    # State must advance: at n_z=1 the load-factor model holds altitude (lift tracks
    # the commanded load factor), but with zero thrust drag bleeds airspeed.
    assert ac.state().airspeed_m_s < speed_before - 0.1


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------


def test_aircraft_reset_restores_initial_state():
    ac = liteaero_sim_py.Aircraft(_GA_JSON)
    cmd = liteaero_sim_py.AircraftCommand(n_z=1.0, throttle_nd=0.0)
    speed_initial = ac.state().airspeed_m_s

    for _ in range(50):
        ac.step(cmd)
    assert ac.state().airspeed_m_s < speed_initial - 0.1  # state advanced (drag bled speed)

    ac.reset()
    assert ac.state().airspeed_m_s == pytest.approx(speed_initial, abs=0.5)


# ---------------------------------------------------------------------------
# Powered touch-and-go / go-around — end-to-end through the config→propulsion path
# ---------------------------------------------------------------------------
#
# Regression guard for the gap that let a no-engine config fail to take off
# silently: the only test that exercised liftoff
# (BodyColliderOnly_TouchAndGo_BecomesAirborne) was removed, and every C++
# Aircraft test injects a thrust stub rather than building propulsion from the
# config. This test drives the real shipped config (small_uas_ksba.json, which
# carries a prop engine) through make_propulsion(), lands it, then commands a
# powered go-around and asserts the aircraft actually climbs away.


def _repo_config(name):
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "..", "configs", name)


def test_powered_go_around_climbs():
    cfg_path = _repo_config("small_uas_ksba.json")
    if not os.path.isfile(cfg_path):
        pytest.skip(f"config not found: {cfg_path}")
    with open(cfg_path) as f:
        cfg = json.load(f)
    assert "propulsion" in cfg and cfg["propulsion"].get("type") != "none", (
        "small_uas_ksba.json must carry a real engine for a go-around to be possible"
    )

    dt = 0.01
    # Start just above the runway on a shallow approach so it touches down quickly.
    cfg["initial_state"].update(
        {"altitude_m": 5.0, "velocity_north_mps": 16.0,
         "velocity_east_mps": 0.0, "velocity_down_mps": 1.5}
    )
    ac = liteaero_sim_py.Aircraft(json.dumps(cfg), dt_s=dt)
    ac.set_terrain(liteaero_sim_py.FlatTerrain(0.0))

    cmd = liteaero_sim_py.AircraftCommand()
    cmd.n_z = 1.0
    cmd.throttle_nd = 0.0

    first_wow_t = None
    go = False
    settled_agl = None
    max_agl_after_go = -1.0e9

    for _ in range(int(45.0 / dt)):
        s = ac.state()
        wow = ac.contact_forces().weight_on_wheels
        t = s.time_s
        if wow and first_wow_t is None:
            first_wow_t = t
        if first_wow_t is not None and (t - first_wow_t) >= 3.0 and not go:
            go = True
            settled_agl = s.altitude_m
        if go:
            cmd.throttle_nd = 1.0
            cmd.n_z = 2.0
            max_agl_after_go = max(max_agl_after_go, s.altitude_m)
        ac.step(cmd, dt_s=dt, rho_kgm3=1.225)

    assert first_wow_t is not None, "aircraft never touched down"
    assert go, "go-around phase never triggered (never settled on the gear)"
    final = ac.state()
    # The powered go-around must produce a real climb: well above the settled
    # ground height, climbing (negative NED-down velocity) at the end.
    assert max_agl_after_go > settled_agl + 1.5, (
        f"aircraft failed to climb on go-around: settled={settled_agl:.2f} m, "
        f"peak after go-around={max_agl_after_go:.2f} m"
    )
    assert final.velocity_down_mps < 0.0, (
        f"aircraft not climbing at end of go-around (vD={final.velocity_down_mps:.2f} m/s)"
    )
