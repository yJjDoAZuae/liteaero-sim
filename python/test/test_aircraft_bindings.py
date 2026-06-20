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


def test_touch_and_go_rotates_to_climb_fpa():
    # Scenario_TouchAndGo_RotatesToClimbFPA (landing_gear.md §Test Strategy): a powered
    # go-around must ROTATE the aircraft to a commanded climb gradient, not merely creep
    # upward. The command is a closed loop that pulls for a target flight-path angle,
    # n_z = clamp(1 + k(gamma_target - gamma)), so the test fails if the gear/relaxation
    # caps the realized load factor (OQ-LG-22: the latched hard-contact flag pinned the
    # relaxation to full weight, so a 2 g command realized ~1 g and the FPA stuck at +0.3°).
    # Driven through the real config->propulsion path (small_uas_ksba.json has a prop engine).
    import math

    cfg_path = _repo_config("small_uas_ksba.json")
    if not os.path.isfile(cfg_path):
        pytest.skip(f"config not found: {cfg_path}")
    with open(cfg_path) as f:
        cfg = json.load(f)
    assert "propulsion" in cfg and cfg["propulsion"].get("type") != "none", (
        "small_uas_ksba.json must carry a real engine for a go-around to be possible"
    )

    dt = 0.01
    g = 9.80665
    gamma_target = math.radians(6.0)   # commanded climb flight-path angle
    k_fpa = 6.0                        # FPA-error -> load-factor gain
    nz_max = 2.5

    cfg["initial_state"].update(
        {"altitude_m": 20.0, "velocity_north_mps": 15.0 * math.cos(math.radians(6.0)),
         "velocity_east_mps": 0.0, "velocity_down_mps": 15.0 * math.sin(math.radians(6.0))}
    )
    ac = liteaero_sim_py.Aircraft(json.dumps(cfg), dt_s=dt)
    ac.set_terrain(liteaero_sim_py.FlatTerrain(0.0))

    cmd = liteaero_sim_py.AircraftCommand()
    cmd.n_z = 1.0
    cmd.throttle_nd = 0.0

    first_wow_t = None
    go = False
    settled_agl = None
    wow_cleared_after_go = False
    nz_peak = 0.0
    prev_gamma = 0.0
    max_abs_pitch_deg = 0.0
    max_abs_pitch_post_td_deg = 0.0  # tracked from first touchdown onward

    n_steps = int(45.0 / dt)
    for i in range(n_steps):
        s = ac.state()
        wow = ac.contact_forces().weight_on_wheels
        t = s.time_s
        vN, vD = s.velocity_north_mps, s.velocity_down_mps
        V = math.hypot(vN, vD)
        gamma = math.atan2(-vD, max(abs(vN), 1e-3))
        pitch_deg = abs(math.degrees(s.pitch_rad))
        max_abs_pitch_deg = max(max_abs_pitch_deg, pitch_deg)

        if wow and first_wow_t is None:
            first_wow_t = t
        if first_wow_t is not None:
            max_abs_pitch_post_td_deg = max(max_abs_pitch_post_td_deg, pitch_deg)
        if first_wow_t is not None and (t - first_wow_t) >= 3.0 and not go:
            go = True
            settled_agl = s.altitude_m

        if go:
            cmd.throttle_nd = 1.0
            cmd.n_z = max(0.0, min(nz_max, 1.0 + k_fpa * (gamma_target - gamma)))
            if not wow:
                wow_cleared_after_go = True
            # Realized normal load factor from path curvature: n = cos(gamma) + V*gammadot/g.
            gammadot = (gamma - prev_gamma) / dt
            nz_peak = max(nz_peak, math.cos(gamma) + V * gammadot / g)
        prev_gamma = gamma
        ac.step(cmd, dt_s=dt, rho_kgm3=1.225)

    assert first_wow_t is not None, "aircraft never touched down"
    assert go, "go-around phase never triggered (never settled on the gear)"

    final = ac.state()
    final_gamma_deg = math.degrees(
        math.atan2(-final.velocity_down_mps, max(abs(final.velocity_north_mps), 1e-3))
    )

    # 0. The trajectory must stay SANE throughout — no attitude blow-up. A 15 m/s approach,
    #    touchdown, and go-around should never pitch past a steep-but-plausible bound; a
    #    velocity-derived-attitude whip drives it to 80-150° (tumbling). This is the check the
    #    old endpoint-only assertions missed (they passed on a looping trajectory).
    assert max_abs_pitch_post_td_deg < 35.0, (
        f"attitude blew up after touchdown: max |pitch| = {max_abs_pitch_post_td_deg:.1f}° "
        "(expected a bounded rotation, not a tumble)"
    )
    # 1. Genuine lift-off: weight-on-wheels released after the go-around.
    assert wow_cleared_after_go, "weight-on-wheels never cleared after go-around (gear latch)"
    # 2. The aircraft actually rotated: realized load factor followed the >1 g command,
    #    not capped near 1 g by the relaxation. (Suppressed case peaked at ~1.09 g.)
    assert nz_peak > 1.3, f"realized load factor never exceeded 1.3 g (peak {nz_peak:.2f}); command suppressed"
    # 3. It reaches and holds a real climb gradient near the target (suppressed case held +0.3°).
    assert final_gamma_deg > 3.0, (
        f"aircraft did not rotate to a climb (final FPA {final_gamma_deg:.1f}°, target 6°)"
    )
    assert final.velocity_down_mps < 0.0, "aircraft not climbing at end of go-around"


def test_full_stop_landing_settles_on_gear():
    # Scenario_FullStopLanding_SettlesOnGear (landing_gear.md §Test Strategy): a normal landing
    # must SETTLE ONTO THE GEAR, not float on the wing. The apportionment relaxation alone is
    # degenerate and rests in a floating equilibrium (wing ~0.8 W, gear ~0.18 W) at roll-out speed;
    # the additive axial-acceleration settle term (§2b-ii, OQ-LG-23) must put ~full weight on the
    # gear throughout the roll-out. Driven through the real config (small_uas_ksba.json, which has
    # both gear and a body collider).
    import math

    cfg_path = _repo_config("small_uas_ksba.json")
    if not os.path.isfile(cfg_path):
        pytest.skip(f"config not found: {cfg_path}")
    with open(cfg_path) as f:
        cfg = json.load(f)
    weight = cfg["inertia"]["mass_kg"] * 9.80665

    dt = 0.01
    cfg["initial_state"].update(
        {"altitude_m": 20.0, "velocity_north_mps": 15.0 * math.cos(math.radians(6.0)),
         "velocity_east_mps": 0.0, "velocity_down_mps": 15.0 * math.sin(math.radians(6.0))}
    )
    ac = liteaero_sim_py.Aircraft(json.dumps(cfg), dt_s=dt)
    ac.set_terrain(liteaero_sim_py.FlatTerrain(0.0))

    cmd = liteaero_sim_py.AircraftCommand()
    cmd.n_z = 1.0
    cmd.throttle_nd = 0.0

    first_wow_t = None
    min_gear_frac_at_speed = 1.0e9  # min gear load (as fraction of weight) during powered roll-out

    for _ in range(int(40.0 / dt)):
        s = ac.state()
        cf = ac.contact_forces()
        wow = cf.weight_on_wheels
        t = s.time_s
        speed = math.hypot(s.velocity_north_mps, s.velocity_down_mps)

        if wow and first_wow_t is None:
            first_wow_t = t
        # Sample the STEADY roll-out: after the touchdown impact/bounce transient has settled
        # (≥3 s after first contact) and still rolling fast enough that the wing could float the
        # aircraft (well above this UAS's ~6.6 m/s stall). The float defect leaves the gear near
        # zero here; a correct settle keeps it carrying ~full weight. (The brief touchdown bounce
        # is a separate touchdown-dynamics matter, excluded here.)
        if first_wow_t is not None and (t - first_wow_t) >= 3.0 and wow and speed > 7.0:
            min_gear_frac_at_speed = min(min_gear_frac_at_speed, -cf.force_body_n[2] / weight)
        ac.step(cmd, dt_s=dt, rho_kgm3=1.225)

    assert first_wow_t is not None, "aircraft never touched down"
    assert min_gear_frac_at_speed < 1.0e8, "no roll-out samples above 7 m/s after settling"
    # The gear must carry essentially the weight throughout the roll-out — not be unloaded while
    # the wing floats the aircraft. (Floating defect: ~0.18 W.)
    assert min_gear_frac_at_speed > 0.7, (
        f"aircraft floats on the wing during roll-out: min gear load "
        f"{min_gear_frac_at_speed:.2f} W (expected ~1 W on the gear)"
    )
