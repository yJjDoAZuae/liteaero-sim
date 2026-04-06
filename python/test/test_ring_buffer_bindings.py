"""Tests for liteaero_sim_py ring buffer and channel registry bindings.

Design authority: docs/architecture/ring_buffer.md

The module is an optional build target (LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON).
These tests are skipped automatically when the module is not built.
"""
import json
import time

import pytest

liteaero_sim_py = pytest.importorskip(
    "liteaero_sim_py",
    reason="liteaero_sim_py not built (set LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON)",
)

_GA_JSON = json.dumps(
    {
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
)


def _make_runner_batch(duration_s=0.5, dt_s=0.02):
    cfg = liteaero_sim_py.RunnerConfig(dt_s=dt_s, duration_s=duration_s, mode="batch")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=dt_s)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    return runner


# ---------------------------------------------------------------------------
# channel_registry() accessor
# ---------------------------------------------------------------------------


def test_runner_has_channel_registry():
    runner = _make_runner_batch()
    registry = runner.channel_registry()
    assert registry is not None


def test_kinematic_channels_registered_before_start():
    runner = _make_runner_batch()
    channels = runner.channel_registry().available_channels()
    expected = {
        "kinematic/time_s",
        "kinematic/latitude_rad",
        "kinematic/longitude_rad",
        "kinematic/altitude_m",
        "kinematic/velocity_north_mps",
        "kinematic/velocity_east_mps",
        "kinematic/velocity_down_mps",
        "kinematic/heading_rad",
        "kinematic/pitch_rad",
        "kinematic/roll_rad",
        "kinematic/alpha_rad",
        "kinematic/beta_rad",
        "kinematic/airspeed_m_s",
        "kinematic/roll_rate_rad_s",
    }
    assert expected.issubset(set(channels))


# ---------------------------------------------------------------------------
# Subscribe
# ---------------------------------------------------------------------------


def test_subscribe_to_registered_channel_succeeds():
    runner = _make_runner_batch()
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    assert sub is not None


def test_subscribe_to_unregistered_channel_raises():
    runner = _make_runner_batch()
    with pytest.raises(Exception):
        runner.channel_registry().subscribe("nonexistent/channel")


def test_subscriber_channel_name():
    runner = _make_runner_batch()
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    assert sub.channel_name == "kinematic/altitude_m"


# ---------------------------------------------------------------------------
# Drain — no backfill (PP-F37)
# ---------------------------------------------------------------------------


def test_drain_before_run_is_empty():
    runner = _make_runner_batch()
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    assert sub.drain() == []


def test_late_join_after_run_starts_empty():
    """Subscribe after run completes — buffer must be empty (PP-F37: no backfill)."""
    runner = _make_runner_batch(duration_s=0.1)
    runner.start()  # batch — completes before we subscribe
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    assert sub.drain() == []


# ---------------------------------------------------------------------------
# Drain after batch run
# ---------------------------------------------------------------------------


def test_drain_after_batch_run_returns_samples():
    runner = _make_runner_batch(duration_s=0.5, dt_s=0.02)
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    runner.start()

    samples = sub.drain()
    assert len(samples) > 0


def test_drain_samples_are_tuples_of_two_floats():
    runner = _make_runner_batch(duration_s=0.1, dt_s=0.02)
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    runner.start()

    samples = sub.drain()
    assert len(samples) > 0
    for s in samples:
        time_s, value = s
        assert isinstance(time_s, float)
        assert isinstance(value, float)


def test_drain_altitude_samples_are_nonzero():
    runner = _make_runner_batch(duration_s=0.2, dt_s=0.02)
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    runner.start()

    samples = sub.drain()
    assert len(samples) > 0
    for _, alt in samples:
        # Initial altitude 300 m; zero thrust so descending — all > 0
        assert alt > 0.0


def test_drain_resets_buffer():
    runner = _make_runner_batch(duration_s=0.2, dt_s=0.02)
    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    runner.start()

    first = sub.drain()
    assert len(first) > 0
    second = sub.drain()
    assert second == []


# ---------------------------------------------------------------------------
# Multiple subscribers (PP-F36)
# ---------------------------------------------------------------------------


def test_two_subscribers_receive_same_data():
    runner = _make_runner_batch(duration_s=0.2, dt_s=0.02)
    reg = runner.channel_registry()
    sub1 = reg.subscribe("kinematic/altitude_m")
    sub2 = reg.subscribe("kinematic/altitude_m")
    runner.start()

    s1 = sub1.drain()
    s2 = sub2.drain()
    assert len(s1) == len(s2)
    assert len(s1) > 0
    for (t1, v1), (t2, v2) in zip(s1, s2):
        assert t1 == pytest.approx(t2)
        assert v1 == pytest.approx(v2)


# ---------------------------------------------------------------------------
# Realtime mode — live drain
# ---------------------------------------------------------------------------


def test_realtime_subscriber_receives_live_data():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=10.0, mode="realtime")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)

    sub = runner.channel_registry().subscribe("kinematic/altitude_m")
    runner.start()

    time.sleep(0.15)  # let 5–10 steps accumulate
    samples = sub.drain()
    runner.stop()

    assert len(samples) > 0
