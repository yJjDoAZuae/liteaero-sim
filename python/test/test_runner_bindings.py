"""Tests for liteaero_sim_py RunnerConfig and SimRunner bindings.

Design authority: docs/architecture/python_bindings.md

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

# ---------------------------------------------------------------------------
# Minimal GA aircraft config for runner tests
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# RunnerConfig
# ---------------------------------------------------------------------------


def test_runner_config_defaults():
    cfg = liteaero_sim_py.RunnerConfig()
    assert cfg.dt_s == pytest.approx(0.02)
    assert cfg.duration_s == pytest.approx(0.0)
    assert cfg.time_scale == pytest.approx(1.0)


def test_runner_config_kwargs():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.01, duration_s=5.0, time_scale=2.0, mode="batch")
    assert cfg.dt_s == pytest.approx(0.01)
    assert cfg.duration_s == pytest.approx(5.0)
    assert cfg.time_scale == pytest.approx(2.0)


def test_runner_config_mode_batch():
    cfg = liteaero_sim_py.RunnerConfig(mode="batch")
    assert cfg is not None  # mode accepted without error


def test_runner_config_mode_realtime():
    cfg = liteaero_sim_py.RunnerConfig(mode="realtime")
    assert cfg is not None


def test_runner_config_mode_scaled_realtime():
    cfg = liteaero_sim_py.RunnerConfig(mode="scaled_realtime")
    assert cfg is not None


def test_runner_config_invalid_mode_raises():
    with pytest.raises(Exception):
        liteaero_sim_py.RunnerConfig(mode="bogus")


# ---------------------------------------------------------------------------
# SimRunner — not running before start
# ---------------------------------------------------------------------------


def test_runner_not_running_before_start():
    runner = liteaero_sim_py.SimRunner()
    assert not runner.is_running()


def test_runner_elapsed_time_zero_before_start():
    runner = liteaero_sim_py.SimRunner()
    assert runner.elapsed_sim_time_s() == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# SimRunner — batch mode
# ---------------------------------------------------------------------------


def test_runner_batch_run_completes():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=0.1, mode="batch")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    runner.start()  # blocks until duration_s elapsed
    assert not runner.is_running()


def test_runner_batch_elapsed_time():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=0.5, mode="batch")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    runner.start()
    assert runner.elapsed_sim_time_s() >= 0.45


def test_runner_batch_advances_aircraft_state():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=1.0, mode="batch")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    alt_before = ac.state().altitude_m
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    runner.start()
    # Zero thrust → aircraft descends; altitude must differ from initial
    assert ac.state().altitude_m != pytest.approx(alt_before, abs=0.1)


# ---------------------------------------------------------------------------
# SimRunner — realtime mode start/stop
# ---------------------------------------------------------------------------


def test_runner_realtime_starts_and_is_running():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=30.0, mode="realtime")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    runner.start()
    time.sleep(0.05)
    is_now_running = runner.is_running()
    runner.stop()
    assert is_now_running


def test_runner_realtime_stop_terminates():
    cfg = liteaero_sim_py.RunnerConfig(dt_s=0.02, duration_s=30.0, mode="realtime")
    ac = liteaero_sim_py.Aircraft(_GA_JSON, dt_s=0.02)
    runner = liteaero_sim_py.SimRunner()
    runner.initialize(cfg, ac)
    runner.start()
    time.sleep(0.05)
    runner.stop()
    assert not runner.is_running()
