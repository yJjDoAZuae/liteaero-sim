"""Tests for SimSession — live simulation session with UDP broadcast.

Design authority: docs/architecture/live_sim_view.md §Python SimSession Design

Six tests per design spec:
  test_sim_session_initializes    — SimSession constructs without error; is_running() is False
  test_sim_session_start_stop     — start() + stop() completes without deadlock
  test_sim_session_broadcasts     — after a brief real-time run, loopback UDP socket
                                    on the broadcast port has received >= 1 datagram
  test_sim_session_datagram_size  — received datagram size > 0 (protobuf serialized)
  test_scripted_input_wiring      — set_scripted_input() + push(AircraftCommand) does not raise
  test_cli_argument_parsing       — argparse namespace contains expected attributes
"""
from __future__ import annotations

import json
import socket
import time

import pytest

liteaero_sim_py = pytest.importorskip(
    "liteaero_sim_py",
    reason="liteaero_sim_py not built (set LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON)",
)

# Import tools modules via path manipulation — tools/ is a sibling of test/.
import sys
import os

_tools_dir = os.path.join(os.path.dirname(__file__), "..", "tools")
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)

from live_sim_session import SimSession  # noqa: E402

# ---------------------------------------------------------------------------
# Minimal GA aircraft config
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

# Use high-numbered ports unlikely to conflict with system services.
_TEST_PORT      = 19562
_BROADCAST_PORT = 19563


def _open_recv_socket(port: int, timeout_s: float = 0.5) -> socket.socket:
    """Open a UDP socket bound to the loopback on *port*."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(timeout_s)
    return sock


# ---------------------------------------------------------------------------

def test_sim_session_initializes():
    session = SimSession(_GA_JSON, mode="realtime", dt_s=0.02, broadcast_port=_TEST_PORT)
    assert not session.is_running()


def test_sim_session_start_stop():
    session = SimSession(_GA_JSON, mode="realtime", dt_s=0.02, broadcast_port=_TEST_PORT)
    session.start()
    time.sleep(0.05)
    session.stop()
    assert not session.is_running()


def test_sim_session_broadcasts():
    recv_sock = _open_recv_socket(_BROADCAST_PORT)
    try:
        session = SimSession(
            _GA_JSON, mode="realtime", dt_s=0.02, broadcast_port=_BROADCAST_PORT
        )
        session.start()
        time.sleep(0.15)
        session.stop()

        datagram_count = 0
        while True:
            try:
                data = recv_sock.recv(4096)
                if data:
                    datagram_count += 1
            except socket.timeout:
                break
    finally:
        recv_sock.close()

    assert datagram_count >= 1, (
        f"Expected >= 1 datagram on port {_BROADCAST_PORT} after 150 ms; got {datagram_count}"
    )


def test_sim_session_datagram_size():
    recv_sock = _open_recv_socket(_BROADCAST_PORT)
    try:
        session = SimSession(
            _GA_JSON, mode="realtime", dt_s=0.02, broadcast_port=_BROADCAST_PORT
        )
        session.start()
        time.sleep(0.10)
        session.stop()

        # Read the first datagram.
        try:
            data = recv_sock.recv(4096)
        except socket.timeout:
            data = b""
    finally:
        recv_sock.close()

    assert len(data) > 0, "No datagram received — cannot check size"


def test_scripted_input_wiring():
    session = SimSession(_GA_JSON, mode="realtime", dt_s=0.02, broadcast_port=_TEST_PORT)
    scripted = liteaero_sim_py.ScriptedInput()
    session.set_scripted_input(scripted)
    # push() must not raise
    scripted.push(liteaero_sim_py.AircraftCommand(n_z=1.0, n_y=0.0,
                                                   roll_rate_wind_rps=0.0,
                                                   throttle_nd=0.5))


def test_cli_argument_parsing():
    """live_sim.py argparse produces the expected namespace attributes."""
    import importlib.util
    import argparse

    spec = importlib.util.spec_from_file_location(
        "live_sim_cli",
        os.path.join(_tools_dir, "live_sim.py"),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    parser: argparse.ArgumentParser = module.build_arg_parser()
    ns = parser.parse_args(["--config", "aircraft.json", "--dt", "0.01", "--port", "15000"])
    assert ns.config == "aircraft.json"
    assert ns.dt == pytest.approx(0.01)
    assert ns.port == 15000
    assert ns.scripted is True or ns.joystick is None
