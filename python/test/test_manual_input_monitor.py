"""Tests for manual_input_monitor.py — shared library components.

Design authority: docs/architecture/manual_input.md §Manual Input Monitor
"""
from __future__ import annotations

import math
import os
import queue
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

import matplotlib
import matplotlib.figure as mpl_figure
import pytest

from manual_input_monitor import (
    InputMonitorConfig,
    InputMonitorFigure,
    build_args,
    draw_gauge,
    enumerate_devices,
    stdout_reader,
)

# ---------------------------------------------------------------------------
# InputMonitorConfig
# ---------------------------------------------------------------------------


class TestInputMonitorConfig:
    def test_default_idle_throttle(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.idle_throttle_nd == pytest.approx(0.05)

    def test_default_nz_range(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.min_nz_g == pytest.approx(-2.0)
        assert cfg.max_nz_g == pytest.approx(4.0)

    def test_default_max_ny(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.max_ny_g == pytest.approx(2.0)

    def test_default_roll_rate(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.max_roll_rate_rad_s == pytest.approx(math.pi / 2.0)

    def test_default_rate_hz(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.rate_hz == pytest.approx(50.0)

    def test_default_device_index_zero(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.device_index == 0

    def test_default_device_name_empty(self) -> None:
        cfg = InputMonitorConfig()
        assert cfg.device_name == ""


# ---------------------------------------------------------------------------
# build_args
# ---------------------------------------------------------------------------


class TestBuildArgs:
    def _config(self, **kwargs) -> InputMonitorConfig:
        exe = Path("fake/joystick_verify.exe")
        return InputMonitorConfig(joystick_verify=exe, **kwargs)

    def test_device_index_used_when_name_empty(self) -> None:
        cfg = self._config(device_index=2, device_name="")
        args = build_args(cfg)
        assert "--device-index" in args
        assert "2" in args
        assert "--device-name" not in args

    def test_device_name_used_when_set(self) -> None:
        cfg = self._config(device_name="Logitech")
        args = build_args(cfg)
        assert "--device-name" in args
        assert "Logitech" in args
        assert "--device-index" not in args

    def test_rate_hz_included(self) -> None:
        cfg = self._config(rate_hz=100.0)
        args = build_args(cfg)
        assert "--rate-hz" in args
        assert "100.0" in args

    def test_config_file_included_when_set(self) -> None:
        cfg = self._config(config_file="my_config.json")
        args = build_args(cfg)
        assert "--config" in args
        assert "my_config.json" in args

    def test_config_file_omitted_when_empty(self) -> None:
        cfg = self._config(config_file="")
        args = build_args(cfg)
        assert "--config" not in args

    def test_exe_path_is_first_arg(self) -> None:
        cfg = self._config()
        args = build_args(cfg)
        assert "joystick_verify.exe" in args[0]


# ---------------------------------------------------------------------------
# stdout_reader
# ---------------------------------------------------------------------------


class TestStdoutReader:
    def _run(self, lines: list[str]) -> tuple[threading.Event, queue.Queue]:
        mock_proc = MagicMock()
        mock_proc.stdout = iter(lines)
        q: queue.Queue = queue.Queue()
        ready = threading.Event()
        t = threading.Thread(target=stdout_reader, args=(mock_proc, q, ready), daemon=True)
        t.start()
        t.join(timeout=2.0)
        return ready, q

    def test_sets_ready_on_sentinel(self) -> None:
        ready, _ = self._run(["DEVICE 0 6 Logitech\n", "READY\n"])
        assert ready.is_set()

    def test_enqueues_json_lines_after_ready(self) -> None:
        frame = '{"n_z": 1.0, "n_y": 0.0, "roll_rate_wind_rps": 0.0, "throttle_nd": 0.05}\n'
        _, q = self._run(["READY\n", frame])
        assert not q.empty()
        assert '"n_z"' in q.get_nowait()

    def test_device_lines_not_enqueued(self) -> None:
        _, q = self._run(["DEVICE 0 6 Logitech\n", "READY\n"])
        assert q.empty()

    def test_ready_not_set_without_sentinel(self) -> None:
        ready, _ = self._run(["DEVICE 0 6 Logitech\n"])
        assert not ready.is_set()


# ---------------------------------------------------------------------------
# enumerate_devices
# ---------------------------------------------------------------------------


class TestEnumerateDevices:
    def test_parses_device_lines(self, tmp_path: Path) -> None:
        fake_exe = tmp_path / "joystick_verify.exe"
        fake_exe.write_text("")

        output = (
            "DEVICE 0 6 Logitech Extreme 3D Pro\n"
            "DEVICE 1 4 Xbox Controller\n"
            "READY\n"
        )
        mock_proc = MagicMock()
        mock_proc.stdout = iter(output.splitlines(keepends=True))

        with patch("subprocess.Popen", return_value=mock_proc):
            cfg = InputMonitorConfig(joystick_verify=fake_exe)
            devices = enumerate_devices(cfg)

        assert len(devices) == 2
        assert devices[0] == {"index": 0, "axes": 6, "name": "Logitech Extreme 3D Pro"}
        assert devices[1] == {"index": 1, "axes": 4, "name": "Xbox Controller"}

    def test_empty_when_no_devices(self, tmp_path: Path) -> None:
        fake_exe = tmp_path / "joystick_verify.exe"
        fake_exe.write_text("")

        mock_proc = MagicMock()
        mock_proc.stdout = iter(["READY\n"])

        with patch("subprocess.Popen", return_value=mock_proc):
            cfg = InputMonitorConfig(joystick_verify=fake_exe)
            devices = enumerate_devices(cfg)

        assert devices == []

    def test_raises_when_exe_missing(self) -> None:
        cfg = InputMonitorConfig(joystick_verify=Path("nonexistent/joystick_verify.exe"))
        with pytest.raises(FileNotFoundError):
            enumerate_devices(cfg)


# ---------------------------------------------------------------------------
# draw_gauge
# ---------------------------------------------------------------------------


class TestDrawGauge:
    def _ax(self):
        fig = mpl_figure.Figure()
        return fig.add_subplot(1, 1, 1)

    def test_does_not_raise_at_neutral(self) -> None:
        ax = self._ax()
        draw_gauge(ax, 1.0, -2.0, 4.0, 1.0, "n_z", "g")

    def test_does_not_raise_at_limit(self) -> None:
        ax = self._ax()
        draw_gauge(ax, 4.0, -2.0, 4.0, 1.0, "n_z", "g")

    def test_does_not_raise_at_negative_limit(self) -> None:
        ax = self._ax()
        draw_gauge(ax, -2.0, -2.0, 4.0, 1.0, "n_z", "g")


# ---------------------------------------------------------------------------
# InputMonitorFigure
# ---------------------------------------------------------------------------


_NEUTRAL_FRAME = {
    "n_z": 1.0,
    "n_y": 0.0,
    "roll_rate_wind_rps": 0.0,
    "throttle_nd": 0.05,
    "connected": False,
}


class TestInputMonitorFigure:
    def test_constructs_without_error(self) -> None:
        cfg = InputMonitorConfig()
        monitor = InputMonitorFigure(cfg)
        assert monitor is not None

    def test_figure_property_returns_figure(self) -> None:
        cfg = InputMonitorConfig()
        monitor = InputMonitorFigure(cfg)
        assert isinstance(monitor.figure, mpl_figure.Figure)

    def test_redraw_neutral_frame_does_not_raise(self) -> None:
        cfg = InputMonitorConfig()
        monitor = InputMonitorFigure(cfg)
        monitor.redraw(_NEUTRAL_FRAME)

    def test_redraw_extreme_frame_does_not_raise(self) -> None:
        cfg = InputMonitorConfig()
        monitor = InputMonitorFigure(cfg)
        monitor.redraw(
            {
                "n_z": 4.0,
                "n_y": 2.0,
                "roll_rate_wind_rps": math.pi / 2.0,
                "throttle_nd": 1.0,
                "connected": True,
            }
        )

    def test_accepts_external_figure(self) -> None:
        cfg = InputMonitorConfig()
        external_fig = mpl_figure.Figure(figsize=(8, 4))
        monitor = InputMonitorFigure(cfg, fig=external_fig)
        assert monitor.figure is external_fig

    def test_redraw_updates_scatter_positions(self) -> None:
        cfg = InputMonitorConfig()
        monitor = InputMonitorFigure(cfg)
        frame = {
            "n_z": 2.0,
            "n_y": 1.0,
            "roll_rate_wind_rps": math.pi / 4.0,
            "throttle_nd": 0.5,
            "connected": True,
        }
        monitor.redraw(frame)
        # Left scatter: (n_y, throttle)
        left_offsets = monitor._scatter_left.get_offsets()
        assert left_offsets[0, 0] == pytest.approx(1.0)
        assert left_offsets[0, 1] == pytest.approx(0.5)
        # Right scatter: (roll_deg, n_z)
        right_offsets = monitor._scatter_right.get_offsets()
        assert right_offsets[0, 0] == pytest.approx(math.degrees(math.pi / 4.0))
        assert right_offsets[0, 1] == pytest.approx(2.0)
