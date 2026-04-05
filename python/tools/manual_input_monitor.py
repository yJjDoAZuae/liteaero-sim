"""Manual input monitor — shared library and standalone Qt app.

Library components (importable from notebooks and scripts):
    InputMonitorConfig  — configuration dataclass
    enumerate_devices   — list attached joystick devices via joystick_verify
    build_args          — construct joystick_verify command-line argument list
    stdout_reader       — background thread target for reading subprocess output
    draw_gauge          — draw one horizontal bar gauge on a matplotlib Axes
    InputMonitorFigure  — pure-matplotlib display class (no Qt dependency)

Standalone app:
    InputMonitorWindow  — QMainWindow wrapping InputMonitorFigure
    main()              — argparse entry point

Design authority: docs/architecture/manual_input.md §Manual Input Monitor
"""
from __future__ import annotations

import argparse
import json
import os
import math
import queue
import subprocess
import sys
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib.figure as mpl_figure
import matplotlib.gridspec as gridspec
import numpy as np


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass
class InputMonitorConfig:
    """All configurable parameters for the manual input monitor."""

    joystick_verify: Path = field(
        default_factory=lambda: Path("../build/tools/joystick_verify.exe")
    )
    device_name: str = ""
    device_index: int = 0
    config_file: str = ""
    rate_hz: float = 50.0
    idle_throttle_nd: float = 0.05
    min_nz_g: float = -2.0
    max_nz_g: float = 4.0
    max_ny_g: float = 2.0
    max_roll_rate_rad_s: float = math.pi / 2.0


# ---------------------------------------------------------------------------
# Subprocess helpers
# ---------------------------------------------------------------------------


def _subprocess_env() -> dict:
    """Build an environment for joystick_verify subprocesses.

    Prepends the MSYS2 ucrt64 bin directory to PATH so that SDL2.dll and its
    MinGW runtime dependencies (libgcc, libstdc++, libwinpthread) are found
    without requiring ucrt64/bin to be on the global system PATH.
    Sets SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1 so SDL enumerates HID joysticks
    when the process has no foreground window.
    """
    env = os.environ.copy()
    ucrt64_bin = r"C:\msys64\ucrt64\bin"
    if os.path.isdir(ucrt64_bin):
        env["PATH"] = ucrt64_bin + os.pathsep + env.get("PATH", "")
    env["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
    return env


def enumerate_devices(config: InputMonitorConfig) -> list[dict]:
    """Launch joystick_verify briefly to list attached joystick devices.

    Returns a list of dicts with keys "index" (int), "axes" (int), "name" (str).
    Raises FileNotFoundError if the joystick_verify executable does not exist.
    """
    exe = Path(config.joystick_verify).resolve()
    if not exe.exists():
        raise FileNotFoundError(
            f"joystick_verify not found at {exe}\n"
            "Build the project first: mingw32-make -C build"
        )
    env = _subprocess_env()
    proc = subprocess.Popen(
        [str(exe), "--device-index", str(config.device_index)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    devices: list[dict] = []
    for line in proc.stdout:
        line = line.rstrip()
        if line == "READY":
            break
        if line.startswith("DEVICE "):
            # Format: DEVICE <index> <num_axes> <name>
            parts = line.split(" ", 3)
            devices.append({"index": int(parts[1]), "axes": int(parts[2]), "name": parts[3]})
    proc.terminate()
    proc.wait()
    return devices


def build_args(config: InputMonitorConfig) -> list[str]:
    """Construct the joystick_verify command-line argument list from config."""
    args = [str(Path(config.joystick_verify).resolve()), "--rate-hz", str(config.rate_hz)]
    if config.device_name:
        args += ["--device-name", config.device_name]
    else:
        args += ["--device-index", str(config.device_index)]
    if config.config_file:
        args += ["--config", config.config_file]
    return args


def stdout_reader(
    proc: subprocess.Popen,
    q: queue.Queue,
    ready: threading.Event,
) -> None:
    """Background daemon thread target.

    Reads lines from proc.stdout. Sets ready on the READY sentinel.
    Puts all subsequent JSON-frame lines into queue q.
    DEVICE lines and blank lines are discarded.
    """
    for line in proc.stdout:
        line = line.rstrip()
        if line == "READY":
            ready.set()
            continue
        if line.startswith("DEVICE ") or not line:
            continue
        q.put(line)


# ---------------------------------------------------------------------------
# Display — pure matplotlib, no Qt dependency
# ---------------------------------------------------------------------------


def draw_gauge(
    ax,
    value: float,
    v_min: float,
    v_max: float,
    v_neutral: float,
    label: str,
    unit: str,
) -> None:
    """Draw one horizontal bar gauge on ax.

    Clears and redraws from scratch. Suitable for one-shot rendering.
    For live updates use init_gauge() + update_gauge() to avoid flashing.

    The bar extends from v_neutral to value. Color transitions:
      blue   — deflection < 25 %
      orange — 25–75 %
      red    — > 75 %
    where deflection = |value − v_neutral| / half_span.
    """
    ax.clear()
    span = v_max - v_min
    ax.barh(0, span, left=v_min, height=0.55, color="#ebebeb", edgecolor="#c8c8c8", linewidth=0.5)

    half_span = span / 2.0
    deflection = abs(value - v_neutral) / half_span if half_span > 0.0 else 0.0
    if deflection < 0.25:
        bar_color = "#1565C0"
    elif deflection < 0.75:
        bar_color = "#FF9800"
    else:
        bar_color = "#F44336"

    ax.barh(0, value - v_neutral, left=v_neutral, height=0.55, color=bar_color, alpha=0.85)
    ax.axvline(v_neutral, color="#303030", linewidth=1.5, zorder=4)
    ax.text(
        0.99,
        0.5,
        f"{value:+6.2f} {unit}",
        transform=ax.transAxes,
        va="center",
        ha="right",
        fontsize=10,
        fontfamily="monospace",
        fontweight="bold",
    )
    ax.set_title(label, fontsize=9, pad=3, loc="left", fontweight="bold", color="#404040")
    ax.set_xlim(v_min, v_max)
    ax.set_ylim(-0.9, 0.9)
    ax.set_yticks([])
    ax.tick_params(axis="x", labelsize=7)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)


def _gauge_color(value: float, v_neutral: float, half_span: float) -> str:
    deflection = abs(value - v_neutral) / half_span if half_span > 0.0 else 0.0
    if deflection < 0.25:
        return "#1565C0"
    elif deflection < 0.75:
        return "#FF9800"
    return "#F44336"


def init_gauge(
    ax,
    v_min: float,
    v_max: float,
    v_neutral: float,
    label: str,
    unit: str,
    initial_value: float = 0.0,
):
    """Initialize a gauge axes with static elements and return (bar_patch, text_artist).

    Call once at figure construction. Then call update_gauge() on each tick to
    update the bar width, color, and value text without clearing the axes.
    """
    span = v_max - v_min
    half_span = span / 2.0

    # Static background track
    ax.barh(0, span, left=v_min, height=0.55, color="#ebebeb", edgecolor="#c8c8c8", linewidth=0.5)
    ax.axvline(v_neutral, color="#303030", linewidth=1.5, zorder=4)
    ax.set_title(label, fontsize=9, pad=3, loc="left", fontweight="bold", color="#404040")
    ax.set_xlim(v_min, v_max)
    ax.set_ylim(-0.9, 0.9)
    ax.set_yticks([])
    ax.tick_params(axis="x", labelsize=7)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)

    # Dynamic value bar — created once, updated in place
    color = _gauge_color(initial_value, v_neutral, half_span)
    bar_container = ax.barh(
        0, initial_value - v_neutral, left=v_neutral, height=0.55, color=color, alpha=0.85
    )
    bar_patch = bar_container.patches[0]

    # Dynamic value text — updated in place
    text_artist = ax.text(
        0.99,
        0.5,
        f"{initial_value:+6.2f} {unit}",
        transform=ax.transAxes,
        va="center",
        ha="right",
        fontsize=10,
        fontfamily="monospace",
        fontweight="bold",
    )
    return bar_patch, text_artist


def update_gauge(
    bar_patch,
    text_artist,
    value: float,
    v_neutral: float,
    half_span: float,
    unit: str,
) -> None:
    """Update a gauge bar patch and text in place. No axes clear."""
    bar_patch.set_x(min(value, v_neutral))
    bar_patch.set_width(abs(value - v_neutral))
    bar_patch.set_facecolor(_gauge_color(value, v_neutral, half_span))
    text_artist.set_text(f"{value:+6.2f} {unit}")


class InputMonitorFigure:
    """Pure-matplotlib display class.

    No dependency on Qt or ipympl. Owns the Figure, six subplot axes, and the
    two scatter artists. Callers that manage their own event loop (Qt, ipympl)
    call redraw() from their timer or animation callback.

    Parameters
    ----------
    config:
        Display configuration.
    fig:
        Optional external Figure. If None, a new Figure is created internally.
        Pass a pyplot-managed figure when using the ipympl widget backend so
        that the figure is automatically registered with Jupyter.

    Design authority: docs/architecture/manual_input.md §Display Layout
    """

    def __init__(
        self,
        config: InputMonitorConfig,
        fig: Optional[mpl_figure.Figure] = None,
    ) -> None:
        self._config = config
        self._fig = fig if fig is not None else mpl_figure.Figure(
            figsize=(10, 5.5), facecolor="#f5f5f5"
        )
        gs = gridspec.GridSpec(
            2,
            4,
            figure=self._fig,
            left=0.06,
            right=0.97,
            top=0.93,
            bottom=0.08,
            hspace=0.55,
            wspace=0.35,
        )
        self._ax_box_left = self._fig.add_subplot(gs[0, 0:2])
        self._ax_box_right = self._fig.add_subplot(gs[0, 2:4])
        self._ax_gauges = [self._fig.add_subplot(gs[1, i]) for i in range(4)]
        self._init_boxes()
        self._init_gauges()

    def _init_boxes(self) -> None:
        cfg = self._config
        max_roll_deg = math.degrees(cfg.max_roll_rate_rad_s)

        ax = self._ax_box_left
        ax.set_title("Throttle / n_y", fontsize=10, fontweight="bold", color="#404040")
        ax.set_xlabel("n_y  (g)", fontsize=8)
        ax.set_ylabel("Throttle", fontsize=8)
        ax.set_xlim(-cfg.max_ny_g, cfg.max_ny_g)
        ax.set_ylim(0.0, 1.0)
        ax.axvline(0.0, color="#aaaaaa", linewidth=0.8)
        ax.axhline(cfg.idle_throttle_nd, color="#aaaaaa", linewidth=0.8)
        ax.tick_params(labelsize=7)
        ax.set_facecolor("#fafafa")
        self._scatter_left = ax.scatter(
            [0.0], [cfg.idle_throttle_nd], s=80, color="#1565C0", zorder=5, clip_on=True
        )

        ax = self._ax_box_right
        ax.set_title("n_z / Roll Rate", fontsize=10, fontweight="bold", color="#404040")
        ax.set_xlabel("Roll rate  (°/s)", fontsize=8)
        ax.set_ylabel("n_z  (g)", fontsize=8)
        ax.set_xlim(-max_roll_deg, max_roll_deg)
        ax.set_ylim(cfg.min_nz_g, cfg.max_nz_g)
        ax.invert_yaxis()
        ax.axvline(0.0, color="#aaaaaa", linewidth=0.8)
        ax.axhline(1.0, color="#aaaaaa", linewidth=0.8)
        ax.tick_params(labelsize=7)
        ax.set_facecolor("#fafafa")
        self._scatter_right = ax.scatter(
            [0.0], [1.0], s=80, color="#1565C0", zorder=5, clip_on=True
        )

    def _init_gauges(self) -> None:
        cfg = self._config
        max_roll_deg = math.degrees(cfg.max_roll_rate_rad_s)
        # (v_min, v_max, v_neutral, label, unit, initial_value)
        specs = [
            (cfg.min_nz_g, cfg.max_nz_g, 1.0, "n_z", "g", 1.0),
            (-cfg.max_ny_g, cfg.max_ny_g, 0.0, "n_y", "g", 0.0),
            (-max_roll_deg, max_roll_deg, 0.0, "Roll rate", "°/s", 0.0),
            (0.0, 100.0, cfg.idle_throttle_nd * 100.0, "Throttle", "%", cfg.idle_throttle_nd * 100.0),
        ]
        self._gauge_artists: list[tuple] = []  # (bar_patch, text_artist, v_neutral, half_span, unit)
        for ax, (vmin, vmax, vneutral, label, unit, initial) in zip(self._ax_gauges, specs):
            bar_patch, text_artist = init_gauge(ax, vmin, vmax, vneutral, label, unit, initial)
            half_span = (vmax - vmin) / 2.0
            self._gauge_artists.append((bar_patch, text_artist, vneutral, half_span, unit))

    def redraw(self, frame: dict) -> None:
        """Update scatter positions and gauge artists from a frame dict.

        frame must contain keys: n_z, n_y, roll_rate_wind_rps, throttle_nd.
        """
        cfg = self._config
        nz = frame["n_z"]
        ny = frame["n_y"]
        roll_rps = frame["roll_rate_wind_rps"]
        throttle = frame["throttle_nd"]
        roll_deg = math.degrees(roll_rps)

        self._scatter_left.set_offsets(np.array([[ny, throttle]]))
        self._scatter_right.set_offsets(np.array([[roll_deg, nz]]))

        values = [nz, ny, roll_deg, throttle * 100.0]
        for val, (bar_patch, text_artist, vneutral, half_span, unit) in zip(
            values, self._gauge_artists
        ):
            update_gauge(bar_patch, text_artist, val, vneutral, half_span, unit)

        self._fig.canvas.draw_idle()

    @property
    def figure(self) -> mpl_figure.Figure:
        return self._fig


# ---------------------------------------------------------------------------
# Standalone Qt app
# ---------------------------------------------------------------------------


class InputMonitorWindow:
    """QMainWindow wrapping InputMonitorFigure.

    Owns the joystick_verify subprocess and the background reader thread.
    A QTimer fires at 20 Hz to drain the frame queue and call
    InputMonitorFigure.redraw(). The subprocess is terminated in closeEvent.

    Import is deferred so the library can be used without PySide6 installed
    (e.g., when imported from a notebook that uses the ipympl backend only).
    """

    UPDATE_MS = 50  # 20 Hz

    def __init__(self, config: InputMonitorConfig) -> None:
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

        self._config = config

        # Subprocess + reader thread
        self._q: queue.Queue = queue.Queue()
        self._ready = threading.Event()
        self._proc = subprocess.Popen(
            build_args(config),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=_subprocess_env(),
        )
        self._reader = threading.Thread(
            target=stdout_reader,
            args=(self._proc, self._q, self._ready),
            daemon=True,
        )
        self._reader.start()

        if not self._ready.wait(timeout=5.0):
            self._proc.terminate()
            raise RuntimeError("joystick_verify did not become ready within 5 s.")

        # Qt window
        self._win = QMainWindow()
        self._win.setWindowTitle("Manual Input Monitor — LiteAero Sim")
        self._win.resize(900, 560)

        self._monitor = InputMonitorFigure(config)
        self._canvas = FigureCanvasQTAgg(self._monitor.figure)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(self._canvas)
        self._win.setCentralWidget(central)

        self._status_label = QLabel("Waiting for frames…")
        self._status_label.setStyleSheet("padding-left: 6px; font-size: 11px;")
        self._win.statusBar().addWidget(self._status_label)

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)

        self._last_frame: dict = {
            "n_z": 1.0,
            "n_y": 0.0,
            "roll_rate_wind_rps": 0.0,
            "throttle_nd": config.idle_throttle_nd,
        }

        self._win.closeEvent = self._close_event

    def show(self) -> None:
        self._timer.start(self.UPDATE_MS)
        self._win.show()

    def _tick(self) -> None:
        frame = None
        while True:
            try:
                line = self._q.get_nowait()
                frame = json.loads(line)
            except queue.Empty:
                break
            except json.JSONDecodeError:
                continue

        if frame is not None:
            self._last_frame = frame

        self._monitor.redraw(self._last_frame)

        connected = self._last_frame.get("connected", False)
        color = "#1565C0" if connected else "#B71C1C"
        status = "CONNECTED" if connected else "DISCONNECTED"
        self._status_label.setText(f"Joystick: {status}")
        self._status_label.setStyleSheet(
            f"padding-left: 6px; font-size: 11px; color: {color};"
        )

        if self._proc.poll() is not None:
            self._timer.stop()
            self._status_label.setText("joystick_verify exited.")

    def _close_event(self, event) -> None:
        self._timer.stop()
        self._proc.terminate()
        self._proc.wait()
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _parse_args() -> InputMonitorConfig:
    defaults = InputMonitorConfig()
    parser = argparse.ArgumentParser(
        description="Manual Input Monitor — displays live AircraftCommand values from joystick_verify."
    )
    parser.add_argument(
        "--joystick-verify",
        type=Path,
        default=defaults.joystick_verify,
        help="Path to joystick_verify executable (default: %(default)s)",
    )
    parser.add_argument(
        "--device-name",
        default=defaults.device_name,
        help="Substring of device name for selection (default: use --device-index)",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=defaults.device_index,
        help="Device index (default: %(default)s)",
    )
    parser.add_argument(
        "--config",
        dest="config_file",
        default=defaults.config_file,
        help="Path to joystick JSON config file (default: none)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=defaults.rate_hz,
        help="Polling rate in Hz (default: %(default)s)",
    )
    ns = parser.parse_args()
    return InputMonitorConfig(
        joystick_verify=ns.joystick_verify,
        device_name=ns.device_name,
        device_index=ns.device_index,
        config_file=ns.config_file,
        rate_hz=ns.rate_hz,
    )


def main() -> None:
    from PySide6.QtWidgets import QApplication

    config = _parse_args()

    app = QApplication.instance() or QApplication(sys.argv)
    win = InputMonitorWindow(config)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
