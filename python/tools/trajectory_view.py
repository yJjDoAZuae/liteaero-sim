"""TrajectoryView, RibbonTrail, HudOverlay — 3D animated trajectory visualization.

RibbonTrail
-----------
Encodes roll attitude as a 3D surface strip.  At each trajectory point the
wing half-span vector is rotated from body frame to world frame using a ZYX
Euler rotation matrix (R_z(ψ) R_y(θ) R_x(φ)).  Ribbon geometry is fully
pre-computed before animation starts; the animation loop only updates object
data, not rotation matrices.

HudOverlay
----------
Fixed-position ``ax.text2D`` artists updated each frame via ``set_text()``.

TrajectoryView
--------------
Wraps a matplotlib 3D figure with ``FuncAnimation``.  Supports loading up to
two sources for dual-source overlay (UC-PP4).
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

if TYPE_CHECKING:
    import pandas as pd
    from mode_overlay import ModeEventSeries

# ---------------------------------------------------------------------------
# Rotation matrix
# ---------------------------------------------------------------------------


def _rotation_matrix(heading_rad: float, pitch_rad: float, roll_rad: float) -> np.ndarray:
    """ZYX Euler rotation: R_z(ψ) @ R_y(θ) @ R_x(φ).  Returns (3, 3) array."""
    cpsi, spsi = math.cos(heading_rad), math.sin(heading_rad)
    cth, sth = math.cos(pitch_rad), math.sin(pitch_rad)
    cphi, sphi = math.cos(roll_rad), math.sin(roll_rad)

    rz = np.array([[cpsi, -spsi, 0.0], [spsi, cpsi, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cth, 0.0, sth], [0.0, 1.0, 0.0], [-sth, 0.0, cth]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cphi, -sphi], [0.0, sphi, cphi]])
    return rz @ ry @ rx


# ---------------------------------------------------------------------------
# RibbonTrail
# ---------------------------------------------------------------------------


class RibbonTrail:
    """Pre-computed 3D ribbon that encodes roll attitude as a surface strip.

    Build once, then call ``collection()`` to get a ``Poly3DCollection``
    suitable for adding to a ``mpl_toolkits.mplot3d`` axes.
    """

    def __init__(self) -> None:
        self._vertices: np.ndarray | None = None  # (N-1, 4, 3)
        self._colors: np.ndarray | None = None     # (N-1, 4) RGBA

    @property
    def vertices(self) -> np.ndarray | None:
        return self._vertices

    @property
    def colors(self) -> np.ndarray | None:
        return self._colors

    def build(
        self,
        positions: np.ndarray,
        roll_rad: np.ndarray,
        heading_rad: np.ndarray,
        pitch_rad: np.ndarray,
        half_width_m: float = 5.0,
    ) -> None:
        """Pre-compute ribbon geometry from trajectory arrays.

        Parameters
        ----------
        positions:
            Shape ``(N, 3)`` world-frame position array.
        roll_rad, heading_rad, pitch_rad:
            Shape ``(N,)`` attitude arrays in radians.
        half_width_m:
            Ribbon half-width in meters (default: 5 m ≈ wing span of a small UA).
        """
        n = len(positions)
        wing_body = np.array([0.0, half_width_m, 0.0])

        v_upper = np.empty((n, 3))
        v_lower = np.empty((n, 3))
        for i in range(n):
            r = _rotation_matrix(float(heading_rad[i]), float(pitch_rad[i]), float(roll_rad[i]))
            w = r @ wing_body
            v_upper[i] = positions[i] + w
            v_lower[i] = positions[i] - w

        # Quads: [v_lower[i], v_upper[i], v_upper[i+1], v_lower[i+1]]
        quads = np.empty((n - 1, 4, 3))
        quads[:, 0] = v_lower[:-1]
        quads[:, 1] = v_upper[:-1]
        quads[:, 2] = v_upper[1:]
        quads[:, 3] = v_lower[1:]
        self._vertices = quads

        # Color: roll mapped through RdBu_r, saturated at ±60° (±π/3)
        norm = mcolors.TwoSlopeNorm(vmin=-math.pi / 3, vcenter=0.0, vmax=math.pi / 3)
        cmap = plt.get_cmap("RdBu_r")
        mid_roll = (roll_rad[:-1] + roll_rad[1:]) / 2.0
        mid_roll_clipped = np.clip(mid_roll, -math.pi / 3, math.pi / 3)
        self._colors = cmap(norm(mid_roll_clipped))  # (N-1, 4) RGBA

    def collection(self) -> Poly3DCollection:
        """Return a Poly3DCollection ready to add to 3D axes."""
        if self._vertices is None:
            raise RuntimeError("Call build() before collection()")
        coll = Poly3DCollection(list(self._vertices), alpha=0.6)
        if self._colors is not None:
            coll.set_facecolor(self._colors)
        return coll


# ---------------------------------------------------------------------------
# HudOverlay
# ---------------------------------------------------------------------------


class HudOverlay:
    """Fixed-position HUD text artists on a 3D axes.

    All elements are ``ax.text2D`` artists at fixed axes-coordinate positions
    updated each frame via ``set_text()``.
    """

    def __init__(self, ax: plt.Axes, source_label: str = "Simulation") -> None:
        self._ax = ax
        kw: dict = dict(transform=ax.transAxes, fontsize=9, fontfamily="monospace")

        self._source = ax.text2D(0.01, 0.97, f"DATA SOURCE: {source_label}", **kw)
        self._time = ax.text2D(0.99, 0.97, "SIM TIME   0.00 s", ha="right", **kw)
        self._ias = ax.text2D(0.01, 0.07, "IAS    --- m/s", **kw)
        self._alt = ax.text2D(0.01, 0.04, "ALT    --- m MSL", **kw)
        self._vs = ax.text2D(0.01, 0.01, "VS     --- m/s", **kw)
        self._roll = ax.text2D(0.25, 0.07, "ROLL   ---°", **kw)
        self._pitch = ax.text2D(0.25, 0.04, "PITCH  ---°", **kw)
        self._hdg = ax.text2D(0.25, 0.01, "HDG    ---°", **kw)
        self._mode = ax.text2D(0.60, 0.01, "MODE: ---", **kw)
        self._banner = ax.text2D(
            0.50, 0.50, "",
            ha="center", va="center",
            fontsize=18, fontweight="bold", alpha=0.0,
            transform=ax.transAxes,
        )
        self._banner_frames_remaining: int = 0
        self._banner_total: int = 60

    def update(self, frame_data: dict) -> None:
        """Update HUD artists from a dict of channel values keyed by channel name."""
        if "time_s" in frame_data:
            self._time.set_text(f"SIM TIME  {float(frame_data['time_s']):8.2f} s")
        if "airspeed_ias_m_s" in frame_data:
            self._ias.set_text(f"IAS  {float(frame_data['airspeed_ias_m_s']):6.1f} m/s")
        if "altitude_m" in frame_data:
            self._alt.set_text(f"ALT  {float(frame_data['altitude_m']):6.1f} m MSL")
        if "vertical_speed_m_s" in frame_data:
            self._vs.set_text(f"VS  {float(frame_data['vertical_speed_m_s']):+6.1f} m/s")
        if "roll_rad" in frame_data:
            self._roll.set_text(f"ROLL  {math.degrees(float(frame_data['roll_rad'])):+6.1f}°")
        if "pitch_rad" in frame_data:
            self._pitch.set_text(f"PITCH  {math.degrees(float(frame_data['pitch_rad'])):+5.1f}°")
        if "heading_rad" in frame_data:
            self._hdg.set_text(
                f"HDG  {math.degrees(float(frame_data['heading_rad'])) % 360.0:.0f}°"
            )
        if "mode_name" in frame_data:
            mode = frame_data["mode_name"]
            self._mode.set_text(f"MODE: {mode}")
            self._banner.set_text(mode)
            self._banner.set_alpha(1.0)
            self._banner_frames_remaining = self._banner_total

        # Fade banner
        if self._banner_frames_remaining > 0:
            self._banner_frames_remaining -= 1
            alpha = self._banner_frames_remaining / self._banner_total
            self._banner.set_alpha(alpha)


# ---------------------------------------------------------------------------
# TrajectoryView
# ---------------------------------------------------------------------------

_POS_COLUMN_SETS = [
    ["pos_north_m", "pos_east_m", "pos_down_m"],
    ["position_north_m", "position_east_m", "position_down_m"],
    ["x_m", "y_m", "z_m"],
]
_ATTITUDE_CANDIDATES = {
    "roll": ["roll_rad"],
    "pitch": ["pitch_rad"],
    "heading": ["heading_rad", "yaw_rad"],
}


class TrajectoryView:
    """3D animated trajectory with ribbon trail and HUD overlay.

    Load one or two sources, then call ``animate()`` to get a
    ``FuncAnimation``.  Call ``show()`` to display interactively.
    Tests should call ``animate()`` or ``build()`` only — never ``show()``.
    """

    def __init__(self) -> None:
        self._sources: list[dict] = []
        self._mode_events: ModeEventSeries | None = None

    def load(
        self,
        df: pd.DataFrame,
        label: str = "Simulation",
        color: str = "steelblue",
    ) -> None:
        """Add a source DataFrame (call once or twice for dual-source overlay)."""
        self._sources.append({"df": df, "label": label, "color": color})

    def set_mode_events(self, events: ModeEventSeries) -> None:
        self._mode_events = events

    def animate(self, interval_ms: int = 50) -> FuncAnimation:
        """Build the figure and return a FuncAnimation (does not open a window).

        The animation uses ``blit=False`` because 3D axes cannot blit when
        the projection changes during interactive rotation.
        """
        if not self._sources:
            raise RuntimeError("Call load() at least once before animate()")

        fig = plt.figure(figsize=(12, 9))
        ax3d = fig.add_axes([0.0, 0.15, 1.0, 0.83], projection="3d")

        src = self._sources[0]
        df: pd.DataFrame = src["df"]

        positions = _extract_positions(df)
        roll = _extract_attitude(df, "roll")
        pitch = _extract_attitude(df, "pitch")
        heading = _extract_attitude(df, "heading")
        n = len(df)

        # Pre-compute ribbon
        ribbon = RibbonTrail()
        ribbon.build(positions, roll, heading, pitch)

        ghost = ribbon.collection()
        ghost.set_alpha(0.15)
        ax3d.add_collection3d(ghost)

        n_trail = min(200, n - 1)
        live = Poly3DCollection([], alpha=0.8)
        if ribbon.colors is not None:
            live.set_facecolor(ribbon.colors[:0])
        ax3d.add_collection3d(live)

        marker, = ax3d.plot([], [], [], "o", color=src["color"], markersize=8, zorder=10)

        hud = HudOverlay(ax3d, source_label=src["label"])

        # Axis limits with margin
        margin = max(50.0, 0.05 * np.ptp(positions, axis=0).max())
        ax3d.set_xlim(positions[:, 0].min() - margin, positions[:, 0].max() + margin)
        ax3d.set_ylim(positions[:, 1].min() - margin, positions[:, 1].max() + margin)
        ax3d.set_zlim(positions[:, 2].min() - margin, positions[:, 2].max() + margin)
        ax3d.set_xlabel("X (m)")
        ax3d.set_ylabel("Y (m)")
        ax3d.set_zlabel("Z (m)")

        # Second source overlay
        if len(self._sources) > 1:
            src2 = self._sources[1]
            df2: pd.DataFrame = src2["df"]
            pos2 = _extract_positions(df2)
            r2 = RibbonTrail()
            r2.build(pos2, _extract_attitude(df2, "roll"),
                     _extract_attitude(df2, "heading"), _extract_attitude(df2, "pitch"))
            ghost2 = r2.collection()
            ghost2.set_alpha(0.15)
            ax3d.add_collection3d(ghost2)

        def _update(frame: int) -> tuple:
            pos = positions[frame]
            marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])

            start = max(0, frame - n_trail)
            if ribbon.vertices is not None and frame > 0:
                live.set_verts(list(ribbon.vertices[start:frame]))
                if ribbon.colors is not None:
                    live.set_facecolor(ribbon.colors[start:frame])

            frame_data: dict = {"time_s": float(df.index[frame])}
            row = df.iloc[frame]
            for col in df.columns:
                frame_data[col] = row[col]
            hud.update(frame_data)

            return (marker, live)

        anim = FuncAnimation(fig, _update, frames=n, interval=interval_ms, blit=False)
        return anim

    def show(self) -> None:
        """Build and display the animation interactively."""
        self.animate().event_source  # ensure built
        plt.show()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _extract_positions(df: pd.DataFrame) -> np.ndarray:
    for cols in _POS_COLUMN_SETS:
        if all(c in df.columns for c in cols):
            return df[cols].to_numpy(dtype=float)
    # Fall back to zeros if no position columns present
    return np.zeros((len(df), 3))


def _extract_attitude(df: pd.DataFrame, kind: str) -> np.ndarray:
    for col in _ATTITUDE_CANDIDATES[kind]:
        if col in df.columns:
            return df[col].to_numpy(dtype=float)
    return np.zeros(len(df))
