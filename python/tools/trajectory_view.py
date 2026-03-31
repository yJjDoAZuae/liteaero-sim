"""TrajectoryView, RibbonTrail, HudOverlay — 3D animated trajectory visualization.

RibbonTrail
-----------
Encodes roll attitude as a 3D surface strip.  At each trajectory point the
wing half-span vector is rotated from body frame to world frame using a ZYX
Euler rotation matrix (R_z(ψ) R_y(θ) R_x(φ)).  Ribbon geometry is fully
pre-computed before animation starts.

The ribbon uses CCW quad winding ``(v_lower[i], v_lower[i+1], v_upper[i+1],
v_upper[i])`` and returns a Vispy ``Mesh`` visual via ``mesh()``.

When ``timestamps`` are provided, per-segment alpha fade is computed by
``alpha_at(t_current)`` using the formula ``α_i = log(2 − τ_i) / log(2)``
where ``τ_i = (t_current − t_mid_i) / trail_duration_s``.

HudOverlay
----------
Vispy ``Text`` visuals placed in a 2D overlay view at fixed screen positions.
Updated each animation frame via ``update(frame_data)``.

TrajectoryView
--------------
``QMainWindow`` embedding a Vispy ``SceneCanvas``.  ``animate()`` prepares and
returns the canvas (headless-safe); ``show()`` opens the interactive Qt window.

Supports loading up to two sources for dual-source overlay (UC-PP4).
Terrain glTF tiles loaded via ``pygltflib``; terrain colour saturation
adjustable via ``set_terrain_saturation()``.
"""
from __future__ import annotations

import enum
import math
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

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
# CameraMode
# ---------------------------------------------------------------------------


class CameraMode(enum.Enum):
    """Camera tracking modes for TrajectoryView."""
    FPV = "fpv"
    TRAIL = "trail"
    GODS_EYE = "gods_eye"
    LOCAL_TOP = "local_top"


# ---------------------------------------------------------------------------
# RibbonTrail
# ---------------------------------------------------------------------------


class RibbonTrail:
    """Pre-computed 3D ribbon that encodes roll attitude as a surface strip.

    Build once, then call ``mesh()`` to obtain a Vispy ``Mesh`` visual
    suitable for adding to a Vispy scene.
    """

    def __init__(self) -> None:
        self._vertices: np.ndarray | None = None   # (N-1, 4, 3) quad vertices
        self._colors: np.ndarray | None = None      # (N-1, 4) RGBA per quad
        self._timestamps: np.ndarray | None = None  # (N-1,) midpoint times
        self._trail_duration_s: float = 30.0

    @property
    def vertices(self) -> np.ndarray | None:
        return self._vertices

    @property
    def colors(self) -> np.ndarray | None:
        return self._colors

    @property
    def timestamps(self) -> np.ndarray | None:
        return self._timestamps

    def build(
        self,
        positions: np.ndarray,
        roll_rad: np.ndarray,
        heading_rad: np.ndarray,
        pitch_rad: np.ndarray,
        wing_span_m: float = 10.0,
        timestamps: np.ndarray | None = None,
        trail_duration_s: float = 30.0,
        show_edges: bool = False,  # noqa: ARG002  (used by Vispy mesh in task F)
    ) -> None:
        """Pre-compute ribbon geometry from trajectory arrays.

        Parameters
        ----------
        positions:
            Shape ``(N, 3)`` world-frame position array.
        roll_rad, heading_rad, pitch_rad:
            Shape ``(N,)`` attitude arrays in radians.
        wing_span_m:
            Full wing span in meters (half-span used for ribbon width).
        timestamps:
            Shape ``(N,)`` sample times in seconds.  Required for alpha fade.
        trail_duration_s:
            Duration of the fade window in seconds.
        show_edges:
            Reserved for Vispy mesh edge rendering.
        """
        n = len(positions)
        half_span = wing_span_m / 2.0
        wing_body = np.array([0.0, half_span, 0.0])

        v_upper = np.empty((n, 3))
        v_lower = np.empty((n, 3))
        for i in range(n):
            r = _rotation_matrix(float(heading_rad[i]), float(pitch_rad[i]), float(roll_rad[i]))
            w = r @ wing_body
            v_upper[i] = positions[i] + w
            v_lower[i] = positions[i] - w

        # CCW winding: [v_lower[i], v_lower[i+1], v_upper[i+1], v_upper[i]]
        quads = np.empty((n - 1, 4, 3))
        quads[:, 0] = v_lower[:-1]
        quads[:, 1] = v_lower[1:]
        quads[:, 2] = v_upper[1:]
        quads[:, 3] = v_upper[:-1]
        self._vertices = quads

        # Color: roll mapped through RdBu_r, saturated at ±60° (±π/3)
        norm = mcolors.TwoSlopeNorm(vmin=-math.pi / 3, vcenter=0.0, vmax=math.pi / 3)
        cmap = plt.get_cmap("RdBu_r")
        mid_roll = (roll_rad[:-1] + roll_rad[1:]) / 2.0
        mid_roll_clipped = np.clip(mid_roll, -math.pi / 3, math.pi / 3)
        self._colors = cmap(norm(mid_roll_clipped))  # (N-1, 4) RGBA

        # Segment midpoint timestamps for per-frame alpha fade
        if timestamps is not None:
            self._timestamps = (timestamps[:-1] + timestamps[1:]) / 2.0
        else:
            self._timestamps = None
        self._trail_duration_s = trail_duration_s

    def alpha_at(self, t_current: float) -> np.ndarray | None:
        """Compute per-segment alpha values at ``t_current``.

        Formula: ``α_i = log(2 − τ_i) / log(2)``
        where ``τ_i = (t_current − t_mid_i) / trail_duration_s``.

        Returns ``None`` when no timestamps were supplied to ``build()``.
        Values are clipped to ``[0, 1]``.
        """
        if self._timestamps is None:
            return None
        tau = (t_current - self._timestamps) / self._trail_duration_s
        raw = np.log(np.clip(2.0 - tau, 1e-9, 2.0)) / math.log(2.0)
        return np.clip(raw, 0.0, 1.0)

    def mesh(self) -> object:
        """Return a Vispy MeshVisual with triangulated ribbon geometry.

        Raises ``RuntimeError`` if called before ``build()``.
        """
        if self._vertices is None:
            raise RuntimeError("Call build() before mesh()")

        from vispy.visuals import MeshVisual

        n_quads = len(self._vertices)
        verts = self._vertices.reshape(-1, 3).astype(np.float32)

        # Triangulate quads: each quad i → 2 triangles
        idx = np.arange(n_quads, dtype=np.uint32)
        base = 4 * idx
        tri0 = np.stack([base, base + 1, base + 2], axis=1)
        tri1 = np.stack([base, base + 2, base + 3], axis=1)
        faces = np.concatenate([tri0, tri1], axis=0)

        if self._colors is not None:
            vert_colors = np.repeat(self._colors, 4, axis=0).astype(np.float32)
        else:
            vert_colors = None

        return MeshVisual(vertices=verts, faces=faces, vertex_colors=vert_colors)


# ---------------------------------------------------------------------------
# HudOverlay
# ---------------------------------------------------------------------------


class HudOverlay:
    """Vispy Text visual HUD for the 3D trajectory view.

    All text elements are placed in a 2D overlay ViewBox that sits in front
    of the 3D scene.  Each element is updated each frame via ``update()``.
    """

    def __init__(
        self,
        canvas: object,
        source_label: str = "Simulation",
    ) -> None:
        from vispy.scene.visuals import Text

        grid = canvas.central_widget.add_grid()
        self._view = grid.add_view(row=0, col=0)
        self._view.camera = "null"

        kw = dict(
            color="white",
            font_size=9,
            anchor_x="left",
            anchor_y="top",
            parent=self._view.scene,
        )
        h, w = canvas.size[1], canvas.size[0]

        self._source = Text(f"DATA SOURCE: {source_label}", pos=(4, 4), **kw)
        self._time   = Text("SIM TIME   0.00 s",            pos=(w - 4, 4),
                            anchor_x="right", color="white", font_size=9,
                            parent=self._view.scene)
        self._ias   = Text("IAS    --- m/s",   pos=(4, h - 60), **kw)
        self._alt   = Text("ALT    --- m MSL", pos=(4, h - 40), **kw)
        self._vs    = Text("VS     --- m/s",   pos=(4, h - 20), **kw)
        self._roll  = Text("ROLL   ---°",      pos=(200, h - 60), **kw)
        self._pitch = Text("PITCH  ---°",      pos=(200, h - 40), **kw)
        self._hdg   = Text("HDG    ---°",      pos=(200, h - 20), **kw)
        self._mode  = Text("MODE: ---",        pos=(w // 2, h - 20),
                           anchor_x="center", color="white", font_size=9,
                           parent=self._view.scene)
        self._banner = Text("", pos=(w // 2, h // 2),
                            anchor_x="center", anchor_y="center",
                            color="white", font_size=18, bold=True,
                            parent=self._view.scene)
        self._banner_frames_remaining: int = 0
        self._banner_total: int = 60

    def update(self, frame_data: dict) -> None:
        """Update HUD text visuals from a dict of channel values."""
        if "time_s" in frame_data:
            self._time.text = f"SIM TIME  {float(frame_data['time_s']):8.2f} s"
        if "airspeed_ias_m_s" in frame_data:
            self._ias.text = f"IAS  {float(frame_data['airspeed_ias_m_s']):6.1f} m/s"
        if "altitude_m" in frame_data:
            self._alt.text = f"ALT  {float(frame_data['altitude_m']):6.1f} m MSL"
        if "vertical_speed_m_s" in frame_data:
            self._vs.text = f"VS  {float(frame_data['vertical_speed_m_s']):+6.1f} m/s"
        if "roll_rad" in frame_data:
            self._roll.text = f"ROLL  {math.degrees(float(frame_data['roll_rad'])):+6.1f}°"
        if "pitch_rad" in frame_data:
            self._pitch.text = f"PITCH  {math.degrees(float(frame_data['pitch_rad'])):+5.1f}°"
        if "heading_rad" in frame_data:
            self._hdg.text = (
                f"HDG  {math.degrees(float(frame_data['heading_rad'])) % 360.0:.0f}°"
            )
        if "mode_name" in frame_data:
            mode = frame_data["mode_name"]
            self._mode.text = f"MODE: {mode}"
            self._banner.text = mode
            self._banner.color = (1.0, 1.0, 1.0, 1.0)
            self._banner_frames_remaining = self._banner_total

        if self._banner_frames_remaining > 0:
            self._banner_frames_remaining -= 1
            alpha = self._banner_frames_remaining / self._banner_total
            self._banner.color = (1.0, 1.0, 1.0, alpha)


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

    Load one or two sources via ``load()``, then call ``animate()`` to prepare
    the Vispy canvas (headless-safe).  Call ``show()`` to open the interactive
    Qt window.

    Tests should call ``animate()`` only — never ``show()``.
    """

    def __init__(self) -> None:
        self._sources: list[dict] = []
        self._mode_events: ModeEventSeries | None = None
        self._terrain_saturation: float = 1.0
        self._terrain_gltf: object | None = None

    def load(
        self,
        df: pd.DataFrame,
        label: str = "Simulation",
        color: str = "steelblue",
    ) -> None:
        """Add a source DataFrame (call once or twice for dual-source overlay)."""
        self._sources.append({"df": df, "label": label, "color": color})

    def set_mode_events(self, events: ModeEventSeries) -> None:
        """Supply mode transition events for HUD and timeline overlay."""
        self._mode_events = events

    def set_terrain_saturation(self, value: float) -> None:
        """Set terrain colour saturation in the range [0, 1]."""
        self._terrain_saturation = float(value)

    def load_terrain(self, path: str | Path) -> None:
        """Load a terrain glTF/GLB file for scene overlay.

        Raises ``FileNotFoundError`` if ``path`` does not exist.
        """
        import pygltflib

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Terrain file not found: {path}")
        self._terrain_gltf = pygltflib.GLTF2().load(str(path))

    def animate(self, interval_ms: int = 50) -> object:
        """Prepare the Vispy scene and return the SceneCanvas.

        Does not open a window.  Safe to call in headless / offscreen contexts.
        """
        from vispy import scene

        if not self._sources:
            raise RuntimeError("Call load() at least once before animate()")

        canvas = scene.SceneCanvas(size=(1280, 720), show=False, bgcolor="black")
        view = canvas.central_widget.add_view()
        view.camera = "turntable"

        src = self._sources[0]
        df = src["df"]
        positions = _extract_positions(df)
        roll = _extract_attitude(df, "roll")
        pitch = _extract_attitude(df, "pitch")
        heading = _extract_attitude(df, "heading")
        t = df.index.to_numpy(dtype=float)

        # Build ribbon geometry and add to scene
        ribbon = RibbonTrail()
        ribbon.build(positions, roll, heading, pitch, timestamps=t)

        if ribbon.vertices is not None:
            n_quads = len(ribbon.vertices)
            verts = ribbon.vertices.reshape(-1, 3).astype(np.float32)
            idx = np.arange(n_quads, dtype=np.uint32)
            base = 4 * idx
            tri0 = np.stack([base, base + 1, base + 2], axis=1)
            tri1 = np.stack([base, base + 2, base + 3], axis=1)
            faces = np.concatenate([tri0, tri1], axis=0)
            vert_colors = (
                np.repeat(ribbon.colors, 4, axis=0).astype(np.float32)
                if ribbon.colors is not None
                else None
            )
            scene.visuals.Mesh(
                vertices=verts,
                faces=faces,
                vertex_colors=vert_colors,
                parent=view.scene,
            )

        # Trajectory centroid line
        scene.visuals.Line(
            pos=positions.astype(np.float32),
            color=src.get("color", "steelblue"),
            parent=view.scene,
        )

        # Second source overlay (ghost trail)
        if len(self._sources) > 1:
            src2 = self._sources[1]
            df2 = src2["df"]
            pos2 = _extract_positions(df2)
            ribbon2 = RibbonTrail()
            ribbon2.build(
                pos2,
                _extract_attitude(df2, "roll"),
                _extract_attitude(df2, "heading"),
                _extract_attitude(df2, "pitch"),
            )
            scene.visuals.Line(
                pos=pos2.astype(np.float32),
                color=src2.get("color", "orange"),
                parent=view.scene,
            )

        # Auto-scale camera to trajectory extent
        margin = float(max(50.0, 0.05 * np.ptp(positions, axis=0).max()))
        center = positions.mean(axis=0)
        view.camera.set_range(
            x=(positions[:, 0].min() - margin, positions[:, 0].max() + margin),
            y=(positions[:, 1].min() - margin, positions[:, 1].max() + margin),
            z=(positions[:, 2].min() - margin, positions[:, 2].max() + margin),
        )

        return canvas

    def show(self) -> None:
        """Open the animated view in an interactive Qt window.

        Creates a ``QMainWindow`` embedding the Vispy canvas and enters the
        PySide6 event loop.  Do not call from tests.
        """
        from PySide6.QtWidgets import QApplication, QMainWindow
        from vispy import app

        qt_app = QApplication.instance() or QApplication([])
        canvas = self.animate()
        canvas.show()

        win = QMainWindow()
        win.setCentralWidget(canvas.native)
        win.resize(1280, 720)
        win.show()

        qt_app.exec()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _extract_positions(df: pd.DataFrame) -> np.ndarray:
    for cols in _POS_COLUMN_SETS:
        if all(c in df.columns for c in cols):
            return df[cols].to_numpy(dtype=float)
    return np.zeros((len(df), 3))


def _extract_attitude(df: pd.DataFrame, kind: str) -> np.ndarray:
    for col in _ATTITUDE_CANDIDATES[kind]:
        if col in df.columns:
            return df[col].to_numpy(dtype=float)
    return np.zeros(len(df))
