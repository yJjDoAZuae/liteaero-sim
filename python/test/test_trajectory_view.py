"""Tests for TrajectoryView — trajectory_view.py.

All tests use QT_QPA_PLATFORM=offscreen so no physical display is required.
vispy.use("pyside6") is set before any Vispy/Qt imports.
"""
from __future__ import annotations

import os

# Must be set before any Qt/Vispy imports.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import math
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

import vispy
vispy.use("pyside6")


# Ensure a QApplication exists for the test session.
@pytest.fixture(scope="session", autouse=True)
def qt_app():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def _make_df(n: int = 20) -> pd.DataFrame:
    t = np.linspace(0.0, 10.0, n)
    return pd.DataFrame(
        {
            "pos_north_m": np.linspace(0.0, 100.0, n),
            "pos_east_m": np.zeros(n),
            "pos_down_m": np.zeros(n),
            "altitude_m": 50.0 + np.zeros(n),
            "roll_rad": 0.1 * np.sin(t),
            "pitch_rad": np.zeros(n),
            "heading_rad": np.zeros(n),
            "airspeed_ias_m_s": 30.0 + np.zeros(n),
        },
        index=pd.Index(t, name="time_s"),
    )


@pytest.fixture
def sample_df() -> pd.DataFrame:
    return _make_df()


# ---------------------------------------------------------------------------
# CameraMode
# ---------------------------------------------------------------------------


class TestCameraMode:
    def test_camera_mode_has_fpv(self) -> None:
        from trajectory_view import CameraMode
        assert CameraMode.FPV is not None

    def test_camera_mode_has_trail(self) -> None:
        from trajectory_view import CameraMode
        assert CameraMode.TRAIL is not None

    def test_camera_mode_has_gods_eye(self) -> None:
        from trajectory_view import CameraMode
        assert CameraMode.GODS_EYE is not None

    def test_camera_mode_has_local_top(self) -> None:
        from trajectory_view import CameraMode
        assert CameraMode.LOCAL_TOP is not None


# ---------------------------------------------------------------------------
# TrajectoryView — data API (no rendering)
# ---------------------------------------------------------------------------


class TestTrajectoryViewDataApi:
    def test_instantiation(self) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        assert tv is not None

    def test_load_stores_source(self, sample_df: pd.DataFrame) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        tv.load(sample_df, label="test_source")
        # No exception = pass; internal state verified via animate()

    def test_load_two_sources(self, sample_df: pd.DataFrame) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        tv.load(sample_df, label="source_a")
        tv.load(sample_df, label="source_b")

    def test_set_mode_events(self, sample_df: pd.DataFrame) -> None:
        from mode_overlay import ModeEvent, ModeEventSeries
        from trajectory_view import TrajectoryView
        events = ModeEventSeries([ModeEvent(time_s=2.0, mode_id=1, mode_name="CRUISE")])
        tv = TrajectoryView()
        tv.set_mode_events(events)

    def test_set_terrain_saturation_accepts_float(self) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        tv.set_terrain_saturation(0.5)
        tv.set_terrain_saturation(0.0)
        tv.set_terrain_saturation(1.0)

    def test_load_terrain_raises_for_missing_file(self) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        with pytest.raises(FileNotFoundError):
            tv.load_terrain(Path("/nonexistent/terrain.glb"))

    def test_load_terrain_accepts_glb(self, tmp_path: Path) -> None:
        """A minimal valid GLB file can be loaded without raising."""
        import pygltflib
        from trajectory_view import TrajectoryView

        glb = tmp_path / "terrain.glb"
        gltf = pygltflib.GLTF2()
        gltf.save(str(glb))

        tv = TrajectoryView()
        tv.load_terrain(glb)  # should not raise


# ---------------------------------------------------------------------------
# TrajectoryView — animate headless
# ---------------------------------------------------------------------------


class TestTrajectoryViewAnimate:
    def test_animate_raises_without_load(self) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        with pytest.raises(RuntimeError):
            tv.animate()

    def test_animate_returns_object(self, sample_df: pd.DataFrame) -> None:
        from trajectory_view import TrajectoryView
        tv = TrajectoryView()
        tv.load(sample_df, label="test")
        anim = tv.animate()
        assert anim is not None

    def test_animate_with_mode_events(self, sample_df: pd.DataFrame) -> None:
        from mode_overlay import ModeEvent, ModeEventSeries
        from trajectory_view import TrajectoryView
        events = ModeEventSeries([ModeEvent(time_s=2.0, mode_id=1, mode_name="CRUISE")])
        tv = TrajectoryView()
        tv.load(sample_df)
        tv.set_mode_events(events)
        anim = tv.animate()
        assert anim is not None
