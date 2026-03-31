"""Tests for RibbonTrail — trajectory_view.py."""
from __future__ import annotations

import math

import numpy as np
import pytest

from trajectory_view import RibbonTrail


def _straight_positions(n: int) -> np.ndarray:
    """Straight flight along the X-axis at Z=0."""
    return np.column_stack([np.arange(n, dtype=float), np.zeros(n), np.zeros(n)])


class TestRibbonTrail:
    def test_ribbon_wings_level_horizontal(self) -> None:
        """At roll=0, pitch=0, heading=0 the wing vector is purely in the Y direction.

        wing_span_m=2.0 → half-span=1.0 m.
        CCW winding: [v_lower[i], v_lower[i+1], v_upper[i+1], v_upper[i]]
        v_upper at i=0 is at vertex index 3.
        """
        n = 3
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
            wing_span_m=2.0,
        )
        assert ribbon.vertices is not None
        # CCW winding: index 0 = v_lower[i], index 3 = v_upper[i]
        v_upper_0 = ribbon.vertices[0, 3]
        v_lower_0 = ribbon.vertices[0, 0]
        np.testing.assert_allclose(v_upper_0, [0.0, 1.0, 0.0], atol=1e-10)
        np.testing.assert_allclose(v_lower_0, [0.0, -1.0, 0.0], atol=1e-10)

    def test_ribbon_rolled_90_vertical(self) -> None:
        """At roll=+90°, the wing vector is purely vertical (+Z)."""
        n = 3
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.full(n, math.pi / 2),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
            wing_span_m=2.0,
        )
        assert ribbon.vertices is not None
        v_upper_0 = ribbon.vertices[0, 3]  # CCW: v_upper[i] is at index 3
        np.testing.assert_allclose(v_upper_0, [0.0, 0.0, 1.0], atol=1e-10)

    def test_ribbon_vertex_count(self) -> None:
        """N positions produce N-1 quads, each with 4 vertices and 3 coordinates."""
        n = 10
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
        )
        assert ribbon.vertices is not None
        assert ribbon.vertices.shape == (n - 1, 4, 3)

    def test_ribbon_color_at_zero_roll_is_neutral(self) -> None:
        """Roll=0 maps to the neutral center of the RdBu_r colormap (R ≈ B)."""
        n = 3
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
        )
        assert ribbon.colors is not None
        r, g, b, _a = ribbon.colors[0]  # first quad, RGBA
        assert abs(r - b) < 0.1, (
            f"Expected neutral color (R ≈ B) at zero roll, got R={r:.3f} B={b:.3f}"
        )

    def test_mesh_raises_before_build(self) -> None:
        with pytest.raises(RuntimeError):
            RibbonTrail().mesh()

    def test_mesh_returns_after_build(self) -> None:
        """mesh() should return a non-None Vispy Mesh after build() is called."""
        n = 5
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
            wing_span_m=2.0,
        )
        m = ribbon.mesh()
        assert m is not None

    def test_ribbon_timestamps_stored(self) -> None:
        """Segment midpoint timestamps are stored when timestamps are provided."""
        n = 5
        t = np.linspace(0.0, 4.0, n)
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
            wing_span_m=2.0,
            timestamps=t,
        )
        assert ribbon.timestamps is not None
        assert len(ribbon.timestamps) == n - 1  # one midpoint per segment

    def test_ribbon_alpha_recent_segments_more_opaque(self) -> None:
        """Segments closer to t_current have higher alpha than older segments."""
        n = 5
        t = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
            wing_span_m=2.0,
            timestamps=t,
            trail_duration_s=4.0,
        )
        alphas = ribbon.alpha_at(t_current=4.0)
        assert alphas[-1] > alphas[0], "Most recent segment should have higher alpha"
        assert np.all(alphas >= 0.0)
        assert np.all(alphas <= 1.0)

    def test_ribbon_alpha_none_without_timestamps(self) -> None:
        """alpha_at() returns None when no timestamps were provided at build."""
        n = 5
        ribbon = RibbonTrail()
        ribbon.build(
            positions=_straight_positions(n),
            roll_rad=np.zeros(n),
            heading_rad=np.zeros(n),
            pitch_rad=np.zeros(n),
        )
        assert ribbon.alpha_at(t_current=2.0) is None
