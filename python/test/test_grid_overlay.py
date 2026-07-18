"""Tests for grid_overlay.py — executable spec for the viewer conformal-grid coverage math.

This math is mirrored verbatim by `godot/addons/liteaero_sim/terrain_grade.gdshader`.
Coverage is a per-fragment quantity in [0, 1]: 1 on a grid line, 0 away from any line.
`aa` is the metres-per-pixel screen derivative (the shader's `fwidth`), passed explicitly
here so the physical line width and sub-pixel behaviour are testable without a GPU.
"""

from __future__ import annotations

import numpy as np
import pytest


def test_on_line_full_coverage() -> None:
    from grid_overlay import line_coverage

    spacing, width_m, aa = 1000.0, 50.0, 1.0
    for k in (-2, -1, 0, 1, 2, 5):
        cov = line_coverage(k * spacing, spacing, width_m, aa)
        assert cov == pytest.approx(1.0, abs=1e-6), f"k={k}"


def test_midway_zero_coverage() -> None:
    from grid_overlay import line_coverage

    spacing, width_m, aa = 1000.0, 50.0, 1.0
    for k in (-1, 0, 1, 3):
        cov = line_coverage((k + 0.5) * spacing, spacing, width_m, aa)
        assert cov == pytest.approx(0.0, abs=1e-6), f"k={k}"


def test_physical_width_edges() -> None:
    from grid_overlay import line_coverage

    spacing, aa = 1000.0, 1.0
    # A line sits at c=0; the half-coverage edge is at |c| = width_m/2.
    for width_m in (20.0, 50.0, 120.0):
        hw = 0.5 * width_m
        assert line_coverage(hw, spacing, width_m, aa) == pytest.approx(0.5, abs=1e-3)
        assert line_coverage(hw - 3 * aa, spacing, width_m, aa) > 0.98  # just inside
        assert line_coverage(hw + 3 * aa, spacing, width_m, aa) < 0.02  # just outside


def test_subpixel_line_dims() -> None:
    from grid_overlay import line_coverage

    spacing, aa = 1000.0, 2.0
    # On the line, a sub-pixel-wide line (width_m < aa) reads < 1 (partial coverage), rising
    # monotonically with width_m, and equals width_m/aa exactly in the sub-pixel regime.
    covs = [line_coverage(0.0, spacing, w, aa) for w in (0.5, 1.0, 1.5)]
    assert covs[0] < 1.0
    assert all(a < b for a, b in zip(covs, covs[1:]))
    assert line_coverage(0.0, spacing, 1.0, aa) == pytest.approx(1.0 / aa, abs=1e-6)  # width/aa
    # A line much wider than a pixel is fully covered.
    assert line_coverage(0.0, spacing, 20.0, aa) == pytest.approx(1.0, abs=1e-6)


def test_grid_is_axis_max() -> None:
    from grid_overlay import grid_coverage, line_coverage

    spacing, width_m, aa = 1000.0, 50.0, 1.0
    # On an east line (east=0) but far from any north line (north at midway): grid reads full.
    east, north = 0.0, 0.5 * spacing
    assert line_coverage(north, spacing, width_m, aa) == pytest.approx(0.0, abs=1e-6)
    assert grid_coverage(east, north, spacing, width_m, aa) == pytest.approx(1.0, abs=1e-6)
    # Away from both → 0.
    assert grid_coverage(0.5 * spacing, 0.5 * spacing, spacing, width_m, aa) == pytest.approx(0.0, abs=1e-6)


def test_spacing_scales() -> None:
    from grid_overlay import line_coverage

    width_m, aa = 20.0, 1.0
    # With spacing 500, c=500 is a line and c=250 is midway.
    assert line_coverage(500.0, 500.0, width_m, aa) == pytest.approx(1.0, abs=1e-6)
    assert line_coverage(250.0, 500.0, width_m, aa) == pytest.approx(0.0, abs=1e-6)
    # With spacing 1000, that same c=500 is now midway (not a line).
    assert line_coverage(500.0, 1000.0, width_m, aa) == pytest.approx(0.0, abs=1e-6)


def test_vectorized() -> None:
    from grid_overlay import line_coverage

    spacing, width_m, aa = 1000.0, 50.0, 1.0
    c = np.array([0.0, 500.0, 1000.0, 1500.0, 2000.0])
    cov = line_coverage(c, spacing, width_m, aa)
    np.testing.assert_allclose(cov, [1.0, 0.0, 1.0, 0.0, 1.0], atol=1e-6)
