"""grid_overlay.py — executable specification of the viewer conformal-grid coverage math.

The live-sim viewer draws a north/east-aligned reference grid on the terrain as a term in
``godot/addons/liteaero_sim/terrain_grade.gdshader``.  This module is the reference for the
per-fragment grid coverage the shader computes; the GLSL mirrors these functions verbatim so the
math is unit-tested (line placement, physical line width, sub-pixel anti-aliasing) without a GPU.

Coordinates are metres from the terrain-region centroid (the world origin): ``east`` and
``north``.  Lines sit at integer multiples of ``spacing`` on each axis.  ``width_m`` is the
physical (world) line width in metres.  ``aa`` is the screen derivative in metres-per-pixel (the
shader's ``fwidth`` of the world coordinate); it softens the line edge by ~one pixel and gives
sub-pixel lines partial coverage so they dim gracefully at altitude instead of aliasing.

Design authority: docs/design/godot_plugin.md §Conformal Terrain Grid.
"""

from __future__ import annotations

import numpy as np


def line_coverage(coord, spacing: float, width_m: float, aa: float):
    """Coverage in [0, 1] of the nearest grid line to ``coord`` on one axis.

    Analytic 1-D pixel coverage: the fraction of a pixel of size ``aa`` (metres, the ``fwidth`` of
    the world coordinate) that a line of physical half-width ``width_m/2`` occupies at distance
    ``d`` from the pixel centre.  This is robust at grazing angles where ``aa`` becomes large
    (a naive derivative-scaled smoothstep over-widens or produces NaN and fills the screen): a
    line wider than a pixel reads 1, a sub-pixel line dims to ``width_m/aa`` (so it fades cleanly
    at altitude / grazing instead of aliasing).  Accepts a scalar or a NumPy array.
    """
    c = np.asarray(coord, dtype=float)
    g = c / spacing
    f = np.abs((g - 0.5) - np.floor(g - 0.5) - 0.5)   # fract(g-0.5) then distance to line, [0, 0.5]
    d = f * spacing                                    # metres to the nearest line (>= 0)
    half_w = 0.5 * width_m
    # 1-D overlap of the pixel span [d - aa/2, d + aa/2] with the line span [-half_w, half_w],
    # as a fraction of the pixel.
    overlap = np.minimum(half_w, d + 0.5 * aa) - np.maximum(-half_w, d - 0.5 * aa)
    cov = np.clip(overlap / aa, 0.0, 1.0)
    return cov if c.ndim else float(cov)


def grid_coverage(east, north, spacing: float, width_m: float, aa: float):
    """Combined grid coverage = max of the east-line and north-line coverages."""
    cov = np.maximum(
        line_coverage(east, spacing, width_m, aa),
        line_coverage(north, spacing, width_m, aa),
    )
    return cov if (np.ndim(east) or np.ndim(north)) else float(cov)


def minor_line_coverage(coord, minor_spacing: float, major_spacing: float, width_m: float, aa: float):
    """Minor-line coverage on one axis, **suppressed** where the minor line coincides with a major
    line, so a coincident intersection shows only the major line (no blended overlap).

    The viewer derives ``minor_spacing = major_spacing / divisions`` (an integer ratio), so a minor
    line coincides with a major line exactly every ``divisions`` minor lines.  With
    ``major_spacing <= 0`` there is no major grid and nothing is suppressed.
    """
    c = np.asarray(coord, dtype=float)
    cov = np.asarray(line_coverage(c, minor_spacing, width_m, aa), dtype=float)
    if major_spacing and major_spacing > 0.0:
        nearest = np.round(c / minor_spacing) * minor_spacing   # nearest minor line position
        ratio = nearest / major_spacing
        coincident = np.abs(ratio - np.round(ratio)) < 1e-3     # nearest minor line is a major line
        cov = np.where(coincident, 0.0, cov)
    return cov if c.ndim else float(cov)
