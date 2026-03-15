"""Tests for verify.py — Python mesh quality verification (Step 19)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from las_terrain import TerrainTileData


def _make_tile(
    vertices: np.ndarray,
    indices: np.ndarray,
    clat_rad: float = 0.0,
    clon_rad: float = 0.0,
    ch_m: float = 0.0,
) -> TerrainTileData:
    colors = np.full((len(indices), 3), 128, dtype=np.uint8)
    return TerrainTileData(
        lod=0,
        centroid_lat_rad=clat_rad,
        centroid_lon_rad=clon_rad,
        centroid_height_m=ch_m,
        lat_min_rad=-0.001,
        lat_max_rad=0.001,
        lon_min_rad=-0.001,
        lon_max_rad=0.001,
        height_min_m=-10.0,
        height_max_m=10.0,
        vertices=vertices,
        indices=indices,
        colors=colors,
    )


def _equilateral_tile(side_m: float = 100.0) -> TerrainTileData:
    """Single equilateral triangle in the ENU east-north plane."""
    h = side_m * math.sqrt(3.0) / 2.0
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [side_m, 0.0, 0.0],
            [side_m / 2.0, h, 0.0],
        ],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2]], dtype=np.uint32)
    return _make_tile(vertices, indices)


# T1 — equilateral triangle: min_interior_angle ≈ 60 ± 0.1°; check() does not raise.
def test_equilateral_mesh_passes() -> None:
    from verify import check, verify

    tile = _equilateral_tile(side_m=1000.0)
    report = verify(tile)

    assert abs(report.min_interior_angle_deg - 60.0) < 0.1, (
        f"Expected min_angle ≈ 60°, got {report.min_interior_angle_deg:.3f}°"
    )
    check(tile)  # Should not raise.


# T2 — collinear vertices → degenerate_facet_count == 1; check() raises MeshQualityError.
def test_degenerate_facet_detected() -> None:
    from verify import MeshQualityError, check, verify

    # Three collinear points → zero-area triangle.
    vertices = np.array(
        [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2]], dtype=np.uint32)
    tile = _make_tile(vertices, indices)

    report = verify(tile)
    assert report.degenerate_facet_count == 1, (
        f"Expected 1 degenerate facet, got {report.degenerate_facet_count}"
    )

    with pytest.raises(MeshQualityError):
        check(tile)


# T3 — single triangle tile → open_boundary_edge_count == 3.
def test_open_boundary_edges() -> None:
    from verify import verify

    tile = _equilateral_tile(side_m=500.0)
    report = verify(tile)

    assert report.open_boundary_edge_count == 3, (
        f"Single triangle should have 3 open boundary edges, got {report.open_boundary_edge_count}"
    )


# T4 — very thin triangle → max_aspect_ratio > 15; check() raises MeshQualityError.
def test_thin_triangle_fails_aspect_ratio() -> None:
    from verify import MeshQualityError, check, verify

    # Base = 1000 m, height = 1 m → very thin triangle.
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 1.0, 0.0]],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2]], dtype=np.uint32)
    tile = _make_tile(vertices, indices)

    report = verify(tile)
    assert report.max_aspect_ratio > 15.0, (
        f"Expected max_aspect_ratio > 15, got {report.max_aspect_ratio:.2f}"
    )

    with pytest.raises(MeshQualityError):
        check(tile)
