"""Tests for simplify.py — LOD simplification (Step 18)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pyfqmr")
pytest.importorskip("scipy")


def _make_dense_tile(n: int = 10) -> "TerrainTileData":  # type: ignore[name-defined]
    """Build a flat n×n regular-grid tile at LOD 0 for simplification testing."""
    from las_terrain import TerrainTileData
    from scipy.spatial import Delaunay

    spacing = 10.0  # 10 m between vertices
    xs = np.arange(n) * spacing
    ys = np.arange(n) * spacing
    xg, yg = np.meshgrid(xs, ys)
    east = xg.ravel().astype(np.float32)
    north = yg.ravel().astype(np.float32)
    up = np.zeros_like(east)
    vertices = np.column_stack([east, north, up]).astype(np.float32)

    pts2d = np.column_stack([east, north])
    tri = Delaunay(pts2d)
    indices = tri.simplices.astype(np.uint32)

    # Assign a non-grey color to half the facets.
    colors = np.full((len(indices), 3), 128, dtype=np.uint8)
    colors[: len(indices) // 2] = [200, 100, 50]

    return TerrainTileData(
        lod=0,
        centroid_lat_rad=0.0,
        centroid_lon_rad=0.0,
        centroid_height_m=0.0,
        lat_min_rad=-0.001,
        lat_max_rad=0.001,
        lon_min_rad=-0.001,
        lon_max_rad=0.001,
        height_min_m=0.0,
        height_max_m=0.0,
        vertices=vertices,
        indices=indices,
        colors=colors,
    )


# T1 — L0 → L1: output face count < input face count.
def test_simplify_reduces_face_count() -> None:
    from simplify import simplify

    tile = _make_dense_tile(n=12)
    result = simplify(tile, target_lod=1)
    assert len(result.indices) < len(tile.indices), (
        f"Expected fewer faces after simplification; "
        f"got {len(result.indices)} from {len(tile.indices)}"
    )
    assert result.lod == 1


# T2 — all boundary vertices of L0 appear in L1 output (preserve_border=True).
def test_boundary_vertices_preserved() -> None:
    from simplify import simplify

    tile = _make_dense_tile(n=12)
    result = simplify(tile, target_lod=1)

    # Identify boundary vertices in the input (vertices on convex hull edge).
    src_east = tile.vertices[:, 0]
    src_north = tile.vertices[:, 1]
    e_min, e_max = float(src_east.min()), float(src_east.max())
    n_min, n_max = float(src_north.min()), float(src_north.max())

    tol = 0.1  # metres
    boundary_mask = (
        (src_east <= e_min + tol)
        | (src_east >= e_max - tol)
        | (src_north <= n_min + tol)
        | (src_north >= n_max - tol)
    )
    boundary_verts = tile.vertices[boundary_mask]

    # Every input boundary vertex should appear in the simplified output (within tolerance).
    dst_east = result.vertices[:, 0]
    dst_north = result.vertices[:, 1]
    for v in boundary_verts:
        dist = np.min(np.sqrt((dst_east - v[0]) ** 2 + (dst_north - v[1]) ** 2))
        assert dist < 1.0, (
            f"Boundary vertex {v[:2]} not found in simplified output (nearest dist={dist:.3f} m)"
        )


# T3 — non-grey color in L0 is present in L1 facets after color transfer.
def test_color_transferred_to_simplified_mesh() -> None:
    from simplify import simplify

    tile = _make_dense_tile(n=12)
    result = simplify(tile, target_lod=1)

    # At least some facets should have the non-grey color [200, 100, 50].
    has_orange = np.any(np.all(result.colors == [200, 100, 50], axis=1))
    assert has_orange, "Expected non-grey color to survive simplification"


# T4 — tile with zero-area facets raises MeshQualityError.
def test_degenerate_input_raises() -> None:
    from las_terrain import TerrainTileData
    from simplify import simplify
    from verify import MeshQualityError

    # Build a tile where one facet is degenerate (collinear vertices).
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],  # collinear with v0 and v1 → zero area
            [10.0, 10.0, 0.0],
        ],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2], [0, 1, 3]], dtype=np.uint32)
    colors = np.full((2, 3), 128, dtype=np.uint8)

    tile = TerrainTileData(
        lod=0,
        centroid_lat_rad=0.0,
        centroid_lon_rad=0.0,
        centroid_height_m=0.0,
        lat_min_rad=-0.001,
        lat_max_rad=0.001,
        lon_min_rad=-0.001,
        lon_max_rad=0.001,
        height_min_m=0.0,
        height_max_m=0.0,
        vertices=vertices,
        indices=indices,
        colors=colors,
    )

    with pytest.raises(MeshQualityError):
        simplify(tile, target_lod=1)
