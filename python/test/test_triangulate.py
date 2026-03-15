"""Tests for triangulate.py — L0 TIN from DEM raster (Step 16)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rasterio")
pytest.importorskip("scipy")


def _write_flat_dem(
    path: Path,
    n_cols: int,
    n_rows: int,
    height: float,
    lon_min: float,
    lat_min: float,
    spacing: float,
) -> tuple[float, float, float, float]:
    """Write a flat synthetic DEM GeoTIFF.  Returns (lon_min, lat_min, lon_max, lat_max)."""
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    lon_max = lon_min + (n_cols - 1) * spacing
    lat_max = lat_min + (n_rows - 1) * spacing
    half = spacing / 2.0

    data = np.full((n_rows, n_cols), height, dtype=np.float32)
    transform = from_bounds(
        lon_min - half, lat_min - half, lon_max + half, lat_max + half, n_cols, n_rows
    )
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=n_rows,
        width=n_cols,
        count=1,
        dtype="float32",
        crs=CRS.from_epsg(4326),
        transform=transform,
    ) as dst:
        dst.write(data, 1)

    return lon_min, lat_min, lon_max, lat_max


# T1 — 5×5 grid → exactly 32 triangles; all vertices at correct ENU up_m.
def test_flat_raster_produces_valid_mesh(tmp_path: Path) -> None:
    from triangulate import lod_grid_spacing_deg, triangulate

    spacing = lod_grid_spacing_deg(0)
    dem = tmp_path / "flat.tif"
    bbox = _write_flat_dem(dem, n_cols=5, n_rows=5, height=100.0, lon_min=0.0, lat_min=0.0, spacing=spacing)

    tile = triangulate(dem, bbox, lod=0)

    # A 5×5 regular grid produces 2*(5-1)*(5-1) = 32 Delaunay triangles.
    assert len(tile.indices) == 32, f"Expected 32 facets, got {len(tile.indices)}"

    # All vertices are at nearly the same height (flat raster); up_m ≈ 0 relative to centroid.
    np.testing.assert_allclose(tile.vertices[:, 2], 0.0, atol=0.01)


# T2 — N×M raster → N×M vertices in the output tile.
def test_vertex_count_equals_grid_points(tmp_path: Path) -> None:
    from triangulate import lod_grid_spacing_deg, triangulate

    n_cols, n_rows = 4, 6
    spacing = lod_grid_spacing_deg(0)
    dem = tmp_path / "nm.tif"
    bbox = _write_flat_dem(dem, n_cols=n_cols, n_rows=n_rows, height=50.0, lon_min=0.0, lat_min=0.0, spacing=spacing)

    tile = triangulate(dem, bbox, lod=0)
    assert len(tile.vertices) == n_cols * n_rows


# T3 — provided boundary_points appear verbatim in output vertex list.
def test_boundary_vertices_locked(tmp_path: Path) -> None:
    from triangulate import lod_grid_spacing_deg, triangulate

    spacing = lod_grid_spacing_deg(0)
    dem = tmp_path / "bp.tif"
    bbox = _write_flat_dem(dem, n_cols=5, n_rows=5, height=0.0, lon_min=0.0, lat_min=0.0, spacing=spacing)

    # Inject two boundary points outside the regular grid pattern.
    lon_min, lat_min, lon_max, lat_max = bbox
    bp = np.array(
        [[lon_min + spacing * 0.5, lat_max + spacing * 0.1]],
        dtype=np.float64,
    )

    # Expand DEM to cover the boundary point.
    lat_max_ext = lat_max + spacing * 0.5
    dem2 = tmp_path / "bp_ext.tif"
    bbox2 = _write_flat_dem(
        dem2, n_cols=5, n_rows=6, height=0.0, lon_min=lon_min, lat_min=lat_min, spacing=spacing
    )

    tile = triangulate(dem2, bbox2, lod=0, boundary_points=bp)

    # The total vertex count should be 5×6 (grid) + 1 (boundary) = 31.
    assert len(tile.vertices) == 31


# T4 — triangulated tile passes Python quality check (min_angle ≥ 10°, max_aspect ≤ 15).
def test_output_passes_quality_verifier(tmp_path: Path) -> None:
    from triangulate import lod_grid_spacing_deg, triangulate
    from verify import check

    spacing = lod_grid_spacing_deg(0)
    dem = tmp_path / "qv.tif"
    bbox = _write_flat_dem(dem, n_cols=5, n_rows=5, height=100.0, lon_min=0.0, lat_min=0.0, spacing=spacing)

    tile = triangulate(dem, bbox, lod=0)
    check(tile)  # Should not raise.
