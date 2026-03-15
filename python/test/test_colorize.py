"""Tests for colorize.py — per-facet colorization from imagery raster (Step 17)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rasterio")


def _make_tile_at_origin() -> "TerrainTileData":  # type: ignore[name-defined]
    """Minimal 2-facet tile centered at the equator/prime meridian."""
    from las_terrain import TerrainTileData

    # 4 vertices forming a simple quad split into 2 triangles.
    vertices = np.array(
        [
            [-50.0, -50.0, 0.0],  # SW
            [50.0, -50.0, 0.0],  # SE
            [50.0, 50.0, 0.0],  # NE
            [-50.0, 50.0, 0.0],  # NW
        ],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
    colors = np.full((2, 3), 128, dtype=np.uint8)

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


def _write_rgb_raster(
    path: Path,
    r_val: int,
    g_val: int,
    b_val: int,
    nodata: int | None = None,
    use_nodata_center: bool = False,
) -> None:
    """Write a 5×5 3-band raster (sentinel2-style 16-bit integers) at (0,0)."""
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    # Covers ±0.002° around origin — enough to sample our test tile facet centroids.
    extent = 0.002
    transform = from_bounds(-extent, -extent, extent, extent, 5, 5)

    data = np.zeros((3, 5, 5), dtype=np.uint16)
    data[0] = r_val  # Band 1 → Red (band 4 in sentinel2 is index 4, but rasterio uses 1-index)
    data[1] = g_val  # Band 2 → ...
    data[2] = b_val  # Band 3

    # For sentinel2 we need bands 4, 3, 2 (1-indexed).
    # Write a 4-band raster so band 4 = Red, band 3 = Green, band 2 = Blue.
    full_data = np.zeros((4, 5, 5), dtype=np.uint16)
    full_data[3] = r_val  # Band 4 (Red)
    full_data[2] = g_val  # Band 3 (Green)
    full_data[1] = b_val  # Band 2 (Blue)
    full_data[0] = 9999   # Band 1 (not used)

    if use_nodata_center and nodata is not None:
        full_data[3, 2, 2] = nodata
        full_data[2, 2, 2] = nodata

    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=5,
        width=5,
        count=4,
        dtype="uint16",
        crs=CRS.from_epsg(4326),
        transform=transform,
        nodata=nodata,
    ) as dst:
        dst.write(full_data)


# T1 — constant-value raster → all facets get identical color.
def test_solid_color_raster_all_facets_same_color(tmp_path: Path) -> None:
    from colorize import SCALE_FACTORS, colorize

    tile = _make_tile_at_origin()
    imagery_path = tmp_path / "solid.tif"
    # Sentinel2 scale: 1/10000. A raw value of 5000 → reflectance 0.5 → 127 in 8-bit.
    raw_val = 5000
    _write_rgb_raster(imagery_path, r_val=raw_val, g_val=raw_val, b_val=raw_val)

    result = colorize(tile, imagery_path, source="sentinel2")

    # All facets should have the same color.
    assert result.colors.shape == (len(tile.indices), 3)
    assert np.all(result.colors[0] == result.colors[1]), (
        f"Expected all facets same color, got {result.colors}"
    )
    expected = int(np.clip(raw_val * SCALE_FACTORS["sentinel2"] * 255.0, 0, 255))
    np.testing.assert_array_equal(result.colors[0], [expected, expected, expected])


# T2 — nodata at centroid position → {128, 128, 128}.
def test_cloud_masked_pixel_returns_default_grey(tmp_path: Path) -> None:
    from colorize import colorize

    tile = _make_tile_at_origin()
    imagery_path = tmp_path / "nodata.tif"
    # nodata = 0; write 0 everywhere so all centroids hit nodata.
    _write_rgb_raster(imagery_path, r_val=0, g_val=0, b_val=0, nodata=0)

    result = colorize(tile, imagery_path, source="sentinel2")

    np.testing.assert_array_equal(result.colors[0], [128, 128, 128])
    np.testing.assert_array_equal(result.colors[1], [128, 128, 128])


# T3 — known 16-bit value with known scale factor → expected 8-bit result.
def test_color_scaling_16bit_to_8bit(tmp_path: Path) -> None:
    from colorize import SCALE_FACTORS, colorize

    tile = _make_tile_at_origin()
    imagery_path = tmp_path / "scaling.tif"
    # Raw value 2000 with sentinel2 scale 1/10000 → reflectance 0.2 → 0.2*255 = 51.
    raw_r, raw_g, raw_b = 2000, 3000, 4000
    _write_rgb_raster(imagery_path, r_val=raw_r, g_val=raw_g, b_val=raw_b)

    result = colorize(tile, imagery_path, source="sentinel2")

    scale = SCALE_FACTORS["sentinel2"]
    expected_r = int(np.clip(raw_r * scale * 255, 0, 255))
    expected_g = int(np.clip(raw_g * scale * 255, 0, 255))
    expected_b = int(np.clip(raw_b * scale * 255, 0, 255))

    np.testing.assert_array_equal(result.colors[0], [expected_r, expected_g, expected_b])
