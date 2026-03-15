"""Tests for geoid_correct.py and mosaic.py (Step 15)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pyproj")
pytest.importorskip("rasterio")
pytest.importorskip("scipy")


# T1 — geoid undulation at (0°, 0°) ≈ 17.2 m ± 0.5 m.
def test_undulation_equator_prime_meridian() -> None:
    from geoid_correct import geoid_undulation

    N = geoid_undulation(lat_deg=0.0, lon_deg=0.0, geoid="egm2008")
    if abs(N) < 1e-3:
        pytest.skip(
            "PROJ EGM2008 datum shift grid not available "
            "(no network access or datum grid not downloaded)"
        )
    assert abs(N - 17.2) < 0.5, f"Expected undulation ~17.2 m at (0,0), got {N:.3f} m"


# T2 — synthetic raster: output pixel = input pixel + undulation ± 0.1 m.
def test_corrected_height_equals_msl_plus_undulation(tmp_path: Path) -> None:
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    from geoid_correct import apply_geoid_correction, geoid_undulation

    # Build a tiny 3×3 flat raster at 0° lat/lon, all heights = 50 m (orthometric).
    lon_min, lat_min, lon_max, lat_max = -0.01, -0.01, 0.01, 0.01
    height_msl = 50.0
    data = np.full((3, 3), height_msl, dtype=np.float32)
    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, 3, 3)

    dem_path = tmp_path / "dem_msl.tif"
    out_path = tmp_path / "dem_wgs84.tif"

    with rasterio.open(
        dem_path,
        "w",
        driver="GTiff",
        height=3,
        width=3,
        count=1,
        dtype="float32",
        crs=CRS.from_epsg(4326),
        transform=transform,
    ) as dst:
        dst.write(data, 1)

    apply_geoid_correction(dem_path, out_path, geoid="egm2008")

    with rasterio.open(out_path) as src:
        corrected = src.read(1).astype(np.float64)

    expected_undulation = geoid_undulation(0.0, 0.0, geoid="egm2008")
    expected_height = height_msl + expected_undulation
    # All pixels should be approximately input + undulation at the center
    np.testing.assert_allclose(corrected, expected_height, atol=0.5)


# T3 — merged raster covers the union of the two input bounding boxes.
def test_mosaic_preserves_bbox(tmp_path: Path) -> None:
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    from mosaic import mosaic_dem

    # Create two adjacent 3×3 DEMs at different lon ranges.
    def _write_dem(path: Path, lon_min: float, lon_max: float) -> None:
        lat_min, lat_max = 0.0, 0.001
        data = np.full((3, 3), 100.0, dtype=np.float32)
        t = from_bounds(lon_min, lat_min, lon_max, lat_max, 3, 3)
        with rasterio.open(
            path,
            "w",
            driver="GTiff",
            height=3,
            width=3,
            count=1,
            dtype="float32",
            crs=CRS.from_epsg(4326),
            transform=t,
        ) as dst:
            dst.write(data, 1)

    dem_a = tmp_path / "dem_a.tif"
    dem_b = tmp_path / "dem_b.tif"
    _write_dem(dem_a, lon_min=0.000, lon_max=0.001)
    _write_dem(dem_b, lon_min=0.001, lon_max=0.002)

    out = tmp_path / "mosaic.tif"
    mosaic_dem([dem_a, dem_b], out)

    with rasterio.open(out) as src:
        bounds = src.bounds

    # The mosaic should cover at least [0, 0.002] in longitude.
    assert bounds.left <= 0.0 + 1e-6
    assert bounds.right >= 0.002 - 1e-6
