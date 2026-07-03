"""Tests for raster_sample.py — in-memory vectorized raster sampling.

The RasterSampler reads a raster once and samples arbitrary lon/lat points by
flooring to the containing pixel.  Its contract is to be *bit-identical* to
rasterio.DatasetReader.sample() while eliminating per-point I/O.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rasterio")


def _write_ramp_dem(path: Path, n: int = 20) -> tuple[float, float, float, float]:
    """Write an n×n single-band GeoTIFF whose values are a distinct per-pixel ramp.

    Returns the (lon_min, lat_min, lon_max, lat_max) of the pixel *centers*.
    """
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    spacing = 0.001
    lon_min, lat_min = -1.0, 40.0
    lon_max = lon_min + (n - 1) * spacing
    lat_max = lat_min + (n - 1) * spacing
    half = spacing / 2.0

    # Distinct value per pixel so any mis-indexing is detectable.
    data = np.arange(n * n, dtype=np.float32).reshape(n, n)
    transform = from_bounds(
        lon_min - half, lat_min - half, lon_max + half, lat_max + half, n, n
    )
    with rasterio.open(
        path, "w", driver="GTiff", height=n, width=n, count=1,
        dtype="float32", crs=CRS.from_epsg(4326), transform=transform,
        nodata=-9999.0,
    ) as dst:
        dst.write(data, 1)
    return lon_min, lat_min, lon_max, lat_max


def _write_multiband(path: Path, n: int = 12, bands: int = 4) -> tuple[float, float, float, float]:
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    spacing = 0.002
    lon_min, lat_min = 10.0, -5.0
    lon_max = lon_min + (n - 1) * spacing
    lat_max = lat_min + (n - 1) * spacing
    half = spacing / 2.0
    data = np.stack(
        [np.arange(n * n, dtype=np.uint16).reshape(n, n) + b * 1000 for b in range(bands)]
    )
    transform = from_bounds(
        lon_min - half, lat_min - half, lon_max + half, lat_max + half, n, n
    )
    with rasterio.open(
        path, "w", driver="GTiff", height=n, width=n, count=bands,
        dtype="uint16", crs=CRS.from_epsg(4326), transform=transform,
    ) as dst:
        dst.write(data)
    return lon_min, lat_min, lon_max, lat_max


def test_sample_matches_rasterio_single_band(tmp_path: Path) -> None:
    import rasterio
    from raster_sample import RasterSampler

    dem = tmp_path / "ramp.tif"
    lon_min, lat_min, lon_max, lat_max = _write_ramp_dem(dem, n=20)

    rng = np.random.default_rng(0)
    lons = rng.uniform(lon_min, lon_max, 500)
    lats = rng.uniform(lat_min, lat_max, 500)

    sampler = RasterSampler.from_path(dem)
    got = sampler.sample(lons, lats)

    with rasterio.open(dem) as src:
        ref = np.array([v[0] for v in src.sample(list(zip(lons.tolist(), lats.tolist())))])

    np.testing.assert_array_equal(got, ref)


def test_sample_matches_rasterio_multiband(tmp_path: Path) -> None:
    import rasterio
    from raster_sample import RasterSampler

    img = tmp_path / "img.tif"
    lon_min, lat_min, lon_max, lat_max = _write_multiband(img, n=12, bands=4)

    rng = np.random.default_rng(1)
    lons = rng.uniform(lon_min, lon_max, 200)
    lats = rng.uniform(lat_min, lat_max, 200)

    sampler = RasterSampler.from_path(img, multiband=True)
    got = sampler.sample(lons, lats)  # (B, N)
    assert got.shape == (4, 200)

    with rasterio.open(img) as src:
        ref = np.array(
            list(src.sample(list(zip(lons.tolist(), lats.tolist()))))
        ).T  # rasterio yields (N, B) → transpose to (B, N)

    np.testing.assert_array_equal(got, ref)


def test_bounds_and_nodata_exposed(tmp_path: Path) -> None:
    from raster_sample import RasterSampler

    dem = tmp_path / "ramp.tif"
    _write_ramp_dem(dem, n=10)
    sampler = RasterSampler.from_path(dem)

    assert sampler.nodata == -9999.0
    left, bottom, right, top = sampler.bounds
    assert left < right and bottom < top
    assert sampler.width == 10 and sampler.height == 10


def test_out_of_bounds_points_clip_to_edge(tmp_path: Path) -> None:
    """Points outside the raster clamp to the nearest edge pixel (no exception)."""
    from raster_sample import RasterSampler

    dem = tmp_path / "ramp.tif"
    lon_min, lat_min, lon_max, lat_max = _write_ramp_dem(dem, n=10)
    sampler = RasterSampler.from_path(dem)

    far = sampler.sample(np.array([lon_max + 5.0]), np.array([lat_min - 5.0]))
    assert np.isfinite(far).all()
