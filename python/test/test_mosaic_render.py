"""Tests for mosaic_render.py — JPEG mosaic texture renderer (TB-T1, TB-T5).

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D
"""

from __future__ import annotations

import io
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rasterio")
pytest.importorskip("PIL")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_synthetic_imagery(tmp_path: Path) -> Path:
    """Create a minimal Sentinel-2-shaped GeoTIFF for mosaic_render tests.

    100 × 100 pixels, 4 bands (Sentinel-2 order), covering a 0.01° × 0.01° bbox.
    Band values are in the Sentinel-2 L2A DN range (0–10000).
    """
    import rasterio
    from rasterio.transform import from_bounds

    W, H = 100, 100
    lon_min, lat_min, lon_max, lat_max = -119.85, 34.42, -119.84, 34.43
    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, W, H)

    # 4-band array in rasterio band order (1-indexed):
    #   Band 1: unused (0)
    #   Band 2: Blue data  (Sentinel-2 B02)
    #   Band 3: Green data (Sentinel-2 B03)
    #   Band 4: Red data   (Sentinel-2 B04)
    band_data = np.zeros((4, H, W), dtype=np.uint16)
    band_data[3] = np.tile(np.linspace(2000, 5000, W, dtype=np.uint16), (H, 1))   # Red
    band_data[2] = np.tile(np.linspace(3000, 6000, H, dtype=np.uint16).reshape(-1, 1), (1, W))  # Green
    band_data[1] = 4000  # Blue (uniform)
    band_data[0] = 0

    out_path = tmp_path / "imagery.tif"
    with rasterio.open(
        out_path, "w",
        driver="GTiff", width=W, height=H, count=4,
        dtype="uint16", crs="EPSG:4326",
        transform=transform,
    ) as dst:
        dst.write(band_data)

    return out_path


# Bbox matching the synthetic imagery extent.
_BBOX = (-119.85, 34.42, -119.84, 34.43)


# ---------------------------------------------------------------------------
# TB-T1: MosaicDescriptor geographic bounds
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_geographic_bounds(tmp_path: Path) -> None:
    """MosaicDescriptor.lon_min/max and lat_min/max equal the requested bbox."""
    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    desc = render_mosaic(_BBOX, img_path, source="sentinel2", max_pixel_dim=8192)

    lon_min, lat_min, lon_max, lat_max = _BBOX
    assert desc.lon_min_deg == pytest.approx(lon_min)
    assert desc.lat_min_deg == pytest.approx(lat_min)
    assert desc.lon_max_deg == pytest.approx(lon_max)
    assert desc.lat_max_deg == pytest.approx(lat_max)


# ---------------------------------------------------------------------------
# TB-T1: power-of-two pixel dimensions
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_pixel_dims_power_of_two(tmp_path: Path) -> None:
    """width_pixels and height_pixels are powers of two."""
    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    desc = render_mosaic(_BBOX, img_path, source="sentinel2", max_pixel_dim=8192)

    assert desc.width_pixels > 0
    assert desc.height_pixels > 0
    assert (desc.width_pixels & (desc.width_pixels - 1)) == 0, \
        f"width_pixels {desc.width_pixels} is not a power of two"
    assert (desc.height_pixels & (desc.height_pixels - 1)) == 0, \
        f"height_pixels {desc.height_pixels} is not a power of two"


# ---------------------------------------------------------------------------
# TB-T1: pixel dims >= native resolution
# ---------------------------------------------------------------------------

def test_mosaic_pixel_dims_at_least_native_resolution(tmp_path: Path) -> None:
    """PoT pixel dimensions are >= the native imagery resolution for the bbox."""
    import math

    import rasterio
    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    lon_min, lat_min, lon_max, lat_max = _BBOX

    with rasterio.open(img_path) as src:
        resx = abs(src.transform.a)
        resy = abs(src.transform.e)

    w_native = math.ceil((lon_max - lon_min) / resx)
    h_native = math.ceil((lat_max - lat_min) / resy)

    desc = render_mosaic(_BBOX, img_path, source="sentinel2", max_pixel_dim=8192)

    assert desc.width_pixels >= w_native
    assert desc.height_pixels >= h_native


# ---------------------------------------------------------------------------
# TB-T1: valid JPEG output
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_jpeg_decodes_correctly(tmp_path: Path) -> None:
    """jpeg_bytes decodes to an RGB image with the declared pixel dimensions."""
    from PIL import Image

    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    desc = render_mosaic(_BBOX, img_path, source="sentinel2", max_pixel_dim=8192)

    assert desc.jpeg_bytes[:2] == b"\xff\xd8", "Expected JPEG SOI marker (FF D8)"

    img = Image.open(io.BytesIO(desc.jpeg_bytes))
    assert img.mode == "RGB"
    assert img.size == (desc.width_pixels, desc.height_pixels)


# ---------------------------------------------------------------------------
# TB-T5: MosaicTooLargeError raised when PoT exceeds ceiling
# ---------------------------------------------------------------------------

def test_mosaic_too_large_error_raised(tmp_path: Path) -> None:
    """MosaicTooLargeError is raised when the PoT resolution exceeds max_pixel_dim.

    Synthetic imagery is 100×100 px; native PoT = 128×128.
    Capping at 64 forces the error.
    """
    from mosaic_render import MosaicTooLargeError, render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)

    with pytest.raises(MosaicTooLargeError):
        render_mosaic(_BBOX, img_path, source="sentinel2", max_pixel_dim=64)
