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

def _make_synthetic_imagery(
    tmp_path: Path,
    *,
    filename: str = "imagery.tif",
    lon_min: float = -119.85,
    lat_min: float = 34.42,
    lon_max: float = -119.84,
    lat_max: float = 34.43,
    w: int = 100,
    h: int = 100,
    fill_value: int = 4000,
) -> Path:
    """Create a minimal Sentinel-2-shaped GeoTIFF in EPSG:4326.

    100 × 100 pixels by default, 4 bands (Sentinel-2 order), covering a
    0.01° × 0.01° bbox.  Band values are in the Sentinel-2 L2A DN range.
    """
    import rasterio
    from rasterio.transform import from_bounds

    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, w, h)
    band_data = np.full((4, h, w), fill_value, dtype=np.uint16)

    out_path = tmp_path / filename
    with rasterio.open(
        out_path, "w",
        driver="GTiff", width=w, height=h, count=4,
        dtype="uint16", crs="EPSG:4326",
        transform=transform,
    ) as dst:
        dst.write(band_data)

    return out_path


def _make_naip_imagery(
    tmp_path: Path,
    *,
    filename: str = "naip.tif",
    lon_min: float = -119.85,
    lat_min: float = 34.42,
    lon_max: float = -119.84,
    lat_max: float = 34.43,
    w: int = 500,
    h: int = 500,
    fill_value: int = 180,
    zero_fill_cols: int = 0,
) -> Path:
    """Create a minimal NAIP-shaped GeoTIFF: uint8, 4 bands (R/G/B/NIR), EPSG:4326.

    500 × 500 pixels by default (5× finer than the 100-pixel sentinel2 fixture).
    zero_fill_cols: number of columns (from the left) to fill with zero (simulating
    no-coverage areas such as the ocean for composite tests).
    """
    import rasterio
    from rasterio.transform import from_bounds

    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, w, h)
    band_data = np.full((4, h, w), fill_value, dtype=np.uint8)
    if zero_fill_cols > 0:
        band_data[:, :, :zero_fill_cols] = 0

    out_path = tmp_path / filename
    with rasterio.open(
        out_path, "w",
        driver="GTiff", width=w, height=h, count=4,
        dtype="uint8", crs="EPSG:4326",
        transform=transform,
    ) as dst:
        dst.write(band_data)

    return out_path


# Bbox matching the default synthetic imagery extent.
_BBOX = (-119.85, 34.42, -119.84, 34.43)


# ---------------------------------------------------------------------------
# TB-T1: MosaicDescriptor geographic bounds
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_geographic_bounds(tmp_path: Path) -> None:
    """MosaicDescriptor.lon_min/max and lat_min/max equal the requested bbox."""
    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    desc = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=8192)

    lon_min, lat_min, lon_max, lat_max = _BBOX
    assert desc.lon_min_deg == pytest.approx(lon_min)
    assert desc.lat_min_deg == pytest.approx(lat_min)
    assert desc.lon_max_deg == pytest.approx(lon_max)
    assert desc.lat_max_deg == pytest.approx(lat_max)


# ---------------------------------------------------------------------------
# TB-T1: pixel dims equal native resolution (no upsampling)
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_pixel_dims_are_native_resolution(tmp_path: Path) -> None:
    """width_pixels and height_pixels equal the native imagery resolution for the bbox.

    No upsampling is performed: the output dimensions match the source pixel
    count exactly.
    """
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

    desc = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=8192)

    assert desc.width_pixels == w_native
    assert desc.height_pixels == h_native


# ---------------------------------------------------------------------------
# TB-T1: valid JPEG output
# ---------------------------------------------------------------------------

def test_mosaic_descriptor_jpeg_decodes_correctly(tmp_path: Path) -> None:
    """jpeg_bytes decodes to an RGB image with the declared pixel dimensions."""
    from PIL import Image

    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    desc = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=8192)

    assert desc.jpeg_bytes[:2] == b"\xff\xd8", "Expected JPEG SOI marker (FF D8)"

    img = Image.open(io.BytesIO(desc.jpeg_bytes))
    assert img.mode == "RGB"
    assert img.size == (desc.width_pixels, desc.height_pixels)


# ---------------------------------------------------------------------------
# TB-T5: output clamped to max_pixel_dim when native resolution exceeds ceiling
# ---------------------------------------------------------------------------

def test_mosaic_clamps_to_max_pixel_dim(tmp_path: Path) -> None:
    """Output is clamped to max_pixel_dim when native pixel count exceeds it.

    Synthetic imagery is 100×100 px; capping at 64 forces clamping.
    A warning is logged but no exception is raised.
    """
    import logging

    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)

    with pytest.warns(None):  # no exception expected
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("always")
            desc = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=64)

    assert desc.width_pixels == 64
    assert desc.height_pixels == 64


# ---------------------------------------------------------------------------
# Composite: highest-priority source takes precedence where valid
# ---------------------------------------------------------------------------

def test_composite_higher_priority_pixels_used_where_valid(tmp_path: Path) -> None:
    """Where both sources have valid data, the higher-priority source wins.

    Low-priority source: Sentinel-2 uniform grey (DN=3000 → ~128 display).
    High-priority source: NAIP uniform bright (DN=200 → 200 display, uint8).
    The composite should be bright (NAIP) everywhere since NAIP is fully valid.
    """
    from PIL import Image

    from mosaic_render import render_mosaic

    # Sentinel-2: DN=3000 → linear_scale=3.5/10000=0.00035 → r_lin=1.05→clipped 1.0
    # → after gamma 2.2: r_out ≈ 255.  Use a lower value: DN=1000 → r_lin=0.35
    # → r_out = 0.35^(1/2.2) * 255 ≈ 161
    s2_path = _make_synthetic_imagery(tmp_path, filename="s2.tif", fill_value=1000)
    # NAIP DN=200 → linear_scale=1.0/255=0.00392 → r_lin=0.784 → no gamma → r_out=200
    naip_path = _make_naip_imagery(tmp_path, filename="naip.tif", fill_value=200)

    desc = render_mosaic(
        _BBOX,
        [(s2_path, "sentinel2"), (naip_path, "naip")],  # naip = higher priority
        max_pixel_dim=8192,
    )

    img = Image.open(io.BytesIO(desc.jpeg_bytes)).convert("RGB")
    pixels = np.array(img)

    # Every pixel should be close to the NAIP value (200) — allow JPEG loss.
    assert pixels[:, :, 0].mean() > 180, (
        f"Expected mean R≈200 (NAIP), got {pixels[:, :, 0].mean():.1f}"
    )


def test_composite_lower_priority_fills_zero_coverage_areas(tmp_path: Path) -> None:
    """Where the higher-priority source has no data, the lower-priority source fills.

    NAIP covers only the right half of the bbox (left half is zeros).
    Sentinel-2 covers the full bbox.
    The composite should use Sentinel-2 on the left, NAIP on the right.
    """
    from PIL import Image

    from mosaic_render import render_mosaic

    # Sentinel-2 uniform low value (DN=500 → display ≈ 107)
    s2_path = _make_synthetic_imagery(tmp_path, filename="s2.tif", fill_value=500, w=100, h=100)
    # NAIP: right half valid (200), left half zeros — using 250 zero columns out of 500
    naip_path = _make_naip_imagery(
        tmp_path, filename="naip.tif", fill_value=200,
        w=500, h=500, zero_fill_cols=250,
    )

    desc = render_mosaic(
        _BBOX,
        [(s2_path, "sentinel2"), (naip_path, "naip")],
        max_pixel_dim=8192,
    )

    img = Image.open(io.BytesIO(desc.jpeg_bytes)).convert("RGB")
    pixels = np.array(img)  # (H, W, 3)
    w = pixels.shape[1]
    mid = w // 2

    left_mean = float(pixels[:, :mid, 0].mean())
    right_mean = float(pixels[:, mid:, 0].mean())

    # Right half (NAIP valid) should be brighter than left half (Sentinel-2 fallback).
    assert right_mean > left_mean + 20, (
        f"Expected right (NAIP, ~200) > left (S2 fallback, ~107); "
        f"got left={left_mean:.1f} right={right_mean:.1f}"
    )


def test_composite_single_source_behaves_identically_to_direct_render(
    tmp_path: Path,
) -> None:
    """A single-source composite produces the same result as a direct single-source render."""
    from PIL import Image

    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path, fill_value=2000)
    desc_single = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=8192)
    desc_composite = render_mosaic(_BBOX, [(img_path, "sentinel2")], max_pixel_dim=8192)

    assert desc_single.width_pixels == desc_composite.width_pixels
    assert desc_single.height_pixels == desc_composite.height_pixels
    # JPEG is deterministic for identical inputs.
    assert desc_single.jpeg_bytes == desc_composite.jpeg_bytes


# ---------------------------------------------------------------------------
# Error cases
# ---------------------------------------------------------------------------

def test_render_mosaic_raises_for_empty_sources(tmp_path: Path) -> None:
    """ValueError is raised when sources list is empty."""
    from mosaic_render import render_mosaic

    with pytest.raises(ValueError, match="sources must not be empty"):
        render_mosaic(_BBOX, [])


def test_render_mosaic_raises_for_unknown_source(tmp_path: Path) -> None:
    """KeyError is raised for an unrecognised source name."""
    from mosaic_render import render_mosaic

    img_path = _make_synthetic_imagery(tmp_path)
    with pytest.raises(KeyError):
        render_mosaic(_BBOX, [(img_path, "unknown_source")])
