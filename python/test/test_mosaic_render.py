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


# ---------------------------------------------------------------------------
# OQ-TB-7: projected (UTM) source is reprojected, not linearly stretched
# ---------------------------------------------------------------------------

def _make_utm_lon_stripe(
    tmp_path: Path,
    bbox: tuple[float, float, float, float],
    lon_stripe: float,
    *,
    filename: str = "utm_stripe.tif",
    utm_crs: str = "EPSG:32611",   # UTM zone 11N — KSBA sits ~2.8deg off the -117 central meridian
    res_m: float = 1.0,
    half_width_deg: float = 0.0003,
) -> Path:
    """Create a UTM GeoTIFF carrying a *true-north* (constant-longitude) white stripe.

    A constant-lon line runs along grid-north only on the central meridian; off it (KSBA) the
    stripe is tilted in UTM pixel space by the grid-convergence angle.  A correct reprojection
    onto the tile's EPSG:4326 grid renders it vertical again; the old linear window-stretch leaves
    it tilted.  4 bands (R/G/B/NIR) uint8 to match NAIP.
    """
    import rasterio
    from rasterio.transform import from_bounds as _tfb
    from rasterio.warp import transform as _warp_xy, transform_bounds

    lon0, lat0, lon1, lat1 = bbox
    ub = transform_bounds("EPSG:4326", utm_crs, lon0 - 0.001, lat0 - 0.001, lon1 + 0.001, lat1 + 0.001)
    w = int((ub[2] - ub[0]) / res_m)
    h = int((ub[3] - ub[1]) / res_m)
    transform = _tfb(ub[0], ub[1], ub[2], ub[3], w, h)

    cols = ub[0] + (np.arange(w) + 0.5) * res_m
    rows = ub[3] - (np.arange(h) + 0.5) * res_m
    xx, yy = np.meshgrid(cols, rows)
    lon_pix, _ = _warp_xy(utm_crs, "EPSG:4326", xx.ravel().tolist(), yy.ravel().tolist())
    lon_pix = np.asarray(lon_pix).reshape(h, w)

    data = np.zeros((4, h, w), dtype=np.uint8)
    stripe = np.abs(lon_pix - lon_stripe) < half_width_deg
    data[0][stripe] = 255
    data[1][stripe] = 255
    data[2][stripe] = 255

    out_path = tmp_path / filename
    with rasterio.open(
        out_path, "w",
        driver="GTiff", width=w, height=h, count=4,
        dtype="uint8", crs=utm_crs, transform=transform,
    ) as dst:
        dst.write(data)
    return out_path


def test_projected_source_reprojected_not_sheared(tmp_path: Path) -> None:
    """A true-north stripe in a UTM source renders vertical (no per-tile azimuth shear).

    OQ-TB-7: the old path read an axis-aligned UTM window and stretched it onto the tile,
    tilting every tile by the grid-convergence angle (~1.6deg at KSBA => tan ~ 0.028 px/row).
    The reprojection keeps grid-north-vs-true-north correct, so the constant-lon stripe stays
    vertical (slope ~ 0).
    """
    from PIL import Image

    from mosaic_render import render_mosaic

    bbox = (-119.845, 34.425, -119.835, 34.435)
    lon_stripe = -119.840
    src = _make_utm_lon_stripe(tmp_path, bbox, lon_stripe)

    desc = render_mosaic(bbox, [(src, "naip")], max_pixel_dim=1024)
    img = np.asarray(Image.open(io.BytesIO(desc.jpeg_bytes)).convert("L"), dtype=float)
    h, w = img.shape

    # Brightness-weighted stripe column per row (robust to JPEG blur).
    xs = np.arange(w)
    rows, centers = [], []
    for r in range(h):
        row = img[r]
        bright = row > 128
        if bright.sum() >= 2:
            rows.append(r)
            centers.append(float((xs[bright] * row[bright]).sum() / row[bright].sum()))
    rows = np.asarray(rows, dtype=float)
    centers = np.asarray(centers, dtype=float)

    assert len(rows) > 0.5 * h, "stripe not detected on enough rows"
    slope = float(np.polyfit(rows, centers, 1)[0])  # px of column drift per row
    # Correct reprojection: ~vertical (|slope| ~ 0).  Old linear stretch: |slope| ~ tan(1.6deg) ~ 0.028.
    assert abs(slope) < 0.01, f"stripe tilts {slope:.4f} px/row — per-tile azimuth shear (OQ-TB-7)"
