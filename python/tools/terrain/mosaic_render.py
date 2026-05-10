"""mosaic_render.py — Render a composite JPEG mosaic texture from one or more imagery sources.

Produces a JPEG byte buffer from an ordered list of imagery GeoTIFFs.  Sources are
ordered lowest-priority to highest-priority.  At each pixel position, the highest-priority
source that has valid (non-zero) data is used; lower-priority sources fill positions where
higher-priority sources have no valid data.  This allows a high-resolution source such as
NAIP (1 m/pixel, CONUS-only) to be composited over a global fallback such as Sentinel-2
(10 m/pixel), producing a single texture with full coverage at the highest achievable
resolution everywhere.

Output pixel count equals the native pixel count of the highest-priority source for the
requested bbox, capped at max_pixel_dim.  No upsampling is performed: if the finest source
is already the highest-resolution available, the output will not exceed its native pixel
density.  Lower-priority sources are bilinearly upsampled to the output grid as needed.

All terrain tiles in a build share this single texture via per-vertex TEXCOORD_0 in the GLB.

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D (TB-T1, TB-T5)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MosaicDescriptor:
    """Output of render_mosaic: JPEG payload and geographic bounds of the rendered texture."""

    jpeg_bytes: bytes
    lon_min_deg: float
    lat_min_deg: float
    lon_max_deg: float
    lat_max_deg: float
    width_pixels: int
    height_pixels: int


# ---------------------------------------------------------------------------
# Imagery-source constants (mirrors colorize.py)
# ---------------------------------------------------------------------------

# DN → linear reflectance [0, 1]
_SCALE_FACTORS: dict[str, float] = {
    "sentinel2": 1.0 / 10000.0,
    "landsat9":  1.0 / 55000.0,
    "modis":     1.0 / 32767.0,
    "naip":      1.0 / 255.0,   # 8-bit uint8, already display-calibrated
}

# Brightness gain applied before gamma correction (natural-colour Sentinel Hub setting)
_DISPLAY_GAIN: dict[str, float] = {
    "sentinel2": 3.5,
    "landsat9":  3.5,
    "modis":     3.5,
    "naip":      1.0,           # already display-calibrated
}

# Display gamma (sRGB standard: 2.2)
_DISPLAY_GAMMA: dict[str, float] = {
    "sentinel2": 2.2,
    "landsat9":  2.2,
    "modis":     2.2,
    "naip":      1.0,           # already gamma-corrected
}

# 1-indexed rasterio band order for (R, G, B)
_BAND_ORDER: dict[str, tuple[int, int, int]] = {
    "sentinel2": (4, 3, 2),  # B04 (Red), B03 (Green), B02 (Blue)
    "landsat9":  (4, 3, 2),  # Band 4, 3, 2
    "modis":     (1, 4, 3),  # Band 1 (Red), Band 4 (Green), Band 3 (Blue)
    "naip":      (1, 2, 3),  # Band 1=R, Band 2=G, Band 3=B (Band 4=NIR, unused)
}


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _encode_jpeg(rgb: np.ndarray, quality: int = 92) -> bytes:
    """Encode an (H, W, 3) uint8 RGB array as JPEG bytes."""
    from PIL import Image
    import io as _io

    img = Image.fromarray(rgb, mode="RGB")
    buf = _io.BytesIO()
    img.save(buf, format="JPEG", quality=quality, subsampling=0)
    return buf.getvalue()


def _native_pixel_count(
    src: "rasterio.DatasetReader",
    lon_min: float,
    lat_min: float,
    lon_max: float,
    lat_max: float,
) -> tuple[int, int]:
    """Return (w_native, h_native) at src's native pixel density for the bbox.

    Handles both EPSG:4326 sources (pixel size in degrees) and projected sources
    such as NAIP UTM (pixel size in metres) by transforming the bbox into the
    source CRS before dividing by the pixel size.
    """
    from rasterio.warp import transform_bounds

    if src.crs and src.crs.to_epsg() != 4326:
        bounds = transform_bounds("EPSG:4326", src.crs, lon_min, lat_min, lon_max, lat_max)
    else:
        bounds = (lon_min, lat_min, lon_max, lat_max)

    span_x = bounds[2] - bounds[0]
    span_y = bounds[3] - bounds[1]
    resx = abs(src.transform.a)
    resy = abs(src.transform.e)
    w = math.ceil(span_x / resx) if resx > 0 else 1
    h = math.ceil(span_y / resy) if resy > 0 else 1
    return max(1, w), max(1, h)


def _read_source_rgb(
    src: "rasterio.DatasetReader",
    r_band: int,
    g_band: int,
    b_band: int,
    lon_min: float,
    lat_min: float,
    lon_max: float,
    lat_max: float,
    h_pixels: int,
    w_pixels: int,
) -> "np.ndarray":
    """Read RGB bands resampled to (h_pixels, w_pixels) regardless of source CRS.

    Handles projected sources (NAIP UTM etc.) by transforming the geographic bbox
    into the source CRS before computing the read window.  Out-of-bounds areas are
    filled with 0 via boundless=True.
    """
    from rasterio.enums import Resampling
    from rasterio.warp import transform_bounds
    from rasterio.windows import from_bounds as _window_from_bounds

    if src.crs and src.crs.to_epsg() != 4326:
        bounds = transform_bounds("EPSG:4326", src.crs, lon_min, lat_min, lon_max, lat_max)
    else:
        bounds = (lon_min, lat_min, lon_max, lat_max)

    window = _window_from_bounds(*bounds, src.transform)
    return src.read(
        [r_band, g_band, b_band],
        window=window,
        out_shape=(3, h_pixels, w_pixels),
        resampling=Resampling.bilinear,
        fill_value=0,
        boundless=True,
    )


def _apply_color_pipeline(
    data: "np.ndarray",
    source: str,
) -> "np.ndarray":
    """Convert raw band data (3, H, W) → (H, W, 3) uint8 RGB via DN→reflectance→gamma."""
    linear_scale = float(_SCALE_FACTORS[source] * _DISPLAY_GAIN[source])
    inv_gamma = 1.0 / _DISPLAY_GAMMA[source]

    r_lin = np.clip(data[0].astype(np.float32) * linear_scale, 0.0, 1.0)
    g_lin = np.clip(data[1].astype(np.float32) * linear_scale, 0.0, 1.0)
    b_lin = np.clip(data[2].astype(np.float32) * linear_scale, 0.0, 1.0)

    r_out = (np.power(r_lin, inv_gamma) * 255.0).astype(np.uint8)
    g_out = (np.power(g_lin, inv_gamma) * 255.0).astype(np.uint8)
    b_out = (np.power(b_lin, inv_gamma) * 255.0).astype(np.uint8)

    return np.stack([r_out, g_out, b_out], axis=-1)  # (H, W, 3)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def render_mosaic(
    bbox_deg: tuple[float, float, float, float],
    sources: list[tuple[Path, str]],
    max_pixel_dim: int = 8192,
) -> MosaicDescriptor:
    """Render a composite JPEG mosaic texture from one or more imagery sources.

    Parameters
    ----------
    bbox_deg:
        (lon_min, lat_min, lon_max, lat_max) in decimal degrees.
    sources:
        Ordered list of (imagery_path, source_name) pairs, from lowest to highest
        priority.  At each pixel position, the highest-priority source with valid
        (non-zero) data is used; lower-priority sources fill positions where
        higher-priority sources have no coverage.  Must not be empty.
    max_pixel_dim:
        Maximum pixel dimension for the output texture.  If the native pixel count
        of the highest-priority source exceeds this on either axis, the output is
        clamped to max_pixel_dim and a warning is logged.  Use 4096 for low-end GPU
        targets, 8192 for desktop GPUs (GTX 1050 and later), 16384 for high-end.

    Returns
    -------
    MosaicDescriptor
        JPEG bytes and the geographic bounds of the rendered texture (equal to bbox_deg).

    Raises
    ------
    ValueError:
        If sources is empty or any source name is not in the supported set.
    KeyError:
        If a source name is not recognised.
    """
    import rasterio

    if not sources:
        raise ValueError("sources must not be empty")

    for _, source in sources:
        if source not in _SCALE_FACTORS:
            raise KeyError(
                f"Unsupported source '{source}'. Choose from: {list(_SCALE_FACTORS)}"
            )

    lon_min, lat_min, lon_max, lat_max = bbox_deg

    # Output resolution is determined by the highest-priority (finest) source.
    highest_path, highest_source = sources[-1]
    with rasterio.open(highest_path) as src:
        w_native, h_native = _native_pixel_count(src, lon_min, lat_min, lon_max, lat_max)

    # Clamp to GPU texture ceiling — finest-source resolution is preserved up to that limit.
    if w_native > max_pixel_dim or h_native > max_pixel_dim:
        _log.warning(
            "Finest source '%s' native resolution %d×%d px exceeds max_pixel_dim=%d; "
            "clamping output — highest-resolution detail will be reduced.",
            highest_source, w_native, h_native, max_pixel_dim,
        )
    w_pixels = min(w_native, max_pixel_dim)
    h_pixels = min(h_native, max_pixel_dim)

    # Build composite: initialise from the lowest-priority source, then overlay
    # each higher-priority source where it has valid (non-zero) pixels.
    composite: np.ndarray | None = None  # (h_pixels, w_pixels, 3) uint8

    for imagery_path, source in sources:
        r_band, g_band, b_band = _BAND_ORDER[source]

        with rasterio.open(imagery_path) as src:
            data = _read_source_rgb(
                src, r_band, g_band, b_band,
                lon_min, lat_min, lon_max, lat_max,
                h_pixels, w_pixels,
            )

        rgb = _apply_color_pipeline(data, source)

        if composite is None:
            composite = rgb
        else:
            # Pixels are valid where any channel is non-zero.  Zero-filled areas
            # (nodata, out-of-coverage) are left as the lower-priority composite.
            valid = np.any(rgb > 0, axis=2)
            composite[valid] = rgb[valid]

    if composite is None:
        composite = np.zeros((h_pixels, w_pixels, 3), dtype=np.uint8)

    jpeg_bytes = _encode_jpeg(composite)

    return MosaicDescriptor(
        jpeg_bytes=jpeg_bytes,
        lon_min_deg=lon_min,
        lat_min_deg=lat_min,
        lon_max_deg=lon_max,
        lat_max_deg=lat_max,
        width_pixels=w_pixels,
        height_pixels=h_pixels,
    )
