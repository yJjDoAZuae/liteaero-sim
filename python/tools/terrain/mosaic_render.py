"""mosaic_render.py — Render a single JPEG mosaic texture from a geographic bbox.

Produces a JPEG byte buffer at imagery-native pixel density rounded up to the
nearest power-of-two dimension and capped at max_pixel_dim.  All terrain tiles in
a build share this single texture via per-vertex TEXCOORD_0 in the GLB.

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D (TB-T1, TB-T5)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------

class MosaicTooLargeError(Exception):
    """Raised when the power-of-two pixel resolution exceeds the GPU texture ceiling."""


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
}

# Brightness gain applied before gamma correction (natural-colour Sentinel Hub setting)
_DISPLAY_GAIN: dict[str, float] = {
    "sentinel2": 3.5,
    "landsat9":  3.5,
    "modis":     3.5,
}

# Display gamma (sRGB standard: 2.2)
_DISPLAY_GAMMA: dict[str, float] = {
    "sentinel2": 2.2,
    "landsat9":  2.2,
    "modis":     2.2,
}

# 1-indexed rasterio band order for (R, G, B)
_BAND_ORDER: dict[str, tuple[int, int, int]] = {
    "sentinel2": (4, 3, 2),  # B04 (Red), B03 (Green), B02 (Blue)
    "landsat9":  (4, 3, 2),  # Band 4, 3, 2
    "modis":     (1, 4, 3),  # Band 1 (Red), Band 4 (Green), Band 3 (Blue)
}


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _next_power_of_two(n: int) -> int:
    """Return the smallest power of two >= n (minimum 1)."""
    if n <= 1:
        return 1
    return 1 << (n - 1).bit_length()


def _encode_jpeg(rgb: np.ndarray, quality: int = 92) -> bytes:
    """Encode an (H, W, 3) uint8 RGB array as JPEG bytes."""
    from PIL import Image
    import io as _io

    img = Image.fromarray(rgb, mode="RGB")
    buf = _io.BytesIO()
    img.save(buf, format="JPEG", quality=quality, subsampling=0)
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def render_mosaic(
    bbox_deg: tuple[float, float, float, float],
    imagery_path: Path,
    source: str = "sentinel2",
    max_pixel_dim: int = 8192,
) -> MosaicDescriptor:
    """Render a JPEG mosaic texture covering bbox_deg from imagery_path.

    Parameters
    ----------
    bbox_deg:
        (lon_min, lat_min, lon_max, lat_max) in decimal degrees.
    imagery_path:
        Path to the source imagery GeoTIFF produced by mosaic.py.
    source:
        Imagery source: "sentinel2", "landsat9", or "modis".
    max_pixel_dim:
        Maximum pixel dimension for the output texture.  Raises
        MosaicTooLargeError if either axis of the power-of-two-rounded
        resolution would exceed this ceiling.  Use 4096 for low-end GPU
        targets, 8192 for desktop GPUs (GTX 1050 and later), 16384 for
        high-end.

    Returns
    -------
    MosaicDescriptor
        JPEG bytes and the geographic bounds of the rendered texture
        (equal to bbox_deg).

    Raises
    ------
    KeyError:
        If source is not "sentinel2", "landsat9", or "modis".
    MosaicTooLargeError:
        If either PoT-rounded axis exceeds max_pixel_dim.  Reduce the
        coverage area or switch to Option E (multi-mosaic regional
        textures, not yet implemented).
    """
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.windows import from_bounds as _window_from_bounds

    if source not in _SCALE_FACTORS:
        raise KeyError(
            f"Unsupported source '{source}'. Choose from: {list(_SCALE_FACTORS)}"
        )

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    r_band, g_band, b_band = _BAND_ORDER[source]
    linear_scale = float(_SCALE_FACTORS[source] * _DISPLAY_GAIN[source])
    inv_gamma = 1.0 / _DISPLAY_GAMMA[source]

    with rasterio.open(imagery_path) as src:
        # Native pixel size from the affine transform.
        resx = abs(src.transform.a)  # degrees per pixel (longitude / column)
        resy = abs(src.transform.e)  # degrees per pixel (latitude  / row)

        # Pixel count at native resolution for the requested bbox.
        w_native = math.ceil((lon_max - lon_min) / resx)
        h_native = math.ceil((lat_max - lat_min) / resy)

        # Round up to next power of two.
        w_pixels = _next_power_of_two(max(1, w_native))
        h_pixels = _next_power_of_two(max(1, h_native))

        if w_pixels > max_pixel_dim or h_pixels > max_pixel_dim:
            raise MosaicTooLargeError(
                f"Computed mosaic resolution {w_pixels}×{h_pixels} px exceeds "
                f"the GPU texture ceiling {max_pixel_dim}×{max_pixel_dim} px.  "
                "Reduce the coverage area or switch to Option E (multi-mosaic "
                "regional textures — not yet implemented)."
            )

        # Window in raster pixel space for the requested bbox.
        window = _window_from_bounds(lon_min, lat_min, lon_max, lat_max, src.transform)

        # Read and resample to (3, h_pixels, w_pixels); fill OOB areas with 0.
        data = src.read(
            [r_band, g_band, b_band],
            window=window,
            out_shape=(3, h_pixels, w_pixels),
            resampling=Resampling.bilinear,
            fill_value=0,
            boundless=True,
        )

    # DN → reflectance → gain → gamma → uint8  (same formula as colorize.py)
    r_raw = data[0].astype(np.float32)
    g_raw = data[1].astype(np.float32)
    b_raw = data[2].astype(np.float32)

    r_lin = np.clip(r_raw * linear_scale, 0.0, 1.0)
    g_lin = np.clip(g_raw * linear_scale, 0.0, 1.0)
    b_lin = np.clip(b_raw * linear_scale, 0.0, 1.0)

    r_out = (np.power(r_lin, inv_gamma) * 255.0).astype(np.uint8)
    g_out = (np.power(g_lin, inv_gamma) * 255.0).astype(np.uint8)
    b_out = (np.power(b_lin, inv_gamma) * 255.0).astype(np.uint8)

    # Stack to (H, W, 3) RGB for JPEG encoding.
    rgb = np.stack([r_out, g_out, b_out], axis=-1)
    jpeg_bytes = _encode_jpeg(rgb)

    return MosaicDescriptor(
        jpeg_bytes=jpeg_bytes,
        lon_min_deg=lon_min,
        lat_min_deg=lat_min,
        lon_max_deg=lon_max,
        lat_max_deg=lat_max,
        width_pixels=w_pixels,
        height_pixels=h_pixels,
    )
