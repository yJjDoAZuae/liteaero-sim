"""colorize.py — Assign per-facet RGB colors to a TerrainTileData by sampling imagery.

For each facet:
1. Compute the facet centroid in geodetic coordinates from the tile centroid
   and the three vertex ENU offsets (first-order ENU→geodetic approximation).
2. Sample the imagery raster at the centroid (lon, lat) using rasterio.DatasetReader.sample().
3. Extract the R, G, B bands per BAND_ORDER[source].
4. Scale from raw integer to 8-bit: clamp(raw * SCALE_FACTORS[source] * 255, 0, 255).
5. If the sampled pixel is nodata, use the default grey {128, 128, 128}.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from geodesy import enu_to_lonlat_deg
from las_terrain import TerrainTileData

# DN → linear reflectance [0, 1]
SCALE_FACTORS: dict[str, float] = {
    "sentinel2": 1.0 / 10000.0,  # Sentinel-2 L2A surface reflectance × 10000
    "landsat9":  1.0 / 55000.0,  # Landsat C2L2 surface reflectance × 55000
    "modis":     1.0 / 32767.0,  # MODIS MCD43A4 BRDF-adjusted reflectance × 32767
    "naip":      1.0 / 255.0,    # NAIP 8-bit uint8, already display-calibrated
}

# Brightness gain applied to linear reflectance before gamma correction.
# Natural land cover reflectance is typically 0.02–0.30; a gain of 3.5 (standard
# Sentinel Hub true-color) maps this to ~0.07–1.0 before gamma, producing
# display values in a natural brightness range.
DISPLAY_GAIN: dict[str, float] = {
    "sentinel2": 3.5,
    "landsat9":  3.5,
    "modis":     3.5,
    "naip":      1.0,  # already display-calibrated; no additional gain applied
}

# Display gamma (sRGB standard: 2.2).
# Applied as: display_value = (reflectance * gain) ^ (1 / gamma)
DISPLAY_GAMMA: dict[str, float] = {
    "sentinel2": 2.2,
    "landsat9":  2.2,
    "modis":     2.2,
    "naip":      1.0,  # already gamma-corrected in the source imagery
}

# Band indices (1-indexed, as used by rasterio) for R, G, B mapping.
BAND_ORDER: dict[str, tuple[int, int, int]] = {
    "sentinel2": (4, 3, 2),  # B04 (Red), B03 (Green), B02 (Blue)
    "landsat9":  (4, 3, 2),  # Band 4, 3, 2
    "modis":     (1, 4, 3),  # Band 1 (Red), Band 4 (Green), Band 3 (Blue)
    "naip":      (1, 2, 3),  # Band 1=R, Band 2=G, Band 3=B (4=NIR, unused)
}

_DEFAULT_GREY = np.array([128, 128, 128], dtype=np.uint8)


def colorize(
    tile: TerrainTileData,
    imagery_path: Path,
    source: str = "sentinel2",
) -> TerrainTileData:
    """Return a new TerrainTileData with colors populated from imagery_path.

    For each facet:
    1. Compute centroid geodetic position from tile centroid + mean ENU offsets.
    2. Sample the imagery raster at centroid (lon, lat).
    3. Scale and clamp raw band values to uint8.
    4. Nodata pixels → default grey {128, 128, 128}.

    Returns a new TerrainTileData (input tile is not modified).

    Raises:
        KeyError: if source is not in SCALE_FACTORS.
    """
    import rasterio

    if source not in SCALE_FACTORS:
        raise KeyError(f"Unsupported source '{source}'. Choose from: {list(SCALE_FACTORS)}")

    scale = SCALE_FACTORS[source]
    gain = DISPLAY_GAIN[source]
    inv_gamma = 1.0 / DISPLAY_GAMMA[source]
    r_band, g_band, b_band = BAND_ORDER[source]

    # Compute facet centroid ENU offsets (mean of 3 vertex ENU positions).
    v0 = tile.vertices[tile.indices[:, 0]]  # (F, 3)
    v1 = tile.vertices[tile.indices[:, 1]]
    v2 = tile.vertices[tile.indices[:, 2]]
    centroid_enu = (v0 + v1 + v2) / 3.0  # (F, 3) float32

    east_m = centroid_enu[:, 0].astype(np.float64)
    north_m = centroid_enu[:, 1].astype(np.float64)

    # Convert to geodetic lon/lat.
    lon_arr, lat_arr = enu_to_lonlat_deg(east_m, north_m, tile.centroid_lat_rad, tile.centroid_lon_rad)

    n_facets = len(tile.indices)
    new_colors = np.full((n_facets, 3), 128, dtype=np.uint8)

    with rasterio.open(imagery_path) as src:
        nodata = src.nodata
        # Convert all geodetic coords to pixel indices in one vectorized call.
        rows, cols = rasterio.transform.rowcol(src.transform, lon_arr, lat_arr)
        rows = np.asarray(rows, dtype=np.intp)
        cols = np.asarray(cols, dtype=np.intp)

        # Mark facets whose centroid falls outside the raster extent.
        in_bounds = (rows >= 0) & (rows < src.height) & (cols >= 0) & (cols < src.width)
        rows_safe = np.clip(rows, 0, src.height - 1)
        cols_safe = np.clip(cols, 0, src.width - 1)

        # Read the three required bands into memory (H × W each).
        r_data = src.read(r_band)
        g_data = src.read(g_band)
        b_data = src.read(b_band)

    # Sample all facets at once via numpy fancy indexing.
    r_raw = r_data[rows_safe, cols_safe].astype(np.float32)
    g_raw = g_data[rows_safe, cols_safe].astype(np.float32)
    b_raw = b_data[rows_safe, cols_safe].astype(np.float32)

    # DN → reflectance → gain → gamma → uint8
    # display = clip(DN * scale * gain, 0, 1) ^ (1/gamma) * 255
    linear_scale = float(scale * gain)
    r_lin = np.clip(r_raw * linear_scale, 0.0, 1.0)
    g_lin = np.clip(g_raw * linear_scale, 0.0, 1.0)
    b_lin = np.clip(b_raw * linear_scale, 0.0, 1.0)
    new_colors[:, 0] = (np.power(r_lin, inv_gamma) * 255.0).astype(np.uint8)
    new_colors[:, 1] = (np.power(g_lin, inv_gamma) * 255.0).astype(np.uint8)
    new_colors[:, 2] = (np.power(b_lin, inv_gamma) * 255.0).astype(np.uint8)

    # Replace nodata and out-of-bounds facets with default grey.
    oob = ~in_bounds
    if nodata is not None:
        oob |= (r_raw == nodata) | (g_raw == nodata) | (b_raw == nodata)
    new_colors[oob] = _DEFAULT_GREY

    import dataclasses

    return dataclasses.replace(tile, colors=new_colors)
