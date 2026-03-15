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

import math
from pathlib import Path

import numpy as np

from las_terrain import TerrainTileData

_WGS84_A: float = 6_378_137.0
_WGS84_F: float = 1.0 / 298.257223563
_WGS84_E2: float = 2.0 * _WGS84_F - _WGS84_F**2

# Surface reflectance scale factors: raw_integer * scale → reflectance [0, 1]
SCALE_FACTORS: dict[str, float] = {
    "sentinel2": 1.0 / 10000.0,  # Sentinel-2 L2A surface reflectance
    "landsat9": 1.0 / 55000.0,  # Landsat C2L2 surface reflectance
    "modis": 1.0 / 32767.0,  # MODIS MCD43A4 BRDF-adjusted reflectance
}

# Band indices (1-indexed, as used by rasterio) for R, G, B mapping.
BAND_ORDER: dict[str, tuple[int, int, int]] = {
    "sentinel2": (4, 3, 2),  # B04 (Red), B03 (Green), B02 (Blue)
    "landsat9": (4, 3, 2),  # Band 4, 3, 2
    "modis": (1, 4, 3),  # Band 1 (Red), Band 4 (Green), Band 3 (Blue)
}

_DEFAULT_GREY = np.array([128, 128, 128], dtype=np.uint8)


def _enu_to_lonlat_deg(
    east_m: np.ndarray,
    north_m: np.ndarray,
    clat_rad: float,
    clon_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """First-order ENU → geodetic approximation for small offsets.

    Accurate to < 1 m for offsets up to 50 km from the centroid.
    Returns (lon_deg, lat_deg) arrays.
    """
    sin_lat = math.sin(clat_rad)
    cos_lat = math.cos(clat_rad)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    M = _WGS84_A * (1.0 - _WGS84_E2) / (1.0 - _WGS84_E2 * sin_lat**2) ** 1.5

    dlat_rad = north_m / M
    dlon_rad = east_m / (N * cos_lat + 1e-30)

    lon_deg = math.degrees(clon_rad) + np.degrees(dlon_rad)
    lat_deg = math.degrees(clat_rad) + np.degrees(dlat_rad)
    return lon_deg, lat_deg


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
    r_band, g_band, b_band = BAND_ORDER[source]

    # Compute facet centroid ENU offsets (mean of 3 vertex ENU positions).
    v0 = tile.vertices[tile.indices[:, 0]]  # (F, 3)
    v1 = tile.vertices[tile.indices[:, 1]]
    v2 = tile.vertices[tile.indices[:, 2]]
    centroid_enu = (v0 + v1 + v2) / 3.0  # (F, 3) float32

    east_m = centroid_enu[:, 0].astype(np.float64)
    north_m = centroid_enu[:, 1].astype(np.float64)

    # Convert to geodetic lon/lat.
    lon_arr, lat_arr = _enu_to_lonlat_deg(east_m, north_m, tile.centroid_lat_rad, tile.centroid_lon_rad)

    n_facets = len(tile.indices)
    new_colors = np.full((n_facets, 3), 128, dtype=np.uint8)

    with rasterio.open(imagery_path) as src:
        nodata = src.nodata
        coords = list(zip(lon_arr.tolist(), lat_arr.tolist()))
        # sample() returns an iterator of arrays; each element is a 1-D array of band values.
        samples = list(src.sample(coords, indexes=[r_band, g_band, b_band]))

    for i, (r_raw, g_raw, b_raw) in enumerate(samples):
        if nodata is not None and (r_raw == nodata or g_raw == nodata or b_raw == nodata):
            new_colors[i] = _DEFAULT_GREY
            continue
        r8 = int(np.clip(float(r_raw) * scale * 255.0, 0.0, 255.0))
        g8 = int(np.clip(float(g_raw) * scale * 255.0, 0.0, 255.0))
        b8 = int(np.clip(float(b_raw) * scale * 255.0, 0.0, 255.0))
        new_colors[i] = [r8, g8, b8]

    import dataclasses

    return dataclasses.replace(tile, colors=new_colors)
