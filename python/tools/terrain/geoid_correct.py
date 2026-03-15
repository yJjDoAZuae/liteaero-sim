"""geoid_correct.py — Orthometric → WGS84 ellipsoidal height conversion.

Converts DEM raster heights from orthometric (mean sea level) to WGS84 ellipsoidal
heights by applying the EGM96 or EGM2008 geoid undulation.

Copernicus DEM is already ellipsoidal and does NOT need this step.
SRTM and NASADEM provide orthometric heights and do need it.

Uses pyproj.Transformer to look up undulation from the PROJ datum grid.
PROJ datum shift grids are downloaded automatically on first use (requires internet;
cached afterward in the PROJ data directory).
"""

from __future__ import annotations

from pathlib import Path

import pyproj.network
from pyproj import CRS, Transformer

# Enable PROJ network access so datum shift grids are used from the local PROJ data cache
# (downloaded once to $PROJ_USER_WRITABLE_DIRECTORY; requires internet on first download).
pyproj.network.set_network_enabled(True)

# Compound CRS: horizontal WGS84 + vertical geoid model
_GEOID_CRS: dict[str, CRS] = {
    "egm2008": CRS.from_epsg(9518),  # WGS 84 + EGM2008 height
    "egm96": CRS.from_epsg(9707),  # WGS 84 + EGM96 height
}
_WGS84_3D = CRS.from_epsg(4979)  # WGS 84 (3D geographic)


def geoid_undulation(
    lat_deg: float,
    lon_deg: float,
    geoid: str = "egm2008",
) -> float:
    """Return the geoid undulation N (meters) at a single point.

    h_WGS84 = H_MSL + N.

    Uses pyproj CRS EPSG:9518 (WGS84 + EGM2008) → EPSG:4979 (WGS84 3D).
    At H_MSL = 0, the output ellipsoidal height equals N directly.

    Args:
        lat_deg: WGS84 latitude in degrees.
        lon_deg: WGS84 longitude in degrees.
        geoid:   "egm2008" (default) or "egm96".

    Returns:
        Geoid undulation N in meters.

    Raises:
        KeyError: if geoid name is not recognized.
    """
    if geoid not in _GEOID_CRS:
        raise KeyError(f"Unsupported geoid '{geoid}'. Choose 'egm2008' or 'egm96'.")
    t = Transformer.from_crs(_GEOID_CRS[geoid], _WGS84_3D, always_xy=True)
    _, _, h_ellipsoidal = t.transform(lon_deg, lat_deg, 0.0)
    return float(h_ellipsoidal)


def apply_geoid_correction(
    dem_path: Path,
    output_path: Path,
    geoid: str = "egm2008",
) -> None:
    """Add geoid undulation to every pixel of a DEM raster.

    Reads the input raster in blocks, vectorises the undulation look-up over a
    pre-sampled grid using scipy.interpolate.RegularGridInterpolator (for
    efficiency), and writes the corrected raster.

    Output is a new GeoTIFF with WGS84 ellipsoidal heights.

    Args:
        dem_path:    Path to input DEM GeoTIFF (orthometric heights, EPSG:4326).
        output_path: Path for output GeoTIFF (WGS84 ellipsoidal heights).
        geoid:       "egm2008" (default) or "egm96".

    Raises:
        KeyError: if geoid name is not recognized.
    """
    import numpy as np
    import rasterio
    from scipy.interpolate import RegularGridInterpolator

    if geoid not in _GEOID_CRS:
        raise KeyError(f"Unsupported geoid '{geoid}'. Choose 'egm2008' or 'egm96'.")

    with rasterio.open(dem_path) as src:
        profile = src.profile.copy()
        bounds = src.bounds
        data = src.read(1).astype(np.float64)
        nodata = src.nodata

        # Build a coarse undulation grid over the raster bounds for efficiency.
        # Sample at 0.1-degree spacing (EGM2008 is smooth; 0.1° ≈ 11 km is sufficient).
        grid_step = 0.1
        lons_grid = np.arange(bounds.left, bounds.right + grid_step, grid_step)
        lats_grid = np.arange(bounds.bottom, bounds.top + grid_step, grid_step)
        # Ensure at least 2 grid points in each direction for interpolation.
        if len(lons_grid) < 2:
            lons_grid = np.array([bounds.left, bounds.right])
        if len(lats_grid) < 2:
            lats_grid = np.array([bounds.bottom, bounds.top])

        undulation_grid = np.zeros((len(lats_grid), len(lons_grid)), dtype=np.float64)
        t = Transformer.from_crs(_GEOID_CRS[geoid], _WGS84_3D, always_xy=True)
        for j, lat in enumerate(lats_grid):
            lons_row = lons_grid
            lats_row = np.full_like(lons_row, lat)
            heights_in = np.zeros_like(lons_row)
            _, _, h_out = t.transform(lons_row, lats_row, heights_in)
            undulation_grid[j, :] = h_out

        # Build interpolator (lat, lon) → undulation
        interp = RegularGridInterpolator(
            (lats_grid, lons_grid),
            undulation_grid,
            method="linear",
            bounds_error=False,
            fill_value=None,
        )

        # Build coordinate arrays for every pixel
        rows, cols = np.meshgrid(
            np.arange(data.shape[0]), np.arange(data.shape[1]), indexing="ij"
        )
        xs, ys = rasterio.transform.xy(src.transform, rows.ravel(), cols.ravel())
        xs = np.array(xs, dtype=np.float64)
        ys = np.array(ys, dtype=np.float64)
        undulation_flat = interp((ys, xs))
        undulation_map = undulation_flat.reshape(data.shape)

        corrected = data + undulation_map
        if nodata is not None:
            corrected[data == nodata] = nodata

    output_path.parent.mkdir(parents=True, exist_ok=True)
    profile.update(dtype="float64")
    with rasterio.open(output_path, "w", **profile) as dst:
        dst.write(corrected.astype(np.float64), 1)
