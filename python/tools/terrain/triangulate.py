"""triangulate.py — Build an L0 TIN from a DEM raster using Delaunay triangulation.

Boundary points shared with adjacent tiles are injected before triangulation to
guarantee crack-free joins between tile edges.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from scipy.spatial import Delaunay

from las_terrain import TerrainTileData

_WGS84_A: float = 6_378_137.0
_WGS84_F: float = 1.0 / 298.257223563
_WGS84_E2: float = 2.0 * _WGS84_F - _WGS84_F**2

# Approximate grid sampling interval in degrees for each LOD level.
# LOD 0 ≈ 10 m vertex spacing; LOD 6 ≈ 10 km vertex spacing.
_LOD_GRID_SPACING_DEG: list[float] = [
    0.000090,  # LOD 0 ≈  10 m
    0.000270,  # LOD 1 ≈  30 m
    0.000900,  # LOD 2 ≈ 100 m
    0.002700,  # LOD 3 ≈ 300 m
    0.009000,  # LOD 4 ≈ 1 km
    0.027000,  # LOD 5 ≈ 3 km
    0.090000,  # LOD 6 ≈ 10 km
]


def lod_grid_spacing_deg(lod: int) -> float:
    """Return the approximate grid sampling interval in degrees for the given LOD."""
    return _LOD_GRID_SPACING_DEG[lod]


def _geodetic_to_ecef_scalar(
    lat_rad: float, lon_rad: float, h_m: float
) -> tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - _WGS84_E2) + h_m) * sin_lat
    return x, y, z


def _geodetic_to_enu_vec(
    lat_rad: np.ndarray,
    lon_rad: np.ndarray,
    h_m: np.ndarray,
    clat_rad: float,
    clon_rad: float,
    ch_m: float,
) -> np.ndarray:
    """Convert geodetic arrays to ENU float32 offsets from centroid.

    Returns (N, 3) float32 array with columns [east_m, north_m, up_m].
    """
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    N_arr = _WGS84_A / np.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    px = (N_arr + h_m) * cos_lat * cos_lon
    py = (N_arr + h_m) * cos_lat * sin_lon
    pz = (N_arr * (1.0 - _WGS84_E2) + h_m) * sin_lat

    cx, cy, cz = _geodetic_to_ecef_scalar(clat_rad, clon_rad, ch_m)
    dx = px - cx
    dy = py - cy
    dz = pz - cz

    # ENU basis vectors at centroid
    sin_clat = math.sin(clat_rad)
    cos_clat = math.cos(clat_rad)
    sin_clon = math.sin(clon_rad)
    cos_clon = math.cos(clon_rad)

    east = -sin_clon * dx + cos_clon * dy
    north = -sin_clat * cos_clon * dx - sin_clat * sin_clon * dy + cos_clat * dz
    up = cos_clat * cos_clon * dx + cos_clat * sin_clon * dy + sin_clat * dz

    return np.column_stack([east, north, up]).astype(np.float32)


def triangulate(
    dem_path: Path,
    bbox_deg: tuple[float, float, float, float],  # (lon_min, lat_min, lon_max, lat_max)
    lod: int = 0,
    boundary_points: np.ndarray | None = None,  # (K, 2) float64 lon/lat of locked boundary verts
) -> TerrainTileData:
    """Build a TIN from dem_path over bbox_deg at the given LOD.

    Algorithm:
    1. Sample DEM at a regular grid with spacing lod_grid_spacing_deg(lod).
    2. Inject boundary_points (if provided) without modification.
    3. Run scipy.spatial.Delaunay on the (lon, lat) 2D projection.
    4. Convert all points to ENU float32 offsets from the tile centroid.
    5. Assign default grey color {128, 128, 128} to all facets.
    6. Return TerrainTileData.

    Raises:
        ValueError: if dem_path does not cover bbox_deg.
        ValueError: if the triangulation produces fewer than 2 facets.
    """
    import rasterio

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    spacing = lod_grid_spacing_deg(lod)

    # Sample DEM on a regular lon/lat grid.
    lons = np.arange(lon_min, lon_max + spacing * 0.5, spacing)
    lats = np.arange(lat_min, lat_max + spacing * 0.5, spacing)

    if len(lons) < 2 or len(lats) < 2:
        raise ValueError(
            f"bbox_deg too small for LOD {lod} grid spacing {spacing}° — "
            f"need at least 2 points per axis"
        )

    lon_grid, lat_grid = np.meshgrid(lons, lats)
    lon_flat = lon_grid.ravel().astype(np.float64)
    lat_flat = lat_grid.ravel().astype(np.float64)

    # Sample heights from DEM.
    with rasterio.open(dem_path) as src:
        bounds = src.bounds
        eps = 1e-8
        if (
            lon_min < bounds.left - eps
            or lat_min < bounds.bottom - eps
            or lon_max > bounds.right + eps
            or lat_max > bounds.top + eps
        ):
            raise ValueError(
                f"dem_path does not cover bbox_deg: DEM bounds={bounds}, "
                f"requested bbox={bbox_deg}"
            )

        coords = list(zip(lon_flat.tolist(), lat_flat.tolist()))
        heights_flat = np.array([v[0] for v in src.sample(coords)], dtype=np.float64)
        src_nodata = src.nodata

    if src_nodata is not None:
        heights_flat[heights_flat == src_nodata] = 0.0

    # Inject boundary points.
    if boundary_points is not None and len(boundary_points) > 0:
        with rasterio.open(dem_path) as src:
            bp_coords = list(zip(boundary_points[:, 0].tolist(), boundary_points[:, 1].tolist()))
            bp_heights = np.array(
                [v[0] for v in src.sample(bp_coords)], dtype=np.float64
            )
        lon_flat = np.concatenate([lon_flat, boundary_points[:, 0]])
        lat_flat = np.concatenate([lat_flat, boundary_points[:, 1]])
        heights_flat = np.concatenate([heights_flat, bp_heights])

    if len(lon_flat) < 3:
        raise ValueError("Triangulation requires at least 3 points")

    # Delaunay triangulation on (lon, lat) 2D projection.
    points_2d = np.column_stack([lon_flat, lat_flat])
    tri = Delaunay(points_2d)

    if len(tri.simplices) < 2:
        raise ValueError(
            f"Triangulation produced {len(tri.simplices)} facet(s); need at least 2"
        )

    # Tile centroid = geographic center of bbox.
    centroid_lon_deg = (lon_min + lon_max) / 2.0
    centroid_lat_deg = (lat_min + lat_max) / 2.0
    centroid_height = float(np.mean(heights_flat))
    clat = math.radians(centroid_lat_deg)
    clon = math.radians(centroid_lon_deg)

    # Convert all points to ENU float32 offsets from centroid.
    lat_rad = np.radians(lat_flat)
    lon_rad = np.radians(lon_flat)
    vertices = _geodetic_to_enu_vec(lat_rad, lon_rad, heights_flat, clat, clon, centroid_height)

    indices = tri.simplices.astype(np.uint32)
    colors = np.full((len(indices), 3), 128, dtype=np.uint8)

    return TerrainTileData(
        lod=lod,
        centroid_lat_rad=clat,
        centroid_lon_rad=clon,
        centroid_height_m=float(centroid_height),
        lat_min_rad=float(np.min(lat_rad)),
        lat_max_rad=float(np.max(lat_rad)),
        lon_min_rad=float(np.min(lon_rad)),
        lon_max_rad=float(np.max(lon_rad)),
        height_min_m=float(np.min(heights_flat)),
        height_max_m=float(np.max(heights_flat)),
        vertices=vertices,
        indices=indices,
        colors=colors,
    )
