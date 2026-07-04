"""triangulate.py — Build an L0 TIN from a DEM raster using Delaunay triangulation.

Boundary points shared with adjacent tiles are injected before triangulation to
guarantee crack-free joins between tile edges.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import Delaunay

from geodesy import enu_from_geodetic
from las_terrain import TerrainTileData
from raster_sample import RasterSampler

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


def triangulate(
    dem: RasterSampler,
    bbox_deg: tuple[float, float, float, float],  # (lon_min, lat_min, lon_max, lat_max)
    lod: int = 0,
    boundary_points: np.ndarray | None = None,  # (K, 2) float64 lon/lat of locked boundary verts
    spacing_deg: float | None = None,  # grid spacing override (else lod_grid_spacing_deg(lod))
) -> TerrainTileData:
    """Build a TIN from an in-memory DEM sampler over bbox_deg at the given LOD.

    Algorithm:
    1. Sample DEM at a regular grid with spacing ``spacing_deg`` (or, if None, the
       LOD's native ``lod_grid_spacing_deg(lod)``).  A coarser-than-native spacing is
       clamped by the caller so a small fixed-footprint cell still yields a valid mesh.
    2. Inject boundary_points (if provided) without modification.
    3. Run scipy.spatial.Delaunay on the (lon, lat) 2D projection.
    4. Convert all points to ENU float32 offsets from the tile centroid.
    5. Assign default grey color {128, 128, 128} to all facets.
    6. Return TerrainTileData.

    ``dem`` is a :class:`RasterSampler` (read once, reused across all cells) so that
    the per-cell cost is the Delaunay triangulation, not per-point raster I/O.

    Raises:
        ValueError: if the DEM does not cover bbox_deg.
        ValueError: if the triangulation produces fewer than 2 facets.
    """
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    spacing = spacing_deg if spacing_deg is not None else lod_grid_spacing_deg(lod)

    if not dem.covers(bbox_deg):
        raise ValueError(
            f"DEM does not cover bbox_deg: DEM bounds={dem.bounds}, "
            f"requested bbox={bbox_deg}"
        )

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

    heights_flat = dem.sample(lon_flat, lat_flat).astype(np.float64)
    if dem.nodata is not None:
        heights_flat[heights_flat == dem.nodata] = 0.0

    # Inject boundary points.
    if boundary_points is not None and len(boundary_points) > 0:
        bp_heights = dem.sample(boundary_points[:, 0], boundary_points[:, 1]).astype(np.float64)
        if dem.nodata is not None:
            bp_heights[bp_heights == dem.nodata] = 0.0
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
    vertices = enu_from_geodetic(lat_rad, lon_rad, heights_flat, clat, clon, centroid_height)

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
