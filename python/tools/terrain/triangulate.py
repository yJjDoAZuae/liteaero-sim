"""triangulate.py — Build a terrain TIN from a DEM raster.

Every tile is a regular lat/lon grid, whose triangulation is a fixed index pattern (each cell
splits into two upward-facing triangles) computed analytically in pure numpy — no general
triangulator, and critically no ``scipy``/OpenBLAS import in the build's parallel workers (that
import's per-thread buffers were a memory-scaling hazard).  Only the optional ``boundary_points``
path (crack-free stitching of an irregular point set) needs a general Delaunay, which is imported
lazily there.
"""

from __future__ import annotations

import math

import numpy as np

from geodesy import enu_from_geodetic
from las_terrain import TerrainTileData
from raster_sample import RasterSampler


def _grid_triangulation(nx: int, ny: int) -> np.ndarray:
    """Triangle indices for a regular ``nx``×``ny`` grid in row-major point order.

    Point index is ``lat_row * nx + lon_col`` (matching ``np.meshgrid(lons, lats).ravel()``).
    Each of the ``(nx-1)·(ny-1)`` cells splits into two triangles wound counter-clockwise in the
    east-north plane, so their normals face up.  Returns a ``(2·(nx-1)·(ny-1), 3)`` uint32 array.
    """
    i = np.arange(ny - 1)
    j = np.arange(nx - 1)
    ii, jj = np.meshgrid(i, j, indexing="ij")
    a = (ii * nx + jj).ravel()          # (i,   j)
    b = (ii * nx + jj + 1).ravel()      # (i,   j+1)  — east of a
    c = ((ii + 1) * nx + jj).ravel()    # (i+1, j)    — north of a
    d = ((ii + 1) * nx + jj + 1).ravel()  # (i+1, j+1)
    tri1 = np.stack([a, b, c], axis=1)
    tri2 = np.stack([b, d, c], axis=1)
    return np.concatenate([tri1, tri2], axis=0).astype(np.uint32)

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
    2. Triangulate: for a regular grid, the analytic two-triangles-per-cell pattern
       (:func:`_grid_triangulation`); when ``boundary_points`` are injected (irregular set),
       a lazily-imported ``scipy.spatial.Delaunay``.
    3. Convert all points to ENU float32 offsets from the tile centroid.
    4. Assign default grey color {128, 128, 128} to all facets; return TerrainTileData.

    ``dem`` is a :class:`RasterSampler` sampled by vectorized fancy-index, so the per-cell cost
    is negligible.

    Raises:
        ValueError: if the DEM does not cover bbox_deg or the grid is smaller than 2×2.
    """
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    spacing = spacing_deg if spacing_deg is not None else lod_grid_spacing_deg(lod)

    if not dem.covers(bbox_deg):
        raise ValueError(
            f"DEM does not cover bbox_deg: DEM bounds={dem.bounds}, "
            f"requested bbox={bbox_deg}"
        )

    # Sample the DEM on a regular lon/lat grid whose endpoints are LOCKED to the cell edges
    # (OQ-TB-6). np.linspace includes lon_min/lon_max (and lat_min/lat_max) exactly, so every tile
    # spans its cell precisely and abutting same-LOD tiles SHARE their boundary vertices — a road or
    # runway no longer shifts across a seam. Point counts are chosen so the realized vertex spacing is
    # within half a step of the nominal `spacing`; the sub-percent deviation is immaterial to sampling.
    # (The old np.arange started from lon_min and, unless the cell width was an exact multiple of
    # `spacing`, terminated up to half a step off the cell edge, mis-aligning neighbours.)
    nx = max(2, int(round((lon_max - lon_min) / spacing)) + 1)
    ny = max(2, int(round((lat_max - lat_min) / spacing)) + 1)
    lons = np.linspace(lon_min, lon_max, nx)
    lats = np.linspace(lat_min, lat_max, ny)

    nx, ny = len(lons), len(lats)
    lon_grid, lat_grid = np.meshgrid(lons, lats)
    lon_flat = lon_grid.ravel().astype(np.float64)
    lat_flat = lat_grid.ravel().astype(np.float64)

    heights_flat = dem.sample(lon_flat, lat_flat).astype(np.float64)
    if dem.nodata is not None:
        heights_flat[heights_flat == dem.nodata] = 0.0

    if boundary_points is not None and len(boundary_points) > 0:
        # Irregular point set (grid + injected boundary verts) → general Delaunay.  Imported
        # lazily so the regular-grid build path never pulls in scipy/OpenBLAS.
        from scipy.spatial import Delaunay

        bp_heights = dem.sample(boundary_points[:, 0], boundary_points[:, 1]).astype(np.float64)
        if dem.nodata is not None:
            bp_heights[bp_heights == dem.nodata] = 0.0
        lon_flat = np.concatenate([lon_flat, boundary_points[:, 0]])
        lat_flat = np.concatenate([lat_flat, boundary_points[:, 1]])
        heights_flat = np.concatenate([heights_flat, bp_heights])
        indices = Delaunay(np.column_stack([lon_flat, lat_flat])).simplices.astype(np.uint32)
    else:
        # Regular grid → analytic two-triangles-per-cell pattern (no triangulator).
        indices = _grid_triangulation(nx, ny)

    if len(indices) < 2:
        raise ValueError(
            f"Triangulation produced {len(indices)} facet(s); need at least 2"
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
