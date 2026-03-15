"""export_gltf.py — Trimesh-based GLB export for terrain tiles.

Exports TerrainTileData tiles to a binary GLB file using trimesh.
Axis convention matches the C++ TerrainMesh::exportGltf() implementation:
    glTF X = ENU East,  glTF Y = ENU Up,  glTF Z = −ENU North.

Per-facet COLOR_0 requires vertex duplication (3 unique vertices per triangle).
No pybind11 or C++ interop needed — this is a pure-Python implementation.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from las_terrain import TerrainTileData

_WGS84_A: float = 6_378_137.0
_WGS84_F: float = 1.0 / 298.257223563
_WGS84_E2: float = 2.0 * _WGS84_F - _WGS84_F**2


def _ecef_from_geodetic(lat_rad: float, lon_rad: float, h_m: float) -> np.ndarray:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - _WGS84_E2) + h_m) * sin_lat
    return np.array([x, y, z], dtype=np.float64)


def _enu_offset_between(
    lat1: float, lon1: float, h1: float,
    lat2: float, lon2: float, h2: float,
) -> np.ndarray:
    """Return ENU offset from (lat2, lon2, h2) to (lat1, lon1, h1) in the frame of point 2."""
    p1 = _ecef_from_geodetic(lat1, lon1, h1)
    p2 = _ecef_from_geodetic(lat2, lon2, h2)
    dp = p1 - p2

    sin_lat = math.sin(lat2)
    cos_lat = math.cos(lat2)
    sin_lon = math.sin(lon2)
    cos_lon = math.cos(lon2)

    east = -sin_lon * dp[0] + cos_lon * dp[1]
    north = -sin_lat * cos_lon * dp[0] - sin_lat * sin_lon * dp[1] + cos_lat * dp[2]
    up = cos_lat * cos_lon * dp[0] + cos_lat * sin_lon * dp[1] + sin_lat * dp[2]
    return np.array([east, north, up], dtype=np.float64)


def export_gltf(
    tiles: list[TerrainTileData],
    output_path: Path,
    world_origin: tuple[float, float, float] | None = None,  # (lat_rad, lon_rad, height_m)
) -> None:
    """Export tiles to a binary GLB file.

    Per-facet COLOR_0: requires vertex duplication (3 unique vertices per triangle).
    Vertex positions are ENU offsets from the tile centroid, transformed to glTF space:
        glTF X = ENU East,  glTF Y = ENU Up,  glTF Z = −ENU North.

    Node translations are ENU offsets of the tile centroid from world_origin.

    Root-level scene extras:
        {"liteaerosim_terrain": true, "schema_version": 1,
         "world_origin_lat_rad": ..., "world_origin_lon_rad": ..., "world_origin_height_m": ...}

    Raises:
        ValueError: if tiles is empty.
        IOError:    on write failure.
    """
    import trimesh
    import trimesh.exchange.gltf as gltf_io

    if not tiles:
        raise ValueError("tiles must not be empty")

    if world_origin is None:
        world_origin = (
            tiles[0].centroid_lat_rad,
            tiles[0].centroid_lon_rad,
            tiles[0].centroid_height_m,
        )
    origin_lat, origin_lon, origin_h = world_origin

    scene = trimesh.scene.Scene()

    for idx, tile in enumerate(tiles):
        n_facets = len(tile.indices)

        # Duplicate vertices: 3 unique vertices per triangle (required for per-facet COLOR_0).
        dup_verts = np.zeros((n_facets * 3, 3), dtype=np.float32)
        dup_verts[0::3] = tile.vertices[tile.indices[:, 0]]
        dup_verts[1::3] = tile.vertices[tile.indices[:, 1]]
        dup_verts[2::3] = tile.vertices[tile.indices[:, 2]]

        dup_faces = np.arange(n_facets * 3, dtype=np.int64).reshape(-1, 3)

        # Per-vertex colors (repeat facet color 3 times); add alpha = 255.
        dup_colors = np.zeros((n_facets * 3, 4), dtype=np.uint8)
        dup_colors[0::3, :3] = tile.colors
        dup_colors[1::3, :3] = tile.colors
        dup_colors[2::3, :3] = tile.colors
        dup_colors[:, 3] = 255

        # glTF axis permutation: X=East, Y=Up, Z=−North.
        gltf_verts = np.column_stack(
            [
                dup_verts[:, 0],  # East  → X
                dup_verts[:, 2],  # Up    → Y
                -dup_verts[:, 1],  # −North → Z
            ]
        ).astype(np.float32)

        mesh = trimesh.Trimesh(
            vertices=gltf_verts,
            faces=dup_faces,
            vertex_colors=dup_colors,
            process=False,
        )

        # Node translation: ENU offset of tile centroid from world origin.
        tile_offset = _enu_offset_between(
            tile.centroid_lat_rad, tile.centroid_lon_rad, tile.centroid_height_m,
            origin_lat, origin_lon, origin_h,
        )
        # Apply axis permutation to translation as well.
        gltf_translation = np.array(
            [tile_offset[0], tile_offset[2], -tile_offset[1]], dtype=np.float64
        )

        matrix = np.eye(4)
        matrix[:3, 3] = gltf_translation

        scene.add_geometry(
            mesh,
            geom_name=f"tile_lod{tile.lod}_{idx}",
            transform=matrix,
        )

    # Root-level extras (stored in scene.metadata so trimesh writes them to the GLB).
    extras = {
        "liteaerosim_terrain": True,
        "schema_version": 1,
        "world_origin_lat_rad": origin_lat,
        "world_origin_lon_rad": origin_lon,
        "world_origin_height_m": origin_h,
    }
    scene.metadata["extras"] = extras

    glb_bytes = gltf_io.export_glb(scene)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(glb_bytes)
