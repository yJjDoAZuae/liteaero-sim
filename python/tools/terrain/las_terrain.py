"""las_terrain.py — Pure-Python reader/writer for the .las_terrain binary format.

This module is the single source of truth for Python-side serialization and is imported
by all other terrain pipeline tools.  No pybind11 or C++ interop needed.

File format (little-endian):
    [4 bytes]   magic = 0x4C415354 ("LAST")
    [4 bytes]   format_version = 1  (uint32)
    [4 bytes]   tile_count          (uint32)
    For each tile:
        [4 bytes]   metadata_length (uint32)
        [N bytes]   metadata JSON   (UTF-8, no BOM)
        [4 bytes]   vertex_count    (uint32)
        [V*12 bytes] vertices       (float32 east_m, float32 north_m, float32 up_m)
        [4 bytes]   facet_count     (uint32)
        [F*15 bytes] facets         (uint32 i0, uint32 i1, uint32 i2, uint8 r, uint8 g, uint8 b)
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_MAGIC: int = 0x4C415354  # "LAST" in little-endian
_FORMAT_VERSION: int = 1


@dataclass
class TerrainTileData:
    """In-memory representation of one terrain tile at a single LOD level.

    All ENU coordinates are float32 offsets from the tile centroid.
    Centroid position and geodetic bounds are float64.
    This mirrors the C++ TerrainVertex / TerrainFacet layout.
    """

    lod: int  # 0–6 (TerrainLod enum value)
    centroid_lat_rad: float  # WGS84 geodetic latitude of tile centroid (rad, float64)
    centroid_lon_rad: float  # WGS84 geodetic longitude of tile centroid (rad, float64)
    centroid_height_m: float  # WGS84 ellipsoidal height of tile centroid (m)
    lat_min_rad: float
    lat_max_rad: float
    lon_min_rad: float
    lon_max_rad: float
    height_min_m: float
    height_max_m: float
    vertices: np.ndarray  # (N, 3) float32 — ENU (east, north, up) offsets in meters
    indices: np.ndarray  # (F, 3) uint32  — vertex indices, CCW winding
    colors: np.ndarray  # (F, 3) uint8   — R8G8B8 representative color per facet


def write_las_terrain(path: Path, tiles: list[TerrainTileData]) -> None:
    """Write one or more tiles to a .las_terrain binary file.

    Raises:
        ValueError: if tiles is empty.
        IOError:    on write failure.
    """
    if not tiles:
        raise ValueError("tiles must not be empty")

    with open(path, "wb") as f:
        # File header: magic, format_version, tile_count
        f.write(struct.pack("<III", _MAGIC, _FORMAT_VERSION, len(tiles)))

        for tile in tiles:
            # Metadata JSON
            meta: dict = {
                "schema_version": 1,
                "lod": tile.lod,
                "centroid_lat_rad": tile.centroid_lat_rad,
                "centroid_lon_rad": tile.centroid_lon_rad,
                "centroid_height_m": float(tile.centroid_height_m),
                "lat_min_rad": tile.lat_min_rad,
                "lat_max_rad": tile.lat_max_rad,
                "lon_min_rad": tile.lon_min_rad,
                "lon_max_rad": tile.lon_max_rad,
                "height_min_m": float(tile.height_min_m),
                "height_max_m": float(tile.height_max_m),
            }
            meta_bytes = json.dumps(meta).encode("utf-8")
            f.write(struct.pack("<I", len(meta_bytes)))
            f.write(meta_bytes)

            # Vertices: vertex_count + V * 12 bytes (3 × float32)
            verts = np.asarray(tile.vertices, dtype=np.float32)
            assert verts.ndim == 2 and verts.shape[1] == 3
            f.write(struct.pack("<I", len(verts)))
            f.write(verts.tobytes())

            # Facets: facet_count + F * 15 bytes (3 × uint32 + 3 × uint8)
            inds = np.asarray(tile.indices, dtype=np.uint32)
            cols = np.asarray(tile.colors, dtype=np.uint8)
            assert inds.ndim == 2 and inds.shape[1] == 3
            assert cols.ndim == 2 and cols.shape[1] == 3
            f.write(struct.pack("<I", len(inds)))
            for i in range(len(inds)):
                f.write(struct.pack("<III", int(inds[i, 0]), int(inds[i, 1]), int(inds[i, 2])))
                f.write(struct.pack("BBB", int(cols[i, 0]), int(cols[i, 1]), int(cols[i, 2])))


def read_las_terrain(path: Path) -> list[TerrainTileData]:
    """Read all tiles from a .las_terrain binary file.

    Raises:
        ValueError: if magic bytes != LAST or format_version != 1.
        IOError:    on read failure.
    """
    with open(path, "rb") as f:
        # File header
        header = f.read(12)
        if len(header) < 12:
            raise ValueError("File too short to contain a valid .las_terrain header")
        magic, version, tile_count = struct.unpack("<III", header)

        if magic != _MAGIC:
            raise ValueError(
                f"Invalid magic bytes: expected {_MAGIC:#010x} ('LAST'), got {magic:#010x}"
            )
        if version != _FORMAT_VERSION:
            raise ValueError(
                f"Unsupported format_version {version}; expected {_FORMAT_VERSION}"
            )

        tiles: list[TerrainTileData] = []
        for _ in range(tile_count):
            # Metadata JSON
            meta_length = struct.unpack("<I", f.read(4))[0]
            meta: dict = json.loads(f.read(meta_length).decode("utf-8"))

            # Vertices
            vertex_count = struct.unpack("<I", f.read(4))[0]
            verts_raw = f.read(vertex_count * 12)
            vertices = np.frombuffer(verts_raw, dtype=np.float32).reshape(vertex_count, 3).copy()

            # Facets
            facet_count = struct.unpack("<I", f.read(4))[0]
            indices = np.zeros((facet_count, 3), dtype=np.uint32)
            colors = np.zeros((facet_count, 3), dtype=np.uint8)
            for i in range(facet_count):
                i0, i1, i2 = struct.unpack("<III", f.read(12))
                r, g, b = struct.unpack("BBB", f.read(3))
                indices[i] = [i0, i1, i2]
                colors[i] = [r, g, b]

            tiles.append(
                TerrainTileData(
                    lod=int(meta["lod"]),
                    centroid_lat_rad=float(meta["centroid_lat_rad"]),
                    centroid_lon_rad=float(meta["centroid_lon_rad"]),
                    centroid_height_m=float(meta["centroid_height_m"]),
                    lat_min_rad=float(meta["lat_min_rad"]),
                    lat_max_rad=float(meta["lat_max_rad"]),
                    lon_min_rad=float(meta["lon_min_rad"]),
                    lon_max_rad=float(meta["lon_max_rad"]),
                    height_min_m=float(meta["height_min_m"]),
                    height_max_m=float(meta["height_max_m"]),
                    vertices=vertices,
                    indices=indices,
                    colors=colors,
                )
            )

    return tiles
