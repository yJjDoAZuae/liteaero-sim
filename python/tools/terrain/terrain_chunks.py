"""terrain_chunks.py — Per-region chunk packaging for streamable terrain.

Realizes OQ-LS-20 Alternative 2 (per-region chunk files plus a coordinate-addressable
index).  Instead of one monolithic GLB, the terrain is split into per-(LOD, chunk) GLB
files named ``L{lod}/c_{cx}_{cy}.glb`` plus a small top-level ``descriptor.json``.  A chunk
is a square cell of the world-origin ENU plane whose side is **per-LOD**,
``chunk_sizes_m[lod] = tile_footprints_m[lod] × chunk_footprints`` (per-LOD footprints, OQ-LS-22
Alternative 3, give per-LOD chunk grids); a tile belongs to the chunk that contains its centroid.

The chunk coordinate of a location at a given LOD is derived directly from its ENU position
(``cx = floor(east_m / chunk_sizes_m[lod])``, ``cy = floor(north_m / chunk_sizes_m[lod])``), so the
Godot streaming manager (OQ-LS-21) computes exactly which chunk file it needs from the
aircraft position — no global manifest is scanned.  This gives the format headroom for
regions far larger than the current UAS dataset (several-hundred-km fast-jet worlds).

Chunk placement uses the same world-origin ENU offset that ``export_gltf`` uses for node
translation (``geodesy.enu_offset``), so chunk coordinates and node positions agree.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

from export_gltf import export_gltf
from geodesy import ecef_from_geodetic, enu_offset
from las_terrain import TerrainTileData
from mosaic_render import MosaicDescriptor

DESCRIPTOR_NAME = "descriptor.json"


def chunk_index(value_m: float, chunk_size_m: float) -> int:
    """Return the chunk index containing ``value_m`` along one axis (floor division).

    A small epsilon (1e-9 chunk-widths ≈ sub-micron) is added before flooring so that a
    position sitting on a chunk boundary — e.g. a tile centroid at the world origin, whose
    ENU east/north is 0 up to floating-point noise — is not flipped into the chunk below by
    a tiny negative rounding error.  Boundaries still belong to the upper chunk (standard
    floor semantics); the epsilon only absorbs noise far below terrain precision.
    """
    return int(math.floor(value_m / chunk_size_m + 1e-9))


def tile_centroid_enu(
    tile: TerrainTileData, world_origin: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Return the tile centroid's (east, north, up) offset in metres from the world origin.

    Uses the same ECEF→ENU offset that :func:`export_gltf.export_gltf` applies to node
    translations, so chunk coordinates align with rendered node positions.
    """
    origin_lat, origin_lon, origin_h = world_origin
    origin_ecef = ecef_from_geodetic(origin_lat, origin_lon, origin_h)
    tile_ecef = ecef_from_geodetic(
        tile.centroid_lat_rad, tile.centroid_lon_rad, tile.centroid_height_m
    )
    enu = enu_offset(origin_lat, origin_lon, tile_ecef, origin_ecef)
    return float(enu[0]), float(enu[1]), float(enu[2])


def tile_chunk_coord(
    tile: TerrainTileData,
    world_origin: tuple[float, float, float],
    chunk_size_m: float,
) -> tuple[int, int]:
    """Return the ``(cx, cy)`` chunk coordinate of a tile from its centroid ENU position."""
    east, north, _up = tile_centroid_enu(tile, world_origin)
    return chunk_index(east, chunk_size_m), chunk_index(north, chunk_size_m)


def chunk_file_name(lod: int, cx: int, cy: int) -> str:
    """Return the descriptor-relative path of a chunk GLB (forward slashes for Godot)."""
    return f"L{lod}/c_{cx}_{cy}.glb"


def assign_tiles_to_chunks(
    tiles: list[TerrainTileData],
    world_origin: tuple[float, float, float],
    chunk_sizes_m: list[float],
) -> dict[tuple[int, int, int], list[int]]:
    """Group tile indices by ``(lod, cx, cy)`` chunk key.

    ``chunk_sizes_m`` is the per-LOD chunk side length (metres); each tile is chunked at its
    own LOD's size (``chunk_sizes_m[tile.lod]``), so per-LOD footprints yield per-LOD chunk
    grids while the ``(lod, cx, cy)`` coordinate addressing is unchanged.
    """
    groups: dict[tuple[int, int, int], list[int]] = {}
    for idx, tile in enumerate(tiles):
        cx, cy = tile_chunk_coord(tile, world_origin, chunk_sizes_m[tile.lod])
        groups.setdefault((tile.lod, cx, cy), []).append(idx)
    return groups


@dataclass
class TerrainChunkDescriptor:
    """Top-level index for a chunked terrain dataset (written as ``descriptor.json``).

    The ``chunks`` list is a sparse occupancy record — one entry per non-empty
    (LOD, chunk) — that lets the loader enumerate what exists (e.g. to preload the coarse
    LODs) without imposing a per-tile manifest.  Individual chunk files are also derivable
    by coordinate via :func:`chunk_file_name`, so the streaming manager never scans it.
    """

    schema_version: int
    world_origin_lat_rad: float
    world_origin_lon_rad: float
    world_origin_height_m: float
    tile_footprints_m: list[float]  # per-LOD tile footprint (m); index = LOD
    chunk_footprints: int           # chunk side in footprints (uniform across LODs)
    chunk_sizes_m: list[float]      # per-LOD chunk side (m) = tile_footprints_m[l] * chunk_footprints
    lod_count: int
    bounds_deg: list[float]  # [lon_min, lat_min, lon_max, lat_max]
    chunks: list[dict]       # [{"lod", "cx", "cy", "file", "tile_count"}, ...]
    # Rendering-LOD policy parameters (terrain_lod_rendering.md); the viewer recomputes the
    # visibility bands from these at the live resolution/FOV.  Empty for the interim uniform build.
    lod_policy: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "TerrainChunkDescriptor":
        return cls(**data)

    def write(self, path: Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=4))

    @classmethod
    def read(cls, path: Path) -> "TerrainChunkDescriptor":
        return cls.from_dict(json.loads(Path(path).read_text()))


def export_chunked_terrain(
    tile_mosaics: list[tuple[TerrainTileData, MosaicDescriptor]],
    out_dir: Path,
    world_origin: tuple[float, float, float],
    *,
    tile_footprints_m: list[float],
    chunk_footprints: int,
    bounds_deg: tuple[float, float, float, float],
    lod_count: int = 7,
    lod_policy: dict | None = None,
) -> TerrainChunkDescriptor:
    """Export tiles as per-(LOD, chunk) GLB files plus a ``descriptor.json`` index.

    Each tile is assigned to the chunk containing its centroid, using its **own LOD's** chunk
    size (``tile_footprints_m[lod] * chunk_footprints``), so per-LOD footprints produce per-LOD
    chunk grids.  All tiles in a chunk are written to one GLB via the existing
    :func:`export_gltf.export_gltf` (per-tile textures, node translation, and the per-tile
    curvature rotation unchanged); every chunk GLB is placed in the single ``world_origin`` ENU
    frame so tiles are consistent across chunk boundaries.

    ``tile_footprints_m`` must have at least ``lod_count`` entries (one footprint per LOD).

    Returns the written :class:`TerrainChunkDescriptor`.

    Raises:
        ValueError: if ``tile_mosaics`` is empty or ``tile_footprints_m`` is too short.
    """
    if not tile_mosaics:
        raise ValueError("tile_mosaics must not be empty")
    if len(tile_footprints_m) < lod_count:
        raise ValueError(
            f"tile_footprints_m has {len(tile_footprints_m)} entries; need >= lod_count={lod_count}"
        )

    chunk_sizes_m = [f * chunk_footprints for f in tile_footprints_m]
    out_dir = Path(out_dir)
    tiles = [tm[0] for tm in tile_mosaics]
    groups = assign_tiles_to_chunks(tiles, world_origin, chunk_sizes_m)

    chunk_records: list[dict] = []
    for (lod, cx, cy), idxs in sorted(groups.items()):
        rel = chunk_file_name(lod, cx, cy)
        chunk_path = out_dir / rel
        chunk_path.parent.mkdir(parents=True, exist_ok=True)
        export_gltf([tile_mosaics[i] for i in idxs], chunk_path, world_origin=world_origin)
        chunk_records.append(
            {"lod": lod, "cx": cx, "cy": cy, "file": rel, "tile_count": len(idxs)}
        )

    descriptor = TerrainChunkDescriptor(
        schema_version=1,
        world_origin_lat_rad=world_origin[0],
        world_origin_lon_rad=world_origin[1],
        world_origin_height_m=world_origin[2],
        tile_footprints_m=list(tile_footprints_m),
        chunk_footprints=chunk_footprints,
        chunk_sizes_m=chunk_sizes_m,
        lod_count=lod_count,
        bounds_deg=list(bounds_deg),
        chunks=chunk_records,
        lod_policy=dict(lod_policy) if lod_policy else {},
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    descriptor.write(out_dir / DESCRIPTOR_NAME)
    return descriptor
