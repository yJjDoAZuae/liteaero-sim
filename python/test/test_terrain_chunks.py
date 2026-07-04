"""Tests for terrain_chunks.py — per-region chunk packaging (OQ-LS-20 Alternative 2)."""

from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pygltflib")
pytest.importorskip("scipy")
pytest.importorskip("PIL")

# World origin used across the tests (Santa Barbara-ish, in radians).
_ORIGIN = (0.6008, -2.0937, 3.0)


def _make_tile(lod: int, centroid_lat_rad: float, centroid_lon_rad: float):
    """Build a small flat tile centered at the given geodetic position."""
    from las_terrain import TerrainTileData
    from scipy.spatial import Delaunay

    verts = np.array(
        [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [100.0, 100.0, 0.0],
         [0.0, 100.0, 0.0], [50.0, 50.0, 0.0]],
        dtype=np.float32,
    )
    indices = Delaunay(verts[:, :2]).simplices.astype(np.uint32)
    colors = np.tile([[128, 128, 128]], (len(indices), 1)).astype(np.uint8)
    return TerrainTileData(
        lod=lod,
        centroid_lat_rad=centroid_lat_rad,
        centroid_lon_rad=centroid_lon_rad,
        centroid_height_m=0.0,
        lat_min_rad=centroid_lat_rad - 1e-4,
        lat_max_rad=centroid_lat_rad + 1e-4,
        lon_min_rad=centroid_lon_rad - 1e-4,
        lon_max_rad=centroid_lon_rad + 1e-4,
        height_min_m=0.0,
        height_max_m=0.0,
        vertices=verts,
        indices=indices,
        colors=colors,
    )


def _make_mosaic():
    from PIL import Image
    from mosaic_render import MosaicDescriptor

    buf = io.BytesIO()
    Image.fromarray(np.zeros((4, 4, 3), dtype=np.uint8), "RGB").save(buf, format="JPEG")
    return MosaicDescriptor(
        jpeg_bytes=buf.getvalue(),
        lon_min_deg=-120.0, lat_min_deg=34.0, lon_max_deg=-119.0, lat_max_deg=35.0,
        width_pixels=4, height_pixels=4,
    )


def test_chunk_index_floors_including_negatives() -> None:
    from terrain_chunks import chunk_index

    assert chunk_index(0.0, 100.0) == 0
    assert chunk_index(50.0, 100.0) == 0
    assert chunk_index(150.0, 100.0) == 1
    assert chunk_index(-0.1, 100.0) == -1
    assert chunk_index(-100.0, 100.0) == -1
    assert chunk_index(-100.1, 100.0) == -2


def test_tile_at_origin_maps_to_chunk_zero() -> None:
    from terrain_chunks import tile_centroid_enu, tile_chunk_coord

    tile = _make_tile(0, _ORIGIN[0], _ORIGIN[1])
    east, north, _up = tile_centroid_enu(tile, _ORIGIN)
    assert abs(east) < 1.0 and abs(north) < 1.0
    assert tile_chunk_coord(tile, _ORIGIN, 100.0) == (0, 0)


def test_chunk_coord_sign_follows_direction() -> None:
    """+lat → +north (positive cy); +lon → +east (positive cx); negatives mirror."""
    from terrain_chunks import tile_centroid_enu, tile_chunk_coord

    dlat = 5e-4  # ~3.2 km north
    dlon = 5e-4  # east
    ne = _make_tile(0, _ORIGIN[0] + dlat, _ORIGIN[1] + dlon)
    sw = _make_tile(0, _ORIGIN[0] - dlat, _ORIGIN[1] - dlon)

    e_ne, n_ne, _ = tile_centroid_enu(ne, _ORIGIN)
    assert e_ne > 0 and n_ne > 0
    cx, cy = tile_chunk_coord(ne, _ORIGIN, 1000.0)
    assert cx >= 0 and cy >= 0

    cx2, cy2 = tile_chunk_coord(sw, _ORIGIN, 1000.0)
    assert cx2 < 0 and cy2 < 0


def test_assign_tiles_groups_by_lod_and_coord() -> None:
    from terrain_chunks import assign_tiles_to_chunks, tile_chunk_coord

    tiles = [
        _make_tile(0, _ORIGIN[0], _ORIGIN[1]),            # L0 chunk (0,0)
        _make_tile(0, _ORIGIN[0], _ORIGIN[1]),            # L0 chunk (0,0) again
        _make_tile(0, _ORIGIN[0] + 5e-4, _ORIGIN[1]),     # L0 different chunk
        _make_tile(2, _ORIGIN[0], _ORIGIN[1]),            # L2 chunk (0,0)
    ]
    chunk_sizes = [1000.0] * 7  # uniform sizes here; per-LOD sizing tested separately
    groups = assign_tiles_to_chunks(tiles, _ORIGIN, chunk_sizes)

    key00_l0 = (0,) + tile_chunk_coord(tiles[0], _ORIGIN, 1000.0)
    assert sorted(groups[key00_l0]) == [0, 1]
    # The L2 tile at the origin shares the (cx,cy) but a distinct LOD key.
    key00_l2 = (2,) + tile_chunk_coord(tiles[3], _ORIGIN, 1000.0)
    assert groups[key00_l2] == [3]
    assert sum(len(v) for v in groups.values()) == 4


def test_per_lod_chunk_sizes_partition_independently() -> None:
    """Each LOD is chunked at its own size: a coarse LOD's larger chunk groups tiles that a
    fine LOD's smaller chunk would split."""
    from terrain_chunks import assign_tiles_to_chunks, tile_chunk_coord

    # Two tiles ~450 m apart in north.  With a 400 m chunk they fall in different chunks;
    # with a 2000 m chunk they fall in the same chunk.
    dlat = 450.0 / 6_371_000.0
    a = _make_tile(0, _ORIGIN[0], _ORIGIN[1])
    b = _make_tile(0, _ORIGIN[0] + dlat, _ORIGIN[1])
    c = _make_tile(4, _ORIGIN[0], _ORIGIN[1])
    d = _make_tile(4, _ORIGIN[0] + dlat, _ORIGIN[1])

    chunk_sizes = [400.0, 400.0, 400.0, 400.0, 2000.0, 2000.0, 2000.0]
    groups = assign_tiles_to_chunks([a, b, c, d], _ORIGIN, chunk_sizes)

    # L0 (400 m): the two tiles split into two chunks.
    l0_keys = {k for k in groups if k[0] == 0}
    assert len(l0_keys) == 2
    # L4 (2000 m): the two tiles share one chunk.
    l4_keys = {k for k in groups if k[0] == 4}
    assert len(l4_keys) == 1
    # Sanity: the coordinate helper agrees at each LOD's own size.
    assert tile_chunk_coord(a, _ORIGIN, 400.0) != tile_chunk_coord(b, _ORIGIN, 400.0)
    assert tile_chunk_coord(c, _ORIGIN, 2000.0) == tile_chunk_coord(d, _ORIGIN, 2000.0)


def test_descriptor_roundtrip() -> None:
    from terrain_chunks import TerrainChunkDescriptor

    footprints = [190.0, 670.0, 1900.0, 6700.0, 19000.0, 67000.0, 67000.0]
    d = TerrainChunkDescriptor(
        schema_version=1,
        world_origin_lat_rad=_ORIGIN[0], world_origin_lon_rad=_ORIGIN[1],
        world_origin_height_m=_ORIGIN[2],
        tile_footprints_m=footprints, chunk_footprints=4,
        chunk_sizes_m=[f * 4 for f in footprints],
        lod_count=7, bounds_deg=[-120.0, 34.0, -119.0, 35.0],
        chunks=[{"lod": 0, "cx": 0, "cy": 0, "file": "L0/c_0_0.glb", "tile_count": 3}],
    )
    restored = TerrainChunkDescriptor.from_dict(json.loads(json.dumps(d.to_dict())))
    assert restored == d


def test_export_chunked_writes_files_and_descriptor(tmp_path: Path) -> None:
    from terrain_chunks import export_chunked_terrain, tile_chunk_coord

    mosaic = _make_mosaic()
    tiles = [
        _make_tile(0, _ORIGIN[0], _ORIGIN[1]),           # chunk A
        _make_tile(0, _ORIGIN[0] + 1e-3, _ORIGIN[1]),    # chunk B (far north)
        _make_tile(4, _ORIGIN[0], _ORIGIN[1]),           # coarse LOD, chunk A coord
    ]
    tile_mosaics = [(t, mosaic) for t in tiles]

    out_dir = tmp_path / "terrain_tiles"
    descriptor = export_chunked_terrain(
        tile_mosaics, out_dir, world_origin=_ORIGIN,
        tile_footprints_m=[400.0] * 7, chunk_footprints=1,
        bounds_deg=(-120.0, 34.0, -119.0, 35.0),
    )

    # descriptor.json written with the expected top-level fields.
    desc_path = out_dir / "descriptor.json"
    assert desc_path.exists()
    meta = json.loads(desc_path.read_text())
    assert meta["schema_version"] == 1
    assert meta["chunk_sizes_m"] == [400.0] * 7
    assert meta["tile_footprints_m"] == [400.0] * 7

    # Every listed chunk file exists and is a valid GLB.
    total_tiles = 0
    for rec in descriptor.chunks:
        f = out_dir / rec["file"]
        assert f.exists(), f"missing chunk file {rec['file']}"
        assert f.read_bytes()[:4] == b"glTF", f"{rec['file']} is not a GLB"
        total_tiles += rec["tile_count"]
    assert total_tiles == 3

    # The two L0 tiles are in different chunks (different north position).
    l0_coords = {(tile_chunk_coord(tiles[0], _ORIGIN, 400.0)),
                 (tile_chunk_coord(tiles[1], _ORIGIN, 400.0))}
    assert len(l0_coords) == 2
