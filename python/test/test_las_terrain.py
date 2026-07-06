"""Tests for las_terrain.py — .las_terrain binary round-trip (Step 14)."""

from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np
import pytest

from las_terrain import (
    TerrainTileData,
    read_las_terrain,
    read_las_terrain_footprints,
    write_las_terrain,
)

_MAGIC = 0x4C415354  # "LAST"


def _make_tile(lod: int = 0, n_verts: int = 3, n_facets: int = 1) -> TerrainTileData:
    """Build a minimal synthetic TerrainTileData for testing."""
    vertices = np.array(
        [[float(i) * 10.0, float(i) * 5.0, float(i) * 2.0] for i in range(n_verts)],
        dtype=np.float32,
    )
    indices = np.zeros((n_facets, 3), dtype=np.uint32)
    for f in range(n_facets):
        indices[f] = [f % n_verts, (f + 1) % n_verts, (f + 2) % n_verts]
    colors = np.full((n_facets, 3), 128, dtype=np.uint8)

    return TerrainTileData(
        lod=lod,
        centroid_lat_rad=0.523598775598299,  # 30 deg
        centroid_lon_rad=1.570796326794897,  # 90 deg
        centroid_height_m=250.0,
        lat_min_rad=0.52,
        lat_max_rad=0.53,
        lon_min_rad=1.56,
        lon_max_rad=1.58,
        height_min_m=200.0,
        height_max_m=300.0,
        vertices=vertices,
        indices=indices,
        colors=colors,
    )


# T1 — write one tile, read back; vertex values match to float32 precision.
def test_round_trip_single_tile(tmp_path: Path) -> None:
    tile = _make_tile(lod=0, n_verts=4, n_facets=2)
    path = tmp_path / "single.las_terrain"

    write_las_terrain(path, [tile])
    recovered = read_las_terrain(path)

    assert len(recovered) == 1
    rt = recovered[0]
    assert rt.lod == tile.lod
    np.testing.assert_array_almost_equal(rt.vertices, tile.vertices, decimal=5)
    np.testing.assert_array_equal(rt.indices, tile.indices)
    np.testing.assert_array_equal(rt.colors, tile.colors)
    assert abs(rt.centroid_lat_rad - tile.centroid_lat_rad) < 1e-12
    assert abs(rt.centroid_lon_rad - tile.centroid_lon_rad) < 1e-12


# T2 — two tiles; tile count and centroids preserved.
def test_round_trip_two_tiles(tmp_path: Path) -> None:
    tile0 = _make_tile(lod=0, n_verts=3, n_facets=1)
    tile1 = _make_tile(lod=2, n_verts=6, n_facets=4)
    tile1.centroid_lat_rad = 0.1
    tile1.centroid_lon_rad = 0.2
    path = tmp_path / "two.las_terrain"

    write_las_terrain(path, [tile0, tile1])
    recovered = read_las_terrain(path)

    assert len(recovered) == 2
    assert recovered[0].lod == 0
    assert recovered[1].lod == 2
    assert abs(recovered[1].centroid_lat_rad - 0.1) < 1e-12
    assert abs(recovered[1].centroid_lon_rad - 0.2) < 1e-12


# T3 — first 4 bytes of file are 0x4C415354 ("LAST").
def test_magic_bytes(tmp_path: Path) -> None:
    path = tmp_path / "magic.las_terrain"
    write_las_terrain(path, [_make_tile()])
    data = path.read_bytes()
    magic = struct.unpack("<I", data[:4])[0]
    assert magic == _MAGIC, f"Expected magic {_MAGIC:#010x}, got {magic:#010x}"


# T4 — metadata JSON uses nested "centroid" and "bounds" objects matching the C++ schema.
def test_metadata_json_schema(tmp_path: Path) -> None:
    tile = _make_tile(lod=2)
    path = tmp_path / "schema.las_terrain"
    write_las_terrain(path, [tile])

    data = path.read_bytes()
    # Skip 40-byte file header (magic + version + tile_count + 7 × float32 lod_footprints_m).
    offset = 40
    meta_len = struct.unpack_from("<I", data, offset)[0]
    offset += 4
    meta = json.loads(data[offset : offset + meta_len])

    assert "centroid" in meta, "metadata must have a 'centroid' object"
    assert "bounds" in meta, "metadata must have a 'bounds' object"
    assert "latitude_rad" in meta["centroid"]
    assert "longitude_rad" in meta["centroid"]
    assert "height_wgs84_m" in meta["centroid"]
    assert abs(meta["centroid"]["latitude_rad"] - tile.centroid_lat_rad) < 1e-12
    assert "lat_min_rad" in meta["bounds"]
    assert "lat_max_rad" in meta["bounds"]
    assert "lon_min_rad" in meta["bounds"]
    assert "lon_max_rad" in meta["bounds"]
    assert "height_min_m" in meta["bounds"]
    assert "height_max_m" in meta["bounds"]


# T5 — read file with format_version != 1 raises ValueError.
def test_schema_version_mismatch_raises(tmp_path: Path) -> None:
    path = tmp_path / "bad_version.las_terrain"
    # Write a valid file then patch the version byte to 99.
    write_las_terrain(path, [_make_tile()])
    data = bytearray(path.read_bytes())
    struct.pack_into("<I", data, 4, 99)  # overwrite format_version at offset 4
    path.write_bytes(bytes(data))

    with pytest.raises(ValueError, match="format_version"):
        read_las_terrain(path)


# T6 — per-LOD footprints written to the header round-trip through read_las_terrain_footprints,
# and the tile payload is unaffected by the added header block.
def test_lod_footprints_round_trip(tmp_path: Path) -> None:
    footprints = [286.0, 668.0, 1909.0, 6682.0, 19092.0, 66822.0, 222739.0]
    path = tmp_path / "footprints.las_terrain"

    write_las_terrain(path, [_make_tile(lod=0), _make_tile(lod=1)], lod_footprints_m=footprints)

    recovered = read_las_terrain_footprints(path)
    assert len(recovered) == 7
    for got, exp in zip(recovered, footprints):
        assert abs(got - exp) < 1e-3, f"footprint {got} != {exp}"

    # The tile payload still reads correctly past the 40-byte header.
    tiles = read_las_terrain(path)
    assert len(tiles) == 2
    assert tiles[0].lod == 0
    assert tiles[1].lod == 1


# T7 — writing without footprints records an all-zero array ("not recorded").
def test_lod_footprints_default_all_zero(tmp_path: Path) -> None:
    path = tmp_path / "no_footprints.las_terrain"
    write_las_terrain(path, [_make_tile()])
    assert read_las_terrain_footprints(path) == [0.0] * 7


# T8 — a footprint list of the wrong length is rejected.
def test_lod_footprints_wrong_length_raises(tmp_path: Path) -> None:
    path = tmp_path / "bad_footprints.las_terrain"
    with pytest.raises(ValueError, match="7 entries"):
        write_las_terrain(path, [_make_tile()], lod_footprints_m=[1.0, 2.0, 3.0])
