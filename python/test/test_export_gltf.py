"""Tests for export_gltf.py — trimesh-based GLB export (Step 20)."""

from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("trimesh")


def _make_tile(n_facets: int = 4) -> "TerrainTileData":  # type: ignore[name-defined]
    """Build a simple flat tile for GLB export testing."""
    from las_terrain import TerrainTileData
    from scipy.spatial import Delaunay

    pytest.importorskip("scipy")
    size = 100.0
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [size, 0.0, 0.0],
            [size, size, 0.0],
            [0.0, size, 0.0],
            [size / 2, size / 2, 0.0],
        ],
        dtype=np.float32,
    )
    pts2d = verts[:, :2]
    from scipy.spatial import Delaunay as DT

    tri = DT(pts2d)
    indices = tri.simplices.astype(np.uint32)
    colors = np.tile([[200, 100, 50]], (len(indices), 1)).astype(np.uint8)

    return TerrainTileData(
        lod=0,
        centroid_lat_rad=0.0,
        centroid_lon_rad=0.0,
        centroid_height_m=0.0,
        lat_min_rad=-0.001,
        lat_max_rad=0.001,
        lon_min_rad=-0.001,
        lon_max_rad=0.001,
        height_min_m=0.0,
        height_max_m=0.0,
        vertices=verts,
        indices=indices,
        colors=colors,
    )


def _read_glb_json_chunk(path: Path) -> dict:
    """Extract and parse the JSON chunk from a GLB file."""
    data = path.read_bytes()
    # GLB: 12-byte file header, then chunk 0 (JSON).
    # Chunk 0 header starts at byte 12: [chunk_length (4), chunk_type (4), chunk_data]
    chunk_length = struct.unpack("<I", data[12:16])[0]
    json_bytes = data[20 : 20 + chunk_length]
    return json.loads(json_bytes.rstrip(b"\x20"))


# T1 — first 4 bytes of output file are b"glTF" (0x67 0x6C 0x54 0x46).
def test_glb_magic_bytes(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out)

    magic = out.read_bytes()[:4]
    assert magic == b"glTF", f"Expected GLB magic b'glTF', got {magic!r}"


# T2 — POSITION accessor element count == 3 × facet count.
def test_position_accessor_count(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out)

    gltf = _read_glb_json_chunk(out)
    n_facets = len(tile.indices)
    expected_vertex_count = 3 * n_facets

    # Sum POSITION accessor counts across all meshes.
    accessors = gltf.get("accessors", [])
    position_counts = []
    for mesh in gltf.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            pos_idx = primitive.get("attributes", {}).get("POSITION")
            if pos_idx is not None:
                position_counts.append(accessors[pos_idx]["count"])

    total_positions = sum(position_counts)
    assert total_positions == expected_vertex_count, (
        f"Expected {expected_vertex_count} POSITION vertices (3 × {n_facets} facets), "
        f"got {total_positions}"
    )


# T3 — GLB JSON chunk contains "liteaerosim_terrain" key.
def test_extras_present(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out)

    raw = out.read_bytes()
    # Quick text search — the key must appear somewhere in the JSON chunk.
    assert b"liteaerosim_terrain" in raw, (
        "Expected 'liteaerosim_terrain' key in GLB JSON chunk"
    )
