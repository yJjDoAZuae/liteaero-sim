"""Tests for export_gltf.py — pygltflib-based textured GLB export (TB-T3).

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D
"""

from __future__ import annotations

import io
import json
import struct
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pygltflib")
pytest.importorskip("scipy")
pytest.importorskip("PIL")


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _make_tile() -> "TerrainTileData":  # type: ignore[name-defined]
    """Build a simple flat tile for GLB export testing."""
    from las_terrain import TerrainTileData
    from scipy.spatial import Delaunay

    size = 100.0
    verts = np.array(
        [
            [0.0,      0.0,    0.0],
            [size,     0.0,    0.0],
            [size,     size,   0.0],
            [0.0,      size,   0.0],
            [size / 2, size / 2, 0.0],
        ],
        dtype=np.float32,
    )
    tri = Delaunay(verts[:, :2])
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


def _make_minimal_mosaic() -> "MosaicDescriptor":  # type: ignore[name-defined]
    """Build a 4×4 JPEG MosaicDescriptor covering the test tile's geographic extent."""
    from PIL import Image

    from mosaic_render import MosaicDescriptor

    img = Image.fromarray(np.zeros((4, 4, 3), dtype=np.uint8), "RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)

    return MosaicDescriptor(
        jpeg_bytes=buf.getvalue(),
        lon_min_deg=-0.01,
        lat_min_deg=-0.01,
        lon_max_deg=0.01,
        lat_max_deg=0.01,
        width_pixels=4,
        height_pixels=4,
    )


def _read_glb_json_chunk(path: Path) -> dict:
    """Extract and parse the JSON chunk from a GLB file."""
    data = path.read_bytes()
    # GLB: 12-byte file header, then chunk 0 (JSON).
    # Chunk 0 header: [chunk_length (4), chunk_type (4), chunk_data]
    chunk_length = struct.unpack("<I", data[12:16])[0]
    json_bytes = data[20 : 20 + chunk_length]
    return json.loads(json_bytes.rstrip(b"\x20"))


# ---------------------------------------------------------------------------
# T1 — GLB magic bytes
# ---------------------------------------------------------------------------

def test_glb_magic_bytes(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    magic = out.read_bytes()[:4]
    assert magic == b"glTF", f"Expected GLB magic b'glTF', got {magic!r}"


# ---------------------------------------------------------------------------
# T2 — POSITION count = unique vertex count (vertex deduplication)
# ---------------------------------------------------------------------------

def test_position_accessor_count_deduped(tmp_path: Path) -> None:
    """After deduplication, POSITION accessor count equals the unique vertex count."""
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    gltf = _read_glb_json_chunk(out)
    n_unique_verts = len(tile.vertices)  # 5 unique vertices

    accessors = gltf.get("accessors", [])
    position_counts = []
    for mesh in gltf.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            pos_idx = primitive.get("attributes", {}).get("POSITION")
            if pos_idx is not None:
                position_counts.append(accessors[pos_idx]["count"])

    assert position_counts, "No POSITION accessor found"
    assert sum(position_counts) == n_unique_verts, (
        f"Expected {n_unique_verts} POSITION entries (unique vertices), "
        f"got {sum(position_counts)}"
    )


# ---------------------------------------------------------------------------
# T3 — scene extras contain "liteaerosim_terrain" key
# ---------------------------------------------------------------------------

def test_extras_present(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    raw = out.read_bytes()
    assert b"liteaerosim_terrain" in raw, (
        "Expected 'liteaerosim_terrain' key in GLB JSON chunk"
    )


# ---------------------------------------------------------------------------
# T4 — TEXCOORD_0 accessor present with count = vertex count
# ---------------------------------------------------------------------------

def test_texcoord0_present(tmp_path: Path) -> None:
    """TEXCOORD_0 accessor is present and has count equal to unique vertex count."""
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    gltf = _read_glb_json_chunk(out)
    n_unique_verts = len(tile.vertices)

    accessors = gltf.get("accessors", [])
    uv_counts = []
    for mesh in gltf.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            uv_idx = primitive.get("attributes", {}).get("TEXCOORD_0")
            if uv_idx is not None:
                uv_counts.append(accessors[uv_idx]["count"])

    assert uv_counts, "No TEXCOORD_0 accessor found"
    assert sum(uv_counts) == n_unique_verts, (
        f"TEXCOORD_0 count {sum(uv_counts)} != vertex count {n_unique_verts}"
    )


# ---------------------------------------------------------------------------
# T5 — single material with baseColorTexture, single image
# ---------------------------------------------------------------------------

def test_single_material_with_texture(tmp_path: Path) -> None:
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    gltf = _read_glb_json_chunk(out)

    materials = gltf.get("materials", [])
    assert len(materials) == 1, f"Expected 1 material, got {len(materials)}"
    pbr = materials[0].get("pbrMetallicRoughness", {})
    assert "baseColorTexture" in pbr, "Material must have pbrMetallicRoughness.baseColorTexture"

    textures = gltf.get("textures", [])
    assert len(textures) == 1, f"Expected 1 texture, got {len(textures)}"

    images = gltf.get("images", [])
    assert len(images) == 1, f"Expected 1 image, got {len(images)}"


# ---------------------------------------------------------------------------
# T6 — COLOR_0 is absent
# ---------------------------------------------------------------------------

def test_color0_absent(tmp_path: Path) -> None:
    """The texture-based export must not emit per-vertex COLOR_0."""
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    gltf = _read_glb_json_chunk(out)
    for mesh in gltf.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            assert "COLOR_0" not in primitive.get("attributes", {}), (
                "COLOR_0 must be absent in texture-based GLB export"
            )


# ---------------------------------------------------------------------------
# T7 — node names encode LOD level (tile_L{N}_...)
# ---------------------------------------------------------------------------

def test_node_names_encode_lod(tmp_path: Path) -> None:
    """Every node name follows the tile_L{N}_... convention."""
    from export_gltf import export_gltf

    tile = _make_tile()
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([tile], out, mosaic=mosaic)

    gltf = _read_glb_json_chunk(out)
    nodes = gltf.get("nodes", [])
    assert nodes, "No nodes in GLB"
    for node in nodes:
        name = node.get("name", "")
        assert name.startswith("tile_L"), (
            f"Node name '{name}' does not start with 'tile_L'"
        )


# ---------------------------------------------------------------------------
# T8 — ValueError when mosaic is None
# ---------------------------------------------------------------------------

def test_raises_without_mosaic(tmp_path: Path) -> None:
    """export_gltf raises ValueError when no mosaic is supplied."""
    from export_gltf import export_gltf

    tile = _make_tile()
    out = tmp_path / "terrain.glb"
    with pytest.raises(ValueError, match="mosaic"):
        export_gltf([tile], out, mosaic=None)
