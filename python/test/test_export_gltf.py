"""Tests for export_gltf.py — pygltflib-based textured GLB export (TB-T3).

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D
"""

from __future__ import annotations

import io
import json
import math
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

def _make_tile(
    centroid_lat_rad: float = 0.0,
    centroid_lon_rad: float = 0.0,
    centroid_height_m: float = 0.0,
    vertices: "np.ndarray | None" = None,
) -> "TerrainTileData":  # type: ignore[name-defined]
    """Build a simple flat tile for GLB export testing."""
    from las_terrain import TerrainTileData
    from scipy.spatial import Delaunay

    size = 100.0
    if vertices is None:
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
    else:
        verts = np.asarray(vertices, dtype=np.float32)
    tri = Delaunay(verts[:, :2])
    indices = tri.simplices.astype(np.uint32)
    colors = np.tile([[200, 100, 50]], (len(indices), 1)).astype(np.uint8)

    return TerrainTileData(
        lod=0,
        centroid_lat_rad=centroid_lat_rad,
        centroid_lon_rad=centroid_lon_rad,
        centroid_height_m=centroid_height_m,
        lat_min_rad=centroid_lat_rad - 0.001,
        lat_max_rad=centroid_lat_rad + 0.001,
        lon_min_rad=centroid_lon_rad - 0.001,
        lon_max_rad=centroid_lon_rad + 0.001,
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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

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
    export_gltf([(tile, mosaic)], out)

    gltf = _read_glb_json_chunk(out)
    nodes = gltf.get("nodes", [])
    assert nodes, "No nodes in GLB"
    for node in nodes:
        name = node.get("name", "")
        assert name.startswith("tile_L"), (
            f"Node name '{name}' does not start with 'tile_L'"
        )


# ---------------------------------------------------------------------------
# T8 — ValueError when no tiles are supplied
# ---------------------------------------------------------------------------

def test_raises_without_tiles(tmp_path: Path) -> None:
    """export_gltf raises ValueError when the tile list is empty."""
    from export_gltf import export_gltf

    out = tmp_path / "terrain.glb"
    with pytest.raises(ValueError, match="tile_mosaics"):
        export_gltf([], out)


# ---------------------------------------------------------------------------
# T9 — per-tile node rotation places centroid-relative vertices in the
#      world-origin ENU frame (IP-LV-1, live_sim_view.md Issue 8)
# ---------------------------------------------------------------------------

def _quaternion_to_matrix(quat: list[float]) -> np.ndarray:
    """glTF ``[x, y, z, w]`` quaternion → 3×3 rotation matrix."""
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def test_per_tile_rotation_places_vertices_in_world_frame(tmp_path: Path) -> None:
    """translation + R·v_centroid must equal the vertex's exact world-origin ECEF→ENU
    position (in glTF axes), so a tile ~20 km from the origin is not left in its own
    tilted tangent plane."""
    from export_gltf import export_gltf, _GLTF_FROM_ENU
    from geodesy import ecef_from_geodetic, enu_offset, enu_to_ecef_rotation

    origin_lat = math.radians(34.40)
    origin_lon = math.radians(-119.70)
    origin_h = 12.0

    # Tile centroid ~0.18° north / 0.20° east of the world origin (~20 km) — far enough that
    # translation-only placement would leave a multi-metre curvature gap.
    c_lat = math.radians(34.58)
    c_lon = math.radians(-119.50)
    c_h = 47.0

    # Non-trivial vertices in the tile's centroid-tangent ENU frame (east, north, up).
    verts_enu = np.array([
        [0.0,    0.0,   0.0],
        [400.0,  0.0,   5.0],
        [0.0,    400.0, -3.0],
        [400.0,  400.0, 8.0],
        [200.0,  200.0, 2.0],
    ], dtype=np.float32)

    tile = _make_tile(
        centroid_lat_rad=c_lat,
        centroid_lon_rad=c_lon,
        centroid_height_m=c_h,
        vertices=verts_enu,
    )
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([(tile, mosaic)], out, world_origin=(origin_lat, origin_lon, origin_h))

    gltf = _read_glb_json_chunk(out)
    node = gltf["nodes"][0]
    translation = np.array(node["translation"], dtype=np.float64)
    rotation = _quaternion_to_matrix(node["rotation"])

    origin_ecef = ecef_from_geodetic(origin_lat, origin_lon, origin_h)
    centroid_ecef = ecef_from_geodetic(c_lat, c_lon, c_h)
    centroid_enu_to_ecef = enu_to_ecef_rotation(c_lat, c_lon)

    for v_enu in verts_enu.astype(np.float64):
        # Exact world-origin ECEF→ENU of this vertex, then permuted to glTF axes.
        p_ecef = centroid_ecef + centroid_enu_to_ecef @ v_enu
        v_world_enu = enu_offset(origin_lat, origin_lon, p_ecef, origin_ecef)
        expected_gltf = _GLTF_FROM_ENU @ v_world_enu

        # Placement realized by the glTF node (vertex stored in glTF axes).
        v_gltf = _GLTF_FROM_ENU @ v_enu
        reconstructed = translation + rotation @ v_gltf

        assert np.allclose(reconstructed, expected_gltf, atol=1e-2), (
            f"vertex {v_enu} mis-placed: got {reconstructed}, expected {expected_gltf}"
        )


def test_per_tile_rotation_identity_at_origin(tmp_path: Path) -> None:
    """A tile whose centroid is the world origin gets an identity rotation."""
    from export_gltf import export_gltf

    tile = _make_tile()  # centroid at (0, 0, 0)
    mosaic = _make_minimal_mosaic()
    out = tmp_path / "terrain.glb"
    export_gltf([(tile, mosaic)], out, world_origin=(0.0, 0.0, 0.0))

    gltf = _read_glb_json_chunk(out)
    rotation = gltf["nodes"][0]["rotation"]
    assert np.allclose(rotation, [0.0, 0.0, 0.0, 1.0], atol=1e-9), (
        f"Expected identity quaternion at origin, got {rotation}"
    )
