"""export_gltf.py — pygltflib-based GLB export for terrain tiles with a mosaic texture.

Exports TerrainTileData tiles to a binary GLB where all tile mesh primitives share a
single JPEG mosaic texture embedded in the file.  Vertices are deduplicated (one entry
per logical vertex); per-vertex TEXCOORD_0 maps each vertex onto the shared mosaic.

Axis convention:
    glTF X = ENU East,  glTF Y = ENU Up,  glTF Z = −ENU North.

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 Option D (TB-T3)
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pygltflib

from las_terrain import TerrainTileData
from mosaic_render import MosaicDescriptor

# ---------------------------------------------------------------------------
# WGS-84 constants (duplicated locally to keep this module self-contained)
# ---------------------------------------------------------------------------

_WGS84_A: float = 6_378_137.0
_WGS84_F: float = 1.0 / 298.257223563
_WGS84_E2: float = 2.0 * _WGS84_F - _WGS84_F**2

# glTF sampler filter / wrap constants (OpenGL enumerants)
_LINEAR = 9729
_LINEAR_MIPMAP_LINEAR = 9987
_CLAMP_TO_EDGE = 33071


# ---------------------------------------------------------------------------
# Geodetic helpers
# ---------------------------------------------------------------------------

def _ecef_from_geodetic(lat_rad: float, lon_rad: float, h_m: float) -> np.ndarray:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    return np.array([
        (N + h_m) * cos_lat * math.cos(lon_rad),
        (N + h_m) * cos_lat * math.sin(lon_rad),
        (N * (1.0 - _WGS84_E2) + h_m) * sin_lat,
    ], dtype=np.float64)


def _enu_offset(
    ref_lat_rad: float, ref_lon_rad: float,
    point_ecef: np.ndarray, ref_ecef: np.ndarray,
) -> np.ndarray:
    """Return ENU vector from ref to point, expressed in the ENU frame at ref."""
    dp = point_ecef - ref_ecef
    sl, cl = math.sin(ref_lat_rad), math.cos(ref_lat_rad)
    sn, cn = math.sin(ref_lon_rad), math.cos(ref_lon_rad)
    east  = -sn * dp[0] + cn * dp[1]
    north = -sl * cn * dp[0] - sl * sn * dp[1] + cl * dp[2]
    up    =  cl * cn * dp[0] + cl * sn * dp[1] + sl * dp[2]
    return np.array([east, north, up], dtype=np.float64)


def _enu_to_lonlat_deg(
    east_m: np.ndarray,
    north_m: np.ndarray,
    clat_rad: float,
    clon_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """First-order ENU → geodetic approximation (accurate to < 1 m for offsets ≤ 50 km)."""
    sin_lat = math.sin(clat_rad)
    cos_lat = math.cos(clat_rad)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    M = _WGS84_A * (1.0 - _WGS84_E2) / (1.0 - _WGS84_E2 * sin_lat**2) ** 1.5

    lon_deg = math.degrees(clon_rad) + np.degrees(east_m  / (N * cos_lat + 1e-30))
    lat_deg = math.degrees(clat_rad) + np.degrees(north_m / M)
    return lon_deg, lat_deg


# ---------------------------------------------------------------------------
# Binary buffer helpers
# ---------------------------------------------------------------------------

def _align4(buf: bytearray) -> None:
    """Pad buf with zero bytes to the next 4-byte boundary."""
    while len(buf) % 4:
        buf.append(0)


def _add_buffer_view(
    gltf: pygltflib.GLTF2,
    blob: bytearray,
    data: bytes,
    target: int | None,
) -> int:
    """Append data (4-byte aligned) to blob; register a BufferView; return its index."""
    offset = len(blob)
    blob.extend(data)
    _align4(blob)
    bv = pygltflib.BufferView(
        buffer=0,
        byteOffset=offset,
        byteLength=len(data),
        target=target,
    )
    gltf.bufferViews.append(bv)
    return len(gltf.bufferViews) - 1


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def export_gltf(
    tiles: list[TerrainTileData],
    output_path: Path,
    world_origin: tuple[float, float, float] | None = None,
    mosaic: MosaicDescriptor | None = None,
) -> None:
    """Export tiles to a binary GLB with a single embedded JPEG mosaic texture.

    Vertex deduplication: each logical vertex appears once in POSITION, NORMAL,
    and TEXCOORD_0 buffers.  COLOR_0 is not emitted.  All tile mesh primitives
    reference material index 0, which carries the mosaic JPEG as baseColorTexture.

    Node names follow the ``tile_L{lod}_{index:04d}`` convention required by
    TerrainLoader.gd's visibility-range assignment pass.

    Node translations carry the ENU offset of each tile centroid from world_origin
    (glTF axis permutation applied: East→X, Up→Y, −North→Z).

    Root-level scene extras carry ``"liteaerosim_terrain": true`` and world-origin
    fields consumed by live_sim.cpp / GodotEnuProjector configuration.

    Raises
    ------
    ValueError
        If tiles is empty or mosaic is None.
    IOError
        On write failure.
    """
    if not tiles:
        raise ValueError("tiles must not be empty")
    if mosaic is None:
        raise ValueError(
            "mosaic is required for texture-based GLB export; "
            "call mosaic_render.render_mosaic() first"
        )

    if world_origin is None:
        world_origin = (
            tiles[0].centroid_lat_rad,
            tiles[0].centroid_lon_rad,
            tiles[0].centroid_height_m,
        )
    origin_lat, origin_lon, origin_h = world_origin
    origin_ecef = _ecef_from_geodetic(origin_lat, origin_lon, origin_h)

    gltf = pygltflib.GLTF2()
    gltf.asset = pygltflib.Asset(version="2.0", generator="liteaero-sim build_terrain")
    gltf.scene = 0

    blob = bytearray()
    node_indices: list[int] = []

    for tile_idx, tile in enumerate(tiles):
        n_verts = len(tile.vertices)
        n_faces = len(tile.indices)
        if n_verts == 0 or n_faces == 0:
            continue

        # ----------------------------------------------------------------
        # Vertex positions in glTF space: East→X, Up→Y, −North→Z
        # ----------------------------------------------------------------
        verts_enu = np.asarray(tile.vertices, dtype=np.float32)
        gltf_verts = np.column_stack([
            verts_enu[:, 0],   # East   → glTF X
            verts_enu[:, 2],   # Up     → glTF Y
            -verts_enu[:, 1],  # −North → glTF Z
        ]).astype(np.float32)

        # ----------------------------------------------------------------
        # Per-vertex normals (weighted average of adjacent face normals)
        # ----------------------------------------------------------------
        f = np.asarray(tile.indices, dtype=np.int64)
        v0 = gltf_verts[f[:, 0]].astype(np.float64)
        v1 = gltf_verts[f[:, 1]].astype(np.float64)
        v2 = gltf_verts[f[:, 2]].astype(np.float64)
        face_normals = np.cross(v1 - v0, v2 - v0)  # (F, 3) unnormalized

        vertex_normals = np.zeros((n_verts, 3), dtype=np.float64)
        np.add.at(vertex_normals, f[:, 0], face_normals)
        np.add.at(vertex_normals, f[:, 1], face_normals)
        np.add.at(vertex_normals, f[:, 2], face_normals)

        lengths = np.linalg.norm(vertex_normals, axis=1, keepdims=True)
        vertex_normals = np.where(
            lengths > 1e-10,
            vertex_normals / lengths,
            np.array([[0.0, 1.0, 0.0]]),
        ).astype(np.float32)

        # ----------------------------------------------------------------
        # Per-vertex UV from geodetic position relative to mosaic bounds
        # ----------------------------------------------------------------
        east_m  = verts_enu[:, 0].astype(np.float64)
        north_m = verts_enu[:, 1].astype(np.float64)
        lon_deg, lat_deg = _enu_to_lonlat_deg(
            east_m, north_m, tile.centroid_lat_rad, tile.centroid_lon_rad
        )
        lon_range = mosaic.lon_max_deg - mosaic.lon_min_deg
        lat_range = mosaic.lat_max_deg - mosaic.lat_min_deg
        u = (lon_deg - mosaic.lon_min_deg) / lon_range
        v_coord = (mosaic.lat_max_deg - lat_deg) / lat_range  # row 0 = north
        uv = np.column_stack([u, v_coord]).astype(np.float32)

        # ----------------------------------------------------------------
        # Pack POSITION → buffer view + accessor
        # ----------------------------------------------------------------
        pos_bv = _add_buffer_view(
            gltf, blob, gltf_verts.tobytes(), pygltflib.ARRAY_BUFFER
        )
        pos_acc = len(gltf.accessors)
        gltf.accessors.append(pygltflib.Accessor(
            bufferView=pos_bv,
            byteOffset=0,
            componentType=pygltflib.FLOAT,
            count=n_verts,
            type=pygltflib.VEC3,
            min=gltf_verts.min(axis=0).tolist(),
            max=gltf_verts.max(axis=0).tolist(),
        ))

        # ----------------------------------------------------------------
        # Pack NORMAL → buffer view + accessor
        # ----------------------------------------------------------------
        nor_bv = _add_buffer_view(
            gltf, blob, vertex_normals.tobytes(), pygltflib.ARRAY_BUFFER
        )
        nor_acc = len(gltf.accessors)
        gltf.accessors.append(pygltflib.Accessor(
            bufferView=nor_bv,
            byteOffset=0,
            componentType=pygltflib.FLOAT,
            count=n_verts,
            type=pygltflib.VEC3,
        ))

        # ----------------------------------------------------------------
        # Pack TEXCOORD_0 → buffer view + accessor
        # ----------------------------------------------------------------
        uv_bv = _add_buffer_view(
            gltf, blob, uv.tobytes(), pygltflib.ARRAY_BUFFER
        )
        uv_acc = len(gltf.accessors)
        gltf.accessors.append(pygltflib.Accessor(
            bufferView=uv_bv,
            byteOffset=0,
            componentType=pygltflib.FLOAT,
            count=n_verts,
            type=pygltflib.VEC2,
        ))

        # ----------------------------------------------------------------
        # Pack INDEX → buffer view + accessor (flat uint32)
        # ----------------------------------------------------------------
        indices_flat = np.asarray(tile.indices, dtype=np.uint32).flatten()
        idx_bv = _add_buffer_view(
            gltf, blob, indices_flat.tobytes(), pygltflib.ELEMENT_ARRAY_BUFFER
        )
        idx_acc = len(gltf.accessors)
        gltf.accessors.append(pygltflib.Accessor(
            bufferView=idx_bv,
            byteOffset=0,
            componentType=pygltflib.UNSIGNED_INT,
            count=n_faces * 3,
            type=pygltflib.SCALAR,
            min=[int(indices_flat.min())],
            max=[int(indices_flat.max())],
        ))

        # ----------------------------------------------------------------
        # Mesh + Node
        # ----------------------------------------------------------------
        name = f"tile_L{tile.lod}_{tile_idx:04d}"

        mesh = pygltflib.Mesh(
            name=name,
            primitives=[pygltflib.Primitive(
                attributes=pygltflib.Attributes(
                    POSITION=pos_acc,
                    NORMAL=nor_acc,
                    TEXCOORD_0=uv_acc,
                ),
                indices=idx_acc,
                material=0,
            )],
        )
        mesh_idx = len(gltf.meshes)
        gltf.meshes.append(mesh)

        # Node translation: ENU offset of tile centroid from world origin (axis permuted).
        tile_ecef = _ecef_from_geodetic(
            tile.centroid_lat_rad, tile.centroid_lon_rad, tile.centroid_height_m
        )
        enu = _enu_offset(origin_lat, origin_lon, tile_ecef, origin_ecef)
        translation = [float(enu[0]), float(enu[2]), float(-enu[1])]

        node = pygltflib.Node(
            name=name,
            mesh=mesh_idx,
            translation=translation,
        )
        node_idx = len(gltf.nodes)
        gltf.nodes.append(node)
        node_indices.append(node_idx)

    # ----------------------------------------------------------------
    # Embed JPEG image data in the binary blob
    # ----------------------------------------------------------------
    img_bv = _add_buffer_view(gltf, blob, mosaic.jpeg_bytes, None)

    # ----------------------------------------------------------------
    # Single image, sampler, texture, material — shared by all tile primitives
    # ----------------------------------------------------------------
    gltf.images = [
        pygltflib.Image(bufferView=img_bv, mimeType="image/jpeg")
    ]
    gltf.samplers = [
        pygltflib.Sampler(
            magFilter=_LINEAR,
            minFilter=_LINEAR_MIPMAP_LINEAR,
            wrapS=_CLAMP_TO_EDGE,
            wrapT=_CLAMP_TO_EDGE,
        )
    ]
    gltf.textures = [pygltflib.Texture(source=0, sampler=0)]
    gltf.materials = [
        pygltflib.Material(
            pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(
                baseColorTexture=pygltflib.TextureInfo(index=0),
                metallicFactor=0.0,
                roughnessFactor=1.0,
            ),
            doubleSided=True,
            alphaMode="OPAQUE",
        )
    ]

    # ----------------------------------------------------------------
    # Scene with world-origin extras
    # ----------------------------------------------------------------
    scene_extras = {
        "liteaerosim_terrain": True,
        "schema_version": 1,
        "world_origin_lat_rad": origin_lat,
        "world_origin_lon_rad": origin_lon,
        "world_origin_height_m": origin_h,
    }
    gltf.scenes = [pygltflib.Scene(nodes=node_indices, extras=scene_extras)]

    # ----------------------------------------------------------------
    # Finalize buffer and write GLB
    # ----------------------------------------------------------------
    _align4(blob)
    gltf.buffers = [pygltflib.Buffer(byteLength=len(blob))]
    gltf.set_binary_blob(bytes(blob))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    gltf.save(str(output_path))
