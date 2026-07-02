"""export_gltf.py — pygltflib-based GLB export for terrain tiles with per-tile textures.

Exports TerrainTileData tiles to a binary GLB where each tile mesh primitive carries its
own JPEG texture covering only that tile's geographic footprint.  Vertices are deduplicated
(one entry per logical vertex); per-vertex TEXCOORD_0 maps each vertex to the tile's
own mosaic bounds (u=0 at tile west edge, u=1 at tile east edge; v=0 at tile north edge).

Axis convention:
    glTF X = ENU East,  glTF Y = ENU Up,  glTF Z = −ENU North.

Design authority: docs/architecture/terrain_build.md §OQ-TB-5 (per-tile texture variant)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pygltflib

from geodesy import ecef_from_geodetic, enu_offset, enu_to_ecef_rotation, enu_to_lonlat_deg
from geometry import rotation_matrix_to_quaternion
from las_terrain import TerrainTileData
from mosaic_render import MosaicDescriptor

# glTF sampler filter / wrap constants (OpenGL enumerants)
_LINEAR = 9729
_LINEAR_MIPMAP_LINEAR = 9987
_CLAMP_TO_EDGE = 33071

# Permutation mapping an ENU vector (east, north, up) to the glTF axis
# convention (X=east, Y=up, Z=−north).  Orthogonal, det = +1.
_GLTF_FROM_ENU: np.ndarray = np.array(
    [
        [1.0,  0.0, 0.0],   # glTF X =  east
        [0.0,  0.0, 1.0],   # glTF Y =  up
        [0.0, -1.0, 0.0],   # glTF Z = −north
    ],
    dtype=np.float64,
)


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
    tile_mosaics: list[tuple[TerrainTileData, MosaicDescriptor]],
    output_path: Path,
    world_origin: tuple[float, float, float] | None = None,
) -> None:
    """Export tiles to a binary GLB with one embedded JPEG texture per tile.

    Each tile mesh primitive carries its own material referencing a JPEG texture
    covering only that tile's geographic footprint.  Vertex deduplication: each
    logical vertex appears once in POSITION, NORMAL, and TEXCOORD_0 buffers.
    COLOR_0 is not emitted.  TEXCOORD_0 maps u=0/1 to the tile's west/east edge
    and v=0/1 to the tile's north/south edge.

    Node names follow the ``tile_L{lod}_{index:04d}`` convention required by
    TerrainLoader.gd's visibility-range assignment pass.

    Node translations carry the ENU offset of each tile centroid from world_origin
    (glTF axis permutation applied: East→X, Up→Y, −North→Z).  Node rotations carry the
    orientation of each tile's centroid-tangent ENU frame relative to the world-origin ENU
    frame, so translation + rotation places each tile's centroid-relative vertices exactly in
    the single world-origin frame the aircraft is projected into (reconciles Earth curvature
    across tiles — live_sim_view.md Issue 8).

    Root-level scene extras carry ``"liteaerosim_terrain": true`` and world-origin
    fields consumed by live_sim.cpp / GodotEnuProjector configuration.

    Raises
    ------
    ValueError
        If tile_mosaics is empty.
    IOError
        On write failure.
    """
    if not tile_mosaics:
        raise ValueError("tile_mosaics must not be empty")

    if world_origin is None:
        first_tile = tile_mosaics[0][0]
        world_origin = (
            first_tile.centroid_lat_rad,
            first_tile.centroid_lon_rad,
            first_tile.centroid_height_m,
        )
    origin_lat, origin_lon, origin_h = world_origin
    origin_ecef = ecef_from_geodetic(origin_lat, origin_lon, origin_h)
    origin_enu_to_ecef = enu_to_ecef_rotation(origin_lat, origin_lon)

    gltf = pygltflib.GLTF2()
    gltf.asset = pygltflib.Asset(version="2.0", generator="liteaero-sim build_terrain")
    gltf.scene = 0

    blob = bytearray()
    node_indices: list[int] = []

    for tile_idx, (tile, mosaic) in enumerate(tile_mosaics):
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
        # Per-vertex UV from geodetic position relative to this tile's mosaic bounds.
        # The mosaic covers exactly this tile's bbox so u ∈ [0, 1] across the tile.
        # ----------------------------------------------------------------
        east_m  = verts_enu[:, 0].astype(np.float64)
        north_m = verts_enu[:, 1].astype(np.float64)
        lon_deg, lat_deg = enu_to_lonlat_deg(
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

        # ----------------------------------------------------------------
        # Embed per-tile JPEG image → image, sampler, texture, material
        # ----------------------------------------------------------------
        img_bv = _add_buffer_view(gltf, blob, mosaic.jpeg_bytes, None)
        img_idx = len(gltf.images)
        gltf.images.append(pygltflib.Image(bufferView=img_bv, mimeType="image/jpeg"))

        if not gltf.samplers:
            gltf.samplers = [pygltflib.Sampler(
                magFilter=_LINEAR,
                minFilter=_LINEAR_MIPMAP_LINEAR,
                wrapS=_CLAMP_TO_EDGE,
                wrapT=_CLAMP_TO_EDGE,
            )]

        tex_idx = len(gltf.textures)
        gltf.textures.append(pygltflib.Texture(source=img_idx, sampler=0))

        mat_idx = len(gltf.materials)
        gltf.materials.append(pygltflib.Material(
            pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(
                baseColorTexture=pygltflib.TextureInfo(index=tex_idx),
                metallicFactor=0.0,
                roughnessFactor=1.0,
            ),
            doubleSided=True,
            alphaMode="OPAQUE",
        ))

        mesh = pygltflib.Mesh(
            name=name,
            primitives=[pygltflib.Primitive(
                attributes=pygltflib.Attributes(
                    POSITION=pos_acc,
                    NORMAL=nor_acc,
                    TEXCOORD_0=uv_acc,
                ),
                indices=idx_acc,
                material=mat_idx,
            )],
        )
        mesh_idx = len(gltf.meshes)
        gltf.meshes.append(mesh)

        # Node translation: ENU offset of tile centroid from world origin (axis permuted).
        tile_ecef = ecef_from_geodetic(
            tile.centroid_lat_rad, tile.centroid_lon_rad, tile.centroid_height_m
        )
        enu = enu_offset(origin_lat, origin_lon, tile_ecef, origin_ecef)
        translation = [float(enu[0]), float(enu[2]), float(-enu[1])]

        # Node rotation: the vertices are stored in this tile's centroid-tangent ENU frame,
        # but the aircraft lives in the single world-origin ENU frame.  R maps the centroid-ENU
        # basis onto the world-origin-ENU basis (R = R_originᵀ · R_centroid); conjugating by the
        # glTF axis permutation expresses it in glTF axes so that
        # v_world = translation + R_gltf · v_centroid is exactly the vertex's world-origin
        # ECEF→ENU position.  Without it, translation-only placement leaves each tile in its own
        # tilted tangent plane (Earth curvature) → stacked terrain surfaces (Issue 8).
        centroid_enu_to_ecef = enu_to_ecef_rotation(
            tile.centroid_lat_rad, tile.centroid_lon_rad
        )
        rotation_enu = origin_enu_to_ecef.T @ centroid_enu_to_ecef
        rotation_gltf = _GLTF_FROM_ENU @ rotation_enu @ _GLTF_FROM_ENU.T
        rotation = rotation_matrix_to_quaternion(rotation_gltf)

        node = pygltflib.Node(
            name=name,
            mesh=mesh_idx,
            translation=translation,
            rotation=rotation,
        )
        node_idx = len(gltf.nodes)
        gltf.nodes.append(node)
        node_indices.append(node_idx)

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
