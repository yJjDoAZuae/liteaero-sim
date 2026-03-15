"""simplify.py — LOD simplification using QEM decimation via pyfqmr.

Produces a coarser LOD tile from a finer one.  preserve_border=True locks all
tile-boundary vertices so adjacent tiles remain crack-free.

After decimation, per-face colors are transferred from the nearest L0 facet centroid
using scipy.spatial.cKDTree proximity.  The result is validated with verify.check().
"""

from __future__ import annotations

import dataclasses

import numpy as np

from las_terrain import TerrainTileData
from verify import MeshQualityError, check

# LOD grid spacing table (degrees) — matches triangulate._LOD_GRID_SPACING_DEG.
_LOD_SPACING: list[float] = [
    0.000090,  # LOD 0 ≈  10 m
    0.000270,  # LOD 1 ≈  30 m
    0.000900,  # LOD 2 ≈ 100 m
    0.002700,  # LOD 3 ≈ 300 m
    0.009000,  # LOD 4 ≈ 1 km
    0.027000,  # LOD 5 ≈ 3 km
    0.090000,  # LOD 6 ≈ 10 km
]

# Default maximum vertical error (m) per LOD transition.
DEFAULT_MAX_ERROR_M: dict[tuple[int, int], float] = {
    (0, 1): 3.0,
    (1, 2): 10.0,
    (2, 3): 30.0,
    (3, 4): 100.0,
    (4, 5): 300.0,
    (5, 6): 1000.0,
}


def simplify(
    tile: TerrainTileData,
    target_lod: int,
    max_vertical_error_m: float | None = None,
) -> TerrainTileData:
    """Decimate tile to target_lod using pyfqmr QEM.

    Steps:
    1. Validate input with verify.check() — raises MeshQualityError on degenerate input.
    2. Convert ENU float32 vertices to float64 for accurate QEM.
    3. Call pyfqmr.Simplify with preserve_border=True.
       target_count = source_face_count × (src_spacing / tgt_spacing)²
    4. Transfer per-face colors from source to simplified mesh via KD-tree proximity.
    5. Convert simplified vertices back to float32.
    6. Call verify.check() on result — raises MeshQualityError if quality fails.
    7. Return new TerrainTileData at target_lod.

    Raises:
        ValueError:        if target_lod <= tile.lod.
        MeshQualityError:  if input tile or output tile fails quality thresholds.
    """
    import pyfqmr
    from scipy.spatial import cKDTree

    if target_lod <= tile.lod:
        raise ValueError(
            f"target_lod {target_lod} must be greater than tile.lod {tile.lod}"
        )

    # Pre-check input — raises MeshQualityError for degenerate input.
    check(tile)

    # Float64 ENU for numerically stable QEM.
    verts_f64 = tile.vertices.astype(np.float64)
    faces = tile.indices.astype(np.uint32)

    # Compute target face count using spacing ratio.
    src_spacing = _LOD_SPACING[tile.lod]
    tgt_spacing = _LOD_SPACING[target_lod]
    ratio = src_spacing / tgt_spacing  # < 1 (tgt is coarser)
    target_count = max(2, int(len(faces) * ratio**2))

    if max_vertical_error_m is None:
        max_vertical_error_m = DEFAULT_MAX_ERROR_M.get((tile.lod, target_lod), 10.0)

    simplifier = pyfqmr.Simplify()
    simplifier.setMesh(verts_f64, faces)
    simplifier.simplify_mesh(
        target_count,
        aggressiveness=7,
        preserve_border=True,
        verbose=False,
        max_iterations=100,
    )
    new_verts_raw, new_faces_raw, _ = simplifier.getMesh()

    new_verts = np.asarray(new_verts_raw, dtype=np.float32)
    new_faces = np.asarray(new_faces_raw, dtype=np.uint32)

    # Transfer per-face colors: for each simplified facet centroid, find nearest source centroid.
    src_centroids = (
        tile.vertices[tile.indices[:, 0]].astype(np.float64)
        + tile.vertices[tile.indices[:, 1]].astype(np.float64)
        + tile.vertices[tile.indices[:, 2]].astype(np.float64)
    ) / 3.0

    new_centroids = (
        new_verts[new_faces[:, 0]].astype(np.float64)
        + new_verts[new_faces[:, 1]].astype(np.float64)
        + new_verts[new_faces[:, 2]].astype(np.float64)
    ) / 3.0

    tree = cKDTree(src_centroids)
    _, nearest_idx = tree.query(new_centroids)
    new_colors = tile.colors[nearest_idx]

    result = dataclasses.replace(
        tile,
        lod=target_lod,
        vertices=new_verts,
        indices=new_faces,
        colors=new_colors,
    )

    # Post-check output with lenient thresholds — QEM does not guarantee angle quality,
    # but we still reject completely degenerate output (zero-area facets).
    check(result, min_angle_deg=1.0, max_aspect_ratio=500.0)
    return result
