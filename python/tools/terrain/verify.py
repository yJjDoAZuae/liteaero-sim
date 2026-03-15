"""verify.py — Python reimplementation of MeshQualityVerifier.

Angles and edge lengths are computed in ECEF space, matching the C++ MeshQualityVerifier
implementation.  No pybind11 or C++ interop required.

This runs offline in the ingestion pipeline (not in the simulation loop), so pure-Python
numpy operations are acceptable.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass

import numpy as np

from las_terrain import TerrainTileData

_WGS84_A: float = 6_378_137.0
_WGS84_F: float = 1.0 / 298.257223563
_WGS84_E2: float = 2.0 * _WGS84_F - _WGS84_F**2


@dataclass
class MeshQualityReport:
    """Mesh quality metrics computed by verify()."""

    min_interior_angle_deg: float = 180.0
    max_interior_angle_deg: float = 0.0
    mean_interior_angle_deg: float = 60.0
    min_edge_length_m: float = float("inf")
    max_edge_length_m: float = 0.0
    max_aspect_ratio: float = 0.0
    degenerate_facet_count: int = 0
    non_manifold_edge_count: int = 0
    open_boundary_edge_count: int = 0


class MeshQualityError(ValueError):
    """Raised by check() when a tile fails quality thresholds."""


def _enu_to_ecef(
    enu: np.ndarray,
    clat_rad: float,
    clon_rad: float,
    ch_m: float,
) -> np.ndarray:
    """Convert ENU offsets (N, 3) to ECEF positions (N, 3), float64."""
    sin_lat = math.sin(clat_rad)
    cos_lat = math.cos(clat_rad)
    sin_lon = math.sin(clon_rad)
    cos_lon = math.cos(clon_rad)

    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    cx = (N + ch_m) * cos_lat * cos_lon
    cy = (N + ch_m) * cos_lat * sin_lon
    cz = (N * (1.0 - _WGS84_E2) + ch_m) * sin_lat

    # ENU-to-ECEF rotation: columns are the East, North, Up unit vectors.
    R = np.array(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=np.float64,
    )

    enu_d = enu.astype(np.float64)
    ecef = enu_d @ R.T + np.array([cx, cy, cz], dtype=np.float64)
    return ecef


def verify(tile: TerrainTileData) -> MeshQualityReport:
    """Compute quality metrics for tile.

    Angles and ECEF edge lengths are computed using numpy vectorised operations.
    Edge valence is counted via collections.Counter on sorted (i, j) edge pairs.
    """
    report = MeshQualityReport()

    if len(tile.indices) == 0:
        return report

    # Convert all vertices to ECEF float64.
    ecef = _enu_to_ecef(tile.vertices, tile.centroid_lat_rad, tile.centroid_lon_rad, tile.centroid_height_m)

    i0 = tile.indices[:, 0]
    i1 = tile.indices[:, 1]
    i2 = tile.indices[:, 2]

    v0 = ecef[i0]  # (F, 3)
    v1 = ecef[i1]
    v2 = ecef[i2]

    # Edge vectors (F, 3)
    e01 = v1 - v0
    e12 = v2 - v1
    e20 = v0 - v2

    # Edge lengths (F,)
    l01 = np.linalg.norm(e01, axis=1)
    l12 = np.linalg.norm(e12, axis=1)
    l20 = np.linalg.norm(e20, axis=1)

    all_lengths = np.concatenate([l01, l12, l20])
    report.min_edge_length_m = float(np.min(all_lengths))
    report.max_edge_length_m = float(np.max(all_lengths))

    # Facet areas: 0.5 * |e01 × e02| where e02 = v2 - v0 = -e20
    e02 = -e20
    cross = np.cross(e01, e02)  # (F, 3)
    areas = 0.5 * np.linalg.norm(cross, axis=1)  # (F,)

    # Degenerate facets: area < 1e-6 m²
    report.degenerate_facet_count = int(np.sum(areas < 1.0e-6))

    # Interior angles via arc-cosine of normalised edge dot products.
    inv_l01 = 1.0 / (l01 + 1.0e-30)
    inv_l12 = 1.0 / (l12 + 1.0e-30)
    inv_l20 = 1.0 / (l20 + 1.0e-30)
    inv_l02 = 1.0 / (np.linalg.norm(e02, axis=1) + 1.0e-30)

    e01_n = e01 * inv_l01[:, None]
    e02_n = e02 * inv_l02[:, None]
    e12_n = e12 * inv_l12[:, None]

    # Angle at v0: between e01 and e02
    cos_a0 = np.clip(np.sum(e01_n * e02_n, axis=1), -1.0, 1.0)
    # Angle at v1: between -e01 and e12
    cos_a1 = np.clip(np.sum(-e01_n * e12_n, axis=1), -1.0, 1.0)
    # Angle at v2: between -e12 and -e02  (= e12 dot e02 normalised)
    cos_a2 = np.clip(np.sum(-e12_n * (-e02_n), axis=1), -1.0, 1.0)

    angles_rad = np.concatenate([np.arccos(cos_a0), np.arccos(cos_a1), np.arccos(cos_a2)])
    angles_deg = np.degrees(angles_rad)

    report.min_interior_angle_deg = float(np.min(angles_deg))
    report.max_interior_angle_deg = float(np.max(angles_deg))
    report.mean_interior_angle_deg = float(np.mean(angles_deg))

    # Aspect ratio: max_edge² / (2 * area)  (consistent with C++ MeshQualityVerifier)
    max_edges = np.maximum(np.maximum(l01, l12), l20)
    aspect_ratios = max_edges**2 / (2.0 * areas + 1.0e-30)
    report.max_aspect_ratio = float(np.max(aspect_ratios))

    # Edge valence via Counter keyed on sorted (i, j) pairs.
    edges: list[tuple[int, int]] = []
    for facet in tile.indices:
        a, b, c = int(facet[0]), int(facet[1]), int(facet[2])
        edges.append((min(a, b), max(a, b)))
        edges.append((min(b, c), max(b, c)))
        edges.append((min(a, c), max(a, c)))

    valence = Counter(edges)
    report.open_boundary_edge_count = sum(1 for v in valence.values() if v == 1)
    report.non_manifold_edge_count = sum(1 for v in valence.values() if v > 2)

    return report


def check(
    tile: TerrainTileData,
    min_angle_deg: float = 10.0,
    max_aspect_ratio: float = 15.0,
) -> None:
    """Run verify() and raise MeshQualityError if any threshold is exceeded.

    Used as a guard in simplify() and after triangulate().

    Raises:
        MeshQualityError: if degenerate facets exist, min angle < threshold, or
                          max aspect ratio > threshold.
    """
    report = verify(tile)

    if report.degenerate_facet_count > 0:
        raise MeshQualityError(
            f"Tile has {report.degenerate_facet_count} degenerate facet(s) (area < 1e-6 m²)"
        )
    if report.min_interior_angle_deg < min_angle_deg:
        raise MeshQualityError(
            f"Minimum interior angle {report.min_interior_angle_deg:.3f}° "
            f"< threshold {min_angle_deg}°"
        )
    if report.max_aspect_ratio > max_aspect_ratio:
        raise MeshQualityError(
            f"Maximum aspect ratio {report.max_aspect_ratio:.2f} "
            f"> threshold {max_aspect_ratio}"
        )
