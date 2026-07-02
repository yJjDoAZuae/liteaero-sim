"""geodesy.py — WGS-84 geodetic / ECEF / ENU conversions shared across the tooling.

Single home for the geographic-frame math that the terrain ingestion pipeline and
exporters previously duplicated in module-local copies (``export_gltf``, ``colorize``,
``triangulate``, ``verify``, ``build_terrain``).  Frame conventions:

* **ECEF** — Earth-centered, Earth-fixed, WGS-84 ellipsoid, metres.
* **ENU** — local East / North / Up tangent frame at a reference geodetic point.

Rotation-matrix / quaternion helpers that do not depend on a geographic frame live in
``geometry.py``.
"""

from __future__ import annotations

import math

import numpy as np

# ---------------------------------------------------------------------------
# WGS-84 ellipsoid constants
# ---------------------------------------------------------------------------

WGS84_A: float = 6_378_137.0                       # semi-major axis [m]
WGS84_F: float = 1.0 / 298.257223563               # flattening
WGS84_E2: float = 2.0 * WGS84_F - WGS84_F**2       # first eccentricity squared

# Mean Earth radius used for the equirectangular distance approximation.
_MEAN_EARTH_RADIUS_M: float = 6_371_000.0


# ---------------------------------------------------------------------------
# Geodetic → ECEF
# ---------------------------------------------------------------------------

def ecef_from_geodetic(lat_rad: float, lon_rad: float, h_m: float) -> np.ndarray:
    """Geodetic (lat, lon, ellipsoidal height) → ECEF position as a 3-vector [m]."""
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat**2)
    return np.array([
        (N + h_m) * cos_lat * math.cos(lon_rad),
        (N + h_m) * cos_lat * math.sin(lon_rad),
        (N * (1.0 - WGS84_E2) + h_m) * sin_lat,
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# ENU basis
# ---------------------------------------------------------------------------

def enu_to_ecef_rotation(lat_rad: float, lon_rad: float) -> np.ndarray:
    """ENU→ECEF rotation whose columns are the ECEF east, north, up basis vectors.

    ``ecef_vector = R @ enu_vector``; its transpose is the ECEF→ENU rotation applied by
    :func:`enu_offset`.
    """
    sl, cl = math.sin(lat_rad), math.cos(lat_rad)
    sn, cn = math.sin(lon_rad), math.cos(lon_rad)
    return np.array([
        [-sn, -sl * cn, cl * cn],   # ECEF X components of (east, north, up)
        [ cn, -sl * sn, cl * sn],   # ECEF Y components
        [0.0,       cl,      sl],   # ECEF Z components
    ], dtype=np.float64)


def enu_offset(
    ref_lat_rad: float, ref_lon_rad: float,
    point_ecef: np.ndarray, ref_ecef: np.ndarray,
) -> np.ndarray:
    """ENU vector from ``ref`` to ``point``, expressed in the ENU frame at ``ref``."""
    dp = point_ecef - ref_ecef
    sl, cl = math.sin(ref_lat_rad), math.cos(ref_lat_rad)
    sn, cn = math.sin(ref_lon_rad), math.cos(ref_lon_rad)
    east  = -sn * dp[0] + cn * dp[1]
    north = -sl * cn * dp[0] - sl * sn * dp[1] + cl * dp[2]
    up    =  cl * cn * dp[0] + cl * sn * dp[1] + sl * dp[2]
    return np.array([east, north, up], dtype=np.float64)


def ecef_from_enu(
    enu: np.ndarray, ref_lat_rad: float, ref_lon_rad: float, ref_h_m: float,
) -> np.ndarray:
    """Convert ENU offsets ``(N, 3)`` about a reference to ECEF positions ``(N, 3)``, float64."""
    ref_ecef = ecef_from_geodetic(ref_lat_rad, ref_lon_rad, ref_h_m)
    rotation = enu_to_ecef_rotation(ref_lat_rad, ref_lon_rad)
    return enu.astype(np.float64) @ rotation.T + ref_ecef


def enu_from_geodetic(
    lat_rad: np.ndarray,
    lon_rad: np.ndarray,
    h_m: np.ndarray,
    ref_lat_rad: float,
    ref_lon_rad: float,
    ref_h_m: float,
) -> np.ndarray:
    """Convert geodetic arrays to ENU float32 offsets from a reference point.

    Returns an ``(N, 3)`` float32 array with columns ``[east_m, north_m, up_m]``.
    """
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    N_arr = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat**2)
    px = (N_arr + h_m) * cos_lat * cos_lon
    py = (N_arr + h_m) * cos_lat * sin_lon
    pz = (N_arr * (1.0 - WGS84_E2) + h_m) * sin_lat

    ref_ecef = ecef_from_geodetic(ref_lat_rad, ref_lon_rad, ref_h_m)
    dx = px - ref_ecef[0]
    dy = py - ref_ecef[1]
    dz = pz - ref_ecef[2]

    sin_rlat = math.sin(ref_lat_rad)
    cos_rlat = math.cos(ref_lat_rad)
    sin_rlon = math.sin(ref_lon_rad)
    cos_rlon = math.cos(ref_lon_rad)

    east = -sin_rlon * dx + cos_rlon * dy
    north = -sin_rlat * cos_rlon * dx - sin_rlat * sin_rlon * dy + cos_rlat * dz
    up = cos_rlat * cos_rlon * dx + cos_rlat * sin_rlon * dy + sin_rlat * dz

    return np.column_stack([east, north, up]).astype(np.float32)


def enu_to_lonlat_deg(
    east_m: np.ndarray,
    north_m: np.ndarray,
    ref_lat_rad: float,
    ref_lon_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """First-order ENU → geodetic approximation for small offsets.

    Accurate to < 1 m for offsets up to 50 km from the reference.  Returns
    ``(lon_deg, lat_deg)`` arrays.
    """
    sin_lat = math.sin(ref_lat_rad)
    cos_lat = math.cos(ref_lat_rad)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat**2)
    M = WGS84_A * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sin_lat**2) ** 1.5

    lon_deg = math.degrees(ref_lon_rad) + np.degrees(east_m / (N * cos_lat + 1e-30))
    lat_deg = math.degrees(ref_lat_rad) + np.degrees(north_m / M)
    return lon_deg, lat_deg


# ---------------------------------------------------------------------------
# Distances
# ---------------------------------------------------------------------------

def geodetic_distance_m(
    lat1_rad: float, lon1_rad: float,
    lat2_rad: float, lon2_rad: float,
) -> float:
    """Equirectangular great-circle distance in metres (sufficient for LOD partitioning)."""
    dlat = lat2_rad - lat1_rad
    dlon = (lon2_rad - lon1_rad) * math.cos((lat1_rad + lat2_rad) * 0.5)
    return math.sqrt(dlat**2 + dlon**2) * _MEAN_EARTH_RADIUS_M


def bbox_min_distance_m(
    point_lat_rad: float,
    point_lon_rad: float,
    lat_min_rad: float,
    lat_max_rad: float,
    lon_min_rad: float,
    lon_max_rad: float,
) -> float:
    """Minimum great-circle distance from a point to a geodetic bbox (metres).

    Returns 0.0 if the point is inside the bbox.
    """
    clamped_lat = max(lat_min_rad, min(lat_max_rad, point_lat_rad))
    clamped_lon = max(lon_min_rad, min(lon_max_rad, point_lon_rad))
    return geodetic_distance_m(point_lat_rad, point_lon_rad, clamped_lat, clamped_lon)
