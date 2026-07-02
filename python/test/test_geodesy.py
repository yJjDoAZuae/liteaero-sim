"""Tests for geodesy.py — WGS-84 geodetic / ECEF / ENU conversions."""

from __future__ import annotations

import math

import numpy as np

from geodesy import (
    WGS84_A,
    bbox_min_distance_m,
    ecef_from_enu,
    ecef_from_geodetic,
    enu_from_geodetic,
    enu_offset,
    enu_to_ecef_rotation,
    enu_to_lonlat_deg,
    geodetic_distance_m,
)


# ---------------------------------------------------------------------------
# ecef_from_geodetic
# ---------------------------------------------------------------------------

def test_ecef_at_prime_meridian_equator() -> None:
    ecef = ecef_from_geodetic(0.0, 0.0, 0.0)
    assert np.allclose(ecef, [WGS84_A, 0.0, 0.0], atol=1e-6)


def test_ecef_height_adds_radially_at_equator() -> None:
    ecef = ecef_from_geodetic(0.0, 0.0, 100.0)
    assert np.allclose(ecef, [WGS84_A + 100.0, 0.0, 0.0], atol=1e-6)


# ---------------------------------------------------------------------------
# enu_to_ecef_rotation
# ---------------------------------------------------------------------------

def test_enu_rotation_is_orthonormal() -> None:
    r = enu_to_ecef_rotation(math.radians(34.4), math.radians(-119.7))
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-12)


def test_enu_up_column_is_radial_at_equator_prime_meridian() -> None:
    # At (0, 0) the local Up axis (3rd column) points along +ECEF-X.
    r = enu_to_ecef_rotation(0.0, 0.0)
    assert np.allclose(r[:, 2], [1.0, 0.0, 0.0], atol=1e-12)


# ---------------------------------------------------------------------------
# enu_offset  ↔  ecef_from_enu  (round trips)
# ---------------------------------------------------------------------------

def test_enu_offset_zero_at_reference() -> None:
    lat, lon, h = math.radians(34.4), math.radians(-119.7), 12.0
    ref_ecef = ecef_from_geodetic(lat, lon, h)
    assert np.allclose(enu_offset(lat, lon, ref_ecef, ref_ecef), [0.0, 0.0, 0.0], atol=1e-9)


def test_ecef_from_enu_inverts_enu_offset() -> None:
    ref_lat, ref_lon, ref_h = math.radians(34.4), math.radians(-119.7), 12.0
    ref_ecef = ecef_from_geodetic(ref_lat, ref_lon, ref_h)
    pt_ecef = ecef_from_geodetic(math.radians(34.5), math.radians(-119.5), 80.0)

    enu = enu_offset(ref_lat, ref_lon, pt_ecef, ref_ecef)
    back = ecef_from_enu(np.array([enu]), ref_lat, ref_lon, ref_h)[0]
    assert np.allclose(back, pt_ecef, atol=1e-6)


# ---------------------------------------------------------------------------
# enu_from_geodetic  (vectorized)  vs  enu_offset  (scalar)
# ---------------------------------------------------------------------------

def test_enu_from_geodetic_matches_enu_offset() -> None:
    ref_lat, ref_lon, ref_h = math.radians(34.4), math.radians(-119.7), 12.0
    ref_ecef = ecef_from_geodetic(ref_lat, ref_lon, ref_h)

    lats = np.array([math.radians(34.45), math.radians(34.35)])
    lons = np.array([math.radians(-119.65), math.radians(-119.75)])
    hs = np.array([50.0, 20.0])

    enu_vec = enu_from_geodetic(lats, lons, hs, ref_lat, ref_lon, ref_h)
    for i in range(len(lats)):
        pt_ecef = ecef_from_geodetic(float(lats[i]), float(lons[i]), float(hs[i]))
        expected = enu_offset(ref_lat, ref_lon, pt_ecef, ref_ecef)
        assert np.allclose(enu_vec[i], expected, atol=1e-2)  # float32 output


# ---------------------------------------------------------------------------
# enu_to_lonlat_deg
# ---------------------------------------------------------------------------

def test_enu_to_lonlat_recovers_reference_at_zero_offset() -> None:
    ref_lat, ref_lon = math.radians(34.4), math.radians(-119.7)
    lon_deg, lat_deg = enu_to_lonlat_deg(np.array([0.0]), np.array([0.0]), ref_lat, ref_lon)
    assert math.isclose(float(lon_deg[0]), math.degrees(ref_lon), abs_tol=1e-9)
    assert math.isclose(float(lat_deg[0]), math.degrees(ref_lat), abs_tol=1e-9)


# ---------------------------------------------------------------------------
# distances
# ---------------------------------------------------------------------------

def test_geodetic_distance_zero_for_same_point() -> None:
    p = math.radians(34.4), math.radians(-119.7)
    assert math.isclose(geodetic_distance_m(*p, *p), 0.0, abs_tol=1e-9)


def test_bbox_min_distance_zero_inside() -> None:
    d = bbox_min_distance_m(0.0, 0.0, -0.01, 0.01, -0.01, 0.01)
    assert d == 0.0


def test_bbox_min_distance_positive_outside() -> None:
    # Point south-west of the bbox by ~1 km in latitude.
    offset = 1_000.0 / 6_371_000.0
    d = bbox_min_distance_m(-offset, 0.0, 0.0, 0.02, 0.0, 0.02)
    assert math.isclose(d, 1_000.0, rel_tol=1e-3)
