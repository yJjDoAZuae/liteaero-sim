"""Tests for geometry.py — frame-agnostic rotation utilities."""

from __future__ import annotations

import math

import numpy as np

from geometry import euler_zyx_to_rotation_matrix, rotation_matrix_to_quaternion


def _quaternion_to_matrix(quat: list[float]) -> np.ndarray:
    """glTF [x, y, z, w] quaternion → 3×3 rotation matrix (independent reference impl)."""
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# euler_zyx_to_rotation_matrix
# ---------------------------------------------------------------------------

def test_euler_identity() -> None:
    assert np.allclose(euler_zyx_to_rotation_matrix(0.0, 0.0, 0.0), np.eye(3))


def test_euler_is_orthonormal() -> None:
    r = euler_zyx_to_rotation_matrix(0.7, -0.3, 1.1)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-12)


def test_euler_yaw_90_maps_x_to_y() -> None:
    r = euler_zyx_to_rotation_matrix(math.pi / 2, 0.0, 0.0)
    assert np.allclose(r @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


# ---------------------------------------------------------------------------
# rotation_matrix_to_quaternion
# ---------------------------------------------------------------------------

def test_quaternion_identity() -> None:
    assert np.allclose(rotation_matrix_to_quaternion(np.eye(3)), [0.0, 0.0, 0.0, 1.0])


def test_quaternion_is_unit() -> None:
    r = euler_zyx_to_rotation_matrix(0.4, 0.9, -0.6)
    q = rotation_matrix_to_quaternion(r)
    assert math.isclose(math.sqrt(sum(c * c for c in q)), 1.0, abs_tol=1e-12)


def test_quaternion_roundtrips_all_branches() -> None:
    # A generic rotation plus 180° rotations about each axis exercise all four
    # branches of the matrix→quaternion conversion.
    rotations = [
        euler_zyx_to_rotation_matrix(0.3, -0.7, 1.2),
        euler_zyx_to_rotation_matrix(0.0, 0.0, math.pi),   # 180° roll  (x-branch)
        euler_zyx_to_rotation_matrix(0.0, math.pi, 0.0),   # 180° pitch (y-branch)
        euler_zyx_to_rotation_matrix(math.pi, 0.0, 0.0),   # 180° yaw   (z-branch)
    ]
    for r in rotations:
        q = rotation_matrix_to_quaternion(r)
        assert np.allclose(_quaternion_to_matrix(q), r, atol=1e-9)
