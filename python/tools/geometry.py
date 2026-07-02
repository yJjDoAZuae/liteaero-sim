"""geometry.py — coordinate-frame-agnostic rotation utilities.

Pure geometry helpers shared across the tooling (terrain export, trajectory
visualization, ...).  Nothing here depends on the WGS-84 ellipsoid or any
geographic frame; for geodetic / ECEF / ENU conversions see ``geodesy.py``.
"""

from __future__ import annotations

import math

import numpy as np


def euler_zyx_to_rotation_matrix(
    heading_rad: float, pitch_rad: float, roll_rad: float
) -> np.ndarray:
    """ZYX intrinsic Euler rotation ``R_z(ψ) @ R_y(θ) @ R_x(φ)``.  Returns a (3, 3) array."""
    cpsi, spsi = math.cos(heading_rad), math.sin(heading_rad)
    cth, sth = math.cos(pitch_rad), math.sin(pitch_rad)
    cphi, sphi = math.cos(roll_rad), math.sin(roll_rad)

    rz = np.array([[cpsi, -spsi, 0.0], [spsi, cpsi, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cth, 0.0, sth], [0.0, 1.0, 0.0], [-sth, 0.0, cth]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cphi, -sphi], [0.0, sphi, cphi]])
    return rz @ ry @ rx


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> list[float]:
    """Convert a proper rotation matrix to a glTF ``[x, y, z, w]`` quaternion (normalized)."""
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / norm, y / norm, z / norm, w / norm]
