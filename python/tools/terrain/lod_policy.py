"""lod_policy.py — Screen-space-error rendering-LOD thresholds and per-LOD tile footprints.

Realizes the tile-scale and threshold policy of ``docs/design/terrain_lod_rendering.md``
(OQ-LS-22 → Alternative 3 per-LOD footprints; OQ-LR-2 → Alternative 1 parameters), from the
derivations in ``docs/algorithms/screen_space_lod_selection.md`` (adequacy range) and
``docs/algorithms/lod_culling_geometry.md`` (per-LOD tile-size bound).

The build bakes the per-LOD footprints at the reference resolution ``H_REF_PX`` and records the
policy parameters in the terrain descriptor so the viewer can recompute visibility bands at the
live resolution and FOV (OQ-LR-1 → Alternative 2).
"""

from __future__ import annotations

import math

# --- Adopted policy parameters (terrain_lod_rendering.md; OQ-LR-2 → Alt 1, OQ-LR-1 → Alt 2) ---
TAU_PX: float = 1.0                        # geometric pixel tolerance (max on-screen mesh error)
H_REF_PX: float = 1080.0                   # reference viewport height for the baked footprints
FOV_REF_RAD: float = math.radians(90.0)    # reference vertical field of view
GAMMA: float = 0.25                        # tile-size purity fraction (impure share of a band)
DELTA: float = 0.15                        # hysteresis half-width (fractional range dead-band)

# Per-transition geometric error (m): the max vertical deviation introduced going from LOD ℓ−1 to
# LOD ℓ, from the simplification schedule (terrain.md §Mesh Simplification), for ℓ = 1..6.
LOD_TRANSITION_ERROR_M: list[float] = [3.0, 10.0, 30.0, 100.0, 300.0, 1000.0]

_LOD_COUNT: int = 7


def adequacy_range_m(
    epsilon_m: float,
    h_px: float = H_REF_PX,
    tau_px: float = TAU_PX,
    fov_rad: float = FOV_REF_RAD,
) -> float:
    """Screen-space adequacy range ``R = eps * H / (2 * tau * tan(fov/2))`` (metres).

    The distance at or beyond which a world-space error ``epsilon_m`` projects to at most
    ``tau_px`` pixels on a viewport of height ``h_px`` with vertical field of view ``fov_rad``.
    """
    return epsilon_m * h_px / (2.0 * tau_px * math.tan(fov_rad / 2.0))


def lod_band_edges_m(
    h_px: float = H_REF_PX,
    tau_px: float = TAU_PX,
    fov_rad: float = FOV_REF_RAD,
) -> list[float]:
    """Return the ``_LOD_COUNT + 1`` band edges ``[0, R1, ..., R6, R7]`` (metres).

    LOD ``ℓ`` occupies ``[edges[ℓ], edges[ℓ+1]]``.  ``edges[0] = 0`` (the finest LOD is the floor,
    used from zero range); ``edges[1..6]`` are the transition adequacy ranges; ``edges[7]`` is the
    coarsest LOD's outer edge, extrapolated geometrically (the coarsest is used to infinity, but a
    finite edge is needed to size its footprint).
    """
    edges = [0.0] + [adequacy_range_m(e, h_px, tau_px, fov_rad) for e in LOD_TRANSITION_ERROR_M]
    # Extrapolate the coarsest outer edge by the geometric ratio of the last band step.
    if edges[-2] > 0.0:
        ratio = edges[-1] / edges[-2]
    else:
        ratio = 3.0
    edges.append(edges[-1] + (edges[-1] - edges[-2]) * ratio)
    return edges


def lod_thresholds_m(
    h_px: float = H_REF_PX,
    tau_px: float = TAU_PX,
    fov_rad: float = FOV_REF_RAD,
) -> list[float]:
    """Return the six transition ranges ``[R1, ..., R6]`` (metres) — the LOD switch distances."""
    return lod_band_edges_m(h_px, tau_px, fov_rad)[1:_LOD_COUNT]


def lod_footprints_m(gamma: float = GAMMA) -> list[float]:
    """Return the per-LOD tile footprint ``f_ℓ`` (metres), one per LOD.

    ``f_ℓ ≈ (gamma / sqrt(2)) * (band width of LOD ℓ)`` — the per-LOD tile-size bound from
    lod_culling_geometry.md, so each tile is smaller than its own LOD's band and per-node
    centroid-distance culling stays crisp.
    """
    edges = lod_band_edges_m()
    scale = gamma / math.sqrt(2.0)
    return [scale * (edges[lod + 1] - edges[lod]) for lod in range(_LOD_COUNT)]


def lod_texture_max_px(
    tau_tex: float = TAU_PX,
    h_px: float = H_REF_PX,
    fov_rad: float = FOV_REF_RAD,
    gamma: float = GAMMA,
    near_field_cap: int = 2048,
    min_px: int = 64,
) -> list[int]:
    """Return the per-LOD texture side (pixels) from the screen-space-error texel budget.

    A LOD's texture only needs a texel to project to ``tau_tex`` pixels at the *closest* range
    the LOD renders (its lower band edge ``R_ell``); beyond that the tile is farther and texels
    project sub-pixel.  So ``texel_ground = tau_tex * R_ell * 2*tan(fov/2)/H`` and the texture
    side is ``footprint / texel_ground``.  Because footprint and ``R_ell`` both grow
    geometrically, this is roughly constant (~200 px) across LODs — far smaller than a fixed
    per-LOD cap for the coarse LODs, which is what bounds terrain texture VRAM.

    The finest LOD (L0) renders down to zero range, where the budget diverges, so it uses the
    imagery-native resolution capped at ``near_field_cap`` (``render_mosaic`` clamps to the
    smaller of this and the source's native pixel count).
    """
    edges = lod_band_edges_m(h_px, TAU_PX, fov_rad)  # mesh transition ranges = LOD near-edges
    footprints = lod_footprints_m(gamma)
    k = 2.0 * math.tan(fov_rad / 2.0) / h_px  # texel_ground = tau_tex * R * k
    out: list[int] = []
    for lod in range(_LOD_COUNT):
        near = edges[lod]
        if near <= 0.0:
            out.append(near_field_cap)  # L0 near field → source-native, capped
        else:
            texel_ground = tau_tex * near * k
            side = int(math.ceil(footprints[lod] / texel_ground))
            out.append(max(min_px, min(side, near_field_cap)))
    return out


def lod_policy_dict() -> dict:
    """Return the policy parameters as a JSON-serializable dict for the terrain descriptor.

    The viewer uses this to recompute the visibility bands at the live resolution/FOV
    (terrain_lod_rendering.md OQ-LR-1 Alternative 2).
    """
    return {
        "transition_error_m": list(LOD_TRANSITION_ERROR_M),
        "tau_px": TAU_PX,
        "h_ref_px": H_REF_PX,
        "fov_ref_rad": FOV_REF_RAD,
        "gamma": GAMMA,
        "delta": DELTA,
    }
