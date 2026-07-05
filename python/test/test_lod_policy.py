"""Tests for lod_policy.py — screen-space-error LOD thresholds and per-LOD footprints.

Verifies the design values from docs/design/terrain_lod_rendering.md (OQ-LR-2 → Alt 1) and the
screen-space-error formula from docs/algorithms/screen_space_lod_selection.md.
"""

from __future__ import annotations

import math


def test_adequacy_range_formula() -> None:
    from lod_policy import adequacy_range_m

    # R = eps * H / (2 tau tan(fov/2)); at H=1080, tau=1, fov=90deg (tan45=1) -> R = eps*540.
    assert math.isclose(adequacy_range_m(1.0, 1080.0, 1.0, math.radians(90.0)), 540.0)
    assert math.isclose(adequacy_range_m(3.0, 1080.0, 1.0, math.radians(90.0)), 1620.0)
    # Sensitivities: R ∝ H, ∝ 1/tau.
    assert math.isclose(adequacy_range_m(3.0, 2160.0, 1.0, math.radians(90.0)), 3240.0)
    assert math.isclose(adequacy_range_m(3.0, 1080.0, 2.0, math.radians(90.0)), 810.0)


def test_transition_thresholds_are_the_reference_ranges() -> None:
    from lod_policy import lod_thresholds_m

    # Transition ranges R1..R6 from the error schedule (3,10,30,100,300,1000) at 540 m/m.
    r = lod_thresholds_m()
    assert len(r) == 6
    expected = [1620.0, 5400.0, 16200.0, 54000.0, 162000.0, 540000.0]
    for got, exp in zip(r, expected):
        assert math.isclose(got, exp, rel_tol=1e-9)


def test_footprints_are_per_lod_and_increasing() -> None:
    from lod_policy import lod_footprints_m

    f = lod_footprints_m()
    assert len(f) == 7
    # Geometric growth (each coarser LOD's footprint larger).
    for a, b in zip(f, f[1:]):
        assert b > a
    # L0 band is [0, R1]=1620 -> f0 = (0.25/sqrt2)*1620 ~= 286.4 m.
    assert math.isclose(f[0], (0.25 / math.sqrt(2.0)) * 1620.0, rel_tol=1e-6)
    # Interior LOD (L1): band = R2-R1 = 3780 -> f1 = 0.1768*3780 ~= 668 m.
    assert math.isclose(f[1], (0.25 / math.sqrt(2.0)) * (5400.0 - 1620.0), rel_tol=1e-6)
    # Fine footprint is sub-km; coarse is tens of km.
    assert f[0] < 500.0
    assert f[4] > 10000.0


def test_texture_resolution_is_sse_bounded_and_small_for_coarse_lods() -> None:
    from lod_policy import lod_texture_max_px

    tex = lod_texture_max_px()
    assert len(tex) == 7
    # L0 renders to zero range → near-field cap (source-native, capped).
    assert tex[0] == 2048
    # Coarse LODs are ~200 px, far below the superseded 4096-8192 px ceilings.
    for lod in range(1, 7):
        assert 64 <= tex[lod] <= 400, f"L{lod} texture {tex[lod]} px out of expected ~200 px range"
    # A texel projects to ~1 px at the LOD's near edge: side ≈ footprint / (R_near/540) at the ref.
    from lod_policy import lod_band_edges_m, lod_footprints_m
    edges = lod_band_edges_m()
    f = lod_footprints_m()
    for lod in range(1, 7):
        expected = f[lod] / (edges[lod] / 540.0)  # tau_tex=1, H=1080, tan45=1 → texel = R/540
        assert abs(tex[lod] - expected) <= 2.0


def test_policy_dict_roundtrips_and_reproduces_thresholds() -> None:
    from lod_policy import adequacy_range_m, lod_policy_dict

    d = lod_policy_dict()
    for key in ("transition_error_m", "tau_px", "h_ref_px", "fov_ref_rad", "gamma", "delta"):
        assert key in d
    # A consumer (the viewer) can reproduce R1 from the recorded parameters.
    r1 = adequacy_range_m(d["transition_error_m"][0], d["h_ref_px"], d["tau_px"], d["fov_ref_rad"])
    assert math.isclose(r1, 1620.0, rel_tol=1e-9)
