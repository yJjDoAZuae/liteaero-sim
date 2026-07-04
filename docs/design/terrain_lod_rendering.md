# Terrain Rendering LOD — Tile Scale and Threshold Policy

**Status:** design authority for how the live viewer's terrain **rendering** LOD thresholds and
per-LOD tile footprints are determined. It replaces the earlier convention-based band table
(the "≈30 vertices per line of sight, $r_0=300$ m" rule in
[terrain.md §Slant-Range Selection](terrain.md), which is not tied to rendered resolution) with
a screen-space-error policy.

**Scope.** This document governs LOD selection for **on-screen rendering** only — deciding which
terrain tile mesh and texture are drawn at a given observer distance so that the on-screen error
stays within a pixel budget. It does **not** govern the simulator's internal terrain LOD for
collision and sensor queries ([terrain.md §LOD levels](terrain.md), `LodSelector`), which is
selected against a *physical* error tolerance (metres of surface deviation acceptable to a
sensor or the contact model), not a screen-space one. That path needs an analogous but separate
treatment and is out of scope here.

**Mathematical authority.** The policy is an application of two algorithm documents and
reproduces their results without re-deriving them:

- [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md) — the pixel
  projection of a world-space error and the adequacy range $R_\ell$.
- [lod_culling_geometry.md](../algorithms/lod_culling_geometry.md) — the tile-size, transition
  alignment, and tile-count consequences of applying those thresholds to finite tiles.

---

## Use Cases

| ID | Use case | Mechanism |
| --- | --- | --- |
| UC-LR-1 | Choose the LOD drawn at every observer distance | Screen-space adequacy range $R_\ell$ sets each LOD's visibility band |
| UC-LR-2 | Choose each LOD's tile footprint | Purity bound $h_\ell \le \tfrac12\gamma(R_{\ell+1}-R_\ell)$ |
| UC-LR-3 | Blend adjacent LODs without popping | Hysteresis dead-band $\delta$ → crossfade width $B_\ell = 2\delta R_\ell$ |
| UC-LR-4 | Keep behavior correct as the region grows | Per-LOD footprints bound the tile count (scales to large operating areas) |

Class hierarchy, interface, and serialization sections of the canonical design format are **not
applicable**: this is a parameter-and-policy document realized in existing components (the Python
terrain build and the Godot loader), not a new class cluster. The realizing components are named
in the Integration Contract.

---

## Design Parameters

The policy has four free parameters; every threshold and footprint below is a function of them
and of the per-LOD geometric error schedule. Recommended defaults are given but are **pending
confirmation and in-engine validation** (see Open Questions); they are the design's starting
values, not measured optima.

| Symbol | Meaning | Recommended default | Source / notes |
| --- | --- | --- | --- |
| $\tau$ | Geometric pixel tolerance (max on-screen mesh error) | $1$ px | Quality target; $\ge 1$ px loosens, coarsens sooner |
| $\tau_\text{tex}$ | Texture pixel tolerance (max on-screen texel size) | $1$ px | Governs texture-LOD; usually met by GPU mipmapping |
| $H_\text{ref}$ | Reference viewport height (pixels) used to precompute thresholds | $1080$ | The actual runtime height may differ — see OQ-LR-1 |
| $\phi$ | Vertical field of view | $90^\circ$ (from the viewer camera) | $\tan(\phi/2)=1$; if the camera FOV changes, thresholds move |
| $\gamma$ | Tile-size purity fraction (impure share of a band) | $0.25$ | Fraction of each LOD band allowed to be single-LOD-impure |
| $\delta$ | Hysteresis half-width (fractional range dead-band) | $0.15$ | Crossfade width $B_\ell = 2\delta R_\ell$; $\approx 2\tau\delta$ px dead-band |

The **per-LOD geometric error** $\varepsilon_\ell$ is the maximum vertical deviation of LOD
$\ell$'s mesh from the true surface, taken from the simplification error schedule
([terrain.md §Mesh Simplification](terrain.md)): approximately
$\varepsilon = (\,\varepsilon_0,\,3,\,10,\,30,\,100,\,300,\,1000\,)\ \text{m}$ for
$\ell = 0\dots6$, where $\varepsilon_0$ is the finest level's residual (bounded by the DEM
sampling, taken as $\approx 1$ m).

---

## Model

### §1 Rendering LOD thresholds (UC-LR-1)

Each LOD's visibility band lower bound is its adequacy range from
[screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md):

$$
R_\ell = \frac{\varepsilon_\ell\,H_\text{ref}}{2\,\tau\,\tan(\phi/2)} .
$$

LOD $\ell$ is drawn over the observer-distance band $[\,R_\ell,\ R_{\ell+1}\,]$ (the coarsest
level runs to infinity). With the recommended defaults and $\tan(45^\circ)=1$, the scale factor
is $H_\text{ref}/(2\tau) = 540$ m per metre of error, giving the **illustrative** thresholds:

| LOD $\ell$ | $\varepsilon_\ell$ (m) | $R_\ell = 540\,\varepsilon_\ell$ (m) | Band $[R_\ell, R_{\ell+1}]$ (m) |
| --- | --- | --- | --- |
| 0 | $\approx 1$ | 540 | 540 – 1 620 |
| 1 | 3 | 1 620 | 1 620 – 5 400 |
| 2 | 10 | 5 400 | 5 400 – 16 200 |
| 3 | 30 | 16 200 | 16 200 – 54 000 |
| 4 | 100 | 54 000 | 54 000 – 162 000 |
| 5 | 300 | 162 000 | 162 000 – 540 000 |
| 6 | 1 000 | 540 000 | 540 000 – ∞ |

These bands are $\approx 5\times$ larger than the superseded convention's (300, 900, 2 700 … m),
because the convention encoded no resolution, FOV, or tolerance. The hysteresis dead-band adds
switch-to-coarser at $R_\ell(1+\delta)$ and switch-to-finer at $R_\ell(1-\delta)$; the overlap
$[\,R_\ell(1-\delta),\,R_\ell(1+\delta)\,]$ of width $B_\ell = 2\delta R_\ell$ is the crossfade
interval.

### §2 Per-LOD tile footprint (UC-LR-2)

From the single-LOD purity bound of
[lod_culling_geometry.md](../algorithms/lod_culling_geometry.md), a tile's bounding radius must
satisfy $h_\ell \le \tfrac12\gamma\,(R_{\ell+1}-R_\ell)$, so the footprint (square side
$f_\ell \approx \sqrt2\,h_\ell$) is

$$
f_\ell \;\approx\; \frac{\gamma}{\sqrt2}\,\bigl(R_{\ell+1}-R_\ell\bigr).
$$

With the illustrative thresholds and $\gamma = 0.25$:

| LOD $\ell$ | Band width $R_{\ell+1}-R_\ell$ (m) | Footprint $f_\ell \approx 0.177\,(R_{\ell+1}-R_\ell)$ (m) |
| --- | --- | --- |
| 0 | 1 080 | ≈ 190 |
| 1 | 3 780 | ≈ 670 |
| 2 | 10 800 | ≈ 1 900 |
| 3 | 37 800 | ≈ 6 700 |
| 4 | 108 000 | ≈ 19 000 |
| 5 | 378 000 | ≈ 67 000 |

The footprint grows geometrically (≈ $\times 3.5$ per level), so coarse LODs are few large tiles
and fine LODs are many small tiles. This is the intended structure: tile size is matched to each
LOD's band, not to the finest band.

### §3 Uniform vs. per-LOD footprint (UC-LR-4)

The purity bound is per-LOD, so a **single uniform footprint** must be chosen for the *finest*
band ($f \le f_0 \approx 190$ m) and then applied to every LOD, where it is far smaller than
each coarse band requires. By the tile-count scaling of
[lod_culling_geometry.md](../algorithms/lod_culling_geometry.md), uniform tiling then costs
$L\,A/f_0^2$ tiles versus $\approx 1.1\,A/f_0^2$ for per-LOD ($\approx 6\times$ more at $L=7$),
and the coarse LODs collapse to geometrically identical meshes (redundant relief). Uniform tiling
also does not bound the count as the region grows — the several-hundred-kilometre operating areas
required by the streaming format ([live_sim_view.md OQ-LS-20](live_sim_view.md)) are infeasible
at a sub-kilometre uniform footprint.

**The design therefore adopts per-LOD footprints (§2).** Its one cost, established in
[lod_culling_geometry.md §Transition Alignment](../algorithms/lod_culling_geometry.md), is that
adjacent-LOD tiles covering the same point have offset centroids (offset $\le h_\ell+h_{\ell+1}$),
so their crossfades must overlap: the condition is $h_{\ell+1}\lesssim B_{\ell+1}$, i.e.

$$
\tfrac{\gamma}{2}\,(R_{\ell+2}-R_{\ell+1}) \;\lesssim\; 2\,\delta\,R_{\ell+1},
$$

a ratio that is constant across LODs and, with the recommended $\gamma,\delta$, is $O(1)$ — it
must be verified, not assumed (Open Questions, Test Strategy). The uniform (shared-grid) scheme
has zero alignment offset and remains the fallback if the per-LOD crossfade cannot be made clean.

---

## Integration Contract

- **Terrain build** (`build_terrain` / `terrain_chunks`, Python). Computes $R_\ell$ (§1) and
  $f_\ell$ (§2) from the design parameters and $\varepsilon_\ell$ schedule; tiles each LOD at its
  own footprint $f_\ell$; groups tiles into chunks and writes the descriptor. The chunk grid
  becomes **per-LOD** (chunk size derived from $f_\ell$), which the coordinate-addressable
  descriptor already supports per `(lod, cx, cy)`. The current build uses a single uniform
  footprint and the convention thresholds — updating it to this policy is a work item.
- **Viewer** (`TerrainLoader.gd`, Godot). Sets each LOD node's visibility band to
  $[\,R_\ell(1-\delta)-h_\ell,\ R_{\ell+1}(1+\delta)+h_\ell\,]$ (bands padded by the tile radius
  per [lod_culling_geometry.md](../algorithms/lod_culling_geometry.md)) with a crossfade over
  $B_\ell$. The streaming manager
  ([live_sim_view.md OQ-LS-21](live_sim_view.md)) pages the fine LODs whose footprints and counts
  §2 quantifies. The viewer currently sets bands from the convention table — updating it is a work
  item.
- **Parameter provenance.** $\varepsilon_\ell$ is owned by the simplification schedule
  ([terrain.md](terrain.md)); $\phi$ by the viewer camera; $H_\text{ref}$, $\tau$, $\tau_\text{tex}$,
  $\gamma$, $\delta$ by this document.

---

## Open Questions

| ID | Question | Blocking |
| --- | --- | --- |
| OQ-LR-1 | Build-time vs. runtime thresholds | Not blocking (has a safe default) |
| OQ-LR-2 | Confirmation of the parameter defaults ($\tau,\gamma,\delta,H_\text{ref}$) | Blocking the re-tile |

### OQ-LR-1 — Build-time versus runtime threshold evaluation

**Problem.** $R_\ell \propto H_\text{ref}/\tan(\phi/2)$, so the thresholds depend on the actual
output resolution and camera FOV, which can change at runtime (window resize, FOV/zoom change).
Precomputing tile footprints at build time is unavoidable — geometry cannot be re-tiled per
frame — but the *visibility bands* could be recomputed at runtime from the live viewport height
and FOV. **Alternatives.** (1) Fix both at $H_\text{ref}, \phi_\text{ref}$ at build time — simple,
but a $2\times$ resolution or FOV change makes the thresholds off by $2\times$. (2) Precompute
footprints at build time, recompute visibility bands each frame from the live $H,\phi$ — keeps
thresholds correct as the window/FOV change, with the footprint fixed to the build-time reference
(a mismatch bounded by the resolution ratio, absorbed by $\gamma$'s margin). **Recommendation.**
Alternative 2 for the bands; footprints from $H_\text{ref}$. Non-blocking because the fixed
default is usable for validation.

### OQ-LR-2 — Confirmation of the parameter defaults

**Problem.** The recommended $\tau=1$ px, $\gamma=0.25$, $\delta=0.15$, $H_\text{ref}=1080$ are
engineering starting points, not measured. They set the absolute tile count and the alignment
ratio of §3, so they gate the (expensive) re-tile. **Recommendation.** Confirm $\tau$ and
$H_\text{ref}$ against the intended display, then validate $\gamma,\delta$ against the Test
Strategy metric before committing to a full re-tile; treat the first per-LOD build as a tuning
iteration. Blocking the re-tile run.

---

## Test Strategy

- **Analytical.** With the chosen parameters, evaluate the transition-alignment condition
  $h_{\ell+1}\le B_{\ell+1}$ at every LOD (§3) and the single-LOD purity at every band (§2);
  reject parameter sets that violate them.
- **Culling metric.** Reuse the opaque-LOD-per-location predictor: sweep observer positions and
  confirm exactly one opaque LOD covers each ground point except within the intended crossfade
  interval $B_\ell$ (no gaps, no stacking).
- **In-engine.** In `run_sim.sh`, confirm a single terrain surface across LOD transitions from
  both low grazing and high nadir viewpoints, and no visible pop or seam at the per-LOD footprint
  boundaries — the case the geometry flags as the residual risk of per-LOD tiling.

---

## References

| Reference | Relevance |
| --- | --- |
| [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md) | Adequacy range $R_\ell$, hysteresis dead-band, worked thresholds |
| [lod_culling_geometry.md](../algorithms/lod_culling_geometry.md) | Tile-size bound, transition alignment, tile-count scaling |
| [terrain.md](terrain.md) | Simplification error schedule $\varepsilon_\ell$; sim-side (physical) LOD model |
| [live_sim_view.md](live_sim_view.md) | Streaming format/manager (OQ-LS-20/21) and the footprint question (OQ-LS-22) this policy informs |
