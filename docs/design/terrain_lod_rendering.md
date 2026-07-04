# Terrain Rendering LOD — Tile Scale and Threshold Policy

**Status:** design authority for how the live viewer's terrain **rendering** LOD thresholds and
per-LOD tile footprints are determined. It replaces the earlier convention-based band table
(the "≈30 vertices per line of sight, $r_0=300$ m" rule in
[terrain.md §Slant-Range Selection](terrain.md), which is not tied to rendered resolution) with
a screen-space-error policy. Per-LOD footprints are the resolution of
[live_sim_view.md OQ-LS-22](live_sim_view.md) (Alternative 3); the numeric parameters are resolved
(OQ-LR-2 → $\tau=1$ px, $\gamma=0.25$; OQ-LR-1 → runtime bands with footprints baked at
$H_\text{ref}=1080$, $\delta=0.15$), so the policy is fully specified and ready to implement.

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
and of the per-LOD geometric error schedule. The values below are **adopted** — the quality/cost
operating point $(\tau,\gamma)$ is [OQ-LR-2](#oq-lr-2--rendering-lod-qualitycost-operating-point-tau-gamma)
resolved to Alternative 1, $H_\text{ref}$ is fixed by [OQ-LR-1](#oq-lr-1--build-time-versus-runtime-evaluation-of-the-lod-thresholds)
(footprints baked at the reference, visibility bands recomputed at runtime), and $\delta$ inherits
the established anti-flicker value. They remain subject to confirmation-by-measurement at IP-LV-6
(the re-tile is treated as a tuning iteration), but they are the design's committed starting point,
not placeholders.

| Symbol | Meaning | Adopted value | Source / notes |
| --- | --- | --- | --- |
| $\tau$ | Geometric pixel tolerance (max on-screen mesh error) | $1$ px | OQ-LR-2 Alt 1; $\ge 1$ px loosens, coarsens sooner |
| $\tau_\text{tex}$ | Texture pixel tolerance (max on-screen texel size) | $1$ px | Governs texture-LOD; usually met by GPU mipmapping |
| $H_\text{ref}$ | Reference viewport height (pixels) for the **baked footprints** | $1080$ | OQ-LR-1 Alt 2: footprints from $H_\text{ref}$; runtime bands from live $H$ |
| $\phi$ | Vertical field of view | $90^\circ$ (from the viewer camera) | $\tan(\phi/2)=1$; runtime bands track the live FOV (OQ-LR-1 Alt 2) |
| $\gamma$ | Tile-size purity fraction (impure share of a band) | $0.25$ | OQ-LR-2 Alt 1; fraction of each LOD band allowed single-LOD-impure |
| $\delta$ | Hysteresis half-width (fractional range dead-band) | $0.15$ | Inherited anti-flicker value; crossfade width $B_\ell = 2\delta R_\ell$ |

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
level runs to infinity). With the adopted parameters and $\tan(45^\circ)=1$, the scale factor is
$H_\text{ref}/(2\tau) = 540$ m per metre of error, giving the reference thresholds (the runtime
viewer rescales the *bands* by the live $H/H_\text{ref}$ and $\cot(\phi/2)$ per OQ-LR-1 Alt 2; the
baked *footprints* of §2 use these reference values):

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

With the adopted reference thresholds and $\gamma = 0.25$ (footprints are baked at $H_\text{ref}$,
so these are the values the build uses):

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

**The design adopts per-LOD footprints (§2)** — the resolution of
[live_sim_view.md OQ-LS-22](live_sim_view.md) (Alternative 3). Its one cost, established in
[lod_culling_geometry.md §Transition Alignment](../algorithms/lod_culling_geometry.md), is that
adjacent-LOD tiles covering the same point have offset centroids (offset $\le h_\ell+h_{\ell+1}$),
so their crossfades must overlap: the condition is $h_{\ell+1}\lesssim B_{\ell+1}$, i.e.

$$
\tfrac{\gamma}{2}\,(R_{\ell+2}-R_{\ell+1}) \;\lesssim\; 2\,\delta\,R_{\ell+1},
$$

a ratio that is constant across LODs and, with the recommended $\gamma,\delta$, is $O(1)$. This is
a **validation gate, not an assumption**: it must be confirmed per the Test Strategy before the
per-LOD tiling is committed. Should it prove impossible to make the per-LOD crossfade seamless for
achievable $\gamma,\delta$, that is a design escalation back to OQ-LS-22 (the uniform shared-grid
scheme has zero alignment offset and would be reconsidered there) — not a silent fallback.

---

## Integration Contract

- **Terrain build** (`build_terrain` / `terrain_chunks`, Python). Computes the reference $R_\ell$
  (§1, at $H_\text{ref}$) and the footprints $f_\ell$ (§2) from the design parameters and
  $\varepsilon_\ell$ schedule; tiles each LOD at its own footprint $f_\ell$; groups tiles into
  chunks and writes the descriptor. The chunk grid becomes **per-LOD** (chunk size derived from
  $f_\ell$), which the coordinate-addressable descriptor already supports per `(lod, cx, cy)`. The
  descriptor records the parameters ($\varepsilon_\ell$, $H_\text{ref}$, $\tau$, $\phi_\text{ref}$,
  $\gamma$, $\delta$) so the viewer can recompute bands. The current build uses a single uniform
  footprint and the convention thresholds — updating it is IP-LV-5/9.
- **Viewer** (`TerrainLoader.gd`, Godot). Per OQ-LR-1 Alternative 2, **recomputes** each LOD's
  visibility band each frame (or on viewport/FOV change) from the *live* viewport height $H$ and
  FOV $\phi$: $R_\ell = \varepsilon_\ell H/(2\tau\tan(\phi/2))$, then sets the node band to
  $[\,R_\ell(1-\delta)-h_\ell,\ R_{\ell+1}(1+\delta)+h_\ell\,]$ (padded by the tile radius per
  [lod_culling_geometry.md](../algorithms/lod_culling_geometry.md)) with a crossfade over
  $B_\ell = 2\delta R_\ell$; the streaming radii `R_fetch`/`R_unload`
  ([live_sim_view.md OQ-LS-21](live_sim_view.md)) derive from the same live $R_\ell$. The baked
  footprints stay fixed at $H_\text{ref}$. The current viewer sets frozen bands from the
  convention table — updating it is IP-LV-10.
- **Parameter provenance.** $\varepsilon_\ell$ is owned by the simplification schedule
  ([terrain.md](terrain.md)); $\phi$ by the viewer camera; $H_\text{ref}$, $\tau$, $\tau_\text{tex}$,
  $\gamma$, $\delta$ by this document.

---

## Open Questions

| ID | Summary | Blocking |
| --- | --- | --- |
| OQ-LR-1 | Build-time vs. runtime evaluation of the LOD thresholds and crossfades | Resolved — Alternative 2 (runtime bands from live $H,\phi$; footprints baked at $H_\text{ref}$) |
| OQ-LR-2 | Rendering-LOD quality/cost operating point ($\tau$, $\gamma$) | Resolved — Alternative 1 ($\tau=1$ px, $\gamma=0.25$) |

### OQ-LR-1 — Build-time versus runtime evaluation of the LOD thresholds

**Resolved — Alternative 2 (runtime bands, build-time footprints).** The tile footprints $f_\ell$
are baked at the reference $(H_\text{ref}=1080,\ \phi_\text{ref}=90^\circ)$; the viewer recomputes
the visibility bands $[R_\ell,R_{\ell+1}]$ and crossfade widths $B_\ell$ each frame (or on
viewport/FOV change) from the *live* $H$ and $\phi$, and derives the streaming radii from the same
live $R_\ell$. This keeps the pixel-accurate thresholds correct as the window resizes or the FOV
changes, for a few divisions per frame; the only residual — a footprint/band mismatch away from
$H_\text{ref}$ — is bounded by the resolution ratio and absorbed by the $\gamma$ purity margin. The
descriptor carries the parameters so the viewer can evaluate $R_\ell$. Realized in IP-LV-10.
**Background and alternatives retained below.**

**Problem.** A LOD's adequacy range is $R_\ell = \varepsilon_\ell\,H/(2\tau\tan(\phi/2))$
(§1): it is proportional to the viewport height $H$ (pixels) and to $\cot(\phi/2)$, where $\phi$
is the vertical field of view. The tile footprints $f_\ell$ (§2) are baked geometry and must be
fixed at build time from a reference $(H_\text{ref}, \phi_\text{ref})$ — geometry cannot be
re-tiled per frame. The *visibility bands* $[R_\ell, R_{\ell+1}]$ and crossfade widths
$B_\ell = 2\delta R_\ell$ applied by the viewer, however, may either be frozen at the same
reference values or recomputed at runtime from the live viewport height and camera FOV, which
change on window resize and on any FOV/zoom change. If the bands are frozen and the runtime
viewport differs from $H_\text{ref}$ by a factor $k$, every threshold is wrong by $k$: on a
$2160$-pixel display against an $H_\text{ref}=1080$ reference ($k=2$), each LOD switches at half
its correct distance — visible over-coarsening at mid range. Affected use cases: UC-LR-1
(thresholds), UC-LR-3 (crossfade). Not affected: UC-LR-2 (footprints), which are unavoidably
build-time.

**Alternatives.**

1. **Fixed reference for everything.** Freeze $R_\ell$ and $B_\ell$ at the build-time
   $(H_\text{ref}, \phi_\text{ref})$; the viewer uses the baked band table verbatim. *Benefits:*
   simplest; the bands are compile-time constants; no per-frame arithmetic; fully deterministic
   across runs. *Drawbacks:* thresholds are off by the resolution/FOV ratio whenever the runtime
   display differs from the reference — a factor of $2$ at $2160$p vs $1080$p, with correspondingly
   early coarsening. *Prerequisite:* a chosen $(H_\text{ref}, \phi_\text{ref})$ (OQ-LR-2 and the
   viewer camera).
2. **Runtime bands, build-time footprints.** Bake $f_\ell$ from $H_\text{ref}$; recompute $R_\ell$
   and $B_\ell$ each frame (or on resize / FOV change) from the live $H$ and $\phi$. *Benefits:*
   thresholds stay pixel-correct as the window resizes or the FOV changes, at the cost of a few
   divisions per frame; the footprint is fixed to $H_\text{ref}$, and any footprint-versus-band
   mismatch away from the reference is bounded by the resolution ratio and absorbed by the $\gamma$
   purity margin (§2). *Drawbacks:* the footprint-to-band match is exact only at $H_\text{ref}$; a
   small runtime coupling of the loader to the live camera parameters. *Prerequisite:* the viewer
   reads the live viewport height and FOV each frame.
3. **Runtime footprints too.** Re-tile the geometry to the live resolution. *Benefits:* exact
   footprint-to-band match at any resolution. *Drawbacks:* infeasible — tiling is an offline,
   multi-minute build; it cannot run per frame or per resize. *Prerequisite:* none that is
   achievable at runtime.

**Recommendation.** Alternative 2: recompute the visibility bands and crossfades at runtime from
the live $H, \phi$, with footprints baked from $H_\text{ref}$. It keeps the pixel-accurate
thresholds correct across displays for a handful of per-frame divisions, and the only residual —
the footprint/band mismatch away from the reference — is bounded by $\gamma$. Not blocking: the
fixed-reference default (Alternative 1) is adequate for the first in-engine validation, so the
runtime path can follow.

### OQ-LR-2 — Rendering-LOD quality/cost operating point ($\tau$, $\gamma$)

**Resolved — Alternative 1 ($\tau = 1$ px, $\gamma = 0.25$).** The operating point is pixel-accurate
(sub-pixel LOD error at the reference resolution) with the safest crossfade-alignment margin
(smallest $h/B$), giving the most forgiving per-LOD transition. Its tile-count cost — the highest of
the alternatives, finest footprint $\approx 190$ m at $1080$p — is exactly what the residency
streaming manager (IP-LV-7) is there to bound. The first per-LOD build is a tuning iteration
(IP-LV-6); $\gamma$ is raised toward Alternative 2 only if profiling shows the count is a problem
*and* the transition-alignment gate still passes at the larger $\gamma$. $H_\text{ref}=1080$ and
$\delta=0.15$ stand (OQ-LR-1 and the inherited anti-flicker value). **Background and alternatives
retained below.**

**Problem.** The policy has two operating-point parameters that trade rendered quality against
tile count and build cost; the LOD thresholds and per-LOD footprints are their outputs, so the
choice sets the size of every downstream build. $\tau$ (pixels) is the maximum on-screen
geometric error tolerated; since $R_\ell \propto 1/\tau$, a larger $\tau$ moves every LOD
transition nearer and coarsens sooner — going from $\tau=1$ px to $\tau=2$ px halves every
adequacy range. $\gamma \in (0,1)$ is the fraction of each LOD band allowed to be single-LOD-impure
(§2); the footprint is $f_\ell \approx (\gamma/\sqrt2)(R_{\ell+1}-R_\ell)$, so the tile count per
LOD is $N_\ell = A/f_\ell^2 \propto 1/\gamma^2$ — halving $\gamma$ quadruples the tiles per LOD
and simultaneously tightens the crossfade-alignment margin $h/B$ (§3). The two remaining
parameters are not free here: the reference resolution $H_\text{ref}$ scales all $R_\ell$
($\propto H$) and is fixed by the resolution of OQ-LR-1, and the hysteresis $\delta$ inherits the
established $0.15$ anti-flicker value (from [terrain.md §LOD Hysteresis Band](terrain.md)) unless a
specific reason to change it arises. The operating point sets the absolute tile count for a region
— at the illustrative $\tau=1$, $\gamma=0.25$, $H_\text{ref}=1080$ the finest footprint is
$\approx 190$ m — and therefore the build time, VRAM, and draw-call load, and it gates the
(expensive) re-tile. Affected use cases: UC-LR-1, UC-LR-2, UC-LR-4.

**Alternatives.** (Each is a $(\tau, \gamma)$ pair; $H_\text{ref}$ from OQ-LR-1 and $\delta=0.15$
throughout.)

1. **Pixel-accurate, generous tiles — $\tau = 1$ px, $\gamma = 0.25$.** *Benefits:* sub-pixel LOD
   error at the reference resolution; the largest crossfade-alignment margin of the set (smallest
   $h/B$), so the per-LOD transition is the most forgiving; crispest result. *Drawbacks:* the
   highest tile count and build cost (finest footprint $\approx 190$ m at $1080$p → the most fine
   tiles), and the heaviest VRAM / draw-call load. *Prerequisite:* the residency streaming manager
   (IP-LV-7, done) to bound VRAM to a neighborhood of the aircraft.
2. **Pixel-accurate, tighter tiles — $\tau = 1$ px, $\gamma = 0.5$.** *Benefits:* the same pixel
   accuracy with $\approx 4\times$ fewer tiles than Alternative 1 (finest footprint $\approx 380$
   m); a much lighter build. *Drawbacks:* half of each LOD band is single-LOD-impure, so the
   crossfade must span a larger share of the band and the alignment margin $h/B$ is tighter —
   a higher risk that the per-LOD crossfade shows a seam. *Prerequisite:* the transition-alignment
   gate (Test Strategy) must pass at this $\gamma$.
3. **Performance-first — $\tau = 2$ px, $\gamma = 0.35$.** *Benefits:* adequacy ranges halved
   (coarsen sooner) combined with a moderate $\gamma$ gives far fewer tiles overall and the
   lightest runtime of the set. *Drawbacks:* up to a $2$-pixel geometric error on screen at the
   reference resolution — mildly visible LOD popping / under-detail; coarser near-ground detail
   transitions. *Prerequisite:* confirm that a $2$-pixel error is acceptable for the intended use.
4. **Match the maximum anticipated display — $\tau = 1$ px, $\gamma = 0.25$, $H_\text{ref} = 2160$.**
   *Benefits:* pixel-accurate even on a $4$K display with no runtime recompute (pairs with OQ-LR-1
   Alternative 1). *Drawbacks:* $\approx 4\times$ the tile count of Alternative 1 at the reference
   (footprints halve); the heaviest of all options, and wasteful when the display is usually
   $1080$p. *Prerequisite:* a justification that $4$K is the operating target.

**Recommendation.** Alternative 1 ($\tau = 1$ px, $\gamma = 0.25$) as the starting operating point,
confirmed at IP-LV-6. It is pixel-accurate with the safest alignment margin, and its tile-count
cost is precisely what the streaming manager (IP-LV-7) exists to absorb; the first per-LOD build
is treated as a tuning iteration, and $\gamma$ is raised toward Alternative 2 only if profiling
shows the tile count is a problem *and* the alignment gate still passes at the larger $\gamma$.
Blocking the re-tile run.

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
| [live_sim_view.md](live_sim_view.md) | Streaming format/manager (OQ-LS-20/21); OQ-LS-22 resolved to Alternative 3 (per-LOD footprints), which this policy defines |
