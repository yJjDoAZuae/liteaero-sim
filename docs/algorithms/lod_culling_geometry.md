# Distance-to-Centroid LOD Culling — Geometry and Error

Terrain is drawn as many discrete tiles, each carrying one level of detail (LOD), and a tile
is shown or hidden by comparing the observer-to-tile distance against a per-LOD visibility
band. The distance is customarily measured to the tile's **origin (centroid)**. This document
derives the geometric error that the centroid proxy introduces, the constraint it places on
tile size, and the conditions under which a tiling of many LODs selects one LOD per ground
location without gaps or double-drawing. It is the mathematical reference cited by the terrain
rendering-LOD design; the numeric error tolerances and screen-space thresholds themselves are
derived in a companion document,
[screen_space_lod_selection.md](screen_space_lod_selection.md).

The analysis is renderer-agnostic: it assumes only a per-tile visibility test of the form
"draw tile when $r_C \in [b,\,e]$", where $r_C$ is the observer-to-centroid distance and
$[b,e]$ is the LOD's band, optionally with a crossfade margin near the band edges.

---

## Centroid-Distance Selection Error

Let $\mathbf{X}$ be the observer position, $\mathbf{P}$ a point on the terrain surface inside a
tile, and $\mathbf{C}$ the tile centroid, all in a common metric frame. Define the true slant
range and the culling range:

$$
r_P = \lVert \mathbf{P} - \mathbf{X} \rVert, \qquad r_C = \lVert \mathbf{C} - \mathbf{X} \rVert .
$$

Write $\mathbf{d} = \mathbf{P} - \mathbf{X}$ and the unit view direction
$\hat{\mathbf{u}} = \mathbf{d}/r_P$. With $\boldsymbol{\Delta} = \mathbf{C} - \mathbf{P}$
(so $\lVert\boldsymbol{\Delta}\rVert \le h$, where $h$ is the tile's bounding radius),

$$
r_C = \lVert \mathbf{d} + \boldsymbol{\Delta} \rVert
    = \sqrt{r_P^2 + 2\,\mathbf{d}\cdot\boldsymbol{\Delta} + \lVert\boldsymbol{\Delta}\rVert^2}
    \;\approx\; r_P + \hat{\mathbf{u}}\cdot\boldsymbol{\Delta} ,
$$

to first order in $\lVert\boldsymbol{\Delta}\rVert / r_P$. The selection error is therefore

$$
\boxed{\; r_C - r_P \approx \hat{\mathbf{u}}\cdot\boldsymbol{\Delta}, \qquad
\lvert r_C - r_P \rvert \le h. \;}
$$

The bound $h$ is attained when $\boldsymbol{\Delta}$ is parallel to the line of sight — a tile
viewed **edge-on (grazing)**, which is the normal case for distant terrain seen from a low
aircraft. When the tile is viewed face-on (nadir), $\hat{\mathbf{u}}\cdot\boldsymbol{\Delta}=0$
and the residual is second order, $\lVert\boldsymbol{\Delta}\rVert^2/(2r_P)$. **Worst-case
analysis must therefore use the full extent $h$**, not the foreshortened value.

For a square tile of side $f$, the bounding radius is

$$
h = \frac{f}{\sqrt 2}\ \ (\text{circumradius}) \quad\text{or}\quad h \approx \frac{f}{2}\ \ (\text{in-plane, axis-aligned view}),
$$

so $h$ and the footprint $f$ are the same quantity up to an $O(1)$ factor; the constraints
below are stated in $h$ and hold for $f$ under the corresponding factor.

---

## Tile Extent and the Single-LOD Constraint

A tile carries exactly one LOD across its whole area, but the LOD that is *adequate* depends on
range and therefore varies across the tile's own depth span. Let $R_\ell$ denote the range at
or beyond which LOD $\ell$ is adequate (its rendered error is at most the chosen tolerance;
$R_\ell$ is derived in [screen_space_lod_selection.md](screen_space_lod_selection.md)). LOD
$\ell$ is the correct choice on the band $r_P \in [R_\ell,\,R_{\ell+1}]$.

A tile centered at culling range $r_C$ spans true ranges $[\,r_C - h,\ r_C + h\,]$ (from the
error bound above, at grazing incidence). Its single assigned LOD is adequate over the whole
tile only if the entire span lies within one LOD band. Near a band boundary $R$ the tile's near
edge $r_C - h$ and far edge $r_C + h$ straddle $R$ whenever

$$
\lvert r_C - R \rvert < h ,
$$

so a band of width $2h$ in observer distance around every LOD boundary is **impure** — the tile
contains ground that wants a finer LOD and ground that wants a coarser one, and no single-LOD
tile can satisfy both. The fraction of LOD $\ell$'s band that is single-LOD-pure is

$$
\text{purity}_\ell = 1 - \frac{2h}{R_{\ell+1}-R_\ell}.
$$

Requiring purity $\ge 1-\gamma$ for a chosen tolerance $\gamma \in (0,1)$ gives the tile-size
constraint

$$
\boxed{\; h \le \tfrac{1}{2}\,\gamma \,\bigl(R_{\ell+1}-R_\ell\bigr). \;}
$$

Because adequate-range boundaries grow geometrically (Section *Tile-Count Scaling*), the band
width $R_{\ell+1}-R_\ell$ grows with $\ell$, so the constraint is a **per-LOD bound**: the
admissible tile size scales with the LOD's range. Two consequences follow immediately:

- A **single uniform tile size** must satisfy the constraint at the *finest* LOD, where the band
  is narrowest: $h \le \tfrac12\gamma\,(R_1 - R_0)$. Using a footprint larger than this makes the
  finest transitions impure over a wide observer-distance band; using one small enough for the
  finest LOD applies that same tiny size to every coarse LOD, where it is far smaller than
  required.
- A **per-LOD tile size** $h_\ell \propto (R_{\ell+1}-R_\ell)$ meets the constraint at every LOD
  with the largest admissible tile at each level.

The impure band cannot be removed by shrinking $h$ to zero (that is the uniform-tiny limit,
which explodes the tile count); it is instead **absorbed by a crossfade** of width $\ge 2h$
centered on each boundary, over which the two LODs blend. A crossfade margin narrower than the
tile diameter leaves a residual hard transition; a margin wider than the LOD band over-blends
adjacent levels. The crossfade width is thus itself bounded below by $2h$ and above by the band
width, reinforcing $h \lesssim$ (band width).

---

## Transition Alignment Across LOD Grids

Consider a single ground point $\mathbf{P}$ and the two tiles that cover it at adjacent LODs:
$T_\ell$ with centroid $\mathbf{C}_\ell$ and $T_{\ell+1}$ with centroid $\mathbf{C}_{\ell+1}$.
Their culling ranges are, by the first-order result above,

$$
r_{C_\ell} \approx r_P + \hat{\mathbf{u}}\cdot(\mathbf{C}_\ell - \mathbf{P}), \qquad
r_{C_{\ell+1}} \approx r_P + \hat{\mathbf{u}}\cdot(\mathbf{C}_{\ell+1} - \mathbf{P}).
$$

The two tiles hand off at the shared boundary $R_{\ell+1}$, but each hands off when *its own*
culling range reaches $R_{\ell+1}$, and those differ by

$$
\boxed{\; \delta_{\text{align}} = r_{C_{\ell+1}} - r_{C_\ell}
   \approx \hat{\mathbf{u}}\cdot(\mathbf{C}_{\ell+1} - \mathbf{C}_\ell),
   \qquad \lvert\delta_{\text{align}}\rvert \le h_\ell + h_{\ell+1}. \;}
$$

Two tiling schemes bound this differently:

- **Shared cell grid (uniform footprint, all LODs on one grid).** The tile covering $\mathbf{P}$
  at every LOD is the *same cell*, so $\mathbf{C}_\ell = \mathbf{C}_{\ell+1}$ and
  $\delta_{\text{align}} = 0$ **identically**. Adjacent LODs transition at the same observer
  distance at every location — transitions are aligned by construction, independent of viewing
  angle. This is the defining geometric advantage of a shared uniform grid.
- **Distinct per-LOD grids (per-LOD footprint).** $\mathbf{C}_\ell \ne \mathbf{C}_{\ell+1}$ in
  general, so $\delta_{\text{align}}$ ranges up to $h_\ell + h_{\ell+1} \approx h_{\ell+1}$ (the
  coarser tile dominates). The fine tile's fade-out and the coarse tile's fade-in are offset by
  up to $\delta_{\text{align}}$ in observer distance.

For a distinct-grid transition to be free of gaps (both tiles transparent) and of stacking (both
opaque), the alignment offset must be absorbed by the crossfade of width $B$:

$$
\lvert\delta_{\text{align}}\rvert \lesssim B \quad\Longleftrightarrow\quad h_{\ell+1} \lesssim B.
$$

With per-LOD footprint $h_{\ell+1}\propto R_{\ell+1}$ and crossfade $B\propto R_{\ell+1}$, the
ratio $h_{\ell+1}/B$ is **constant across LODs** — the condition holds at all levels or none,
governed by a single dimensionless number set by the tile-size fraction $\gamma$ and the
crossfade width. Its numerical value, and therefore whether distinct-grid tiling transitions
cleanly, cannot be settled from geometry alone: it depends on the crossfade alpha profile of the
specific renderer and should be confirmed against the chosen thresholds (design document) and by
measurement. The shared-grid scheme sidesteps the question entirely ($\delta_{\text{align}}=0$)
but at the tile-count cost quantified next.

---

## Tile-Count Scaling

A region of area $A$ tiled at footprint $f$ contains

$$
N(f) = \frac{A}{f^2}
$$

tiles per LOD. For $L$ levels:

- **Uniform footprint $f$:** every LOD tiles the whole region, so the total is
  $N_\text{tot} = L\,A/f^2$. If $f$ is chosen to satisfy the finest-LOD single-LOD constraint,
  $f = f_0$, the total is $L\,A/f_0^2$ — the finest-LOD count replicated $L$ times. Moreover,
  coarse LODs whose native vertex spacing exceeds $f$ collapse to the same minimum mesh, so
  those levels are **geometrically identical**, differing only in texture: replicated geometry
  with no added relief detail.
- **Per-LOD footprint $f_\ell \propto R_\ell$ with geometric ranges $R_\ell = R_0\rho^{\ell}$
  ($\rho>1$):** $N_\ell = A/f_\ell^2 = (A/f_0^2)\rho^{-2\ell}$, a geometric series, so

$$
N_\text{tot} = \frac{A}{f_0^2}\sum_{\ell=0}^{L-1}\rho^{-2\ell}
             \;\xrightarrow[L\to\infty]{}\; \frac{A}{f_0^2}\cdot\frac{\rho^{2}}{\rho^{2}-1}.
$$

For $\rho=3$ the factor is $9/8=1.125$: the per-LOD total is about **$1.1\times$ the finest-LOD
count**, versus $L\times$ (e.g. $7\times$) for uniform. The ratio of the two schemes' totals is
$\approx L(\rho^2-1)/\rho^2$ — for $L=7,\rho=3$, per-LOD uses about **$6\times$ fewer** tiles at
equal finest-LOD resolution.

The absolute count also scales with region area $A$. Because $N_0 = A/f_0^2$ dominates both
schemes, a fixed finest footprint $f_0$ makes the tile count grow linearly with mapped area — the
constraint that dominates for large regions (e.g. a several-hundred-kilometre operating area at a
sub-kilometre finest footprint yields $10^6$–$10^7$ finest tiles regardless of scheme). Reducing
the total there requires either a coarser finest footprint (weaker near-ground culling) or
restricting the finest LODs to a neighborhood of the observer (residency streaming), which is
orthogonal to the per-LOD-versus-uniform choice analyzed here.

---

## Summary of Governing Quantities

| Quantity | Expression | Consequence |
| --- | --- | --- |
| Centroid selection error | $\lvert r_C - r_P\rvert \le h$ (grazing) | Culling distance is uncertain by one tile radius |
| Single-LOD purity | $1 - 2h/(R_{\ell+1}-R_\ell)$ | Impure band of width $2h$ at every LOD boundary |
| Tile-size bound | $h \le \tfrac12\gamma(R_{\ell+1}-R_\ell)$ | Admissible size scales with LOD range (per-LOD) |
| Transition alignment | $\lvert\delta_\text{align}\rvert \le h_\ell + h_{\ell+1}$ | $0$ for shared grid; $\sim h_{\ell+1}$ for per-LOD grids |
| Distinct-grid cleanliness | $h_{\ell+1} \lesssim B$ (crossfade width) | Constant ratio across LODs; renderer-dependent, must be measured |
| Tiles per LOD | $N(f)=A/f^2$ | Uniform: $L\,A/f_0^2$; per-LOD ($\rho=3$): $\approx 1.1\,A/f_0^2$ |

---

## References

| Reference | Relevance |
| --- | --- |
| J. H. Clark, "Hierarchical Geometric Models for Visible Surface Algorithms," *Communications of the ACM* 19(10), 1976 | Origin of distance-based LOD selection and its bounding-volume error |
| D. Luebke et al., *Level of Detail for 3D Graphics*, Morgan Kaufmann, 2003 | LOD selection metrics, screen-space error, transition/popping and blending |
| T. Ulrich, "Rendering Massive Terrains using Chunked Level of Detail Control," SIGGRAPH Course, 2002 | Chunked terrain LOD, per-chunk bounding-sphere distance selection, tile-count scaling |
| P. Lindstrom, V. Pascucci, "Terrain Simplification Simplified," *IEEE TVCG* 8(3), 2002 | View-dependent terrain LOD, screen-space error thresholds, crack/transition handling |
