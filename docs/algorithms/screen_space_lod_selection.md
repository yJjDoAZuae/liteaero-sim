# Screen-Space Error LOD Selection

Level-of-detail selection decides, for a given observer distance, the coarsest representation of
an object whose rendered error is still imperceptible. This document derives the mapping from a
world-space error measure to its size in screen pixels, inverts it to obtain the slant range at
which each LOD becomes adequate, and adds a hysteresis dead-band expressed in the same pixel
units. It is the mathematical reference cited by the terrain rendering-LOD design; the geometric
consequences of applying these thresholds to finite-size tiles are analyzed in
[lod_culling_geometry.md](lod_culling_geometry.md).

The method is agnostic to the error measure: $\varepsilon$ may be a mesh's maximum vertical
deviation from the true surface, a texel's ground footprint, or any world-space length whose
on-screen projection defines "detail."

---

## Screen-Space Projection of a World-Space Error

Consider a perspective (pinhole) camera with vertical field of view $\phi$ and a viewport of
height $H$ pixels. At slant range $r$ from the camera, the view frustum has a vertical world
extent equal to its full height at that range,

$$
W(r) = 2\,r\,\tan\!\frac{\phi}{2},
$$

which is imaged across the $H$ pixels of the viewport. The pixel density (pixels per world
metre) at range $r$ is therefore $H / W(r)$, and a world-space length $\varepsilon$ oriented
perpendicular to the view direction projects to

$$
\boxed{\; s_\text{px}(\varepsilon, r) = \frac{\varepsilon\,H}{2\,r\,\tan(\phi/2)} \quad[\text{pixels}]. \;}
$$

This is exact for a feature at the image center lying in the plane perpendicular to the optical
axis. Off-axis and foreshortened features project *smaller*, so evaluating with the
perpendicular extent $\varepsilon$ is a conservative (upper) bound on the on-screen error —
appropriate for a quality guarantee. The horizontal case is identical with $\phi$ and $H$
replaced by their horizontal counterparts; the vertical convention is used throughout because
terrain relief error is predominantly vertical.

Two properties are used below:

$$
\frac{\partial s_\text{px}}{\partial r} = -\frac{\varepsilon H}{2 r^2 \tan(\phi/2)} < 0,
\qquad
s_\text{px}\ \text{is linear in } \varepsilon\ \text{and in } H,\ \text{and} \propto \cot(\phi/2).
$$

Screen-space error decreases monotonically with range and grows with resolution and with zoom
(smaller $\phi$).

---

## Required LOD at a Slant Range

Let the available representations be indexed $\ell = 0,1,\dots,L-1$ from finest to coarsest, with
world-space error

$$
\varepsilon_0 < \varepsilon_1 < \cdots < \varepsilon_{L-1},
$$

a monotone sequence (guaranteed when each coarser level is produced by error-bounded
simplification of the finer). Fix a **pixel tolerance** $\tau$ — the maximum on-screen error
permitted (e.g. $\tau = 1$ pixel). LOD $\ell$ is *adequate* at range $r$ when its projected error
does not exceed the tolerance:

$$
s_\text{px}(\varepsilon_\ell, r) \le \tau
\quad\Longleftrightarrow\quad
r \ge \frac{\varepsilon_\ell\,H}{2\,\tau\,\tan(\phi/2)} \;=:\; R_\ell .
$$

$R_\ell$ is the **adequacy range** of LOD $\ell$: the minimum distance at which it meets the
tolerance. Because $\varepsilon_\ell$ is increasing, the ranges are ordered
$R_0 < R_1 < \cdots < R_{L-1}$. The selection rule chooses the coarsest adequate level:

$$
\boxed{\; \ell^{*}(r) = \max\{\,\ell : R_\ell \le r\,\}
        = \max\Bigl\{\ell : \varepsilon_\ell \le \tfrac{2\,\tau\,r\,\tan(\phi/2)}{H}\Bigr\}. \;}
$$

Equivalently, the **transition** from LOD $\ell$ to LOD $\ell+1$ (finer to coarser, as range
increases) occurs at the boundary

$$
r_{\ell\to\ell+1} = R_{\ell+1} = \frac{\varepsilon_{\ell+1}\,H}{2\,\tau\,\tan(\phi/2)} .
$$

Below $R_1$ the finest level $\ell=0$ is used; beyond $R_{L-1}$ the coarsest level is used to
infinity. This replaces any distance table chosen by convention: every boundary is now a
function of the level's own error $\varepsilon_{\ell+1}$ and the three viewing parameters
$(H,\ \phi,\ \tau)$.

For textured tiles the same rule applies with $\varepsilon$ set to the texel ground size
$t_\ell$ and $\tau$ to a texture pixel tolerance $\tau_\text{tex}$; the mesh-LOD and
texture-LOD adequacy ranges are in general different, and a tile's rendered level must satisfy
the more demanding (smaller-range) of the two, or the two error budgets must be balanced so that
$R_\ell^\text{geom} \approx R_\ell^\text{tex}$ at every level.

---

## Hysteresis Dead-Band

Selecting strictly on $r = R_{\ell+1}$ causes rapid flip-flop ("popping") when the observer
hovers at a boundary. A dead-band is introduced by committing to a transition only after the
range crosses the boundary by a margin. Expressed as a fractional range band with half-width
$\delta$:

$$
r > R_{\ell+1}(1+\delta) \Rightarrow \text{commit to coarser } \ell+1, \qquad
r < R_{\ell+1}(1-\delta) \Rightarrow \text{revert to finer } \ell .
$$

The committed representation's on-screen error then ranges over
$s_\text{px} \in \bigl[\tau/(1+\delta),\ \tau/(1-\delta)\bigr]$, i.e. the dead-band corresponds
to a **pixel window**

$$
\Delta s_\text{px} = \tau\left(\frac{1}{1-\delta} - \frac{1}{1+\delta}\right)
                   = \frac{2\,\tau\,\delta}{1-\delta^2} \;\approx\; 2\,\tau\,\delta .
$$

So a range-fraction hysteresis $\delta$ is equivalent to allowing the on-screen error to drift by
$\approx 2\tau\delta$ pixels before switching. Choosing the dead-band directly in pixels,
$\Delta s_\text{px}$, and inverting gives $\delta \approx \Delta s_\text{px}/(2\tau)$. The
overlapping ranges $[\,R_{\ell+1}(1-\delta),\ R_{\ell+1}(1+\delta)\,]$ of adjacent levels define
the interval over which a crossfade blends them; its width is

$$
B_{\ell+1} = R_{\ell+1}\bigl[(1+\delta) - (1-\delta)\bigr] = 2\,\delta\,R_{\ell+1},
$$

which scales with the adequacy range and is the crossfade budget referenced by the tile-size and
transition-alignment constraints of [lod_culling_geometry.md](lod_culling_geometry.md).

---

## Numerical Properties

**Ordering and well-posedness.** The thresholds are strictly ordered iff the error sequence is
strictly increasing; equal errors ($\varepsilon_\ell = \varepsilon_{\ell+1}$) collapse two
levels to the same range and one is redundant. A simplification schedule with a strictly growing
error budget is a precondition.

**Geometric structure.** If the error sequence is geometric, $\varepsilon_\ell = \varepsilon_0\,\rho^{\ell}$
with ratio $\rho>1$, then the adequacy ranges are geometric with the *same* ratio,
$R_\ell = R_0\,\rho^{\ell}$, and each LOD band has width
$R_{\ell+1}-R_\ell = R_0\,\rho^{\ell}(\rho-1)$. The familiar "$\times\rho$ per level" spacing is
thus recovered — but its absolute scale $R_0 = \varepsilon_0 H/(2\tau\tan(\phi/2))$ is fixed by
the viewing parameters rather than chosen freely.

**Parameter sensitivity.** From $R_\ell \propto H,\ R_\ell \propto \tau^{-1},\ R_\ell \propto \cot(\phi/2)$:

- Doubling output resolution $H$ doubles every adequacy range (finer LODs are held to twice the
  distance).
- Loosening the tolerance from $\tau=1$ to $\tau=2$ px halves every range (coarsen twice as
  early).
- Halving the field of view (a $2\times$ zoom) increases $\cot(\phi/2)$ and pushes the ranges out
  (more detail demanded when magnified).

**Worked example.** Take a per-level vertical error sequence
$\varepsilon = (3,\,10,\,30,\,100,\,300,\,1000)\ \text{m}$ for $\ell=1,\dots,6$, a viewport
height $H = 1080$ px, tolerance $\tau = 1$ px, and $\phi = 90^\circ$ (so $\tan(\phi/2)=1$). Then

$$
R_\ell = \frac{\varepsilon_\ell \cdot 1080}{2\cdot 1\cdot 1} = 540\,\varepsilon_\ell
\;\Rightarrow\; R = (1620,\ 5400,\ 16200,\ 54000,\ 162000,\ 540000)\ \text{m}.
$$

A geometric "fixed samples per line of sight" convention with base $300\ \text{m}$ and ratio $3$
would instead place these boundaries at $(300, 900, 2700, 8100, 24300, 72900)\ \text{m}$ —
smaller by a factor of $\approx 5$–$7$. The two rules disagree by that factor precisely because
the convention encodes no output resolution, field of view, or error tolerance; the sign of the
disagreement here (the convention coarsens $\sim 5\times$ nearer than a $1$-pixel tolerance
allows) shows the convention under-resolves at $1080$p. The screen-space thresholds move with
$(H,\phi,\tau)$ as the numerical properties require, and the convention does not.

---

## References

| Reference | Relevance |
| --- | --- |
| D. Luebke et al., *Level of Detail for 3D Graphics*, Morgan Kaufmann, 2003 (ch. on view-dependent simplification and screen-space error) | Canonical screen-space error metric and pixel-tolerance LOD selection |
| J. H. Clark, "Hierarchical Geometric Models for Visible Surface Algorithms," *Communications of the ACM* 19(10), 1976 | Distance-based LOD selection foundation |
| P. Lindstrom, V. Pascucci, "Terrain Simplification Simplified," *IEEE TVCG* 8(3), 2002 | Screen-space error thresholds for view-dependent terrain LOD |
| T. Ulrich, "Rendering Massive Terrains using Chunked Level of Detail Control," SIGGRAPH Course, 2002 | Per-chunk screen-space error and hysteresis in practice |
