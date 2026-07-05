# Live-Viewer Terrain Rendering Corrections — Implementation Plan

**Scope:** Two related live-viewer terrain-rendering defects, both surfacing as "stacked terrain
surfaces the aircraft drops through":

1. **Frame reconciliation (done).** Reconcile the terrain *surface* with the single world-origin ENU
   frame the aircraft is projected into. Resolves [live_sim_view.md Issue 8](../design/live_sim_view.md)
   via [OQ-LS-19](../design/live_sim_view.md) (Option 1 — per-tile node rotation). Issue 7 re-marked
   incomplete (aircraft-only). — **IP-LV-1..3.**
2. **LOD stacking (pending).** Make per-node distance culling select one LOD per location by sizing each
   tile smaller than its own LOD's culling band. The original stacking came from single large coarse
   tiles ([OQ-LS-18](../design/live_sim_view.md) resolved the observation). The tile-sizing *strategy*
   was then resolved by [OQ-LS-22](../design/live_sim_view.md) to **per-LOD footprints scaled to each
   LOD's band** (Alternative 3) — small fine tiles, large coarse tiles — with LOD thresholds set by a
   screen-space-error policy rather than the earlier unfounded distance convention. An interim
   uniform-footprint tiling was built first and is now superseded. This gated on: **triangulator speed**
   (done, IP-LV-4), a **streamable chunk format** (done, IP-LV-8), a **streaming manager** (done,
   IP-LV-7), and now the **per-LOD footprint + SSE-threshold rework** (IP-LV-9/5/10). — **IP-LV-4..10.**

**Design authority:** [live_sim_view.md Issue 8 / OQ-LS-19](../design/live_sim_view.md),
[live_sim_view.md Issue 9 / OQ-LS-18 / OQ-LS-22](../design/live_sim_view.md),
[terrain_lod_rendering.md](../design/terrain_lod_rendering.md) (tile-scale + threshold policy) built on
[lod_culling_geometry.md](../algorithms/lod_culling_geometry.md) and
[screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md),
[terrain.md §Local Tile Frame](../design/terrain.md).

**Background (1 — frame):** tiles are placed at their centroid's world-origin ECEF→ENU offset (correct)
but their vertices live in each tile's **own centroid-tangent ENU frame** and the glTF nodes carry
**translation only**. The two diverge by the per-tile/per-LOD tangent-plane tilt (Earth curvature). The
fix adds a per-tile node rotation so translation+rotation places each tile exactly in the world-origin
frame.

**Background (2 — LOD stacking):** [`_LOD_CELL_SIDE_DEG`](../../python/tools/terrain/build_terrain.py)
sets each cell side to `100 × grid_spacing(lod)`, so tile *footprints* differ ~3× per LOD (L2 ≈ 600 m,
L4 ≈ 6 km). Godot culls by camera-to-**centroid** distance, which selects one LOD per location only when
a tile ≈ its slant range. A big coarse tile centered at the origin has one centroid, so it is all-or-nothing:
it either stacks on top of the fine LODs at the airport, or (if culled by its band) removes all distant
terrain when viewed from the center. The governing condition (derived in
[lod_culling_geometry.md](../algorithms/lod_culling_geometry.md)) is that a tile's half-extent must be
small relative to *its own* LOD's band width, $h_\ell \le \tfrac12\gamma(R_{\ell+1}-R_\ell)$ — a **per-LOD**
bound. A first attempt used a *uniform* small footprint for all LODs (which satisfies the bound at every
level but only by applying the finest LOD's tiny size everywhere: ≈ 6× more tiles than necessary and
geometrically redundant coarse levels — measured at 25,620 tiles for the 12 km flight region at 400 m).
[OQ-LS-22](../design/live_sim_view.md) therefore resolved the strategy to **per-LOD footprints**
$f_\ell \propto (R_{\ell+1}-R_\ell)$, with the thresholds $R_\ell$ set by screen-space error rather than the
old distance convention. The uniform tiling is retained only as an interim baseline. See
[terrain_lod_rendering.md](../design/terrain_lod_rendering.md).

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LV-1 | **done** | In `export_gltf.py`, emit a per-tile node **rotation** $R = R_\text{origin}^{\top} R_\text{centroid}$ (the relative orientation of the tile's centroid-ENU basis vs the world-origin-ENU basis, in the glTF axis convention `(east, up, −north)`) alongside the existing centroid translation. Vertices stay centroid-relative (float32 precision preserved). Unit test the rotation math: a vertex's `(translation + R·v_centroid)` must equal its exact world-origin ECEF→ENU position within float tolerance. | — | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-2 | **done** | Re-exported the `small_uas_ksba_flight` GLB via `build_terrain.py` (all 13 display-tile nodes now carry rotations). Plugin `.dll` verified already current and curvature-aware (`SimulationReceiver` reads `viewer_x/y/z_m`), so no `./build.sh gdext` rebuild was needed. | IP-LV-1 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-3 | quantitative done; visual pending | **Quantitative PASS:** every re-exported node rotation matches an independent pyproj ECEF→geodetic recomputation of $R = P(R_\text{origin}^{\top}R_\text{centroid})P^{\top}$ to 4.4e-16 (machine precision). **Visual (user):** run `run_sim.sh` and confirm (a) a single terrain surface across all LOD transitions (no stacked layers) and (b) the aircraft sits on the surface at touchdown at both dataset center and edges. Note: full single-surface confirmation is blocked by the LOD-stacking defect (IP-LV-4..6). | IP-LV-2 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-4 | **done** | **Fixed the terrain triangulator speed (prerequisite for everything below; `WORK-TB-1`).** Profiling confirmed the ~3 s/cell was fixed per-cell raster I/O, not Delaunay: `triangulate` called `rasterio.sample()` once **per grid point** (~400 ms/cell) and `colorize` read the **entire** imagery raster **per cell** (~320 ms/cell). Fix: a new `RasterSampler` ([`raster_sample.py`](../../python/tools/terrain/raster_sample.py)) reads each raster once and samples by vectorized floor-index (bit-exact to `rasterio.sample`); `triangulate`/`colorize` take a sampler instead of a path; `verify`'s per-facet Python edge loop is vectorized with `np.unique`; and `build_terrain` gains a `workers` param that distributes cells over a `ProcessPoolExecutor` (each worker reads the rasters once via a process-local cache; only small path/bbox tuples cross the process boundary). **Measured:** per-cell raster cost 730 ms → ~0.3 ms; output bit-identical to the old path (serial and parallel); remaining per-cell cost is now Delaunay-bound. | — | [terrain-implementation-plan.md WORK-TB-1](../roadmap/terrain-implementation-plan.md) |
| IP-LV-5 | **done** (re-tile run pending, user) | **Per-LOD footprint tiling + screen-space-error thresholds.** New [`lod_policy.py`](../../python/tools/terrain/lod_policy.py) computes the adequacy ranges $R_\ell = \varepsilon_\ell H_\text{ref}/(2\tau\tan(\phi/2))$ (transition errors 3/10/30/100/300/1000 m) and per-LOD footprints $f_\ell \approx \tfrac{\gamma}{\sqrt2}(\text{band width})$ from the adopted parameters ($\tau=1$, $\gamma=0.25$, $H_\text{ref}=1080$, $\phi_\text{ref}=90°$, $\delta=0.15$), tested in `test_lod_policy.py`. [`build_terrain.py`](../../python/tools/terrain/build_terrain.py) tiles **each LOD on its own grid** at $f_\ell$, records `lod_policy` in the descriptor, and passes the per-LOD footprints to `export_chunked_terrain` (the interim `tile_footprint_m` / uniform path removed). Real-DEM validation: R=[1620…540000] m, f=[286…222739] m, coarse LODs collapse to 16/4/1/1 tiles (was 3660 each), all triangulate cleanly, total 8543 tiles (3× fewer; ~6× at equal finest footprint). **The re-tile of `small_uas_ksba_flight` is a separate user run.** | IP-LV-4, IP-LV-8, IP-LV-9 | [terrain_lod_rendering.md](../design/terrain_lod_rendering.md), [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md), [live_sim_view.md OQ-LS-22](../design/live_sim_view.md) |
| IP-LV-9 | **done** | **Per-LOD chunk grid.** [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py) descriptor now records `tile_footprints_m` / `chunk_sizes_m` as **per-LOD lists**; `assign_tiles_to_chunks(tiles, origin, chunk_sizes_m)` keys each tile on `chunk_sizes_m[lod]`; [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) reads `_chunk_sizes_m` and its `chunk_coord_for_position(pos, lod)` / streaming use the per-LOD size. Coordinate addressing `(lod, cx, cy)` unchanged. Tests updated + `test_per_lod_chunk_sizes_partition_independently`; GDScript compiles clean. | IP-LV-8 | [terrain_lod_rendering.md §Integration](../design/terrain_lod_rendering.md), [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |
| IP-LV-10 | **done** | **Adopt screen-space-error thresholds in the viewer (runtime bands).** [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) removed the convention `_LOD_VIS_*` constants; `_recompute_lod_bands()` builds the band edges $[0,R_1,…,R_6,R_7]$ from the descriptor's `lod_policy` at the **live** viewport height and (KEEP_WIDTH-derived) **vertical** FOV, sets each node band to $[R_\ell(1-\delta)-h_\ell,\ R_{\ell+1}(1+\delta)+h_\ell]$ with `VISIBILITY_RANGE_FADE_SELF`, and drives `R_fetch`/`R_unload` from the same live edges. `_maybe_recompute_lod_bands()` re-applies to all resident chunks on viewport/FOV change (OQ-LR-1 Alt 2). At the reference (vfov 90°, H 1080) the edges reproduce `lod_policy.py` exactly; GDScript compiles clean. | IP-LV-9 | [terrain_lod_rendering.md §1 / Integration / OQ-LR-1](../design/terrain_lod_rendering.md), [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md) |
| IP-LV-6 | ready (user-run) | **Validate culling + profile runtime.** Confirm **distinct opaque LODs vs. observer position** = 1 everywhere in the region (only the intended crossfade $B_\ell$ overlap at transitions); in `run_sim.sh` a single terrain surface renders and the aircraft no longer passes through stacked layers. **Per-LOD transition-alignment gate:** verify the crossfade condition $h_{\ell+1} \lesssim B_{\ell+1}$ ([terrain_lod_rendering.md Test Strategy](../design/terrain_lod_rendering.md)) both analytically and in-engine (grazing + nadir) — if per-LOD crossfades cannot be made seamless, escalate to OQ-LS-22 rather than reverting silently. Profile in the Godot editor (draw calls + frame time + VRAM). The parameter derivation the earlier note anticipated is now delivered as the screen-space-error policy ([terrain_lod_rendering.md](../design/terrain_lod_rendering.md) + [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md)); this item confirms/tunes its parameters (OQ-LR-2) against measurement rather than hand-setting them. | IP-LV-5, IP-LV-9, IP-LV-10, IP-LV-7 | [terrain_lod_rendering.md](../design/terrain_lod_rendering.md), [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-7 | **done** | **Tile-residency / streaming manager.** Implemented in [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd): `_update_terrain_streaming` (per-`_process`, re-evaluated only when the aircraft crosses into a new chunk) pages fine-LOD chunks (`0..stream_lod_max`) by proximity with **asymmetric hysteresis** (OQ-LS-21 Alternative 1) — load within `R_fetch(lod) = visibility_end(lod) + fetch_margin_m`, free only beyond `R_unload(lod) = visibility_end(lod) + unload_margin_m` (`unload_margin_m > fetch_margin_m`). Coarse LODs (`> stream_lod_max`) load once and stay resident as the backdrop. Tunable `@export`s: `enable_streaming`, `stream_lod_max` (default 2), `fetch_margin_m` (800), `unload_margin_m` (2400). GDScript compiles clean under Godot 4.6.2; in-engine validation is IP-LV-6. | IP-LV-8 | [live_sim_view.md OQ-LS-21](../design/live_sim_view.md) |
| IP-LV-8 | **done** | **Streamable asset format.** Replaced the monolithic GLB with **per-(LOD, chunk) GLB files** addressed by **integer chunk coordinate** `(lod, cx, cy)` (OQ-LS-20 Alternative 2) plus a top-level `descriptor.json` (world origin, `tile_footprint_m`, `chunk_footprints`, `chunk_size_m`, LOD count, coverage bounds, sparse chunk list) — **no flat all-tiles manifest**, so the format scales to several-hundred-km regions. New [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py) (`chunk_index` / `tile_chunk_coord` / `assign_tiles_to_chunks` / `TerrainChunkDescriptor` / `export_chunked_terrain`, tested in `test_terrain_chunks.py`); [`build_terrain.py`](../../python/tools/terrain/build_terrain.py) emits chunks with configurable `tile_footprint_m` / `chunk_footprints` (config `terrain.*`, `--tile-footprint-m` / `--chunk-footprints`, or args; defaults 400 m / 4) and `terrain_config.json` now carries `terrain_descriptor_path`; [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) reads the descriptor and exposes `load_chunk` / `free_chunk` / `chunk_coord_for_position` (loads all chunks up front for now; IP-LV-7 adds proximity streaming). GDScript compiles clean under Godot 4.6.2. | — | [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |

**Execution order** (the numeric IDs predate the dependency analysis and no longer match it): the frame
work (IP-LV-1/2/3), the triangulator fix (IP-LV-4), the streamable chunk format (IP-LV-8), and the
streaming manager (IP-LV-7) are **done**. OQ-LS-22 resolved to **per-LOD footprints** (Alternative 3),
which superseded the interim uniform-footprint tiling and re-scoped the remainder to the screen-space-error
policy of [terrain_lod_rendering.md](../design/terrain_lod_rendering.md). All design questions are resolved (OQ-LS-22 → per-LOD footprints; OQ-LR-2 → $\tau=1$ px, $\gamma=0.25$;
OQ-LR-1 → runtime bands, footprints baked at $H_\text{ref}=1080$) and **all code is implemented**:
IP-LV-9 (per-LOD chunk grid), IP-LV-5 (per-LOD footprint tiling + SSE thresholds, `lod_policy.py`), and
IP-LV-10 (runtime SSE visibility bands + streaming radii) are **done** — Python tests green, GDScript
compiles clean, per-LOD tiling validated on the real DEM. The only remaining item is **IP-LV-6**, which
is user-run: re-tile `small_uas_ksba_flight` with the new build, then validate in-engine (one opaque LOD
per location, the per-LOD transition-alignment gate, and a Godot profile of draw calls / frame time /
VRAM).

---

## Notes

- **Windowed imagery reads (build memory fix).** The first per-LOD re-tile OOM'd: IP-LV-4's
  `RasterSampler` reads the whole raster into memory, and with a ~0.5 GB NAIP tile held per parallel
  worker (`--workers 12`), the 34 GB machine hit Windows' commit limit ("paging file too small") during
  worker startup. Fix: a new [`WindowedRasterSampler`](../../python/tools/terrain/raster_sample.py) keeps
  the imagery dataset open and reads only each tile's bounding window (decimation-capped for
  region-spanning coarse tiles), so imagery memory is bounded by the window, not the raster × workers.
  The DEM stays whole-read (small). Wired into `build_terrain` (serial + `ProcessPoolExecutor`); tested in
  `test_raster_sample.py` (bit-exact to `rasterio.sample` for undecimated windows) and validated on the
  real 0.5 GB NAIP tile (open 0.002 s, no full read). Design decision recorded in
  [terrain_build.md §Design Decisions](../design/terrain_build.md).
- **LOD budget audit (three fixes).** A texture-clamp warning during the first re-tile prompted an audit
  of per-LOD budgets stale relative to the SSE per-LOD design: (1) **texture resolution** — `_LOD_MAX_PIXEL_DIM`
  (fixed 2048–8192 px) was ~20× too large for coarse LODs (an L3 tile viewed from 16 km rendered at 8192 px;
  16 resident L3 tiles ≈ 3 GB VRAM). Replaced by [`lod_policy.lod_texture_max_px`](../../python/tools/terrain/lod_policy.py)
  (SSE texel budget → ~200 px/tile; coarse resident textures 4.7 GB → 4.2 MB). (2) **Camera far clip** —
  fixed at 20 km, but the SSE bands put L3 at 16–54 km etc., so terrain and the whole coarse backdrop past
  20 km were clipped; `TerrainLoader.gd` now sets the far clip from the descriptor's dataset extent
  (region diagonal × 1.5). (3) **Dead constants** — build_terrain's `_VIS_BEGIN_M`/`_VIS_END_M` (the old
  convention bands, superseded by the runtime SSE bands) removed. Documented in
  [terrain_lod_rendering.md §4/§5](../design/terrain_lod_rendering.md) (+ UC-LR-5/6) and terrain_build.md.
- **Analytic grid triangulation + parallel render (build speed & robustness).** Two follow-ups after the
  re-tile ran: (1) the per-tile texture-render loop was still **serial** — it historically processed only
  the ~13 nested display tiles, but IP-LV-5 made it process all 8,543, turning it into the wall-clock long
  pole. Parallelized over the same `ProcessPoolExecutor` (`render_mosaic` reads only each tile's window and
  imports no scipy, so it parallelizes cleanly). (2) The tiles are regular grids, so
  [`triangulate`](../../python/tools/terrain/triangulate.py) now emits the two-triangles-per-cell pattern
  **analytically** (`_grid_triangulation`, pure numpy, consistent up-winding) instead of `scipy.Delaunay`
  — validated that scipy is no longer imported on the build path (only lazily on the unused `boundary_points`
  path). That removes scipy/OpenBLAS from the workers **at the root**, so the thread-cap below is now only a
  minor safety margin (numpy still bundles OpenBLAS). Both loops emit count/rate/ETA progress. Recorded in
  [terrain_build.md §Design Decisions](../design/terrain_build.md) and the OQ-TB-4 note.
- **BLAS thread cap (second build memory fix).** After windowing the imagery, the re-tile still OOM'd —
  but at *scipy import* in the workers (`DLL load failed … paging file is too small`), before any imagery
  is touched. Cause: scipy/numpy's bundled OpenBLAS reserves per-thread scratch buffers sized to the CPU
  count, so 12 worker processes each importing it blew the Windows commit limit. Fix: `build_terrain`
  sets `OPENBLAS_NUM_THREADS`/`OMP_NUM_THREADS`/`MKL_NUM_THREADS`/`NUMEXPR_NUM_THREADS` = 1 at module top
  (before scipy loads, inherited by each spawned worker) — the triangulation is single-threaded and
  parallelised at the process level, so no speed loss. With both fixes the re-tile is memory-safe at full
  parallelism.
- **Crossfade margins (IP-LV-10 fix).** `_apply_visibility_ranges` now sets
  `visibility_range_begin_margin` / `visibility_range_end_margin` to the hysteresis-overlap width
  `2·δ·edge`, not just `fade_mode = FADE_SELF`. Without the margins the fade distance is zero (a hard
  cut), so the band overlap would render two opaque LOD surfaces at once — stacking. The exact Godot
  margin semantics (fade direction/extent) still need in-engine confirmation at IP-LV-6.
- **Transition-alignment gate (analytical, pre-re-tile).** With the adopted $\gamma=0.25$, $\delta=0.15$
  and the SSE band edges, the per-LOD crossfade-alignment ratio is **marginal**: under the
  coarser-tile-dominates approximation $h_{\ell+1}/B_{\ell+1} \approx 0.83$–$0.97$ (passes), but under
  the full worst-case offset $(h_\ell+h_{\ell+1})/B_{\ell+1} \approx 1.1$–$1.4$ (exceeds) at every
  transition. So per-LOD crossfades may seam under worst-case grazing geometry — exactly the risk
  [lod_culling_geometry.md](../algorithms/lod_culling_geometry.md) flagged. **Mitigation knobs** if IP-LV-6
  shows seams: raise $\delta$ (wider crossfade $B=2\delta R$) or lower $\gamma$ (smaller tiles → smaller
  offset); both are `lod_policy.py` constants, no structural change. If neither suffices, escalate to
  OQ-LS-22.
- **IP-LV-1 implementation (done).** The per-tile rotation is emitted in
  [`export_gltf.py`](../../python/tools/terrain/export_gltf.py): `R = R_originᵀ·R_centroid` conjugated
  by the ENU→glTF axis permutation `P` (`R_gltf = P·R·Pᵀ`) and written as a node quaternion.
  Unit-tested in [`test_export_gltf.py`](../../python/test/test_export_gltf.py)
  (`test_per_tile_rotation_places_vertices_in_world_frame`, `..._identity_at_origin`).
- **Shared geometry/geodesy consolidation (done, out of original scope).** The geodetic/rotation math
  the exporter needed was previously duplicated across the terrain tools (WGS-84 constants in 4 files;
  ENU/ECEF/quaternion helpers in 6). It was consolidated into project-level
  [`tools/geodesy.py`](../../python/tools/geodesy.py) (WGS-84, ECEF/ENU conversions, distances) and
  [`tools/geometry.py`](../../python/tools/geometry.py) (`euler_zyx_to_rotation_matrix`,
  `rotation_matrix_to_quaternion`), with `export_gltf`, `colorize`, `triangulate`, `verify`,
  `build_terrain`, and `trajectory_view` repointed. Terrain entry scripts pick up `tools/` on
  `sys.path` via [`tools/terrain/_bootstrap.py`](../../python/tools/terrain/_bootstrap.py). New tests:
  `test_geodesy.py`, `test_geometry.py`.
- **IP-LV-4 implementation (done).** The dominant build cost was per-cell raster I/O, not Delaunay:
  [`triangulate`](../../python/tools/terrain/triangulate.py) called `rasterio.sample()` once per grid
  point and [`colorize`](../../python/tools/terrain/colorize.py) read the full imagery raster per cell.
  Both now take a [`RasterSampler`](../../python/tools/terrain/raster_sample.py) (each raster read once,
  vectorized floor-indexed sampling, bit-exact to `rasterio.sample`), [`verify`](../../python/tools/terrain/verify.py)'s
  per-facet edge loop is vectorized (`np.unique`), and [`build_terrain`](../../python/tools/terrain/build_terrain.py)
  gained a `workers` parameter (default 1 in-process; the CLI defaults `--workers` to all cores) that
  distributes cells over a `ProcessPoolExecutor` with a process-local sampler cache. New tests:
  `test_raster_sample.py`; `test_triangulate.py`/`test_colorize.py`/`test_pipeline.py` updated to the
  sampler API. Measured per-cell raster cost 730 ms → ~0.3 ms; output bit-identical (serial and parallel).
- **IP-LV-8 implementation (done).** [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py)
  groups tiles into `(lod, cx, cy)` chunks by centroid ENU (`chunk_index` adds a sub-micron epsilon so a
  centroid on a chunk boundary — e.g. at the world origin — is not flipped into the chunk below by
  floating-point noise) and writes per-chunk GLBs (reusing `export_gltf`, so per-tile textures and the
  IP-LV-1 curvature rotation are unchanged) plus `descriptor.json`. `build_terrain` emits this under
  `derived/terrain_tiles/`, returns the descriptor path, and records `terrain_descriptor_path` in
  `terrain_config.json` (replacing `glb_path`).
- **Loader dispatch (design note).** [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd)
  dispatches on the config: `terrain_descriptor_path` → chunked streaming path; `glb_path` → single
  monolithic GLB. The single-GLB path is **not** a deprecated-format shim — it serves the distinct
  `gen_test_assets` static axis-prism debug scene, which builds its terrain via `trimesh` (not
  `export_gltf`) and has no need for chunking. Converting that debug asset to the chunk format (to drop
  the single-GLB branch entirely) is a possible future cleanup, not required by the live-viewer feature.
- **Gap (work item): no export-only rebuild path.** Re-exporting the GLB requires either the full
  `build_terrain --force` (which deletes the cached `.las_terrain` and re-runs triangulation + texture
  mosaic) or bespoke tooling. IP-LV-4 makes the triangulation portion fast (minutes, parallel), but a
  `build_terrain --export-only` mode that reloads the cached `.las_terrain` tiles and re-renders mosaics
  from the cached source rasters, then calls the existing `export_gltf`, would still make pure
  GLB-format changes cheaper. Surfaced here per the cleanup guideline; not yet implemented.
- **C++ `TerrainMesh::exportGltf` mirrors the Python path** (same translation-only placement). It is not on
  the live-viewer path (the GLB the viewer loads is produced by `export_gltf.py`), but if it is used for
  any export it needs the same per-tile rotation for consistency — surface as a follow-up if confirmed in
  use.
- **Precision:** the per-tile rotation preserves the centroid-relative float32 vertex design
  ([terrain.md §Local Tile Frame](../design/terrain.md)); only one quaternion per node is added.
- **No aircraft-side change:** `GodotEnuProjector` and the curvature-aware `SimulationReceiver` are
  unchanged; the correction is entirely in terrain export.
