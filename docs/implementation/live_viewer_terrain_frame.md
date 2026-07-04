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
| IP-LV-5 | re-scoped (blocked on OQ-LR-2) | **Per-LOD footprint tiling + screen-space-error thresholds.** OQ-LS-22 resolved to **Alternative 3 (per-LOD footprints)**, which **supersedes the uniform-footprint tiling first implemented here** (the uniform `_cell_side_deg` / `_effective_spacing_deg` grid remains as an interim baseline but is not the target design). Re-scope per [terrain_lod_rendering.md](../design/terrain_lod_rendering.md): compute per-LOD adequacy ranges $R_\ell = \varepsilon_\ell H_\text{ref}/(2\tau\tan(\phi/2))$ and per-LOD footprints $f_\ell \approx \tfrac{\gamma}{\sqrt2}(R_{\ell+1}-R_\ell)$ from the design parameters and the simplification error schedule; tile each LOD on its **own** grid at $f_\ell$ (small fine tiles, large coarse tiles, ≈ 6× fewer tiles than uniform, no redundant coarse geometry); export region-wide per LOD, then re-tile `small_uas_ksba_flight`. The numeric parameters ($\tau, H_\text{ref}, \gamma, \delta$) are terrain_lod_rendering.md **OQ-LR-2** and gate the run. | IP-LV-4, IP-LV-8, IP-LV-9, OQ-LR-2 | [terrain_lod_rendering.md](../design/terrain_lod_rendering.md), [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md), [live_sim_view.md OQ-LS-22](../design/live_sim_view.md) |
| IP-LV-9 | ready | **Per-LOD chunk grid.** The chunk format currently records a single `chunk_size_m`; per-LOD footprints require a **per-LOD chunk size** (chunk side = integer × $f_\ell$). Extend [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py) and the descriptor to record per-LOD chunk sizes, `assign_tiles_to_chunks` to key each tile on its LOD's size, and [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) `chunk_coord_for_position` / streaming to use the per-LOD size. Coordinate addressing `(lod, cx, cy)` is unchanged; only the metre-per-chunk factor becomes per-LOD. | IP-LV-8 | [terrain_lod_rendering.md §Integration](../design/terrain_lod_rendering.md), [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |
| IP-LV-10 | blocked on OQ-LR-2 | **Adopt screen-space-error thresholds in the viewer.** Replace the convention visibility bands ([`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) `_LOD_VIS_BEGIN_M`/`_LOD_VIS_END_M`, derived from the superseded 300 m rule) and the streaming margins with the SSE adequacy ranges $R_\ell$ (terrain_lod_rendering.md §1) plus per-LOD tile-radius padding and crossfade $B_\ell = 2\delta R_\ell$; drive `R_fetch`/`R_unload` from $R_\ell$. Optionally recompute bands at runtime from the live viewport height / FOV (terrain_lod_rendering.md OQ-LR-1). | IP-LV-9 | [terrain_lod_rendering.md §1](../design/terrain_lod_rendering.md), [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md) |
| IP-LV-6 | blocked on IP-LV-5/9/10 | **Validate culling + profile runtime.** Confirm **distinct opaque LODs vs. observer position** = 1 everywhere in the region (only the intended crossfade $B_\ell$ overlap at transitions); in `run_sim.sh` a single terrain surface renders and the aircraft no longer passes through stacked layers. **Per-LOD transition-alignment gate:** verify the crossfade condition $h_{\ell+1} \lesssim B_{\ell+1}$ ([terrain_lod_rendering.md Test Strategy](../design/terrain_lod_rendering.md)) both analytically and in-engine (grazing + nadir) — if per-LOD crossfades cannot be made seamless, escalate to OQ-LS-22 rather than reverting silently. Profile in the Godot editor (draw calls + frame time + VRAM). The parameter derivation the earlier note anticipated is now delivered as the screen-space-error policy ([terrain_lod_rendering.md](../design/terrain_lod_rendering.md) + [screen_space_lod_selection.md](../algorithms/screen_space_lod_selection.md)); this item confirms/tunes its parameters (OQ-LR-2) against measurement rather than hand-setting them. | IP-LV-5, IP-LV-9, IP-LV-10, IP-LV-7 | [terrain_lod_rendering.md](../design/terrain_lod_rendering.md), [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-7 | **done** | **Tile-residency / streaming manager.** Implemented in [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd): `_update_terrain_streaming` (per-`_process`, re-evaluated only when the aircraft crosses into a new chunk) pages fine-LOD chunks (`0..stream_lod_max`) by proximity with **asymmetric hysteresis** (OQ-LS-21 Alternative 1) — load within `R_fetch(lod) = visibility_end(lod) + fetch_margin_m`, free only beyond `R_unload(lod) = visibility_end(lod) + unload_margin_m` (`unload_margin_m > fetch_margin_m`). Coarse LODs (`> stream_lod_max`) load once and stay resident as the backdrop. Tunable `@export`s: `enable_streaming`, `stream_lod_max` (default 2), `fetch_margin_m` (800), `unload_margin_m` (2400). GDScript compiles clean under Godot 4.6.2; in-engine validation is IP-LV-6. | IP-LV-8 | [live_sim_view.md OQ-LS-21](../design/live_sim_view.md) |
| IP-LV-8 | **done** | **Streamable asset format.** Replaced the monolithic GLB with **per-(LOD, chunk) GLB files** addressed by **integer chunk coordinate** `(lod, cx, cy)` (OQ-LS-20 Alternative 2) plus a top-level `descriptor.json` (world origin, `tile_footprint_m`, `chunk_footprints`, `chunk_size_m`, LOD count, coverage bounds, sparse chunk list) — **no flat all-tiles manifest**, so the format scales to several-hundred-km regions. New [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py) (`chunk_index` / `tile_chunk_coord` / `assign_tiles_to_chunks` / `TerrainChunkDescriptor` / `export_chunked_terrain`, tested in `test_terrain_chunks.py`); [`build_terrain.py`](../../python/tools/terrain/build_terrain.py) emits chunks with configurable `tile_footprint_m` / `chunk_footprints` (config `terrain.*`, `--tile-footprint-m` / `--chunk-footprints`, or args; defaults 400 m / 4) and `terrain_config.json` now carries `terrain_descriptor_path`; [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) reads the descriptor and exposes `load_chunk` / `free_chunk` / `chunk_coord_for_position` (loads all chunks up front for now; IP-LV-7 adds proximity streaming). GDScript compiles clean under Godot 4.6.2. | — | [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |

**Execution order** (the numeric IDs predate the dependency analysis and no longer match it): the frame
work (IP-LV-1/2/3), the triangulator fix (IP-LV-4), the streamable chunk format (IP-LV-8), and the
streaming manager (IP-LV-7) are **done**. OQ-LS-22 resolved to **per-LOD footprints** (Alternative 3),
which superseded the interim uniform-footprint tiling and re-scoped the remainder to the screen-space-error
policy of [terrain_lod_rendering.md](../design/terrain_lod_rendering.md). The remaining chain is
**IP-LV-9 → (IP-LV-5, IP-LV-10) → IP-LV-6**: IP-LV-9 makes the chunk grid per-LOD; IP-LV-5 (per-LOD
footprint tiling + SSE thresholds in the build) and IP-LV-10 (SSE visibility bands + streaming radii in
the viewer) both depend on it and are gated on the numeric-parameter choice
[terrain_lod_rendering.md OQ-LR-2](../design/terrain_lod_rendering.md); IP-LV-6 validates culling, the
per-LOD transition-alignment gate, and runtime cost.

---

## Notes

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
