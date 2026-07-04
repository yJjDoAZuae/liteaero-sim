# Live-Viewer Terrain Rendering Corrections — Implementation Plan

**Scope:** Two related live-viewer terrain-rendering defects, both surfacing as "stacked terrain
surfaces the aircraft drops through":

1. **Frame reconciliation (done).** Reconcile the terrain *surface* with the single world-origin ENU
   frame the aircraft is projected into. Resolves [live_sim_view.md Issue 8](../design/live_sim_view.md)
   via [OQ-LS-19](../design/live_sim_view.md) (Option 1 — per-tile node rotation). Issue 7 re-marked
   incomplete (aircraft-only). — **IP-LV-1..3.**
2. **LOD stacking (pending).** Make per-node distance culling select one LOD per location by using a
   **uniform small tile footprint for every LOD across the whole specified terrain region** (LOD =
   vertex density within the fixed footprint), so each tile's centroid ≈ its location and culling is
   crisp everywhere the aircraft flies. Resolves [live_sim_view.md Issue 9](../design/live_sim_view.md)
   via [OQ-LS-18](../design/live_sim_view.md). This produces thousands of small tiles, which gates on
   two things: **the triangulator speed** (done, IP-LV-4 — the build is now minutes, not hours) and
   a **tile-residency/streaming manager** (Godot culling does not unload memory; thousands of
   high-detail textures exceed VRAM). — **IP-LV-4..8.**

**Design authority:** [live_sim_view.md Issue 8 / OQ-LS-19](../design/live_sim_view.md),
[live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md),
[terrain.md §Local Tile Frame / §Slant-Range Selection](../design/terrain.md).

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
terrain when viewed from the center. **A per-LOD multiplier cannot fix this** — the correct structure is a
**uniform small tile footprint for all LODs**, so every tile's centroid ≈ its location. A `6× grid_spacing`
attempt (2026-07-02) confirmed this: it fixed L2/L3 but L4 (still 6 km) kept stacking, produced 302 tiles /
352 MB, and took ~3 h because the triangulator re-tiled the whole 24 km bbox at fine granularity. The
uniform-footprint design is region-scoped (the parameterized terrain extent), which bounds the small-tile
count, but still yields thousands of tiles — hence the triangulator-speed and streaming prerequisites.

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LV-1 | **done** | In `export_gltf.py`, emit a per-tile node **rotation** $R = R_\text{origin}^{\top} R_\text{centroid}$ (the relative orientation of the tile's centroid-ENU basis vs the world-origin-ENU basis, in the glTF axis convention `(east, up, −north)`) alongside the existing centroid translation. Vertices stay centroid-relative (float32 precision preserved). Unit test the rotation math: a vertex's `(translation + R·v_centroid)` must equal its exact world-origin ECEF→ENU position within float tolerance. | — | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-2 | **done** | Re-exported the `small_uas_ksba_flight` GLB via `build_terrain.py` (all 13 display-tile nodes now carry rotations). Plugin `.dll` verified already current and curvature-aware (`SimulationReceiver` reads `viewer_x/y/z_m`), so no `./build.sh gdext` rebuild was needed. | IP-LV-1 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-3 | quantitative done; visual pending | **Quantitative PASS:** every re-exported node rotation matches an independent pyproj ECEF→geodetic recomputation of $R = P(R_\text{origin}^{\top}R_\text{centroid})P^{\top}$ to 4.4e-16 (machine precision). **Visual (user):** run `run_sim.sh` and confirm (a) a single terrain surface across all LOD transitions (no stacked layers) and (b) the aircraft sits on the surface at touchdown at both dataset center and edges. Note: full single-surface confirmation is blocked by the LOD-stacking defect (IP-LV-4..6). | IP-LV-2 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-4 | **done** | **Fixed the terrain triangulator speed (prerequisite for everything below; `WORK-TB-1`).** Profiling confirmed the ~3 s/cell was fixed per-cell raster I/O, not Delaunay: `triangulate` called `rasterio.sample()` once **per grid point** (~400 ms/cell) and `colorize` read the **entire** imagery raster **per cell** (~320 ms/cell). Fix: a new `RasterSampler` ([`raster_sample.py`](../../python/tools/terrain/raster_sample.py)) reads each raster once and samples by vectorized floor-index (bit-exact to `rasterio.sample`); `triangulate`/`colorize` take a sampler instead of a path; `verify`'s per-facet Python edge loop is vectorized with `np.unique`; and `build_terrain` gains a `workers` param that distributes cells over a `ProcessPoolExecutor` (each worker reads the rasters once via a process-local cache; only small path/bbox tuples cross the process boundary). **Measured:** per-cell raster cost 730 ms → ~0.3 ms; output bit-identical to the old path (serial and parallel); remaining per-cell cost is now Delaunay-bound. | — | [terrain-implementation-plan.md WORK-TB-1](../roadmap/terrain-implementation-plan.md) |
| IP-LV-5 | code done; rebuild + footprint-review pending | **Uniform small tile footprint, region-wide.** Implemented in [`build_terrain.py`](../../python/tools/terrain/build_terrain.py): a single uniform cell grid (`_cell_side_deg` / `_compute_cell_grid`) shared by all LODs; per-LOD vertex density via `_effective_spacing_deg`, which **clamps** a coarse LOD's native spacing so a small cell still yields ≥ `_MIN_CELL_POINTS` points (a small tile becomes a coarse mesh, validated on real DEM: L0 2009 verts → L3-L6 9 verts, no degeneration); export is every tile for every LOD region-wide (the old `_select_display_tiles` export-radius selection and `_LOD_CELL_SIDE_DEG` / `_LOD_EXPORT_RADIUS_M` are removed). Footprint drives both tiling and chunking via `tile_footprint_m`. **Finding (needs review before the rebuild):** at 400 m the `small_uas_ksba_flight` region = **3,660 cells → 25,620 tiles**, of which L3-L6 (**14,640 tiles**) are geometrically identical (clamped) and differ only by texture; a 400 m footprint is far finer than the coarse LODs' culling bands require. This is a lot of resident coarse geometry (draw-call risk → IP-LV-6) and mosaic renders per build. This footprint strategy is now the open question **OQ-LS-22** (recommend Alternative 3, per-LOD footprint scaled to each LOD's band), which **gates the re-tile run**. **The actual re-tile of `small_uas_ksba_flight` is a separate run.** | IP-LV-4, IP-LV-8, OQ-LS-22 | [live_sim_view.md OQ-LS-18, OQ-LS-20, OQ-LS-22](../design/live_sim_view.md) |
| IP-LV-6 | ready | **Validate culling + profile runtime.** Measured **distinct opaque LODs vs. observer position** = 1 everywhere in the region (only the intended hysteresis-band overlap at LOD transitions); in `run_sim.sh` a single terrain surface renders across the region and the aircraft no longer passes through stacked layers. Profile in the Godot editor (draw calls + frame time + VRAM) to confirm the resident tile count is CPU/VRAM-comfortable; add a texture atlas / `MultiMesh` (OQ-LS-20 Alternative 4) only if profiling shows a draw-call bottleneck. The tunable parameters (`tile_footprint_m`, `chunk_footprints`, `R_fetch`/`R_unload`) are first chosen by testing here; **if empirical tuning proves insufficient**, a follow-on analysis step may derive them from fundamentals — output resolution, camera field of view, per-LOD max texture resolution, and any other parameter governing the rendered resolution of the projected imagery — rather than by hand (candidate for a small analysis tool / roadmap item). | IP-LV-5, IP-LV-7 | [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-7 | **done** | **Tile-residency / streaming manager.** Implemented in [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd): `_update_terrain_streaming` (per-`_process`, re-evaluated only when the aircraft crosses into a new chunk) pages fine-LOD chunks (`0..stream_lod_max`) by proximity with **asymmetric hysteresis** (OQ-LS-21 Alternative 1) — load within `R_fetch(lod) = visibility_end(lod) + fetch_margin_m`, free only beyond `R_unload(lod) = visibility_end(lod) + unload_margin_m` (`unload_margin_m > fetch_margin_m`). Coarse LODs (`> stream_lod_max`) load once and stay resident as the backdrop. Tunable `@export`s: `enable_streaming`, `stream_lod_max` (default 2), `fetch_margin_m` (800), `unload_margin_m` (2400). GDScript compiles clean under Godot 4.6.2; in-engine validation is IP-LV-6. | IP-LV-8 | [live_sim_view.md OQ-LS-21](../design/live_sim_view.md) |
| IP-LV-8 | **done** | **Streamable asset format.** Replaced the monolithic GLB with **per-(LOD, chunk) GLB files** addressed by **integer chunk coordinate** `(lod, cx, cy)` (OQ-LS-20 Alternative 2) plus a top-level `descriptor.json` (world origin, `tile_footprint_m`, `chunk_footprints`, `chunk_size_m`, LOD count, coverage bounds, sparse chunk list) — **no flat all-tiles manifest**, so the format scales to several-hundred-km regions. New [`terrain_chunks.py`](../../python/tools/terrain/terrain_chunks.py) (`chunk_index` / `tile_chunk_coord` / `assign_tiles_to_chunks` / `TerrainChunkDescriptor` / `export_chunked_terrain`, tested in `test_terrain_chunks.py`); [`build_terrain.py`](../../python/tools/terrain/build_terrain.py) emits chunks with configurable `tile_footprint_m` / `chunk_footprints` (config `terrain.*`, `--tile-footprint-m` / `--chunk-footprints`, or args; defaults 400 m / 4) and `terrain_config.json` now carries `terrain_descriptor_path`; [`TerrainLoader.gd`](../../godot/addons/liteaero_sim/TerrainLoader.gd) reads the descriptor and exposes `load_chunk` / `free_chunk` / `chunk_coord_for_position` (loads all chunks up front for now; IP-LV-7 adds proximity streaming). GDScript compiles clean under Godot 4.6.2. | — | [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |

**Execution order** (the numeric IDs predate the dependency analysis and no longer match it): the frame
work (IP-LV-1/2/3), the triangulator fix (IP-LV-4), and the streamable chunk format (IP-LV-8) are
**done**; the remaining chain is **IP-LV-7 → IP-LV-5 → IP-LV-6**. Both streaming design questions were
resolved (OQ-LS-20 → per-region coordinate-addressable chunk files; OQ-LS-21 → radius-based paging with
asymmetric hysteresis). IP-LV-7 (streaming manager) drives `TerrainLoader.gd`'s existing
`load_chunk` / `free_chunk` / `chunk_coord_for_position` API; IP-LV-5 (uniform re-tile) already emits the
chunk format via `build_terrain`, so it can run once its footprint is set.

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
