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
| IP-LV-5 | blocked on OQ-LS-20 | **Uniform small tile footprint, region-wide.** Set [`_LOD_CELL_SIDE_DEG`](../../python/tools/terrain/build_terrain.py) to a single small footprint (start ≈ 300–500 m) for **all** LODs, and export every LOD across the whole parameterized terrain region (so each LOD covers the region, LOD = vertex density within the fixed footprint). Confine small-tile generation to the specified region (do not re-tile past the dataset boundary). Re-tile `small_uas_ksba_flight`. **Ordering:** the re-tile must emit the streamable format (IP-LV-8) — a monolithic GLB would have to be regenerated — so IP-LV-8 / OQ-LS-20 precede this. | IP-LV-4, IP-LV-8 | [live_sim_view.md OQ-LS-18, OQ-LS-20](../design/live_sim_view.md) |
| IP-LV-6 | ready | **Validate culling + profile runtime.** Measured **distinct opaque LODs vs. observer position** = 1 everywhere in the region (only the intended hysteresis-band overlap at LOD transitions); in `run_sim.sh` a single terrain surface renders across the region and the aircraft no longer passes through stacked layers. Profile in the Godot editor (draw calls + frame time + VRAM) to confirm the resident tile count is CPU/VRAM-comfortable; add a texture atlas / `MultiMesh` (OQ-LS-20 Alternative 4) only if profiling shows a draw-call bottleneck. | IP-LV-5, IP-LV-7 | [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-7 | blocked on OQ-LS-21 | **Tile-residency / streaming manager.** Godot `visibility_range` culls *rendering* only — every loaded tile's mesh + texture stays resident, and thousands of high-detail L0 textures exceed the ~4 GB GPU budget (≈ 3,400 L0 tiles alone at 512² RGBA8 ≈ 3.4 GB). Add a manager that pages fine-LOD tiles in/out by aircraft proximity (load within a radius, unload beyond, with hysteresis) while keeping the coarse LODs permanently resident as the far backdrop. Approach (radius, hysteresis, threading, async load) is **OQ-LS-21** (recommend Alternative 1, radius-based paging). | IP-LV-8 | [live_sim_view.md OQ-LS-21](../design/live_sim_view.md) |
| IP-LV-8 | blocked on OQ-LS-20 | **Streamable asset format.** The current single monolithic GLB (loaded whole at startup via `ResourceLoader.load()` + `instantiate()`) cannot be paged. Split the export into individually loadable units plus a spatial index the streaming manager (IP-LV-7) queries. Format is **OQ-LS-20** (recommend Alternative 2, per-region chunk files + index). Requires changes to `export_gltf.py` / `build_terrain.py` and `TerrainLoader.gd`. | — | [live_sim_view.md OQ-LS-20](../design/live_sim_view.md) |

**Execution order** (the numeric IDs predate the dependency analysis and no longer match it): the frame
work (IP-LV-1/2/3) and the triangulator fix (IP-LV-4) are done; the remaining chain is
**IP-LV-8 → IP-LV-7 → IP-LV-5 → IP-LV-6**. The two streaming design questions (OQ-LS-20 for IP-LV-8,
OQ-LS-21 for IP-LV-7) must be resolved by the user before their implementation begins, and IP-LV-8
(asset format) precedes IP-LV-5 (re-tile) so the re-tile emits the streamable format directly.

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
