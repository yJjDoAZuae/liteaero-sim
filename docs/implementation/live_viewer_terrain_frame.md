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
   two things: **fixing the pathologically slow triangulator** (so the build is minutes, not hours) and
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
| IP-LV-4 | ready | **Fix the terrain triangulator speed (prerequisite for everything below; `WORK-TB-1`).** The per-cell cost is ~3 s even for tiny cells (1960 L2 cells → 1h42), so it is fixed overhead, not Delaunay: profile and strip per-cell overhead (raster re-windowing per cell, the mesh-quality verify pass) and parallelize the per-cell loop (`ProcessPoolExecutor`). Target: a few thousand small cells build in minutes. Without this, generating the uniform-footprint tiling is a multi-hour build per iteration. | — | [terrain-implementation-plan.md WORK-TB-1](../roadmap/terrain-implementation-plan.md) |
| IP-LV-5 | ready | **Uniform small tile footprint, region-wide.** Set [`_LOD_CELL_SIDE_DEG`](../../python/tools/terrain/build_terrain.py) to a single small footprint (start ≈ 300–500 m) for **all** LODs, and export every LOD across the whole parameterized terrain region (so each LOD covers the region, LOD = vertex density within the fixed footprint). Confine small-tile generation to the specified region (do not re-tile past the dataset boundary). Re-tile `small_uas_ksba_flight`. | IP-LV-4 | [live_sim_view.md OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-6 | ready | **Validate culling + profile runtime.** Measured **distinct opaque LODs vs. observer position** = 1 everywhere in the region (only the intended hysteresis-band overlap at LOD transitions); in `run_sim.sh` a single terrain surface renders across the region and the aircraft no longer passes through stacked layers. Profile in the Godot editor (draw calls + frame time) to confirm the tile count is CPU-comfortable; add a texture atlas / `MultiMesh` only if profiling shows a draw-call bottleneck. | IP-LV-5, IP-LV-7 | [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-7 | design needed | **Tile-residency / streaming manager.** Godot `visibility_range` culls *rendering* only — every loaded tile's mesh + texture stays resident, and thousands of high-detail L0 textures exceed the ~4 GB GPU budget. Add a manager that pages fine-LOD tiles in/out by aircraft proximity (load within a radius, unload beyond, with hysteresis) while keeping the coarse LODs permanently resident as the far backdrop. The approach (radius, hysteresis band, threading, async load) warrants its own design/OQ before implementation. | IP-LV-8 | (new OQ — TBD) |
| IP-LV-8 | design needed | **Streamable asset format.** The current single monolithic GLB (loaded whole at startup) cannot be paged. Split the export into individually loadable units (per-tile or per-region files) plus a spatial index the streaming manager (IP-LV-7) queries. Requires changes to `export_gltf.py` / `build_terrain.py` and `TerrainLoader.gd`. | — | (new OQ — TBD) |

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
- **Gap (work item): no export-only rebuild path.** Re-exporting the GLB requires either the full
  `build_terrain --force` (which deletes the cached `.las_terrain` and re-runs the ~43 min
  single-threaded LOD-0 triangulation + ~7 min texture mosaic — the dominant cost, unrelated to the
  export step) or bespoke tooling. A `build_terrain --export-only` mode that reloads the cached
  `.las_terrain` tiles and re-renders mosaics from the cached source rasters, then calls the existing
  `export_gltf`, would make GLB-format changes (like this one) cheap. Surfaced here per the cleanup
  guideline; not yet implemented. Related: `WORK-TB-1` (parallelize triangulation).
- **C++ `TerrainMesh::exportGltf` mirrors the Python path** (same translation-only placement). It is not on
  the live-viewer path (the GLB the viewer loads is produced by `export_gltf.py`), but if it is used for
  any export it needs the same per-tile rotation for consistency — surface as a follow-up if confirmed in
  use.
- **Precision:** the per-tile rotation preserves the centroid-relative float32 vertex design
  ([terrain.md §Local Tile Frame](../design/terrain.md)); only one quaternion per node is added.
- **No aircraft-side change:** `GodotEnuProjector` and the curvature-aware `SimulationReceiver` are
  unchanged; the correction is entirely in terrain export.
