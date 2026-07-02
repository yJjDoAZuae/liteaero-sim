# Live-Viewer Terrain Rendering Corrections — Implementation Plan

**Scope:** Two related live-viewer terrain-rendering defects, both surfacing as "stacked terrain
surfaces the aircraft drops through":

1. **Frame reconciliation (done).** Reconcile the terrain *surface* with the single world-origin ENU
   frame the aircraft is projected into. Resolves [live_sim_view.md Issue 8](../design/live_sim_view.md)
   via [OQ-LS-19](../design/live_sim_view.md) (Option 1 — per-tile node rotation). Issue 7 re-marked
   incomplete (aircraft-only). — **IP-LV-1..3.**
2. **LOD stacking (pending).** Make the per-node distance culling select one LOD per location by
   sizing display tiles smaller than their slant-range bands. Resolves
   [live_sim_view.md Issue 9](../design/live_sim_view.md) via
   [OQ-LS-18](../design/live_sim_view.md) (Alternative 1 — shrink the coarse-LOD cells). — **IP-LV-4..6.**

**Design authority:** [live_sim_view.md Issue 8 / OQ-LS-19](../design/live_sim_view.md),
[live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md),
[terrain.md §Local Tile Frame / §Slant-Range Selection](../design/terrain.md).

**Background (1 — frame):** tiles are placed at their centroid's world-origin ECEF→ENU offset (correct)
but their vertices live in each tile's **own centroid-tangent ENU frame** and the glTF nodes carry
**translation only**. The two diverge by the per-tile/per-LOD tangent-plane tilt (Earth curvature). The
fix adds a per-tile node rotation so translation+rotation places each tile exactly in the world-origin
frame.

**Background (2 — LOD stacking):** [`_LOD_CELL_SIDE_DEG`](../../python/tools/terrain/build_terrain.py)
sets each cell side to `100 × grid_spacing(lod)`, so every display tile is ~3× wider than its LOD band.
Godot culls tiles by camera-to-**centroid** distance, which only selects one LOD per location when a
tile ≈ one slant range. With oversized tiles the coarse LODs collapse to single origin-centered tiles
that cannot be culled under the aircraft, so up to 5 LODs render at once near the dataset center. The
fix shrinks the coarse-LOD cells (leaving L0/L1 — the triangulation bottleneck — unchanged) so
centroid-distance culling becomes accurate.

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LV-1 | **done** | In `export_gltf.py`, emit a per-tile node **rotation** $R = R_\text{origin}^{\top} R_\text{centroid}$ (the relative orientation of the tile's centroid-ENU basis vs the world-origin-ENU basis, in the glTF axis convention `(east, up, −north)`) alongside the existing centroid translation. Vertices stay centroid-relative (float32 precision preserved). Unit test the rotation math: a vertex's `(translation + R·v_centroid)` must equal its exact world-origin ECEF→ENU position within float tolerance. | — | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-2 | **done** | Re-exported the `small_uas_ksba_flight` GLB via `build_terrain.py` (all 13 display-tile nodes now carry rotations). Plugin `.dll` verified already current and curvature-aware (`SimulationReceiver` reads `viewer_x/y/z_m`), so no `./build.sh gdext` rebuild was needed. | IP-LV-1 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-3 | quantitative done; visual pending | **Quantitative PASS:** every re-exported node rotation matches an independent pyproj ECEF→geodetic recomputation of $R = P(R_\text{origin}^{\top}R_\text{centroid})P^{\top}$ to 4.4e-16 (machine precision). **Visual (user):** run `run_sim.sh` and confirm (a) a single terrain surface across all LOD transitions (no stacked layers) and (b) the aircraft sits on the surface at touchdown at both dataset center and edges. Note: full single-surface confirmation is blocked by the LOD-stacking defect (IP-LV-4..6). | IP-LV-2 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-4 | ready | **(Recommended prerequisite.)** Add an export-only / partial-rebuild path so re-exporting the GLB does not trigger the full ~50 min `--force` rebuild: reload the cached `.las_terrain` tiles, re-render only the affected mosaics from the cached source rasters, and call the existing `export_gltf`. Optionally re-triangulate only the LODs whose cell size changed. Without this, each cell-size iteration in IP-LV-5 costs a full rebuild. | — | [live_sim_view.md OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-5 | ready | Retune [`_LOD_CELL_SIDE_DEG`](../../python/tools/terrain/build_terrain.py) so each **coarse** LOD's cell side is a fraction of its slant-range band (start ≈ band width / N, N ≈ 2–4; leave L0/L1 unchanged). Re-triangulate the affected LODs and re-export the `small_uas_ksba_flight` GLB. The exact sizes are tuned empirically against the metric in IP-LV-6. | IP-LV-4 | [live_sim_view.md OQ-LS-18](../design/live_sim_view.md) |
| IP-LV-6 | ready | Validate: with the re-exported GLB, the measured count of **distinct opaque LODs vs. observer position** is 1 near the dataset center (and does not exceed the intended hysteresis overlap elsewhere); confirm in `run_sim.sh` that a single terrain surface renders at the airport and the aircraft no longer passes through stacked layers. Iterate IP-LV-5 cell sizes until met. | IP-LV-5 | [live_sim_view.md Issue 9 / OQ-LS-18](../design/live_sim_view.md) |

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
