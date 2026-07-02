# Live-Viewer Terrain Frame Correction — Implementation Plan

**Scope:** Reconcile the terrain *surface* with the single world-origin ENU frame the aircraft is
projected into, so the curvature-aware live viewer renders one consistent terrain surface and the
aircraft sits on it. Resolves [live_sim_view.md Issue 8](../design/live_sim_view.md) (Option 1 —
per-tile node rotation). Issue 7 is re-marked incomplete (aircraft-only).

**Design authority:** [live_sim_view.md Issue 8](../design/live_sim_view.md),
[terrain.md §Local Tile Frame](../design/terrain.md).

**Background:** tiles are placed at their centroid's world-origin ECEF→ENU offset (correct) but their
vertices live in each tile's **own centroid-tangent ENU frame** and the glTF nodes carry **translation
only**. The aircraft uses a single world-origin ENU frame. The two diverge by the per-tile/per-LOD
tangent-plane tilt (Earth curvature), producing stacked surfaces the aircraft drops through. The fix adds
a per-tile node rotation so translation+rotation places each tile exactly in the world-origin frame.

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LV-1 | ready | In `export_gltf.py`, emit a per-tile node **rotation** $R = R_\text{origin}^{\top} R_\text{centroid}$ (the relative orientation of the tile's centroid-ENU basis vs the world-origin-ENU basis, in the glTF axis convention `(east, up, −north)`) alongside the existing centroid translation. Vertices stay centroid-relative (float32 precision preserved). Unit test the rotation math: a vertex's `(translation + R·v_centroid)` must equal its exact world-origin ECEF→ENU position within float tolerance. | — | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-2 | ready | Re-export the GLB for the test dataset(s) via `build_terrain.py`; rebuild the plugin (`./build.sh gdext`). The committed curvature-aware `SimulationReceiver`/`TerrainLoader.gd` baseline is already restored (flat-Earth A/B revert removed). | IP-LV-1 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |
| IP-LV-3 | ready | Validate in the viewer: (a) a single terrain surface renders across all LOD transitions (no stacked layers); (b) the aircraft sits on the surface at touchdown at both the dataset center and its edges. Quantitative: `live_sim --verbose` `viewer_y` equals the GLB-projected surface height at sampled lat/lon within float tolerance. | IP-LV-2 | [live_sim_view.md Issue 8](../design/live_sim_view.md) |

---

## Notes

- **C++ `TerrainMesh::exportGltf` mirrors the Python path** (same translation-only placement). It is not on
  the live-viewer path (the GLB the viewer loads is produced by `export_gltf.py`), but if it is used for
  any export it needs the same per-tile rotation for consistency — surface as a follow-up if confirmed in
  use.
- **Precision:** the per-tile rotation preserves the centroid-relative float32 vertex design
  ([terrain.md §Local Tile Frame](../design/terrain.md)); only one quaternion per node is added.
- **No aircraft-side change:** `GodotEnuProjector` and the curvature-aware `SimulationReceiver` are
  unchanged; the correction is entirely in terrain export.
