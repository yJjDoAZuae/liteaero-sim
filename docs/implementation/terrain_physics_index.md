# Terrain Physics Index — Per-LOD Footprint Consistency — Implementation Plan

**Scope:** Fix the physics `TerrainMesh` spatial index so it stays consistent with the build's
per-LOD footprint tiling, closing the ground-height defect where `elevation_m()` (and therefore
AGL, landing-gear contact, and the body collider) returns a coarse-tile surface that floats above
the drawn terrain (~15 m at the KSBA runway). Derives from [terrain.md](../design/terrain.md):
**OQ-T-1** (data-driven index) and **OQ-T-3 → Alternative 4** (store the per-LOD footprints in
every serialized format *and* derive the operative grid side from the tile bounds, validating the
stored value against the derivation on load). All blocking open questions are resolved. Splits
naturally into the **operative fix** (derive the grid side from tile bounds — needs no dataset
regeneration, works for every construction path) and the **validated-record layer** (store the
array in each format and fail loudly if it disagrees with the geometry). Does **not** cover
deleting the superseded slant-range / `LodSelector` code (OQ-T-2 follow-up — separate plan).

**Design authority:** [terrain.md](../design/terrain.md) (§Internal Spatial Index, §`.las_terrain`
File Format, §Serialization, OQ-T-1, OQ-T-3).

**Last updated:** 2026-07-05

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-TPI-1 | todo | Derive per-LOD grid side from tile bounds in `TerrainMesh`: `cellKey()`/`cellExtentRad()` use a per-instance per-LOD footprint computed from tile `GeodeticAABB` extents (max over each LOD's tiles, robust to clipped edge tiles); remove the static `kCellSideM`. Works for every construction path (`addCell`, all `deserialize*`). Add a dense-L0-region no-tile-loss test and an `elevation_m`-matches-vertices test | — | [terrain.md §Internal Spatial Index](../design/terrain.md), [terrain.md OQ-T-1](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-2 | todo | Add the per-LOD footprint array to the `.las_terrain` header in `las_terrain.py` (`write_las_terrain` writes it, `read_las_terrain` returns it) and wire `build_terrain.py` Step 4 to pass `lod_policy.lod_footprints_m()`; round-trip test in `test_las_terrain.py` | — | [terrain.md §.las_terrain File Format](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-3 | todo | Store + validate the footprint record in C++: read the array from the `.las_terrain` header and from new JSON/proto fields; compare against the IP-TPI-1 derived footprints within tolerance and raise on mismatch; write the array in `serializeLasTerrain`/`serializeJson`/`serializeProto`; optional `setLodFootprints()`. Extend `LasTerrain_CrossLang_test.cpp` and add a mismatch-fails test | IP-TPI-1, IP-TPI-2 | [terrain.md §Serialization](../design/terrain.md), [terrain.md §.las_terrain File Format](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-4 | todo | Validate physics/visual agreement: assert `elevation_m` at the KSBA runway matches the chunk-descriptor surface and AGL is sane on the ground (runs on the existing dataset after IP-TPI-1); regenerate `small_uas_ksba_flight` to populate the new header record | IP-TPI-1, IP-TPI-3 | [terrain.md §Internal Spatial Index](../design/terrain.md) |

---

## Notes

**The fix and the record are separable.** IP-TPI-1 alone closes the ground-height defect — it
derives the grid side from tile bounds already present in the file, so it needs **no dataset
regeneration** and fixes every construction path (including `addCell`, which `TerrainMesh_test.cpp`
uses without any serialized footprint). IP-TPI-2/IP-TPI-3 add the OQ-T-3 Alternative 4
explicit-record-plus-validation layer on top; IP-TPI-1 and IP-TPI-2 are independent and may be done
in either order.

**Validation semantics (IP-TPI-3).** Per OQ-T-3 Alt 4 the derived-from-geometry footprint is the
operative value; the stored array is a cross-check. A stored value disagreeing with the geometry is
a hard load-time error, which also catches geometry that does not tile on the expected grid. Where
no array is stored, the derived value is used, so no setter is mandatory.

**Serialization sequencing.** IP-TPI-3 follows IP-TPI-2 (Python header field) and IP-TPI-1
(in-memory derived footprints to validate against), per the standing rule that format/field
additions are sequenced around the code that consumes them.

**No imagery re-tile.** Nothing here touches the texture render. IP-TPI-4's regeneration is the fast
pre-render `.las_terrain` write, and is needed only to populate the new stored record — not for the
ground-height correctness, which lands with IP-TPI-1.

**Out of scope (OQ-T-2 follow-up).** Deleting the superseded slant-range selection prose and the
`selectLodBySlantRange` / `LodSelector` code (and `LodSelector_test.cpp`) is a separate cleanup
tracked against the OQ-T-2 resolution, not part of this plan.
