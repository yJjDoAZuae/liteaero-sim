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

**Last updated:** 2026-07-06 — IP-TPI-1/2/3 done; only IP-TPI-4 (regenerate + validate) remains.

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-TPI-1 | done | Derive per-LOD grid side from tile bounds in `TerrainMesh`: `cellKey()`/`cellExtentRad()` use a per-instance per-LOD footprint computed from tile `GeodeticAABB` extents (max over each LOD's tiles, robust to clipped edge tiles); remove the static `kCellSideM`. Works for every construction path (`addCell`, all `deserialize*`). Add a dense-L0-region no-tile-loss test and an `elevation_m`-matches-vertices test | — | [terrain.md §Internal Spatial Index](../design/terrain.md), [terrain.md OQ-T-1](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-2 | done | Add the per-LOD footprint array to the `.las_terrain` header in `las_terrain.py` (`write_las_terrain` optional `lod_footprints_m`; `read_las_terrain_footprints()` returns it) and wire `build_terrain.py` Step 4 to pass `lod_policy.lod_footprints_m()`; round-trip test in `test_las_terrain.py` | — | [terrain.md §.las_terrain File Format](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-3 | done | Store + validate the footprint record in C++: read the array from the `.las_terrain` header and new JSON/proto (`TerrainMeshProto.lod_footprints_m`) fields; compare against the IP-TPI-1 derived footprints (25% tolerance, m↔rad) and raise on mismatch; write it in `serializeLasTerrain`/`serializeJson`/`serializeProto`; plus a bounds guard so a stale/old-header file throws instead of crashing. Cross-lang both directions + mismatch-fails + truncated-throws tests | IP-TPI-1, IP-TPI-2 | [terrain.md §Serialization](../design/terrain.md), [terrain.md §.las_terrain File Format](../design/terrain.md), [terrain.md OQ-T-3](../design/terrain.md) |
| IP-TPI-4 | done | Validated physics/visual agreement. Regenerated `small_uas_ksba_flight` loads cleanly in `live_sim` (8543 tiles, footprint record validates across all 7 LODs). **User-confirmed in-engine:** body-collider runway contact now matches the visual runway and AGL ≈ 0 at KSBA ground level. | IP-TPI-1, IP-TPI-3 | [terrain.md §Internal Spatial Index](../design/terrain.md) |

---

## Notes

**The fix and the record are separable.** IP-TPI-1 alone closes the ground-height defect — it
derives the grid side from tile bounds already present in the file, so it needs **no dataset
regeneration** and fixes every construction path (including `addCell`, which `TerrainMesh_test.cpp`
uses without any serialized footprint). IP-TPI-2/IP-TPI-3 add the OQ-T-3 Alternative 4
explicit-record-plus-validation layer on top; IP-TPI-1 and IP-TPI-2 are independent and may be done
in either order.

**Validation semantics (IP-TPI-3).** Per OQ-T-3 Alt 4 the derived-from-geometry footprint is the
operative value; the stored array is a cross-check. Where no array is stored, the derived value is
used, so no setter is mandatory.

The record holds the build's **nominal** per-LOD footprint (from `lod_policy`), but the build rounds
the region into an integer number of cells (`region / ceil(region / footprint)`) and clips edge
tiles, so **realized tiles are always ≤ the nominal** — for a small region a coarse LOD's tiles are
much smaller than its nominal footprint (found during IP-TPI-4: KSBA L4 nominal 19092 m vs realized
~12009 m). The cross-check is therefore **directional**: it flags only the impossible/corrupt
direction — realized tiles materially *larger* than the record — not the expected nominal ≥ realized
gap. This was a symmetric-vs-directional correction made during IP-TPI-4; the design note in
[terrain.md OQ-T-3 resolution](../design/terrain.md) and the `.las_terrain` format spec record it.

**Serialization sequencing.** IP-TPI-3 follows IP-TPI-2 (Python header field) and IP-TPI-1
(in-memory derived footprints to validate against), per the standing rule that format/field
additions are sequenced around the code that consumes them.

**No imagery re-tile.** Nothing here touches the texture render. IP-TPI-4's regeneration is the fast
pre-render `.las_terrain` write, and is needed only to populate the new stored record — not for the
ground-height correctness, which lands with IP-TPI-1.

**Mandatory regeneration.** The `.las_terrain` header grew from 12 to 40 bytes (format_version stays
1 per the initial-development policy). Any dataset built before IP-TPI-2 has the old header and is now
**rejected on load** with a clear "regenerate with build_terrain.py" error (the IP-TPI-3 bounds guard
turns what would have been a read-past-end crash into that exception). So `small_uas_ksba_flight` — and
the `godot/terrain_test` asset via `gen_test_assets.py` — must be rebuilt before use.

**Out of scope (OQ-T-2 follow-up).** Deleting the superseded slant-range selection prose and the
`selectLodBySlantRange` / `LodSelector` code (and `LodSelector_test.cpp`) is a separate cleanup
tracked against the OQ-T-2 resolution, not part of this plan.
