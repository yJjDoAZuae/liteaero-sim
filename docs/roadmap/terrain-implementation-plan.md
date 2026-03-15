# Plan: Terrain Model — Roadmap Item 1 (1a and 1b)

## Context

Implement the full terrain subsystem per the design authority
`docs/architecture/terrain.md`. These are Domain Layer components; no I/O, all SI.
The roadmap splits the work as:

- Item 1a: `V_Terrain` + `FlatTerrain` (prerequisite for sensors and guidance)
- Item 1b: `TerrainMesh`, `TerrainTile`, `TerrainCell`, `LodSelector`,
  `MeshQualityVerifier`, serialization, glTF export, and `SimulationFrame`/`TrajectoryFile`

After plan approval, implementation stops here — sensor and guidance integration are
separate future steps.

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| No CMakeLists changes | `src/CMakeLists.txt` uses `GLOB_RECURSE`; `test/CMakeLists.txt` uses `GLOB` — new files auto-discovered |
| `TerrainVertex` as float32 ENU offsets | Per design authority §Local Grid Vertex Encoding — float32 at 50 km gives ~3 mm precision |
| `GeodeticPoint` as double lat/lon | Centroid stores double to avoid geodetic precision loss |
| `queryLocalAABB` is primary simulation interface | Metric extents free of geodetic singularities; natural for sensor/guidance use |
| `GeodeticAABB` retained for ingestion only | Bounds storage and data-management queries |
| `LodSelector` owns hysteresis state | `TerrainMesh` remains const-queryable; state belongs to the observer, not the mesh |
| Vertex duplication for glTF per-facet color | glTF `COLOR_0` is per-vertex; 3 unique vertices per triangle required |
| `tinygltf` (header-only, MIT) for glTF export | FetchContent dependency; avoids manual GLB serialization |
| `TerrainTile` is immutable after construction | Enforces const-correctness throughout the query pipeline |
| `SimulationFrame` is a value object (no I/O) | Transport (UDP, shared memory) is Interface Layer — not in Domain Layer |
| `.las_terrain` binary: 12 bytes/vertex | float32 east/north/up (was 20 bytes with double lat/lon) |
| No C++ `TerrainSimplifier` | Simplification is Python-only (pyfqmr + trimesh); Python is the authoritative implementation because it lives in the user's data-preparation workflow |

---

## Files to Create / Modify

| File | Action |
|------|--------|
| `include/environment/GeodeticPoint.hpp` | ✅ Done (Step 1) |
| `include/environment/TerrainVertex.hpp` | ✅ Done (Step 1) |
| `include/environment/TerrainFacet.hpp` | ✅ Done (Step 1) |
| `include/environment/GeodeticAABB.hpp` | ✅ Done (Step 1) |
| `include/environment/LocalAABB.hpp` | ✅ Done (Step 1) |
| `include/environment/Terrain.hpp` | ✅ Done (Step 2) — `V_Terrain` + `FlatTerrain` |
| `include/environment/TerrainTile.hpp` | ✅ Done (Step 3) — includes `TerrainLod` enum class |
| `include/environment/TerrainCell.hpp` | ✅ Done (Step 3) |
| `include/environment/TerrainMesh.hpp` | ✅ Done (Steps 4–6) |
| `include/environment/LodSelector.hpp` | ✅ Done (Step 7) |
| `include/environment/MeshQualityVerifier.hpp` | ✅ Done (Step 9) |
| `include/SimulationFrame.hpp` | ✅ Done (Step 12) |
| `src/environment/Terrain.cpp` | ✅ Done (Step 2) |
| `src/environment/TerrainTile.cpp` | ✅ Done (Step 3, 10) |
| `src/environment/TerrainCell.cpp` | ✅ Done (Step 3) |
| `src/environment/TerrainMesh.cpp` | ✅ Done (Steps 4–11) |
| `src/environment/LodSelector.cpp` | ✅ Done (Step 7) |
| `src/environment/MeshQualityVerifier.cpp` | ✅ Done (Step 9) |
| `src/environment/tinygltf_impl.cpp` | ✅ Done (Step 11) — TINYGLTF_IMPLEMENTATION TU |
| `test/Terrain_test.cpp` | ✅ Done (Step 2) — 4 tests |
| `test/TerrainTile_test.cpp` | ✅ Done (Step 3) — 8 tests |
| `test/TerrainMesh_test.cpp` | ✅ Done (Steps 4–11, 30 tests) |
| `test/LodSelector_test.cpp` | ✅ Done (Step 7) — 5 tests |
| `test/MeshQualityVerifier_test.cpp` | ✅ Done (Step 9) — 4 tests |
| `test/TrajectoryFile_test.cpp` | ✅ Done (Step 12) — 2 tests |
| `proto/liteaerosim.proto` | ✅ Done (Steps 10, 12) — terrain + trajectory messages added |
| `CMakeLists.txt` | ✅ Done (Step 11) — `tinygltf` v2.9.3 FetchContent block |
| `src/CMakeLists.txt` | ✅ Done (Step 11) — `tinygltf_headers` linked to `liteaerosim` |
| `python/tools/terrain/__init__.py` | ✅ Done (Step 14) |
| `python/tools/terrain/las_terrain.py` | ✅ Done (Step 14) — pure-Python `.las_terrain` reader/writer |
| `python/tools/terrain/download.py` | ✅ Done (Step 15) |
| `python/tools/terrain/mosaic.py` | ✅ Done (Step 15) |
| `python/tools/terrain/geoid_correct.py` | ✅ Done (Step 15) |
| `python/tools/terrain/triangulate.py` | ✅ Done (Step 16) |
| `python/tools/terrain/colorize.py` | ✅ Done (Step 17) |
| `python/tools/terrain/simplify.py` | ✅ Done (Step 18) |
| `python/tools/terrain/verify.py` | ✅ Done (Step 19) |
| `python/tools/terrain/export.py` | ✅ Done (Step 20) — thin wrapper |
| `python/tools/terrain/export_gltf.py` | ✅ Done (Step 20) |
| `python/test/test_las_terrain.py` | ✅ Done (Step 14) — 4 tests |
| `python/test/test_geoid_correct.py` | ✅ Done (Step 15) — 3 tests (1 skipped without PROJ datum grid) |
| `python/test/test_triangulate.py` | ✅ Done (Step 16) — 4 tests |
| `python/test/test_colorize.py` | ✅ Done (Step 17) — 3 tests |
| `python/test/test_simplify.py` | ✅ Done (Step 18) — 4 tests |
| `python/test/test_verify.py` | ✅ Done (Step 19) — 4 tests |
| `python/test/test_export_gltf.py` | ✅ Done (Step 20) — 3 tests |
| `python/test/test_pipeline.py` | ✅ Done (Step 21) — 1 integration test |
| `python/pyproject.toml` | ✅ Done (Step 14) — terrain dependency group added |

`test/CMakeLists.txt` needs **no changes**.

---

## Step 1 — Plain Data-Model Headers (no tests — pure structs) ✅

No tests required. These are pure aggregate types with no behavior.

### `include/environment/GeodeticPoint.hpp`

```cpp
namespace liteaerosim::environment {
struct GeodeticPoint {
    double latitude_rad;
    double longitude_rad;
    float  height_wgs84_m;
};
} // namespace liteaerosim::environment
```

### `include/environment/TerrainVertex.hpp`

ENU offset from the containing tile's centroid; stored as float32.

```cpp
namespace liteaerosim::environment {
struct TerrainVertex {
    float east_m;
    float north_m;
    float up_m;
};
} // namespace liteaerosim::environment
```

### `include/environment/TerrainFacet.hpp`

```cpp
namespace liteaerosim::environment {
struct FacetColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
struct TerrainFacet {
    uint32_t  vertex_indices[3];
    FacetColor color;
};
} // namespace liteaerosim::environment
```

### `include/environment/GeodeticAABB.hpp`

Used for bounds storage and data-management queries only.

```cpp
namespace liteaerosim::environment {
struct GeodeticAABB {
    double lat_min_rad;
    double lat_max_rad;
    double lon_min_rad;
    double lon_max_rad;
    float  height_min_m;
    float  height_max_m;
};
} // namespace liteaerosim::environment
```

### `include/environment/LocalAABB.hpp`

Metric, vehicle-centered query box; free of geodetic singularities.

```cpp
namespace liteaerosim::environment {
struct LocalAABB {
    float half_extent_north_m;
    float half_extent_east_m;
    float height_min_m;
    float height_max_m;
};
} // namespace liteaerosim::environment
```

### `TerrainLod` enum class (in `include/environment/TerrainTile.hpp`)

```cpp
namespace liteaerosim::environment {
enum class TerrainLod : int {
    L0_Finest        = 0,  // ~10 m vertex spacing
    L1_VeryHighDetail = 1,
    L2_HighDetail    = 2,
    L3_MediumDetail  = 3,
    L4_LowDetail     = 4,
    L5_VeryLowDetail = 5,
    L6_Coarsest      = 6   // ~10 km vertex spacing
};
} // namespace liteaerosim::environment
```

---

## Step 2 — `V_Terrain` + `FlatTerrain` (roadmap 1a, 4 tests) ✅

### Failing Tests — `test/Terrain_test.cpp`

- **T1** `FlatTerrain_ElevationConstant` — `FlatTerrain(300.f).elevation_m(lat, lon)` == 300.f for any lat/lon
- **T2** `HeightAboveGround_AboveTerrain` — `FlatTerrain(300.f).heightAboveGround_m(500.f, ...)` == 200.f
- **T3** `HeightAboveGround_AtTerrain` — `FlatTerrain(300.f).heightAboveGround_m(300.f, ...)` == 0.f
- **T4** `HeightAboveGround_BelowTerrain_Clamped` — `FlatTerrain(300.f).heightAboveGround_m(100.f, ...)` == 0.f (no negative)

### Header — `include/environment/Terrain.hpp`

```cpp
namespace liteaerosim::environment {

class V_Terrain {
public:
    [[nodiscard]] virtual float elevation_m(double latitude_rad,
                                            double longitude_rad) const = 0;
    [[nodiscard]] float heightAboveGround_m(float  altitude_m,
                                            double latitude_rad,
                                            double longitude_rad) const;
    virtual ~V_Terrain() = default;
};

class FlatTerrain : public V_Terrain {
public:
    explicit FlatTerrain(float elevation_m = 0.f);
    [[nodiscard]] float elevation_m(double latitude_rad,
                                    double longitude_rad) const override;
private:
    float elevation_m_;
};

} // namespace liteaerosim::environment
```

### Implementation — `src/environment/Terrain.cpp`

- `heightAboveGround_m`: `return std::max(0.f, altitude_m - elevation_m(lat, lon))`
- `FlatTerrain::elevation_m`: returns stored `elevation_m_` unconditionally

---

## Step 3 — `TerrainTile` + `TerrainCell` (8 tests) ✅

### Failing Tests — `test/TerrainTile_test.cpp`

- **T1** `ConstructorAccessors` — `lod()`, `centroid()`, `bounds()`, `vertices().size()`, `facets().size()` match construction args
- **T2** `FacetCentroid_RightTriangle` — `facetCentroid(0)` for a right-triangle tile returns the ENU-averaged centroid converted back to geodetic
- **T3** `FacetNormal_HorizontalFace` — `facetNormal(0)` for a horizontal face at the equator has Z-component > 0.99 (points outward)
- **T4** `JsonRoundTrip_VerticesAndIndicesPreserved` — `TerrainTile` survives `serializeJson()`/`deserializeJson()` — all vertex ENU values and facet indices preserved (EXPECT_NEAR tol 1e-4f)
- **T5** `JsonSchemaVersionMismatch_Throws` — schema version mismatch in JSON throws `std::runtime_error`
- **T6** `TerrainCell_AddTileAndHasLod` — `TerrainCell::addTile()` + `hasLod()` + `tile(lod)` round-trip
- **T7** `TerrainCell_MissingLod_Throws` — `TerrainCell::tile(missing_lod)` throws `std::out_of_range`
- **T8** `TerrainCell_FinestCoarsestLod` — `finestAvailableLod()` / `coarsestAvailableLod()` correct when only L1 and L3 are populated

### Headers

**`include/environment/TerrainTile.hpp`** — `TerrainTile` is immutable after construction; exposes:

- `lod()`, `centroid()`, `bounds()`, `vertices()`, `facets()` (all const)
- `facetCentroid(size_t index) const` → `GeodeticPoint`
- `facetNormal(size_t index) const` → `std::array<double, 3>` (ECEF unit normal)
- `serializeJson() const` / `static deserializeJson(const nlohmann::json&)`
- `serializeProto() const` / `static deserializeProto(const std::vector<uint8_t>&)`

**`include/environment/TerrainCell.hpp`** — holds up to 7 `TerrainTile` objects (one per LOD); exposes:

- `addTile(TerrainTile tile)`
- `hasLod(TerrainLod lod) const`
- `tile(TerrainLod lod) const` — throws `std::out_of_range` if not present
- `finestAvailableLod() const` / `coarsestAvailableLod() const`
- `bounds() const` → `GeodeticAABB`

### Implementation Notes

- `facetNormal`: compute ENU cross-product of two edge vectors, convert to ECEF using the tile centroid rotation matrix; use double arithmetic internally to avoid float32 precision loss on small facets.
- `TerrainCell` storage: `std::array<std::optional<TerrainTile>, 7>` indexed by `static_cast<int>(lod)`.
- JSON schema version: `"schema_version": 1` in every serialized object; mismatch throws `std::runtime_error`.

---

## Step 4 — `TerrainMesh` Core: `addCell()`, `cellAt()`, `elevation_m()` (5 tests) ✅

### Failing Tests — `test/TerrainMesh_test.cpp`

- **T1** `AddCell_ThenCellAtCentroid_ReturnsNonNull` — `addCell()` then `cellAt(centroid_lat, centroid_lon)` returns non-null
- **T2** `CellAt_OffsetWithinHalfExtent_FindsCell` — `cellAt()` with a point offset from centroid by less than half_extent still finds the cell
- **T3** `CellAt_OutsideAllCells_ReturnsNullptr` — `cellAt()` outside all cell extents returns `nullptr`
- **T4** `ElevationM_AtKnownVertex` — `elevation_m()` at a known vertex position returns the stored height ± 0.01 m
- **T5** `ElevationM_OutsideAllCells_ReturnsZero` — `elevation_m()` outside all cells returns 0.f

### Header — `include/environment/TerrainMesh.hpp`

Concrete `V_Terrain`. Key interface (full interface in design authority):

- `addCell(TerrainCell cell)` — single-threaded load phase only (see §Design Constraints)
- `cellAt(double lat_rad, double lon_rad) const` → `const TerrainCell*`
- `elevation_m(double lat_rad, double lon_rad) const override`

### Implementation Notes for `elevation_m()`

1. Find cell via `cellAt()`; return 0.f if null.
2. Select finest available LOD tile.
3. Convert query lat/lon to ENU offset from tile centroid.
4. Iterate facets; find containing facet using point-in-triangle test (2D in ENU north/east plane).
5. Interpolate height via barycentric coordinates of the three vertex up-components.
6. O(N_facets) per query — acceptable for regional meshes (≤10,000 facets); see Risk Note 1.

---

## Step 5 — Coordinate Transforms: `toECEF()`, `toNED()` (4 tests) ✅

### Failing Tests (add to `test/TerrainMesh_test.cpp`)

- **T1** `ToECEF_AtEquatorPrimeMeridian` — vertex at ENU (0,0,0) with centroid at (0°N, 0°E, 0m): ECEF x ≈ 6,378,137 m ± 0.1 m
- **T2** `ToECEF_AtEquator90E` — vertex at ENU (0,0,0) with centroid at (0°N, 90°E, 0m): ECEF y ≈ 6,378,137 m ± 0.1 m
- **T3** `ToNED_AtCentroidPosition` — `toNED()` vertex at centroid position → NED (0, 0, 0) ± 0.01 m
- **T4** `ToNED_100mEast` — `toNED()` vertex 100 m east of reference → NED (0, 100, 0) ± 0.1 m

### Implementation Notes

ENU-to-ECEF rotation matrix (λ = longitude, φ = latitude):

```
R_enu = [[-sinλ,       cosλ,      0    ],
         [-sinφ·cosλ, -sinφ·sinλ,  cosφ ],
         [ cosφ·cosλ,  cosφ·sinλ,  sinφ ]]ᵀ
```

`P_ecef = P_centroid_ecef + R_enu * [east; north; up]`

NED from ECEF: apply `R_ned` at the reference point; `P_NED = R_ned * (P_ecef - P_ref_ecef)`.

---

## Step 6 — Subset Queries: `queryLocalAABB()`, `queryGeodeticAABB()`, `querySphere()` (6 tests) ✅

### Failing Tests (add to `test/TerrainMesh_test.cpp`)

- **T1** `QueryLocalAABB_CenteredOnTile_ReturnsCell` — 500 m half-extents centered on a tile centroid → returns that tile
- **T2** `QueryLocalAABB_FarFromAllTiles_ReturnsEmpty` — offset 10 km away from all tile centroids → returns empty
- **T3** `QueryGeodeticAABB_Overlapping_ReturnsTile` — overlapping tile bounds → returns tile
- **T4** `QueryGeodeticAABB_NonOverlapping_ReturnsEmpty` — non-overlapping → returns empty
- **T5** `QuerySphere_WithinRadius_ReturnsTile` — radius > distance-to-centroid → returns tile
- **T6** `Query_MaxLodFiltering` — `max_lod = L1_VeryHighDetail` when only L0 and L2 are present → returns L2 (coarsest ≤ L1 available)

### Method Signatures

```cpp
std::vector<TileRef> queryLocalAABB(double           center_lat_rad,
                                    double           center_lon_rad,
                                    float            center_height_m,
                                    const LocalAABB& aabb,
                                    TerrainLod       max_lod) const;

std::vector<TileRef> queryGeodeticAABB(const GeodeticAABB& bounds,
                                       TerrainLod          max_lod) const;

std::vector<TileRef> querySphere(double     center_lat_rad,
                                 double     center_lon_rad,
                                 float      center_height_m,
                                 float      radius_m,
                                 TerrainLod max_lod) const;
```

---

## Step 7 — LOD Selection + `LodSelector` (5 tests) ✅

### Failing Tests — `test/LodSelector_test.cpp`

- **T1** `SelectBySlantRange_NominalBoundaries` — at r < 300 m → L0; at r = 500 m (between 300 and 900) → L1
- **T2** `FirstCall_UseNominalFormula` — `LodSelector::select()` first call uses nominal formula (same as T1)
- **T3** `SecondCall_InsideDeadBand_NoTransition` — slant range moves from 270 m to 310 m (just above nominal boundary 300 m, below coarser threshold 300×1.15=345 m) → L0 unchanged (inside dead band)
- **T4** `SecondCall_OutsideDeadBand_Transitions` — slant range moves from 270 m to 360 m (above 345 m threshold) → transitions to L1
- **T5** `Reset_ClearsHysteresisState` — `LodSelector::reset()` followed by call at previous slant range → cold selection (no carry-over)

### Header — `include/environment/LodSelector.hpp`

`LodSelector` owns hysteresis state; `TerrainMesh` remains const-queryable.

```cpp
namespace liteaerosim::environment {

struct LodSelectorConfig {
    float hysteresis_fraction_nd = 0.15f;  // δ — dead-band half-width fraction
    float r0_m                   = 300.f;  // slant-range at the L0/L1 boundary
};

class LodSelector {
public:
    explicit LodSelector(LodSelectorConfig config = {});
    [[nodiscard]] std::vector<TileRef> select(const TerrainMesh& mesh,
                                              double observer_lat_rad,
                                              double observer_lon_rad,
                                              double observer_alt_m);
    void reset();
private:
    LodSelectorConfig                        config_;
    std::unordered_map<uint64_t, TerrainLod> committed_lod_;
};

} // namespace liteaerosim::environment
```

---

## Step 8 — Line-of-Sight Query (3 tests) ✅

### Failing Tests (add to `test/TerrainMesh_test.cpp`)

- **T1** `LineOfSight_AboveFlatTerrain_ReturnsTrue` — both points well above flat zero-elevation terrain → `lineOfSight()` returns true
- **T2** `LineOfSight_BelowTerrain_ReturnsFalse` — one point at h = -1 m (below terrain elevation 0 m) → returns false
- **T3** `LineOfSight_RidgeMesh_ReturnsFalse` — ridge mesh (3 triangles forming a peak): two points on opposite sides at horizon height → returns false

### Method Signature

```cpp
[[nodiscard]] bool lineOfSight(double lat1_rad, double lon1_rad, float h1_m,
                                double lat2_rad, double lon2_rad, float h2_m) const;
```

Implementation: ray–triangle intersection in ECEF space using the Möller–Trumbore algorithm.
Candidate tiles identified via `querySphere` centered at the segment midpoint with radius
equal to half the segment length plus maximum terrain height.

---

## Step 9 — `MeshQualityVerifier` (4 tests) ✅

### Failing Tests — `test/MeshQualityVerifier_test.cpp`

- **T1** `EquilateralMesh_Passes` — equilateral-triangle mesh (all angles 60°) → `passes()` true; `min_interior_angle_deg` ≈ 60 ± 0.1
- **T2** `DegenerateMesh_ZeroAreaFacet` — one zero-area triangle → `report.degenerate_facet_count == 1`; `passes()` false
- **T3** `SingleTriangle_BoundaryEdgeCount` — single-triangle mesh → `open_boundary_edge_count == 3`
- **T4** `ThinTriangle_HighAspectRatio` — very thin triangle → `max_aspect_ratio > 15`; `passes()` false

### Header — `include/environment/MeshQualityVerifier.hpp`

See full struct definition in design authority §Mesh Quality Verification. Minimal required fields:

```cpp
namespace liteaerosim::environment {

struct MeshQualityReport {
    float    min_interior_angle_deg   = 180.f;
    float    max_aspect_ratio         = 0.f;
    uint32_t degenerate_facet_count   = 0;
    uint32_t open_boundary_edge_count = 0;
    // ... additional fields per design authority
};

class MeshQualityVerifier {
public:
    [[nodiscard]] static MeshQualityReport verify(const TerrainTile& tile);
    [[nodiscard]] static bool              passes(const MeshQualityReport& report,
                                                  const MeshQualityThresholds& thresholds = {});
};

} // namespace liteaerosim::environment
```

---

## Step 10 — Serialization: JSON + Proto + `.las_terrain` (5 tests) ✅

### Failing Tests (add to existing test files)

- **T1** `TerrainMesh_JsonRoundTrip_TwoTiles` — `TerrainMesh` with 2 tiles (different LODs) survives `serializeJson()`/`deserializeJson()` — tile count and centroid preserved ± 1e-6
- **T2** `TerrainMesh_ProtoRoundTrip` — proto round-trip for `TerrainMesh`
- **T3** `TerrainTile_LasTerrain_BinaryRoundTrip` — `.las_terrain` binary: `TerrainTile` written and re-read has identical vertices (within float32 precision) and facet indices
- **T4** `TerrainMesh_JsonSchemaVersionMismatch_Throws` — JSON schema version mismatch → `std::runtime_error`
- **T5** `TerrainMesh_EmptySerializeDeserialize` — empty `TerrainMesh` round-trips without error; `cellAt()` returns `nullptr`

### Proto Messages — append to `proto/liteaerosim.proto`

```proto
message TerrainTileProto {
    int32  schema_version     = 1;
    int32  lod                = 2;
    double centroid_lat_rad   = 3;
    double centroid_lon_rad   = 4;
    float  centroid_height_m  = 5;
    double lat_min_rad        = 6;
    double lat_max_rad        = 7;
    double lon_min_rad        = 8;
    double lon_max_rad        = 9;
    float  height_min_m       = 10;
    float  height_max_m       = 11;
    repeated float  vertices  = 12;  // packed: east0,north0,up0, east1,...
    repeated uint32 indices   = 13;  // 3 per facet
    repeated uint32 colors    = 14;  // packed R8G8B8 per facet (upper byte unused)
}

message TerrainMeshState {
    int32  schema_version   = 1;
    string las_terrain_path = 2;
    int32  lod_min          = 3;
    int32  lod_max          = 4;
}
```

### `.las_terrain` Binary Format

12 bytes per vertex (float32 east/north/up); little-endian.
File header: magic `0x4C415354` ("LAST"), format version, tile count.
Full specification in design authority §`.las_terrain` File Format.

---

## Step 11 — `TerrainMesh::exportGltf()` (4 tests) ✅

### Failing Tests (add to `test/TerrainMesh_test.cpp`)

- **T1** `ExportGltf_GlbMagicBytes` — first 4 bytes are `0x46546C67` ("glTF" magic)
- **T2** `ExportGltf_LiteaerosimExtrasPresent` — GLB JSON chunk contains `"liteaerosim_terrain": true` in root node `extras`
- **T3** `ExportGltf_PositionAccessorCount` — `POSITION` accessor count == 3 × facet count (vertex duplication for per-facet color)
- **T4** `ExportGltf_ColorAccessorMatchesPosition` — `COLOR_0` accessor count equals `POSITION` accessor count

### `tinygltf` FetchContent — `cmake/Dependencies.cmake`

Add a FetchContent block pinned to the v2.x stable tag (MIT license, header-only).
Disable `stb_image` with `TINYGLTF_NO_STB_IMAGE` compile definition; `nlohmann_json` is
already provided by the existing dependency.

### Implementation Notes

Vertex buffer layout: `[east_m, north_m, up_m, r, g, b, a]` per vertex (4-byte aligned).
Use `tinygltf::Model` to build the scene graph: one `Mesh` per tile, one `Node` with
`translation` = centroid ENU offset from world origin.
Root node `extras` JSON: `{"liteaerosim_terrain": true}`.
Vertex duplication: 3 unique vertices per triangle (required for per-vertex `COLOR_0`).

---

## Step 12 — `SimulationFrame` + `TrajectoryFile` (2 tests) ✅

### Failing Tests — `test/TrajectoryFile_test.cpp`

- **T1** `TrajectoryFile_ProtoRoundTrip_100Frames` — `TrajectoryFile` with 100 frames survives proto round-trip; frame count and first/last timestamps preserved
- **T2** `TrajectoryFrame_AllFieldsRoundTrip` — all fields survive ± 1e-9 for doubles, ± 1e-6 for floats

### Header — `include/SimulationFrame.hpp`

`SimulationFrame` is a value object (no I/O). Transport is Interface Layer only.

```cpp
namespace liteaerosim {
struct SimulationFrame {
    double timestamp_s;
    double latitude_rad;
    double longitude_rad;
    float  height_wgs84_m;
    float  quaternion_w;
    float  quaternion_x;
    float  quaternion_y;
    float  quaternion_z;
};
} // namespace liteaerosim
```

### Proto Messages — append to `proto/liteaerosim.proto`

```proto
message TrajectoryFrame {
    double timestamp_s    = 1;
    double latitude_rad   = 2;
    double longitude_rad  = 3;
    float  height_wgs84_m = 4;
    float  q_w            = 5;
    float  q_x            = 6;
    float  q_y            = 7;
    float  q_z            = 8;
}

message TrajectoryFile {
    int32  schema_version           = 1;
    double world_origin_lat_rad     = 2;
    double world_origin_lon_rad     = 3;
    float  world_origin_h_m         = 4;
    repeated TrajectoryFrame frames = 5;
}
```

---

## Step 13 — C++ Build and Verify ✅

```bash
cmd.exe /c "set PATH=C:\msys64\ucrt64\bin;%PATH% && cmake --build build && build\test\liteaerosim_test.exe --gtest_filter=TerrainTest.*:TerrainTileTest.*:TerrainMeshTest.*:LodSelectorTest.*:MeshQualityVerifierTest.*:TrajectoryFileTest.*"
```

All C++ tests pass. Pre-existing `FilterTFTest` failures unchanged.

---

## Step 14 — Python Package Scaffold + `.las_terrain` I/O

### Key Design Decision

The Python tools write and read `.las_terrain` using a pure-Python implementation of the
binary format (no pybind11 needed — the format is simple struct-packed data).
`las_terrain.py` is the **single source of truth** for Python-side serialization and is
the foundation all subsequent tools depend on.

### `python/pyproject.toml` additions

Add to `[project.dependencies]`:

```toml
rasterio>=1.3          # GeoTIFF I/O, reprojection, raster sampling
scipy>=1.11            # Delaunay triangulation (triangulate.py)
numpy>=1.26            # array operations throughout
pyproj>=3.6            # CRS transformations (mosaic.py, geoid_correct.py)
pyfqmr>=0.2            # QEM decimation (simplify.py)
trimesh>=4.0           # mesh I/O, GLB export (simplify.py, export_gltf.py)
requests>=2.31         # authenticated API downloads (download.py)
earthaccess>=0.8       # NASA EarthData token management (download.py)
```

Add to `[project.optional-dependencies]` `dev`:

```toml
pytest>=8
pytest-cov>=5
```

### `python/tools/terrain/las_terrain.py`

Public API:
```python
def write_las_terrain(path: Path, tiles: list[TerrainTileData]) -> None: ...
def read_las_terrain(path: Path) -> list[TerrainTileData]: ...

@dataclass
class TerrainTileData:
    lod: int
    centroid_lat_rad: float; centroid_lon_rad: float; centroid_height_m: float
    lat_min_rad: float; lat_max_rad: float
    lon_min_rad: float; lon_max_rad: float
    height_min_m: float; height_max_m: float
    vertices: np.ndarray   # shape (N, 3), float32 — east, north, up
    indices:  np.ndarray   # shape (F, 3), uint32
    colors:   np.ndarray   # shape (F, 3), uint8  — R, G, B per facet
```

Binary layout: struct-packed per §`.las_terrain` File Format in `terrain.md`.

### Failing Tests — `python/test/test_las_terrain.py` (4 tests)

- **T1** `test_round_trip_single_tile` — write one tile, read back; vertex values match to float32 precision
- **T2** `test_round_trip_two_tiles` — two tiles; tile count and centroids preserved
- **T3** `test_magic_bytes` — first 4 bytes of file are `LAST` (0x4C415354 LE)
- **T4** `test_schema_version_mismatch_raises` — read file with format_version ≠ 1 raises `ValueError`

---

## Step 15 — DEM Download, Mosaic, and Geoid Correction

### `python/tools/terrain/download.py`

Downloads elevation and imagery tiles from:

- **Copernicus Data Space** (Sentinel-2 L2A, Copernicus DEM GLO-30) — OAuth2 token via `requests`
- **NASA EarthData** (SRTM, Landsat-8/9, MODIS MCD43A4) — bearer token via `earthaccess`

Public API:

```python
def download_dem(bbox_deg: tuple[float,float,float,float],
                 output_dir: Path,
                 source: str = "copernicus_dem") -> list[Path]: ...

def download_imagery(bbox_deg: tuple[float,float,float,float],
                     output_dir: Path,
                     source: str = "sentinel2",
                     lod: int = 0) -> list[Path]: ...
```

### `python/tools/terrain/mosaic.py`

Merges and reprojects downloaded GeoTIFF tiles to a common CRS and resolution.

```python
def mosaic_dem(input_paths: list[Path],
               output_path: Path,
               target_epsg: int = 4326,
               target_resolution_deg: float | None = None) -> None: ...
```

Uses rasterio.merge and rasterio.warp.reproject.

### `python/tools/terrain/geoid_correct.py`

Converts raster heights from geoid (mean sea level) to ellipsoidal (WGS84) by adding the
EGM2008 geoid undulation.  Undulation values are sampled from the 1-arcminute EGM2008
grid shipped as a small bundled data file (`python/tools/terrain/data/egm2008_1min.npz`).

```python
def apply_geoid_correction(dem_path: Path, output_path: Path) -> None: ...
def geoid_undulation(lat_deg: float, lon_deg: float) -> float: ...
```

### Failing Tests — `python/test/test_geoid_correct.py` (3 tests)

- **T1** `test_undulation_equator_prime_meridian` — geoid undulation at (0°, 0°) ≈ 17.2 m ± 0.5 m
- **T2** `test_corrected_height_equals_msl_plus_undulation` — synthetic raster: output = input + undulation ± 0.1 m
- **T3** `test_mosaic_preserves_bbox` — merged raster covers union of input bounding boxes

---

## Step 16 — L0 TIN Triangulation

### `python/tools/terrain/triangulate.py`

Builds an L0 TIN from a DEM raster using Delaunay triangulation.  Boundary vertices are
locked to ensure adjacent tiles share identical edge positions.

```python
def triangulate(dem_path: Path,
                output_tile: Path,
                lod: int = 0,
                max_edge_length_m: float = 1000.0,
                boundary_points: np.ndarray | None = None) -> TerrainTileData: ...
```

Algorithm:

1. Sample DEM on a regular grid with spacing ≈ target LOD vertex spacing.
2. Add boundary points from `boundary_points` argument (shared with adjacent tiles) if provided.
3. Run `scipy.spatial.Delaunay` on the (lat, lon) 2D point set.
4. Convert each point to ENU float32 offset from tile centroid.
5. Return `TerrainTileData` with all facets assigned default color `{128, 128, 128}`.

### Failing Tests — `python/test/test_triangulate.py` (4 tests)

- **T1** `test_flat_raster_produces_valid_mesh` — 5×5 grid → 32 triangles; all vertices at correct ENU z
- **T2** `test_vertex_count_equals_grid_points` — N×M raster → N×M vertices (before Delaunay)
- **T3** `test_boundary_vertices_locked` — boundary points appear verbatim in output vertex list
- **T4** `test_output_passesqualityverifier` — triangulated tile passes Python quality check (min_angle ≥ 10°, max_aspect_ratio ≤ 15)

---

## Step 17 — Facet Colorization

### `python/tools/terrain/colorize.py`

Assigns per-facet RGB colors by sampling the source imagery raster at each facet centroid
(mean of 3 vertex lat/lon positions in geodetic space).

```python
def colorize(tile: TerrainTileData,
             imagery_path: Path,
             source: str = "sentinel2") -> TerrainTileData: ...
```

For each facet:

1. Compute centroid lat/lon from the tile's `centroid_lat_rad`/`centroid_lon_rad` plus ENU offsets.
2. Sample imagery raster with `rasterio.DatasetReader.sample()`.
3. Scale from source 16-bit integer to 8-bit using product reflectance scale factor.
4. Facets with no coverage (cloud mask, out-of-bounds): default `{128, 128, 128}`.

### Failing Tests — `python/test/test_colorize.py` (3 tests)

- **T1** `test_solid_color_raster_all_facets_same_color` — constant-value raster → all facets identical color
- **T2** `test_cloud_masked_pixel_returns_default_grey` — raster with nodata at centroid position → `{128, 128, 128}`
- **T3** `test_color_scaling_16bit_to_8bit` — known 16-bit value with known scale factor → expected 8-bit result

---

## Step 18 — LOD Simplification

### `python/tools/terrain/simplify.py`

Produces L1–L6 tiles from an L0 tile using QEM (quadric error metric) decimation via
pyfqmr.  `preserve_border=True` locks all tile-boundary vertices.

```python
def simplify(tile: TerrainTileData,
             target_lod: int,
             max_vertical_error_m: float | None = None) -> TerrainTileData: ...
```

`max_vertical_error_m` defaults per the transition table in §Mesh Simplification in
`terrain.md`.  After decimation, per-face colors are transferred from the nearest L0 facet
centroid using trimesh's KD-tree proximity.  The result is passed through `verify()` (Step 19);
tiles that fail quality thresholds raise `MeshQualityError`.

### Failing Tests — `python/test/test_simplify.py` (4 tests)

- **T1** `test_simplify_reduces_face_count` — L0 → L1; output face count < input face count
- **T2** `test_boundary_vertices_preserved` — all boundary vertices of L0 appear in L1 output
- **T3** `test_color_transferred_to_simplified_mesh` — non-grey color in L0 is present in L1 facets
- **T4** `test_degenerate_input_raises` — tile with zero-area facets raises `MeshQualityError`

---

## Step 19 — Python Quality Verification

### `python/tools/terrain/verify.py`

Reimplements `MeshQualityVerifier` in Python/numpy.  This is intentionally a Python-only
reimplementation — no pybind11 bindings needed — because the verification runs offline in
the ingestion pipeline, not in the simulation loop.

```python
@dataclass
class MeshQualityReport:
    min_interior_angle_deg: float
    max_aspect_ratio: float
    degenerate_facet_count: int
    open_boundary_edge_count: int

class MeshQualityError(ValueError): ...

def verify(tile: TerrainTileData) -> MeshQualityReport: ...
def check(tile: TerrainTileData,
          min_angle_deg: float = 10.0,
          max_aspect_ratio: float = 15.0) -> None: ...  # raises MeshQualityError on failure
```

Angles and edge lengths are computed in ECEF (matching C++ `MeshQualityVerifier`).

### Failing Tests — `python/test/test_verify.py` (4 tests)

- **T1** `test_equilateral_mesh_passes` — equilateral triangle tile: `min_interior_angle_deg` ≈ 60 ± 0.1; `check()` does not raise
- **T2** `test_degenerate_facet_detected` — collinear vertices → `degenerate_facet_count == 1`; `check()` raises `MeshQualityError`
- **T3** `test_open_boundary_edges` — single triangle tile → `open_boundary_edge_count == 3`
- **T4** `test_thin_triangle_fails_aspect_ratio` — base 1000 m, height 10 m → `max_aspect_ratio > 15`; `check()` raises

---

## Step 20 — GLB Export

### `python/tools/terrain/export_gltf.py`

Exports a `TerrainTileData` (or list of tiles) to a binary GLB file using **trimesh**.
Axis convention matches the C++ `exportGltf()` implementation: X=East, Y=Up, Z=−North.
Per-facet color is baked to per-vertex `COLOR_0` (3 vertices per triangle).

```python
def export_gltf(tiles: list[TerrainTileData],
                output_path: Path,
                world_origin: tuple[float, float, float] | None = None) -> None: ...
```

The `world_origin` is (lat_rad, lon_rad, height_m); tile node translations are ENU offsets
from this origin.  If `None`, the first tile centroid is used.

The root scene extras carry:

```json
{"liteaerosim_terrain": true, "schema_version": 1,
 "world_origin_lat_rad": ..., "world_origin_lon_rad": ..., "world_origin_height_m": ...}
```

### Failing Tests — `python/test/test_export_gltf.py` (3 tests)

- **T1** `test_glb_magic_bytes` — first 4 bytes of output file are `b"glTF"` (`0x67 0x6C 0x54 0x46`)
- **T2** `test_position_accessor_count` — POSITION accessor element count == 3 × facet count
- **T3** `test_extras_present` — GLB JSON chunk contains `"liteaerosim_terrain"`

---

## Step 21 — Full Pipeline Integration Test

### Failing Test — `python/test/test_pipeline.py` (1 test)

- **T1** `test_synthetic_pipeline_end_to_end` — synthetic 5×5 km flat DEM at the equator:
  1. `triangulate()` → L0 tile
  2. `simplify()` → L1 tile (face count < L0)
  3. `colorize()` → all facets white (solid-color imagery)
  4. `export.write_las_terrain()` → file size > 0; `read_las_terrain()` recovers both tiles
  5. `export_gltf()` → output file has GLB magic bytes and POSITION count == 3 × facet count

All intermediate quality checks pass.

---

## Step 22 — Python Build and Verify

```bash
cd python && uv run pytest test/ -v --tb=short
```

All new Python tests pass.

---

## Risk Notes

1. **`elevation_m()` performance** — point-in-triangle search is O(N_facets) per query; acceptable for regional meshes (≤10,000 facets) but may need a spatial acceleration structure (e.g. bounding-volume hierarchy or uniform grid) for dense L0 meshes.
2. **`facetNormal()` precision** — ECEF cross-products use float32 vertex offsets; for very small facets at L0 (10 m spacing) the normal vector may have float32 rounding. Use double arithmetic internally.
3. **tinygltf API surface** — tinygltf v2.x is header-only but pulls in `nlohmann_json` and `stb_image`; the project already has `nlohmann_json` so only `stb_image` is new (disable with `TINYGLTF_NO_STB_IMAGE`).
4. **`.las_terrain` endianness** — write and read as little-endian (x86/ARM default); document in the file format spec.
5. **`LodSelector` cell hash key** — must match the `TerrainMesh` internal key encoding exactly (see §Internal Spatial Index in `terrain.md`); mismatched keys cause silent LOD state carry-over on the wrong cell.
