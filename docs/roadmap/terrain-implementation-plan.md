# Plan: Terrain Model — Roadmap Item 1 (1a and 1b)

## Context

Implement the full terrain subsystem per the design authority
`docs/architecture/terrain.md`. These are Domain Layer components; no I/O, all SI.
The roadmap splits the work as:

- Item 1a: `V_Terrain` + `FlatTerrain` (prerequisite for sensors and guidance)
- Item 1b: `TerrainMesh`, `TerrainTile`, `TerrainCell`, `LodSelector`, `TerrainSimplifier`,
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

---

## Files to Create / Modify

| File | Action |
|------|--------|
| `include/environment/GeodeticPoint.hpp` | **Create** |
| `include/environment/TerrainVertex.hpp` | **Create** |
| `include/environment/TerrainFacet.hpp` | **Create** |
| `include/environment/GeodeticAABB.hpp` | **Create** |
| `include/environment/LocalAABB.hpp` | **Create** |
| `include/environment/Terrain.hpp` | **Create** — `V_Terrain` + `FlatTerrain` |
| `include/environment/TerrainTile.hpp` | **Create** — includes `TerrainLod` enum class |
| `include/environment/TerrainCell.hpp` | **Create** |
| `include/environment/TerrainMesh.hpp` | **Create** |
| `include/environment/LodSelector.hpp` | **Create** |
| `include/environment/TerrainSimplifier.hpp` | **Create** |
| `include/environment/MeshQualityVerifier.hpp` | **Create** |
| `include/SimulationFrame.hpp` | **Create** |
| `src/environment/Terrain.cpp` | **Create** |
| `src/environment/TerrainTile.cpp` | **Create** |
| `src/environment/TerrainCell.cpp` | **Create** |
| `src/environment/TerrainMesh.cpp` | **Create** |
| `src/environment/LodSelector.cpp` | **Create** |
| `src/environment/TerrainSimplifier.cpp` | **Create** |
| `src/environment/MeshQualityVerifier.cpp` | **Create** |
| `test/Terrain_test.cpp` | **Create** — 4 tests |
| `test/TerrainTile_test.cpp` | **Create** — 8 tests |
| `test/TerrainMesh_test.cpp` | **Create** — steps 4–8, 11–12; ~25 tests |
| `test/LodSelector_test.cpp` | **Create** — 5 tests |
| `test/TerrainSimplifier_test.cpp` | **Create** — 4 tests |
| `test/MeshQualityVerifier_test.cpp` | **Create** — 4 tests |
| `test/TrajectoryFile_test.cpp` | **Create** — 2 tests |
| `proto/liteaerosim.proto` | **Modify** — append `TerrainTileProto`, `TerrainMeshState`, `TrajectoryFrame`, `TrajectoryFile` |
| `cmake/Dependencies.cmake` | **Modify** — add `tinygltf` FetchContent block |

`CMakeLists.txt`, `src/CMakeLists.txt`, and `test/CMakeLists.txt` need **no changes**.

---

## Step 1 — Plain Data-Model Headers (no tests — pure structs)

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

## Step 2 — `V_Terrain` + `FlatTerrain` (roadmap 1a, 4 tests)

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

## Step 3 — `TerrainTile` + `TerrainCell` (8 tests)

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

## Step 4 — `TerrainMesh` Core: `addCell()`, `cellAt()`, `elevation_m()` (5 tests)

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

## Step 5 — Coordinate Transforms: `toECEF()`, `toNED()` (4 tests)

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

## Step 6 — Subset Queries: `queryLocalAABB()`, `queryGeodeticAABB()`, `querySphere()` (6 tests)

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

## Step 7 — LOD Selection + `LodSelector` (5 tests)

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

## Step 8 — Line-of-Sight Query (3 tests)

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

## Step 9 — `TerrainSimplifier` (4 tests)

### Failing Tests — `test/TerrainSimplifier_test.cpp`

- **T1** `OutputLod_IsInputPlusOne` — output `lod()` equals `static_cast<TerrainLod>(static_cast<int>(input.lod()) + 1)`
- **T2** `OutputHasFewerVertices` — output `vertices().size()` < input `vertices().size()` for a mesh with many vertices
- **T3** `MaxVerticalError_Respected` — for every output vertex, its height differs from the nearest input surface by ≤ `max_vertical_error_m`
- **T4** `BoundaryVerticesPreserved` — every vertex that was on the input tile boundary is present in the output tile (boundary locking)

### Header — `include/environment/TerrainSimplifier.hpp`

```cpp
namespace liteaerosim::environment {

struct SimplificationParams {
    float max_vertical_error_m = 10.f;
    float crease_angle_rad     = 1.047f;  // ~60°
    float min_angle_rad        = 0.087f;  // ~5°
};

class TerrainSimplifier {
public:
    [[nodiscard]] static TerrainTile simplify(const TerrainTile&          source_tile,
                                              const SimplificationParams& params = {});
};

} // namespace liteaerosim::environment
```

### Implementation Notes

Half-edge collapse with QEM (Quadric Error Metrics):
- Vertical error metric augmented into the quadric.
- Boundary vertices locked by checking if vertex lies on the tile `GeodeticAABB` edge.
- Stopping criterion: next collapse would exceed `max_vertical_error_m`.
- See Risk Note 2 for a simplified first-pass strategy.

---

## Step 10 — `MeshQualityVerifier` (4 tests)

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

## Step 11 — Serialization: JSON + Proto + `.las_terrain` (5 tests)

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

## Step 12 — `TerrainMesh::exportGltf()` (4 tests)

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

## Step 13 — `SimulationFrame` + `TrajectoryFile` (2 tests)

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

## Step 14 — Build and Verify

```bash
cmd.exe /c "set PATH=C:\msys64\ucrt64\bin;%PATH% && cmake --build build && build\test\liteaerosim_test.exe --gtest_filter=TerrainTest.*:TerrainTileTest.*:TerrainMeshTest.*:LodSelectorTest.*:TerrainSimplifierTest.*:MeshQualityVerifierTest.*:TrajectoryFileTest.*"
```

All new tests pass. Pre-existing `FilterTFTest` failures unchanged.

---

## Risk Notes

1. **`elevation_m()` performance** — point-in-triangle search is O(N_facets) per query; acceptable for regional meshes (≤10,000 facets) but may need a spatial acceleration structure (e.g. bounding-volume hierarchy or uniform grid) for dense L0 meshes.
2. **QEM simplification complexity** — full QEM with priority queue is O(N log N); a simplified greedy approach may be sufficient for offline use. First pass: use edge-length heuristic without full quadric error matrix; upgrade if quality thresholds are not met.
3. **`facetNormal()` precision** — ECEF cross-products use float32 vertex offsets; for very small facets at L0 (10 m spacing) the normal vector may have float32 rounding. Use double arithmetic internally.
4. **tinygltf API surface** — tinygltf v2.x is header-only but pulls in `nlohmann_json` and `stb_image`; the project already has `nlohmann_json` so only `stb_image` is new (disable with `TINYGLTF_NO_STB_IMAGE`).
5. **`.las_terrain` endianness** — write and read as little-endian (x86/ARM default); document in the file format spec.
6. **`LodSelector` cell hash key** — must match the `TerrainMesh` internal key encoding exactly (see §Internal Spatial Index in `terrain.md`); mismatched keys cause silent LOD state carry-over on the wrong cell.
