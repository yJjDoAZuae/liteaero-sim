#define _USE_MATH_DEFINES
#include "environment/TerrainMesh.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaerosim::environment;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Minimal 3-vertex tile at a given centroid.  V0 is at ENU (0,0,100m),
// V1 at (1000,0,50m), V2 at (0,1000,50m).  Bounds cover ±1e-4 rad around centroid.
static TerrainTile makeTriangleTile(TerrainLod lod, GeodeticPoint centroid) {
    GeodeticAABB bounds{
        centroid.latitude_rad  - 1e-4, centroid.latitude_rad  + 1e-4,
        centroid.longitude_rad - 1e-4, centroid.longitude_rad + 1e-4,
        -1.f, 101.f,
    };
    std::vector<TerrainVertex> vertices{
        {   0.f,    0.f, 100.f},  // V0 — at centroid lat/lon, up = 100 m
        {1000.f,    0.f,  50.f},  // V1 — 1 km east
        {   0.f, 1000.f,  50.f}, // V2 — 1 km north
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {128, 64, 32}}};
    return TerrainTile(lod, centroid, bounds,
                       std::move(vertices), std::move(facets));
}

static TerrainMesh makeEquatorMesh(TerrainLod lod = TerrainLod::L2_HighDetail) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    TerrainCell cell;
    cell.addTile(makeTriangleTile(lod, centroid));
    TerrainMesh mesh;
    mesh.addCell(std::move(cell));
    return mesh;
}

// ---------------------------------------------------------------------------
// Step 4 — TerrainMesh Core
// ---------------------------------------------------------------------------

TEST(TerrainMeshTest, AddCell_ThenCellAtCentroid_ReturnsNonNull) {
    const TerrainMesh mesh = makeEquatorMesh();
    EXPECT_NE(mesh.cellAt(0.0, 0.0), nullptr);
}

TEST(TerrainMeshTest, CellAt_OffsetWithinHalfExtent_FindsCell) {
    const TerrainMesh mesh = makeEquatorMesh();
    // Bounds are ±1e-4 rad; 5e-5 < 1e-4 in both lat and lon.
    EXPECT_NE(mesh.cellAt( 5e-5,  0.0), nullptr);
    EXPECT_NE(mesh.cellAt( 0.0,  5e-5), nullptr);
}

TEST(TerrainMeshTest, CellAt_OutsideAllCells_ReturnsNullptr) {
    const TerrainMesh mesh = makeEquatorMesh();
    EXPECT_EQ(mesh.cellAt(1.0, 1.0), nullptr);
}

// elevation_m at centroid lat/lon: query maps to ENU (0,0) = V0 → height = 100 m.
TEST(TerrainMeshTest, ElevationM_AtCentroid_ReturnsVertexHeight) {
    const TerrainMesh mesh = makeEquatorMesh();
    EXPECT_NEAR(mesh.elevation_m(0.0, 0.0), 100.f, 0.01f);
}

TEST(TerrainMeshTest, ElevationM_OutsideAllCells_ReturnsZero) {
    const TerrainMesh mesh = makeEquatorMesh();
    EXPECT_FLOAT_EQ(mesh.elevation_m(1.0, 1.0), 0.f);
}

// ---------------------------------------------------------------------------
// Step 5 — Coordinate Transforms
// ---------------------------------------------------------------------------

// Vertex at ENU (0,0,0) from centroid at (0°N, 0°E, 0m) → ECEF x ≈ a (semi-major axis).
TEST(TerrainMeshTest, ToECEF_AtEquatorPrimeMeridian) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-5, 1e-5, -1e-5, 1e-5, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
    std::vector<TerrainFacet>  facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    TerrainMesh mesh;
    const auto ecef = mesh.toECEF(tile);

    ASSERT_EQ(ecef.size(), 3u);
    EXPECT_NEAR(ecef[0][0], 6378137.0, 0.1);  // X ≈ a
    EXPECT_NEAR(ecef[0][1], 0.0,       0.1);
    EXPECT_NEAR(ecef[0][2], 0.0,       0.1);
}

// Vertex at ENU (0,0,0) from centroid at (0°N, 90°E, 0m) → ECEF y ≈ a.
TEST(TerrainMeshTest, ToECEF_AtEquator90E) {
    const double lon_90e = M_PI / 2.0;
    GeodeticPoint centroid{0.0, lon_90e, 0.f};
    GeodeticAABB  bounds{-1e-5, 1e-5, lon_90e - 1e-5, lon_90e + 1e-5, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
    std::vector<TerrainFacet>  facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    TerrainMesh mesh;
    const auto ecef = mesh.toECEF(tile);

    ASSERT_EQ(ecef.size(), 3u);
    EXPECT_NEAR(ecef[0][0], 0.0,       0.1);
    EXPECT_NEAR(ecef[0][1], 6378137.0, 0.1);  // Y ≈ a
    EXPECT_NEAR(ecef[0][2], 0.0,       0.1);
}

// Vertex at ENU (0,0,0) from centroid (0°N, 0°E, 0m) → NED (0, 0, 0).
TEST(TerrainMeshTest, ToNED_AtCentroidPosition_ReturnsZero) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-5, 1e-5, -1e-5, 1e-5, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
    std::vector<TerrainFacet>  facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    TerrainMesh mesh;
    const auto ned = mesh.toNED(tile, 0.0, 0.0, 0.0);

    ASSERT_EQ(ned.size(), 3u);
    EXPECT_NEAR(ned[0][0], 0.f, 0.01f);  // N = 0
    EXPECT_NEAR(ned[0][1], 0.f, 0.01f);  // E = 0
    EXPECT_NEAR(ned[0][2], 0.f, 0.01f);  // D = 0
}

// Vertex 100 m east of centroid (0°N, 0°E, 0m); NED relative to same ref → (0, 100, 0).
TEST(TerrainMeshTest, ToNED_100mEast_ReturnsCorrectNED) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-5, 1e-5, -1e-5, 1e-5, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{
        {100.f, 0.f, 0.f},
        {100.f, 0.f, 0.f},
        {100.f, 0.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    TerrainMesh mesh;
    const auto ned = mesh.toNED(tile, 0.0, 0.0, 0.0);

    ASSERT_EQ(ned.size(), 3u);
    EXPECT_NEAR(ned[0][0],   0.f, 0.1f);  // N ≈ 0
    EXPECT_NEAR(ned[0][1], 100.f, 0.1f);  // E ≈ 100
    EXPECT_NEAR(ned[0][2],   0.f, 0.1f);  // D ≈ 0
}

// ---------------------------------------------------------------------------
// Step 6 — Subset Queries
// ---------------------------------------------------------------------------

TEST(TerrainMeshTest, QueryLocalAABB_CenteredOnTile_ReturnsCell) {
    const TerrainMesh mesh = makeEquatorMesh(TerrainLod::L0_Finest);
    // ±5 km half-extents centered on tile centroid (0,0,0) — easily covers the tile.
    LocalAABB aabb{5000.f, 5000.f, -10.f, 200.f};
    const auto refs = mesh.queryLocalAABB(0.0, 0.0, 0.f, aabb, TerrainLod::L0_Finest);
    EXPECT_EQ(refs.size(), 1u);
}

TEST(TerrainMeshTest, QueryLocalAABB_FarFromAllTiles_ReturnsEmpty) {
    const TerrainMesh mesh = makeEquatorMesh(TerrainLod::L0_Finest);
    // ±500 m AABB centered ~640 km from the tile.
    LocalAABB aabb{500.f, 500.f, -10.f, 200.f};
    const auto refs = mesh.queryLocalAABB(0.1, 0.1, 0.f, aabb, TerrainLod::L0_Finest);
    EXPECT_EQ(refs.size(), 0u);
}

TEST(TerrainMeshTest, QueryGeodeticAABB_Overlapping_ReturnsTile) {
    const TerrainMesh mesh = makeEquatorMesh(TerrainLod::L0_Finest);
    // Bounds larger than the tile bounds — guaranteed overlap.
    GeodeticAABB query{-5e-4, 5e-4, -5e-4, 5e-4, -10.f, 200.f};
    const auto refs = mesh.queryGeodeticAABB(query, TerrainLod::L0_Finest);
    EXPECT_EQ(refs.size(), 1u);
}

TEST(TerrainMeshTest, QueryGeodeticAABB_NonOverlapping_ReturnsEmpty) {
    const TerrainMesh mesh = makeEquatorMesh(TerrainLod::L0_Finest);
    // Bounds far from tile at (0,0).
    GeodeticAABB query{0.5, 1.0, 0.5, 1.0, -10.f, 200.f};
    const auto refs = mesh.queryGeodeticAABB(query, TerrainLod::L0_Finest);
    EXPECT_EQ(refs.size(), 0u);
}

TEST(TerrainMeshTest, QuerySphere_WithinRadius_ReturnsTile) {
    const TerrainMesh mesh = makeEquatorMesh(TerrainLod::L0_Finest);
    // 10 km sphere centered at tile centroid — easily covers the tile.
    const auto refs = mesh.querySphere(0.0, 0.0, 0.f, 10000.f, TerrainLod::L0_Finest);
    EXPECT_EQ(refs.size(), 1u);
}

// Cell with L0 and L2 tiles; max_lod = L1 → finest tile with lod >= L1 is L2.
TEST(TerrainMeshTest, Query_MaxLodFiltering) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    TerrainCell cell;
    cell.addTile(makeTriangleTile(TerrainLod::L0_Finest,    centroid));
    cell.addTile(makeTriangleTile(TerrainLod::L2_HighDetail, centroid));

    TerrainMesh mesh;
    mesh.addCell(std::move(cell));

    const auto refs = mesh.querySphere(0.0, 0.0, 0.f, 10000.f,
                                        TerrainLod::L1_VeryHighDetail);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L2_HighDetail);
}
