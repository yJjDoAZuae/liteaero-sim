#define _USE_MATH_DEFINES
#include "environment/TerrainMesh.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <nlohmann/json.hpp>

using namespace liteaero::terrain;
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

// ---------------------------------------------------------------------------
// Step 8 — Line-of-Sight
// ---------------------------------------------------------------------------

// Flat terrain at up=0.  Two points at h=100 m with no terrain in between.
static TerrainMesh makeFlatZeroMesh() {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-4, 1e-4, -1e-4, 1e-4, -2.f, 2.f};
    std::vector<TerrainVertex> vertices{
        {   0.f,    0.f, 0.f},
        {1000.f,    0.f, 0.f},
        {   0.f, 1000.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {128, 64, 32}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));
    TerrainCell cell;
    cell.addTile(std::move(tile));
    TerrainMesh mesh;
    mesh.addCell(std::move(cell));
    return mesh;
}

// Vertical triangle in the ENU east=0 plane (blocks east-west LOS).
static TerrainMesh makeRidgeMesh() {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-2, 1e-2, -1e-2, 1e-2, -2.f, 600.f};
    std::vector<TerrainVertex> vertices{
        {0.f, -5000.f,   0.f},  // V0 — south
        {0.f,  5000.f,   0.f},  // V1 — north
        {0.f,     0.f, 500.f},  // V2 — peak
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {128, 64, 32}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));
    TerrainCell cell;
    cell.addTile(std::move(tile));
    TerrainMesh mesh;
    mesh.addCell(std::move(cell));
    return mesh;
}

// Both endpoints well above flat terrain at h=0 — no intersection.
TEST(TerrainMeshTest, LineOfSight_AboveFlatTerrain_ReturnsTrue) {
    const TerrainMesh mesh = makeFlatZeroMesh();
    EXPECT_TRUE(mesh.lineOfSight(0.0, 0.0, 100.f, 0.0, 0.00005, 100.f));
}

// P1 at h=-1 m (below terrain surface at h≈0) → segment crosses terrain.
TEST(TerrainMeshTest, LineOfSight_BelowTerrain_ReturnsFalse) {
    const TerrainMesh mesh = makeFlatZeroMesh();
    // Segment goes from slightly underground to well above — crosses the flat triangle.
    EXPECT_FALSE(mesh.lineOfSight(0.0, 0.0, -1.f, 0.0, 0.00005, 100.f));
}

// Vertical ridge triangle at east=0 blocks LOS between west and east points.
TEST(TerrainMeshTest, LineOfSight_RidgeMesh_ReturnsFalse) {
    const TerrainMesh mesh = makeRidgeMesh();
    // Points on opposite sides of the ridge at h=100 m.
    EXPECT_FALSE(mesh.lineOfSight(0.0, -0.001, 100.f, 0.0, 0.001, 100.f));
}

// ---------------------------------------------------------------------------
// Step 10 — Serialization
// ---------------------------------------------------------------------------

// JSON round-trip with two tiles in different cells preserves centroid ± 1e-6.
TEST(TerrainMeshTest, TerrainMesh_JsonRoundTrip_TwoTiles) {
    TerrainMesh mesh;
    {
        GeodeticPoint c1{0.0, 0.0, 0.f};
        TerrainCell cell1;
        cell1.addTile(makeTriangleTile(TerrainLod::L0_Finest, c1));
        mesh.addCell(std::move(cell1));
    }
    {
        GeodeticPoint c2{0.1, 0.1, 0.f};
        TerrainCell cell2;
        cell2.addTile(makeTriangleTile(TerrainLod::L0_Finest, c2));
        mesh.addCell(std::move(cell2));
    }

    const nlohmann::json j = mesh.serializeJson();
    TerrainMesh mesh2;
    mesh2.deserializeJson(j);

    EXPECT_NE(mesh2.cellAt(0.0, 0.0),   nullptr);
    EXPECT_NE(mesh2.cellAt(0.1, 0.1),   nullptr);
    EXPECT_EQ(mesh2.cellAt(0.5, 0.5),   nullptr);
}

// Proto round-trip preserves vertex count and LOD.
TEST(TerrainMeshTest, TerrainMesh_ProtoRoundTrip) {
    TerrainMesh mesh;
    {
        GeodeticPoint c{0.0, 0.0, 0.f};
        TerrainCell cell;
        cell.addTile(makeTriangleTile(TerrainLod::L2_HighDetail, c));
        mesh.addCell(std::move(cell));
    }

    const auto bytes = mesh.serializeProto();
    TerrainMesh mesh2;
    mesh2.deserializeProto(bytes);

    const TerrainCell* cell = mesh2.cellAt(0.0, 0.0);
    ASSERT_NE(cell, nullptr);
    EXPECT_TRUE(cell->hasLod(TerrainLod::L2_HighDetail));
    EXPECT_EQ(cell->tile(TerrainLod::L2_HighDetail).vertices().size(), 3u);
}

// .las_terrain binary: TerrainMesh with one tile survives write→read.
TEST(TerrainMeshTest, TerrainTile_LasTerrain_BinaryRoundTrip) {
    TerrainMesh mesh;
    {
        GeodeticPoint c{0.0, 0.0, 0.f};
        TerrainCell cell;
        cell.addTile(makeTriangleTile(TerrainLod::L0_Finest, c));
        mesh.addCell(std::move(cell));
    }

    const std::vector<uint8_t> binary = mesh.serializeLasTerrain();
    TerrainMesh mesh2;
    mesh2.deserializeLasTerrain(binary);

    const TerrainCell* cell = mesh2.cellAt(0.0, 0.0);
    ASSERT_NE(cell, nullptr);
    const TerrainTile& tile = cell->tile(TerrainLod::L0_Finest);
    ASSERT_EQ(tile.vertices().size(), 3u);
    EXPECT_NEAR(tile.vertices()[0].up_m, 100.f, 1e-4f);
    EXPECT_NEAR(tile.vertices()[1].east_m, 1000.f, 1e-4f);
}

// JSON schema version mismatch throws std::runtime_error.
TEST(TerrainMeshTest, TerrainMesh_JsonSchemaVersionMismatch_Throws) {
    nlohmann::json j;
    j["schema_version"] = 999;
    j["tiles"]          = nlohmann::json::array();
    TerrainMesh mesh;
    EXPECT_THROW(mesh.deserializeJson(j), std::runtime_error);
}

// Empty TerrainMesh round-trips without error; cellAt returns nullptr.
TEST(TerrainMeshTest, TerrainMesh_EmptySerializeDeserialize) {
    TerrainMesh mesh;
    const nlohmann::json j = mesh.serializeJson();
    TerrainMesh mesh2;
    mesh2.deserializeJson(j);
    EXPECT_EQ(mesh2.cellAt(0.0, 0.0), nullptr);
}

// ---------------------------------------------------------------------------
// Step 11 — glTF / GLB export
// ---------------------------------------------------------------------------

// Helper: read a binary file into a byte vector.
static std::vector<uint8_t> readGlbBytes(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {});
}

// Helper: extract the JSON chunk string from a GLB byte stream.
// GLB layout: [12-byte file header] [chunk0: 4-byte len, 4-byte type "JSON", N bytes content] ...
static std::string glbJsonChunk(const std::vector<uint8_t>& glb) {
    if (glb.size() < 20) return {};
    uint32_t json_len = 0;
    std::memcpy(&json_len, glb.data() + 12, 4);
    const size_t json_start = 20;
    if (glb.size() < json_start + json_len) return {};
    return std::string(glb.begin() + json_start, glb.begin() + json_start + json_len);
}

// T1 — first 4 bytes are the GLB magic "glTF" (0x46546C67 little-endian).
TEST(TerrainMeshTest, ExportGltf_GlbMagicBytes) {
    const auto tmp = std::filesystem::temp_directory_path() / "las_test_magic.glb";
    makeEquatorMesh().exportGltf(tmp, TerrainLod::L0_Finest);

    const auto glb = readGlbBytes(tmp);
    ASSERT_GE(glb.size(), 4u);
    uint32_t magic = 0;
    std::memcpy(&magic, glb.data(), 4);
    EXPECT_EQ(magic, 0x46546C67u);
}

// T2 — root node extras contains "liteaerosim_terrain": true.
TEST(TerrainMeshTest, ExportGltf_LiteaerosimExtrasPresent) {
    const auto tmp = std::filesystem::temp_directory_path() / "las_test_extras.glb";
    makeEquatorMesh().exportGltf(tmp, TerrainLod::L0_Finest);

    const auto json_str = glbJsonChunk(readGlbBytes(tmp));
    EXPECT_NE(json_str.find("liteaerosim_terrain"), std::string::npos);
}

// T3 — POSITION accessor element count == 3 × facet count.
TEST(TerrainMeshTest, ExportGltf_PositionAccessorCount) {
    // makeTriangleTile has 1 facet → expect 3 duplicated vertices.
    const auto tmp = std::filesystem::temp_directory_path() / "las_test_pos.glb";
    makeEquatorMesh().exportGltf(tmp, TerrainLod::L0_Finest);

    const auto json_str = glbJsonChunk(readGlbBytes(tmp));
    const nlohmann::json j = nlohmann::json::parse(json_str);

    int position_count = 0;
    for (const auto& mesh : j["meshes"]) {
        for (const auto& prim : mesh["primitives"]) {
            const auto& attrs = prim["attributes"];
            if (attrs.contains("POSITION")) {
                position_count += j["accessors"][attrs["POSITION"].get<int>()]["count"].get<int>();
            }
        }
    }
    EXPECT_EQ(position_count, 3);  // 1 facet × 3 vertices
}

// T4 — COLOR_0 accessor element count equals POSITION accessor element count.
TEST(TerrainMeshTest, ExportGltf_ColorAccessorMatchesPosition) {
    const auto tmp = std::filesystem::temp_directory_path() / "las_test_color.glb";
    makeEquatorMesh().exportGltf(tmp, TerrainLod::L0_Finest);

    const auto json_str = glbJsonChunk(readGlbBytes(tmp));
    const nlohmann::json j = nlohmann::json::parse(json_str);

    int position_count = 0;
    int color_count    = 0;
    for (const auto& mesh : j["meshes"]) {
        for (const auto& prim : mesh["primitives"]) {
            const auto& attrs = prim["attributes"];
            if (attrs.contains("POSITION")) {
                position_count += j["accessors"][attrs["POSITION"].get<int>()]["count"].get<int>();
            }
            if (attrs.contains("COLOR_0")) {
                color_count += j["accessors"][attrs["COLOR_0"].get<int>()]["count"].get<int>();
            }
        }
    }
    EXPECT_GT(position_count, 0);
    EXPECT_EQ(color_count, position_count);
}
