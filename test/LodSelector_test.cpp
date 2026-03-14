#define _USE_MATH_DEFINES
#include "environment/LodSelector.hpp"
#include "environment/TerrainMesh.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaerosim::environment;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Tile with height bounds near 0 so slant range equals observer altitude.
static TerrainTile makeFlatTile(TerrainLod lod, GeodeticPoint centroid) {
    GeodeticAABB bounds{
        centroid.latitude_rad  - 1e-4, centroid.latitude_rad  + 1e-4,
        centroid.longitude_rad - 1e-4, centroid.longitude_rad + 1e-4,
        -1.f, 1.f,  // height centroid = 0 m
    };
    std::vector<TerrainVertex> vertices{
        {   0.f,    0.f, 0.f},
        {1000.f,    0.f, 0.f},
        {   0.f, 1000.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {128, 64, 32}}};
    return TerrainTile(lod, centroid, bounds, std::move(vertices), std::move(facets));
}

static TerrainMesh makeDualLodMesh() {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    TerrainCell cell;
    cell.addTile(makeFlatTile(TerrainLod::L0_Finest,        centroid));
    cell.addTile(makeFlatTile(TerrainLod::L1_VeryHighDetail, centroid));
    TerrainMesh mesh;
    mesh.addCell(std::move(cell));
    return mesh;
}

// ---------------------------------------------------------------------------
// Step 7 — LodSelector
// ---------------------------------------------------------------------------

// TerrainMesh::selectLodBySlantRange: observer altitude sets slant range.
// Cell centroid at (0,0,h=0).  Observer at (0,0,alt_m) → r ≈ alt_m.
// r < 300 m → L0;  300 ≤ r < 900 m → L1.
TEST(LodSelectorTest, SelectBySlantRange_NominalBoundaries) {
    const TerrainMesh mesh = makeDualLodMesh();

    // r ≈ 200 m < 300 m → L0.
    auto refs_close = mesh.selectLodBySlantRange(0.0, 0.0, 200.0);
    ASSERT_EQ(refs_close.size(), 1u);
    EXPECT_EQ(refs_close[0].lod, TerrainLod::L0_Finest);

    // r ≈ 500 m, in [300, 900) → L1.
    auto refs_mid = mesh.selectLodBySlantRange(0.0, 0.0, 500.0);
    ASSERT_EQ(refs_mid.size(), 1u);
    EXPECT_EQ(refs_mid[0].lod, TerrainLod::L1_VeryHighDetail);
}

// LodSelector first call uses same nominal formula.
TEST(LodSelectorTest, FirstCall_UseNominalFormula) {
    const TerrainMesh mesh = makeDualLodMesh();
    LodSelector selector;

    auto refs = selector.select(mesh, 0.0, 0.0, 200.0);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L0_Finest);

    selector.reset();
    refs = selector.select(mesh, 0.0, 0.0, 500.0);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L1_VeryHighDetail);
}

// r moves from 270 m to 310 m: above nominal L0/L1 boundary (300 m)
// but inside dead band (300 × 1.15 = 345 m) → L0 unchanged.
TEST(LodSelectorTest, SecondCall_InsideDeadBand_NoTransition) {
    const TerrainMesh mesh = makeDualLodMesh();
    LodSelector selector;

    (void)selector.select(mesh, 0.0, 0.0, 270.0);  // first call → L0 committed

    const auto refs = selector.select(mesh, 0.0, 0.0, 310.0);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L0_Finest);
}

// r moves from 270 m to 360 m: above dead-band threshold 345 m → transitions to L1.
TEST(LodSelectorTest, SecondCall_OutsideDeadBand_Transitions) {
    const TerrainMesh mesh = makeDualLodMesh();
    LodSelector selector;

    (void)selector.select(mesh, 0.0, 0.0, 270.0);  // first call → L0 committed

    const auto refs = selector.select(mesh, 0.0, 0.0, 360.0);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L1_VeryHighDetail);
}

// After reset(), committed state is cleared — cold selection on next call.
TEST(LodSelectorTest, Reset_ClearsHysteresisState) {
    const TerrainMesh mesh = makeDualLodMesh();
    LodSelector selector;

    // Drive committed to L1.
    (void)selector.select(mesh, 0.0, 0.0, 270.0);  // → L0
    (void)selector.select(mesh, 0.0, 0.0, 360.0);  // → L1 (past dead band)

    selector.reset();

    // After reset, r ≈ 270 m → L0 (no carry-over of L1 committed state).
    const auto refs = selector.select(mesh, 0.0, 0.0, 270.0);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_EQ(refs[0].lod, TerrainLod::L0_Finest);
}
