#include <liteaero/terrain/V_Terrain.hpp>
#include <gtest/gtest.h>

using namespace liteaero::terrain;

// ---------------------------------------------------------------------------
// T1: FlatTerrain returns the same elevation regardless of position
// ---------------------------------------------------------------------------
TEST(TerrainTest, FlatTerrain_ElevationConstant) {
    FlatTerrain t(300.f);
    EXPECT_FLOAT_EQ(t.elevation_m(0.0, 0.0), 300.f);
    EXPECT_FLOAT_EQ(t.elevation_m(0.5, 1.0), 300.f);
    EXPECT_FLOAT_EQ(t.elevation_m(-1.0, 3.14), 300.f);
}

// ---------------------------------------------------------------------------
// T2: heightAboveGround is the altitude minus terrain elevation when positive
// ---------------------------------------------------------------------------
TEST(TerrainTest, HeightAboveGround_AboveTerrain) {
    FlatTerrain t(300.f);
    EXPECT_FLOAT_EQ(t.heightAboveGround_m(500.f, 0.0, 0.0), 200.f);
}

// ---------------------------------------------------------------------------
// T3: heightAboveGround is zero when exactly at terrain elevation
// ---------------------------------------------------------------------------
TEST(TerrainTest, HeightAboveGround_AtTerrain) {
    FlatTerrain t(300.f);
    EXPECT_FLOAT_EQ(t.heightAboveGround_m(300.f, 0.0, 0.0), 0.f);
}

// ---------------------------------------------------------------------------
// T4: heightAboveGround is clamped to zero when below terrain (no negative)
// ---------------------------------------------------------------------------
TEST(TerrainTest, HeightAboveGround_BelowTerrain_Clamped) {
    FlatTerrain t(300.f);
    EXPECT_FLOAT_EQ(t.heightAboveGround_m(100.f, 0.0, 0.0), 0.f);
}
