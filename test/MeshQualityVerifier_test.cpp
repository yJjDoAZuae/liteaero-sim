#define _USE_MATH_DEFINES
#include "environment/MeshQualityVerifier.hpp"
#include "environment/TerrainTile.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaerosim::environment;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static TerrainTile makeTile(std::vector<TerrainVertex> vertices,
                             std::vector<TerrainFacet>  facets) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-1e-4, 1e-4, -1e-4, 1e-4, -1.f, 1.f};
    return TerrainTile(TerrainLod::L0_Finest, centroid, bounds,
                       std::move(vertices), std::move(facets));
}

// ---------------------------------------------------------------------------
// Step 9 — MeshQualityVerifier
// ---------------------------------------------------------------------------

// Equilateral triangle: all interior angles ≈ 60°, aspect ratio ≈ 1.15 < 15.
TEST(MeshQualityVerifierTest, EquilateralMesh_Passes) {
    // Side length 1000 m, equilateral.
    const float s  = 1000.f;
    const float h3 = s * 0.8660254f;  // s * sqrt(3)/2
    std::vector<TerrainVertex> verts{
        {0.f,    0.f, 0.f},
        {  s,    0.f, 0.f},
        {s/2.f, h3,  0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    const TerrainTile tile = makeTile(std::move(verts), std::move(facets));

    const auto report = MeshQualityVerifier::verify(tile);

    EXPECT_NEAR(report.min_interior_angle_deg, 60.f, 0.1f);
    EXPECT_TRUE(MeshQualityVerifier::passes(report));
}

// Zero-area facet (collinear vertices) → degenerate_facet_count == 1, fails.
TEST(MeshQualityVerifierTest, DegenerateMesh_ZeroAreaFacet) {
    // Three collinear points.
    std::vector<TerrainVertex> verts{
        {   0.f, 0.f, 0.f},
        { 500.f, 0.f, 0.f},
        {1000.f, 0.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    const TerrainTile tile = makeTile(std::move(verts), std::move(facets));

    const auto report = MeshQualityVerifier::verify(tile);

    EXPECT_EQ(report.degenerate_facet_count, 1u);
    EXPECT_FALSE(MeshQualityVerifier::passes(report));
}

// Single triangle has exactly 3 open boundary edges.
TEST(MeshQualityVerifierTest, SingleTriangle_BoundaryEdgeCount) {
    std::vector<TerrainVertex> verts{
        {   0.f,    0.f, 0.f},
        {1000.f,    0.f, 0.f},
        {   0.f, 1000.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    const TerrainTile tile = makeTile(std::move(verts), std::move(facets));

    const auto report = MeshQualityVerifier::verify(tile);

    EXPECT_EQ(report.open_boundary_edge_count, 3u);
}

// Very thin triangle: base 1000 m, height 10 m → aspect ratio = 100 > 15.
TEST(MeshQualityVerifierTest, ThinTriangle_HighAspectRatio) {
    std::vector<TerrainVertex> verts{
        {   0.f, 0.f, 0.f},
        {1000.f, 0.f, 0.f},
        { 500.f, 10.f, 0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    const TerrainTile tile = makeTile(std::move(verts), std::move(facets));

    const auto report = MeshQualityVerifier::verify(tile);

    EXPECT_GT(report.max_aspect_ratio, 15.f);
    EXPECT_FALSE(MeshQualityVerifier::passes(report));
}
