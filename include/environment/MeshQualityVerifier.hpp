#pragma once
#include <liteaero/terrain/TerrainTile.hpp>
#include <cstdint>
#include <limits>

namespace liteaerosim::environment {

struct MeshQualityReport {
    float    min_interior_angle_deg   = 180.f;
    float    max_interior_angle_deg   = 0.f;
    float    mean_interior_angle_deg  = 0.f;
    float    min_edge_length_m        = std::numeric_limits<float>::max();
    float    max_edge_length_m        = 0.f;
    float    max_aspect_ratio         = 0.f;
    uint32_t degenerate_facet_count   = 0;
    uint32_t non_manifold_edge_count  = 0;
    uint32_t duplicate_vertex_count   = 0;
    uint32_t open_boundary_edge_count = 0;
    bool     boundary_gap_detected    = false;
};

struct MeshQualityThresholds {
    float    min_interior_angle_deg   = 5.f;    // below this → fail
    float    max_aspect_ratio         = 15.f;   // above this → fail
    uint32_t max_degenerate_facets    = 0;      // non-zero count → fail
};

class MeshQualityVerifier {
public:
    [[nodiscard]] static MeshQualityReport verify(const liteaero::terrain::TerrainTile& tile);
    [[nodiscard]] static bool              passes(const MeshQualityReport& report,
                                                  const MeshQualityThresholds& thresholds = {});
};

} // namespace liteaerosim::environment
