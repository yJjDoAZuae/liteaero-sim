#include "environment/MeshQualityVerifier.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace liteaerosim::environment {

// Normalize an edge key so (a,b) and (b,a) map to the same entry.
static std::pair<uint32_t, uint32_t> edgeKey(uint32_t a, uint32_t b) {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

MeshQualityReport MeshQualityVerifier::verify(const TerrainTile& tile) {
    MeshQualityReport report;

    const auto& verts  = tile.vertices();
    const auto& facets = tile.facets();

    if (facets.empty()) return report;

    // Per-facet metrics.
    float angle_sum   = 0.f;
    int   angle_count = 0;

    // Edge valence map: count how many facets share each edge.
    std::map<std::pair<uint32_t, uint32_t>, int> edge_count;

    for (const auto& f : facets) {
        const float e0 = verts[f.v[1]].east_m  - verts[f.v[0]].east_m;
        const float n0 = verts[f.v[1]].north_m - verts[f.v[0]].north_m;
        const float u0 = verts[f.v[1]].up_m    - verts[f.v[0]].up_m;

        const float e1 = verts[f.v[2]].east_m  - verts[f.v[1]].east_m;
        const float n1 = verts[f.v[2]].north_m - verts[f.v[1]].north_m;
        const float u1 = verts[f.v[2]].up_m    - verts[f.v[1]].up_m;

        const float e2 = verts[f.v[0]].east_m  - verts[f.v[2]].east_m;
        const float n2 = verts[f.v[0]].north_m - verts[f.v[2]].north_m;
        const float u2 = verts[f.v[0]].up_m    - verts[f.v[2]].up_m;

        // Edge lengths.
        const float len_a = std::sqrt(e0*e0 + n0*n0 + u0*u0);  // v0→v1
        const float len_b = std::sqrt(e1*e1 + n1*n1 + u1*u1);  // v1→v2
        const float len_c = std::sqrt(e2*e2 + n2*n2 + u2*u2);  // v2→v0

        report.min_edge_length_m = std::min(report.min_edge_length_m, std::min({len_a, len_b, len_c}));
        report.max_edge_length_m = std::max(report.max_edge_length_m, std::max({len_a, len_b, len_c}));

        // Triangle area via cross product of two edges from v0.
        const float ex = e0, nx = n0, ux = u0;
        const float ey = -e2, ny = -n2, uy = -u2;  // v0→v2 = -(v2→v0)
        const float cx = nx*uy - ux*ny;
        const float cy = ux*ey - ex*uy;
        const float cz = ex*ny - nx*ey;
        const float area2 = std::sqrt(cx*cx + cy*cy + cz*cz);  // 2 × area

        constexpr float kDegenerateThreshold = 1e-6f;
        if (area2 < kDegenerateThreshold) {
            ++report.degenerate_facet_count;
        } else {
            // Interior angles via law of cosines.
            // Angle at v0: between edges v0→v1 and v0→v2.
            const float dot_a = ex*e0 + ey*n0 + ey*u0;  // intentional reuse avoided below
            // Use dot product formula: cos θ = (AB · AC) / (|AB| |AC|)
            const float ve0[3] = {  e0,  n0,  u0};   // v0→v1
            const float ve1[3] = {-e2, -n2, -u2};    // v0→v2
            const float ve2[3] = {-e0, -n0, -u0};    // v1→v0
            const float ve3[3] = { e1,  n1,  u1};    // v1→v2
            const float ve4[3] = {-e1, -n1, -u1};    // v2→v1
            const float ve5[3] = { e2,  n2,  u2};    // v2→v0

            auto angleDeg = [](const float a[3], const float b[3]) -> float {
                const float la = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
                const float lb = std::sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
                if (la < 1e-12f || lb < 1e-12f) return 0.f;
                const float cosA = std::clamp((a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (la*lb),
                                              -1.f, 1.f);
                return std::acos(cosA) * (180.f / 3.14159265358979323846f);
            };

            const float a0 = angleDeg(ve0, ve1);
            const float a1 = angleDeg(ve2, ve3);
            const float a2 = angleDeg(ve4, ve5);

            for (float ang : {a0, a1, a2}) {
                report.min_interior_angle_deg = std::min(report.min_interior_angle_deg, ang);
                report.max_interior_angle_deg = std::max(report.max_interior_angle_deg, ang);
                angle_sum += ang;
                ++angle_count;
            }

            // Aspect ratio: longest_edge / min_altitude.
            // min_altitude = 2 * area / longest_edge  ⇒  ratio = longest_edge² / (2 * area)
            const float max_edge = std::max({len_a, len_b, len_c});
            const float area     = area2 * 0.5f;
            const float ratio    = max_edge * max_edge / (2.0f * area);
            report.max_aspect_ratio = std::max(report.max_aspect_ratio, ratio);
        }

        // Count edge valence.
        edge_count[edgeKey(f.v[0], f.v[1])]++;
        edge_count[edgeKey(f.v[1], f.v[2])]++;
        edge_count[edgeKey(f.v[2], f.v[0])]++;
    }

    if (angle_count > 0) {
        report.mean_interior_angle_deg = angle_sum / static_cast<float>(angle_count);
    }

    // Open boundary edges: edges shared by exactly one triangle.
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            ++report.open_boundary_edge_count;
        } else if (count > 2) {
            ++report.non_manifold_edge_count;
        }
    }

    return report;
}

bool MeshQualityVerifier::passes(const MeshQualityReport& report,
                                  const MeshQualityThresholds& thresholds) {
    if (report.degenerate_facet_count > thresholds.max_degenerate_facets) return false;
    if (report.min_interior_angle_deg < thresholds.min_interior_angle_deg) return false;
    if (report.max_aspect_ratio       > thresholds.max_aspect_ratio)       return false;
    return true;
}

} // namespace liteaerosim::environment
