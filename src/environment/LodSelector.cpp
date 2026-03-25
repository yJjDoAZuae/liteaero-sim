#include "environment/LodSelector.hpp"
#include <cmath>

namespace liteaerosim::environment {

using namespace liteaero::terrain;

// ---------------------------------------------------------------------------
// WGS84 constants (duplicated locally to keep this file self-contained)
// ---------------------------------------------------------------------------
static constexpr double kWgs84A  = 6378137.0;
static constexpr double kWgs84E2 = 6.69437999014e-3;

static double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
}

static void geodeticToEcef(double lat, double lon, double h,
                            double& X, double& Y, double& Z) {
    const double N  = primeVerticalRadius(lat);
    const double cl = std::cos(lat), sl = std::sin(lat);
    const double co = std::cos(lon), so = std::sin(lon);
    X = (N + h) * cl * co;
    Y = (N + h) * cl * so;
    Z = (N * (1.0 - kWgs84E2) + h) * sl;
}

// ---------------------------------------------------------------------------
// Nominal LOD from slant range.
// L0: r < r0;  L1: r0 ≤ r < 3·r0;  L2: 3·r0 ≤ r < 9·r0; ...
// ---------------------------------------------------------------------------
static int nominalLodFromRange(float r, float r0) {
    if (r < r0) return 0;
    int   lod      = 1;
    float boundary = r0;  // L0/L1 boundary
    while (lod < 6 && r >= boundary * 3.0f) {
        boundary *= 3.0f;
        ++lod;
    }
    return lod;
}

// Upper slant-range boundary between lod and lod+1: r0 × 3^lod.
static float lodUpperBoundary(int lod, float r0) {
    float b = r0;
    for (int i = 0; i < lod; ++i) b *= 3.0f;
    return b;
}

// ---------------------------------------------------------------------------
// LodSelector
// ---------------------------------------------------------------------------

LodSelector::LodSelector(LodSelectorConfig config) : config_(config) {}

void LodSelector::reset() {
    committed_lod_.clear();
}

std::vector<TileRef> LodSelector::select(const TerrainMesh& mesh,
                                          double observer_lat_rad,
                                          double observer_lon_rad,
                                          double observer_alt_m)
{
    auto refs = mesh.selectLodBySlantRange(observer_lat_rad, observer_lon_rad, observer_alt_m);

    // Observer ECEF.
    double Xo, Yo, Zo;
    geodeticToEcef(observer_lat_rad, observer_lon_rad, observer_alt_m, Xo, Yo, Zo);

    for (auto& ref : refs) {
        // Slant range to cell centroid.
        const GeodeticAABB& b = ref.cell->bounds();
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const double h_c   = static_cast<double>(b.height_min_m + b.height_max_m) * 0.5;
        double Xc, Yc, Zc;
        geodeticToEcef(lat_c, lon_c, h_c, Xc, Yc, Zc);
        const float r = static_cast<float>(
            std::sqrt((Xo-Xc)*(Xo-Xc) + (Yo-Yc)*(Yo-Yc) + (Zo-Zc)*(Zo-Zc)));

        const int nominal = nominalLodFromRange(r, config_.r0_m);
        const uint64_t key = reinterpret_cast<uint64_t>(ref.cell);

        auto it = committed_lod_.find(key);
        if (it == committed_lod_.end()) {
            // First call for this cell: use nominal directly.
            committed_lod_[key] = static_cast<TerrainLod>(nominal);
            ref.lod = static_cast<TerrainLod>(nominal);
        } else {
            int committed = static_cast<int>(it->second);
            const float delta = config_.hysteresis_fraction_nd;

            if (nominal > committed) {
                // Want coarser; check against upper boundary of current LOD.
                const float r_b = lodUpperBoundary(committed, config_.r0_m);
                if (r > r_b * (1.0f + delta)) {
                    committed = nominal;
                    it->second = static_cast<TerrainLod>(committed);
                }
            } else if (nominal < committed) {
                // Want finer; check against lower boundary of current LOD.
                const float r_b = lodUpperBoundary(committed - 1, config_.r0_m);
                if (r < r_b * (1.0f - delta)) {
                    committed = nominal;
                    it->second = static_cast<TerrainLod>(committed);
                }
            }
            ref.lod = static_cast<TerrainLod>(committed);
        }
    }
    return refs;
}

} // namespace liteaerosim::environment
