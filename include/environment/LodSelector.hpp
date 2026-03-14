#pragma once
#include "environment/TerrainMesh.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace liteaerosim::environment {

struct LodSelectorConfig {
    float hysteresis_fraction_nd = 0.15f;  // δ — dead-band half-width fraction
    float r0_m                   = 300.f;  // nominal L0/L1 slant-range boundary
};

// Wraps TerrainMesh::selectLodBySlantRange with per-cell hysteresis state.
// The TerrainMesh remains const-queryable; all mutable state lives here.
class LodSelector {
public:
    explicit LodSelector(LodSelectorConfig config = {});

    // Returns one TileRef per cell with LOD selected by slant-range + hysteresis.
    // Non-const because committed_lod_ is updated on each call.
    [[nodiscard]] std::vector<TileRef> select(const TerrainMesh& mesh,
                                              double observer_lat_rad,
                                              double observer_lon_rad,
                                              double observer_alt_m);

    // Clear all committed-LOD state; next select() call behaves as if first call.
    void reset();

private:
    LodSelectorConfig                        config_;
    // Key: reinterpret_cast<uint64_t>(TerrainCell*) — stable for the mesh lifetime.
    std::unordered_map<uint64_t, TerrainLod> committed_lod_;
};

} // namespace liteaerosim::environment
