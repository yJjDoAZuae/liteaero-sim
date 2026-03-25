#include "environment/TerrainCell.hpp"
#include <stdexcept>

namespace liteaerosim::environment {

using namespace liteaero::terrain;

void TerrainCell::addTile(TerrainTile tile) {
    const int idx = static_cast<int>(tile.lod());
    tiles_[idx] = std::move(tile);
}

bool TerrainCell::hasLod(TerrainLod lod) const {
    return tiles_[static_cast<int>(lod)].has_value();
}

const TerrainTile& TerrainCell::tile(TerrainLod lod) const {
    const auto& opt = tiles_[static_cast<int>(lod)];
    if (!opt.has_value()) {
        throw std::out_of_range("TerrainCell::tile: requested LOD not present");
    }
    return *opt;
}

TerrainLod TerrainCell::finestAvailableLod() const {
    for (int i = 0; i < kLodCount; ++i) {
        if (tiles_[i].has_value()) {
            return static_cast<TerrainLod>(i);
        }
    }
    throw std::logic_error("TerrainCell::finestAvailableLod: cell is empty");
}

TerrainLod TerrainCell::coarsestAvailableLod() const {
    for (int i = kLodCount - 1; i >= 0; --i) {
        if (tiles_[i].has_value()) {
            return static_cast<TerrainLod>(i);
        }
    }
    throw std::logic_error("TerrainCell::coarsestAvailableLod: cell is empty");
}

const GeodeticAABB& TerrainCell::bounds() const {
    for (const auto& opt : tiles_) {
        if (opt.has_value()) {
            return opt->bounds();
        }
    }
    throw std::logic_error("TerrainCell::bounds: cell is empty");
}

} // namespace liteaerosim::environment
