#pragma once
#include <liteaero/terrain/GeodeticAABB.hpp>
#include <liteaero/terrain/TerrainTile.hpp>
#include <array>
#include <optional>
#include <stdexcept>

namespace liteaerosim::environment {

class TerrainCell {
public:
    void addTile(liteaero::terrain::TerrainTile tile);
    bool hasLod(liteaero::terrain::TerrainLod lod) const;
    const liteaero::terrain::TerrainTile& tile(liteaero::terrain::TerrainLod lod) const;  // throws std::out_of_range if absent
    liteaero::terrain::TerrainLod finestAvailableLod()   const;
    liteaero::terrain::TerrainLod coarsestAvailableLod() const;

    // Geographic bounds of the cell (union of all tile bounds — identical per cell).
    const liteaero::terrain::GeodeticAABB& bounds() const;

private:
    static constexpr int kLodCount = 7;
    std::array<std::optional<liteaero::terrain::TerrainTile>, kLodCount> tiles_;  // indexed by TerrainLod
};

} // namespace liteaerosim::environment
