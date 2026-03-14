#pragma once
#include "environment/GeodeticPoint.hpp"
#include "environment/LocalAABB.hpp"
#include "environment/TerrainCell.hpp"
#include "environment/Terrain.hpp"
#include <nlohmann/json.hpp>
#include <array>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace liteaerosim::environment {

// Reference returned by spatial query methods.
// `lod` is the selected LOD level for this cell (respecting the max_lod filter).
struct TileRef {
    const TerrainCell* cell;
    TerrainLod         lod;
};

class TerrainMesh : public V_Terrain {
public:
    // V_Terrain — barycentric interpolation of the finest available LOD tile.
    [[nodiscard]] float elevation_m(double latitude_rad,
                                    double longitude_rad) const override;

    // ---------------------------------------------------------------------------
    // Cell management
    // ---------------------------------------------------------------------------

    // Add a cell to the mesh.  Single-threaded load phase only; no concurrent writes.
    void addCell(TerrainCell cell);

    // Return a pointer to the cell whose geographic bounds contain (lat, lon),
    // or nullptr if no such cell exists.
    [[nodiscard]] const TerrainCell* cellAt(double lat_rad, double lon_rad) const;

    // ---------------------------------------------------------------------------
    // Coordinate transforms
    // ---------------------------------------------------------------------------

    // Returns the ECEF position (X, Y, Z in meters) of every vertex in tile.
    [[nodiscard]] std::vector<std::array<double, 3>>
        toECEF(const TerrainTile& tile) const;

    // Returns the NED displacement (N, E, D in meters) of every vertex in tile
    // relative to the supplied geodetic reference position.
    [[nodiscard]] std::vector<std::array<float, 3>>
        toNED(const TerrainTile& tile,
              double ref_lat_rad, double ref_lon_rad, double ref_alt_m) const;

    // ---------------------------------------------------------------------------
    // Spatial queries
    // ---------------------------------------------------------------------------
    // max_lod acts as a minimum-coarseness filter: tiles finer than max_lod are
    // excluded.  Among acceptable tiles (lod >= max_lod), the finest is returned.
    // Example: max_lod = L0_Finest returns L0 tiles only.
    //          max_lod = L6_Coarsest returns only L6 tiles if available.

    // Primary simulation interface — metric extents in the local tangent plane.
    [[nodiscard]] std::vector<TileRef>
        queryLocalAABB(double           center_lat_rad,
                       double           center_lon_rad,
                       float            center_height_m,
                       const LocalAABB& aabb,
                       TerrainLod       max_lod) const;

    // Ingestion / data-management interface — geodetic rectangle.
    [[nodiscard]] std::vector<TileRef>
        queryGeodeticAABB(const GeodeticAABB& aabb, TerrainLod max_lod) const;

    // Spherical neighborhood query — primary interface for sensor models.
    [[nodiscard]] std::vector<TileRef>
        querySphere(double center_lat_rad, double center_lon_rad,
                    float  center_height_m, float  radius_m,
                    TerrainLod max_lod) const;

    // Returns one TileRef per cell with LOD selected by slant-range rule.
    // Not yet implemented (Step 7).
    [[nodiscard]] std::vector<TileRef>
        selectLodBySlantRange(double observer_lat_rad,
                              double observer_lon_rad,
                              double observer_alt_m) const;

    // ---------------------------------------------------------------------------
    // Line-of-sight (Step 8)
    // ---------------------------------------------------------------------------

    [[nodiscard]] bool lineOfSight(double lat1_rad, double lon1_rad, float h1_m,
                                   double lat2_rad, double lon2_rad, float h2_m) const;

    // ---------------------------------------------------------------------------
    // Game engine export (Step 11)
    // ---------------------------------------------------------------------------

    void exportGltf(const std::filesystem::path& output_path,
                    TerrainLod                   max_lod,
                    const GeodeticPoint*         world_origin = nullptr) const;

    // ---------------------------------------------------------------------------
    // Serialization (Step 10)
    // ---------------------------------------------------------------------------

    [[nodiscard]] nlohmann::json       serializeJson()                              const;
    void                               deserializeJson(const nlohmann::json&        j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                             const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    // Spatial index keyed by encoded (lat_idx, lon_idx, lod).
    // See §Internal Spatial Index in docs/architecture/terrain.md.
    std::unordered_map<uint64_t, TerrainCell> cells_;

    static uint64_t  cellKey(double lat_rad, double lon_rad, TerrainLod lod);
    static double    cellExtentRad(TerrainLod lod);
};

} // namespace liteaerosim::environment
