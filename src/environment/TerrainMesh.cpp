#define _USE_MATH_DEFINES
#include "environment/TerrainMesh.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>

namespace liteaerosim::environment {

// ---------------------------------------------------------------------------
// WGS84 constants
// ---------------------------------------------------------------------------
static constexpr double kWgs84A       = 6378137.0;       // semi-major axis (m)
static constexpr double kWgs84E2      = 6.69437999014e-3; // first eccentricity squared
static constexpr double kEarthMeanR   = 6371000.0;        // mean radius for cell extent (m)
static constexpr int    kLodCount     = 7;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Metric cell side per LOD level (m).  L4–L6 share 100 km.
static constexpr double kCellSideM[kLodCount] = {
    1000.0,    // L0
    3000.0,    // L1
    10000.0,   // L2
    30000.0,   // L3
    100000.0,  // L4
    100000.0,  // L5
    100000.0,  // L6
};

double TerrainMesh::cellExtentRad(TerrainLod lod) {
    return kCellSideM[static_cast<int>(lod)] / kEarthMeanR;
}

uint64_t TerrainMesh::cellKey(double lat_rad, double lon_rad, TerrainLod lod) {
    const double extent = cellExtentRad(lod);
    const auto lat_idx = static_cast<uint64_t>(std::floor((lat_rad + M_PI / 2.0) / extent));
    const auto lon_idx = static_cast<uint64_t>(std::floor((lon_rad + M_PI)       / extent));
    return (lat_idx << 48) | (lon_idx << 32) | (static_cast<uint64_t>(static_cast<int>(lod)) << 16);
}

// Prime-vertical radius of curvature N(lat).
static double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
}

// Geodetic (lat, lon, h) → ECEF (X, Y, Z).
static void geodeticToEcef(double lat, double lon, double h,
                            double& X, double& Y, double& Z) {
    const double N  = primeVerticalRadius(lat);
    const double cl = std::cos(lat), sl = std::sin(lat);
    const double co = std::cos(lon), so = std::sin(lon);
    X = (N + h) * cl * co;
    Y = (N + h) * cl * so;
    Z = (N * (1.0 - kWgs84E2) + h) * sl;
}

// ECEF offset dXYZ → ENU components at reference (lat, lon).
//   east  = R_ENU(0,:) · dXYZ
//   north = R_ENU(1,:) · dXYZ
//   up    = R_ENU(2,:) · dXYZ
static void ecefOffsetToEnu(double lat, double lon,
                             double dX, double dY, double dZ,
                             double& east, double& north, double& up) {
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    east  = -so * dX  +  co * dY;
    north = -sl * co * dX  - sl * so * dY  + cl * dZ;
    up    =  cl * co * dX  + cl * so * dY  + sl * dZ;
}

// ENU (east, north, up) → ECEF offset dXYZ at reference (lat, lon).
static void enuToEcefOffset(double lat, double lon,
                             double east, double north, double up,
                             double& dX, double& dY, double& dZ) {
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    dX = -so * east  + (-sl * co) * north  + (cl * co) * up;
    dY =  co * east  + (-sl * so) * north  + (cl * so) * up;
    dZ =               cl * north           + sl * up;
}

// Select the finest LOD level that is >= max_lod and present in cell.
// Returns empty if no acceptable tile exists.
static std::optional<TerrainLod> selectTileLod(const TerrainCell& cell, TerrainLod max_lod) {
    for (int i = static_cast<int>(max_lod); i < kLodCount; ++i) {
        const auto lod = static_cast<TerrainLod>(i);
        if (cell.hasLod(lod)) return lod;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Cell management
// ---------------------------------------------------------------------------

void TerrainMesh::addCell(TerrainCell cell) {
    const GeodeticAABB& b = cell.bounds();
    const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
    const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
    const TerrainLod lod = cell.finestAvailableLod();
    cells_[cellKey(lat_c, lon_c, lod)] = std::move(cell);
}

const TerrainCell* TerrainMesh::cellAt(double lat_rad, double lon_rad) const {
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();
        if (lat_rad >= b.lat_min_rad && lat_rad <= b.lat_max_rad &&
            lon_rad >= b.lon_min_rad && lon_rad <= b.lon_max_rad) {
            return &cell;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// V_Terrain — elevation_m
// ---------------------------------------------------------------------------

float TerrainMesh::elevation_m(double lat_rad, double lon_rad) const {
    const TerrainCell* cell = cellAt(lat_rad, lon_rad);
    if (!cell) return 0.f;

    const TerrainTile& tile = cell->tile(cell->finestAvailableLod());
    const GeodeticPoint& centroid = tile.centroid();

    // Convert query point to ENU offset from tile centroid.
    double Xc, Yc, Zc, Xq, Yq, Zq;
    geodeticToEcef(centroid.latitude_rad, centroid.longitude_rad,
                   static_cast<double>(centroid.height_wgs84_m), Xc, Yc, Zc);
    geodeticToEcef(lat_rad, lon_rad,
                   static_cast<double>(centroid.height_wgs84_m), Xq, Yq, Zq);

    double east_q, north_q, up_q;
    ecefOffsetToEnu(centroid.latitude_rad, centroid.longitude_rad,
                    Xq - Xc, Yq - Yc, Zq - Zc,
                    east_q, north_q, up_q);

    const float ep = static_cast<float>(east_q);
    const float np = static_cast<float>(north_q);

    // Find containing facet via 2D barycentric test in the ENU east/north plane.
    const auto& verts  = tile.vertices();
    const auto& facets = tile.facets();

    for (const auto& f : facets) {
        const float e0 = verts[f.v[0]].east_m,  n0 = verts[f.v[0]].north_m;
        const float e1 = verts[f.v[1]].east_m,  n1 = verts[f.v[1]].north_m;
        const float e2 = verts[f.v[2]].east_m,  n2 = verts[f.v[2]].north_m;

        const float denom = (n1 - n2) * (e0 - e2) + (e2 - e1) * (n0 - n2);
        if (std::abs(denom) < 1e-10f) continue;  // degenerate facet

        const float u = ((n1 - n2) * (ep - e2) + (e2 - e1) * (np - n2)) / denom;
        const float v = ((n2 - n0) * (ep - e2) + (e0 - e2) * (np - n2)) / denom;
        const float w = 1.f - u - v;

        if (u >= 0.f && v >= 0.f && w >= 0.f) {
            return centroid.height_wgs84_m
                 + u * verts[f.v[0]].up_m
                 + v * verts[f.v[1]].up_m
                 + w * verts[f.v[2]].up_m;
        }
    }
    return 0.f;  // query point within cell bounds but outside all facets
}

// ---------------------------------------------------------------------------
// Coordinate transforms
// ---------------------------------------------------------------------------

std::vector<std::array<double, 3>> TerrainMesh::toECEF(const TerrainTile& tile) const {
    const GeodeticPoint& centroid = tile.centroid();
    double Xc, Yc, Zc;
    geodeticToEcef(centroid.latitude_rad, centroid.longitude_rad,
                   static_cast<double>(centroid.height_wgs84_m), Xc, Yc, Zc);

    std::vector<std::array<double, 3>> result;
    result.reserve(tile.vertices().size());
    for (const auto& v : tile.vertices()) {
        double dX, dY, dZ;
        enuToEcefOffset(centroid.latitude_rad, centroid.longitude_rad,
                        static_cast<double>(v.east_m),
                        static_cast<double>(v.north_m),
                        static_cast<double>(v.up_m),
                        dX, dY, dZ);
        result.push_back({Xc + dX, Yc + dY, Zc + dZ});
    }
    return result;
}

std::vector<std::array<float, 3>> TerrainMesh::toNED(
    const TerrainTile& tile,
    double ref_lat_rad, double ref_lon_rad, double ref_alt_m) const
{
    double Xr, Yr, Zr;
    geodeticToEcef(ref_lat_rad, ref_lon_rad, ref_alt_m, Xr, Yr, Zr);

    const double sl0 = std::sin(ref_lat_rad), cl0 = std::cos(ref_lat_rad);
    const double so0 = std::sin(ref_lon_rad), co0 = std::cos(ref_lon_rad);

    const auto ecef = toECEF(tile);

    std::vector<std::array<float, 3>> result;
    result.reserve(ecef.size());
    for (const auto& p : ecef) {
        const double dX = p[0] - Xr, dY = p[1] - Yr, dZ = p[2] - Zr;
        // R_NED * [dX; dY; dZ]
        const double N = -sl0 * co0 * dX  - sl0 * so0 * dY  + cl0 * dZ;
        const double E = -so0 * dX         + co0 * dY;
        const double D = -cl0 * co0 * dX  - cl0 * so0 * dY  - sl0 * dZ;
        result.push_back({static_cast<float>(N),
                          static_cast<float>(E),
                          static_cast<float>(D)});
    }
    return result;
}

// ---------------------------------------------------------------------------
// Spatial queries
// ---------------------------------------------------------------------------

std::vector<TileRef> TerrainMesh::queryGeodeticAABB(
    const GeodeticAABB& query, TerrainLod max_lod) const
{
    std::vector<TileRef> refs;
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();
        if (b.lat_max_rad    < query.lat_min_rad ||
            b.lat_min_rad    > query.lat_max_rad) continue;
        if (b.lon_max_rad    < query.lon_min_rad ||
            b.lon_min_rad    > query.lon_max_rad) continue;
        if (b.height_max_m   < query.height_min_m ||
            b.height_min_m   > query.height_max_m) continue;

        const auto lod = selectTileLod(cell, max_lod);
        if (lod) refs.push_back({&cell, *lod});
    }
    return refs;
}

std::vector<TileRef> TerrainMesh::querySphere(
    double center_lat_rad, double center_lon_rad,
    float  center_height_m, float  radius_m,
    TerrainLod max_lod) const
{
    double Xs, Ys, Zs;
    geodeticToEcef(center_lat_rad, center_lon_rad,
                   static_cast<double>(center_height_m), Xs, Ys, Zs);

    std::vector<TileRef> refs;
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();

        // Cell centroid in ECEF.
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const double h_c   = (static_cast<double>(b.height_min_m) + b.height_max_m) * 0.5;
        double Xc, Yc, Zc;
        geodeticToEcef(lat_c, lon_c, h_c, Xc, Yc, Zc);

        // Conservative sphere–AABB test: use distance to centroid plus a
        // half-diagonal estimate from one corner to the centroid.
        double Xk, Yk, Zk;
        geodeticToEcef(b.lat_min_rad, b.lon_min_rad,
                       static_cast<double>(b.height_min_m), Xk, Yk, Zk);
        const double half_diag = std::sqrt((Xk - Xc) * (Xk - Xc) +
                                            (Yk - Yc) * (Yk - Yc) +
                                            (Zk - Zc) * (Zk - Zc));
        const double dist = std::sqrt((Xc - Xs) * (Xc - Xs) +
                                       (Yc - Ys) * (Yc - Ys) +
                                       (Zc - Zs) * (Zc - Zs));
        if (dist > static_cast<double>(radius_m) + half_diag) continue;

        const auto lod = selectTileLod(cell, max_lod);
        if (lod) refs.push_back({&cell, *lod});
    }
    return refs;
}

std::vector<TileRef> TerrainMesh::queryLocalAABB(
    double           center_lat_rad,
    double           center_lon_rad,
    float            center_height_m,
    const LocalAABB& aabb,
    TerrainLod       max_lod) const
{
    double Xq, Yq, Zq;
    geodeticToEcef(center_lat_rad, center_lon_rad,
                   static_cast<double>(center_height_m), Xq, Yq, Zq);

    std::vector<TileRef> refs;
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();

        // Cell centroid in ECEF.
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const double h_c   = (static_cast<double>(b.height_min_m) + b.height_max_m) * 0.5;
        double Xc, Yc, Zc;
        geodeticToEcef(lat_c, lon_c, h_c, Xc, Yc, Zc);

        // Project cell centroid offset into the query center's ENU frame.
        double east_off, north_off, up_off;
        ecefOffsetToEnu(center_lat_rad, center_lon_rad,
                        Xc - Xq, Yc - Yq, Zc - Zq,
                        east_off, north_off, up_off);

        // Cell metric half-extents (approximate with local radii).
        // Meridional radius M ≈ a*(1-e²) / (1 - e²sin²φ)^(3/2) ; use a*(1-e²) as lower bound.
        // Prime vertical east radius: N*cos(lat) ≈ a*cos(lat).
        const double lat_half_rad = (b.lat_max_rad - b.lat_min_rad) * 0.5;
        const double lon_half_rad = (b.lon_max_rad - b.lon_min_rad) * 0.5;
        const double R_north = kWgs84A * (1.0 - kWgs84E2);          // conservative meridional
        const double R_east  = kWgs84A * std::cos(lat_c);            // prime vertical * cos(lat)
        const double cell_north_half = lat_half_rad * R_north;
        const double cell_east_half  = lon_half_rad * R_east;

        // Overlap test: |offset| <= cell_half + query_half_extent
        if (std::abs(north_off) > cell_north_half + static_cast<double>(aabb.half_extent_north_m)) continue;
        if (std::abs(east_off)  > cell_east_half  + static_cast<double>(aabb.half_extent_east_m))  continue;

        // Height overlap.
        if (b.height_max_m < aabb.height_min_m ||
            b.height_min_m > aabb.height_max_m) continue;

        const auto lod = selectTileLod(cell, max_lod);
        if (lod) refs.push_back({&cell, *lod});
    }
    return refs;
}

// ---------------------------------------------------------------------------
// Not yet implemented (Steps 7, 8, 10, 11)
// ---------------------------------------------------------------------------

std::vector<TileRef> TerrainMesh::selectLodBySlantRange(
    double, double, double) const
{
    throw std::runtime_error("TerrainMesh::selectLodBySlantRange: not implemented until Step 7");
}

bool TerrainMesh::lineOfSight(double, double, float, double, double, float) const {
    throw std::runtime_error("TerrainMesh::lineOfSight: not implemented until Step 8");
}

nlohmann::json TerrainMesh::serializeJson() const {
    throw std::runtime_error("TerrainMesh::serializeJson: not implemented until Step 10");
}

void TerrainMesh::deserializeJson(const nlohmann::json&) {
    throw std::runtime_error("TerrainMesh::deserializeJson: not implemented until Step 10");
}

std::vector<uint8_t> TerrainMesh::serializeProto() const {
    throw std::runtime_error("TerrainMesh::serializeProto: not implemented until Step 10");
}

void TerrainMesh::deserializeProto(const std::vector<uint8_t>&) {
    throw std::runtime_error("TerrainMesh::deserializeProto: not implemented until Step 10");
}

void TerrainMesh::exportGltf(const std::filesystem::path&, TerrainLod, const GeodeticPoint*) const {
    throw std::runtime_error("TerrainMesh::exportGltf: not implemented until Step 11");
}

} // namespace liteaerosim::environment
