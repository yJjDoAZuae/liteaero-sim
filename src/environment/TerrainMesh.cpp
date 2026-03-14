#define _USE_MATH_DEFINES
#include "environment/TerrainMesh.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <cstring>
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
// Step 7 — selectLodBySlantRange (nominal formula, no hysteresis)
// ---------------------------------------------------------------------------

// Nominal LOD from slant range r (meters) with L0/L1 boundary at r0.
// L0: r < r0;  L1: r0 ≤ r < 3·r0;  L2: 3·r0 ≤ r < 9·r0; ...
static int nominalLod(float r, float r0 = 300.f) {
    if (r < r0) return 0;
    int   lod      = 1;
    float boundary = r0;
    while (lod < kLodCount - 1 && r >= boundary * 3.0f) {
        boundary *= 3.0f;
        ++lod;
    }
    return lod;
}

std::vector<TileRef> TerrainMesh::selectLodBySlantRange(
    double observer_lat_rad, double observer_lon_rad, double observer_alt_m) const
{
    double Xo, Yo, Zo;
    geodeticToEcef(observer_lat_rad, observer_lon_rad, observer_alt_m, Xo, Yo, Zo);

    std::vector<TileRef> refs;
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const double h_c   = static_cast<double>(b.height_min_m + b.height_max_m) * 0.5;
        double Xc, Yc, Zc;
        geodeticToEcef(lat_c, lon_c, h_c, Xc, Yc, Zc);
        const float r = static_cast<float>(
            std::sqrt((Xo-Xc)*(Xo-Xc) + (Yo-Yc)*(Yo-Yc) + (Zo-Zc)*(Zo-Zc)));

        const int target_lod = nominalLod(r);
        const auto lod = selectTileLod(cell, static_cast<TerrainLod>(target_lod));
        if (lod) refs.push_back({&cell, *lod});
    }
    return refs;
}

// ---------------------------------------------------------------------------
// Step 8 — lineOfSight (Möller–Trumbore in ECEF)
// ---------------------------------------------------------------------------

// Returns true if ray (orig + t*dir, t in [t_min, t_max]) intersects triangle
// (v0, v1, v2).  Uses Möller–Trumbore algorithm.
static bool rayTriangleIntersect(
    const std::array<double, 3>& orig,
    const std::array<double, 3>& dir,
    const std::array<double, 3>& v0,
    const std::array<double, 3>& v1,
    const std::array<double, 3>& v2,
    double t_min, double t_max)
{
    const double e1[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
    const double e2[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};

    const double h[3] = {
        dir[1]*e2[2] - dir[2]*e2[1],
        dir[2]*e2[0] - dir[0]*e2[2],
        dir[0]*e2[1] - dir[1]*e2[0],
    };
    const double a = e1[0]*h[0] + e1[1]*h[1] + e1[2]*h[2];
    if (std::abs(a) < 1e-10) return false;  // parallel or degenerate

    const double f = 1.0 / a;
    const double s[3] = {orig[0]-v0[0], orig[1]-v0[1], orig[2]-v0[2]};
    const double u = f * (s[0]*h[0] + s[1]*h[1] + s[2]*h[2]);
    if (u < 0.0 || u > 1.0) return false;

    const double q[3] = {
        s[1]*e1[2] - s[2]*e1[1],
        s[2]*e1[0] - s[0]*e1[2],
        s[0]*e1[1] - s[1]*e1[0],
    };
    const double v = f * (dir[0]*q[0] + dir[1]*q[1] + dir[2]*q[2]);
    if (v < 0.0 || u + v > 1.0) return false;

    const double t = f * (e2[0]*q[0] + e2[1]*q[1] + e2[2]*q[2]);
    return t >= t_min && t <= t_max;
}

bool TerrainMesh::lineOfSight(double lat1_rad, double lon1_rad, float h1_m,
                               double lat2_rad, double lon2_rad, float h2_m) const
{
    double X1, Y1, Z1, X2, Y2, Z2;
    geodeticToEcef(lat1_rad, lon1_rad, static_cast<double>(h1_m), X1, Y1, Z1);
    geodeticToEcef(lat2_rad, lon2_rad, static_cast<double>(h2_m), X2, Y2, Z2);

    const std::array<double, 3> orig = {X1, Y1, Z1};
    const std::array<double, 3> dir  = {X2-X1, Y2-Y1, Z2-Z1};
    const double seg_len = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);

    // Candidate tiles: sphere at midpoint, radius = half segment + 200 m margin.
    const double mid_lat = (lat1_rad + lat2_rad) * 0.5;
    const double mid_lon = (lon1_rad + lon2_rad) * 0.5;
    const float  mid_h   = (h1_m + h2_m) * 0.5f;
    const float  radius  = static_cast<float>(seg_len * 0.5 + 200.0);

    const auto refs = querySphere(mid_lat, mid_lon, mid_h, radius, TerrainLod::L0_Finest);

    for (const auto& ref : refs) {
        const TerrainTile& tile = ref.cell->tile(ref.lod);
        const auto ecef_verts = toECEF(tile);
        for (const auto& f : tile.facets()) {
            if (rayTriangleIntersect(orig, dir,
                                     ecef_verts[f.v[0]],
                                     ecef_verts[f.v[1]],
                                     ecef_verts[f.v[2]],
                                     1e-6, 1.0 - 1e-6)) {
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Step 10 — Serialization: JSON
// ---------------------------------------------------------------------------

nlohmann::json TerrainMesh::serializeJson() const {
    nlohmann::json j;
    j["schema_version"] = 1;
    nlohmann::json tiles_array = nlohmann::json::array();
    for (const auto& [key, cell] : cells_) {
        for (int lod_i = 0; lod_i < kLodCount; ++lod_i) {
            const auto lod = static_cast<TerrainLod>(lod_i);
            if (cell.hasLod(lod)) {
                tiles_array.push_back(cell.tile(lod).serializeJson());
            }
        }
    }
    j["tiles"] = std::move(tiles_array);
    return j;
}

void TerrainMesh::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("TerrainMesh::deserializeJson: unsupported schema_version");
    }
    cells_.clear();
    for (const auto& jt : j.at("tiles")) {
        TerrainTile tile = TerrainTile::deserializeJson(jt);
        const GeodeticAABB& b = tile.bounds();
        const TerrainLod lod = tile.lod();

        // Find or create a cell for this tile's bounds centroid at this LOD.
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const uint64_t key = cellKey(lat_c, lon_c, lod);

        // If an existing cell with compatible bounds exists, add the tile to it.
        // Otherwise create a new cell.
        auto it = cells_.find(key);
        if (it != cells_.end()) {
            it->second.addTile(std::move(tile));
        } else {
            TerrainCell cell;
            cell.addTile(std::move(tile));
            cells_[key] = std::move(cell);
        }
    }
}

// ---------------------------------------------------------------------------
// Step 10 — Serialization: Proto
// ---------------------------------------------------------------------------

std::vector<uint8_t> TerrainMesh::serializeProto() const {
    las_proto::TerrainMeshProto proto;
    proto.set_schema_version(1);
    for (const auto& [key, cell] : cells_) {
        for (int lod_i = 0; lod_i < kLodCount; ++lod_i) {
            const auto lod = static_cast<TerrainLod>(lod_i);
            if (!cell.hasLod(lod)) continue;
            const auto tile_bytes = cell.tile(lod).serializeProto();
            las_proto::TerrainTileProto tile_proto;
            tile_proto.ParseFromArray(tile_bytes.data(), static_cast<int>(tile_bytes.size()));
            *proto.add_tiles() = tile_proto;
        }
    }
    std::string buf;
    proto.SerializeToString(&buf);
    return {buf.begin(), buf.end()};
}

void TerrainMesh::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::TerrainMeshProto proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("TerrainMesh::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error("TerrainMesh::deserializeProto: unsupported schema_version");
    }
    cells_.clear();
    for (const auto& tile_proto : proto.tiles()) {
        std::string buf;
        tile_proto.SerializeToString(&buf);
        const std::vector<uint8_t> tile_bytes(buf.begin(), buf.end());
        TerrainTile tile = TerrainTile::deserializeProto(tile_bytes);

        const GeodeticAABB& b = tile.bounds();
        const TerrainLod lod = tile.lod();
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const uint64_t key = cellKey(lat_c, lon_c, lod);

        auto it = cells_.find(key);
        if (it != cells_.end()) {
            it->second.addTile(std::move(tile));
        } else {
            TerrainCell cell;
            cell.addTile(std::move(tile));
            cells_[key] = std::move(cell);
        }
    }
}

// ---------------------------------------------------------------------------
// Step 10 — Serialization: .las_terrain binary format
//
// Layout (little-endian):
//   [0..3]   uint32  magic = 0x4C415354 ("LAST")
//   [4..7]   uint32  format_version = 1
//   [8..11]  uint32  tile_count
//   Per tile:
//     uint32  metadata_json_length
//     [N bytes]  metadata JSON (UTF-8)
//     uint32  vertex_count
//     [V × 12 bytes]  vertices: float32 east, north, up
//     uint32  facet_count
//     [F × 15 bytes]  facets: uint32 v0, uint32 v1, uint32 v2, uint8 r, g, b
// ---------------------------------------------------------------------------

static void writeU32LE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v        & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >>  8) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

static void writeF32LE(std::vector<uint8_t>& buf, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    writeU32LE(buf, bits);
}

static uint32_t readU32LE(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])        |
           (static_cast<uint32_t>(p[1]) <<  8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

static float readF32LE(const uint8_t* p) {
    const uint32_t bits = readU32LE(p);
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
}

std::vector<uint8_t> TerrainMesh::serializeLasTerrain() const {
    static constexpr uint32_t kMagic         = 0x4C415354u;
    static constexpr uint32_t kFormatVersion = 1u;

    // Count tiles.
    uint32_t tile_count = 0;
    for (const auto& [key, cell] : cells_) {
        for (int i = 0; i < kLodCount; ++i) {
            if (cell.hasLod(static_cast<TerrainLod>(i))) ++tile_count;
        }
    }

    std::vector<uint8_t> buf;
    buf.reserve(64 + tile_count * 256);

    writeU32LE(buf, kMagic);
    writeU32LE(buf, kFormatVersion);
    writeU32LE(buf, tile_count);

    for (const auto& [key, cell] : cells_) {
        for (int lod_i = 0; lod_i < kLodCount; ++lod_i) {
            const auto lod = static_cast<TerrainLod>(lod_i);
            if (!cell.hasLod(lod)) continue;
            const TerrainTile& tile = cell.tile(lod);

            // Metadata JSON (everything except vertex/facet data).
            nlohmann::json meta;
            meta["schema_version"]    = 1;
            meta["lod"]               = static_cast<int>(tile.lod());
            meta["centroid"] = {
                {"latitude_rad",   tile.centroid().latitude_rad},
                {"longitude_rad",  tile.centroid().longitude_rad},
                {"height_wgs84_m", tile.centroid().height_wgs84_m},
            };
            meta["bounds"] = {
                {"lat_min_rad",  tile.bounds().lat_min_rad},
                {"lat_max_rad",  tile.bounds().lat_max_rad},
                {"lon_min_rad",  tile.bounds().lon_min_rad},
                {"lon_max_rad",  tile.bounds().lon_max_rad},
                {"height_min_m", tile.bounds().height_min_m},
                {"height_max_m", tile.bounds().height_max_m},
            };
            const std::string meta_str = meta.dump();
            writeU32LE(buf, static_cast<uint32_t>(meta_str.size()));
            buf.insert(buf.end(), meta_str.begin(), meta_str.end());

            // Vertices.
            writeU32LE(buf, static_cast<uint32_t>(tile.vertices().size()));
            for (const auto& v : tile.vertices()) {
                writeF32LE(buf, v.east_m);
                writeF32LE(buf, v.north_m);
                writeF32LE(buf, v.up_m);
            }

            // Facets.
            writeU32LE(buf, static_cast<uint32_t>(tile.facets().size()));
            for (const auto& f : tile.facets()) {
                writeU32LE(buf, f.v[0]);
                writeU32LE(buf, f.v[1]);
                writeU32LE(buf, f.v[2]);
                buf.push_back(f.color.r);
                buf.push_back(f.color.g);
                buf.push_back(f.color.b);
            }
        }
    }
    return buf;
}

void TerrainMesh::deserializeLasTerrain(const std::vector<uint8_t>& data) {
    if (data.size() < 12) {
        throw std::runtime_error("TerrainMesh::deserializeLasTerrain: data too short");
    }
    const uint8_t* p = data.data();
    const uint32_t magic  = readU32LE(p);       p += 4;
    const uint32_t fmtver = readU32LE(p);       p += 4;
    const uint32_t n_tiles = readU32LE(p);      p += 4;

    if (magic != 0x4C415354u) {
        throw std::runtime_error("TerrainMesh::deserializeLasTerrain: invalid magic");
    }
    if (fmtver != 1) {
        throw std::runtime_error("TerrainMesh::deserializeLasTerrain: unsupported format_version");
    }

    cells_.clear();

    for (uint32_t ti = 0; ti < n_tiles; ++ti) {
        // Metadata JSON.
        const uint32_t meta_len = readU32LE(p); p += 4;
        const std::string meta_str(reinterpret_cast<const char*>(p), meta_len);
        p += meta_len;
        const nlohmann::json meta = nlohmann::json::parse(meta_str);

        const TerrainLod lod = static_cast<TerrainLod>(meta["lod"].get<int>());
        const auto& jc = meta["centroid"];
        const GeodeticPoint centroid{
            jc["latitude_rad"].get<double>(),
            jc["longitude_rad"].get<double>(),
            jc["height_wgs84_m"].get<float>(),
        };
        const auto& jb = meta["bounds"];
        const GeodeticAABB bounds{
            jb["lat_min_rad"].get<double>(),
            jb["lat_max_rad"].get<double>(),
            jb["lon_min_rad"].get<double>(),
            jb["lon_max_rad"].get<double>(),
            jb["height_min_m"].get<float>(),
            jb["height_max_m"].get<float>(),
        };

        // Vertices.
        const uint32_t n_verts = readU32LE(p);  p += 4;
        std::vector<TerrainVertex> vertices;
        vertices.reserve(n_verts);
        for (uint32_t vi = 0; vi < n_verts; ++vi) {
            const float e = readF32LE(p); p += 4;
            const float n = readF32LE(p); p += 4;
            const float u = readF32LE(p); p += 4;
            vertices.push_back({e, n, u});
        }

        // Facets.
        const uint32_t n_facets = readU32LE(p); p += 4;
        std::vector<TerrainFacet> facets;
        facets.reserve(n_facets);
        for (uint32_t fi = 0; fi < n_facets; ++fi) {
            TerrainFacet f;
            f.v[0]    = readU32LE(p); p += 4;
            f.v[1]    = readU32LE(p); p += 4;
            f.v[2]    = readU32LE(p); p += 4;
            f.color.r = *p++;
            f.color.g = *p++;
            f.color.b = *p++;
            facets.push_back(f);
        }

        TerrainTile tile(lod, centroid, bounds, std::move(vertices), std::move(facets));
        const double lat_c = (bounds.lat_min_rad + bounds.lat_max_rad) * 0.5;
        const double lon_c = (bounds.lon_min_rad + bounds.lon_max_rad) * 0.5;
        const uint64_t key = cellKey(lat_c, lon_c, lod);

        auto it = cells_.find(key);
        if (it != cells_.end()) {
            it->second.addTile(std::move(tile));
        } else {
            TerrainCell cell;
            cell.addTile(std::move(tile));
            cells_[key] = std::move(cell);
        }
    }
}

void TerrainMesh::exportGltf(const std::filesystem::path&, TerrainLod, const GeodeticPoint*) const {
    throw std::runtime_error("TerrainMesh::exportGltf: not implemented until Step 11");
}

} // namespace liteaerosim::environment
