#define _USE_MATH_DEFINES
#include "environment/TerrainMesh.hpp"
#include "geodesy/Wgs84.hpp"
#include "liteaerosim.pb.h"
#include "tiny_gltf.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace liteaero::simulation {

using namespace liteaero::terrain;

// ---------------------------------------------------------------------------
// WGS84 / Earth constants
//
// Geodetic transform helpers (geodeticToEcef, ecefOffsetToEnu, enuToEcefOffset,
// primeVerticalRadius) and the WGS84 ellipsoid constants live in the shared
// liteaero::geodesy module — see include/geodesy/Wgs84.hpp.  Only the cell-
// extent constants specific to TerrainMesh remain here.
// ---------------------------------------------------------------------------
static constexpr int    kLodCount     = 7;
static constexpr double kEarthRadiusM = 6371000.0;  // mean radius, for the footprint-record cross-check

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Single-tile index grid side (rad): the smaller of the tile's two angular spans.  Using the
// smaller span keeps the packed (lat_idx, lon_idx) key injective in both dimensions — a coarser
// grid could map two distinct tiles to one bucket in the finer dimension.
static double tileExtentRad(const GeodeticAABB& b) {
    const double lat_span = b.lat_max_rad - b.lat_min_rad;
    const double lon_span = b.lon_max_rad - b.lon_min_rad;
    return std::min(lat_span, lon_span);
}

double TerrainMesh::cellExtentRad(TerrainLod lod) const {
    return lod_extent_rad_[static_cast<int>(lod)];
}

uint64_t TerrainMesh::cellKey(double lat_rad, double lon_rad, TerrainLod lod) const {
    const double extent = cellExtentRad(lod);
    const auto lat_idx = static_cast<uint64_t>(std::floor((lat_rad + M_PI / 2.0) / extent));
    const auto lon_idx = static_cast<uint64_t>(std::floor((lon_rad + M_PI)       / extent));
    return (lat_idx << 48) | (lon_idx << 32) | (static_cast<uint64_t>(static_cast<int>(lod)) << 16);
}

// Rebuild the index from a flat tile list.  Pass 1 sets the per-LOD grid side to the largest
// single-tile extent at that LOD (so a clipped boundary tile does not shrink the grid below the
// true spacing); pass 2 keys and inserts every tile.  This is the OQ-T-1 / OQ-T-3 derive-from-
// bounds mechanism that replaces the former compiled-in per-LOD cell-side constant.
void TerrainMesh::indexTiles(std::vector<TerrainTile> tiles) {
    cells_.clear();
    lod_extent_rad_.fill(0.0);
    for (const auto& t : tiles) {
        double& slot = lod_extent_rad_[static_cast<int>(t.lod())];
        slot = std::max(slot, tileExtentRad(t.bounds()));
    }
    for (auto& t : tiles) {
        const GeodeticAABB& b = t.bounds();
        const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        const uint64_t key = cellKey(lat_c, lon_c, t.lod());
        auto it = cells_.find(key);
        if (it != cells_.end()) {
            it->second.addTile(std::move(t));
        } else {
            TerrainCell cell;
            cell.addTile(std::move(t));
            cells_[key] = std::move(cell);
        }
    }
}

// Cross-check a recorded per-LOD metric footprint against the bounds-derived grid side (OQ-T-3
// Alternative 4).  The recorded value is the build's NOMINAL per-LOD grid pitch; the build rounds
// the region into an integer number of cells (cell = region / ceil(region / footprint)) and clips
// edge tiles, so the realized tiles are ALWAYS ≤ the nominal — for a small region a coarse LOD's
// tiles can be much smaller than its nominal footprint, and that is expected, not an error.  The
// derived extent is min(lat_span, lon_span) ≈ footprint / R, so extent * R recovers the realized
// footprint in meters (to within the mean-vs-meridian radius, ~0.3%).  We therefore flag only the
// impossible/corrupt direction: tiles materially LARGER than the record, which means the record
// does not describe these tiles.  LODs with a zero record or no tiles are skipped.
void TerrainMesh::validateStoredFootprints(const std::array<double, 7>& stored_footprint_m) const {
    constexpr double kTolerance = 0.25;  // 25% headroom over the nominal for the radius approximation
    for (int l = 0; l < kLodCount; ++l) {
        const double stored     = stored_footprint_m[l];
        const double extent_rad = lod_extent_rad_[l];
        if (stored <= 0.0 || extent_rad <= 0.0) continue;
        const double derived_m = extent_rad * kEarthRadiusM;
        if (derived_m > stored * (1.0 + kTolerance)) {
            throw std::runtime_error(
                "TerrainMesh: tiles at LOD " + std::to_string(l) + " (~" + std::to_string(derived_m) +
                " m) are larger than the recorded per-LOD footprint (" + std::to_string(stored) +
                " m); the footprint record does not describe these tiles");
        }
    }
}

// Geodesy helpers now live in liteaero::geodesy (see include/geodesy/Wgs84.hpp).
using liteaero::geodesy::geodeticToEcef;
using liteaero::geodesy::ecefOffsetToEnu;
using liteaero::geodesy::enuToEcefOffset;

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
    const TerrainLod lod = cell.finestAvailableLod();
    // Derive this LOD's grid side from the cell's own bounds (OQ-T-1 / OQ-T-3), widening the
    // per-LOD extent if this is the largest tile seen at that LOD so far.
    double& slot = lod_extent_rad_[static_cast<int>(lod)];
    slot = std::max(slot, tileExtentRad(b));
    const double lat_c = (b.lat_min_rad + b.lat_max_rad) * 0.5;
    const double lon_c = (b.lon_min_rad + b.lon_max_rad) * 0.5;
    cells_[cellKey(lat_c, lon_c, lod)] = std::move(cell);
}

const TerrainCell* TerrainMesh::cellAt(double lat_rad, double lon_rad) const {
    const TerrainCell* best     = nullptr;
    int                best_lod = kLodCount;  // sentinel: worse than any valid LOD
    for (const auto& [key, cell] : cells_) {
        const GeodeticAABB& b = cell.bounds();
        if (lat_rad >= b.lat_min_rad && lat_rad <= b.lat_max_rad &&
            lon_rad >= b.lon_min_rad && lon_rad <= b.lon_max_rad) {
            const int lod = static_cast<int>(cell.finestAvailableLod());
            if (lod < best_lod) {
                best     = &cell;
                best_lod = lod;
            }
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Terrain — elevation_m
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
        const double R_north = liteaero::geodesy::kWgs84A
                             * (1.0 - liteaero::geodesy::kWgs84E2);   // conservative meridional
        const double R_east  = liteaero::geodesy::kWgs84A * std::cos(lat_c);
                                                                     // prime vertical * cos(lat)
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
    j["lod_footprints_m"] = lod_footprint_m_;  // per-LOD footprint record (OQ-T-3 Alt 4)
    return j;
}

void TerrainMesh::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("TerrainMesh::deserializeJson: unsupported schema_version");
    }
    std::vector<TerrainTile> tiles;
    for (const auto& jt : j.at("tiles")) {
        tiles.push_back(TerrainTile::deserializeJson(jt));
    }
    indexTiles(std::move(tiles));

    std::array<double, 7> stored{};
    if (j.contains("lod_footprints_m")) {
        const auto& arr = j.at("lod_footprints_m");
        for (int l = 0; l < kLodCount && l < static_cast<int>(arr.size()); ++l) {
            stored[l] = arr[l].get<double>();
        }
    }
    lod_footprint_m_ = stored;
    validateStoredFootprints(stored);
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
    for (int l = 0; l < kLodCount; ++l) {
        proto.add_lod_footprints_m(static_cast<float>(lod_footprint_m_[l]));
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
    std::vector<TerrainTile> tiles;
    for (const auto& tile_proto : proto.tiles()) {
        std::string buf;
        tile_proto.SerializeToString(&buf);
        const std::vector<uint8_t> tile_bytes(buf.begin(), buf.end());
        tiles.push_back(TerrainTile::deserializeProto(tile_bytes));
    }
    indexTiles(std::move(tiles));

    std::array<double, 7> stored{};
    const int n = std::min(kLodCount, proto.lod_footprints_m_size());
    for (int l = 0; l < n; ++l) stored[l] = proto.lod_footprints_m(l);
    lod_footprint_m_ = stored;
    validateStoredFootprints(stored);
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
    // Per-LOD footprint record (7 × float32, L0..L6); echoed from what was loaded, else zeros.
    for (int l = 0; l < kLodCount; ++l) writeF32LE(buf, static_cast<float>(lod_footprint_m_[l]));

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
    if (data.size() < 40) {
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

    // Per-LOD footprint record (7 × float32, L0..L6).
    std::array<double, 7> stored_footprint_m{};
    for (int l = 0; l < kLodCount; ++l) { stored_footprint_m[l] = readF32LE(p); p += 4; }

    // Bounds guard: reject a truncated or misaligned buffer with a clear error instead of reading
    // past the end.  A file written before the per-LOD footprint header (12-byte header) trips this.
    const uint8_t* const end = data.data() + data.size();
    auto require = [&](std::size_t n) {
        if (static_cast<std::size_t>(end - p) < n) {
            throw std::runtime_error(
                "TerrainMesh::deserializeLasTerrain: truncated or malformed data — the file may "
                "predate the per-LOD footprint header; regenerate it with build_terrain.py");
        }
    };

    std::vector<TerrainTile> tiles;
    tiles.reserve(n_tiles);

    for (uint32_t ti = 0; ti < n_tiles; ++ti) {
        // Metadata JSON.
        require(4);
        const uint32_t meta_len = readU32LE(p); p += 4;
        require(meta_len);
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
        require(4);
        const uint32_t n_verts = readU32LE(p);  p += 4;
        require(static_cast<std::size_t>(n_verts) * 12);
        std::vector<TerrainVertex> vertices;
        vertices.reserve(n_verts);
        for (uint32_t vi = 0; vi < n_verts; ++vi) {
            const float e = readF32LE(p); p += 4;
            const float n = readF32LE(p); p += 4;
            const float u = readF32LE(p); p += 4;
            vertices.push_back({e, n, u});
        }

        // Facets.
        require(4);
        const uint32_t n_facets = readU32LE(p); p += 4;
        require(static_cast<std::size_t>(n_facets) * 15);
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

        tiles.emplace_back(lod, centroid, bounds, std::move(vertices), std::move(facets));
    }

    indexTiles(std::move(tiles));
    lod_footprint_m_ = stored_footprint_m;
    validateStoredFootprints(stored_footprint_m);
}

// ---------------------------------------------------------------------------
// Step 11 — exportGltf (glTF 2.0 / GLB)
// ---------------------------------------------------------------------------

void TerrainMesh::exportGltf(const std::filesystem::path& output_path,
                              TerrainLod                   max_lod,
                              const GeodeticPoint*         world_origin) const {
    // Determine world origin: use provided value, or centroid of first cell found.
    GeodeticPoint origin{0.0, 0.0, 0.f};
    if (world_origin) {
        origin = *world_origin;
    } else if (!cells_.empty()) {
        const GeodeticAABB& b = cells_.begin()->second.bounds();
        origin.latitude_rad  = (b.lat_min_rad + b.lat_max_rad) * 0.5;
        origin.longitude_rad = (b.lon_min_rad + b.lon_max_rad) * 0.5;
        origin.height_wgs84_m = (b.height_min_m + b.height_max_m) * 0.5f;
    }

    double Xo, Yo, Zo;
    geodeticToEcef(origin.latitude_rad, origin.longitude_rad,
                   static_cast<double>(origin.height_wgs84_m), Xo, Yo, Zo);

    tinygltf::Model model;
    model.asset.version   = "2.0";
    model.asset.generator = "LiteAeroSim";

    tinygltf::Scene scene;
    scene.name = "terrain";

    // Single combined binary buffer; all buffer views index into buffer 0.
    std::vector<uint8_t> bin_data;

    for (const auto& [key, cell] : cells_) {
        const auto lod_opt = selectTileLod(cell, max_lod);
        if (!lod_opt) continue;
        const TerrainTile& tile = cell.tile(*lod_opt);

        const auto& verts  = tile.vertices();
        const auto& facets = tile.facets();
        const size_t n_verts = 3 * facets.size();  // vertex duplication for per-facet COLOR_0

        // --- position buffer (float32, glTF Y-up: X=East, Y=Up, Z=-North) ---
        std::vector<float> positions;
        positions.reserve(n_verts * 3);
        std::array<float, 3> pos_min = { FLT_MAX,  FLT_MAX,  FLT_MAX};
        std::array<float, 3> pos_max = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

        // --- color buffer (uint8 RGBA) ---
        std::vector<uint8_t> colors;
        colors.reserve(n_verts * 4);

        for (const auto& facet : facets) {
            for (int vi = 0; vi < 3; ++vi) {
                const TerrainVertex& v = verts[facet.v[vi]];
                const float gx = v.east_m;   // glTF X = ENU East
                const float gy = v.up_m;     // glTF Y = ENU Up
                const float gz = -v.north_m; // glTF Z = ENU -North
                positions.push_back(gx);
                positions.push_back(gy);
                positions.push_back(gz);
                pos_min[0] = std::min(pos_min[0], gx);
                pos_min[1] = std::min(pos_min[1], gy);
                pos_min[2] = std::min(pos_min[2], gz);
                pos_max[0] = std::max(pos_max[0], gx);
                pos_max[1] = std::max(pos_max[1], gy);
                pos_max[2] = std::max(pos_max[2], gz);
                colors.push_back(facet.color.r);
                colors.push_back(facet.color.g);
                colors.push_back(facet.color.b);
                colors.push_back(255u);
            }
        }

        // Append position bytes to bin_data (4-byte aligned).
        const size_t pos_byte_offset = bin_data.size();
        const size_t pos_byte_len    = positions.size() * sizeof(float);
        bin_data.resize(bin_data.size() + pos_byte_len);
        std::memcpy(bin_data.data() + pos_byte_offset, positions.data(), pos_byte_len);
        while (bin_data.size() % 4 != 0) bin_data.push_back(0);

        // Append color bytes to bin_data (4-byte aligned).
        const size_t col_byte_offset = bin_data.size();
        const size_t col_byte_len    = colors.size();
        bin_data.resize(bin_data.size() + col_byte_len);
        std::memcpy(bin_data.data() + col_byte_offset, colors.data(), col_byte_len);
        while (bin_data.size() % 4 != 0) bin_data.push_back(0);

        // Buffer views.
        const int pos_bv_idx = static_cast<int>(model.bufferViews.size());
        {
            tinygltf::BufferView bv;
            bv.buffer     = 0;
            bv.byteOffset = pos_byte_offset;
            bv.byteLength = pos_byte_len;
            bv.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
            model.bufferViews.push_back(std::move(bv));
        }
        const int col_bv_idx = static_cast<int>(model.bufferViews.size());
        {
            tinygltf::BufferView bv;
            bv.buffer     = 0;
            bv.byteOffset = col_byte_offset;
            bv.byteLength = col_byte_len;
            bv.target     = TINYGLTF_TARGET_ARRAY_BUFFER;
            model.bufferViews.push_back(std::move(bv));
        }

        // Accessors.
        const int pos_acc_idx = static_cast<int>(model.accessors.size());
        {
            tinygltf::Accessor acc;
            acc.bufferView    = pos_bv_idx;
            acc.byteOffset    = 0;
            acc.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
            acc.type          = TINYGLTF_TYPE_VEC3;
            acc.count         = n_verts;
            acc.minValues     = {static_cast<double>(pos_min[0]),
                                 static_cast<double>(pos_min[1]),
                                 static_cast<double>(pos_min[2])};
            acc.maxValues     = {static_cast<double>(pos_max[0]),
                                 static_cast<double>(pos_max[1]),
                                 static_cast<double>(pos_max[2])};
            model.accessors.push_back(std::move(acc));
        }
        const int col_acc_idx = static_cast<int>(model.accessors.size());
        {
            tinygltf::Accessor acc;
            acc.bufferView    = col_bv_idx;
            acc.byteOffset    = 0;
            acc.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
            acc.type          = TINYGLTF_TYPE_VEC4;
            acc.normalized    = true;
            acc.count         = n_verts;
            model.accessors.push_back(std::move(acc));
        }

        // Mesh + primitive.
        const int mesh_idx = static_cast<int>(model.meshes.size());
        {
            tinygltf::Primitive prim;
            prim.attributes["POSITION"] = pos_acc_idx;
            prim.attributes["COLOR_0"]  = col_acc_idx;
            prim.mode                   = TINYGLTF_MODE_TRIANGLES;
            tinygltf::Mesh gltf_mesh;
            gltf_mesh.primitives.push_back(std::move(prim));
            model.meshes.push_back(std::move(gltf_mesh));
        }

        // Node: tile centroid as ENU offset from world origin, mapped to glTF axes.
        const GeodeticPoint& c = tile.centroid();
        double Xt, Yt, Zt;
        geodeticToEcef(c.latitude_rad, c.longitude_rad,
                       static_cast<double>(c.height_wgs84_m), Xt, Yt, Zt);
        double east, north, up;
        ecefOffsetToEnu(origin.latitude_rad, origin.longitude_rad,
                        Xt - Xo, Yt - Yo, Zt - Zo,
                        east, north, up);
        // glTF: X=East, Y=Up, Z=-North
        tinygltf::Node tile_node;
        tile_node.mesh        = mesh_idx;
        tile_node.translation = {east, up, -north};
        const int tile_node_idx = static_cast<int>(model.nodes.size());
        model.nodes.push_back(std::move(tile_node));
        scene.nodes.push_back(tile_node_idx);
    }

    // Root node that groups all tile nodes and carries LiteAero Sim GLTF extras.
    {
        tinygltf::Value::Object extras;
        extras["liteaerosim_terrain"]  = tinygltf::Value(true);
        extras["schema_version"]       = tinygltf::Value(1);
        extras["world_origin_lat_rad"] = tinygltf::Value(origin.latitude_rad);
        extras["world_origin_lon_rad"] = tinygltf::Value(origin.longitude_rad);
        extras["world_origin_height_m"]= tinygltf::Value(static_cast<double>(origin.height_wgs84_m));
        extras["lod"]                  = tinygltf::Value(static_cast<int>(max_lod));

        tinygltf::Node root;
        root.name     = "terrain_root";
        root.children = scene.nodes;
        root.extras   = tinygltf::Value(std::move(extras));
        const int root_idx = static_cast<int>(model.nodes.size());
        model.nodes.push_back(std::move(root));
        scene.nodes = {root_idx};
    }

    // Binary buffer (buffer 0).
    {
        tinygltf::Buffer buf;
        buf.data = std::move(bin_data);
        model.buffers.push_back(std::move(buf));
    }

    model.scenes.push_back(std::move(scene));
    model.defaultScene = 0;

    tinygltf::TinyGLTF writer;
    const bool embed_images  = false;
    const bool embed_buffers = true;
    const bool pretty_print  = false;
    const bool write_binary  = true;
    writer.WriteGltfSceneToFile(&model, output_path.string(),
                                embed_images, embed_buffers,
                                pretty_print, write_binary);
}

} // namespace liteaero::simulation
