#include "collision/BodyCollider.hpp"
#include "liteaerosim.pb.h"
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

static CollisionVolumeParams parseVolume(const nlohmann::json& j) {
    CollisionVolumeParams v;
    v.name = j.value("name", std::string{});
    const auto& he = j.at("half_extents_body_m");
    v.half_extents_body_m = {he.at(0).get<float>(), he.at(1).get<float>(), he.at(2).get<float>()};
    const auto& co = j.at("center_offset_body_m");
    v.center_offset_body_m = {co.at(0).get<float>(), co.at(1).get<float>(), co.at(2).get<float>()};
    v.stiffness_npm = j.at("stiffness_npm").get<float>();
    v.damping_nspm  = j.at("damping_nspm").get<float>();
    return v;
}

static nlohmann::json serializeVolume(const CollisionVolumeParams& v) {
    return {
        {"name",                 v.name},
        {"half_extents_body_m",  {v.half_extents_body_m.x(),  v.half_extents_body_m.y(),  v.half_extents_body_m.z()}},
        {"center_offset_body_m", {v.center_offset_body_m.x(), v.center_offset_body_m.y(), v.center_offset_body_m.z()}},
        {"stiffness_npm",        v.stiffness_npm},
        {"damping_nspm",         v.damping_nspm},
    };
}

// ---------------------------------------------------------------------------

void BodyCollider::recomputeMaxReach() {
    _max_reach_m = 0.f;
    for (const auto& v : _volumes) {
        // Bounding sphere from CG to any corner of this volume.
        const float reach = v.center_offset_body_m.norm() + v.half_extents_body_m.norm();
        _max_reach_m = std::max(_max_reach_m, reach);
    }
}

void BodyCollider::initialize(const nlohmann::json& config) {
    _volumes.clear();
    for (const auto& jv : config.at("volumes")) {
        _volumes.push_back(parseVolume(jv));
    }
    recomputeMaxReach();
}

// ---------------------------------------------------------------------------

ContactForces BodyCollider::step(const liteaero::nav::KinematicStateSnapshot& snap,
                                  const liteaero::terrain::Terrain& terrain) const {
    const float h_ac     = snap.position.altitude_m;
    const float terrain_h = terrain.elevation_m(snap.position.latitude_rad,
                                                  snap.position.longitude_rad);

    // AGL early exit: skip corner loops if no volume can possibly reach terrain.
    if (h_ac - terrain_h > _max_reach_m) return ContactForces{};

    const Eigen::Matrix3f R_nb = liteaero::nav::KinematicStateUtil::q_nb(snap).toRotationMatrix();
    const Eigen::Matrix3f R_bn = R_nb.transpose();
    const Eigen::Vector3f v_body = R_bn * snap.velocity_ned_mps;
    const Eigen::Vector3f omega  = snap.rates_body_rps;

    ContactForces out;

    for (const auto& vol : _volumes) {
        const float hx = vol.half_extents_body_m.x();
        const float hy = vol.half_extents_body_m.y();
        const float hz = vol.half_extents_body_m.z();

        const std::array<Eigen::Vector3f, 8> corners = {{
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy, -hz},
        }};

        for (const auto& corner_body : corners) {
            // Corner altitude: NED z positive = down, altitude positive = up
            const float corner_altitude = h_ac - (R_nb * corner_body).z();
            const float pen = terrain_h - corner_altitude;
            if (pen <= 0.f) continue;

            // Penetration rate: NED-z of corner velocity (positive = sinking)
            const float pen_dot = (R_nb * (v_body + omega.cross(corner_body))).z();

            // Cap effective penetration at 2× the volume half-depth to prevent
            // runaway spring forces during extreme embedding (e.g. a crash where
            // the aircraft clips deep through the terrain).  Two-way damping
            // (pen_dot enters without clamping) absorbs energy on both sink and
            // rise; the max(0) floor guarantees no suction on exit.
            const float pen_cap = 2.0f * vol.half_extents_body_m.z();
            const float pen_eff = std::min(pen, pen_cap);
            const float F_pen   = std::max(0.f,
                vol.stiffness_npm * pen_eff + vol.damping_nspm * pen_dot);

            // Upward NED force → body frame
            const Eigen::Vector3f F_body = R_bn * Eigen::Vector3f{0.f, 0.f, -F_pen};

            out.force_body_n   += F_body;
            out.moment_body_nm += corner_body.cross(F_body);
            out.weight_on_wheels = true;
        }
    }

    return out;
}

ContactForces BodyCollider::step(const liteaero::nav::KinematicStateSnapshot& snap) const {
    static const liteaero::terrain::FlatTerrain zero_terrain{0.f};
    return step(snap, zero_terrain);
}

float BodyCollider::maxCornerPenetration_m(
    const liteaero::nav::KinematicStateSnapshot& snap,
    const liteaero::terrain::Terrain& terrain) const
{
    const float h_ac      = snap.position.altitude_m;
    const float terrain_h = terrain.elevation_m(snap.position.latitude_rad,
                                                  snap.position.longitude_rad);

    if (h_ac - terrain_h > _max_reach_m) return 0.f;

    const Eigen::Matrix3f R_nb =
        liteaero::nav::KinematicStateUtil::q_nb(snap).toRotationMatrix();

    float max_pen = 0.f;

    for (const auto& vol : _volumes) {
        const float hx = vol.half_extents_body_m.x();
        const float hy = vol.half_extents_body_m.y();
        const float hz = vol.half_extents_body_m.z();

        const std::array<Eigen::Vector3f, 8> corners = {{
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy, -hz},
        }};

        for (const auto& corner_body : corners) {
            const float corner_altitude = h_ac - (R_nb * corner_body).z();
            const float pen = terrain_h - corner_altitude;
            if (pen > max_pen) max_pen = pen;
        }
    }

    return max_pen;
}

float BodyCollider::minCornerClearance_m(
    const liteaero::nav::KinematicStateSnapshot& snap,
    const liteaero::terrain::Terrain& terrain) const
{
    if (_volumes.empty()) return std::numeric_limits<float>::max();

    const float h_ac      = snap.position.altitude_m;
    const float terrain_h = terrain.elevation_m(snap.position.latitude_rad,
                                                  snap.position.longitude_rad);

    const Eigen::Matrix3f R_nb =
        liteaero::nav::KinematicStateUtil::q_nb(snap).toRotationMatrix();

    float min_clearance = std::numeric_limits<float>::max();

    for (const auto& vol : _volumes) {
        const float hx = vol.half_extents_body_m.x();
        const float hy = vol.half_extents_body_m.y();
        const float hz = vol.half_extents_body_m.z();

        const std::array<Eigen::Vector3f, 8> corners = {{
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{ hx, -hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx,  hy, -hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy,  hz},
            vol.center_offset_body_m + Eigen::Vector3f{-hx, -hy, -hz},
        }};

        for (const auto& corner_body : corners) {
            const float corner_altitude = h_ac - (R_nb * corner_body).z();
            const float clearance = corner_altitude - terrain_h;
            if (clearance < min_clearance) min_clearance = clearance;
        }
    }

    return min_clearance;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

nlohmann::json BodyCollider::serializeJson() const {
    nlohmann::json j;
    j["schema_version"] = 1;
    nlohmann::json vols = nlohmann::json::array();
    for (const auto& v : _volumes) vols.push_back(serializeVolume(v));
    j["volumes"] = vols;
    return j;
}

void BodyCollider::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("BodyCollider::deserializeJson: unsupported schema_version");
    _volumes.clear();
    for (const auto& jv : j.at("volumes")) {
        _volumes.push_back(parseVolume(jv));
    }
    recomputeMaxReach();
}

std::vector<uint8_t> BodyCollider::serializeProto() const {
    las_proto::BodyColliderParams proto;
    proto.set_schema_version(1);
    for (const auto& v : _volumes) {
        auto* pv = proto.add_volumes();
        pv->set_name(v.name);
        pv->mutable_half_extents_body_m()->set_x(v.half_extents_body_m.x());
        pv->mutable_half_extents_body_m()->set_y(v.half_extents_body_m.y());
        pv->mutable_half_extents_body_m()->set_z(v.half_extents_body_m.z());
        pv->mutable_center_offset_body_m()->set_x(v.center_offset_body_m.x());
        pv->mutable_center_offset_body_m()->set_y(v.center_offset_body_m.y());
        pv->mutable_center_offset_body_m()->set_z(v.center_offset_body_m.z());
        pv->set_stiffness_npm(v.stiffness_npm);
        pv->set_damping_nspm(v.damping_nspm);
    }
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void BodyCollider::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::BodyColliderParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("BodyCollider::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("BodyCollider::deserializeProto: unsupported schema_version");
    _volumes.clear();
    for (int i = 0; i < proto.volumes_size(); ++i) {
        const auto& pv = proto.volumes(i);
        CollisionVolumeParams v;
        v.name = pv.name();
        v.half_extents_body_m  = {pv.half_extents_body_m().x(),
                                   pv.half_extents_body_m().y(),
                                   pv.half_extents_body_m().z()};
        v.center_offset_body_m = {pv.center_offset_body_m().x(),
                                   pv.center_offset_body_m().y(),
                                   pv.center_offset_body_m().z()};
        v.stiffness_npm = pv.stiffness_npm();
        v.damping_nspm  = pv.damping_nspm();
        _volumes.push_back(v);
    }
    recomputeMaxReach();
}

} // namespace liteaero::simulation
