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
    return v;
}

static nlohmann::json serializeVolume(const CollisionVolumeParams& v) {
    return {
        {"name",                 v.name},
        {"half_extents_body_m",  {v.half_extents_body_m.x(),  v.half_extents_body_m.y(),  v.half_extents_body_m.z()}},
        {"center_offset_body_m", {v.center_offset_body_m.x(), v.center_offset_body_m.y(), v.center_offset_body_m.z()}},
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

void BodyCollider::initialize(const nlohmann::json& config, float mass_kg, float dt_s) {
    _volumes.clear();
    for (const auto& jv : config.at("volumes")) {
        _volumes.push_back(parseVolume(jv));
    }
    // §5b coefficient of restitution: single, optional, non-dimensional, [0, 1).
    _restitution_nd =
        std::clamp(config.value("restitution_nd", 0.f), 0.f, std::nextafter(1.0f, 0.0f));

    // §5d scrape friction: single, optional, non-dimensional knobs, >= 0.
    _friction_coulomb_nd = std::max(0.f, config.value("friction_coulomb_nd", 0.f));
    _friction_viscous_nd = std::max(0.f, config.value("friction_viscous_nd", 0.f));

    // §5a velocity-arrest damping, derived (OQ-BC-5): distribute an aggregate
    // b_total = mass / (kArrestSteps * dt) across all corners so that even with
    // every corner penetrating, the aggregate CFL bound b_total*dt/mass = 1/N_arr
    // stays below 1 on any airframe. The user authors no contact coefficient.
    const std::size_t n_corners_total = 8u * _volumes.size();
    _b_corner_nspm =
        (n_corners_total > 0u && dt_s > 0.f)
            ? mass_kg / (static_cast<float>(n_corners_total) * kArrestSteps * dt_s)
            : 0.f;

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

            // Corner velocity in NED: z is the sink rate (positive = sinking);
            // the horizontal (x,y) components are the tangential ground slip.
            const Eigen::Vector3f v_corner_ned = R_nb * (v_body + omega.cross(corner_body));
            const float pen_dot = v_corner_ned.z();

            // §5a velocity-arrest contact (body_collider.md §5a). A purely
            // dissipative, penetration-modulated force F = max(0, c * delta * delta_dot)
            // that opposes the sink rate only — no spring, no static restoring
            // force (the §5b hard constraint owns static non-penetration), and the
            // max(0) floor forbids adhesion / does no work on rebound. The
            // coefficient c = b_corner / hz makes the effective damping (c*delta)
            // equal b_corner at the volume half-depth; penetration is capped at
            // 2*hz so the worst-case effective damping is bounded.
            const float hz      = vol.half_extents_body_m.z();
            const float pen_cap = 2.0f * hz;
            const float pen_eff = std::min(pen, pen_cap);
            const float c_nspm2 = (hz > 0.f) ? _b_corner_nspm / hz : 0.f;
            const float F_pen   = std::max(0.f, c_nspm2 * pen_eff * pen_dot);

            // Normal arrest force, upward in NED (flat-terrain normal = NED -z).
            Eigen::Vector3f F_ned{0.f, 0.f, -F_pen};

            // §5d scrape friction (body_collider.md §5d) at this penetrating corner.
            // Coulomb term -mu*F_pen*vhat_t opposes slip scaled by the normal force
            // (regularized through zero slip to avoid chatter); the viscous "plowing"
            // term -(k_visc*b_corner)*v_t opposes slip directly with a mass-scaled
            // coefficient and acts even in steady scrape where F_pen ~ 0. Both are
            // zero by default (frictionless until configured).
            const Eigen::Vector3f v_t_ned{v_corner_ned.x(), v_corner_ned.y(), 0.f};
            const float speed_t = v_t_ned.norm();
            if (speed_t > 0.f &&
                (_friction_coulomb_nd > 0.f || _friction_viscous_nd > 0.f)) {
                const float c_visc = _friction_viscous_nd * _b_corner_nspm;
                const float coulomb = _friction_coulomb_nd * F_pen
                    / std::sqrt(speed_t * speed_t + kSlipRegMps * kSlipRegMps);
                F_ned -= (coulomb + c_visc) * v_t_ned;
            }

            const Eigen::Vector3f F_body = R_bn * F_ned;
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
    j["restitution_nd"] = _restitution_nd;
    j["arrest_damping_corner_nspm"] = _b_corner_nspm;
    j["friction_coulomb_nd"] = _friction_coulomb_nd;
    j["friction_viscous_nd"] = _friction_viscous_nd;
    return j;
}

void BodyCollider::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("BodyCollider::deserializeJson: unsupported schema_version");
    _volumes.clear();
    for (const auto& jv : j.at("volumes")) {
        _volumes.push_back(parseVolume(jv));
    }
    _restitution_nd =
        std::clamp(j.value("restitution_nd", 0.f), 0.f, std::nextafter(1.0f, 0.0f));
    _b_corner_nspm = j.value("arrest_damping_corner_nspm", 0.f);
    _friction_coulomb_nd = std::max(0.f, j.value("friction_coulomb_nd", 0.f));
    _friction_viscous_nd = std::max(0.f, j.value("friction_viscous_nd", 0.f));
    recomputeMaxReach();
}

std::vector<uint8_t> BodyCollider::serializeProto() const {
    las_proto::BodyColliderParams proto;
    proto.set_schema_version(1);
    proto.set_restitution_nd(_restitution_nd);
    proto.set_arrest_damping_corner_nspm(_b_corner_nspm);
    proto.set_friction_coulomb_nd(_friction_coulomb_nd);
    proto.set_friction_viscous_nd(_friction_viscous_nd);
    for (const auto& v : _volumes) {
        auto* pv = proto.add_volumes();
        pv->set_name(v.name);
        pv->mutable_half_extents_body_m()->set_x(v.half_extents_body_m.x());
        pv->mutable_half_extents_body_m()->set_y(v.half_extents_body_m.y());
        pv->mutable_half_extents_body_m()->set_z(v.half_extents_body_m.z());
        pv->mutable_center_offset_body_m()->set_x(v.center_offset_body_m.x());
        pv->mutable_center_offset_body_m()->set_y(v.center_offset_body_m.y());
        pv->mutable_center_offset_body_m()->set_z(v.center_offset_body_m.z());
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
    _restitution_nd =
        std::clamp(proto.restitution_nd(), 0.f, std::nextafter(1.0f, 0.0f));
    _b_corner_nspm = proto.arrest_damping_corner_nspm();
    _friction_coulomb_nd = std::max(0.f, proto.friction_coulomb_nd());
    _friction_viscous_nd = std::max(0.f, proto.friction_viscous_nd());
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
        _volumes.push_back(v);
    }
    recomputeMaxReach();
}

} // namespace liteaero::simulation
