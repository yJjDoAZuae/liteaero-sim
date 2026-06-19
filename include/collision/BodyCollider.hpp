#pragma once

#include "physics/ContactForces.hpp"
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <liteaero/terrain/Terrain.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace liteaero::simulation {

// One independently-configured OBB collision volume.
// Multiple volumes per BodyCollider allow separate boxes for fuselage,
// wing, and empennage so each part has an appropriately-sized box.
struct CollisionVolumeParams {
    std::string     name;
    Eigen::Vector3f half_extents_body_m  = {0.5f, 0.3f, 0.2f};
    Eigen::Vector3f center_offset_body_m = Eigen::Vector3f::Zero();
    float           stiffness_npm        = 10000.f;
    float           damping_nspm         = 500.f;
};

// Body-axis OBB collision system. Each step checks all 8 corners of every
// configured volume against terrain and applies an inelastic spring-damper
// penalty force. Protects all aircraft orientations (normal, inverted,
// pitched, wing-low) using volumes sized to the part they cover.
//
// BodyCollider is stateless — serializeJson() / deserializeJson() round-trip
// configuration only.
class BodyCollider {
public:
    void initialize(const nlohmann::json& config);
    void reset() {}

    // Compute contact forces against terrain.
    ContactForces step(const liteaero::nav::KinematicStateSnapshot& snap,
                       const liteaero::terrain::Terrain& terrain) const;

    // Convenience: assumes flat terrain at elevation 0 m (useful in unit tests).
    ContactForces step(const liteaero::nav::KinematicStateSnapshot& snap) const;

    // Return the maximum depth (m) by which any corner of any volume penetrates
    // terrain.  Positive means the deepest corner is that far below terrain;
    // zero or negative means all corners are above terrain.  Intended to be
    // called on the post-integration state so the simulation loop can apply
    // the terrain hard constraint without needing the full contact force.
    [[nodiscard]] float maxCornerPenetration_m(
        const liteaero::nav::KinematicStateSnapshot& snap,
        const liteaero::terrain::Terrain& terrain) const;

    // Return the signed clearance (m) of the lowest corner of any volume above
    // terrain: positive means every corner is above terrain (the smallest such
    // margin), negative means the deepest corner is that far below terrain.
    // Unlike maxCornerPenetration_m (clamped to >= 0), this is signed, so the
    // simulation loop can detect genuine separation from the surface — not just
    // penetration — to release a hard-contact latch. Returns a large positive
    // value when no volumes are configured.
    [[nodiscard]] float minCornerClearance_m(
        const liteaero::nav::KinematicStateSnapshot& snap,
        const liteaero::terrain::Terrain& terrain) const;

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json& j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    std::vector<CollisionVolumeParams> _volumes;
    // Maximum distance from CG to any corner of any volume, over all orientations.
    // Used as the AGL early-exit threshold.
    float _max_reach_m = 0.f;

    void recomputeMaxReach();
};

} // namespace liteaero::simulation
