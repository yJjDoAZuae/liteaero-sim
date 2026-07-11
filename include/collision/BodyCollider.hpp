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
    // No per-volume contact parameter (body_collider.md §5a / OQ-BC-5): the
    // velocity-arrest damping is a single collider-level quantity derived from
    // the airframe mass and step in initialize().
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
    // mass_kg and dt_s are the airframe mass and outer step rate; the §5a
    // velocity-arrest damping is derived from them (body_collider.md §5a / OQ-BC-5).
    // inertia_diag_kgm2 is the body-frame inertia diagonal (Ixx, Iyy, Izz); it feeds
    // the OQ-BC-12 Alt B momentum impulse K_c = 1/m + (r x n)^T I^-1 (r x n).
    // The defaults are a convenience for geometry unit tests — Aircraft always
    // passes the real airframe mass, step, and inertia.
    void initialize(const nlohmann::json& config, float mass_kg = 1.0f, float dt_s = 0.01f,
                    const Eigen::Vector3f& inertia_diag_kgm2 = Eigen::Vector3f::Zero());
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

    // Coefficient of restitution for the §5b restitution-consistent terrain hard
    // constraint (body_collider.md §5b / OQ-BC-5). The single user-facing contact
    // parameter; non-dimensional, default 0 (fully inelastic), clamped to [0, 1).
    [[nodiscard]] float restitution_nd() const { return _restitution_nd; }

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json& j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);

private:
    // Fixed internal arrest factor N_arr: the velocity-arrest aggregate damping
    // arrests an impact over ~N_arr outer steps. Chosen so the worst-case
    // aggregate CFL bound b_total*dt/m = 1/N_arr stays well below 1 (§5a).
    static constexpr float kArrestSteps = 3.0f;
    // §5d slip-velocity regularization scale (m/s): smooths the Coulomb friction
    // direction through zero slip to avoid stick-slip chatter.
    static constexpr float kSlipRegMps = 0.05f;

    std::vector<CollisionVolumeParams> _volumes;
    // Single collider-level coefficient of restitution consumed by the §5b hard
    // constraint. Clamped to [0, 1) on load.
    float _restitution_nd = 0.f;
    // Per-corner velocity-arrest damping coefficient (N*s/m), derived in
    // initialize() as mass / (n_corners_total * kArrestSteps * dt) so the
    // worst-case aggregate over all corners meets the §5a stability bound.
    float _b_corner_nspm = 0.f;
    // §5d scrape friction, collider-level non-dimensional knobs (default 0 =
    // frictionless). Coulomb coefficient mu, and a viscous "plowing" factor that
    // scales the §5a per-corner damping (so the dimensional viscous coefficient is
    // mass-scaled and tuning-free). Both clamped to >= 0 on load.
    float _friction_coulomb_nd = 0.f;
    float _friction_viscous_nd = 0.f;
    // Airframe mass (kg) and body-frame inertia diagonal (Ixx, Iyy, Izz, kg*m^2),
    // supplied by Aircraft (OQ-BC-5). Consumed by the OQ-BC-12 Alt B momentum impulse
    // K_c = 1/m + (r x n)^T I^-1 (r x n); serialized so a restored collider keeps them
    // without Aircraft re-supplying mass/inertia.
    float           _mass_kg           = 0.f;
    Eigen::Vector3f _inertia_diag_kgm2 = Eigen::Vector3f::Zero();
    // Maximum distance from CG to any corner of any volume, over all orientations.
    // Used as the AGL early-exit threshold.
    float _max_reach_m = 0.f;

    void recomputeMaxReach();
};

} // namespace liteaero::simulation
