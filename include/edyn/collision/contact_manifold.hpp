#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include <limits>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

struct contact_manifold {
    // Pair of rigid bodies which are touching.
    std::array<entt::entity, 2> body {entt::null, entt::null};

    // If the AABB of one of the bodies inflated by this amount does not
    // intersect the AABB of the other, the manifold will be destroyed.
    // See `edyn::broadphase::destroy_separated_manifolds`.
    scalar separation_threshold;
};

struct contact_manifold_state {
    uint8_t num_points {0};
    entt::entity contact_entity {entt::null};
};

struct clear_contact_manifold_tag {};

/**
 * Tag assigned to contact manifolds with non-zero restitution.
 */
struct contact_manifold_with_restitution {};

template<typename Archive>
void serialize(Archive &archive, contact_manifold &manifold) {
    archive(manifold.body);
    archive(manifold.separation_threshold);
}

template<typename Archive>
void serialize(Archive &archive, contact_manifold_state &manifold) {
    archive(manifold.num_points);
    archive(manifold.contact_entity);
}

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP
