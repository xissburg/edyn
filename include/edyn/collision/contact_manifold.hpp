#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <limits>
#include "edyn/config/constants.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

struct contact_manifold {
    using contact_index_type = unsigned;
    static constexpr auto invalid_index = std::numeric_limits<contact_index_type>::max();

    std::array<entt::entity, 2> body {entt::null, entt::null};
    scalar separation_threshold;
    unsigned num_points {0};
    std::array<contact_index_type, max_contacts> indices;
    std::array<contact_point, max_contacts> point;
};

/**
 * Tag assigned to contact manifolds with non-zero restitution.
 */
struct contact_manifold_with_restitution {};

}


#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP
