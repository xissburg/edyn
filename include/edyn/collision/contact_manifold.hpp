#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/constants.hpp"

namespace edyn {

struct contact_manifold {
    std::array<entt::entity, 2> body;
    size_t num_points {0};
    std::array<entt::entity, max_contacts> point_entity;
};

}


#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP