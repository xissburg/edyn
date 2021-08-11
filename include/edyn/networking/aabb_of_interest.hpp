#ifndef EDYN_NETWORKING_AABB_OF_INTEREST_HPP
#define EDYN_NETWORKING_AABB_OF_INTEREST_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/util/entity_set.hpp"
#include <entt/signal/sigh.hpp>
#include <vector>

namespace edyn {

struct aabb_of_interest {
    AABB aabb {vector3_one * -500, vector3_one * 500};
    entity_set entities;
    std::vector<entt::entity> create_entities;
    std::vector<entt::entity> destroy_entities;
};

}

#endif // EDYN_NETWORKING_AABB_OF_INTEREST_HPP
