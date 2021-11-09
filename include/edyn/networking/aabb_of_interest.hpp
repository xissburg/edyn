#ifndef EDYN_NETWORKING_AABB_OF_INTEREST_HPP
#define EDYN_NETWORKING_AABB_OF_INTEREST_HPP

#include "edyn/comp/aabb.hpp"
#include <entt/signal/sigh.hpp>
#include <entt/entity/sparse_set.hpp>
#include <vector>

namespace edyn {

struct aabb_of_interest {
    AABB aabb {vector3_one * -500, vector3_one * 500};
    entt::sparse_set entities;
    std::vector<entt::entity> create_entities;
    std::vector<entt::entity> destroy_entities;
};

}

#endif // EDYN_NETWORKING_AABB_OF_INTEREST_HPP
