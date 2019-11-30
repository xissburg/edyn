#ifndef EDYN_COMP_ISLAND_HPP
#define EDYN_COMP_ISLAND_HPP

#include <vector>
#include <entt/fwd.hpp>

namespace edyn {

struct island {
    std::vector<entt::entity> entities;
};

struct island_node {
    entt::entity island_entity;
};

}

#endif // EDYN_COMP_ISLAND_HPP