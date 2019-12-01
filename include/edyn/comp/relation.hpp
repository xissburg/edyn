#ifndef EDYN_COMP_RELATION_HPP
#define EDYN_COMP_RELATION_HPP

#include <entt/entt.hpp>
#include "edyn/util/array.hpp"

namespace edyn {

inline constexpr size_t max_relations = 2;

struct relation {
    std::array<entt::entity, max_relations> entity =
        make_array<max_relations>(entt::entity{entt::null});
};

struct relation_container {
    std::vector<entt::entity> entities;
};

}

#endif // EDYN_COMP_RELATION_HPP