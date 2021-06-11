#ifndef EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

class entity_map;

struct merge_context {
    const entt::registry *registry;
    const entity_map *map;
};

template<typename Component>
void merge(const Component *old_comp, Component &new_comp, merge_context &) {}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP
