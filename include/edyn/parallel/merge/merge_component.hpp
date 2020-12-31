#ifndef EDYN_PARALLEL_MERGE_COMPONENT_HPP
#define EDYN_PARALLEL_MERGE_COMPONENT_HPP

#include <entt/fwd.hpp>

namespace edyn {

class registry_delta;
class entity_map;

enum class merge_type : int {
    created, 
    updated
};

struct merge_context {
    const entt::registry *registry;
    const entity_map *map;
    const registry_delta *delta;
};

template<merge_type, typename Component>
void merge(const Component *old_comp, Component &new_comp, merge_context &) {}

}

#endif // EDYN_PARALLEL_MERGE_COMPONENT_HPP