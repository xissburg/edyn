#ifndef EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

class entity_map;

template<typename Component>
void merge(Component &new_comp, entity_map &) {}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_COMPONENT_HPP
