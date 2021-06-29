#ifndef EDYN_PARALLEL_MERGE_MERGE_COLLISION_EXCLUSION_HPP
#define EDYN_PARALLEL_MERGE_MERGE_COLLISION_EXCLUSION_HPP

#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(const collision_exclusion *old_comp, collision_exclusion &new_comp, merge_context &ctx) {
    for (unsigned i = 0 ; i < new_comp.num_entities; ++i) {
        new_comp.entity[i] = ctx.map->remloc(new_comp.entity[i]);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_COLLISION_EXCLUSION_HPP
