#ifndef EDYN_PARALLEL_MERGE_MERGE_GRAVITY_HPP
#define EDYN_PARALLEL_MERGE_MERGE_GRAVITY_HPP

#include "edyn/comp/gravity.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const gravity *old_comp, gravity &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_GRAVITY_HPP
