#ifndef EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP

#include "edyn/comp/constraint.hpp"
#include "edyn/parallel/merge/merge_util.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const constraint *old_comp, constraint &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }

    merge_array<MergeType>(old_comp, new_comp, &constraint::row, ctx);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_HPP