#ifndef EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_ROW_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_ROW_HPP

#include "edyn/comp/constraint_row.hpp"
#include "edyn/parallel/merge/merge_util.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const constraint_row *old_comp, constraint_row &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.entity) {
        entity = ctx.map->remloc(entity);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONSTRAINT_ROW_HPP