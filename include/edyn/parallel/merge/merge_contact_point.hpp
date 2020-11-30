#ifndef EDYN_PARALLEL_MERGE_MERGE_CONTACT_POINT_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONTACT_POINT_HPP

#include "edyn/collision/contact_point.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const contact_point *old_comp, contact_point &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONTACT_POINT_HPP