#ifndef EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP
#define EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP

#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/merge/merge_util.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const contact_manifold *old_comp, contact_manifold &new_comp, merge_context &ctx) {
    for (auto &entity : new_comp.body) {
        entity = ctx.map->remloc(entity);
    }

    merge_array<MergeType>(old_comp, new_comp, &contact_manifold::point, ctx);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_CONTACT_MANIFOLD_HPP