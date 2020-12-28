#ifndef EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
#define EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP

#include "edyn/util/entity_map.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/parallel/merge/merge_util.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const tree_view *old_comp, tree_view &new_comp, merge_context &ctx) {
    new_comp.each([&] (tree_view::tree_node &node) {
        node.entity = ctx.map->remloc(node.entity);
    });
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
