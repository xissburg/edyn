#ifndef EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
#define EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP

#include "edyn/collision/tree_view.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(const tree_view *old_comp, tree_view &new_comp, merge_context &ctx) {
    new_comp.each([&] (tree_view::tree_node &node) {
        node.entity = ctx.map->remloc(node.entity);
    });
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
