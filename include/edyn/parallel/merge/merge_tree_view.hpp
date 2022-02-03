#ifndef EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
#define EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP

#include "edyn/collision/tree_view.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<> inline
void merge(tree_view &new_comp, const entity_map &emap) {
    new_comp.each([&] (tree_view::tree_node &node) {
        node.entity = emap.remloc(node.entity);
    });
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_TREE_VIEW_HPP
