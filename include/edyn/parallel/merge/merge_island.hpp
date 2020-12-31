#ifndef EDYN_PARALLEL_MERGE_MERGE_ISLAND_NODE_HPP
#define EDYN_PARALLEL_MERGE_MERGE_ISLAND_NODE_HPP

#include "edyn/comp/island.hpp"
#include "edyn/parallel/merge/merge_util.hpp"

namespace edyn {

template<merge_type MergeType>
void merge(const island_node *old_comp, island_node &new_comp, merge_context &ctx) {
    merge_unordered_set<MergeType>(old_comp, new_comp, &island_node::entities, ctx);
}

template<merge_type MergeType>
void merge(const island_container *old_comp, island_container &new_comp, merge_context &ctx) {
    merge_unordered_set<MergeType>(old_comp, new_comp, &island_container::entities, ctx);
}

template<merge_type MergeType>
void merge(const island_node_parent *old_comp, island_node_parent &new_comp, merge_context &ctx) {
    merge_unordered_set<MergeType>(old_comp, new_comp, &island_node_parent::children, ctx);
}

template<merge_type MergeType>
void merge(const island_node_child *old_comp, island_node_child &new_comp, merge_context &ctx) {
    new_comp.parent = ctx.map->remloc(new_comp.parent);
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_ISLAND_NODE_HPP