#ifndef EDYN_COLLISION_QUERY_TREE_HPP
#define EDYN_COLLISION_QUERY_TREE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

template<typename Tree, typename NodeIdType, typename TestFunc, typename VisitFunc>
void traverse_tree(const Tree &tree, NodeIdType root_id, NodeIdType null_node_id,
                   TestFunc test_func, VisitFunc visit_func) {
    std::vector<NodeIdType> stack;
    stack.push_back(root_id);

    while (!stack.empty()) {
        auto id = stack.back();
        stack.pop_back();

        if (id == null_node_id) {
            continue;
        }

        auto &node = tree.get_node(id);

        if (test_func(node)) {
            if (node.leaf()) {
                visit_func(id);
            } else {
                stack.push_back(node.child1);
                stack.push_back(node.child2);
            }
        }
    }
}

template<typename Tree, typename NodeIdType, typename Func>
void query_tree(const Tree &tree, NodeIdType root_id, NodeIdType null_node_id,
                const AABB &aabb, Func func) {
    traverse_tree(tree, root_id, null_node_id, [&](auto &node) {
        return intersect(node.aabb, aabb);
    }, func);
}

template<typename Tree, typename NodeIdType, typename Func>
void raycast_tree(const Tree &tree, NodeIdType root_id, NodeIdType null_node_id,
                  const vector3 &p0, const vector3 &p1, Func func) {
    traverse_tree(tree, root_id, null_node_id, [&](auto &node) {
        return intersect_segment_aabb(p0, p1, node.aabb.min, node.aabb.max);
    }, func);
}

}

#endif // EDYN_COLLISION_QUERY_TREE_HPP
