#ifndef EDYN_COLLISION_TREE_VIEW_HPP
#define EDYN_COLLISION_TREE_VIEW_HPP

#include <vector>
#include "edyn/collision/tree_node.hpp"

namespace edyn {

class tree_view {
public:
    struct tree_node {
        entt::entity entity;
        AABB aabb;

        tree_node_id_t child1;
        tree_node_id_t child2;

        bool leaf() const {
            return child1 == null_node_id;
        }
    };

    tree_view()
        : m_root(null_node_id)
    {}

    tree_view(const std::vector<tree_node> &nodes, tree_node_id_t root_id)
        : m_nodes(nodes)
        , m_root(root_id)
    {}

    template<typename Func>
    void query(const AABB &aabb, Func func) const;

    template<typename Func>
    void each(Func func) const;

    template<typename Func>
    void each(Func func);

    const tree_node & get_node(tree_node_id_t id) const {
        return m_nodes[id];
    }

    tree_node_id_t root_id() const {
        return m_root;
    }

    AABB root_aabb() const {
        if (m_root != null_node_id) {
            return m_nodes[m_root].aabb;
        }

        return {vector3_zero, vector3_zero};
    }

private:
    std::vector<tree_node> m_nodes;
    tree_node_id_t m_root;
};

template<typename Func>
void tree_view::query(const AABB &aabb, Func func) const {
    std::vector<tree_node_id_t> stack;
    stack.reserve(m_nodes.size());
    stack.push_back(m_root);

    while (!stack.empty()) {
        auto id = stack.back();
        stack.pop_back();

        if (id == null_node_id) continue;

        auto &node = m_nodes[id];

        if (intersect(node.aabb, aabb)) {
            if (node.leaf()) {
                if (!func(id)) {
                    return;
                }
            } else {
                stack.push_back(node.child1);
                stack.push_back(node.child2);
            }
        }
    }
}

template<typename Func>
void tree_view::each(Func func) const {
    for (const auto &node : m_nodes) {
        if (node.leaf()) {
            func(node);
        }
    }
}

template<typename Func>
void tree_view::each(Func func) {
    for (auto &node : m_nodes) {
        if (node.leaf()) {
            func(node);
        }
    }
}

}

#endif // EDYN_COLLISION_TREE_VIEW_HPP