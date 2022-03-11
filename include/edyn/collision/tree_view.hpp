#ifndef EDYN_COLLISION_TREE_VIEW_HPP
#define EDYN_COLLISION_TREE_VIEW_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/collision/tree_node.hpp"
#include "edyn/collision/query_tree.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

/**
 * @brief View of a tree.
 *
 * Can be used to take a snapshot of a `dynamic_tree` and share it with other
 * parts of the application.
 */
class tree_view {
public:
    struct tree_node {
        entt::entity entity;
        AABB aabb;

        tree_node_id_t child1;
        tree_node_id_t child2;

        bool leaf() const {
            return child1 == null_tree_node_id;
        }
    };

    /**
     * @brief Creates an empty tree view.
     */
    tree_view()
        : m_root(null_tree_node_id)
    {}

    /**
     * @brief Initializes a `tree_view` with the given nodes.
     * @param nodes All tree nodes.
     * @param root_id The id of the root node in the vector of nodes.
     */
    tree_view(const std::vector<tree_node> &nodes, tree_node_id_t root_id)
        : m_nodes(nodes)
        , m_root(root_id)
    {}

    /**
     * @brief Calls the given function for each leaf node that intersects the
     * provided aabb.
     * @tparam Func Type of the function object to invoke.
     * @param aabb The AABB to query.
     * @param func Function that takes one parameter of type `tree_node_id`.
     */
    template<typename Func>
    void query(const AABB &aabb, Func func) const;

    template<typename Func>
    void raycast(vector3 p0, vector3 p1, Func func) const;

    /**
     * @brief Calls the given function for each leaf node.
     * @tparam Func Type of the function object to invoke.
     * @param func Function that takes one parameter of type `tree_view::tree_node &`.
     */
    template<typename Func>
    void each(Func func) const;

    /*! @copydoc each */
    template<typename Func>
    void each(Func func);

    /**
     * @brief Returns a reference to the tree node for the give id.
     * @return Reference to the node for the requested node id.
     */
    const tree_node & get_node(tree_node_id_t id) const {
        return m_nodes[id];
    }

    /**
     * @brief Returns the node if of the root node.
     * @return Node id of the root.
     */
    tree_node_id_t root_id() const {
        return m_root;
    }

    /**
     * @brief Returns the AABB of the root node, or a zero AABB if the tree is
     * empty.
     * @return The AABB of the root node.
     */
    AABB root_aabb() const {
        if (m_root != null_tree_node_id) {
            return m_nodes[m_root].aabb;
        }

        return {vector3_zero, vector3_zero};
    }

    /**
     * @brief Get the total number of nodes in this tree, including empty nodes.
     *
     * Some nodes in `m_nodes` are part of the "free list" (see `dynamic_tree::m_free_list`)
     * thus this is not always exactly the total number of nodes in the tree but
     * it provides an inexpensive way to check if two trees differ greatly in size.
     *
     * @return Approximate number of nodes in the tree.
     */
    size_t size() const {
        return m_nodes.size();
    }

    /**
     * @brief Returns the number of leaves in the tree.
     * @return Number of leaves in the tree.
     */
    size_t count_leaves() const {
        size_t count = 0;
        for (auto &node : m_nodes) {
            count += static_cast<size_t>(node.leaf());
        }
        return count;
    }

    std::vector<tree_node> m_nodes;
    tree_node_id_t m_root;
};

template<typename Func>
void tree_view::query(const AABB &aabb, Func func) const {
    query_tree(*this, m_root, null_tree_node_id, aabb, func);
}

template<typename Func>
void tree_view::each(Func func) const {
    for (const auto &node : m_nodes) {
        if (node.entity != entt::null) {
            func(node);
        }
    }
}

template<typename Func>
void tree_view::each(Func func) {
    for (auto &node : m_nodes) {
        if (node.entity != entt::null) {
            func(node);
        }
    }
}

template<typename Func>
void tree_view::raycast(vector3 p0, vector3 p1, Func func) const {
    raycast_tree(*this, m_root, null_tree_node_id, p0, p1, func);
}

}

#endif // EDYN_COLLISION_TREE_VIEW_HPP
