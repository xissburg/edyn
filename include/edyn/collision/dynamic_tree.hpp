#ifndef EDYN_COLLISION_DYNAMIC_TREE_HPP
#define EDYN_COLLISION_DYNAMIC_TREE_HPP

#include <vector>
#include <cstdint>
#include <entt/fwd.hpp>
#include "edyn/comp/aabb.hpp"

namespace edyn {

/**
 * @brief Dynamic bounding volume hierarchy tree for broad-phase collision detection.
 * 
 * References: 
 *  - Dynamic Bounding Volume Hierarchies, Erin Catto, GDC 2019
 *  - Box2D b2DynamicTree 
 *  - https://github.com/erincatto/box2d/blob/master/include/box2d/b2_dynamic_tree.h
 *  - https://github.com/erincatto/box2d/blob/master/src/collision/b2_dynamic_tree.cpp
 */
class dynamic_tree final {
public:
    using node_id_t = uint32_t;
    constexpr static node_id_t null_node_id = UINT32_MAX;
    constexpr static vector3 aabb_inset = vector3_one * scalar{-0.1};

    struct tree_node {
        entt::entity entity;
        AABB aabb;

        union {
            node_id_t parent;
            node_id_t next;
        };

        node_id_t child1;
        node_id_t child2;

        // Height from the bottom of the tree, i.e. leaf = 0. If free, -1.
        int height;

        bool leaf() const {
            return child1 == null_node_id;
        }
    };

private:
    node_id_t allocate();
    void free(node_id_t);
    node_id_t best(const AABB &);
    void insert(node_id_t);
    void remove(node_id_t);
    void refit(node_id_t);
    node_id_t balance(node_id_t);

public:
    dynamic_tree();

    /**
     * @brief Creates a new leaf node with the given AABB and entity.
     * 
     * Inserts it at the best place into the tree.
     * 
     * @param aabb The leaf node AABB.
     * @param entity The entity associated with this node.
     * @return The new node id.
     */
    node_id_t create(const AABB &, entt::entity);

    /**
     * @brief Attempts to change the AABB of a node.
     * 
     * If the provided AABB is not fully contained within the node's AABB (which
     * is inflated internally), it changes the AABB of the node and reinserts
     * it into the tree.
     * 
     * @param id The node id.
     * @param aabb The new AABB.
     * @return Whether the AABB was changed.
     */
    bool move(node_id_t, const AABB &);

    /**
     * @brief Destroys a node with the given id.
     * 
     * @param id The node id.
     */
    void destroy(node_id_t);

    /**
     * @brief Call `func` for all nodes that overlap `aabb`.
     * 
     * @tparam Func Inferred function parameter type.
     * @param aabb The query AABB.
     * @param func Function to be called for each overlapping node. It takes a
     * single `dynamic_tree::node_id_t` parameter and returns a boolean which stops
     * the query if false.
     */
    template<typename Func>
    void query(const AABB &aabb, Func func);

    /**
     * @brief Gets a tree node.
     * 
     * @param id The node id.
     * @return A reference to the tree node.
     */
    const tree_node & get_node(node_id_t) const;

private:
    node_id_t m_root;

    std::vector<tree_node> m_nodes;
    node_id_t m_free_list;
};

template<typename Func>
void dynamic_tree::query(const AABB &aabb, Func func) {
    std::vector<node_id_t> stack;
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

}

#endif // EDYN_COLLISION_DYNAMIC_TREE_HPP