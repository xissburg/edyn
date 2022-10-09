#ifndef EDYN_COLLISION_DYNAMIC_TREE_HPP
#define EDYN_COLLISION_DYNAMIC_TREE_HPP

#include <vector>
#include <cstdint>
#include <entt/entity/fwd.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/collision/query_tree.hpp"

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
    constexpr static vector3 aabb_inset = vector3_one * scalar(-0.1);

private:
    tree_node_id_t allocate();
    void free(tree_node_id_t);
    tree_node_id_t best(const AABB &);
    void insert(tree_node_id_t);
    void remove(tree_node_id_t);
    void refit(tree_node_id_t);
    tree_node_id_t balance(tree_node_id_t);

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
    tree_node_id_t create(const AABB &, entt::entity);

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
    bool move(tree_node_id_t, const AABB &);

    /**
     * @brief Destroys a node with the given id.
     *
     * @param id The node id.
     */
    void destroy(tree_node_id_t);

    /**
     * @brief Call `func` for all nodes that overlap `aabb`.
     *
     * @tparam Func Inferred function parameter type.
     * @param aabb The query AABB.
     * @param func Function to be called for each overlapping node. It takes a
     * single `tree_node_id_t` parameter.
     */
    template<typename Func>
    void query(const AABB &aabb, Func func) const;

    /**
     * @brief Call `func` for all nodes that intersect the segment [p0, p1].
     * @param p0 First point in the segment.
     * @param p1 Second point in the segment.
     * @param func Function to be called for each overlapping node. It takes a
     * single `tree_node_id_t` parameter.
     */
    template<typename Func>
    void raycast(vector3 p0, vector3 p1, Func func) const;

    /**
     * @brief Gets a tree node.
     *
     * @param id The node id.
     * @return A reference to the tree node.
     */
    const tree_node & get_node(tree_node_id_t) const;

    void clear();

private:
    tree_node_id_t m_root;

    std::vector<tree_node> m_nodes;
    tree_node_id_t m_free_list;
};

template<typename Func>
void dynamic_tree::query(const AABB &aabb, Func func) const {
    query_tree(*this, m_root, null_tree_node_id, aabb, func);
}

template<typename Func>
void dynamic_tree::raycast(vector3 p0, vector3 p1, Func func) const {
    raycast_tree(*this, m_root, null_tree_node_id, p0, p1, func);
}

}

#endif // EDYN_COLLISION_DYNAMIC_TREE_HPP
