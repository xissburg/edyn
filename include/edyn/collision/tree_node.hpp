#ifndef EDYN_COLLISION_TREE_NODE_HPP
#define EDYN_COLLISION_TREE_NODE_HPP

#include <cstdint>
#include <limits>
#include <entt/entity/fwd.hpp>
#include "edyn/comp/aabb.hpp"

namespace edyn {

using tree_node_id_t = uint32_t;
constexpr static tree_node_id_t null_tree_node_id = std::numeric_limits<tree_node_id_t>::max();

struct tree_node {
    entt::entity entity;
    AABB aabb;

    union {
        tree_node_id_t parent;
        tree_node_id_t next;
    };

    tree_node_id_t child1;
    tree_node_id_t child2;

    // Height from the bottom of the tree, i.e. leaf = 0. If free, -1.
    int height;

    bool leaf() const {
        return child1 == null_tree_node_id;
    }
};

}

#endif // EDYN_COLLISION_TREE_NODE_HPP
