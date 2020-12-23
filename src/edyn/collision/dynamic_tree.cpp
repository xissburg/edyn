#include "edyn/collision/dynamic_tree.hpp"
#include <entt/entt.hpp>

namespace edyn {

dynamic_tree::dynamic_tree()
    : m_root(null_node_id)
    , m_free_list(null_node_id)
{}

dynamic_tree::node_id_t dynamic_tree::allocate() {
    if (m_free_list == null_node_id) {
        auto id = static_cast<node_id_t>(m_nodes.size());
        auto &node = m_nodes.emplace_back();
        node.next = null_node_id;
        node.parent = null_node_id;
        node.child1 = null_node_id;
        node.child2 = null_node_id;
        node.entity = entt::null;
        node.height = 0;
        return id;
    } else {
        auto id = m_free_list;
        auto &node = m_nodes[id];
        node.parent = null_node_id;
        node.child1 = null_node_id;
        node.child2 = null_node_id;
        node.entity = entt::null;
        node.height = 0;
        m_free_list = node.next;
        return id;
    }
}

void dynamic_tree::free(node_id_t id) {
    m_nodes[id].next = m_free_list;
    m_nodes[id].height = -1;
    m_free_list = id;
}

dynamic_tree::node_id_t dynamic_tree::create(const AABB &aabb, entt::entity entity) {
    auto id = allocate();
    auto &node = m_nodes[id];
    node.entity = entity;
    node.aabb = aabb.inset(aabb_inset);

    insert(id);

    return id;
}

void dynamic_tree::destroy(dynamic_tree::node_id_t id) {
    EDYN_ASSERT(m_nodes[id].leaf());
    remove(id);
    free(id);
}

void dynamic_tree::insert(dynamic_tree::node_id_t leaf) {
    if (m_root == null_node_id) {
        m_root = leaf;
        m_nodes[m_root].parent = null_node_id;
        return;
    }

    // Find the best sibling for this node.
    auto leaf_aabb = m_nodes[leaf].aabb;
    auto index = m_root;

    while (!m_nodes[index].leaf()) {
        auto &node = m_nodes[index];

        auto enclosing_area = enclosing_aabb(node.aabb, leaf_aabb).area();

        // Cost of creating a new parent for this node and the new leaf.
        auto cost = scalar{2} * enclosing_area;

        // Minimum cost of pushing the leaf further down the tree.
        auto inherit_cost = scalar{2} * (enclosing_area - node.aabb.area());

        // Cost of descending into child1.
        auto &child_node1 = m_nodes[node.child1];
        auto enclosing_area_child1 = enclosing_aabb(child_node1.aabb, leaf_aabb).area();

        auto cost1 = child_node1.leaf() ?
            enclosing_area_child1 + inherit_cost :
            (enclosing_area_child1 - child_node1.aabb.area()) + inherit_cost;

        // Cost of descending into child2.
        auto &child_node2 = m_nodes[node.child2];
        auto enclosing_area_child2 = enclosing_aabb(child_node2.aabb, leaf_aabb).area();

        auto cost2 = child_node2.leaf() ?
            enclosing_area_child2 + inherit_cost :
            (enclosing_area_child2 - child_node2.aabb.area()) + inherit_cost;

        // Descend according to minimum cost.
        if (cost < cost1 && cost < cost2) {
            // Best node found.
            break;
        }

        // Descend into the best child.
        index = cost1 < cost2 ? node.child1 : node.child2;
    }

    const auto sibling = index;
    auto &sibling_node = m_nodes[sibling];

    // Create new parent.
    auto old_parent = sibling_node.parent;
    auto new_parent = allocate();
    auto &parent_node = m_nodes[new_parent];
    parent_node.parent = old_parent;
    parent_node.entity = entt::null;
    parent_node.aabb = enclosing_aabb(sibling_node.aabb, leaf_aabb);
    parent_node.height = sibling_node.height + 1;

    auto &leaf_node = m_nodes[leaf];

    if (old_parent != null_node_id) {
        // The sibling was not the root.
        auto &old_parent_node = m_nodes[old_parent];
        if (old_parent_node.child1 == sibling) {
            old_parent_node.child1 = new_parent;
        } else {
            old_parent_node.child2 = new_parent;
        }

        parent_node.child1 = sibling;
        parent_node.child2 = leaf;
        sibling_node.parent = new_parent;
        leaf_node.parent = new_parent;
    } else {
        // The sibling was the root.
        parent_node.child1 = sibling;
        parent_node.child2 = leaf;
        sibling_node.parent = new_parent;
        leaf_node.parent = new_parent;
        m_root = new_parent;
    }

    // Walk back up the tree refitting AABBs.
    index = leaf_node.parent;
    while (index != null_node_id) {
        index = balance(index);
        auto &node = m_nodes[index];
        EDYN_ASSERT(node.child1 != null_node_id);
        EDYN_ASSERT(node.child2 != null_node_id);
        node.aabb = enclosing_aabb(m_nodes[node.child1].aabb, m_nodes[node.child2].aabb);
        node.height = std::max(m_nodes[node.child1].height, m_nodes[node.child2].height) + 1;
        index = node.parent;
    }
}

void dynamic_tree::remove(node_id_t leaf) {
    if (leaf == m_root) {
        m_root = null_node_id;
        return;
    }

    
}

}