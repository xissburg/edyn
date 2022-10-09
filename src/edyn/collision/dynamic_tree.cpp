#include "edyn/collision/dynamic_tree.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

dynamic_tree::dynamic_tree()
    : m_root(null_tree_node_id)
    , m_free_list(null_tree_node_id)
{}

tree_node_id_t dynamic_tree::allocate() {
    if (m_free_list == null_tree_node_id) {
        auto id = static_cast<tree_node_id_t>(m_nodes.size());
        auto &node = m_nodes.emplace_back();
        node.next = null_tree_node_id;
        node.parent = null_tree_node_id;
        node.child1 = null_tree_node_id;
        node.child2 = null_tree_node_id;
        node.entity = entt::null;
        node.height = 0;
        return id;
    } else {
        auto id = m_free_list;
        auto &node = m_nodes[id];
        node.parent = null_tree_node_id;
        node.child1 = null_tree_node_id;
        node.child2 = null_tree_node_id;
        node.entity = entt::null;
        node.height = 0;
        m_free_list = node.next;
        return id;
    }
}

void dynamic_tree::free(tree_node_id_t id) {
    m_nodes[id].next = m_free_list;
    m_nodes[id].height = -1;
    m_nodes[id].entity = entt::null;
    m_free_list = id;
}

tree_node_id_t dynamic_tree::create(const AABB &aabb, entt::entity entity) {
    auto id = allocate();
    auto &node = m_nodes[id];
    node.entity = entity;
    node.aabb = aabb.inset(aabb_inset);

    insert(id);

    return id;
}

void dynamic_tree::destroy(tree_node_id_t id) {
    EDYN_ASSERT(m_nodes[id].leaf());
    remove(id);
    free(id);
}

bool dynamic_tree::move(tree_node_id_t id, const AABB &aabb) {
    auto &node = m_nodes[id];
    EDYN_ASSERT(node.leaf());

    // If the entity's AABB hasn't moved outside the inflated node AABB,
    // nothing has to be done.
    if (node.aabb.contains(aabb)) {
        return false;
    }

    // Extend AABB.
    auto offset_aabb = aabb.inset(aabb_inset);

    // Reinsert node with updated AABB.
    remove(id);
    m_nodes[id].aabb = offset_aabb;
    insert(id);

    // It moved.
    return true;
}

tree_node_id_t dynamic_tree::best(const AABB &aabb) {
    // Find leaf node that would be the best sibling for a new leaf with the
    // given AABB.
    auto id = m_root;

    while (!m_nodes[id].leaf()) {
        auto &node = m_nodes[id];

        auto enclosing_area = enclosing_aabb(node.aabb, aabb).area();

        // Cost of creating a new parent for this node and the new leaf.
        auto cost = scalar{2} * enclosing_area;

        // Minimum cost of pushing the leaf further down the tree.
        auto inherit_cost = scalar{2} * (enclosing_area - node.aabb.area());

        // Cost of descending into child1.
        auto &child_node1 = m_nodes[node.child1];
        auto enclosing_area_child1 = enclosing_aabb(child_node1.aabb, aabb).area();

        auto cost1 = child_node1.leaf() ?
            enclosing_area_child1 + inherit_cost :
            (enclosing_area_child1 - child_node1.aabb.area()) + inherit_cost;

        // Cost of descending into child2.
        auto &child_node2 = m_nodes[node.child2];
        auto enclosing_area_child2 = enclosing_aabb(child_node2.aabb, aabb).area();

        auto cost2 = child_node2.leaf() ?
            enclosing_area_child2 + inherit_cost :
            (enclosing_area_child2 - child_node2.aabb.area()) + inherit_cost;

        // Descend according to minimum cost.
        if (cost < cost1 && cost < cost2) {
            // Best node found.
            break;
        }

        // Descend into the best child.
        id = cost1 < cost2 ? node.child1 : node.child2;
    }

    return id;
}

void dynamic_tree::insert(tree_node_id_t leaf) {
    if (m_root == null_tree_node_id) {
        m_root = leaf;
        m_nodes[m_root].parent = null_tree_node_id;
        return;
    }

    // Find the best sibling for this node.
    auto leaf_aabb = m_nodes[leaf].aabb;
    const auto sibling = best(leaf_aabb);

    // Create new parent.
    // Careful keeping references to nodes in `m_nodes` created before the call
    // to `allocate` since it emplaces a new entry into the vector.
    auto old_parent = m_nodes[sibling].parent;
    auto new_parent = allocate();
    auto &parent_node = m_nodes[new_parent];
    parent_node.parent = old_parent;

    auto &sibling_node = m_nodes[sibling];
    parent_node.aabb = enclosing_aabb(sibling_node.aabb, leaf_aabb);
    parent_node.height = sibling_node.height + 1;

    auto &leaf_node = m_nodes[leaf];

    if (old_parent != null_tree_node_id) {
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
    refit(leaf_node.parent);
}

void dynamic_tree::remove(tree_node_id_t leaf) {
    if (leaf == m_root) {
        m_root = null_tree_node_id;
        return;
    }

    auto &node = m_nodes[leaf];
    auto parent = node.parent;
    auto &parent_node = m_nodes[parent];
    auto sibling = parent_node.child1 == leaf ? parent_node.child2 : parent_node.child1;
    EDYN_ASSERT(sibling != null_tree_node_id);
    auto &sibling_node = m_nodes[sibling];

    if (parent == m_root) {
        m_root = sibling;
        sibling_node.parent = null_tree_node_id;
        free(parent);
    } else {
        // Destroy parent and connect sibling to grand parent.
        auto grandpa = parent_node.parent;
        EDYN_ASSERT(grandpa != null_tree_node_id);
        auto &grandpa_node = m_nodes[grandpa];

        if (grandpa_node.child1 == parent) {
            grandpa_node.child1 = sibling;
        } else {
            grandpa_node.child2 = sibling;
        }

        sibling_node.parent = parent_node.parent;
        free(parent);

        refit(grandpa);
    }
}

void dynamic_tree::refit(tree_node_id_t id) {
    while (id != null_tree_node_id) {
        id = balance(id);
        auto &node = m_nodes[id];
        EDYN_ASSERT(node.child1 != null_tree_node_id);
        EDYN_ASSERT(node.child2 != null_tree_node_id);
        node.aabb = enclosing_aabb(m_nodes[node.child1].aabb, m_nodes[node.child2].aabb);
        node.height = std::max(m_nodes[node.child1].height, m_nodes[node.child2].height) + 1;
        id = node.parent;
    }
}

tree_node_id_t dynamic_tree::balance(tree_node_id_t idA) {
    EDYN_ASSERT(idA != null_tree_node_id);

    auto &nodeA = m_nodes[idA];

    if (nodeA.leaf() || nodeA.height < 2) {
        return idA;
    }

    auto idB = nodeA.child1;
    auto idC = nodeA.child2;
    auto &nodeB = m_nodes[idB];
    auto &nodeC = m_nodes[idC];

    auto balance = nodeC.height - nodeB.height;

    // Rotate C up.
    if (balance > 1) {
        auto idF = nodeC.child1;
        auto idG = nodeC.child2;
        auto &nodeF = m_nodes[idF];
        auto &nodeG = m_nodes[idG];

        // Swap A and C.
        nodeC.child1 = idA;
        nodeC.parent = nodeA.parent;
        nodeA.parent = idC;

        if (nodeC.parent != null_tree_node_id) {
            // A's old parent should point to C.
            auto &parent_nodeC = m_nodes[nodeC.parent];
            if (parent_nodeC.child1 == idA) {
                parent_nodeC.child1 = idC;
            } else {
                EDYN_ASSERT(parent_nodeC.child2 == idA);
                parent_nodeC.child2 = idC;
            }
        } else {
            m_root = idC;
        }

        // Rotate.
        if (nodeF.height > nodeG.height) {
            nodeC.child2 = idF;
            nodeA.child2 = idG;
            nodeG.parent = idA;
            nodeA.aabb = enclosing_aabb(nodeB.aabb, nodeG.aabb);
            nodeC.aabb = enclosing_aabb(nodeA.aabb, nodeF.aabb);

            nodeA.height = std::max(nodeB.height, nodeG.height) + 1;
            nodeC.height = std::max(nodeA.height, nodeF.height) + 1;
        } else {
            nodeC.child2 = idG;
            nodeA.child2 = idF;
            nodeF.parent = idA;
            nodeA.aabb = enclosing_aabb(nodeB.aabb, nodeF.aabb);
            nodeC.aabb = enclosing_aabb(nodeA.aabb, nodeG.aabb);

            nodeA.height = std::max(nodeB.height, nodeF.height) + 1;
            nodeC.height = std::max(nodeA.height, nodeG.height) + 1;
        }

        return idC;
    }

    // Rotate B up.
    if (balance < -1) {
        auto idD = nodeB.child1;
        auto idE = nodeB.child2;
        auto &nodeD = m_nodes[idD];
        auto &nodeE = m_nodes[idE];

        // Swap A and B.
        nodeB.child1 = idA;
        nodeB.parent = nodeA.parent;
        nodeA.parent = idB;

        if (nodeB.parent != null_tree_node_id) {
            // A's old parent should point to B.
            auto &parent_nodeB = m_nodes[nodeB.parent];
            if (parent_nodeB.child1 == idA) {
                parent_nodeB.child1 = idB;
            } else {
                EDYN_ASSERT(parent_nodeB.child2 == idA);
                parent_nodeB.child2 = idB;
            }
        } else {
            m_root = idB;
        }

        // Rotate.
        if (nodeD.height > nodeE.height) {
            nodeB.child2 = idD;
            nodeA.child1 = idE;
            nodeE.parent = idA;
            nodeA.aabb = enclosing_aabb(nodeC.aabb, nodeE.aabb);
            nodeB.aabb = enclosing_aabb(nodeA.aabb, nodeD.aabb);

            nodeA.height = std::max(nodeC.height, nodeE.height) + 1;
            nodeB.height = std::max(nodeA.height, nodeD.height) + 1;
        } else {
            nodeB.child2 = idE;
            nodeA.child1 = idD;
            nodeD.parent = idA;
            nodeA.aabb = enclosing_aabb(nodeC.aabb, nodeD.aabb);
            nodeB.aabb = enclosing_aabb(nodeA.aabb, nodeE.aabb);

            nodeA.height = std::max(nodeC.height, nodeD.height) + 1;
            nodeB.height = std::max(nodeA.height, nodeE.height) + 1;
        }

        return idB;
    }

    return idA;
}

const tree_node & dynamic_tree::get_node(tree_node_id_t id) const {
    return m_nodes[id];
}

void dynamic_tree::clear() {
    m_root = null_tree_node_id;
    m_free_list = null_tree_node_id;

    if (!m_nodes.empty()) {
        m_free_list = 0;

        for (tree_node_id_t id = 0; id < m_nodes.size(); ++id) {
            auto &node = m_nodes[id];
            node.entity = entt::null;
            node.height = -1;
            node.next = id + 1;
        }

        m_nodes.back().next = null_tree_node_id;
    }
}

}
