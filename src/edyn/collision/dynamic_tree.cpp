#include "edyn/collision/dynamic_tree.hpp"

namespace edyn {

dynamic_tree::node_t dynamic_tree::create() {

}

dynamic_tree::node_t dynamic_tree::create(const AABB &aabb, entt::entity entity) {
    auto id = create();
    auto &node = m_nodes[id];
    node.entity = entity;
    node.aabb = aabb.inset(aabb_inset);

    insert(id);

    return id;
}

}