#ifndef EDYN_COLLISION_BROADPHASE_WORKER_HPP
#define EDYN_COLLISION_BROADPHASE_WORKER_HPP

#include <vector>
#include <entt/fwd.hpp>
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/collision/contact_manifold_map.hpp"

namespace edyn {

class broadphase_worker {
public:
    broadphase_worker(entt::registry &);
    void update();
    tree_view view() const;

    void on_construct_aabb(entt::registry &, entt::entity);
    void on_destroy_node_id(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;
    dynamic_tree m_tree;
    contact_manifold_map m_manifold_map;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_WORKER_HPP