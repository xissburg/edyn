#ifndef EDYN_COLLISION_BROADPHASE_WORKER_HPP
#define EDYN_COLLISION_BROADPHASE_WORKER_HPP

#include <map>
#include <vector>
#include <entt/fwd.hpp>
#include "edyn/collision/contact_manifold_map.hpp"

namespace edyn {

class broadphase_worker {
public:
    broadphase_worker(entt::registry &);
    void update();

private:
    entt::registry *m_registry;
    contact_manifold_map m_manifold_map;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_WORKER_HPP