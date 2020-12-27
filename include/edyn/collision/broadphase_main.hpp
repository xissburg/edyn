#ifndef EDYN_COLLISION_BROADPHASE_MAIN_HPP
#define EDYN_COLLISION_BROADPHASE_MAIN_HPP

#include <map>
#include <vector>
#include <entt/fwd.hpp>
#include "edyn/collision/dynamic_tree.hpp"
#include "edyn/collision/contact_manifold_map.hpp"

namespace edyn {

class broadphase_main {
public:
    broadphase_main(entt::registry &);
    void update();

private:
    entt::registry *m_registry;
    dynamic_tree m_tree;
    contact_manifold_map m_manifold_map;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_MAIN_HPP