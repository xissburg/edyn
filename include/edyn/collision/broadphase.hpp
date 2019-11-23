#ifndef EDYN_COLLISION_BROADPHASE_HPP
#define EDYN_COLLISION_BROADPHASE_HPP

#include <entt/fwd.hpp>
#include <map>
#include <utility>

namespace edyn {

class broadphase {
public:
    broadphase(entt::registry &);
    void update();

private:
    entt::registry *registry;
    std::map<std::pair<entt::entity, entt::entity>, entt::entity> relations;
};

}

#endif // EDYN_COLLISION_BROADPHASE_HPP