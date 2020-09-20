#ifndef EDYN_COLLISION_SIMPLE_BROADPHASE_HPP
#define EDYN_COLLISION_SIMPLE_BROADPHASE_HPP

#include <entt/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include <map>
#include <utility>

namespace edyn {

class simple_broadphase {
public:
    simple_broadphase(entt::registry &);
    void update();

    using intersect_func_t = void(entt::entity, entt::entity);

    entt::sink<intersect_func_t> intersect_sink() {
        return {m_intersect_signal};
    }

private:
    entt::registry *m_registry;
    entt::sigh<intersect_func_t> m_intersect_signal;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_SIMPLE_BROADPHASE_HPP