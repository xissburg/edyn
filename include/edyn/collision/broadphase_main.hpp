#ifndef EDYN_COLLISION_BROADPHASE_MAIN_HPP
#define EDYN_COLLISION_BROADPHASE_MAIN_HPP

#include <map>
#include <vector>
#include <entt/fwd.hpp>
#include <entt/signal/sigh.hpp>

namespace edyn {

class broadphase_main {
public:
    broadphase_main(entt::registry &);
    void update();

    void on_construct_contact_manifold(entt::registry &, entt::entity);
    void on_destroy_contact_manifold(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;

    // A cache that maps pairs of entities to their manifolds for quick look up.
    std::map<std::pair<entt::entity, entt::entity>, entt::entity> m_manifold_map;
    std::vector<entt::scoped_connection> m_connections;

    bool should_collide(entt::entity, entt::entity) const;
};

}

#endif // EDYN_COLLISION_BROADPHASE_MAIN_HPP