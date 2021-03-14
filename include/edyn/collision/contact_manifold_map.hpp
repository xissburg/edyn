#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_MAP
#define EDYN_COLLISION_CONTACT_MANIFOLD_MAP

#include <map>
#include <utility>
#include <entt/fwd.hpp>
#include "edyn/util/entity_pair.hpp"

namespace edyn {

/**
 * Maps a pair of entities to their contact manifold.
 */
class contact_manifold_map {
public:
    contact_manifold_map(entt::registry &);

    bool contains(const entity_pair &) const;
    bool contains(entt::entity, entt::entity) const;

    void on_construct_contact_manifold(entt::registry &, entt::entity);
    void on_destroy_contact_manifold(entt::registry &, entt::entity);

private:
    std::map<entity_pair, entt::entity> m_pair_map;
};

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_MAP