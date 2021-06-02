#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_MAP
#define EDYN_COLLISION_CONTACT_MANIFOLD_MAP

#include <map>
#include <utility>
#include <entt/fwd.hpp>
#include "edyn/util/entity_pair.hpp"

namespace edyn {

/**
 * @brief Maps a pair of entities to their contact manifold.
 */
class contact_manifold_map {
public:
    contact_manifold_map(entt::registry &);

    /**
     * @brief Checks whether a contact manifold exists for a pair of entities.
     * @param pair The pair of entities.
     * @return Whether a contact manifold exits between the two entities.
     */
    bool contains(entity_pair) const;

    /*! @copydoc contains */
    bool contains(entt::entity, entt::entity) const;

    /**
     * @brief Gets the manifold entity joining a pair of entities.
     * @param pair The pair of entities.
     * @return Entity of manifold connecting the two entities.
     */
    entt::entity get(entity_pair) const;

    /*! @copydoc get */
    entt::entity get(entt::entity, entt::entity) const;

    void on_construct_contact_manifold(entt::registry &, entt::entity);
    void on_destroy_contact_manifold(entt::registry &, entt::entity);

private:
    std::map<entity_pair, entt::entity> m_pair_map;
};

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_MAP
