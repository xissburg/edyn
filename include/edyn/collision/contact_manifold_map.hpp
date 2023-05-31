#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_MAP
#define EDYN_COLLISION_CONTACT_MANIFOLD_MAP

#include <map>
#include <utility>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/core/entity_pair.hpp"

namespace edyn {

/**
 * @brief Maps a pair of entities to their contact manifold.
 */
class contact_manifold_map {
public:
    contact_manifold_map(entt::registry &);

    // Cannot be copied because `entt::scoped_connection` deletes its copy ctor.
    contact_manifold_map(const contact_manifold_map &) = delete;
    contact_manifold_map(contact_manifold_map &&) = default;

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

    void clear();

private:
    std::map<entity_pair, entt::entity> m_pair_map;
    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_MAP
