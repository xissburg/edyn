#ifndef EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
#define EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP

#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/core/entity_pair.hpp"
#include <entt/entity/entity.hpp>
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Checks whether there is a contact manifold connecting the two entities.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Whether a contact manifold exists between the two entities.
 */
bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc manifold_exists */
bool manifold_exists(entt::registry &registry, entity_pair entities);

/**
 * @brief Get contact manifold entity for a pair of entities.
 * Asserts if the manifold does not exist.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Contact manifold entity.
 */
entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc get_manifold_entity */
entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities);

template<typename ContactView, typename Func>
void contact_point_for_each(ContactView cp_view, entt::entity contact_entity, Func func) {
    while (contact_entity != entt::null) {
        auto next_entity = cp_view.template get<contact_point_list>(contact_entity).next;
        func(contact_entity);
        contact_entity = next_entity;
    }
}

template<typename Func>
void contact_point_for_each(entt::registry &registry, entt::entity contact_entity, Func func) {
    auto cp_view = registry.view<contact_point_list>();
    contact_point_for_each(cp_view, contact_entity, func);
}

template<typename Func>
void contact_point_for_each(const entt::registry &registry, entt::entity contact_entity, Func func) {
    contact_point_for_each(const_cast<entt::registry &>(registry), contact_entity, func);
}

template<typename Func>
void contact_manifold_each_point(entt::registry &registry, entt::entity manifold_entity, Func func) {
    auto &manifold_state = registry.get<contact_manifold_state>(manifold_entity);
    contact_point_for_each(registry, manifold_state.contact_entity, func);
}

template<typename Func>
void contact_manifold_each_point(const entt::registry &registry, entt::entity manifold_entity, Func func) {
    contact_manifold_each_point(const_cast<entt::registry &>(registry), manifold_entity, func);
}

}

#endif // EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
