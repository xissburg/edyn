#ifndef EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
#define EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP

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
void contact_manifold_each_point(ContactView cp_view, entt::entity contact_list_head_entity, Func func) {
    if (contact_list_head_entity == entt::null) return;

    auto *cp = &cp_view.template get<contact_point_list>(contact_list_head_entity);
    func(contact_list_head_entity);

    while (cp->next != entt::null) {
        auto contact_entity = cp->next;
        cp = &cp_view.template get<contact_point_list>(contact_entity);
        func(contact_entity);
    }
}

template<typename Func>
void contact_manifold_each_point(entt::registry &registry, entt::entity contact_list_head_entity, Func func) {
    if (contact_list_head_entity != entt::null) {
        auto cp_view = registry.view<contact_point_list>();
        contact_manifold_each_point(cp_view, contact_list_head_entity, func);
    }
}

template<typename Func>
void contact_manifold_each_point(const entt::registry &registry, entt::entity contact_list_head_entity, Func func) {
    contact_manifold_each_point(const_cast<entt::registry &>(registry), contact_list_head_entity, func);
}

}

#endif // EDYN_UTIL_CONTACT_MANIFOLD_UTIL_HPP
