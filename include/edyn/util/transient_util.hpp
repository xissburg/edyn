#ifndef EDYN_UTIL_TRANSIENT_UTIL_HPP
#define EDYN_UTIL_TRANSIENT_UTIL_HPP

#include "edyn/comp/transient.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/util/settings_util.hpp"
#include <entt/core/fwd.hpp>
#include <entt/entity/registry.hpp>

namespace edyn {

/**
 * @brief Marks the given component types as transient, i.e. they will be
 * synchronized with the main registry after every simulation step.
 * @tparam Components Component types.
 * @param registry Data source.
 * @param entity The entity containing the given components.
 */
template<typename... Components>
void mark_transient(entt::registry &registry, entt::entity entity) {
    if (registry.all_of<transient>(entity)) {
        registry.patch<transient>(entity, [](transient &tr) {
            (tr.ids.push_back(entt::type_id<Components>().hash()), ...);
        });
    } else {
        auto ids = std::vector<entt::id_type>{};
        ids.reserve(sizeof...(Components));
        (ids.push_back(entt::type_id<Components>().hash()), ...);
        registry.emplace<transient>(entity, std::move(ids));
    }
}

/**
 * @brief Marks the given component types as transient for every new contact point.
 * @tparam Components Component types.
 * @param registry Data source.
 */
template<typename... Components>
void set_contact_point_transient(entt::registry &registry) {
    auto transient_contact = transient{};
    transient_contact.ids.reserve(sizeof...(Components));
    (transient_contact.ids.push_back(entt::type_id<Components>().hash()), ...);
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.async_settings->contact_points_transient = std::move(transient_contact);
    refresh_settings(registry);
}

}

#endif // EDYN_UTIL_TRANSIENT_UTIL_HPP
