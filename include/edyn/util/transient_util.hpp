#ifndef EDYN_UTIL_TRANSIENT_UTIL_HPP
#define EDYN_UTIL_TRANSIENT_UTIL_HPP

#include "edyn/comp/transient.hpp"
#include <entt/core/fwd.hpp>
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename... Components>
void mark_transient(entt::registry &registry, entt::entity entity) {
    if (registry.all_of<transient>(entity)) {
        registry.patch<transient>(entity, [](transient &tr) {
            (tr.ids.push_back(entt::type_index<Components>::value()), ...);
        });
    } else {
        auto ids = std::vector<entt::id_type>{};
        ids.reserve(sizeof...(Components));
        (ids.push_back(entt::type_index<Components>::value()), ...);
        registry.emplace<transient>(entity, std::move(ids));
    }
}

}

#endif // EDYN_UTIL_TRANSIENT_UTIL_HPP
