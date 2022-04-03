#ifndef EDYN_UTIL_ISLAND_UTIL_HPP
#define EDYN_UTIL_ISLAND_UTIL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/island.hpp"

namespace edyn {

template<typename It>
entt::sparse_set collect_islands_from_residents(entt::registry &registry, It first_entity, It last_entity) {
    entt::sparse_set island_entities;

    for (auto it = first_entity; it != last_entity; ++it) {
        auto entity = *it;

        if (auto *resident = registry.try_get<island_resident>(entity)) {
            if (resident->island_entity != entt::null && !island_entities.contains(resident->island_entity)) {
                island_entities.emplace(resident->island_entity);
            }
        } else if (auto *resident = registry.try_get<multi_island_resident>(entity)) {
            for (auto island_entity : resident->island_entities) {
                if (!island_entities.contains(island_entity)) {
                    island_entities.emplace(island_entity);
                }
            }
        }
    }

    return island_entities;
}

}

#endif // EDYN_UTIL_ISLAND_UTIL_HPP
