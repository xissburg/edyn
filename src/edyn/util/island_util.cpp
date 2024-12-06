#include "edyn/util/island_util.hpp"
#include "edyn/comp/island.hpp"
#include "entt/entity/registry.hpp"

namespace edyn {

void remove_sleeping_tag_from_island(entt::registry &registry,
                                     entt::entity island_entity,
                                     const edyn::island &island) {
    registry.remove<sleeping_tag>(island_entity);
    registry.remove<sleeping_tag>(island.nodes.begin(), island.nodes.end());
    registry.remove<sleeping_tag>(island.edges.begin(), island.edges.end());
}

template<typename It>
entt::sparse_set collect_islands_from_residents(entt::registry &registry, It first, It last, const entity_map *emap = nullptr) {
    auto resident_view = registry.view<island_resident>();
    auto multi_resident_view = registry.view<multi_island_resident>();

    auto island_entities = entt::sparse_set{};

    for (; first != last; ++first) {
        auto entity = *first;

        if (emap) {
            if (emap->contains(entity)) {
                entity = emap->at(entity);
            } else {
                continue;
            }
        }

        if (resident_view.contains(entity)) {
            auto [resident] = resident_view.get(entity);

            if (resident.island_entity != entt::null && !island_entities.contains(resident.island_entity)) {
                island_entities.push(resident.island_entity);
            }
        } else if (multi_resident_view.contains(entity)) {
            auto [resident] = multi_resident_view.get(entity);

            for (auto island_entity : resident.island_entities) {
                if (!island_entities.contains(island_entity)) {
                    island_entities.push(island_entity);
                }
            }
        }
    }

    return island_entities;
}

entt::sparse_set collect_islands_from_residents(entt::registry &registry, const std::vector<entt::entity> &entities) {
    return collect_islands_from_residents(registry, entities.begin(), entities.end());
}

entt::sparse_set collect_islands_from_residents(entt::registry &registry, const entt::sparse_set &entities) {
    return collect_islands_from_residents(registry, entities.begin(), entities.end());
}

void wake_up_island(entt::registry &registry, entt::entity island_entity) {
    if (registry.all_of<sleeping_tag>(island_entity)) {
        auto &island = registry.get<edyn::island>(island_entity);
        remove_sleeping_tag_from_island(registry, island_entity, island);
    }
}

template<typename It>
void wake_up_island_residents_range(entt::registry &registry, It first, It last, const entity_map *emap = nullptr) {
    auto island_entities = collect_islands_from_residents(registry, first, last, emap);

    for (auto island_entity : island_entities) {
        wake_up_island(registry, island_entity);
    }
}

void wake_up_island_residents(entt::registry &registry,
                              const std::vector<entt::entity> &entities) {
    wake_up_island_residents_range(registry, entities.begin(), entities.end());
}

void wake_up_island_residents(entt::registry &registry,
                              const entt::sparse_set &entities) {
    wake_up_island_residents_range(registry, entities.begin(), entities.end());
}

void wake_up_island_resident(entt::registry &registry, entt::entity entity) {
    auto arr = std::array<entt::entity, 1>{entity};
    wake_up_island_residents_range(registry, arr.begin(), arr.end());
}

void wake_up_island_residents(entt::registry &registry,
                              const std::vector<entt::entity> &entities,
                              const entity_map &emap) {
    wake_up_island_residents_range(registry, entities.begin(), entities.end(), &emap);
}

void wake_up_island_residents(entt::registry &registry,
                              const entt::sparse_set &entities,
                              const entity_map &emap) {
    wake_up_island_residents_range(registry, entities.begin(), entities.end(), &emap);
}

}
