#include "edyn/util/island_util.hpp"
#include "entt/entity/registry.hpp"

namespace edyn {

void remove_sleeping_tag_from_island(entt::registry &registry,
                                     entt::entity island_entity,
                                     const edyn::island &island) {
    registry.remove<sleeping_tag>(island_entity);
    registry.remove<sleeping_tag>(island.nodes.begin(), island.nodes.end());
    registry.remove<sleeping_tag>(island.edges.begin(), island.edges.end());
}

void wake_up_island(entt::registry &registry, entt::entity island_entity) {
    if (registry.all_of<sleeping_tag>(island_entity)) {
        auto &island = registry.get<edyn::island>(island_entity);
        remove_sleeping_tag_from_island(registry, island_entity, island);
    }
}

template<typename It>
void wake_up_island_residents_range(entt::registry &registry, It first, It last) {
    auto resident_view = registry.view<island_resident>();
    auto island_entities = entt::sparse_set{};

    for (; first != last; ++first) {
        auto entity = *first;
        if (resident_view.contains(entity)) {
            auto [resident] = resident_view.get(entity);

            if (resident.island_entity != entt::null && !island_entities.contains(resident.island_entity)) {
                island_entities.emplace(resident.island_entity);
            }
        }
    }

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

template<typename It>
void wake_up_island_residents_range_emap(entt::registry &registry, It first, It last, const entity_map &emap) {
    auto resident_view = registry.view<island_resident>();
    auto island_entities = entt::sparse_set{};

    for (; first != last; ++first) {
        auto remote_entity = *first;

        if (!emap.contains(remote_entity)) {
            continue;
        }

        auto local_entity = emap.at(remote_entity);

        if (resident_view.contains(local_entity)) {
            auto [resident] = resident_view.get(local_entity);

            if (resident.island_entity != entt::null && !island_entities.contains(resident.island_entity)) {
                island_entities.emplace(resident.island_entity);
            }
        }
    }

    for (auto island_entity : island_entities) {
        wake_up_island(registry, island_entity);
    }
}

void wake_up_island_residents(entt::registry &registry,
                              const std::vector<entt::entity> &entities,
                              const entity_map &emap) {
    wake_up_island_residents_range_emap(registry, entities.begin(), entities.end(), emap);
}

void wake_up_island_residents(entt::registry &registry,
                              const entt::sparse_set &entities,
                              const entity_map &emap) {
    wake_up_island_residents_range_emap(registry, entities.begin(), entities.end(), emap);
}

}
