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

void wake_up_island_residents(entt::registry &registry,
                              const std::vector<entt::entity> &entities) {
    auto resident_view = registry.view<island_resident>();
    auto multi_resident_view = registry.view<multi_island_resident>();

    for (auto entity : entities) {
        if (resident_view.contains(entity)) {
            auto [resident] = resident_view.get(entity);

            if (resident.island_entity != entt::null) {
                wake_up_island(registry, resident.island_entity);
            }
        } else {
            auto [resident] = multi_resident_view.get(entity);

            for (auto island_entity : resident.island_entities) {
                wake_up_island(registry, island_entity);
            }
        }
    }
}

void wake_up_island_residents(entt::registry &registry,
                              const std::vector<entt::entity> &entities,
                              const entity_map &emap) {
    auto resident_view = registry.view<island_resident>();
    auto multi_resident_view = registry.view<multi_island_resident>();

    for (auto remote_entity : entities) {
        if (!emap.contains(remote_entity)) {
            continue;
        }

        auto local_entity = emap.at(remote_entity);

        if (resident_view.contains(local_entity)) {
            auto [resident] = resident_view.get(local_entity);

            if (resident.island_entity != entt::null) {
                wake_up_island(registry, resident.island_entity);
            }
        } else {
            auto [resident] = multi_resident_view.get(local_entity);

            for (auto island_entity : resident.island_entities) {
                wake_up_island(registry, island_entity);
            }
        }
    }
}

}
