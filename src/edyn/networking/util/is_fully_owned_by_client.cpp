#include "edyn/networking/util/is_fully_owned_by_client.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity) {
    auto &client = registry.get<remote_client>(client_entity);

    if (!client.allow_full_ownership) {
        return false;
    }

    auto owner_view = registry.view<entity_owner>();

    if (auto *resident = registry.try_get<island_resident>(entity)) {
        auto [island_owner] = owner_view.get(resident->island_entity);
        return island_owner.client_entity == client_entity;
    } else if (auto *resident = registry.try_get<multi_island_resident>(entity)) {
        for (auto island_entity : resident->island_entities) {
            auto [island_owner] = owner_view.get(island_entity);

            if (island_owner.client_entity != client_entity) {
                return false;
            }
        }

        return true;
    }

    return true;
}

}
