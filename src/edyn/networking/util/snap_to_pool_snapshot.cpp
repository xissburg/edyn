#include "edyn/networking/util/snap_to_pool_snapshot.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/networking/sys/assign_previous_transforms.hpp"
#include "edyn/networking/sys/accumulate_discontinuities.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/util/island_util.hpp"

namespace edyn {

void post_snap_update(entt::registry &registry, const std::vector<entt::entity> &entities) {
    update_origins(registry, entities);
    update_rotated_meshes(registry, entities);
    update_aabbs(registry, entities);
    auto island_entities = collect_islands_from_residents(registry, entities);
    update_island_aabbs(registry, island_entities);
    update_inertias(registry, entities);
}

void snap_to_pool_snapshot(entt::registry &registry, const entity_map &emap,
                           const std::vector<entt::entity> &entities,
                           const std::vector<pool_snapshot> &pools,
                           bool should_accumulate_discontinuities) {
    if (should_accumulate_discontinuities) {
        assign_previous_transforms(registry);
    }

    for (auto &pool : pools) {
        pool.ptr->replace_into_registry(registry, entities, emap);
    }

    if (should_accumulate_discontinuities) {
        accumulate_discontinuities(registry);
    }

    std::vector<entt::entity> local_entities;
    local_entities.reserve(entities.size());

    for (auto remote_entity : entities) {
        if (emap.contains(remote_entity)) {
            local_entities.push_back(emap.at(remote_entity));
        }
    }

    post_snap_update(registry, local_entities);
}

void snap_to_pool_snapshot(entt::registry &registry,
                           const std::vector<entt::entity> &entities,
                           const std::vector<pool_snapshot> &pools,
                           bool should_accumulate_discontinuities) {
    if (should_accumulate_discontinuities) {
        assign_previous_transforms(registry);
    }

    for (auto &pool : pools) {
        pool.ptr->replace_into_registry(registry, entities);
    }

    if (should_accumulate_discontinuities) {
        accumulate_discontinuities(registry);
    }

    post_snap_update(registry, entities);
}

}
