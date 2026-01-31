#include "edyn/networking/util/process_extrapolation_result.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include "edyn/networking/util/import_contact_manifolds.hpp"
#include "edyn/networking/sys/accumulate_discontinuities.hpp"
#include "edyn/networking/sys/assign_previous_transforms.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/util/island_util.hpp"

namespace edyn {

entt::sparse_set get_entities_from_extrapolation_result(const extrapolation_result &result) {
    auto entities = entt::sparse_set{};

    for (auto *op : result.ops.operations) {
        if (!entities.contains(op->entity)) {
            entities.push(op->entity);
        }
    }

    return entities;
}

void post_extrapolation_update(entt::registry &registry, const std::vector<entt::entity> &entities) {
    update_origins(registry, entities);
    update_rotated_meshes(registry, entities);
    update_aabbs(registry, entities);
    auto island_entities = collect_islands_from_residents(registry, entities);
    update_island_aabbs(registry, island_entities);
    update_inertias(registry, entities);
}

void process_extrapolation_result(entt::registry &registry, entity_map &emap,
                                  const extrapolation_result &result) {
    EDYN_ASSERT(!result.ops.empty());

    // Assign current transforms to previous before importing pools into registry.
    assign_previous_transforms(registry);

    result.ops.execute(registry, emap);

    accumulate_discontinuities(registry);
    import_contact_manifolds(registry, emap, result.manifolds, result.contacts);

    // Wake up affected entities.
    auto remote_entities = get_entities_from_extrapolation_result(result);

    std::vector<entt::entity> local_entities;
    local_entities.reserve(remote_entities.size());

    for (auto remote_entity : remote_entities) {
        if (emap.contains(remote_entity)) {
            local_entities.push_back(emap.at(remote_entity));
        }
    }

    wake_up_island_residents(registry, local_entities);
    post_extrapolation_update(registry, local_entities);
}

void process_extrapolation_result(entt::registry &registry,
                                  const extrapolation_result &result) {
    EDYN_ASSERT(!result.ops.empty());

    // Assign current transforms to previous before importing pools into registry.
    assign_previous_transforms(registry);

    result.ops.execute(registry);

    accumulate_discontinuities(registry);
    import_contact_manifolds(registry, result.manifolds, result.contacts);

    // Wake up affected entities.
    auto entities = get_entities_from_extrapolation_result(result);
    wake_up_island_residents(registry, entities);

    std::vector<entt::entity> local_entities;
    local_entities.insert(local_entities.end(), entities.begin(), entities.end());
    post_extrapolation_update(registry, local_entities);
}

}
