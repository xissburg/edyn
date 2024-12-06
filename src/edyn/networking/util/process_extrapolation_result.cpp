#include "edyn/networking/util/process_extrapolation_result.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include "edyn/networking/util/import_contact_manifolds.hpp"
#include "edyn/networking/sys/accumulate_discontinuities.hpp"
#include "edyn/networking/sys/assign_previous_transforms.hpp"
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

void process_extrapolation_result(entt::registry &registry, entity_map &emap,
                                  const extrapolation_result &result) {
    EDYN_ASSERT(!result.ops.empty());

    // Assign current transforms to previous before importing pools into registry.
    assign_previous_transforms(registry);

    result.ops.execute(registry, emap);

    accumulate_discontinuities(registry);
    import_contact_manifolds(registry, emap, result.manifolds);

    // Wake up affected entities.
    auto remote_entities = get_entities_from_extrapolation_result(result);
    wake_up_island_residents(registry, remote_entities, emap);
}

void process_extrapolation_result(entt::registry &registry,
                                  const extrapolation_result &result) {
    EDYN_ASSERT(!result.ops.empty());

    // Assign current transforms to previous before importing pools into registry.
    assign_previous_transforms(registry);

    result.ops.execute(registry);

    accumulate_discontinuities(registry);
    import_contact_manifolds(registry, result.manifolds);

    // Wake up affected entities.
    auto entities = get_entities_from_extrapolation_result(result);
    wake_up_island_residents(registry, entities);
}

}
