#include "edyn/util/island_util.hpp"
#include "edyn/comp.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/dynamics/world.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/comp/island.hpp"
#include <entt/entt.hpp>

namespace edyn {

void wakeup(entt::entity entity, entt::registry &registry) {
    auto &container = registry.get<island_container>(entity);
    for (auto island_entity : container.entities) {
        wakeup_island(island_entity, registry);
    }
}

void wakeup_island(entt::entity island_ent, entt::registry &registry) {
    // Remove the `sleeping_tag` from all the entities associated with the
    // given island.
    registry.remove<sleeping_tag>(island_ent);

    /* auto &isle = registry.get<island>(island_ent);
    isle.sleep_step = UINT64_MAX;

    for (auto e : isle.entities) {
        registry.remove<sleeping_tag>(e);
    } */
}

void put_islands_to_sleep(entt::registry &registry, uint64_t step, scalar dt) {
    /* auto island_view = registry.view<island>(exclude_global);

    island_view.each([&] (auto ent, auto &isle) {
        // Check if there are any entities in this island moving faster than
        // the sleep threshold.
        bool sleep = true;
        for (auto e : isle.entities) {
            // Check if this island has any entities with a `sleeping_disabled_tag`.
            if (registry.has<sleeping_disabled_tag>(e)) {
                sleep = false;
                break;
            }

            const auto *v = registry.try_get<linvel>(e);
            const auto *w = registry.try_get<angvel>(e);

            if ((v && length_sqr(*v) > island_linear_sleep_threshold * island_linear_sleep_threshold) || 
                (w && length_sqr(*w) > island_angular_sleep_threshold * island_angular_sleep_threshold)) {
                sleep = false;
                break;
            }
        }

        if (!sleep) {
            return;
        }

        // Put to sleep if the velocities of all entities in this island have
        // been under the threshold for some time.
        if (isle.sleep_step == UINT64_MAX) {
            isle.sleep_step = step;
        } else if ((step - isle.sleep_step) * dt > island_time_to_sleep) {
            registry.emplace<sleeping_tag>(ent);

            // Assign `sleeping_tag` to all entities in this island.
            for (auto e : isle.entities) {
                if (auto *v = registry.try_get<linvel>(e)) {
                    *v = vector3_zero;
                }

                if (auto *w = registry.try_get<angvel>(e)) {
                    *w = vector3_zero;
                }

                registry.emplace<sleeping_tag>(e);
            }
        }
    }); */
}

entity_set get_island_node_children(const entt::registry &registry, entt::entity node_entity) {
    EDYN_ASSERT(registry.has<island_node_parent>(node_entity));

    entity_set children;
    std::vector<entt::entity> parents_to_visit;
    parents_to_visit.push_back(node_entity);

    while (!parents_to_visit.empty()) {
        auto parent_entity = parents_to_visit.back();
        parents_to_visit.pop_back();

        auto &parent = registry.get<island_node_parent>(parent_entity);
        for (auto child : parent.children) {
            children.insert(child);

            if (registry.has<island_node_parent>(child)) {
                parents_to_visit.push_back(child);
            }
        }
    }

    return children;
}

}