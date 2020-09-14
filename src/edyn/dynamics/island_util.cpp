#include "edyn/dynamics/island_util.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/dynamics/world.hpp"
#include "edyn/parallel/message.hpp"
#include <entt/entt.hpp>

namespace edyn {

void wakeup(entt::entity entity, entt::registry &registry) {
    auto &node = registry.get<island_node>(entity);
    wakeup_island(node.island_entity, registry);
}

void wakeup_island(entt::entity island_ent, entt::registry &registry) {
    // Remove the `sleeping_tag` from all the entities associated with the
    // given island.
    registry.reset<sleeping_tag>(island_ent);

    auto &isle = registry.get<island>(island_ent);
    isle.sleep_step = UINT64_MAX;

    for (auto e : isle.entities) {
        registry.reset<sleeping_tag>(e);

        if (auto rel_con = registry.try_get<relation_container>(e)) {
            for (auto rel_ent : rel_con->entities) {
                registry.reset<sleeping_tag>(rel_ent);

                // If this relation has an associated constraint, also wake up all
                // its rows.
                auto con = registry.try_get<constraint>(rel_ent);
                if (con) {
                    for (size_t i = 0; i < con->num_rows; ++i) {
                        registry.reset<sleeping_tag>(con->row[i]);
                    }
                }
            }
        }
    }
}

void put_islands_to_sleep(entt::registry &registry, uint64_t step, scalar dt) {
    auto vel_view = registry.view<dynamic_tag, linvel, angvel>(exclude_global);
    auto island_view = registry.view<island>(exclude_global);

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

            const auto &v = vel_view.get<linvel>(e);
            const auto &w = vel_view.get<angvel>(e);

            if (length_sqr(v) > island_linear_sleep_threshold * island_linear_sleep_threshold || 
                length_sqr(w) > island_angular_sleep_threshold * island_angular_sleep_threshold) {
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
            registry.assign<sleeping_tag>(ent);

            // Assign `sleeping_tag` to all entities in this island and also
            // to all relations associated with them.
            for (auto e : isle.entities) {
                auto [v, w] = vel_view.get<linvel, angvel>(e);
                v = vector3_zero;
                w = vector3_zero;

                registry.assign<sleeping_tag>(e);

                auto rel_con = registry.get<relation_container>(e);
                for (auto rel_ent : rel_con.entities) {
                    registry.assign_or_replace<sleeping_tag>(rel_ent);

                    // If this relation has an associated constraint, assing a
                    // `sleeping_tag` to all its rows.
                    auto con = registry.try_get<constraint>(rel_ent);
                    if (con) {
                        for (size_t i = 0; i < con->num_rows; ++i) {
                            registry.assign_or_replace<sleeping_tag>(con->row[i]);
                        }
                    }
                }
            }
        }
    });
}

void island_on_construct_relation(entt::entity entity, entt::registry &registry, relation &rel) {
    EDYN_ASSERT(rel.entity[0] != entt::null);
    EDYN_ASSERT(rel.entity[1] != entt::null);

    // Allow the related entities to refer to their relations.
    registry.get_or_assign<relation_container>(rel.entity[0]).entities.push_back(entity);
    registry.get_or_assign<relation_container>(rel.entity[1]).entities.push_back(entity);

    // Find all islands involved in this new relation.
    std::vector<entt::entity> island_ents;

    for (auto ent : rel.entity) {
        if (ent == entt::null) { continue; }

        auto node = registry.try_get<island_node>(ent);
        if (node && 
            std::find(island_ents.begin(), island_ents.end(), 
                      node->island_entity) == island_ents.end()) {
            island_ents.push_back(node->island_entity);
        }
    }

    // All entities are in the same island. Nothing needs to be done.
    if (island_ents.size() < 2) {
        if (!island_ents.empty()) {
            wakeup_island(island_ents[0], registry);
        }
        return;
    }

    // Merge all into one island.
    auto new_island_ent = registry.create();
    auto &new_isle = registry.assign<island>(new_island_ent);

    for (auto isle_ent : island_ents) {
        auto isle = registry.get<island>(isle_ent);
        new_isle.entities.insert(new_isle.entities.end(), isle.entities.begin(), isle.entities.end());
    }

    // Destroy smaller islands. Triggers `on_destroy<island>` which finishes
    // the island worker.
    registry.destroy(island_ents.begin(), island_ents.end());

    // Send snapshot containing all entities in the new island to its associated
    // `island_worker`, which is created at the moment the `island` component is
    // assigned to `new_island_ent`.
    auto buffer = memory_output_archive::buffer_type();
    auto output = memory_output_archive(buffer);
    auto writer = registry_snapshot_writer(registry, all_components{});
    writer.updated<island>(new_island_ent);
    for (auto entity : new_isle.entities) {
        writer.updated(entity, all_components{});
    }
    writer.serialize(output);

    auto &wrld = registry.ctx<world>();
    wrld.m_island_info_map.at(new_island_ent).m_message_queue.send<msg::registry_snapshot>(buffer);
}

void island_on_destroy_relation(entt::entity entity, entt::registry &registry) {
    // Perform graph-walks using the entities in the destroyed relation as the
    // starting point (ignoring this destroyed relation, of course). Store the 
    // entities visited in each case in a set. If all sets are equal, it means
    // the island has not been broken and nothing needs to be done. If the sets
    // are different, destroy this island and create news islands for each set.
    // Update `island_node`s to point the new islands.

    auto &rel = registry.get<relation>(entity);

    // Remove the destroyed relation from the `relation_container` of the
    // related entities.
    for (auto ent : rel.entity) {
        if (ent == entt::null) { continue; }
        auto &container = registry.get<relation_container>(ent);
        auto it = std::find(container.entities.begin(), container.entities.end(), entity);
        std::swap(*it, *(container.entities.end() - 1));
        container.entities.pop_back();
    }

    // Store all entities found while walking the graph from each starting
    // point.
    std::array<std::vector<entt::entity>, max_relations> connected_entities;

    // Walk the graph starting from each entity in the destroyed relation.
    for (size_t i = 0; i < max_relations; ++i) {
        // Only dynamic entities matter since constraints do not affect static
        // and kinematic entities.
        auto rel_ent = rel.entity[i];
        if (rel_ent == entt::null || 
            !registry.has<dynamic_tag>(rel_ent)) {
            continue;
        }

        std::vector<entt::entity> visit_me;
        visit_me.push_back(rel_ent);

        while (!visit_me.empty()) {
            auto visit_ent = visit_me.back();
            visit_me.pop_back();
            auto found_it = std::find(connected_entities[i].begin(), 
                                      connected_entities[i].end(), visit_ent);

            if (found_it == connected_entities[i].end()) {
                connected_entities[i].push_back(visit_ent);
            } else {
                continue; // Already visited.
            }

            // Grab neighboring entities to visit.
            auto &container = registry.get<relation_container>(visit_ent);

            for (auto cont_ent : container.entities) {
                auto &cont_rel = registry.get<relation>(cont_ent);
                
                for (auto ent : cont_rel.entity) {
                    if (ent != entt::null && ent != visit_ent && registry.has<dynamic_tag>(ent)) {
                        visit_me.push_back(ent);
                    }
                }
            }
        }
    }

    // If the sets are different, split island.
    for (size_t i = 0; i < max_relations; ++i) {
        std::sort(connected_entities[i].begin(), connected_entities[i].end());
    }

    auto different = false;
    
    for (size_t i = 0; i < max_relations; ++i) {
        for (size_t j = i; j < max_relations; ++j) {
            if (connected_entities[i] != connected_entities[j]) {
                different = true;
                break;
            }
        }
    }

    if (!different) {
        return;
    }

    // Split into two islands.
    for (size_t i = 0; i < max_relations; ++i) {
        auto new_island_ent = registry.create();
        auto &new_isle = registry.assign<island>(new_island_ent);

        for (auto isle_ent : connected_entities[i]) {
            auto isle = registry.get<island>(isle_ent);
            new_isle.entities.insert(new_isle.entities.end(), isle.entities.begin(), isle.entities.end());
        }

        // Update all nodes.
        for (auto ent : connected_entities[i]) {
            auto &node = registry.get<island_node>(ent);
            node.island_entity = new_island_ent;
        }

        // Wake up all entities in the new island.
        wakeup_island(new_island_ent, registry);

        // Send snapshot containing all entities in the new island to its associated
        // `island_worker`, which is created at the moment the `island` component is
        // assigned to `new_island_ent`.
        auto buffer = memory_output_archive::buffer_type();
        auto output = memory_output_archive(buffer);
        auto writer = registry_snapshot_writer(registry, all_components{});
        writer.updated<island>(new_island_ent);
        for (auto entity : new_isle.entities) {
            writer.updated(entity, all_components{});
        }
        writer.serialize(output);

        auto &wrld = registry.ctx<world>();
        wrld.m_island_info_map.at(new_island_ent).m_message_queue.send<msg::registry_snapshot>(buffer);
    }

    // Destroy original island.
    auto &node = registry.get<island_node>(entity);
    registry.destroy(node.island_entity);
}

}