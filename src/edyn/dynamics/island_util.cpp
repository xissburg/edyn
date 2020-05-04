#include "edyn/dynamics/island_util.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/orientation.hpp"
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

            auto &v = vel_view.get<linvel>(e);
            vector3 w = vel_view.get<angvel>(e);

            auto s = registry.try_get<spin>(e);
            if (s) {
                auto &orn = registry.get<orientation>(e);
                w += rotate(orn, vector3_x) * *s;
            }

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

                auto s = registry.try_get<spin>(e);
                if (s) {
                    s = 0;
                }

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

    if (rel.entity[2] != entt::null) {
        registry.get_or_assign<relation_container>(rel.entity[2]).entities.push_back(entity);
    }

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
    size_t biggest_idx = 0;
    size_t biggest_size = 0;

    for (size_t i = 0; i < island_ents.size(); ++i) {
        auto &isle = registry.get<island>(island_ents[i]);
        if (isle.entities.size() > biggest_size) {
            biggest_size = isle.entities.size();
            biggest_idx = i;
        }
    }

    auto biggest_ent = island_ents[biggest_idx];
    auto &biggest_isle = registry.get<island>(biggest_ent);

    for (size_t i = 0; i < island_ents.size(); ++i) {
        if (i != biggest_idx) {
            auto other_ent = island_ents[i];
            auto &other_isle = registry.get<island>(other_ent);

            for (auto ent : other_isle.entities) {
                biggest_isle.entities.push_back(ent);

                auto &node = registry.get<island_node>(ent);
                node.island_entity = biggest_ent;
            }
        }
    }

    // Destroy other islands after to ensure the `biggest_isle` reference won't
    // be invalidated during the previous loop.
    for (size_t i = 0; i < island_ents.size(); ++i) {
        if (i != biggest_idx) {
            registry.destroy(island_ents[i]);
        }
    }

    wakeup_island(biggest_ent, registry);
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

    size_t biggest_idx = 0;
    size_t biggest_size = 0;

    for (size_t i = 0; i < max_relations; ++i) {
        if (connected_entities[i].size() > biggest_size) {
            biggest_size = connected_entities[i].size();
            biggest_idx = i;
        }
    }

    auto &node = registry.get<island_node>(rel.entity[biggest_idx]);
    auto biggest_ent = node.island_entity;

    for (size_t i = 0; i < max_relations; ++i) {
        if (i == biggest_idx) { continue; }
        if (rel.entity[i] == entt::null) { continue; }
        if (connected_entities[biggest_idx] == connected_entities[i]) { continue; }

        // Remove entities from the biggest island and move them to a new
        // island.
        auto &isle = registry.get<island>(biggest_ent);
        // TODO: minor optimization: swap with last then pop.
        auto erase_it = std::remove_if(isle.entities.begin(), isle.entities.end(), [&] (auto &ent) {
            return std::find(connected_entities[i].begin(), 
                             connected_entities[i].end(), ent) 
                != connected_entities[i].end();
        });
        isle.entities.erase(erase_it, isle.entities.end());

        // Create new island.
        auto [other_island_ent, other_isle] = registry.create<island>();    
        other_isle.entities = std::move(connected_entities[i]);

        // Update all nodes in the other set.
        for (auto ent : other_isle.entities) {
            auto &node = registry.get<island_node>(ent);
            node.island_entity = other_island_ent;
        }

        // Wake up all entities in the new island.
        wakeup_island(other_island_ent, registry);
    }

    // Wake up the biggest island either way since something has changed in it.
    wakeup_island(biggest_ent, registry);
}

}