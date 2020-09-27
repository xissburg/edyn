#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/s11n.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/parallel/registry_snapshot.hpp"

namespace edyn {

void on_construct_or_replace_mass(entt::entity entity, entt::registry &registry, mass &m) {
    EDYN_ASSERT(m > 0);
    registry.assign_or_replace<mass_inv>(entity, m < EDYN_SCALAR_MAX ? 1 / m : 0);
}

void on_destroy_mass(entt::entity entity, entt::registry &registry) {
    registry.reset<mass_inv>(entity);
}

void on_construct_or_replace_inertia(entt::entity entity, entt::registry &registry, inertia &i) {
    EDYN_ASSERT(i > vector3_zero);
    auto &invI = registry.assign_or_replace<inertia_inv>(entity, 
                                                         i.x < EDYN_SCALAR_MAX ? 1 / i.x : 0, 
                                                         i.y < EDYN_SCALAR_MAX ? 1 / i.y : 0, 
                                                         i.z < EDYN_SCALAR_MAX ? 1 / i.z : 0);
    registry.assign_or_replace<inertia_world_inv>(entity, diagonal(invI));
}

void on_destroy_inertia(entt::entity entity, entt::registry &registry) {
    registry.reset<inertia_inv>(entity);
    registry.reset<inertia_world_inv>(entity);
}

void on_construct_shape(entt::entity entity, entt::registry &registry, shape &) {
    registry.assign<AABB>(entity);
    registry.assign<collision_filter>(entity);
}

void on_destroy_shape(entt::entity entity, entt::registry &registry) {
    registry.reset<AABB>(entity);
    registry.reset<collision_filter>(entity);
}

template<typename Snapshot>
void on_registry_snapshot(entt::registry &registry, const Snapshot &snapshot) {
    // Load snapshot from island. It is already mapped into the main
    // registry's domain.
    snapshot.load(registry);
}

void world::on_construct_rigidbody(entt::entity entity) {
    EDYN_ASSERT((m_m_registry->has<position, orientation, linvel, angvel>(entity)));
    m_m_registry->assign<relation_container>(entity);
    m_m_registry->assign<island_node>(entity);
}

void world::on_construct_dynamic_tag(entt::entity entity, entt::registry &registry, dynamic_tag) {
    on_construct_rigidbody(entity);
    m_constructed_dynamic_entities.push_back(entity);
}

void world::on_destroy_dynamic_tag(entt::entity entity, entt::registry &registry) {
    // Remove from island.
    auto &node = registry.get<island_node>(entity);
    auto island_entity = node.island_entities.front();
    auto &isle = registry.get<island>(island_entity);
    auto it = std::find(isle.entities.begin(), isle.entities.end(), entity);
    std::swap(*it, *(isle.entities.end() - 1));
    isle.entities.pop_back();

    // Destroy island if empty.
    if (isle.entities.empty()) {
        registry.destroy(island_entity);
    }
}

void world::on_construct_static_tag(entt::entity entity, entt::registry &registry, static_tag) {
    on_construct_rigidbody(entity);
}

void world::on_destroy_static_tag(entt::entity entity, entt::registry &registry) {

}

void world::on_construct_kinematic_tag(entt::entity entity, entt::registry &registry, kinematic_tag) {
    on_construct_rigidbody(entity);
}

void world::on_destroy_kinematic_tag(entt::entity entity, entt::registry &registry) {

}

void on_construct_island(entt::entity entity, entt::registry &registry, island &isle) {
    isle.timestamp = (double)performance_counter() / (double)performance_frequency();

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();

    auto &wrld = registry.ctx<world>();

    // The `island_worker_context` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto *worker = new island_worker_context(entity, wrld.fixed_dt, message_queue_in_out(main_queue_input, isle_queue_output),
                                             all_components{});

    auto info = island_info(worker, message_queue_in_out(isle_queue_input, main_queue_output));
    wrld.m_island_info_map.insert(std::make_pair(entity, info));

    // Send over a snapshot containing this island entity to the island worker
    // before it even starts.
    using registry_snapshot_type = decltype(registry_snapshot(all_components{}));
    auto snapshot = registry_snapshot(all_components{});
    snapshot.updated(entity, isle);

    info.m_message_queue.send<registry_snapshot_type>(snapshot);

    info.m_message_queue.sink<registry_snapshot_type>()
        .connect<&on_registry_snapshot<registry_snapshot_type>>(registry);
    //info.m_message_queue.sink<msg::entity_created>().connect<&world::on_entity_created>(*this);

    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(worker);
    archive(ctx_intptr);
    job_dispatcher::global().async(j);
}

void on_destroy_island(entt::entity entity, entt::registry &registry) {
    
}

world::world(entt::registry &reg) 
    : m_registry(&reg)
    , sol(reg)
    , bphase(reg)
{
    connections.push_back(reg.on_construct<mass>().connect<&on_construct_or_replace_mass>());
    connections.push_back(reg.on_replace<mass>().connect<&on_construct_or_replace_mass>());
    connections.push_back(reg.on_destroy<mass>().connect<&on_destroy_mass>());

    connections.push_back(reg.on_construct<inertia>().connect<&on_construct_or_replace_inertia>());
    connections.push_back(reg.on_replace<inertia>().connect<&on_construct_or_replace_inertia>());
    connections.push_back(reg.on_destroy<inertia>().connect<&on_destroy_inertia>());

    connections.push_back(reg.on_construct<shape>().connect<&on_construct_shape>());
    connections.push_back(reg.on_destroy<shape>().connect<&on_destroy_shape>());

    connections.push_back(reg.on_construct<dynamic_tag>().connect<&world::on_construct_dynamic_tag>(*this));
    connections.push_back(reg.on_destroy<dynamic_tag>().connect<&world::on_destroy_dynamic_tag>(*this));

    connections.push_back(reg.on_construct<static_tag>().connect<&world::on_construct_static_tag>(*this));
    connections.push_back(reg.on_destroy<static_tag>().connect<&world::on_destroy_static_tag>(*this));

    connections.push_back(reg.on_construct<kinematic_tag>().connect<&world::on_construct_kinematic_tag>(*this));
    connections.push_back(reg.on_destroy<kinematic_tag>().connect<&world::on_destroy_kinematic_tag>(*this));

    connections.push_back(reg.on_construct<island>().connect<&on_construct_island>());
    connections.push_back(reg.on_destroy<island>().connect<&on_destroy_island>());

    connections.push_back(reg.on_construct<relation>().connect<&world::on_construct_relation>(*this));
    connections.push_back(reg.on_destroy<relation>().connect<&world::on_destroy_relation>(*this));

    // Associate a `contact_manifold` to every broadphase relation that's created.
    connections.push_back(bphase.intersect_sink().connect<&world::on_broadphase_intersect>(*this));

    job_dispatcher::global().assure_current_queue();
}

world::~world() {
    
}

void world::process_pending_events() {
    // Find connected components to build new islands for dynamic entities.
    while (!m_constructed_dynamic_entities.empty()) {
        entt::entity island_entity = entt::null;
        std::vector<entt::entity> connected;
        std::vector<entt::entity> relations;
        std::vector<entt::entity> to_visit;
        to_visit.push_back(m_constructed_dynamic_entities.back());

        while (!to_visit.empty()) {
            auto entity = to_visit.back();
            to_visit.pop_back();

            // Add to connected component.
            connected.push_back(entity);

            // Remove from main set.
            m_constructed_dynamic_entities.erase(
                std::remove_if(m_constructed_dynamic_entities.begin(), 
                               m_constructed_dynamic_entities.end(), entity),
                m_constructed_dynamic_entities.end());

            // Add related entities to be visited next.
            auto &rel = m_registry->get<relation_container>(entity);

            for (auto rel_entity : rel.entities) {
                relations.push_back(rel_entity);

                auto &rel = m_registry->get<relation>(rel_entity);

                for (auto related : rel.entity) {
                    if (related == entt::null) continue;

                    auto find_it = std::find(connected.begin(), 
                                             connected.end(), related);
                    auto already_visited = find_it != connected.end();
                    if (already_visited) continue;

                    // If the related entity is dynamic and is already in an island,
                    // it means this new dynamic entity must join the existing island.
                    auto &node = m_registry->get<island_node>(related);
                    if (m_registry->has<dynamic_tag>(related) && !node.island_entities.empty()) {
                        // Dynamic entity must be in only one island.
                        EDYN_ASSERT(node.island_entities.size() == 1);
                        // If another dynamic entity that is already in an island is
                        // found, it must be in the same island that was found earlier.
                        EDYN_ASSERT(island_entity == entt::null || island_entity == node.island_entities.front());
                        island_entity = node.island_entities.front();
                        // This entity must not be visited because it is already in an
                        // island and thus would be added again, plus it'd be wasteful
                        // to visit the entire island here.
                    } else {
                        to_visit.push_back(related);
                    }
                }
            }
        }

        if (island_entity == entt::null) {
            // Need to create a new island for the connected entities.
            island_entity = m_registry->create();
            m_registry->assign<island>(island_entity);
        }

        auto &isle = m_registry->get<island>(island_entity);

        // Insert connected entities into island.
        for (auto entity : connected) {
            isle.entities.push_back(entity);
            auto &node = m_registry->get<island_node>(entity);
            node.island_entities.push_back(island_entity);
        }

        // Send a snapshot to the island worker containing the new entities.
        auto snapshot = registry_snapshot(all_components{});
        snapshot.updated(island_entity, isle);
        for (auto entity : connected) {
            snapshot.maybe_updated(entity, *m_registry, all_components{});
        }
        for (auto entity : relations) {
            snapshot.maybe_updated(entity, *m_registry, all_components{});
        }
        send_message<registry_snapshot_type>(island_entity, snapshot);
    }

    // Process static/kinematic entities.
    
}

void world::refresh_all(entt::entity entity) {
    auto snapshot = registry_snapshot(all_components{});
    snapshot.maybe_updated(entity, *registry, all_components{});
    auto &node = m_registry->get<island_node>(entity);

    for (auto island_entity : node.island_entities) {
        auto &info = m_island_info_map.at(island_entity);
        info.m_message_queue.send<registry_snapshot_type>(snapshot);
    }
}

void world::on_broadphase_intersect(entt::entity e0, entt::entity e1) {
    merge_entities(e0, e1, entt::null);
}

void world::on_construct_relation(entt::entity entity, entt::registry &registry, relation &rel) {
    for (auto entity : rel.entity) {
        auto &container = registry.get<relation_container>(entity);
        container.entities.push_back(entity);
    }
    m_new_relations.push_back(entity);
}

void world::on_destroy_relation(entt::entity entity, entt::registry &registry) {
    auto snapshot = registry_snapshot(all_components{});
    snapshot.destroyed<relation>(entity);
    auto &node = registry.get<island_node>(entity);

    for (auto island_entity : node.island_entities) {
        auto &info = m_island_info_map.at(island_entity);
        info.m_message_queue.send<registry_snapshot_type>(snapshot);
    }
}

void world::merge_entities(entt::entity e0, entt::entity e1, entt::entity rel_entity) {
    // If both entities are static or kinematic nothing needs to be done.
    if ((m_registry->has<static_tag>(e0) || m_registry->has<kinematic_tag>(e0)) &&
        (m_registry->has<static_tag>(e1) || m_registry->has<kinematic_tag>(e1))) {
        return;
    }

    auto &node0 = m_registry->get<island_node>(e0);
    auto &node1 = m_registry->get<island_node>(e1);

    // Check if entities are already in the same island.
    // One of the entities has to be dynamic, which means it must be contained
    // in one and only one island at all times.
    if (m_registry->has<dynamic_tag>(e0)) {
        auto island_entity0 = node0.island_entities.front();
        for (auto island_entity1 : node1.island_entities) {
            if (island_entity0 == island_entity1) {
                // Entities are in the same island, nothing needs to be done.
                return;
            }
        }
    } else if (m_registry->has<dynamic_tag>(e1)) {
        auto island_entity1 = node1.island_entities.front();
        for (auto island_entity0 : node0.island_entities) {
            if (island_entity0 == island_entity1) {
                // Entities are in the same island, nothing needs to be done.
                return;
            }
        }
    }

    // When a dynamic and a static or kinematic entity intersect, the static/
    // kinematic entity is added to the same island where the dynamic entity
    // resides.
    if (m_registry->has<dynamic_tag>(e0) && 
        (m_registry->has<static_tag>(e1) || m_registry->has<kinematic_tag>(e1))) {
        auto island_entity = node0.island_entities.front();
        node1.island_entities.push_back(island_entity);
        auto &isle = m_registry->get<island>(island_entity);
        isle.entities.push_back(e1);

        if (rel_entity != entt::null) {
            // Create association between island and relation.
            auto &rel_node = m_registry->get<island_node>(rel_entity);
            rel_node.island_entities.push_back(island_entity);
            isle.entities.push_back(rel_entity);
        }

        // Include static/kinematic entity into the island worker.
        auto snapshot = registry_snapshot(all_components{});
        snapshot.updated(island_entity, isle);
        snapshot.maybe_updated(e1, *registry, all_components{});
        
        if (rel_entity != entt::null) {
            snapshot.maybe_updated(rel_entity, *registry, all_components{});
        }
        
        auto &info0 = m_island_info_map.at(island_entity);
        info0.m_message_queue.send<registry_snapshot_type>(snapshot);

        return;
    }

    // Do it the other way around also.
    if (m_registry->has<dynamic_tag>(e1) && 
        (m_registry->has<static_tag>(e0) || m_registry->has<kinematic_tag>(e0))) {
        auto island_entity = node1.island_entities.front();
        node1.island_entities.push_back(island_entity);
        auto &isle = m_registry->get<island>(island_entity);
        isle.entities.push_back(e0);

        if (rel_entity != entt::null) {
            // Create association between island and relation.
            auto &rel_node = m_registry->get<island_node>(rel_entity);
            rel_node.island_entities.push_back(island_entity);
            isle.entities.push_back(rel_entity);
        }

        // Include static/kinematic entity into the island worker.
        auto snapshot = registry_snapshot(all_components{});
        snapshot.updated(island_entity, isle);
        snapshot.maybe_updated(e0, *registry, all_components{});

        if (rel_entity != entt::null) {
            snapshot.maybe_updated(rel_entity, *registry, all_components{});
        }

        auto &info1 = m_island_info_map.at(island_entity);
        info1.m_message_queue.send<registry_snapshot_type>(snapshot);

        return;
    }

    // For dynamic entities, if they are in different islands, the islands have
    // to be merged into one.
    if (!m_registry->has<dynamic_tag>(e0) || !m_registry->has<dynamic_tag>(e1)) {
        return;
    }

    auto island_entity0 = node0.island_entities.front();
    auto island_entity1 = node1.island_entities.front();

    // Merge islands.
    auto &island0 = m_registry->get<island>(island_entity0);
    auto &island1 = m_registry->get<island>(island_entity1);

    for (auto ent : island1.entities) {
        island0.entities.push_back(ent);
        auto &node = m_registry->get<island_node>(ent);
        node.island_entities.clear();
        node.island_entities.push_back(island_entity0);
    }

    // Send snapshot containing moved entities to the first island.
    auto snapshot = registry_snapshot(all_components{});
    snapshot.updated(island_entity0, island0);

    for (auto ent : island1.entities) {
        snapshot.maybe_updated(ent, *registry, all_components{});
    }

    auto &info0 = m_island_info_map.at(island_entity0);
    info0.m_message_queue.send<registry_snapshot_type>(snapshot);

    // Destroy empty island.
    auto &info1 = m_island_info_map.at(island_entity1);
    info1.m_worker->finish();
    m_island_info_map.erase(island_entity1);
    m_registry->destroy(island_entity1);
}

void world::update(scalar dt) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    for (auto &pair : m_island_info_map) {
        pair.second.m_message_queue.update();
    }

    if (m_paused) {
        snap_presentation(*registry);
    } else {
        auto time = (double)performance_counter() / (double)performance_frequency();
        update_presentation(*registry, time);
    }

    bphase.update();

    update_signal.publish(dt);
}

void world::run() {
    running = true;

    const auto freq = performance_frequency();
    const auto timescale = 1.0 / freq;
    const auto t0 = performance_counter();
    auto ti = t0;

    // Use an Integral Controller to calculate the right amount of delay to
    // keep `dt` as close as possible to `fixed_dt`.
    const scalar I = 0.5;
    scalar delay_dt = 0;

    while (running) {
        const auto t = performance_counter();
        const auto dt = (t - ti) * timescale;
        update(dt);
        ti = t;
        local_time_ = t * timescale - residual_dt;

        auto err_dt = fixed_dt - dt;
        delay_dt += err_dt * I;

        delay(delay_dt * 1000);
    }
}

void world::quit() {
    running = false;
}

void world::set_paused(bool paused) {
    m_paused = paused;
    
    for (auto &pair : m_island_info_map) {
        pair.second.m_message_queue.send<msg::set_paused>(msg::set_paused{paused});
    }
}

void world::step() {
    for (auto &pair : m_island_info_map) {
        pair.second.m_message_queue.send<msg::step_simulation>();
    }
}

}