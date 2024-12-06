#include "edyn/simulation/simulation_worker.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include "edyn/networking/sys/accumulate_discontinuities.hpp"
#include "edyn/networking/util/process_extrapolation_result.hpp"
#include "edyn/networking/util/snap_to_pool_snapshot.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/rigidbody.hpp"
#include "edyn/util/vector_util.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include <entt/core/type_info.hpp>
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <algorithm>
#include <atomic>
#include <mutex>

namespace edyn {

simulation_worker::simulation_worker(const settings &settings,
                                     const registry_operation_context &reg_op_ctx,
                                     const material_mix_table &material_table)
    : m_raycast_service(m_registry)
    , m_island_manager(m_registry)
    , m_poly_initializer(m_registry)
    , m_solver(m_registry)
    , m_op_builder((*reg_op_ctx.make_reg_op_builder)(m_registry))
    , m_op_observer((*reg_op_ctx.make_reg_op_observer)(*m_op_builder))
    , m_importing(false)
    , m_message_queue(message_dispatcher::global().make_queue<
        msg::set_paused,
        msg::set_settings,
        msg::set_registry_operation_context,
        msg::step_simulation,
        msg::set_com,
        msg::set_material_table,
        msg::update_entities,
        msg::apply_network_pools,
        msg::wake_up_residents,
        msg::change_rigidbody_kind,
        msg::raycast_request,
        msg::query_aabb_request,
        msg::query_aabb_of_interest_request,
        extrapolation_result>("worker"))
{
    m_registry.ctx().emplace<contact_manifold_map>(m_registry);
    m_registry.ctx().emplace<broadphase>(m_registry);
    m_registry.ctx().emplace<narrowphase>(m_registry);
    m_registry.ctx().emplace<entity_graph>();
    m_registry.ctx().emplace<edyn::settings>(settings);
    m_registry.ctx().emplace<registry_operation_context>(reg_op_ctx);
    m_registry.ctx().emplace<material_mix_table>(material_table);
}

simulation_worker::~simulation_worker() {
    stop();

    // The destructor of `polyhedron_shape_initializer` touches `m_registry` when
    // destroying `rotated_mesh_list` elements it creates for compound shapes
    // containing polyhedrons. It's called after `stop()` which finishes the
    // simulation thread. Thus there won't be race conditions in that destructor.
}

void simulation_worker::init() {
    m_connections.push_back(m_registry.on_construct<graph_node>().connect<&simulation_worker::on_construct_shared_entity>(*this));
    m_connections.push_back(m_registry.on_construct<graph_edge>().connect<&simulation_worker::on_construct_shared_entity>(*this));
    m_connections.push_back(m_registry.on_construct<island_tag>().connect<&simulation_worker::on_construct_shared_entity>(*this));

    m_connections.push_back(m_registry.on_destroy<graph_node>().connect<&simulation_worker::on_destroy_shared_entity>(*this));
    m_connections.push_back(m_registry.on_destroy<graph_edge>().connect<&simulation_worker::on_destroy_shared_entity>(*this));
    m_connections.push_back(m_registry.on_destroy<island_tag>().connect<&simulation_worker::on_destroy_shared_entity>(*this));

    m_message_queue.sink<msg::update_entities>().connect<&simulation_worker::on_update_entities>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&simulation_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&simulation_worker::on_step_simulation>(*this);
    m_message_queue.sink<msg::set_com>().connect<&simulation_worker::on_set_com>(*this);
    m_message_queue.sink<msg::set_settings>().connect<&simulation_worker::on_set_settings>(*this);
    m_message_queue.sink<msg::set_registry_operation_context>().connect<&simulation_worker::on_set_reg_op_ctx>(*this);
    m_message_queue.sink<msg::set_material_table>().connect<&simulation_worker::on_set_material_table>(*this);
    m_message_queue.sink<msg::raycast_request>().connect<&simulation_worker::on_raycast_request>(*this);
    m_message_queue.sink<msg::query_aabb_request>().connect<&simulation_worker::on_query_aabb_request>(*this);
    m_message_queue.sink<msg::query_aabb_of_interest_request>().connect<&simulation_worker::on_query_aabb_of_interest_request>(*this);
    m_message_queue.sink<msg::apply_network_pools>().connect<&simulation_worker::on_apply_network_pools>(*this);
    m_message_queue.sink<msg::wake_up_residents>().connect<&simulation_worker::on_wake_up_residents>(*this);
    m_message_queue.sink<msg::change_rigidbody_kind>().connect<&simulation_worker::on_change_rigidbody_kind>(*this);

    auto &settings = m_registry.ctx().get<edyn::settings>();

    // If this is a networked client, expect extrapolation results.
    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        m_message_queue.sink<extrapolation_result>().connect<&simulation_worker::on_extrapolation_result>(*this);
    }

    if (settings.init_callback) {
        (*settings.init_callback)(m_registry);
    }

    m_last_time = m_current_time;
    m_sim_time = m_current_time;
    m_island_manager.set_last_time(m_last_time);
}

void simulation_worker::deinit() {
    auto &settings = m_registry.ctx().get<edyn::settings>();

    if (settings.deinit_callback) {
        (*settings.deinit_callback)(m_registry);
    }
}

void simulation_worker::on_construct_shared_entity(entt::registry &registry, entt::entity entity) {
    m_op_observer->observe(entity);
}

void simulation_worker::on_destroy_shared_entity(entt::registry &registry, entt::entity entity) {
    m_op_observer->unobserve(entity);

    if (m_entity_map.contains_local(entity)) {
        m_entity_map.erase_local(entity);
    }
}

void simulation_worker::on_update_entities(message<msg::update_entities> &msg) {
    auto &ops = msg.content.ops;
    auto &registry = m_registry;
    auto &emap = m_entity_map;

    const auto &settings = registry.ctx().get<edyn::settings>();
    const bool is_client = std::holds_alternative<client_network_settings>(settings.network_settings);

    auto &graph = registry.ctx().get<entity_graph>();
    auto procedural_view = registry.view<procedural_tag>();

    // Import components from main registry.
    m_importing = true;
    m_op_observer->set_active(false);
    ops.execute(registry, emap, [&](operation_base *op){
        auto op_type = op->operation_type();
        auto remote_entity = op->entity;

        // Add all new entity mappings to current op builder which will be sent
        // over to the main thread so it can create corresponding mappings between
        // its new entities and the entities that were just created here in this
        // import.
        if (op_type == registry_operation_type::create) {
            auto local_entity = emap.at(remote_entity);
            m_op_builder->add_entity_mapping(local_entity, remote_entity);
        }

        // Insert nodes in the graph for rigid bodies and external entities, and
        // edges for constraints, because `graph_node` and `graph_edge` are not
        // shared components.
        if (op_type == registry_operation_type::emplace &&
            op->payload_type_any_of<rigidbody_tag, external_tag>())
        {
            auto local_entity = emap.at(remote_entity);
            auto procedural = procedural_view.contains(local_entity);
            auto node_index = graph.insert_node(local_entity, !procedural);
            registry.emplace<graph_node>(local_entity, node_index);

            if (!procedural) {
                // `multi_island_resident` is not a shared component thus add it
                // manually here.
                registry.emplace<multi_island_resident>(local_entity);
            }
        }

        if (op_type == registry_operation_type::emplace &&
           (op->payload_type_any_of(constraints_tuple) || op->payload_type_any_of<null_constraint>()))
        {
            auto local_entity = emap.at(remote_entity);

            // There could be multiple constraints (of different types) assigned to
            // the same entity, which means it could already have an edge.
            if (!registry.any_of<graph_edge>(local_entity)) {
                create_graph_edge_for_constraints(registry, local_entity, graph, constraints_tuple);
                create_graph_edge_for_constraint<null_constraint>(registry, local_entity, graph);
            }
        }

        // When orientation is set manually, a few dependent components must be
        // updated, e.g. AABB, cached origin, inertia_world_inv, rotated meshes...
        if (op_type == registry_operation_type::replace &&
            op->payload_type_any_of<orientation>())
        {
            auto local_entity = emap.at(remote_entity);

            if (auto *origin = registry.try_get<edyn::origin>(local_entity)) {
                auto &com = registry.get<center_of_mass>(local_entity);
                auto &pos = registry.get<position>(local_entity);
                auto &orn = static_cast<operation_replace<orientation> *>(op)->component;
                *origin = to_world_space(-com, pos, orn);
            }

            if (registry.any_of<AABB>(local_entity)) {
                update_aabb(registry, local_entity);
            }

            if (registry.any_of<dynamic_tag>(local_entity)) {
                update_inertia(registry, local_entity);
            }

            if (registry.any_of<rotated_mesh_list>(local_entity)) {
                update_rotated_mesh(registry, local_entity);
            }
        }

        // When position is set manually, the AABB and cached origin must be updated.
        if (op_type == registry_operation_type::replace &&
            op->payload_type_any_of<position>())
        {
            auto local_entity = emap.at(remote_entity);

            if (auto *origin = registry.try_get<edyn::origin>(local_entity)) {
                auto &com = registry.get<center_of_mass>(local_entity);
                auto &orn = registry.get<orientation>(local_entity);
                auto &pos = static_cast<operation_replace<position> *>(op)->component;
                *origin = to_world_space(-com, pos, orn);
            }

            if (registry.any_of<AABB>(local_entity)) {
                update_aabb(registry, local_entity);
            }
        }

        // Assign previous position and orientation components to dynamic entities
        // for client-side networking extrapolation discontinuity mitigation.
        if (is_client && op_type == registry_operation_type::emplace && op->payload_type_any_of<dynamic_tag>()) {
            auto local_entity = emap.at(remote_entity);
            registry.emplace<previous_position>(local_entity);
            registry.emplace<previous_orientation>(local_entity);
        }
    });

    m_importing = false;
    m_op_observer->set_active(true);

    // Wake up all islands involved.
    wake_up_affected_islands(msg.content.ops);
}

void simulation_worker::wake_up_affected_islands(const registry_operation &ops) {
    // Collect islands of all entities which had a component
    // emplaced/replaced/removed by the registry operations and wake them up.
    entt::sparse_set entities;

    for (auto *op : ops.operations) {
        if (op->operation_type() == registry_operation_type::replace && m_entity_map.contains(op->entity)) {
            auto remote_entity = op->entity;
            auto local_entity = m_entity_map.at(remote_entity);

            if (m_registry.valid(local_entity) && !entities.contains(local_entity)) {
                entities.push(local_entity);
            }
        }
    }

    if (!entities.empty()) {
        wake_up_island_residents(m_registry, entities);
    }
}

void simulation_worker::sync() {
    if (!m_op_builder->empty()) {
        auto &&ops = std::move(m_op_builder->finish());
        message_dispatcher::global().send<msg::step_update>(
            {"main"}, m_message_queue.identifier, std::move(ops), m_sim_time);
    }
}

void simulation_worker::start() {
    m_running.store(true, std::memory_order_release);

    auto &settings = m_registry.ctx().get<edyn::settings>();
    (*settings.start_thread_func)([](void *args) {
        std::invoke(&simulation_worker::run, reinterpret_cast<simulation_worker *>(args));
    }, this);
}

void simulation_worker::stop() {
    m_running.store(false, std::memory_order_release);

    std::unique_lock<std::mutex> lock(m_finish_mutex);
    m_finish_cv.wait(lock, [&]() {
        return m_finished.load(std::memory_order_relaxed);
    });
}

void simulation_worker::update() {
    // Must clear before reading messages to avoid accumulating over values that
    // have already been sent to main thread.
    clear_accumulated_discontinuities_quietly(m_registry);

    m_message_queue.update();
    m_raycast_service.update(true);
    consume_raycast_results();

    if (m_paused) {
        m_island_manager.update(m_last_time);
        sync();
        return;
    }

    const auto elapsed = m_current_time - m_last_time;
    m_accumulated_time += elapsed;

    auto &settings = m_registry.ctx().get<edyn::settings>();
    const auto fixed_dt = settings.fixed_dt;
    const auto num_steps = static_cast<int64_t>(std::floor(m_accumulated_time / fixed_dt));
    auto advance_dt = static_cast<double>(num_steps) * fixed_dt;
    m_accumulated_time -= advance_dt;

    auto effective_steps = num_steps;
    auto step_dt = fixed_dt;

    if (num_steps > settings.max_steps_per_update) {
        effective_steps = settings.max_steps_per_update;
        // Scale up the effective delta time of each step. Physics will be
        // updated using fixed dt always but the presentation step dt will be
        // greater thus slowing down the simulation.
        step_dt = advance_dt / effective_steps;
    }

    m_poly_initializer.init_new_shapes();

    auto &nphase = m_registry.ctx().get<narrowphase>();
    auto &bphase = m_registry.ctx().get<broadphase>();
    bphase.init_new_aabb_entities();

    for (unsigned i = 0; i < effective_steps; ++i) {
        if (settings.pre_step_callback) {
            (*settings.pre_step_callback)(m_registry);
        }

        bphase.update(true);
        m_island_manager.update(m_sim_time);
        nphase.update(true);
        m_solver.update(true);

        m_sim_time += step_dt;

        if (settings.clear_actions_func) {
            (*settings.clear_actions_func)(m_registry);
        }

        if (settings.post_step_callback) {
            (*settings.post_step_callback)(m_registry);
        }

        mark_transforms_replaced();
        sync();
    }

    m_last_time = m_current_time;
    m_sim_time = m_last_time - m_accumulated_time;
}

void simulation_worker::run() {
    // Use a PID to keep updates at a fixed and controlled rate.
    auto proportional_term = 0.18;
    auto integral_term = 0.06;
    auto i_term = 0.0;

    m_finished.store(false, std::memory_order_relaxed);
    m_current_time = (*m_registry.ctx().get<settings>().time_func)();
    init();

    while (m_running.load(std::memory_order_relaxed)) {
        auto t1 = (*m_registry.ctx().get<settings>().time_func)();
        auto dt = t1 - m_current_time;
        m_current_time = t1;
        update();
        sync();

        // Apply delay to maintain a fixed update rate.
        auto desired_dt = m_registry.ctx().get<settings>().fixed_dt;
        auto error = desired_dt - dt;
        i_term = std::max(-1.0, std::min(i_term + integral_term * error, 1.0));
        auto delay = std::max(0.0, proportional_term * error + i_term);
        edyn::delay(delay * 1000);
    }

    deinit();

    auto lock = std::lock_guard<std::mutex>(m_finish_mutex);
    m_finished.store(true, std::memory_order_relaxed);
    m_finish_cv.notify_one();
}

void simulation_worker::consume_raycast_results() {
    auto &dispatcher = message_dispatcher::global();
    m_raycast_service.consume_results([&](unsigned id, raycast_result &result) {
        dispatcher.send<msg::raycast_response>(
            {"main"}, m_message_queue.identifier, id, result);
    });
}

void simulation_worker::mark_transforms_replaced() {
    auto body_view = m_registry.view<position, orientation, linvel, angvel, dynamic_tag>(exclude_sleeping_disabled);
    m_op_builder->replace<position>(body_view.begin(), body_view.end());
    m_op_builder->replace<orientation>(body_view.begin(), body_view.end());
    m_op_builder->replace<linvel>(body_view.begin(), body_view.end());
    m_op_builder->replace<angvel>(body_view.begin(), body_view.end());
}

void simulation_worker::on_set_paused(message<msg::set_paused> &msg) {
    m_paused = msg.content.paused;
    m_registry.ctx().get<edyn::settings>().paused = m_paused;
    m_accumulated_time = 0;

    if (!m_paused) {
        m_last_time = m_current_time;
        m_sim_time = m_last_time;
    }
}

void simulation_worker::on_step_simulation(message<msg::step_simulation> &) {
    m_last_time = m_current_time;
    m_sim_time = m_last_time;

    auto &bphase = m_registry.ctx().get<broadphase>();
    auto &nphase = m_registry.ctx().get<narrowphase>();
    auto &settings = m_registry.ctx().get<edyn::settings>();

    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(m_registry);
    }

    m_poly_initializer.init_new_shapes();
    bphase.update(true);
    m_island_manager.update(m_last_time);
    nphase.update(true);
    m_solver.update(true);

    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(m_registry);
    }

    if (settings.post_step_callback) {
        (*settings.post_step_callback)(m_registry);
    }

    mark_transforms_replaced();
    sync();
}

void simulation_worker::on_set_settings(message<msg::set_settings> &msg) {
    const auto &settings = msg.content.settings;
    auto &current = m_registry.ctx().get<edyn::settings>();

    if (settings.init_callback && settings.init_callback != current.init_callback) {
        (*settings.init_callback)(m_registry);
    }

    if (settings.time_func != current.time_func) {
        m_current_time = (*settings.time_func)();
        m_last_time = m_current_time;
        m_sim_time = m_current_time;
        m_island_manager.set_last_time(m_last_time);
    }

    current = settings;

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        m_message_queue.sink<extrapolation_result>().connect<&simulation_worker::on_extrapolation_result>(*this);
    } else {
        m_message_queue.sink<extrapolation_result>().disconnect<&simulation_worker::on_extrapolation_result>(*this);
    }
}

void simulation_worker::on_set_reg_op_ctx(message<msg::set_registry_operation_context> &msg) {
    m_registry.ctx().get<registry_operation_context>() = msg.content.ctx;
    m_op_builder = (*msg.content.ctx.make_reg_op_builder)(m_registry);
    m_op_observer = (*msg.content.ctx.make_reg_op_observer)(*m_op_builder);
}

void simulation_worker::on_set_material_table(message<msg::set_material_table> &msg) {
    m_registry.ctx().get<material_mix_table>() = msg.content.table;
}

void simulation_worker::on_set_com(message<msg::set_com> &msg) {
    if (m_entity_map.contains(msg.content.entity)) {
        auto entity = m_entity_map.at(msg.content.entity);

        if (m_registry.valid(entity)) {
            internal::apply_center_of_mass(m_registry, entity, msg.content.com);
        }
    }
}

void simulation_worker::on_raycast_request(message<msg::raycast_request> &msg) {
    auto ignore_entities = std::vector<entt::entity>{};

    for (auto remote_entity : msg.content.ignore_entities) {
        if (m_entity_map.contains(remote_entity)) {
            auto local_entity = m_entity_map.at(remote_entity);
            ignore_entities.push_back(local_entity);
        }
    }
    m_raycast_service.add_ray(msg.content.p0, msg.content.p1, msg.content.id, ignore_entities);
}

void simulation_worker::on_query_aabb_request(message<msg::query_aabb_request> &msg) {
    auto &bphase = m_registry.ctx().get<broadphase>();
    auto &request = msg.content;
    auto response = msg::query_aabb_response{};
    response.id = msg.content.id;

    if (request.query_islands) {
        bphase.query_islands(request.aabb, [&response](entt::entity island_entity) {
            response.island_entities.push_back(island_entity);
        });
    }

    if (request.query_procedural) {
        bphase.query_procedural(request.aabb, [&response](entt::entity entity) {
            response.procedural_entities.push_back(entity);
        });
    }

    if (request.query_non_procedural) {
        bphase.query_non_procedural(request.aabb, [&response](entt::entity entity) {
            response.non_procedural_entities.push_back(entity);
        });
    }

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::query_aabb_response>(
            {"main"}, m_message_queue.identifier, std::move(response));
}

void simulation_worker::on_query_aabb_of_interest_request(message<msg::query_aabb_of_interest_request> &msg) {
    auto &bphase = m_registry.ctx().get<broadphase>();
    auto &request = msg.content;
    auto island_view = m_registry.view<island>();
    auto manifold_view = m_registry.view<contact_manifold>();
    auto procedural_view = m_registry.view<procedural_tag>();
    entt::sparse_set procedural_entities;
    entt::sparse_set np_entities;
    entt::sparse_set island_entities;

    // Collect entities of islands which intersect the AABB of interest.
    bphase.query_islands(request.aabb, [&](entt::entity island_entity) {
        auto [island] = island_view.get(island_entity);

        for (auto entity : island.nodes) {
            if (!procedural_entities.contains(entity) && procedural_view.contains(entity)) {
                procedural_entities.push(entity);
            }
        }

        for (auto entity : island.edges) {
            // Ignore contact manifolds.
            if (manifold_view.contains(entity)) {
                continue;
            }

            if (!procedural_entities.contains(entity)) {
                procedural_entities.push(entity);
            }
        }

        if (!island_entities.contains(island_entity)) {
            island_entities.push(island_entity);
        }
    });

    bphase.query_non_procedural(request.aabb, [&](entt::entity np_entity) {
        if (!np_entities.contains(np_entity)) {
            np_entities.push(np_entity);
        }
    });

    auto response = msg::query_aabb_response{};
    response.id = msg.content.id;
    response.island_entities.insert(response.island_entities.end(), island_entities.begin(), island_entities.end());
    response.procedural_entities.insert(response.procedural_entities.end(), procedural_entities.begin(), procedural_entities.end());
    response.non_procedural_entities.insert(response.non_procedural_entities.end(), np_entities.begin(), np_entities.end());

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::query_aabb_response>(
            {"main"}, m_message_queue.identifier, std::move(response));
}

void simulation_worker::on_extrapolation_result(message<extrapolation_result> &msg) {
    auto &result = msg.content;
    process_extrapolation_result(m_registry, m_entity_map, result);
}

void simulation_worker::on_apply_network_pools(message<msg::apply_network_pools> &msg) {
    EDYN_ASSERT(!msg.content.pools.empty());
    auto &snap = msg.content;
    snap_to_pool_snapshot(m_registry, m_entity_map, snap.entities, snap.pools, snap.should_accumulate_discontinuities);
    wake_up_island_residents(m_registry, snap.entities, m_entity_map);
}

void simulation_worker::on_wake_up_residents(message<msg::wake_up_residents> &msg) {
    wake_up_island_residents(m_registry, msg.content.residents, m_entity_map);
}

void simulation_worker::on_change_rigidbody_kind(message<msg::change_rigidbody_kind> &msg) {
    for (auto [entity, kind] : msg.content.changes) {
        internal::rigidbody_apply_kind(m_registry, m_entity_map.at(entity), kind, m_island_manager);
    }
}

}
