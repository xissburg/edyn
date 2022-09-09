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
#include "edyn/networking/sys/decay_discontinuities.hpp"
#include "edyn/networking/util/process_extrapolation_result.hpp"
#include "edyn/networking/util/snap_to_pool_snapshot.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/time/time.hpp"
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
#include "edyn/networking/comp/discontinuity.hpp"
#include <algorithm>
#include <entt/entity/registry.hpp>
#include <numeric>

namespace edyn {

void simulation_worker_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t worker_intptr;
    archive(worker_intptr);
    auto *worker = reinterpret_cast<simulation_worker *>(worker_intptr);

    if (worker->is_terminating()) {
        // `worker` is dynamically allocated and must be manually deallocated
        // when it terminates.
        worker->do_terminate();
        delete worker;
    } else {
        worker->update();
    }
}

simulation_worker::simulation_worker(const settings &settings,
                                     const registry_operation_context &reg_op_ctx,
                                     const material_mix_table &material_table)
    : m_state(state::init)
    , m_solver(m_registry)
    , m_op_builder((*reg_op_ctx.make_reg_op_builder)(m_registry))
    , m_op_observer((*reg_op_ctx.make_reg_op_observer)(*m_op_builder))
    , m_raycast_service(m_registry)
    , m_island_manager(m_registry)
    , m_poly_initializer(m_registry)
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

    m_this_job.func = &simulation_worker_func;
    auto archive = fixed_memory_output_archive(m_this_job.data.data(), m_this_job.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);

    m_last_time = performance_time();
    m_step_start_time = m_last_time;
    m_island_manager.set_last_time(m_last_time);

    // Reschedule every time a message is added to the queue.
    m_message_queue.push_sink().connect<&simulation_worker::reschedule>(*this);
}

void simulation_worker::init() {
    m_registry.on_construct<graph_node>().connect<&simulation_worker::on_construct_shared_entity>(*this);
    m_registry.on_construct<graph_edge>().connect<&simulation_worker::on_construct_shared_entity>(*this);
    m_registry.on_construct<island_tag>().connect<&simulation_worker::on_construct_shared_entity>(*this);

    m_registry.on_destroy<graph_node>().connect<&simulation_worker::on_destroy_shared_entity>(*this);
    m_registry.on_destroy<graph_edge>().connect<&simulation_worker::on_destroy_shared_entity>(*this);
    m_registry.on_destroy<island_tag>().connect<&simulation_worker::on_destroy_shared_entity>(*this);

    m_message_queue.sink<msg::update_entities>().connect<&simulation_worker::on_update_entities>(*this);
    m_message_queue.sink<msg::set_paused>().connect<&simulation_worker::on_set_paused>(*this);
    m_message_queue.sink<msg::step_simulation>().connect<&simulation_worker::on_step_simulation>(*this);
    m_message_queue.sink<msg::set_com>().connect<&simulation_worker::on_set_com>(*this);
    m_message_queue.sink<msg::set_settings>().connect<&simulation_worker::on_set_settings>(*this);
    m_message_queue.sink<msg::set_registry_operation_context>().connect<&simulation_worker::on_set_reg_op_ctx>(*this);
    m_message_queue.sink<msg::set_material_table>().connect<&simulation_worker::on_set_material_table>(*this);
    m_message_queue.sink<msg::raycast_request>().connect<&simulation_worker::on_raycast_request>(*this);
    m_message_queue.sink<msg::query_aabb_request>().connect<&simulation_worker::on_query_aabb_request>(*this);
    m_message_queue.sink<msg::apply_network_pools>().connect<&simulation_worker::on_apply_network_pools>(*this);

    auto &settings = m_registry.ctx().at<edyn::settings>();

    // If this is a networked client, expect extrapolation results.
    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        m_message_queue.sink<extrapolation_result>().connect<&simulation_worker::on_extrapolation_result>(*this);
    }

    // Process messages enqueued before the worker was started.
    process_messages();
}

void simulation_worker::on_construct_shared_entity(entt::registry &registry, entt::entity entity) {
    registry.emplace<shared_tag>(entity);
}

void simulation_worker::on_destroy_shared_entity(entt::registry &registry, entt::entity entity) {
    if (m_entity_map.contains_local(entity)) {
        m_entity_map.erase_local(entity);
    }
}

static void import_reg_ops(entt::registry &registry, entity_map &emap, const registry_operation &ops) {
    ops.execute(registry, emap);

    auto &graph = registry.ctx().at<entity_graph>();
    auto node_view = registry.view<graph_node>();
    auto procedural_view = registry.view<procedural_tag>();

    // Insert nodes in the graph for rigid bodies and external entities, and
    // edges for constraints, because `graph_node` and `graph_edge` are not
    // shared components.
    ops.emplace_for_each<rigidbody_tag, external_tag>([&](entt::entity remote_entity) {
        auto local_entity = emap.at(remote_entity);
        auto procedural = procedural_view.contains(local_entity);
        auto node_index = graph.insert_node(local_entity, !procedural);
        registry.emplace<graph_node>(local_entity, node_index);
    });

    auto insert_edge = [&](entt::entity remote_entity, const auto &con) {
        auto local_entity = emap.at(remote_entity);

        // There could be multiple constraints (of different types) assigned to
        // the same entity, which means it could already have an edge.
        if (registry.any_of<graph_edge>(local_entity)) return;

        auto &node0 = node_view.get<graph_node>(emap.at(con.body[0]));
        auto &node1 = node_view.get<graph_node>(emap.at(con.body[1]));
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        registry.emplace<graph_edge>(local_entity, edge_index);
    };
    ops.emplace_for_each(constraints_tuple, insert_edge);
    ops.emplace_for_each<null_constraint>(insert_edge);

    // When orientation is set manually, a few dependent components must be
    // updated, e.g. AABB, cached origin, inertia_world_inv, rotated meshes...
    ops.replace_for_each<orientation>([&](entt::entity remote_entity, const orientation &orn) {
        auto local_entity = emap.at(remote_entity);

        if (auto *origin = registry.try_get<edyn::origin>(local_entity)) {
            auto &com = registry.get<center_of_mass>(local_entity);
            auto &pos = registry.get<position>(local_entity);
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
    });

    // When position is set manually, the AABB and cached origin must be updated.
    ops.replace_for_each<position>([&](entt::entity remote_entity, const position &pos) {
        auto local_entity = emap.at(remote_entity);

        if (auto *origin = registry.try_get<edyn::origin>(local_entity)) {
            auto &com = registry.get<center_of_mass>(local_entity);
            auto &orn = registry.get<orientation>(local_entity);
            *origin = to_world_space(-com, pos, orn);
        }

        if (registry.any_of<AABB>(local_entity)) {
            update_aabb(registry, local_entity);
        }
    });

    auto &settings = registry.ctx().at<edyn::settings>();

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        // Assign previous position and orientation components to dynamic entities
        // for client-side networking extrapolation discontinuity mitigation.
        ops.emplace_for_each<dynamic_tag>([&](entt::entity remote_entity) {
            auto local_entity = emap.at(remote_entity);
            registry.emplace<previous_position>(local_entity);
            registry.emplace<previous_orientation>(local_entity);

            if (registry.all_of<spin_angle>(local_entity)) {
                registry.emplace<previous_spin_angle>(local_entity);
            }
        });
    }
}

void simulation_worker::on_update_entities(const message<msg::update_entities> &msg) {
    // Import components from main registry.
    m_importing = true;
    m_op_observer->set_active(false);

    import_reg_ops(m_registry, m_entity_map, msg.content.ops);

    m_importing = false;
    m_op_observer->set_active(true);

    // Add all new entity mappings to current op builder which will be sent
    // over to the main thread so it can create corresponding mappings between
    // its new entities and the entities that were just created here in this
    // import.
    msg.content.ops.create_for_each([&](entt::entity remote_entity) {
        auto local_entity = m_entity_map.at(remote_entity);
        m_op_builder->add_entity_mapping(local_entity, remote_entity);
    });

    // Wake up all islands involved.
    wake_up_affected_islands(msg.content.ops);
}

bool simulation_worker::all_sleeping() {
    auto sleeping_view = m_registry.view<sleeping_tag>();
    auto island_view = m_registry.view<island_tag>();

    for (auto island_entity : island_view) {
        if (!sleeping_view.contains(island_entity)) {
            return false;
        }
    }

    return true;
}

void simulation_worker::wake_up_affected_islands(const registry_operation &ops) {
    // Collect islands of all entities which had a component
    // emplaced/replaced/removed by the registry operations and wake them up.
    auto resident_view = m_registry.view<island_resident>();
    auto multi_resident_view = m_registry.view<multi_island_resident>();

    for (auto &op : ops.replace_components) {
        for (auto remote_entity : op->entities) {
            if (!m_entity_map.contains(remote_entity)) continue;

            auto local_entity = m_entity_map.at(remote_entity);

            if (resident_view.contains(local_entity)) {
                auto [resident] = resident_view.get(local_entity);

                if (resident.island_entity != entt::null) {
                    m_island_manager.wake_up_island(resident.island_entity);
                }
            } else if (multi_resident_view.contains(local_entity)) {
                auto [resident] = multi_resident_view.get(local_entity);

                for (auto island_entity : resident.island_entities) {
                    m_island_manager.wake_up_island(island_entity);
                }
            }
        }
    }
}

void simulation_worker::sync() {
    // Always update discontinuities since they decay in every step.
    m_op_builder->replace<discontinuity>();
    m_op_builder->replace<discontinuity_spin>();

    // TODO: move this to solver, maybe, idk
    auto body_view = m_registry.view<position, orientation, linvel, angvel>(exclude_sleeping_disabled);
    m_op_builder->replace<position>(body_view.begin(), body_view.end());
    m_op_builder->replace<orientation>(body_view.begin(), body_view.end());
    m_op_builder->replace<linvel>(body_view.begin(), body_view.end());
    m_op_builder->replace<angvel>(body_view.begin(), body_view.end());

    if (!m_op_builder->empty()) {
        auto &&ops = std::move(m_op_builder->finish());
        message_dispatcher::global().send<msg::step_update>(
            {"main"}, m_message_queue.identifier, std::move(ops), m_last_time);
    }
}

void simulation_worker::run_state_machine() {
    switch (m_state) {
    case state::init:
        init();
        m_state = state::start;
        run_state_machine();
        break;
    case state::start:
        process_messages();
        m_state = state::raycast;
        run_state_machine();
        break;
    case state::raycast:
        if (m_raycast_service.update(m_this_job)) {
            consume_raycast_results();
            m_state = state::step;
            run_state_machine();
        }
        break;
    case state::step:
        if (should_step()) {
            m_state = state::begin_step;
            run_state_machine();
        } else {
            m_state = state::start;
            sync();
            maybe_reschedule();
        }
        break;
    case state::begin_step:
        begin_step();
        m_state = state::broadphase;
        run_state_machine();
        break;
    case state::broadphase:
        if (m_registry.ctx().at<broadphase>().update(m_this_job)) {
            // Broadphase creates and destroys manifolds, which are edges in
            // the entity graph. Thus, it is necessary to initialize new edges
            // and split islands right after.
            m_state = state::update_islands;
            run_state_machine();
        }
        break;
    case state::update_islands:
        m_island_manager.update(m_step_start_time);
        m_state = state::narrowphase;
        run_state_machine();
        break;
    case state::narrowphase:
        if (m_registry.ctx().at<narrowphase>().update(m_this_job)) {
            m_state = state::solve;
            run_state_machine();
        }
        break;
    case state::solve:
        if (m_solver.update(m_this_job)) {
            m_state = state::finish_step;
            run_state_machine();
        }
        break;
    case state::finish_step:
        finish_step();
        maybe_reschedule();
        break;
    }
}

void simulation_worker::update() {
    run_state_machine();
}

void simulation_worker::process_messages() {
    m_message_queue.update();

    // Initialize new nodes, edges and shapes that might've been created while
    // processing messages.
    m_island_manager.update(m_step_start_time);
    m_poly_initializer.init_new_shapes();
}

bool simulation_worker::should_step() {
    auto time = performance_time();

    if (m_state == state::begin_step || m_force_step) {
        m_force_step = false;
        m_step_start_time = time;
        return true;
    }

    auto &settings = m_registry.ctx().at<edyn::settings>();

    if (settings.paused || all_sleeping()) {
        return false;
    }

    auto dt = time - m_last_time;

    if (dt < settings.fixed_dt) {
        return false;
    }

    m_step_start_time = time;
    m_state = state::begin_step;

    return true;
}

void simulation_worker::begin_step() {
    EDYN_ASSERT(m_state == state::begin_step);

    auto &settings = m_registry.ctx().at<edyn::settings>();
    if (settings.pre_step_callback) {
        (*settings.pre_step_callback)(m_registry);
    }

    // Calculate islands after running external logic, which could've created
    // and destroyed nodes and edges.
    m_island_manager.update(m_step_start_time);

    // Initialize new shapes. Basically, create rotated meshes for new
    // imported polyhedron shapes.
    m_poly_initializer.init_new_shapes();

    m_state = state::broadphase;
}

void simulation_worker::finish_step() {
    EDYN_ASSERT(m_state == state::finish_step);

    auto dt = m_step_start_time - m_last_time;

    // Set a limit on the number of steps the worker can lag behind the current
    // time to prevent it from getting stuck in the past in case of a
    // substantial slowdown.
    auto &settings = m_registry.ctx().at<edyn::settings>();
    const auto fixed_dt = settings.fixed_dt;

    constexpr int max_lagging_steps = 10;
    auto num_steps = int(std::floor(dt / fixed_dt));

    if (num_steps > max_lagging_steps) {
        auto remainder = dt - num_steps * fixed_dt;
        m_last_time = m_step_start_time - (remainder + max_lagging_steps * fixed_dt);
    } else {
        m_last_time += fixed_dt;
    }

    // Clear actions after they've been consumed.
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(m_registry);
    }

    if (std::holds_alternative<client_network_settings>(settings.network_settings)) {
        auto &network_settings = std::get<client_network_settings>(settings.network_settings);
        decay_discontinuities(m_registry, network_settings.discontinuity_decay_rate);
    }

    if (settings.post_step_callback) {
        (*settings.post_step_callback)(m_registry);
    }

    sync();

    m_state = state::start;
}

void simulation_worker::reschedule_now() {
    job_dispatcher::global().async(m_this_job);
}

void simulation_worker::maybe_reschedule() {
    auto paused = m_registry.ctx().at<edyn::settings>().paused;

    // The update is done and this job can be rescheduled after this point
    auto reschedule_count = m_reschedule_counter.exchange(0, std::memory_order_acq_rel);
    EDYN_ASSERT(reschedule_count != 0);

    // If the number of reschedule requests is greater than one, it means there
    // are external requests involved, not just the normal internal reschedule.
    // Always reschedule for immediate execution in that case.
    if (reschedule_count == 1) {
        if (!paused && !all_sleeping()) {
            reschedule_later();
        }
    } else {
        reschedule();
    }
}

void simulation_worker::reschedule_later() {
    // Only reschedule if it has not been scheduled and updated already.
    auto reschedule_count = m_reschedule_counter.fetch_add(1, std::memory_order_acq_rel);
    if (reschedule_count > 0) return;

    // If the timestamp of the current registry state is more than `m_fixed_dt`
    // before the current time, schedule it to run at a later time.
    auto time = performance_time();
    auto fixed_dt = m_registry.ctx().at<edyn::settings>().fixed_dt;
    auto delta_time = m_last_time + fixed_dt - time;

    if (delta_time > 0) {
        job_dispatcher::global().async_after(delta_time, m_this_job);
    } else {
        job_dispatcher::global().async(m_this_job);
    }
}

void simulation_worker::reschedule() {
    // Only reschedule if it has not been scheduled and updated already.
    auto reschedule_count = m_reschedule_counter.fetch_add(1, std::memory_order_acq_rel);
    if (reschedule_count > 0) return;

    job_dispatcher::global().async(m_this_job);
}

void simulation_worker::consume_raycast_results() {
    auto &dispatcher = message_dispatcher::global();
    m_raycast_service.consume_results([&](unsigned id, raycast_result &result) {
        dispatcher.send<msg::raycast_response>(
            {"main"}, m_message_queue.identifier, id, result);
    });
}

void simulation_worker::on_set_paused(const message<msg::set_paused> &msg) {
    m_registry.ctx().at<edyn::settings>().paused = msg.content.paused;
    m_last_time = performance_time();
}

void simulation_worker::on_step_simulation(const message<msg::step_simulation> &) {
    if (!all_sleeping()) {
        m_force_step = true;
    }
}

void simulation_worker::on_set_settings(const message<msg::set_settings> &msg) {
    m_registry.ctx().at<settings>() = msg.content.settings;
}

void simulation_worker::on_set_reg_op_ctx(const message<msg::set_registry_operation_context> &msg) {
    m_registry.ctx().at<registry_operation_context>() = msg.content.ctx;
    m_op_builder = (*msg.content.ctx.make_reg_op_builder)(m_registry);
    m_op_observer = (*msg.content.ctx.make_reg_op_observer)(*m_op_builder);
}

void simulation_worker::on_set_material_table(const message<msg::set_material_table> &msg) {
    m_registry.ctx().at<material_mix_table>() = msg.content.table;
}

void simulation_worker::on_set_com(const message<msg::set_com> &msg) {
    auto entity = m_entity_map.at(msg.content.entity);
    apply_center_of_mass(m_registry, entity, msg.content.com);
}

void simulation_worker::on_raycast_request(const message<msg::raycast_request> &msg) {
    auto ignore_entities = std::vector<entt::entity>{};

    for (auto remote_entity : msg.content.ignore_entities) {
        if (m_entity_map.contains(remote_entity)) {
            auto local_entity = m_entity_map.at(remote_entity);
            ignore_entities.push_back(local_entity);
        }
    }
    m_raycast_service.add_ray(msg.content.p0, msg.content.p1, msg.content.id, ignore_entities);
}

void simulation_worker::on_query_aabb_request(const message<msg::query_aabb_request> &msg) {
    auto &bphase = m_registry.ctx().at<broadphase>();
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

void simulation_worker::on_query_aabb_of_interest_request(const message<msg::query_aabb_of_interest_request> &msg) {
    auto &bphase = m_registry.ctx().at<broadphase>();
    auto &request = msg.content;
    auto island_view = m_registry.view<island>();
    auto manifold_view = m_registry.view<contact_manifold>();
    entt::sparse_set contained_entities;
    entt::sparse_set island_entities;

    // Collect entities of islands which intersect the AABB of interest.
    bphase.query_islands(request.aabb, [&](entt::entity island_entity) {
        auto [island] = island_view.get(island_entity);

        for (auto entity : island.nodes) {
            if (!contained_entities.contains(entity)) {
                contained_entities.emplace(entity);
            }
        }

        for (auto entity : island.edges) {
            // Ignore contact manifolds.
            if (manifold_view.contains(entity)) {
                continue;
            }

            if (!contained_entities.contains(entity)) {
                contained_entities.emplace(entity);
            }
        }

        if (!island_entities.contains(island_entity)) {
            island_entities.emplace(island_entity);
        }
    });

    bphase.query_non_procedural(request.aabb, [&](entt::entity np_entity) {
        if (!contained_entities.contains(np_entity)) {
            contained_entities.emplace(np_entity);
        }
    });

    auto response = msg::query_aabb_response{};
    response.id = msg.content.id;
    response.island_entities.insert(response.island_entities.end(), island_entities.begin(), island_entities.end());
    response.procedural_entities.insert(response.procedural_entities.end(), contained_entities.begin(), contained_entities.end());

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::query_aabb_response>(
            {"main"}, m_message_queue.identifier, std::move(response));
}

void simulation_worker::on_extrapolation_result(const message<extrapolation_result> &msg) {
    auto &result = msg.content;
    process_extrapolation_result(m_registry, m_entity_map, result);
}

void simulation_worker::on_apply_network_pools(const message<msg::apply_network_pools> &msg) {
    EDYN_ASSERT(!msg.content.pools.empty());
    snap_to_pool_snapshot(m_registry, m_entity_map, msg.content.entities, msg.content.pools);
}

bool simulation_worker::is_terminated() const {
    return m_terminated.load(std::memory_order_acquire);
}

bool simulation_worker::is_terminating() const {
    return m_terminating.load(std::memory_order_acquire);
}

void simulation_worker::terminate() {
    m_terminating.store(true, std::memory_order_release);
    reschedule();
}

void simulation_worker::do_terminate() {
    {
        auto lock = std::lock_guard(m_terminate_mutex);
        m_terminated.store(true, std::memory_order_release);
    }
    m_terminate_cv.notify_one();
}

void simulation_worker::join() {
    auto lock = std::unique_lock(m_terminate_mutex);
    m_terminate_cv.wait(lock, [&] { return is_terminated(); });
}

}
