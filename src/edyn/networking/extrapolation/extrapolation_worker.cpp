#include "edyn/networking/extrapolation/extrapolation_worker.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/extrapolation/extrapolation_operation.hpp"
#include "edyn/networking/extrapolation/extrapolation_request.hpp"
#include "edyn/networking/settings/client_network_settings.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/core/entity_graph.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

extrapolation_worker::extrapolation_worker(const settings &settings,
                                           const registry_operation_context &reg_op_ctx,
                                           const material_mix_table &material_table,
                                           make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp)
    : m_solver(m_registry)
    , m_poly_initializer(m_registry)
    , m_island_manager(m_registry)
    , m_message_queue(message_dispatcher::global().make_queue<
        extrapolation_request,
        extrapolation_operation_create,
        extrapolation_operation_destroy,
        msg::set_settings,
        msg::set_registry_operation_context,
        msg::set_material_table,
        msg::set_extrapolator_context_settings>("extrapolation_worker"))
{
    m_registry.ctx().emplace<contact_manifold_map>(m_registry);
    m_registry.ctx().emplace<broadphase>(m_registry);
    m_registry.ctx().emplace<narrowphase>(m_registry);
    m_registry.ctx().emplace<entity_graph>();
    m_registry.ctx().emplace<edyn::settings>(settings);
    m_registry.ctx().emplace<registry_operation_context>(reg_op_ctx);
    m_registry.ctx().emplace<material_mix_table>(material_table);

    m_message_queue.sink<extrapolation_request>().connect<&extrapolation_worker::on_extrapolation_request>(*this);
    m_message_queue.sink<extrapolation_operation_create>().connect<&extrapolation_worker::on_extrapolation_operation_create>(*this);
    m_message_queue.sink<extrapolation_operation_destroy>().connect<&extrapolation_worker::on_extrapolation_operation_destroy>(*this);
    m_message_queue.sink<msg::set_settings>().connect<&extrapolation_worker::on_set_settings>(*this);
    m_message_queue.sink<msg::set_registry_operation_context>().connect<&extrapolation_worker::on_set_reg_op_ctx>(*this);
    m_message_queue.sink<msg::set_material_table>().connect<&extrapolation_worker::on_set_material_table>(*this);
    m_message_queue.sink<msg::set_extrapolator_context_settings>().connect<&extrapolation_worker::on_set_extrapolator_context_settings>(*this);
    m_message_queue.push_sink().connect<&extrapolation_worker::on_push_message>(*this);

    m_modified_comp = (*make_extrapolation_modified_comp)(m_registry);
}

extrapolation_worker::~extrapolation_worker() {
    stop();
}

void extrapolation_worker::init() {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    if (client_settings.extrapolation_init_callback) {
        (*client_settings.extrapolation_init_callback)(m_registry);
    }
}

void extrapolation_worker::deinit() {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    if (client_settings.extrapolation_deinit_callback) {
        (*client_settings.extrapolation_deinit_callback)(m_registry);
    }
}

void extrapolation_worker::start() {
    m_running.store(true, std::memory_order_release);

    auto &settings = m_registry.ctx().get<edyn::settings>();
    (*settings.start_thread_func)([](void *args) {
        std::invoke(&extrapolation_worker::run, reinterpret_cast<extrapolation_worker *>(args));
    }, this);
}

void extrapolation_worker::stop() {
    m_running.store(false, std::memory_order_release);
    m_cv.notify_one();
}

void extrapolation_worker::set_settings(const edyn::settings &settings) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_settings>(m_message_queue.identifier, {"unknown"}, settings);
}

void extrapolation_worker::set_material_table(const material_mix_table &material_table) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_material_table>(m_message_queue.identifier, {"unknown"}, material_table);
}

void extrapolation_worker::set_registry_operation_context(const registry_operation_context &reg_op_ctx) {
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_registry_operation_context>(m_message_queue.identifier, {"unknown"}, reg_op_ctx);
}

void extrapolation_worker::set_context_settings(std::shared_ptr<input_state_history_reader> input_history,
                                                make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp) {
    EDYN_ASSERT(make_extrapolation_modified_comp != nullptr);
    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<msg::set_extrapolator_context_settings>(m_message_queue.identifier, {"unknown"},
                                                            input_history, make_extrapolation_modified_comp);
}

void extrapolation_worker::on_extrapolation_request(message<extrapolation_request> &msg) {
    if (m_requests.size() == m_max_requests) {
        m_requests.erase(m_requests.begin());
    }

    m_requests.emplace_back(std::move(msg.content));
}

void extrapolation_worker::on_extrapolation_operation_destroy(message<extrapolation_operation_destroy> &msg) {
    for (auto remote_entity : msg.content.entities) {
        if (!m_entity_map.contains(remote_entity)) {
            continue;
        }

        auto local_entity = m_entity_map.at(remote_entity);
        m_entity_map.erase(remote_entity);

        m_owned_entities.remove(local_entity);
        m_modified_comp->remove_entity(local_entity);

        if (m_registry.valid(local_entity)) {
            m_registry.destroy(local_entity);
        }
    }

    // Islands might've been split.
    m_island_manager.update(m_current_time);

    // Force all split islands to stay asleep.
    m_island_manager.put_all_to_sleep();
}

void extrapolation_worker::on_extrapolation_operation_create(message<extrapolation_operation_create> &msg) {
    auto &ops = msg.content.ops;
    auto &emap = m_entity_map;

    auto &graph = m_registry.ctx().get<entity_graph>();
    auto procedural_view = m_registry.view<procedural_tag>();
    entt::sparse_set local_create_entities;

    ops.execute(m_registry, m_entity_map, [&](operation_base *op) {
        auto op_type = op->operation_type();
        auto remote_entity = op->entity;

        // Insert nodes in the graph for rigid bodies and external entities, and
        // edges for constraints, because `graph_node` and `graph_edge` are not
        // shared components.
        if (op_type == registry_operation_type::emplace && op->payload_type_any_of<rigidbody_tag, external_tag>()) {
            auto local_entity = emap.at(remote_entity);
            auto procedural = procedural_view.contains(local_entity);
            auto node_index = graph.insert_node(local_entity, !procedural);
            m_registry.emplace<graph_node>(local_entity, node_index);

            if (!procedural) {
                // `multi_island_resident` is not a shared component thus add it
                // manually here.
                m_registry.emplace<multi_island_resident>(local_entity);
            }
        }

        if (op_type == registry_operation_type::emplace &&
           (op->payload_type_any_of<null_constraint>() || op->payload_type_any_of(constraints_tuple)))
        {
            auto local_entity = emap.at(remote_entity);

            // There could be multiple constraints (of different types) assigned to
            // the same entity, which means it could already have an edge.
            if (!m_registry.any_of<graph_edge>(local_entity)) {
                create_graph_edge_for_constraints(m_registry, local_entity, graph, constraints_tuple);
                create_graph_edge_for_constraint<null_constraint>(m_registry, local_entity, graph);
            }
        }

        if (op_type == registry_operation_type::create) {
            auto local_entity = emap.at(remote_entity);
            local_create_entities.push(local_entity);

            // Observe component changes for this entity.
            m_modified_comp->add_entity(local_entity);
        }
    });

    // Initialize shapes for new entities.
    m_poly_initializer.init_new_shapes();

    // Initialize new nodes and edges and create islands.
    m_island_manager.update(m_current_time);

    // Force all new islands to sleep.
    m_island_manager.put_all_to_sleep();

    // Store copy of imported state into local state storage. This represents
    // the remote state that's been most recently seen.
    if (!local_create_entities.empty()) {
        m_modified_comp->export_remote_state(local_create_entities);
    }

    // Collect owned entities.
    for (auto remote_entity : msg.content.owned_entities) {
        auto local_entity = emap.at(remote_entity);
        m_owned_entities.push(local_entity);
    }
}

void extrapolation_worker::on_set_settings(message<msg::set_settings> &msg) {
    const auto &settings = msg.content.settings;
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    auto &current = m_registry.ctx().get<edyn::settings>();
    auto &client_current = std::get<client_network_settings>(current.network_settings);

    if (client_settings.extrapolation_init_callback &&
        client_settings.extrapolation_init_callback != client_current.extrapolation_init_callback) {
        (*client_settings.extrapolation_init_callback)(m_registry);
    }

    current = settings;
}

void extrapolation_worker::on_set_reg_op_ctx(message<msg::set_registry_operation_context> &msg) {
    m_registry.ctx().get<registry_operation_context>() = msg.content.ctx;
}

void extrapolation_worker::on_set_material_table(message<msg::set_material_table> &msg) {
    m_registry.ctx().get<material_mix_table>() = msg.content.table;
}

void extrapolation_worker::on_set_extrapolator_context_settings(message<msg::set_extrapolator_context_settings> &msg) {
    m_input_history = msg.content.input_history;
    m_modified_comp = (*msg.content.make_extrapolation_modified_comp)(m_registry);
}

void extrapolation_worker::on_push_message() {
    m_has_messages.store(true, std::memory_order_release);
    m_cv.notify_one();
}

void extrapolation_worker::apply_history() {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto since_time = m_current_time - settings.fixed_dt;

    if (m_input_history) {
        m_input_history->import_each(since_time, settings.fixed_dt, m_registry, m_entity_map);
    }
}

bool extrapolation_worker::begin_extrapolation(const extrapolation_request &request) {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    m_init_time = (*settings.time_func)();
    m_current_time = request.start_time;
    m_step_count = 0;
    m_island_manager.set_last_time(m_current_time);
    m_terminated_early = false;

    // Initialize new nodes and edges and create islands.
    m_island_manager.update(m_current_time);

    // Collect indices of nodes present in the snapshot.
    auto &graph = m_registry.ctx().get<entity_graph>();
    std::set<entity_graph::index_type> node_indices;
    auto node_view = m_registry.view<graph_node>();
    auto snapshot_entities = entt::sparse_set{};

    // The snapshot entities only include those that have a component that
    // changed recently. Though the extrapolation must include all entities
    // that belong in the same island because entities cannot be simulated in
    // isolation from their island. An island is always simulated as one unit.
    for (auto remote_entity : request.snapshot.entities) {
        // Abort if snapshot contains unknown entities.
        if (!m_entity_map.contains(remote_entity)) {
            return false;
        }

        auto local_entity = m_entity_map.at(remote_entity);
        snapshot_entities.push(local_entity);

        if (node_view.contains(local_entity)) {
            auto node_index = node_view.get<graph_node>(local_entity).node_index;

            if (graph.is_connecting_node(node_index)) {
                node_indices.insert(node_index);
            }
        }
    }

    // Collection of entities in all involved islands.
    auto entities = entt::sparse_set{};

    graph.reach(
        node_indices.begin(), node_indices.end(),
        [&](entt::entity entity) {
            if (!entities.contains(entity)) {
                entities.push(entity);
            }
        }, [&](entt::entity entity) {
            if (!entities.contains(entity)) {
                entities.push(entity);
            }
        }, [](auto) { return true; }, []() {});

    // Wake up all involved islands.
    auto resident_view = m_registry.view<island_resident>();

    for (auto entity : entities) {
        if (resident_view.contains(entity)) {
            auto [resident] = resident_view.get(entity);
            wake_up_island(m_registry, resident.island_entity);
        }
    }

    // Apply last known remote state as the initial state for extrapolation.
    m_modified_comp->import_remote_state(entities);

    if (m_input_history) {
        // Apply inputs that happened before the start time.
        m_input_history->import_latest(m_current_time, m_registry, m_entity_map);
    }

    // Start observing changes before replacing snapshot contents into registry
    // to ensure these changes will be included in the result.
    m_modified_comp->set_observe_changes(true);

    // Replace client component state by latest server state. The snapshot
    // only contains components which have changed since the last update.
    for (auto &pool : request.snapshot.pools) {
        pool.ptr->replace_into_registry(m_registry, request.snapshot.entities, m_entity_map);
    }

    // Assign current state as the last known remote state which will be used
    // as the initial state for future extrapolations involving these entities.
    // Only necessary to include entities in the snapshot since these are the
    // only ones which changed.
    m_modified_comp->export_remote_state(snapshot_entities);

    // Invoke pre-extrapolation callback after setting up initial state.
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    if (client_settings.extrapolation_begin_callback) {
        (*client_settings.extrapolation_begin_callback)(m_registry);
    }

    // Recalculate properties after setting initial state from server.
    auto origin_view = m_registry.view<position, orientation, center_of_mass, origin>();

    for (auto entity : entities) {
        if (origin_view.contains(entity)) {
            auto [pos, orn, com, orig] = origin_view.get(entity);
            orig = to_world_space(-com, pos, orn);
        }

        if (m_registry.any_of<AABB>(entity)) {
            update_aabb(m_registry, entity);
        }

        if (m_registry.any_of<dynamic_tag>(entity)) {
            update_inertia(m_registry, entity);
        }

        if (m_registry.any_of<rotated_mesh_list>(entity)) {
            update_rotated_mesh(m_registry, entity);
        }
    }

    return true;
}

void extrapolation_worker::finish_extrapolation(const extrapolation_request &request) {
    // Invoke post-extrapolation callback before wrapping up.
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    if (client_settings.extrapolation_finish_callback) {
        (*client_settings.extrapolation_finish_callback)(m_registry);
    }

    // Insert modified components into a registry operation to be sent back to
    // the main thread which will assign the extrapolated state to its entities.
    auto &reg_op_ctx = m_registry.ctx().get<registry_operation_context>();
    auto builder = (*reg_op_ctx.make_reg_op_builder)(m_registry);

    // Local entity mapping must not be included if the result is going to be
    // remapped into remote space.
    if (!request.should_remap) {
        m_entity_map.each([&](auto remote_entity, auto local_entity) {
            builder->add_entity_mapping(local_entity, remote_entity);
        });
    }

    // The export function will ignore input components owned by the local
    // client because user inputs must not be overriden by the last value set
    // by extrapolation.
    m_modified_comp->set_observe_changes(false);
    m_modified_comp->export_to_builder(*builder, m_owned_entities);
    m_modified_comp->clear_modified();

    auto result = extrapolation_result{};
    result.ops = std::move(builder->finish());
    EDYN_ASSERT(!result.ops.empty());

    // All manifolds that are not sleeping have been involved in the
    // extrapolation.
    auto manifold_view = m_registry.view<contact_manifold>(entt::exclude_t<sleeping_tag>{});
    manifold_view.each([&](contact_manifold &manifold) {
        if (manifold.num_points > 0) {
            result.manifolds.push_back(manifold);
        }
    });

    // Put all islands to sleep at the end.
    m_island_manager.put_all_to_sleep();

    // Assign timestamp of the last step.
    result.timestamp = m_current_time;

    if (request.should_remap) {
        // Map all entities (including those contained in components) back to
        // the source registry space.
        m_entity_map.swap();
        result.remap(m_entity_map);
        m_entity_map.swap();
    }

    auto &dispatcher = message_dispatcher::global();
    dispatcher.send<extrapolation_result>(request.destination, m_message_queue.identifier, std::move(result));
}

bool extrapolation_worker::should_step(const extrapolation_request &request) {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto time = (*settings.time_func)();

    if (time - m_init_time > request.execution_time_limit) {
        // Timeout.
        m_terminated_early = true;
        return false;
    }

    if (m_current_time > time) {
        // Job is done.
        return false;
    }

    return true;
}

void extrapolation_worker::begin_step() {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    // Clear all action lists before inserting new actions.
    // This will include any actions imported via the registry operations.
    // Very important to clear those to avoid applying fresh new actions right
    // at the beginning of the extrapolation which would lead to large errors.
    if (settings.clear_actions_func) {
        (*settings.clear_actions_func)(m_registry);
    }

    apply_history();

    if (client_settings.extrapolation_pre_step_callback) {
        (*client_settings.extrapolation_pre_step_callback)(m_registry);
    }
}

void extrapolation_worker::finish_step() {
    auto &settings = m_registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);

    m_current_time += settings.fixed_dt;

    if (client_settings.extrapolation_post_step_callback) {
        (*client_settings.extrapolation_post_step_callback)(m_registry);
    }

    ++m_step_count;
}

void extrapolation_worker::extrapolate(const extrapolation_request &request) {
    if (!begin_extrapolation(request)) {
        return;
    }

    auto &bphase = m_registry.ctx().get<broadphase>();
    auto &nphase = m_registry.ctx().get<narrowphase>();

    while (should_step(request)) {
        begin_step();
        bphase.update(true);
        m_island_manager.update(m_current_time);
        nphase.update(true);
        m_solver.update(true);
        finish_step();
    }

    finish_extrapolation(request);
}

void extrapolation_worker::run() {
    init();

    while (m_running.load(std::memory_order_relaxed)) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&]() {
                return m_has_messages.exchange(false, std::memory_order_relaxed) ||
                    !m_running.load(std::memory_order_relaxed);
            });
        }

        do {
            m_message_queue.update();

            if (!m_requests.empty()) {
                auto req = std::move(m_requests.front());
                m_requests.erase(m_requests.begin());
                extrapolate(req);
            }
        } while (!m_requests.empty() && m_has_messages.exchange(false, std::memory_order_relaxed));
    }

    deinit();
}

}
