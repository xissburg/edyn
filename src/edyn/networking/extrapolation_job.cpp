#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/config/config.h"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/time/time.hpp"
#include <atomic>

namespace edyn {

void extrapolation_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t job_intptr;
    archive(job_intptr);
    auto *job = reinterpret_cast<extrapolation_job *>(job_intptr);
    job->update();
}

extrapolation_job::extrapolation_job(extrapolation_input &&input,
                                     const settings &settings,
                                     const material_mix_table &material_table,
                                     std::shared_ptr<comp_state_history> state_history)
    : m_input(std::move(input))
    , m_state(state::init)
    , m_current_time(input.start_time)
    , m_solver(m_registry)
    , m_state_history(state_history)
{
    m_registry.set<broadphase_worker>(m_registry);
    m_registry.set<narrowphase>(m_registry);
    m_registry.set<entity_graph>();
    m_registry.set<edyn::settings>(settings);
    m_registry.set<contact_manifold_map>(m_registry);
    m_registry.set<material_mix_table>(material_table);

    // Avoid multi-threading issues in the `should_collide` function by
    // pre-allocating the pools required in there.
    m_registry.prepare<collision_filter>();
    m_registry.prepare<collision_exclusion>();

    m_this_job.func = &extrapolation_job_func;
    auto archive = fixed_memory_output_archive(m_this_job.data.data(), m_this_job.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
}

void extrapolation_job::load_input() {
    // Import entities and components.
    m_input.ops.execute(m_registry, m_entity_map);

    auto &graph = m_registry.ctx<entity_graph>();

    // Create nodes for rigid bodies in entity graph.
    auto insert_graph_node = [&] (entt::entity entity) {
        auto non_connecting = !m_registry.any_of<procedural_tag>(entity);
        auto node_index = graph.insert_node(entity, non_connecting);
        m_registry.emplace<graph_node>(entity, node_index);
    };

    std::apply([&] (auto ... t) {
        (m_registry.view<decltype(t)>().each(insert_graph_node), ...);
    }, std::tuple<rigidbody_tag, external_tag>{});

    // Create edges for constraints in entity graph.
    auto node_view = m_registry.view<graph_node>();
    auto insert_graph_edge = [&] (entt::entity entity, auto &&con) {
        if (m_registry.any_of<graph_edge>(entity)) return;

        auto &node0 = node_view.get<graph_node>(con.body[0]);
        auto &node1 = node_view.get<graph_node>(con.body[1]);
        auto edge_index = graph.insert_edge(entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(entity, edge_index);
    };

    std::apply([&] (auto ... t) {
        (m_registry.view<decltype(t)>().each(insert_graph_edge), ...);
    }, constraints_tuple);

    // Create rotated meshes for new imported polyhedron shapes.
    create_rotated_meshes();

    // Replace client component state by server state.
    for (auto &pool : m_input.transient_snapshot.pools) {
        pool.ptr->replace_into_registry(m_registry, m_input.transient_snapshot.entities, m_entity_map);
    }

    // Apply all inputs before the current time to start the simulation
    // with the correct initial inputs.
    m_state_history->until(m_current_time, [&] (auto &&element) {
        element.ops.execute(m_registry, m_entity_map, true);
    });

    // Update calculated properties after setting initial state.
    update_origins(m_registry);
    update_rotated_meshes(m_registry);
    update_aabbs(m_registry);
    update_inertias(m_registry);
}

void extrapolation_job::init() {
    m_start_time = performance_time();

    m_registry.on_destroy<graph_node>().connect<&extrapolation_job::on_destroy_graph_node>(*this);
    m_registry.on_destroy<graph_edge>().connect<&extrapolation_job::on_destroy_graph_edge>(*this);
    m_registry.on_destroy<rotated_mesh_list>().connect<&extrapolation_job::on_destroy_rotated_mesh_list>(*this);

    // Import entities and components to be extrapolated.
    load_input();

    // Initialize external systems.
    auto &settings = m_registry.ctx<edyn::settings>();
    if (settings.external_system_init) {
        (*settings.external_system_init)(m_registry);
    }

    // Run broadphase to initialize the internal dynamic trees with the
    // imported AABBs.
    auto &bphase = m_registry.ctx<broadphase_worker>();
    bphase.update();

    m_state = state::step;
}

void extrapolation_job::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx<entity_graph>();

    m_destroying_node = true;

    graph.visit_edges(node.node_index, [&] (auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);
    });

    m_destroying_node = false;

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);
}

void extrapolation_job::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    if (!m_destroying_node) {
        auto &edge = registry.get<graph_edge>(entity);
        registry.ctx<entity_graph>().remove_edge(edge.edge_index);
    }
}

void extrapolation_job::on_destroy_rotated_mesh_list(entt::registry &registry, entt::entity entity) {
    auto &rotated = registry.get<rotated_mesh_list>(entity);
    if (rotated.next != entt::null) {
        registry.destroy(rotated.next);
    }
}

void extrapolation_job::apply_history() {
    auto &settings = m_registry.ctx<edyn::settings>();
    auto start_time = m_current_time - settings.fixed_dt;

    m_state_history->each(start_time, settings.fixed_dt, [&] (auto &&element) {
        element.ops.execute(m_registry, m_entity_map, true);
    });
}

void extrapolation_job::sync_and_finish() {
    // Update continuous components.
    auto &settings = m_registry.ctx<edyn::settings>();
    auto &index_source = *settings.index_source;
    auto manifold_view = m_registry.view<contact_manifold>();

    // Collect entities per type to be updated, including only components that
    // have changed, i.e. continuous and dirty components.
    auto builder = (*settings.make_reg_op_builder)();

    // Local entity mapping must not be included if the result is going to be
    // remapped into remote space.
    if (!m_input.should_remap) {
        m_entity_map.each([&] (auto remote_entity, auto local_entity) {
            builder->add_entity_mapping(local_entity, remote_entity);
        });
    }

    for (auto remote_entity : m_input.entities) {
        if (!m_entity_map.contains(remote_entity)) continue;

        auto local_entity = m_entity_map.at(remote_entity);

        // Manifolds are shared separately.
        if (!m_registry.valid(local_entity) || manifold_view.contains(local_entity)) continue;

        // Do not include input components of entities owned by the client,
        // since that would cause the latest user inputs to be replaced.
        // Note the use of `remote_entity` next.
        auto is_owned_entity = m_input.owned_entities.contains(remote_entity);

        if (auto *cont = m_registry.try_get<continuous>(local_entity)) {
            for (size_t i = 0; i < cont->size; ++i) {
                auto id = index_source.type_id_of(cont->indices[i]);

                if (!is_owned_entity || !(*m_input.is_input_component_func)(id)) {
                    builder->replace_type_id(m_registry, local_entity, id);
                }
            }
        }

        if (auto *dirty = m_registry.try_get<edyn::dirty>(local_entity)) {
            // Only consider updated indices. Entities and components shouldn't be
            // created during extrapolation.
            for (auto id : dirty->updated_indexes) {
                if (!is_owned_entity || !(*m_input.is_input_component_func)(id)) {
                    builder->replace_type_id(m_registry, local_entity, id);
                }
            }
        }

        m_result.entities.push_back(local_entity);
    }

    m_result.ops = builder->finish();
    EDYN_ASSERT(!m_result.ops.empty());

    // Insert all manifolds into it.
    manifold_view.each([&] (contact_manifold &manifold) {
        m_result.manifolds.push_back(manifold);
    });

    // Assign timestamp of the last step.
    m_result.timestamp = m_current_time;

    if (m_input.should_remap) {
        m_entity_map.swap();
        m_result.remap(m_entity_map);
    }

    m_finished.store(true, std::memory_order_release);
}

void extrapolation_job::update() {
    switch (m_state) {
    case state::init:
        init();
        reschedule();
        break;
    case state::step:
        if (should_step()) {
            begin_step();
            if (run_broadphase()) {
                if (run_narrowphase()) {
                    run_solver();
                    finish_step();
                    reschedule();
                }
            }
        }

        break;
    case state::begin_step:
        begin_step();
        reschedule();
        break;
    case state::solve:
        run_solver();
        finish_step();
        reschedule();
        break;
    case state::broadphase:
        if (run_broadphase()) {
            reschedule();
        }
        break;
    case state::broadphase_async:
        finish_broadphase();
        if (run_narrowphase()) {
            run_solver();
            finish_step();
            reschedule();
        }
        break;
    case state::narrowphase:
        if (run_narrowphase()) {
            run_solver();
            finish_step();
            reschedule();
        }
        break;
    case state::narrowphase_async:
        finish_narrowphase();
        run_solver();
        finish_step();
        reschedule();
        break;
    case state::finish_step:
        finish_step();
        reschedule();
        break;
    }
}

bool extrapolation_job::should_step() {
    auto time = performance_time();

    if (time - m_start_time > m_input.execution_time_limit) {
        // Timeout.
        m_result.terminated_early = true;
        sync_and_finish();
        return false;
    }

    auto &settings = m_registry.ctx<edyn::settings>();

    if (m_current_time + settings.fixed_dt > time) {
        // Job is done.
        sync_and_finish();
        return false;
    }

    m_state = state::begin_step;

    return true;
}

void extrapolation_job::begin_step() {
    EDYN_ASSERT(m_state == state::begin_step);

    apply_history();

    auto &settings = m_registry.ctx<edyn::settings>();
    if (settings.external_system_pre_step) {
        (*settings.external_system_pre_step)(m_registry);
    }

    m_state = state::broadphase;
}

bool extrapolation_job::run_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase);
    auto &bphase = m_registry.ctx<broadphase_worker>();

    if (bphase.parallelizable()) {
        m_state = state::broadphase_async;
        bphase.update_async(m_this_job);
        return false;
    } else {
        bphase.update();
        m_state = state::narrowphase;
        return true;
    }
}

void extrapolation_job::finish_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase_async);
    auto &bphase = m_registry.ctx<broadphase_worker>();
    bphase.finish_async_update();
    m_state = state::narrowphase;
}

bool extrapolation_job::run_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase);
    auto &nphase = m_registry.ctx<narrowphase>();

    if (nphase.parallelizable()) {
        m_state = state::narrowphase_async;
        nphase.update_async(m_this_job);
        return false;
    } else {
        nphase.update();
        m_state = state::solve;
        return true;
    }
}

void extrapolation_job::finish_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase_async);
    auto &nphase = m_registry.ctx<narrowphase>();
    nphase.finish_async_update();
    m_state = state::solve;
}

void extrapolation_job::run_solver() {
    EDYN_ASSERT(m_state == state::solve);
    m_solver.update(m_registry.ctx<edyn::settings>().fixed_dt);
    m_state = state::finish_step;
}

void extrapolation_job::finish_step() {
    EDYN_ASSERT(m_state == state::finish_step);

    auto &settings = m_registry.ctx<edyn::settings>();
    m_current_time += settings.fixed_dt;

    if (settings.external_system_post_step) {
        (*settings.external_system_post_step)(m_registry);
    }

    ++m_step_count;
    m_state = state::step;
}

void extrapolation_job::reschedule() {
    job_dispatcher::global().async(m_this_job);
}

void extrapolation_job::create_rotated_meshes() {
    auto orn_view = m_registry.view<orientation>();
    auto polyhedron_view = m_registry.view<polyhedron_shape>();
    auto compound_view = m_registry.view<compound_shape>();

    for (auto [entity, polyhedron] : polyhedron_view.each()) {
        auto [orn] = orn_view.get(entity);
        auto rotated = make_rotated_mesh(*polyhedron.mesh, orn);
        auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
        polyhedron.rotated = rotated_ptr.get();
        m_registry.emplace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr));
    }

    for (auto [entity, compound] : compound_view.each()) {
        auto [orn] = orn_view.get(entity);
        auto prev_rotated_entity = entt::entity{entt::null};

        for (auto &node : compound.nodes) {
            if (!std::holds_alternative<polyhedron_shape>(node.shape_var)) continue;

            // Assign a `rotated_mesh_list` to this entity for the first
            // polyhedron and link it with more rotated meshes for the
            // remaining polyhedrons.
            auto &polyhedron = std::get<polyhedron_shape>(node.shape_var);
            auto local_orn = orn * node.orientation;
            auto rotated = make_rotated_mesh(*polyhedron.mesh, local_orn);
            auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
            polyhedron.rotated = rotated_ptr.get();

            if (prev_rotated_entity == entt::null) {
                m_registry.emplace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr), node.orientation);
                prev_rotated_entity = entity;
            } else {
                auto next = m_registry.create();
                m_registry.emplace<rotated_mesh_list>(next, polyhedron.mesh, std::move(rotated_ptr), node.orientation);

                auto &prev_rotated_list = m_registry.get<rotated_mesh_list>(prev_rotated_entity);
                prev_rotated_list.next = next;
                prev_rotated_entity = next;
            }
        }
    }
}

}