#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
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

extrapolation_job::extrapolation_job(double start_time,
                                     const settings &settings,
                                     const material_mix_table &material_table,
                                     std::shared_ptr<client_pool_snapshot_importer> pool_snapshot_importer,
                                     non_proc_comp_state_history &state_history,
                                     message_queue_in_out message_queue)
    : m_message_queue(message_queue)
    , m_state(state::init)
    , m_current_time(start_time)
    , m_solver(m_registry)
    , m_pool_snapshot_importer(pool_snapshot_importer)
    , m_state_history(&state_history)
    , m_delta_builder((*settings.make_island_delta_builder)())
    , m_destroying_node(false)
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

void extrapolation_job::init() {
    m_registry.on_destroy<graph_node>().connect<&extrapolation_job::on_destroy_graph_node>(*this);
    m_registry.on_destroy<graph_edge>().connect<&extrapolation_job::on_destroy_graph_edge>(*this);
    m_registry.on_construct<polyhedron_shape>().connect<&extrapolation_job::on_construct_polyhedron_shape>(*this);
    m_registry.on_construct<compound_shape>().connect<&extrapolation_job::on_construct_compound_shape>(*this);
    m_registry.on_destroy<rotated_mesh_list>().connect<&extrapolation_job::on_destroy_rotated_mesh_list>(*this);

    m_message_queue.sink<island_delta>().connect<&extrapolation_job::on_island_delta>(*this);
    m_message_queue.sink<packet::transient_snapshot>().connect<&extrapolation_job::on_transient_snapshot>(*this);

    // Process messages enqueued before the job was started. This includes
    // the island deltas containing the initial entities that were added to
    // this island.
    process_messages();

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

    graph.visit_edges(node.node_index, [&] (entt::entity edge_entity) {
        registry.destroy(edge_entity);
    });

    m_destroying_node = false;

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);

    if (m_entity_map.has_loc(entity)) {
        m_entity_map.erase_loc(entity);
    }
}

void extrapolation_job::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    if (!m_destroying_node) {
        auto &edge = registry.get<graph_edge>(entity);
        registry.ctx<entity_graph>().remove_edge(edge.edge_index);
        m_delta_builder->destroyed(entity);
    }

    if (m_entity_map.has_loc(entity)) {
        m_entity_map.erase_loc(entity);
    }
}

void extrapolation_job::on_construct_polyhedron_shape(entt::registry &registry, entt::entity entity) {
    m_new_polyhedron_shapes.push_back(entity);
}

void extrapolation_job::on_construct_compound_shape(entt::registry &registry, entt::entity entity) {
    m_new_compound_shapes.push_back(entity);
}

void extrapolation_job::on_destroy_rotated_mesh_list(entt::registry &registry, entt::entity entity) {
    auto &rotated = registry.get<rotated_mesh_list>(entity);
    if (rotated.next != entt::null) {
        // Cascade delete. Could lead to mega tall call stacks.
        registry.destroy(rotated.next);
    }
}

void extrapolation_job::on_island_delta(const island_delta &delta) {
    // Import components from main registry.
    delta.import(m_registry, m_entity_map);

    for (auto remote_entity : delta.created_entities()) {
        if (!m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = m_entity_map.remloc(remote_entity);
        m_delta_builder->insert_entity_mapping(remote_entity, local_entity);
    }

    auto &graph = m_registry.ctx<entity_graph>();
    auto node_view = m_registry.view<graph_node>();

    // Insert nodes in the graph for each rigid body.
    auto insert_node = [this] (entt::entity remote_entity, auto &) {
        insert_remote_node(remote_entity);
    };

    delta.created_for_each<dynamic_tag>(insert_node);
    delta.created_for_each<static_tag>(insert_node);
    delta.created_for_each<kinematic_tag>(insert_node);
    delta.created_for_each<external_tag>(insert_node);

    // Insert edges in the graph for constraints.
    delta.created_for_each(constraints_tuple, [&] (entt::entity remote_entity, const auto &con) {
        if (!m_entity_map.has_rem(remote_entity)) return;

        auto local_entity = m_entity_map.remloc(remote_entity);

        if (m_registry.any_of<graph_edge>(local_entity)) return;

        auto &node0 = node_view.get<graph_node>(con.body[0]);
        auto &node1 = node_view.get<graph_node>(con.body[1]);
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(local_entity, edge_index);
    });
}

void extrapolation_job::on_transient_snapshot(const packet::transient_snapshot &snapshot) {
    for (auto &pool : snapshot.pools) {
        m_pool_snapshot_importer->import(m_registry, m_entity_map, pool);
    }

    // Import manifolds later, after initial collision detection is done.
    // Existing imported manifolds from the coordinator exist with respect
    // to the client simulation state. The snapshot manifolds are with respect
    // to the received server state.
    m_imported_manifolds.insert(m_imported_manifolds.end(), snapshot.manifolds.begin(), snapshot.manifolds.end());

    // Apply first history state before the current time to start the simulation
    // with the correct initial inputs.
    if (auto *delta = m_state_history->get_first_before(m_current_time)) {
        delta->import(m_registry, m_entity_map);
    }
}

void swap_manifold(contact_manifold &manifold) {
    std::swap(manifold.body[0], manifold.body[1]);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point[i];
        std::swap(cp.pivotA, cp.pivotB);
        std::swap(cp.featureA, cp.featureB);
        cp.normal *= -1; // Point towards new A.

        if (cp.normal_attachment == contact_normal_attachment::normal_on_A) {
            cp.normal_attachment = contact_normal_attachment::normal_on_B;
        } else if (cp.normal_attachment == contact_normal_attachment::normal_on_B) {
            cp.normal_attachment = contact_normal_attachment::normal_on_A;
        }
    }
}

void extrapolation_job::import_manifolds() {
    if (m_imported_manifolds.empty()) {
        return;
    }

    auto &manifold_map = m_registry.ctx<contact_manifold_map>();

    for (auto &manifold : m_imported_manifolds) {
        if (!m_entity_map.has_rem(manifold.body[0]) || !m_entity_map.has_rem(manifold.body[1])) {
            continue;
        }

        manifold.body[0] = m_entity_map.remloc(manifold.body[0]);
        manifold.body[1] = m_entity_map.remloc(manifold.body[1]);

        // Find a matching manifold and replace it.
        if (manifold_map.contains(manifold.body[0], manifold.body[1])) {
            auto manifold_entity = manifold_map.get(manifold.body[0], manifold.body[1]);
            auto &local_manifold = m_registry.get<contact_manifold>(manifold_entity);

            if (local_manifold.body[0] != manifold.body[0]) {
                swap_manifold(manifold);
            }
            EDYN_ASSERT(local_manifold.body[0] == manifold.body[0]);
            EDYN_ASSERT(local_manifold.body[1] == manifold.body[1]);
            local_manifold = manifold;
        }
    }

    m_imported_manifolds.clear();
}

void extrapolation_job::apply_history() {
    auto &settings = m_registry.ctx<edyn::settings>();
    auto start_time = m_current_time - settings.fixed_dt;

    m_state_history->each(start_time, settings.fixed_dt, [&] (island_delta &delta, double timestamp) {
        delta.import(m_registry, m_entity_map);
    });
}

void extrapolation_job::sync_and_finish() {
    m_registry.view<AABB>().each([&] (entt::entity entity, AABB &aabb) {
        m_delta_builder->updated(entity, aabb);
    });

    m_registry.view<contact_manifold>().each([&] (entt::entity entity, contact_manifold &manifold) {
        m_delta_builder->updated(entity, manifold);
    });

    // Update continuous components.
    auto &settings = m_registry.ctx<edyn::settings>();
    auto &index_source = *settings.index_source;
    m_registry.view<continuous>().each([&] (entt::entity entity, continuous &cont) {
        for (size_t i = 0; i < cont.size; ++i) {
            m_delta_builder->updated(entity, m_registry, index_source.type_id_of(cont.indices[i]));
        }
    });

    auto delta = m_delta_builder->finish();
    m_message_queue.send<extrapolation_completed>(std::move(delta), m_current_time);

    m_finished.store(true, std::memory_order_release);
}

void extrapolation_job::update() {
    switch (m_state) {
    case state::init:
        init();
        reschedule();
        break;
    case state::step:
        process_messages();

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

void extrapolation_job::process_messages() {
    m_message_queue.update();
}

bool extrapolation_job::should_step() {
    auto time = performance_time();
    auto &settings = m_registry.ctx<edyn::settings>();

    if (m_current_time + settings.fixed_dt > time) {
        // job is done.
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

    // Initialize new shapes. Basically, create rotated meshes for new
    // imported polyhedron shapes.
    init_new_shapes();

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
        import_manifolds();
        m_state = state::solve;
        return true;
    }
}

void extrapolation_job::finish_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase_async);
    auto &nphase = m_registry.ctx<narrowphase>();
    nphase.finish_async_update();
    import_manifolds();
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

    m_state = state::step;
}

void extrapolation_job::reschedule() {
    job_dispatcher::global().async(m_this_job);
}

void extrapolation_job::init_new_shapes() {
    auto orn_view = m_registry.view<orientation>();
    auto polyhedron_view = m_registry.view<polyhedron_shape>();
    auto compound_view = m_registry.view<compound_shape>();

    for (auto entity : m_new_polyhedron_shapes) {
        if (!polyhedron_view.contains(entity)) continue;

        auto &polyhedron = polyhedron_view.get<polyhedron_shape>(entity);
        // A new `rotated_mesh` is assigned to it, replacing another reference
        // that could be already in there, thus preventing concurrent access.
        auto rotated = make_rotated_mesh(*polyhedron.mesh, orn_view.get<orientation>(entity));
        auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
        polyhedron.rotated = rotated_ptr.get();
        m_registry.emplace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr));
    }

    for (auto entity : m_new_compound_shapes) {
        if (!compound_view.contains(entity)) continue;

        auto &compound = compound_view.get<compound_shape>(entity);
        auto &orn = orn_view.get<orientation>(entity);
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

    m_new_polyhedron_shapes.clear();
    m_new_compound_shapes.clear();
}

void extrapolation_job::insert_remote_node(entt::entity remote_entity) {
    if (!m_entity_map.has_rem(remote_entity)) return;

    auto local_entity = m_entity_map.remloc(remote_entity);
    auto non_connecting = !m_registry.any_of<procedural_tag>(local_entity);

    auto &graph = m_registry.ctx<entity_graph>();
    auto node_index = graph.insert_node(local_entity, non_connecting);
    m_registry.emplace<graph_node>(local_entity, node_index);
}

}