#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/rotated_mesh_list.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

void extrapolation_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t job_intptr;
    archive(job_intptr);
    auto *job = reinterpret_cast<extrapolation_job *>(job_intptr);
    job->update();
}

extrapolation_job::extrapolation_job(entt::entity island_entity, double target_time,
                                     const settings &settings,
                                     const material_mix_table &material_table,
                                     message_queue_in_out message_queue)
    : m_message_queue(message_queue)
    , m_target_time(target_time)
    , m_state(state::init)
    , m_bphase(m_registry)
    , m_nphase(m_registry)
    , m_solver(m_registry)
    , m_delta_builder((*settings.make_island_delta_builder)())
    , m_importing_delta(false)
    , m_destroying_node(false)
{
    m_registry.set<entity_graph>();
    m_registry.set<edyn::settings>(settings);
    m_registry.set<material_mix_table>(material_table);

    // Avoid multi-threading issues in the `should_collide` function by
    // pre-allocating the pools required in there.
    m_registry.prepare<collision_filter>();
    m_registry.prepare<collision_exclusion>();

    m_island_entity = m_registry.create();
    m_entity_map.insert(island_entity, m_island_entity);

    m_this_job.func = &extrapolation_job_func;
    auto archive = fixed_memory_output_archive(m_this_job.data.data(), m_this_job.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(this);
    archive(ctx_intptr);
}

void extrapolation_job::init() {
    m_registry.on_destroy<graph_node>().connect<&extrapolation_job::on_destroy_graph_node>(*this);
    m_registry.on_destroy<graph_edge>().connect<&extrapolation_job::on_destroy_graph_edge>(*this);
    m_registry.on_destroy<contact_manifold>().connect<&extrapolation_job::on_destroy_contact_manifold>(*this);
    m_registry.on_construct<polyhedron_shape>().connect<&extrapolation_job::on_construct_polyhedron_shape>(*this);
    m_registry.on_construct<compound_shape>().connect<&extrapolation_job::on_construct_compound_shape>(*this);
    m_registry.on_destroy<rotated_mesh_list>().connect<&extrapolation_job::on_destroy_rotated_mesh_list>(*this);
    m_registry.on_construct<polyhedron_shape>().connect<&extrapolation_job::on_construct_polyhedron_shape>(*this);
    m_registry.on_construct<compound_shape>().connect<&extrapolation_job::on_construct_compound_shape>(*this);

    m_message_queue.sink<island_delta>().connect<&extrapolation_job::on_island_delta>(*this);

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
    m_bphase.update();

    // Assign tree view containing the updated broad-phase tree.
    auto tview = m_bphase.view();
    m_registry.emplace<tree_view>(m_island_entity, tview);

    m_state = state::step;
}

void extrapolation_job::on_destroy_contact_manifold(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<contact_manifold>(entity);
    auto num_points = manifold.num_points();

    for (size_t i = 0; i < num_points; ++i) {
        auto contact_entity = manifold.point[i];
        registry.destroy(contact_entity);
        m_delta_builder->destroyed(contact_entity);
    }

    m_delta_builder->destroyed(entity);
}

void extrapolation_job::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx<entity_graph>();

    m_destroying_node = true;

    graph.visit_edges(node.node_index, [&] (entt::entity edge_entity) {
        registry.destroy(edge_entity);
        m_delta_builder->destroyed(edge_entity);
    });

    m_destroying_node = false;

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);

    m_delta_builder->destroyed(entity);
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
    m_importing_delta = true;
    delta.import(m_registry, m_entity_map);

    for (auto remote_entity : delta.created_entities()) {
        if (!m_entity_map.has_rem(remote_entity)) continue;
        auto local_entity = m_entity_map.remloc(remote_entity);
        m_delta_builder->insert_entity_mapping(remote_entity, local_entity);
    }

    auto &graph = m_registry.ctx<entity_graph>();
    auto node_view = m_registry.view<graph_node>();
    auto &index_source = m_delta_builder->get_index_source();

    // Insert nodes in the graph for each rigid body.
    auto insert_node = [this] (entt::entity remote_entity, auto &) {
        insert_remote_node(remote_entity);
    };

    delta.created_for_each<dynamic_tag>(index_source, insert_node);
    delta.created_for_each<static_tag>(index_source, insert_node);
    delta.created_for_each<kinematic_tag>(index_source, insert_node);
    delta.created_for_each<external_tag>(index_source, insert_node);

    // Insert edges in the graph for contact manifolds.
    delta.created_for_each<contact_manifold>(index_source, [&] (entt::entity remote_entity, const contact_manifold &manifold) {
        if (!m_entity_map.has_rem(remote_entity)) return;

        auto local_entity = m_entity_map.remloc(remote_entity);
        auto &node0 = node_view.get(manifold.body[0]);
        auto &node1 = node_view.get(manifold.body[1]);
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(local_entity, edge_index);
        m_new_imported_contact_manifolds.push_back(local_entity);
    });

    // Insert edges in the graph for constraints (except contact constraints).
    delta.created_for_each(constraints_tuple, index_source, [&] (entt::entity remote_entity, const auto &con) {
        // Contact constraints are not added as edges to the graph.
        // The contact manifold which owns them is added instead.
        if constexpr(std::is_same_v<std::decay_t<decltype(con)>, contact_constraint>) return;

        if (!m_entity_map.has_rem(remote_entity)) return;

        auto local_entity = m_entity_map.remloc(remote_entity);
        auto &node0 = node_view.get(con.body[0]);
        auto &node1 = node_view.get(con.body[1]);
        auto edge_index = graph.insert_edge(local_entity, node0.node_index, node1.node_index);
        m_registry.emplace<graph_edge>(local_entity, edge_index);
    });

    // New contact points might be coming from another island after a merge or
    // split and they might not yet have a contact constraint associated with
    // them if they were just created in the last step of the island where it's
    // coming from.
    auto cp_view = m_registry.view<contact_point>();
    auto cc_view = m_registry.view<contact_constraint>();
    auto mat_view = m_registry.view<material>();
    delta.created_for_each<contact_point>(index_source, [&] (entt::entity remote_entity, const contact_point &) {
        if (!m_entity_map.has_rem(remote_entity)) {
            return;
        }

        auto local_entity = m_entity_map.remloc(remote_entity);

        if (cc_view.contains(local_entity)) {
            return;
        }

        auto &cp = cp_view.get(local_entity);

        if (mat_view.contains(cp.body[0]) && mat_view.contains(cp.body[1])) {
            create_contact_constraint(m_registry, local_entity, cp);
        }
    });

    // When orientation is set manually, a few dependent components must be
    // updated, e.g. AABB, cached origin, inertia_world_inv, rotated meshes...
    delta.updated_for_each<orientation>(index_source, [&] (entt::entity remote_entity, const orientation &orn) {
        if (!m_entity_map.has_rem(remote_entity)) return;

        auto local_entity = m_entity_map.remloc(remote_entity);

        if (auto *origin = m_registry.try_get<edyn::origin>(local_entity)) {
            auto &com = m_registry.get<center_of_mass>(local_entity);
            auto &pos = m_registry.get<position>(local_entity);
            *origin = to_world_space(-com, pos, orn);
        }

        if (m_registry.has<AABB>(local_entity)) {
            update_aabb(m_registry, local_entity);
        }

        if (m_registry.has<dynamic_tag>(local_entity)) {
            update_inertia(m_registry, local_entity);
        }

        if (m_registry.has<rotated_mesh_list>(local_entity)) {
            update_rotated_mesh(m_registry, local_entity);
        }
    });

    // When position is set manually, the AABB and cached origin must be updated.
    delta.updated_for_each<position>(index_source, [&] (entt::entity remote_entity, const position &pos) {
        if (!m_entity_map.has_rem(remote_entity)) return;

        auto local_entity = m_entity_map.remloc(remote_entity);

        if (auto *origin = m_registry.try_get<edyn::origin>(local_entity)) {
            auto &com = m_registry.get<center_of_mass>(local_entity);
            auto &orn = m_registry.get<orientation>(local_entity);
            *origin = to_world_space(-com, pos, orn);
        }

        if (m_registry.has<AABB>(local_entity)) {
            update_aabb(m_registry, local_entity);
        }
    });

    m_importing_delta = false;
}

void extrapolation_job::sync() {
    // Always update AABBs since they're needed for broad-phase in the coordinator.
    m_registry.view<AABB>().each([&] (entt::entity entity, AABB &aabb) {
        m_delta_builder->updated(entity, aabb);
    });

    // Always update applied impulses since they're needed to maintain warm starting
    // functioning correctly when constraints are moved from one island to another.
    // TODO: synchronized merges would eliminate the need to share these
    // components continuously.
    m_registry.view<constraint_impulse>().each([&] (entt::entity entity, constraint_impulse &imp) {
        m_delta_builder->updated(entity, imp);
    });

    // Updated contact points are needed when moving entities from one island to
    // another when merging/splitting in the coordinator.
    // TODO: synchronized merges would eliminate the need to share these
    // components continuously.
    m_registry.view<contact_point>().each([&] (entt::entity entity, contact_point &cp) {
        m_delta_builder->updated(entity, cp);
    });

    // Update continuous components.
    m_registry.view<continuous>().each([&] (entt::entity entity, continuous &cont) {
        for (size_t i = 0; i < cont.size; ++i) {
            m_delta_builder->updated(entity, m_registry, cont.types[i]);
        }
    });

    sync_dirty();

    auto delta = m_delta_builder->finish();
    m_message_queue.send<island_delta>(std::move(delta));
}

void extrapolation_job::sync_dirty() {
    // Assign dirty components to the delta builder. This can be called at
    // any time to move the current dirty entities into the next island delta.
    m_registry.view<dirty>().each([&] (entt::entity entity, dirty &dirty) {
        if (dirty.is_new_entity) {
            m_delta_builder->created(entity);
        }

        m_delta_builder->created(entity, m_registry,
            dirty.created_indexes.begin(), dirty.created_indexes.end());
        m_delta_builder->updated(entity, m_registry,
            dirty.updated_indexes.begin(), dirty.updated_indexes.end());
        m_delta_builder->destroyed(entity,
            dirty.destroyed_indexes.begin(), dirty.destroyed_indexes.end());
    });

    m_registry.clear<dirty>();
}

void extrapolation_job::apply_history() {
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto &settings = m_registry.ctx<edyn::settings>();
    auto start_time = isle_time.value - settings.fixed_dt;
    auto end_time = isle_time.value;

    for (auto &delta : m_history) {
        if (delta.m_timestamp >= start_time && delta.m_timestamp < end_time) {
            delta.import(m_registry, m_entity_map);
        }
    }
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
            run_solver();
            if (run_broadphase()) {
                if (run_narrowphase()) {
                    finish_step();
                    reschedule();
                }
            }
        } else {
            reschedule();
        }

        break;
    case state::begin_step:
        begin_step();
        reschedule();
        break;
    case state::solve:
        run_solver();
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
            finish_step();
            reschedule();
        }
        break;
    case state::narrowphase:
        if (run_narrowphase()) {
            finish_step();
            reschedule();
        }
        break;
    case state::narrowphase_async:
        finish_narrowphase();
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

void extrapolation_job::begin_step() {
    EDYN_ASSERT(m_state == state::begin_step);

    apply_history();

    auto &settings = m_registry.ctx<edyn::settings>();
    if (settings.external_system_pre_step) {
        (*settings.external_system_pre_step)(m_registry);
    }

    // Initialize new shapes before new manifolds because it'll perform
    // collision detection for the new manifolds.
    init_new_shapes();
    init_new_imported_contact_manifolds();

    // Create new contact constraints at the beginning of the step. Since
    // contact points are created at the end of a step, creating constraints
    // at that point would mean that they'd have zero applied impulse,
    // which leads to contact point construction observers not getting the
    // value of the initial impulse of a new contact. Doing it here, means
    // that at the end of the step, the `constraint_impulse` will have the
    // value of the impulse applied and the construction of `constraint_impulse`
    // or `contact_constraint` can be observed to capture the initial impact
    // of a new contact.
    m_nphase.create_contact_constraints();

    m_state = state::solve;
}

void extrapolation_job::run_solver() {
    EDYN_ASSERT(m_state == state::solve);
    m_solver.update(m_registry.ctx<edyn::settings>().fixed_dt);
    m_state = state::broadphase;
}

bool extrapolation_job::run_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase);

    if (m_bphase.parallelizable()) {
        m_state = state::broadphase_async;
        m_bphase.update_async(m_this_job);
        return false;
    } else {
        m_bphase.update();
        m_state = state::narrowphase;
        return true;
    }
}

void extrapolation_job::finish_broadphase() {
    EDYN_ASSERT(m_state == state::broadphase_async);
    m_bphase.finish_async_update();
    m_state = state::narrowphase;
}

bool extrapolation_job::run_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase);

    if (m_nphase.parallelizable()) {
        m_state = state::narrowphase_async;
        m_nphase.update_async(m_this_job);
        return false;
    } else {
        m_nphase.update();
        m_state = state::finish_step;
        return true;
    }
}

void extrapolation_job::finish_narrowphase() {
    EDYN_ASSERT(m_state == state::narrowphase_async);
    m_nphase.finish_async_update();
    m_state = state::finish_step;
}

void extrapolation_job::finish_step() {
    EDYN_ASSERT(m_state == state::finish_step);

    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);
    auto &settings = m_registry.ctx<edyn::settings>();
    isle_time.value += settings.fixed_dt;

    m_delta_builder->updated<island_timestamp>(m_island_entity, isle_time);

    // Update tree view.
    auto tview = m_bphase.view();
    m_registry.replace<tree_view>(m_island_entity, tview);
    m_delta_builder->updated(m_island_entity, tview);

    if (settings.external_system_post_step) {
        (*settings.external_system_post_step)(m_registry);
    }

    m_state = state::step;
}

void extrapolation_job::reschedule() {
    auto &isle_time = m_registry.get<island_timestamp>(m_island_entity);

    if (isle_time.value > m_target_time) {
        // job is done.
        sync();
    } else {
        job_dispatcher::global().async(m_this_job);
    }
}

void extrapolation_job::init_new_imported_contact_manifolds() {
    // Entities in the new imported contact manifolds array might've been
    // destroyed. Remove invalid entities before proceeding.
    for (size_t i = 0; i < m_new_imported_contact_manifolds.size();) {
        if (m_registry.valid(m_new_imported_contact_manifolds[i])) {
            ++i;
        } else {
            m_new_imported_contact_manifolds[i] = m_new_imported_contact_manifolds.back();
            m_new_imported_contact_manifolds.pop_back();
        }
    }

    if (m_new_imported_contact_manifolds.empty()) return;

    // Find contact points for new manifolds imported from the main registry.
    m_nphase.update_contact_manifolds(m_new_imported_contact_manifolds.begin(),
                                      m_new_imported_contact_manifolds.end());
    m_new_imported_contact_manifolds.clear();
}

void extrapolation_job::init_new_shapes() {
    auto orn_view = m_registry.view<orientation>();
    auto polyhedron_view = m_registry.view<polyhedron_shape>();
    auto compound_view = m_registry.view<compound_shape>();

    for (auto entity : m_new_polyhedron_shapes) {
        if (!polyhedron_view.contains(entity)) continue;

        auto &polyhedron = polyhedron_view.get(entity);
        // A new `rotated_mesh` is assigned to it, replacing another reference
        // that could be already in there, thus preventing concurrent access.
        auto rotated = make_rotated_mesh(*polyhedron.mesh, orn_view.get(entity));
        auto rotated_ptr = std::make_unique<rotated_mesh>(std::move(rotated));
        polyhedron.rotated = rotated_ptr.get();
        m_registry.emplace<rotated_mesh_list>(entity, polyhedron.mesh, std::move(rotated_ptr));
    }

    for (auto entity : m_new_compound_shapes) {
        if (!compound_view.contains(entity)) continue;

        auto &compound = compound_view.get(entity);
        auto &orn = orn_view.get(entity);
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
    auto non_connecting = !m_registry.has<procedural_tag>(local_entity);

    auto &graph = m_registry.ctx<entity_graph>();
    auto node_index = graph.insert_node(local_entity, non_connecting);
    m_registry.emplace<graph_node>(local_entity, node_index);
}

}