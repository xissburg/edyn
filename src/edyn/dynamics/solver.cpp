#include "edyn/dynamics/solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/context/task.hpp"
#include "edyn/dynamics/island_constraint_entities.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/parallel/atomic_counter_sync.hpp"
#include "edyn/serialization/s11n_util.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/sys/update_inertias.hpp"
#include "edyn/sys/update_origins.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/restitution_solver.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/context/task_util.hpp"
#include "edyn/util/entt_util.hpp"
#include <entt/entity/registry.hpp>
#include <entt/signal/delegate.hpp>
#include <optional>
#include <type_traits>

namespace edyn {

solver::solver(entt::registry &registry)
    : m_registry(&registry)
{
    m_connections.emplace_back(registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>());
    m_connections.emplace_back(registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>());
    m_connections.emplace_back(registry.on_construct<island_tag>().connect<&entt::registry::emplace<row_cache>>());
    m_connections.emplace_back(registry.on_construct<island_tag>().connect<&entt::registry::emplace<island_constraint_entities>>());
    m_connections.emplace_back(registry.on_construct<constraint_tag>().connect<&entt::registry::emplace<constraint_row_prep_cache>>());
}

solver::~solver() {
    m_registry->clear<delta_linvel, delta_angvel>();
    m_registry->clear<row_cache>();
    m_registry->clear<island_constraint_entities>();
    m_registry->clear<constraint_row_prep_cache>();
}

template<typename C, typename BodyView, typename OriginView, typename ManifoldView, typename ProceduralView, typename StaticView>
void invoke_prepare_constraint(entt::registry &registry, entt::entity entity, C &&con,
                               constraint_row_prep_cache &cache, scalar dt,
                               const BodyView &body_view, const OriginView &origin_view,
                               const ManifoldView &manifold_view, const ProceduralView &procedural_view,
                               const StaticView &static_view) {
    auto [posA, ornA, dvA, dwA] = body_view.template get<position, orientation, delta_linvel, delta_angvel>(con.body[0]);
    auto [posB, ornB, dvB, dwB] = body_view.template get<position, orientation, delta_linvel, delta_angvel>(con.body[1]);

    // Get velocity from registry for non-static entities (dynamic and kinematic).
    // Get mass and inertia from registry for procedural entities (dynamic only).
    // Use zero mass, inertia and velocities otherwise.
    vector3 linvelA, linvelB;
    vector3 angvelA, angvelB;
    scalar inv_mA, inv_mB;
    matrix3x3 inv_IA, inv_IB;

    if (procedural_view.contains(con.body[0])) {
        inv_mA = body_view.template get<mass_inv>(con.body[0]);
        inv_IA = body_view.template get<inertia_world_inv>(con.body[0]);
    } else {
        inv_mA = 0;
        inv_IA = matrix3x3_zero;
    }

    if (static_view.contains(con.body[0])) {
        linvelA = vector3_zero;
        angvelA = vector3_zero;
    } else {
        linvelA = body_view.template get<linvel>(con.body[0]);
        angvelA = body_view.template get<angvel>(con.body[0]);
    }

    if (procedural_view.contains(con.body[1])) {
        inv_mB = body_view.template get<mass_inv>(con.body[1]);
        inv_IB = body_view.template get<inertia_world_inv>(con.body[1]);
    } else {
        inv_mB = 0;
        inv_IB = matrix3x3_zero;
    }

    if (static_view.contains(con.body[1])) {
        linvelB = vector3_zero;
        angvelB = vector3_zero;
    } else {
        linvelB = body_view.template get<linvel>(con.body[1]);
        angvelB = body_view.template get<angvel>(con.body[1]);
    }

    auto originA = origin_view.contains(con.body[0]) ?
        origin_view.template get<origin>(con.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(con.body[1]) ?
        origin_view.template get<origin>(con.body[1]) : static_cast<vector3>(posB);

    const auto bodyA = constraint_body{originA, posA, ornA, linvelA, angvelA, inv_mA, inv_IA};
    const auto bodyB = constraint_body{originB, posB, ornB, linvelB, angvelB, inv_mB, inv_IB};

    cache.add_constraint();

    // Grab index of first row so all rows that will be added can be iterated
    // later to finish their setup. Note that no rows could be added as well.
    auto row_start_index = cache.num_rows;

    if constexpr(std::is_same_v<std::decay_t<C>, contact_constraint>) {
        auto &manifold = manifold_view.template get<contact_manifold>(entity);
        EDYN_ASSERT(manifold.body[0] == con.body[0]);
        EDYN_ASSERT(manifold.body[1] == con.body[1]);
        con.prepare(registry, entity, manifold, cache, dt, bodyA, bodyB);
    } else {
        con.prepare(registry, entity, cache, dt, bodyA, bodyB);
    }

    // Assign masses and deltas to new rows.
    for (auto i = row_start_index; i < cache.num_rows; ++i) {
        auto &row = cache.rows[i].row;
        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;

        auto &options = cache.rows[i].options;
        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
    }
}

static void prepare_constraints(entt::registry &registry, scalar dt, bool mt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto origin_view = registry.view<origin>();
    auto cache_view = registry.view<constraint_row_prep_cache>(exclude_sleeping_disabled);
    auto manifold_view = registry.view<contact_manifold>();
    auto procedural_view = registry.view<procedural_tag>();
    auto static_view = registry.view<static_tag>();
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);

    auto for_loop_body = [&registry, body_view, cache_view, origin_view,
                          manifold_view, procedural_view, static_view, con_view_tuple, dt](entt::entity entity) {
        auto &prep_cache = cache_view.get<constraint_row_prep_cache>(entity);
        prep_cache.clear();

        std::apply([&](auto &&... con_view) {
            ((con_view.contains(entity) ?
                invoke_prepare_constraint(registry, entity, std::get<0>(con_view.get(entity)), prep_cache,
                                          dt, body_view, origin_view, manifold_view, procedural_view, static_view) : void(0)), ...);
        }, con_view_tuple);
    };

    const size_t max_sequential_size = 4;
    auto num_constraints = calculate_view_size(cache_view);

    if (mt && num_constraints > max_sequential_size) {
        auto task_func = [&for_loop_body, cache_view](unsigned start, unsigned end) {
            auto first = cache_view.begin();
            std::advance(first, start);
            auto last = first;
            std::advance(last, end - start);

            for (; first != last; ++first) {
                auto entity = *first;
                for_loop_body(entity);
            }
        };

        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(registry, task, calculate_view_size(cache_view));
    } else {
        for (auto entity : cache_view) {
            for_loop_body(entity);
        }
    }
}

void solver::update(bool mt) {
    auto &registry = *m_registry;
    auto &settings = registry.ctx().get<edyn::settings>();
    auto dt = settings.fixed_dt;

    solve_restitution(registry, dt);
    apply_gravity(registry, dt);

    prepare_constraints(registry, dt, mt);

    auto island_view = registry.view<island>(exclude_sleeping_disabled);
    auto num_islands = calculate_view_size(island_view);

    if (mt && num_islands > 1) {
        auto counter = atomic_counter_sync(num_islands);

        for (auto island_entity : island_view) {
            run_island_solver_seq_mt(registry, island_entity,
                                     settings.num_solver_velocity_iterations,
                                     settings.num_solver_position_iterations,
                                     dt, &counter);
        }

        counter.wait();
    } else {
        for (auto island_entity : island_view) {
            run_island_solver_seq(registry, island_entity,
                                  settings.num_solver_velocity_iterations,
                                  settings.num_solver_position_iterations, dt);
        }
    }

    update_origins(registry);

    // Update rotated vertices of convex meshes after rotations change. It is
    // important to do this before `update_aabbs` because the rotated meshes
    // will be used to calculate AABBs of polyhedrons.
    update_rotated_meshes(registry);

    // Update AABBs after transforms change.
    update_aabbs(registry);
    update_island_aabbs(registry);

    // Update world-space moment of inertia.
    update_inertias(registry);
}

}
