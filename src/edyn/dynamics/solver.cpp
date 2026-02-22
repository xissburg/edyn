#include "edyn/dynamics/solver.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/island_solver.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/context/profile.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/profile_util.hpp"
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

static thread_local delta_linvel dummy_dv {vector3_zero};
static thread_local delta_angvel dummy_dw {vector3_zero};

template<typename View, typename Func>
auto make_view_for_each_task_func(View &view, Func &func) {
    return [&](unsigned start, unsigned end) {
        auto first = view.begin();
        std::advance(first, start);
        auto last = first;
        std::advance(last, end - start);

        for (; first != last; ++first) {
            auto entity = *first;
            func(entity);
        }
    };
}

template<typename C, typename BodyView, typename OriginView, typename ProceduralView, typename StaticView>
void invoke_prepare_constraint(entt::registry &/*registry*/, entt::entity /*entity*/, C &&con,
                               constraint_row_prep_cache &cache, scalar dt,
                               const BodyView &body_view, const OriginView &origin_view,
                               const ProceduralView &procedural_view, const StaticView &static_view) {
    auto [posA, ornA] = body_view.template get<position, orientation>(con.body[0]);
    auto [posB, ornB] = body_view.template get<position, orientation>(con.body[1]);

    // Get velocity from registry for non-static entities (dynamic and kinematic).
    // Get mass and inertia from registry for procedural entities (dynamic only).
    // Use zero mass, inertia and velocities otherwise.
    vector3 linvelA, linvelB;
    vector3 angvelA, angvelB;
    scalar inv_mA {0}, inv_mB {0};
    matrix3x3 inv_IA {}, inv_IB {};
    delta_linvel *dvA {nullptr}, *dvB {nullptr};
    delta_angvel *dwA {nullptr}, *dwB {nullptr};

    if (procedural_view.contains(con.body[0])) {
        inv_mA = body_view.template get<mass_inv>(con.body[0]).s;
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

    if (procedural_view.contains(con.body[0])) {
        dvA = &body_view.template get<delta_linvel>(con.body[0]);
        dwA = &body_view.template get<delta_angvel>(con.body[0]);
    } else {
        dvA = &dummy_dv;
        dwA = &dummy_dw;
    }

    if (procedural_view.contains(con.body[1])) {
        inv_mB = body_view.template get<mass_inv>(con.body[1]).s;
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

    if (procedural_view.contains(con.body[1])) {
        dvB = &body_view.template get<delta_linvel>(con.body[1]);
        dwB = &body_view.template get<delta_angvel>(con.body[1]);
    } else {
        dvB = &dummy_dv;
        dwB = &dummy_dw;
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
    con.prepare(cache, dt, bodyA, bodyB);

    // Assign masses and deltas to new rows.
    for (auto i = row_start_index; i < cache.num_rows; ++i) {
        auto &row = cache.rows[i].row;
        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = dvA; row.dwA = dwA;
        row.dvB = dvB; row.dwB = dwB;

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
    auto procedural_view = registry.view<procedural_tag>();
    auto static_view = registry.view<static_tag>();
    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);

    auto for_loop_body = [&](entt::entity entity) {
        auto &prep_cache = cache_view.get<constraint_row_prep_cache>(entity);
        prep_cache.clear();

        std::apply([&](auto &&... con_view) {
            ((con_view.contains(entity) ?
                invoke_prepare_constraint(registry, entity, std::get<0>(con_view.get(entity)), prep_cache,
                                          dt, body_view, origin_view, procedural_view, static_view) : void(0)), ...);
        }, con_view_tuple);
    };

    // Ensure dummy delta velocities are always zero.
    dummy_dv = vector3_zero;
    dummy_dw = vector3_zero;

    const size_t max_sequential_size = 4;
    auto num_constraints = calculate_view_size(cache_view);

    if (mt && num_constraints > max_sequential_size) {
        auto task_func = make_view_for_each_task_func(cache_view, for_loop_body);
        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(registry, task, static_cast<unsigned>(num_constraints));
    } else {
        for (auto entity : cache_view) {
            for_loop_body(entity);
        }
    }
}

void copy_contact_point_to_constraint(contact_constraint &con,
                                      const contact_point &cp,
                                      const contact_point_geometry &cp_geom,
                                      const contact_point_material &cp_mat,
                                      const contact_point_impulse &cp_imp,
                                      bool use_restitution_solver) {
    con.pivotA = cp.pivotA;
    con.pivotB = cp.pivotB;
    con.normal = cp.normal;

    con.local_normal = cp_geom.local_normal;
    con.normal_attachment = cp_geom.normal_attachment;
    con.distance = cp_geom.distance;

    con.friction = cp_mat.friction;
    con.restitution = use_restitution_solver ? scalar{0} : cp_mat.restitution;

    con.applied_normal_impulse = cp_imp.normal_impulse;
    con.applied_friction_impulse = cp_imp.friction_impulse;
}

void copy_contact_point_to_constraint_extras(contact_extras_constraint &con,
                                             const contact_point &cp,
                                             const contact_point_geometry &cp_geom,
                                             const contact_point_material &cp_mat,
                                             const contact_point_impulse &cp_imp,
                                             const contact_point_spin_friction_impulse &spin_imp,
                                             const contact_point_roll_friction_impulse &roll_imp,
                                             const roll_direction &roll_dirA,
                                             const roll_direction &roll_dirB,
                                             unsigned num_points,
                                             bool use_restitution_solver) {
    copy_contact_point_to_constraint(con, cp, cp_geom, cp_mat, cp_imp, use_restitution_solver);

    con.num_points = num_points;

    con.stiffness = cp_mat.stiffness;
    con.damping = cp_mat.damping;

    con.spin_friction = cp_mat.spin_friction;
    con.roll_friction = cp_mat.roll_friction;
    con.roll_dir = {roll_dirA, roll_dirB};

    con.spin_friction_impulse = spin_imp.spin_friction_impulse;
    con.rolling_friction_impulse = roll_imp.rolling_friction_impulse;
}

static constexpr auto transfer_contact_max_sequential_size = 128u;

template<typename ConstraintType>
void transfer_contact_points_to_constraints(entt::registry &registry, bool mt) {
    auto contact_view = registry.view<ConstraintType,
                                      const contact_point,
                                      const contact_point_geometry,
                                      const contact_point_material,
                                      const contact_point_impulse,
                                      const contact_point_list>(exclude_sleeping_disabled);
    auto spin_impulse_view = registry.view<contact_point_spin_friction_impulse>();
    auto roll_impulse_view = registry.view<contact_point_roll_friction_impulse>();
    auto roll_dir_view = registry.view<roll_direction>();
    auto manifold_view = registry.view<const contact_manifold>();
    auto manifold_state_view = registry.view<const contact_manifold_state>();

    // Do not use the traditional restitution path if the restitution solver
    // is being used. Assign zero restitution to constraint.
    auto &settings = registry.ctx().get<edyn::settings>();
    const bool use_restitution_solver = settings.num_restitution_iterations > 0;

    const auto for_loop_body = [&](entt::entity entity) {
        auto [con, cp, cp_geom, cp_mat, cp_imp, cp_list] = contact_view.get(entity);

        if constexpr(std::is_same_v<ConstraintType, contact_constraint>) {
            copy_contact_point_to_constraint(con, cp, cp_geom, cp_mat, cp_imp, use_restitution_solver);
        } else if constexpr(std::is_same_v<ConstraintType, contact_extras_constraint>) {
            auto spin_imp = contact_point_spin_friction_impulse{};
            auto roll_imp = contact_point_roll_friction_impulse{};
            roll_direction roll_dirs[] = {{vector3_zero}, {vector3_zero}};
            auto [manifold] = manifold_view.get(cp_list.parent);

            if (spin_impulse_view.contains(entity)) {
                spin_imp = std::get<0>(spin_impulse_view.get(entity));
            }

            if (roll_impulse_view.contains(entity)) {
                roll_imp = std::get<0>(roll_impulse_view.get(entity));
            }

            for (int i = 0; i < 2; ++i) {
                if (roll_dir_view.contains(manifold.body[i])) {
                    roll_dirs[i] = std::get<0>(roll_dir_view.get(manifold.body[i]));
                }
            }

            auto [manifold_state] = manifold_state_view.get(cp_list.parent);

            copy_contact_point_to_constraint_extras(con, cp, cp_geom, cp_mat, cp_imp,
                                                    spin_imp, roll_imp, roll_dirs[0], roll_dirs[1],
                                                    manifold_state.num_points, use_restitution_solver);
        }
    };

    auto num_constraints = calculate_view_size(registry.view<ConstraintType>(exclude_sleeping_disabled));

    if (mt && num_constraints > transfer_contact_max_sequential_size) {
        auto task_func = make_view_for_each_task_func(contact_view, for_loop_body);
        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(registry, task, static_cast<unsigned>(num_constraints));
    } else {
        for (auto entity : contact_view) {
            for_loop_body(entity);
        }
    }
}

void copy_contact_constraint_to_point(const contact_constraint &con,
                                      contact_point &cp,
                                      contact_point_geometry &cp_geom,
                                      contact_point_impulse &cp_imp) {
    cp.normal = con.normal;
    cp_geom.distance = con.distance;
    cp_imp.normal_impulse = con.applied_normal_impulse;
    cp_imp.friction_impulse = con.applied_friction_impulse;
}

void copy_contact_constraint_extras_to_point(const contact_extras_constraint &con,
                                             contact_point &cp,
                                             contact_point_geometry &cp_geom,
                                             contact_point_impulse &cp_imp,
                                             contact_point_spin_friction_impulse &spin_imp,
                                             contact_point_roll_friction_impulse &roll_imp) {
    copy_contact_constraint_to_point(con, cp, cp_geom, cp_imp);
    spin_imp.spin_friction_impulse = con.spin_friction_impulse;
    roll_imp.rolling_friction_impulse = con.rolling_friction_impulse;
}

template<typename ConstraintType>
void transfer_contact_constraints_to_points(entt::registry &registry, bool mt) {
    auto contact_view = registry.view<const ConstraintType,
                                      contact_point,
                                      contact_point_geometry,
                                      contact_point_impulse>(exclude_sleeping_disabled);
    auto spin_impulse_view = registry.view<contact_point_spin_friction_impulse>();
    auto roll_impulse_view = registry.view<contact_point_roll_friction_impulse>();

    const auto for_loop_body = [&](entt::entity entity) {
        auto [con, cp, cp_geom, cp_imp] = contact_view.get(entity);
        if constexpr(std::is_same_v<ConstraintType, contact_constraint>) {
            copy_contact_constraint_to_point(con, cp, cp_geom, cp_imp);
        } else if constexpr(std::is_same_v<ConstraintType, contact_extras_constraint>) {
            auto spin_imp_dummy = contact_point_spin_friction_impulse{};
            auto roll_imp_dummy = contact_point_roll_friction_impulse{};
            auto *spin_imp = spin_impulse_view.contains(entity) ? &std::get<0>(spin_impulse_view.get(entity)) : &spin_imp_dummy;
            auto *roll_imp = roll_impulse_view.contains(entity) ? &std::get<0>(roll_impulse_view.get(entity)) : &roll_imp_dummy;
            copy_contact_constraint_extras_to_point(con, cp, cp_geom, cp_imp, *spin_imp, *roll_imp);
        }
    };

    auto num_constraints = calculate_view_size(registry.view<ConstraintType>(exclude_sleeping_disabled));

    if (mt && num_constraints > transfer_contact_max_sequential_size) {
        auto task_func = make_view_for_each_task_func(contact_view, for_loop_body);
        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        enqueue_task_wait(registry, task, static_cast<unsigned>(num_constraints));
    } else {
        for (auto entity : contact_view) {
            for_loop_body(entity);
        }
    }
}

void solver::update(bool mt) {
    auto &registry = *m_registry;
    auto &settings = registry.ctx().get<edyn::settings>();
    auto dt = settings.fixed_dt;

#ifndef EDYN_DISABLE_PROFILING
    auto &profile = registry.ctx().get<profile_timers>();
#endif
    EDYN_PROFILE_BEGIN(prof_time);

    solve_restitution(registry, dt);
    EDYN_PROFILE_MEASURE_ACCUM(prof_time, profile, restitution);

    apply_gravity(registry, dt);

    transfer_contact_points_to_constraints<contact_constraint>(registry, mt);
    transfer_contact_points_to_constraints<contact_extras_constraint>(registry, mt);

    prepare_constraints(registry, dt, mt);
    EDYN_PROFILE_MEASURE_ACCUM(prof_time, profile, prepare_constraints);

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

    transfer_contact_constraints_to_points<contact_constraint>(registry, mt);
    transfer_contact_constraints_to_points<contact_extras_constraint>(registry, mt);

#ifndef EDYN_DISABLE_PROFILING
    auto &counters = registry.ctx().get<profile_counters>();
    counters.bodies = registry.view<rigidbody_tag>().size();
    counters.islands = registry.view<island>().size();
    counters.constraints = 0;
    counters.constraint_rows = 0;

    auto con_view_tuple = get_tuple_of_views(registry, constraints_tuple);
    std::apply([&](auto &&... con_view) {
        counters.constraints = (con_view.size() + ...);
    }, con_view_tuple);

    for (auto island_entity : island_view) {
        auto &cache = registry.get<row_cache>(island_entity);
        counters.constraint_rows += cache.rows.size();
    }
#endif

    EDYN_PROFILE_MEASURE_ACCUM(prof_time, profile, solve_islands);

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

    EDYN_PROFILE_MEASURE_ACCUM(prof_time, profile, apply_results);
}

}
