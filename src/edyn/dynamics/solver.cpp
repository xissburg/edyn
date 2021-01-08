#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/dynamics/solver_stage.hpp"
#include "edyn/util/array.hpp"
#include "edyn/util/rigidbody.hpp"
#include "edyn/comp/con_row_iter_data.hpp"
#include "edyn/comp/edge_color.hpp"
#include <entt/entt.hpp>

namespace edyn {

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

static
void prepare(constraint_row &row, 
             scalar inv_mA, scalar inv_mB, 
             const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
             const vector3 &linvelA, const vector3 &linvelB,
             const vector3 &angvelA, const vector3 &angvelB) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * inv_mA +
                     dot(inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * inv_mB +
                     dot(inv_IB * row.J[3], row.J[3]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) + 
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB);
    row.relvel = relvel;
    
    auto restitution = restitution_curve(row.restitution, row.relvel);
    row.rhs = -(row.error * row.erp + relvel * (1 + restitution));
}

static
void apply_impulse(scalar impulse,
                   constraint_row &row,
                   scalar inv_mA, scalar inv_mB,
                   const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
                   delta_linvel &dvA, delta_linvel &dvB,
                   delta_angvel &dwA, delta_angvel &dwB) {
    // Apply linear impulse.
    dvA += inv_mA * row.J[0] * impulse;
    dvB += inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    dwA += inv_IA * row.J[1] * impulse;
    dwB += inv_IB * row.J[3] * impulse;
}

static
void warm_start(constraint_row &row, 
                scalar inv_mA, scalar inv_mB,
                const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
                delta_linvel &dvA, delta_linvel &dvB,
                delta_angvel &dwA, delta_angvel &dwB) {
    apply_impulse(row.impulse, row, 
                  inv_mA, inv_mB, 
                  inv_IA, inv_IB, 
                  dvA, dvB, 
                  dwA, dwB);
}

static
scalar solve(constraint_row &row,
             const vector3 &dvA, const vector3 &dvB,
             const vector3 &dwA, const vector3 &dwB) {
    auto delta_relvel = dot(row.J[0], dvA) + 
                        dot(row.J[1], dwA) +
                        dot(row.J[2], dvB) +
                        dot(row.J[3], dwB);
    auto delta_impulse = (row.rhs - delta_relvel) * row.eff_mass;
    auto impulse = row.impulse + delta_impulse;

    if (impulse < row.lower_limit) {
        delta_impulse = row.lower_limit - row.impulse;
        row.impulse = row.lower_limit;
    } else if (impulse > row.upper_limit) {
        delta_impulse = row.upper_limit - row.impulse;
        row.impulse = row.upper_limit;
    } else {
        row.impulse = impulse;
    }

    return delta_impulse;
}

void update_inertia(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>(entt::exclude<disabled_tag>);
    view.each([] (auto, orientation& orn, inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = scale(basis, inv_I) * transpose(basis);
    });
}

solver::solver(entt::registry &registry) 
    : m_registry(&registry)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
    registry.on_construct<constraint_row>().connect<&entt::registry::emplace<con_row_iter_data>>();
    registry.on_construct<constraint_row>().connect<&entt::registry::emplace<edge_color>>();
}

void solver::update(scalar dt) {
    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, con_row_iter_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    m_registry->sort<constraint_row>([] (const auto &lhs, const auto &rhs) {
        return lhs.priority > rhs.priority;
    });

    row_view.each([&] (constraint_row &row, con_row_iter_data &iter_data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        prepare(row, 
                inv_mA, inv_mB, 
                inv_IA, inv_IB, 
                linvelA, linvelB, 
                angvelA, angvelB);
        warm_start(row, 
                   inv_mA, inv_mB, 
                   inv_IA, inv_IB, 
                   dvA, dvB, dwA, dwB);

        iter_data.inv_mA = inv_mA;
        iter_data.inv_mB = inv_mB;
        iter_data.inv_IA = inv_IA;
        iter_data.inv_IB = inv_IB;
        iter_data.dvA = &dvA;
        iter_data.dvB = &dvB;
        iter_data.dwA = &dwA;
        iter_data.dwB = &dwB;
    });

    // Solve constraints.
    for (uint32_t i = 0; i < iterations; ++i) {
        // Prepare constraints for iteration.
        con_view.each([&] (entt::entity entity, constraint &con) {
            std::visit([&] (auto &&c) {
                c.update(solver_stage_value_t<solver_stage::iteration>{}, entity, con, *m_registry, dt);
            }, con.var);
        });

        // Solve rows.
        row_view.each([&] (constraint_row &row, con_row_iter_data &iter_data) {
            auto delta_impulse = solve(row, *iter_data.dvA, *iter_data.dvB, *iter_data.dwA, *iter_data.dwB);
            apply_impulse(delta_impulse, row, 
                          iter_data.inv_mA, iter_data.inv_mB, 
                          iter_data.inv_IA, iter_data.inv_IB, 
                          *iter_data.dvA, *iter_data.dvB, 
                          *iter_data.dwA, *iter_data.dwB);
        });
    }

    // Apply constraint velocity correction.
    auto linvel_view = m_registry->view<linvel, delta_linvel, dynamic_tag>(entt::exclude<disabled_tag>);
    linvel_view.each([] (auto, linvel &vel, delta_linvel &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    auto angvel_view = m_registry->view<angvel, delta_angvel, dynamic_tag>(entt::exclude<disabled_tag>);
    angvel_view.each([] (auto, angvel &vel, delta_angvel &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*m_registry, dt);
    integrate_angvel(*m_registry, dt);
    update_aabbs(*m_registry);
    
    // Update world-space moment of inertia.
    update_inertia(*m_registry);
}

}