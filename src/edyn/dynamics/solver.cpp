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
#include <entt/entt.hpp>

namespace edyn {

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

static
void prepare(constraint_row &row, con_row_iter_data &data,
             const vector3 &linvelA, const vector3 &linvelB,
             const vector3 &angvelA, const vector3 &angvelB) {
    auto J_invM_JT = dot(data.J[0], data.J[0]) * data.inv_mA +
                     dot(data.inv_IA * data.J[1], data.J[1]) +
                     dot(data.J[2], data.J[2]) * data.inv_mB +
                     dot(data.inv_IB * data.J[3], data.J[3]);
    data.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(data.J[0], linvelA) + 
                  dot(data.J[1], angvelA) +
                  dot(data.J[2], linvelB) +
                  dot(data.J[3], angvelB);
    
    auto restitution = restitution_curve(row.restitution, relvel);
    data.rhs = -(row.error * row.erp + relvel * (1 + restitution));
}

static
void apply_impulse(scalar impulse, con_row_iter_data &data) {
    // Apply linear impulse.
    *data.dvA += data.inv_mA * data.J[0] * impulse;
    *data.dvB += data.inv_mB * data.J[2] * impulse;

    // Apply angular impulse.
    *data.dwA += data.inv_IA * data.J[1] * impulse;
    *data.dwB += data.inv_IB * data.J[3] * impulse;
}

static
void warm_start(con_row_iter_data &data) {
    apply_impulse(data.impulse, data);
}

static
scalar solve(con_row_iter_data &row) {
    auto delta_relvel = dot(row.J[0], *row.dvA) + 
                        dot(row.J[1], *row.dwA) +
                        dot(row.J[2], *row.dvB) +
                        dot(row.J[3], *row.dwB);
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
    , m_constraints_changed(false)
{
    registry.on_construct<linvel>().connect<&entt::registry::emplace<delta_linvel>>();
    registry.on_construct<angvel>().connect<&entt::registry::emplace<delta_angvel>>();
    registry.on_construct<constraint_row>().connect<&solver::on_construct_constraint_row>(*this);
    registry.on_destroy<constraint_row>().connect<&solver::on_destroy_constraint_row>(*this);
}

void solver::on_construct_constraint_row(entt::registry &, entt::entity) {
    m_constraints_changed = true;
}

void solver::on_destroy_constraint_row(entt::registry &, entt::entity) {
    m_constraints_changed = true;
}

void solver::update(scalar dt) {
    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    if (m_constraints_changed) {
        m_registry->sort<constraint_row>([] (const auto &lhs, const auto &rhs) {
            return lhs.priority > rhs.priority;
        });
        m_registry->sort<con_row_iter_data, constraint_row>();
        m_constraints_changed = false;
    }

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, con_row_iter_data>(entt::exclude<disabled_tag>);
    auto iter_data_view = m_registry->view<con_row_iter_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    row_view.each([&] (constraint_row &row, con_row_iter_data &iter_data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        iter_data.inv_mA = inv_mA;
        iter_data.inv_mB = inv_mB;
        iter_data.inv_IA = inv_IA;
        iter_data.inv_IB = inv_IB;

        iter_data.dvA = &dvA;
        iter_data.dvB = &dvB;
        iter_data.dwA = &dwA;
        iter_data.dwB = &dwB;

        prepare(row, iter_data, linvelA, linvelB, angvelA, angvelB);
        warm_start(iter_data);
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
        iter_data_view.each([&] (con_row_iter_data &data) {
            auto delta_impulse = solve(data);
            apply_impulse(delta_impulse, data);
        });
    }

    // Apply constraint velocity correction.
    auto vel_view = m_registry->view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude<disabled_tag>);
    vel_view.each([] (linvel &v, angvel &w, delta_linvel &dv, delta_angvel &dw) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*m_registry, dt);
    integrate_angvel(*m_registry, dt);
    update_aabbs(*m_registry);
    
    // Update world-space moment of inertia.
    update_inertia(*m_registry);
}

}