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
#include "edyn/comp/constraint_color.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include "edyn/parallel/job.hpp"
#include <entt/entt.hpp>

namespace edyn {

static
scalar restitution_curve(scalar restitution, scalar relvel) {
    // TODO: figure out how to adjust the restitution when resting.
    scalar decay = 1;//std::clamp(-relvel * 1.52 - scalar(0.12), scalar(0), scalar(1));
    return restitution * decay;
}

static
void prepare(constraint_row &row, constraint_row_data &data,
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
void apply_impulse(scalar impulse, constraint_row_data &data) {
    // Apply linear impulse.
    *data.dvA += data.inv_mA * data.J[0] * impulse;
    *data.dvB += data.inv_mB * data.J[2] * impulse;

    // Apply angular impulse.
    *data.dwA += data.inv_IA * data.J[1] * impulse;
    *data.dwB += data.inv_IB * data.J[3] * impulse;
}

static
void warm_start(constraint_row_data &data) {
    apply_impulse(data.impulse, data);
}

static
scalar solve(constraint_row_data &data) {
    auto delta_relvel = dot(data.J[0], *data.dvA) + 
                        dot(data.J[1], *data.dwA) +
                        dot(data.J[2], *data.dvB) +
                        dot(data.J[3], *data.dwB);
    auto delta_impulse = (data.rhs - delta_relvel) * data.eff_mass;
    auto impulse = data.impulse + delta_impulse;

    if (impulse < data.lower_limit) {
        delta_impulse = data.lower_limit - data.impulse;
        data.impulse = data.lower_limit;
    } else if (impulse > data.upper_limit) {
        delta_impulse = data.upper_limit - data.impulse;
        data.impulse = data.upper_limit;
    } else {
        data.impulse = impulse;
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

void solver::on_construct_constraint_row(entt::registry &registry, entt::entity entity) {
    m_constraints_changed = true;
    m_new_rows.push_back(entity);
}

void solver::on_destroy_constraint_row(entt::registry &, entt::entity) {
    m_constraints_changed = true;
}

void solver::init_new_rows() {
    if (m_new_rows.empty()) return;
    
    auto node_view = m_registry->view<island_node>();
    auto parent_view = m_registry->view<island_node_parent>();
    auto color_view = m_registry->view<constraint_color>();

    for (auto entity : m_new_rows) {
        auto &row = m_registry->get<constraint_row>(entity);

        std::vector<bool> colors;

        for (auto other : row.entity) {
            auto &other_node = node_view.get(other);

            for (auto other_entity : other_node.entities) {
                if (!parent_view.contains(other_entity)) continue;

                auto &grandpa = parent_view.get(other_entity);

                for (auto parent_entity : grandpa.children) {
                    auto &parent = parent_view.get(parent_entity);

                    for (auto row_entity : parent.children) {
                        if (!color_view.contains(row_entity)) continue;

                        auto &color = color_view.get(row_entity);

                        if (color.value >= colors.size()) {
                            colors.resize(color.value + 1, false);
                        }

                        colors[color.value] = true;
                    }
                }
            }
        }

        auto emplaced = false;

        for (size_t i = 0; i < colors.size(); ++i) {
            if (!colors[i]) {
                m_registry->emplace<constraint_color>(entity, i);
                emplaced = true;
                break;
            }
        }

        if (!emplaced) {
            m_registry->emplace<constraint_color>(entity, colors.size());
        }
    }

    m_new_rows.clear();
}

bool solver::parallelizable() const {
    return m_registry->size<constraint_color>() > 1;
}

void solver::update(scalar dt) {
    init_new_rows();

    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, constraint_row_data>(entt::exclude<disabled_tag>);
    auto data_view = m_registry->view<constraint_row_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    row_view.each([&] (constraint_row &row, constraint_row_data &data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        data.inv_mA = inv_mA;
        data.inv_mB = inv_mB;
        data.inv_IA = inv_IA;
        data.inv_IB = inv_IB;

        data.dvA = &dvA;
        data.dvB = &dvB;
        data.dwA = &dwA;
        data.dwB = &dwB;

        prepare(row, data, linvelA, linvelB, angvelA, angvelB);
        warm_start(data);
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
        data_view.each([&] (constraint_row_data &data) {
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

void solver::start_async_update(scalar dt) {
    init_new_rows();

    // Apply forces and acceleration.
    integrate_linacc(*m_registry, dt);
    apply_gravity(*m_registry, dt);

    // If any constraint_row has been created or destroyed between updates:
    // 1. Assign constraint_color::none to all constraint_color values.
    // 2. For each island_node with a procedural_tag, visit its neighbors which
    // hold an constraint_color and if their color is constraint_color::none, assign the 
    // lowest color value that's unique among all neighbors.
    // 3. Sort constraint_color components by value (none of them should be constraint_color::none
    // at this point).
    // 4. Run one parallel_for_async for each range of identical values running a
    // `solve` followed by `apply_impulse` on the corresponding constraint_row.
    // Reapeat for multiple iterations.

    if (m_constraints_changed) {
        m_registry->sort<constraint_color>([] (const auto &lhs, const auto &rhs) {
            return lhs.value < rhs.value;
        });
        m_registry->sort<constraint_row_data, constraint_color>();

        m_constraints_changed = false;
    }

    // Setup constraints.
    auto body_view = m_registry->view<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>();
    auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
    auto row_view = m_registry->view<constraint_row, constraint_row_data>(entt::exclude<disabled_tag>);

    con_view.each([&] (entt::entity entity, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, *m_registry, dt);
        }, con.var);
    });

    row_view.each([&] (constraint_row &row, constraint_row_data &data) {
        auto [inv_mA, inv_IA, linvelA, angvelA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[0]);
        auto [inv_mB, inv_IB, linvelB, angvelB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, linvel, angvel, delta_linvel, delta_angvel>(row.entity[1]);

        data.inv_mA = inv_mA;
        data.inv_mB = inv_mB;
        data.inv_IA = inv_IA;
        data.inv_IB = inv_IB;

        data.dvA = &dvA;
        data.dvB = &dvB;
        data.dwA = &dwA;
        data.dwB = &dwB;

        prepare(row, data, linvelA, linvelB, angvelA, angvelB);
        warm_start(data);
    });

    m_state.dt = dt;
    m_state.color_value = 0;
    m_state.color_index = 0;
    m_state.iteration = 0;
}

bool solver::continue_async_update(const job &completion) {
    auto color_view = m_registry->view<constraint_color>();
    
    if (m_state.color_index == color_view.size()) {
        m_state.color_value = 0;
        m_state.color_index = 0;
        ++m_state.iteration;
    }

    if (m_state.iteration < iterations) {
        size_t first = m_state.color_index;
        size_t last = first;
        
        while (last < color_view.size() && 
               color_view.get(color_view[last]).value == m_state.color_value) {
            ++last;
        }

        if (m_state.color_value == 0) {
            // Prepare constraints for iteration.
            auto con_view = m_registry->view<constraint>(entt::exclude<disabled_tag>);
            con_view.each([&] (entt::entity entity, constraint &con) {
                std::visit([&] (auto &&c) {
                    c.update(solver_stage_value_t<solver_stage::iteration>{}, entity, con, *m_registry, m_state.dt);
                }, con.var);
            });
        }

        ++m_state.color_value;
        m_state.color_index = last;

        auto &dispatcher = job_dispatcher::global();
        auto data_view = m_registry->view<constraint_row_data>();

        parallel_for_async(dispatcher, first, last, size_t{1}, completion, [data_view] (size_t index) {
            // Solve rows.
            auto &data = data_view.get(data_view[index]);
            auto delta_impulse = solve(data);
            apply_impulse(delta_impulse, data);
        });

        return false;
    } else {
        finish_async_update();
        return true;
    }
}

void solver::finish_async_update() {
    // Apply constraint velocity correction.
    auto vel_view = m_registry->view<linvel, angvel, delta_linvel, delta_angvel, dynamic_tag>(entt::exclude<disabled_tag>);
    vel_view.each([] (linvel &v, angvel &w, delta_linvel &dv, delta_angvel &dw) {
        v += dv;
        w += dw;
        dv = vector3_zero;
        dw = vector3_zero;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*m_registry, m_state.dt);
    integrate_angvel(*m_registry, m_state.dt);
    update_aabbs(*m_registry);
    
    // Update world-space moment of inertia.
    update_inertia(*m_registry);

    m_state.color_value = 0;
    m_state.color_index = 0;
    m_state.iteration = 0;
}

}