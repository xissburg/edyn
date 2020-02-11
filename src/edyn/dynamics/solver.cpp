#include <entt/entity/view.hpp>
#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_angvel.hpp"
#include "edyn/sys/integrate_spin.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/dynamics/solver_stage.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/util/array.hpp"
#include "edyn/util/rigidbody.hpp"

namespace edyn {

static 
void on_construct_constraint(entt::entity entity, entt::registry &registry, constraint &con) {
    auto &rel = registry.get<relation>(entity);

    std::visit([&] (auto &&c) {
        // Initialize actual constraint.
        c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, rel, registry, 0);
    }, con.var);
}

static 
void on_destroy_constraint(entt::entity entity, entt::registry &registry) {
    auto &con = registry.get<constraint>(entity);

    // Destroy all constraint rows.
    for (size_t i = 0; i < con.num_rows; ++i) {
        registry.destroy(con.row[i]);
    }
}

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
    row.rhs = -(row.error + relvel * (1 + restitution));
}

static
void prepare3(constraint_row &row, 
              scalar inv_mA, scalar inv_mB, scalar inv_mC, 
              const matrix3x3 &inv_IA, const matrix3x3 &inv_IB, const matrix3x3 &inv_IC,
              const vector3 &linvelA, const vector3 &linvelB, const vector3 &linvelC,
              const vector3 &angvelA, const vector3 &angvelB, const vector3 &angvelC) {
    auto J_invM_JT = dot(row.J[0], row.J[0]) * inv_mA +
                     dot(inv_IA * row.J[1], row.J[1]) +
                     dot(row.J[2], row.J[2]) * inv_mB +
                     dot(inv_IB * row.J[3], row.J[3]) +
                     dot(row.J[4], row.J[4]) * inv_mC +
                     dot(inv_IC * row.J[5], row.J[5]);
    row.eff_mass = 1 / J_invM_JT;

    auto relvel = dot(row.J[0], linvelA) + 
                  dot(row.J[1], angvelA) +
                  dot(row.J[2], linvelB) +
                  dot(row.J[3], angvelB) +
                  dot(row.J[4], linvelC) +
                  dot(row.J[5], angvelC);
    row.relvel = relvel;
    
    auto restitution = restitution_curve(row.restitution, row.relvel);
    row.rhs = -(row.error + relvel * (1 + restitution));
}

static
void apply_angular_impulse(scalar impulse,
                           entt::registry &registry,
                           constraint_row &row,
                           size_t ent_idx,
                           const matrix3x3 &inv_I,
                           delta_angvel &dw,
                           delta_spin *ds) {
    auto idx_J = ent_idx * 2 + 1;

    if (ds) {
        // Split impulse in a spin component and an angular component.
        auto imp = inv_I * row.J[idx_J] * impulse;
        auto orn = registry.get<orientation>(row.entity[ent_idx]);
        auto axis = rotate(orn, vector3_x);
        auto spin_imp = dot(axis, imp); 
        *ds += spin_imp;
        // Subtract spin impulse to obtain angular impulse.
        dw += imp - axis * spin_imp;
    } else {
        dw += inv_I * row.J[idx_J] * impulse;
    }
}

static
void apply_impulse(scalar impulse,
                   entt::registry &registry,
                   constraint_row &row,
                   scalar inv_mA, scalar inv_mB,
                   const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
                   delta_linvel &dvA, delta_linvel &dvB,
                   delta_angvel &dwA, delta_angvel &dwB,
                   delta_spin *dsA, delta_spin *dsB) {
    // Apply linear impulse.
    dvA += inv_mA * row.J[0] * impulse;
    dvB += inv_mB * row.J[2] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, registry, row, 0, inv_IA, dwA, dsA);
    apply_angular_impulse(impulse, registry, row, 1, inv_IB, dwB, dsB);
}

static
void apply_impulse3(scalar impulse,
                    entt::registry &registry,
                    constraint_row &row,
                    scalar inv_mA, scalar inv_mB, scalar inv_mC,
                    const matrix3x3 &inv_IA, const matrix3x3 &inv_IB, const matrix3x3 &inv_IC,
                    delta_linvel &dvA, delta_linvel &dvB, delta_linvel &dvC,
                    delta_angvel &dwA, delta_angvel &dwB, delta_angvel &dwC,
                    delta_spin *dsA, delta_spin *dsB, delta_spin *dsC) {
    // Apply linear impulse.
    dvA += inv_mA * row.J[0] * impulse;
    dvB += inv_mB * row.J[2] * impulse;
    dvC += inv_mC * row.J[4] * impulse;

    // Apply angular impulse.
    apply_angular_impulse(impulse, registry, row, 0, inv_IA, dwA, dsA);
    apply_angular_impulse(impulse, registry, row, 1, inv_IB, dwB, dsB);
    apply_angular_impulse(impulse, registry, row, 2, inv_IC, dwC, dsC);
}

static
void warm_start(entt::registry &registry, constraint_row &row, 
                scalar inv_mA, scalar inv_mB,
                const matrix3x3 &inv_IA, const matrix3x3 &inv_IB,
                delta_linvel &dvA, delta_linvel &dvB,
                delta_angvel &dwA, delta_angvel &dwB,
                delta_spin *dsA, delta_spin *dsB) {    
    // Do not warm start when there's restitution since this constraint isn't 
    // going to rest and also to prevent adding energy to the system.
    if (restitution_curve(row.restitution, row.relvel) > 0) {
        return;
    }

    apply_impulse(row.impulse, registry, row, 
                  inv_mA, inv_mB, 
                  inv_IA, inv_IB, 
                  dvA, dvB, 
                  dwA, dwB, 
                  dsA, dsB);
}

static
void warm_start3(entt::registry &registry, constraint_row &row, 
                 scalar inv_mA, scalar inv_mB, scalar inv_mC,
                 const matrix3x3 &inv_IA, const matrix3x3 &inv_IB, const matrix3x3 &inv_IC,
                 delta_linvel &dvA, delta_linvel &dvB, delta_linvel &dvC,
                 delta_angvel &dwA, delta_angvel &dwB, delta_angvel &dwC,
                 delta_spin *dsA, delta_spin *dsB, delta_spin *dsC) {    
    // Do not warm start when there's restitution since this constraint isn't 
    // going to rest and also to prevent adding energy to the system.
    if (restitution_curve(row.restitution, row.relvel) > 0) {
        return;
    }

    apply_impulse3(row.impulse, registry, row, 
                   inv_mA, inv_mB, inv_mC, 
                   inv_IA, inv_IB, inv_IC, 
                   dvA, dvB, dvC, 
                   dwA, dwB, dwC, 
                   dsA, dsB, dsC);
}

static
scalar solve(constraint_row &row,
             const vector3 &dvA, const vector3 &dvB,
             const vector3 &dwA, const vector3 &dwB) {
    auto delta_relvel = dot(row.J[0], dvA) + 
                        dot(row.J[1], dwA) +
                        dot(row.J[2], dvB) +
                        dot(row.J[3], dwB);
    auto restitution = restitution_curve(row.restitution, row.relvel + delta_relvel);
    auto delta_impulse = (row.rhs - delta_relvel * (1 + restitution)) * row.eff_mass;

    // Clamp `delta_impulse` for proper shock propagation when there's restitution.
    // This prevents contact constraints from 'sucking' and consequently 
    // eliminating the restitution effect.
    if (row.restitution > 0) {
        delta_impulse = std::clamp(delta_impulse, row.lower_limit, row.upper_limit);
    }

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

static
scalar solve3(constraint_row &row,
              const vector3 &dvA, const vector3 &dvB, const vector3 &dvC,
              const vector3 &dwA, const vector3 &dwB, const vector3 &dwC) {
    auto delta_relvel = dot(row.J[0], dvA) + 
                        dot(row.J[1], dwA) +
                        dot(row.J[2], dvB) +
                        dot(row.J[3], dwB) +
                        dot(row.J[4], dvC) +
                        dot(row.J[5], dwC);
    auto restitution = restitution_curve(row.restitution, row.relvel + delta_relvel);
    auto delta_impulse = (row.rhs - delta_relvel * (1 + restitution)) * row.eff_mass;

    // Clamp `delta_impulse` for proper shock propagation when there's restitution.
    // This prevents contact constraints from 'sucking' and consequently 
    // eliminating the restitution effect.
    if (row.restitution > 0) {
        delta_impulse = std::clamp(delta_impulse, row.lower_limit, row.upper_limit);
    }

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
    auto view = registry.view<dynamic_tag, const orientation, const inertia_inv, inertia_world_inv>(exclude_sleeping);
    view.each([] (auto, auto, const orientation& orn, const inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        auto basis = to_matrix3x3(orn);
        inv_IW = scale(basis, inv_I) * transpose(basis);
    });
}

solver::solver(entt::registry &reg) 
    : registry(&reg)
{
    connections.push_back(reg.on_construct<constraint>().connect<&on_construct_constraint>());
    connections.push_back(reg.on_destroy<constraint>().connect<&on_destroy_constraint>());

    connections.push_back(reg.on_construct<relation>().connect<&island_on_construct_relation>());
    connections.push_back(reg.on_destroy<relation>().connect<&island_on_destroy_relation>());

    connections.push_back(reg.on_construct<linvel>().connect<&entt::registry::assign<delta_linvel>>(reg));
    connections.push_back(reg.on_destroy<linvel>().
        connect<entt::overload<void(entt::entity)>(&entt::registry::reset<delta_linvel>)>(reg));

    connections.push_back(reg.on_construct<angvel>().connect<&entt::registry::assign<delta_angvel>>(reg));
    connections.push_back(reg.on_destroy<angvel>().
        connect<entt::overload<void(entt::entity)>(&entt::registry::reset<delta_angvel>)>(reg));

    connections.push_back(reg.on_construct<spin>().connect<&entt::registry::assign<delta_spin>>(reg));
    connections.push_back(reg.on_destroy<spin>().
        connect<entt::overload<void(entt::entity)>(&entt::registry::reset<delta_spin>)>(reg));
}

void solver::update(uint64_t step, scalar dt) {
    // Apply forces and acceleration.
    integrate_linacc(*registry, dt);
    apply_gravity(*registry, dt);

    // Setup constraints.
    auto mass_inv_view = registry->view<const mass_inv, const inertia_world_inv>(exclude_sleeping);
    auto vel_view = registry->view<const linvel, const angvel>(exclude_sleeping);
    auto delta_view = registry->view<delta_linvel, delta_angvel>(exclude_sleeping);

    auto con_view = registry->view<const relation, constraint>(exclude_sleeping);
    con_view.each([&] (auto entity, const relation &rel, constraint &con) {
        std::visit([&] (auto &&c) {
            c.update(solver_stage_value_t<solver_stage::prepare>{}, entity, con, rel, *registry, dt);
        }, con.var);
    });

    registry->sort<constraint_row>([] (const auto &lhs, const auto &rhs) {
        return lhs.priority > rhs.priority;
    });

    con_view.each([&] (auto entity, const relation &rel, constraint &con) {
        auto [inv_mA, inv_IA] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(rel.entity[0]);
        auto [linvelA, angvelA] = vel_view.get<const linvel, const angvel>(rel.entity[0]);
        auto [dvA, dwA] = delta_view.get<delta_linvel, delta_angvel>(rel.entity[0]);

        auto [inv_mB, inv_IB] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(rel.entity[1]);
        auto [linvelB, angvelB] = vel_view.get<const linvel, const angvel>(rel.entity[1]);
        auto [dvB, dwB] = delta_view.get<delta_linvel, delta_angvel>(rel.entity[1]);

        auto spinvelA = vector3_zero;
        auto spinvelB = vector3_zero;

        if (auto s = registry->try_get<const spin>(rel.entity[0])) {
            auto &orn = registry->get<const orientation>(rel.entity[0]);
            auto axis = rotate(orn, vector3_x);
            spinvelA = axis * *s;
        }

        if (auto s = registry->try_get<const spin>(rel.entity[1])) {
            auto &orn = registry->get<const orientation>(rel.entity[1]);
            auto axis = rotate(orn, vector3_x);
            spinvelB = axis * *s;
        }

        auto dsA = registry->try_get<const delta_spin>(rel.entity[0]);
        auto dsB = registry->try_get<const delta_spin>(rel.entity[1]);

        if (rel.entity[2] == entt::null) {
            for (size_t i = 0; i < con.num_rows; ++i) {
                auto &row = registry->get<constraint_row>(con.row[i]);
                EDYN_ASSERT(row.entity[0] != entt::null && row.entity[1] != entt::null);
                prepare(row, 
                        inv_mA, inv_mB, 
                        inv_IA, inv_IB, 
                        linvelA, linvelB, 
                        row.use_spin[0] ? angvelA + spinvelA : static_cast<vector3>(angvelA), 
                        row.use_spin[1] ? angvelB + spinvelB : static_cast<vector3>(angvelB));
                warm_start(*registry, row, 
                           inv_mA, inv_mB, 
                           inv_IA, inv_IB, 
                           dvA, dvB, dwA, dwB, 
                           row.use_spin[0] ? dsA : nullptr, 
                           row.use_spin[1] ? dsB : nullptr);
            }
        } else {
            auto [inv_mC, inv_IC] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(rel.entity[2]);
            auto [linvelC, angvelC] = vel_view.get<const linvel, const angvel>(rel.entity[2]);
            auto [dvC, dwC] = delta_view.get<delta_linvel, delta_angvel>(rel.entity[2]);
            auto spinvelC = vector3_zero;

            if (auto s = registry->try_get<const spin>(rel.entity[2])) {
                auto &orn = registry->get<const orientation>(rel.entity[2]);
                auto axis = rotate(orn, vector3_x);
                spinvelC = axis * *s;
            }

            auto dsC = registry->try_get<const delta_spin>(rel.entity[2]);

            for (size_t i = 0; i < con.num_rows; ++i) {
                auto &row = registry->get<constraint_row>(con.row[i]);
                EDYN_ASSERT(row.entity[0] != entt::null && row.entity[1] != entt::null);
                prepare3(row, 
                         inv_mA, inv_mB, inv_mC, 
                         inv_IA, inv_IB, inv_IC, 
                         linvelA, linvelB, linvelC, 
                         row.use_spin[0] ? angvelA + spinvelA : static_cast<vector3>(angvelA), 
                         row.use_spin[1] ? angvelB + spinvelB : static_cast<vector3>(angvelB),
                         row.use_spin[2] ? angvelC + spinvelC : static_cast<vector3>(angvelC));
                warm_start3(*registry, row, 
                            inv_mA, inv_mB, inv_mC,
                            inv_IA, inv_IB, inv_IC,
                            dvA, dvB, dvC,
                            dwA, dwB, dwC, 
                            row.use_spin[0] ? dsA : nullptr, 
                            row.use_spin[1] ? dsB : nullptr, 
                            row.use_spin[2] ? dsC : nullptr);
            }
        }
    });

    // Solve constraints.
    auto row_view = registry->view<constraint_row>(exclude_sleeping);

    for (uint32_t i = 0; i < iterations; ++i) {
        // Prepare constraints for iteration.
        con_view.each([&] (auto entity, const relation &rel, constraint &con) {
            std::visit([&] (auto &&c) {
                c.update(solver_stage_value_t<solver_stage::iteration>{}, entity, con, rel, *registry, dt);
            }, con.var);
        });

        // Solve rows.
        row_view.each([&] (auto, auto &row) {
            auto [inv_mA, inv_IA] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(row.entity[0]);
            auto [dvA, dwA] = delta_view.get<delta_linvel, delta_angvel>(row.entity[0]);

            auto [inv_mB, inv_IB] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(row.entity[1]);
            auto [dvB, dwB] = delta_view.get<delta_linvel, delta_angvel>(row.entity[1]);

            // Calculate delta spin to combine with delta angular velocity.
            auto dsA = registry->try_get<delta_spin>(row.entity[0]);
            auto dsB = registry->try_get<delta_spin>(row.entity[1]);

            auto vdsA = vector3_zero;
            auto vdsB = vector3_zero;

            if (row.use_spin[0] && dsA) {
                auto orn = registry->get<orientation>(row.entity[0]);
                vdsA = rotate(orn, vector3_x) * *dsA;
            }

            if (row.use_spin[1] && dsB) {
                auto orn = registry->get<orientation>(row.entity[1]);
                vdsB = rotate(orn, vector3_x) * *dsB;
            }

            if (row.entity[2] == entt::null) {
                auto delta_impulse = solve(row, dvA, dvB, dwA + vdsA, dwB + vdsB);
                apply_impulse(delta_impulse, *registry, row, 
                              inv_mA, inv_mB, inv_IA, inv_IB, 
                              dvA, dvB, dwA, dwB, 
                              row.use_spin[0] ? dsA : nullptr, 
                              row.use_spin[1] ? dsB : nullptr);
            } else {
                auto [inv_mC, inv_IC] = mass_inv_view.get<const mass_inv, const inertia_world_inv>(row.entity[2]);
                auto [dvC, dwC] = delta_view.get<delta_linvel, delta_angvel>(row.entity[2]);

                auto dsC = registry->try_get<delta_spin>(row.entity[2]);
                auto vdsC = vector3_zero;

                if (row.use_spin[2] && dsC) {
                    auto orn = registry->get<orientation>(row.entity[2]);
                    vdsC = rotate(orn, vector3_x) * *dsC;
                }

                auto delta_impulse = solve3(row, 
                                            dvA, dvB, dvC,
                                            dwA + vdsA, dwB + vdsB, dwC + vdsC);
                apply_impulse3(delta_impulse, *registry, row, 
                               inv_mA, inv_mB, inv_mC,
                               inv_IA, inv_IB, inv_IC,
                               dvA, dvB, dvC,
                               dwA, dwB, dwC, 
                               row.use_spin[0] ? dsA : nullptr, 
                               row.use_spin[1] ? dsB : nullptr,
                               row.use_spin[2] ? dsC : nullptr);
            }
        });
    }

    // Apply constraint velocity correction.
    auto linvel_view = registry->view<dynamic_tag, linvel, delta_linvel>(exclude_sleeping);
    linvel_view.each([] (auto, auto, linvel &vel, delta_linvel &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    auto angvel_view = registry->view<dynamic_tag, angvel, delta_angvel>(exclude_sleeping);
    angvel_view.each([] (auto, auto, angvel &vel, delta_angvel &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    auto spin_view = registry->view<dynamic_tag, spin, delta_spin>(exclude_sleeping);
    spin_view.each([] (auto, auto, spin &s, delta_spin &delta) {
        s += delta;
        delta.s = 0;
    });

    // Integrate velocities to obtain new transforms.
    integrate_linvel(*registry, dt);
    integrate_angvel(*registry, dt);
    integrate_spin(*registry, dt);

    // Update world-space moment of inertia.
    update_inertia(*registry);

    put_islands_to_sleep(*registry, step, dt);

    clear_kinematic_velocities(*registry);
}

}