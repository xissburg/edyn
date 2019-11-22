#include <entt/entity/view.hpp>
#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/sys/apply_gravity.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

void on_destroy_linvel(entt::entity entity, entt::registry &registry) {
    if (registry.has<delta_linvel>(entity)) {
        registry.remove<delta_linvel>(entity);
    }
}

void on_destroy_angvel(entt::entity entity, entt::registry &registry) {
    if (registry.has<delta_angvel>(entity)) {
        registry.remove<delta_angvel>(entity);
    }
}

solver::solver(entt::registry &reg) 
    : registry(&reg)
{
    connections.push_back(reg.on_construct<linvel>().connect<&entt::registry::assign<delta_linvel>>(reg));
    connections.push_back(reg.on_destroy<linvel>().connect<&on_destroy_linvel>());

    connections.push_back(reg.on_construct<angvel>().connect<&entt::registry::assign<delta_angvel>>(reg));
    connections.push_back(reg.on_destroy<angvel>().connect<&on_destroy_angvel>());
}

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

    auto rel_vel = dot(row.J[0], linvelA) + 
                   dot(row.J[1], angvelA) +
                   dot(row.J[2], linvelB) +
                   dot(row.J[3], angvelB);
    row.rhs = row.error -rel_vel;
    row.impulse = 0;
}

template<typename MassInvView, typename DeltaView>
void solve(constraint_row &row,
           MassInvView &mass_inv_view,
           DeltaView &delta_view) {
    auto [dvA, dwA] = delta_view.template get<delta_linvel, delta_angvel>(row.entity[0]);
    auto [dvB, dwB] = delta_view.template get<delta_linvel, delta_angvel>(row.entity[1]);

    auto deltaA = dot(row.J[0], dvA) + dot(row.J[2], dwA);
    auto deltaB = dot(row.J[1], dvB) + dot(row.J[3], dwB);
    auto delta_impulse = (row.rhs - deltaA - deltaB) * row.eff_mass;
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

    auto [inv_mA, inv_IA] = mass_inv_view.template get<mass_inv, inertia_world_inv>(row.entity[0]);
    auto [inv_mB, inv_IB] = mass_inv_view.template get<mass_inv, inertia_world_inv>(row.entity[1]);

    dvA += inv_mA * row.J[0] * delta_impulse;
    dwA += inv_IA * row.J[1] * delta_impulse;
    dvB += inv_mB * row.J[2] * delta_impulse;
    dwB += inv_IB * row.J[3] * delta_impulse;
}

void update_inertia(entt::registry& registry) {
    auto view = registry.view<const orientation, const inertia_inv, inertia_world_inv>();
    view.each([] (auto, const orientation& orn, const inertia_inv &inv_I, inertia_world_inv &inv_IW) {
        //inv_IW = orn * inv_I * inverse(orn);
    });
}

void solver::update(scalar dt) {
    integrate_linacc(*registry, dt);
    apply_gravity(*registry, dt);

    auto mass_inv_view = registry->view<mass_inv, inertia_world_inv>();
    auto vel_view = registry->view<linvel, angvel>();

    auto con_view = registry->view<const relation, constraint>();
    con_view.each([&] (auto, const relation &rel, constraint &con) {
        auto [inv_mA, inv_IA] = mass_inv_view.get<mass_inv, inertia_world_inv>(rel.entity[0]);
        auto [inv_mB, inv_IB] = mass_inv_view.get<mass_inv, inertia_world_inv>(rel.entity[1]);
        auto [linvelA, angvelA] = vel_view.get<linvel, angvel>(rel.entity[0]);
        auto [linvelB, angvelB] = vel_view.get<linvel, angvel>(rel.entity[1]);

        std::visit([&] (auto &&c) {
            c.prepare(&con, &rel, *registry, dt);

            for (size_t i = 0; i < std::decay_t<decltype(c)>::num_rows; ++i) {
                auto &row = registry->get<constraint_row>(con.row[i]);
                prepare(row, inv_mA, inv_mB, inv_IA, inv_IB, linvelA, linvelB, angvelA, angvelB);
            }
        }, con.var);
    });

    auto row_view = registry->view<constraint_row>();
    auto delta_view = registry->view<delta_linvel, delta_angvel>();

    for (uint32_t i = 0; i < iterations; ++i) {
        row_view.each([&] (auto, auto &row) {
            solve(row, mass_inv_view, delta_view);
        });
    }

    registry->view<linvel, delta_linvel>().each([] (auto, auto &vel, auto &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    registry->view<angvel, delta_angvel>().each([] (auto, auto &vel, auto &delta) {
        vel += delta;
        delta = vector3_zero;
    });

    integrate_linvel(*registry, dt);

    update_inertia(*registry);
}

}