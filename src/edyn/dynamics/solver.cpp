#include "edyn/dynamics/solver.hpp"
#include "edyn/sys/integrate_linvel.hpp"
#include "edyn/sys/integrate_linacc.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_angvel.hpp"

namespace edyn {

solver::solver(entt::registry &reg) 
    : registry(&reg)
{
    connections.push_back(reg.on_construct<linvel>().connect<&entt::registry::assign<delta_linvel>>(reg));
    connections.push_back(reg.on_destroy<linvel>().connect<&entt::registry::remove<delta_linvel>>(reg));

    connections.push_back(reg.on_construct<angvel>().connect<&entt::registry::assign<delta_angvel>>(reg));
    connections.push_back(reg.on_destroy<angvel>().connect<&entt::registry::remove<delta_angvel>>(reg));
}

void prepare(constraint_row &row, scalar inv_mA, scalar inv_mB, const vector3 &linvelA, const vector3 &linvelB) {
    auto eff_mass = dot(row.J[0], row.J[0]) * inv_mA +
                    //dot(inv_IA * row.J[1], row.J[1]) +
                    dot(row.J[2], row.J[2]) * inv_mB;// +
                    //dot(inv_IB * row.J[3], row.J[3]);
    row.eff_mass_inv = 1 / eff_mass;

    auto rel_vel = dot(row.J[0], linvelA) + 
                   //dot(row.J[1], angvelA) +
                   dot(row.J[2], linvelB);// +
                   //dot(row.J[3], angvelB);
    row.rhs = row.error -rel_vel;
    row.impulse = 0;
}

void solver::update(scalar dt) {
    integrate_linacc(*registry, dt);

    registry->view<constraint>().each([&] (auto, auto &con) {
        scalar inv_mA = 0;
        scalar inv_mB = 0;
        auto linvelA = vector3_zero;
        auto linvelB = vector3_zero;

        if (auto mA = registry->try_get<mass>(con.entity[0])) inv_mA = 1 / *mA;
        if (auto mB = registry->try_get<mass>(con.entity[1])) inv_mB = 1 / *mB;
        if (auto vel = registry->try_get<linvel>(con.entity[0])) linvelA = *vel; 
        if (auto vel = registry->try_get<linvel>(con.entity[1])) linvelB = *vel;

        std::visit([&] (auto&& c) {
            c.prepare(&con, *registry, dt);

            for (size_t i = 0; i < std::decay_t<decltype(c)>::num_rows; ++i) {
                auto &row = registry->get<constraint_row>(con.row[i]);
                prepare(row, inv_mA, inv_mB, linvelA, linvelB);
            }
        }, con.var);
    });

    auto row_view = registry->view<constraint_row>();

    for (uint32_t i = 0; i < iterations; ++i) {
        row_view.each([&] (auto, auto &row) {
            solve(row);
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
}

void solver::solve(constraint_row &row) {
    auto &dvA = registry->get<delta_linvel>(row.entity[0]);
    auto &dwA = registry->get<delta_angvel>(row.entity[0]);
    auto &dvB = registry->get<delta_linvel>(row.entity[1]);
    auto &dwB = registry->get<delta_angvel>(row.entity[1]);

    auto deltaA = dot(row.J[0], dvA) + dot(row.J[2], dwA);
    auto deltaB = dot(row.J[1], dvB) + dot(row.J[3], dwB);
    auto delta_impulse = (row.rhs - deltaA - deltaB) * row.eff_mass_inv;
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

    scalar inv_mA = 0;
    scalar inv_mB = 0;

    if (auto mA = registry->try_get<mass>(row.entity[0])) {
        inv_mA = 1 / *mA;
    }

    if (auto mB = registry->try_get<mass>(row.entity[1])) {
        inv_mB = 1 / *mB;
    }

    dvA += row.J[0] * inv_mA * delta_impulse;
    dvB += row.J[2] * inv_mB * delta_impulse;
}

}