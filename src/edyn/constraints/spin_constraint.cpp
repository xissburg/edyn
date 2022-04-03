#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<spin_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto spin_view = registry.view<spin, delta_spin>();
    auto con_view = registry.view<spin_constraint>();

    con_view.each([&] (entt::entity entity, spin_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto axisA = rotate(ornA, vector3_x);
        auto axisB = rotate(ornB, vector3_x);

        auto spinvelA = vector3_zero;
        auto spinvelB = vector3_zero;

        if (con.m_use_spinA) {
            auto spinA = spin_view.get<spin>(con.body[0]);
            spinvelA = axisA * spinA;
        }

        if (con.m_use_spinB) {
            auto spinB = spin_view.get<spin>(con.body[1]);
            spinvelB = axisB * spinB;
        }

        auto impulse = con.m_max_torque * dt;

        auto &row = cache.rows.emplace_back();
        row.J = {vector3_zero, axisA, vector3_zero, -axisB};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = con.impulse;
        row.use_spin[0] = con.m_use_spinA;
        row.use_spin[1] = con.m_use_spinB;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;

        if (con.m_use_spinA) {
            auto &dsA = spin_view.get<delta_spin>(con.body[0]);
            row.dsA = &dsA;
        } else {
            row.dsA = nullptr;
        }

        if (con.m_use_spinB) {
            auto &dsB = spin_view.get<delta_spin>(con.body[1]);
            row.dsB = &dsB;
        } else {
            row.dsB = nullptr;
        }

        prepare_row(row, {}, linvelA, angvelA + spinvelA, linvelB, angvelB + spinvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}
