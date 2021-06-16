#include "edyn/constraints/spin_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
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
                                   linvel, angvel, spin,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel, delta_spin>();
    auto con_view = registry.view<spin_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, spin_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, spinA, inv_mA, inv_IA, dvA, dwA, dsA] =
            body_view.get<position, orientation, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, spinB, inv_mB, inv_IB, dvB, dwB, dsB] =
            body_view.get<position, orientation, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[1]);

        auto axisA = rotate(ornA, vector3_x);
        auto axisB = rotate(ornB, vector3_x);

        auto spinvelA = axisA * spinA;
        auto spinvelB = axisB * spinB;

        auto impulse = con.m_max_torque * dt;

        auto &row = cache.rows.emplace_back();
        row.J = {vector3_zero, -axisA, vector3_zero, axisB};
        row.lower_limit = -impulse;
        row.upper_limit = impulse;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA; row.dsA = &dsA;
        row.dvB = &dvB; row.dwB = &dwB; row.dsB = &dsB;
        row.impulse = imp_view.get(entity).values[0];
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.spin_axis[0] = axisA;
        row.spin_axis[1] = axisB;

        prepare_row(row, {}, linvelA, linvelB, angvelA + spinvelA, angvelB + spinvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}