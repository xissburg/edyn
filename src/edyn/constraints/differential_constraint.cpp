#include "edyn/constraints/differential_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<differential_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel, spin,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel, delta_spin>();
    auto con_view = registry.view<differential_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, differential_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, spinA, inv_mA, inv_IA, dvA, dwA, dsA] =
            body_view.get<position, orientation, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, spinB, inv_mB, inv_IB, dvB, dwB, dsB] =
            body_view.get<position, orientation, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[1]);
        auto [posC, ornC, linvelC, angvelC, spinC, inv_mC, inv_IC, dvC, dwC, dsC] =
            body_view.get<position, orientation, linvel, angvel, spin, mass_inv, inertia_world_inv, delta_linvel, delta_angvel, delta_spin>(con.body[2]);

        auto axis0 = rotate(ornA, vector3_x);
        auto axis1 = rotate(ornB, vector3_x);

        auto &row = cache.rows.emplace_back();
        row.J = {vector3_zero, axis0,
                 vector3_zero, axis1,
                 vector3_zero, vector3{scalar(2) / con.ratio, 0, 0}};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.inv_mC = inv_mC; row.inv_IC = inv_IC;
        row.dvA = &dvA; row.dwA = &dwA; row.dsA = &dsA;
        row.dvB = &dvB; row.dwB = &dwB; row.dsB = &dsB;
        row.dvC = &dvC; row.dwC = &dwC; row.dsC = &dsC;
        row.impulse = imp_view.get(entity).values[0];
        row.use_spin[0] = true;
        row.use_spin[1] = true;
        row.use_spin[2] = true;
        row.spin_axis[0] = axis0;
        row.spin_axis[1] = axis1;
        row.spin_axis[2] = vector3_x;
        row.num_entities = 3;

        auto spinvelA = axis0 * spinA;
        auto spinvelB = axis1 * spinB;
        auto spinvelC = vector3_x * spinC;

        prepare_row3(row, {}, linvelA, linvelB, linvelC, angvelA + spinvelA, angvelB + spinvelB, angvelC + spinvelC);
        warm_start3(row);

        cache.con_num_rows.push_back(1);
    });
}

}