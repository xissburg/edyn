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
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<differential_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<differential_constraint>();
    auto imp_view = registry.view<constraint_impulse>();

    con_view.each([&] (entt::entity entity, differential_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        auto axis0 = rotate(ornA, -vector3_x);
        auto axis1 = rotate(ornB, -vector3_x);

        auto &row = cache.rows.emplace_back();
        row.J = {vector3_zero, axis0,
                 vector3_zero, axis1,
                 vector3_zero, vector3{scalar(2) / con.ratio, 0, 0}};
        row.lower_limit = -large_scalar;
        row.upper_limit = large_scalar;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = imp_view.get(entity).values[0];

        prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}