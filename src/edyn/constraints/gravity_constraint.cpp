#include "edyn/constraints/gravity_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<gravity_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<gravity_constraint>(entt::exclude_t<disabled_tag>{});

    con_view.each([&] (entt::entity entity, gravity_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto d = posA - posB;
        auto l2 = length_sqr(d);
        l2 = std::max(l2, EDYN_EPSILON);

        auto l = std::sqrt(l2);
        auto dn = d / l;

        auto F = gravitational_constant / (l2 * inv_mA * inv_mB);
        auto P = F * dt;

        auto &row = cache.rows.emplace_back();
        row.J = {dn, vector3_zero, -dn, -vector3_zero};
        row.lower_limit = -P;
        row.upper_limit = P;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = con.impulse;

        auto options = constraint_row_options{};
        options.error = large_scalar;

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}
