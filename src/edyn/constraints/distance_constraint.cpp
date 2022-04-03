#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/math/transform.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraints<distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<distance_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (entt::entity entity, distance_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        auto d = pivotA - pivotB;
        auto dist_sqr = length_sqr(d);

        if (!(dist_sqr > EDYN_EPSILON)) {
            d = vector3_x;
        }

        auto &row = cache.rows.emplace_back();
        row.J = {d, cross(rA, d), -d, -cross(rB, d)};
        row.lower_limit = -large_scalar;
        row.upper_limit =  large_scalar;

        auto options = constraint_row_options{};
        options.error = scalar(0.5) * (dist_sqr - con.distance * con.distance) / dt;

        row.inv_mA = inv_mA; row.inv_IA = inv_IA;
        row.inv_mB = inv_mB; row.inv_IB = inv_IB;
        row.dvA = &dvA; row.dwA = &dwA;
        row.dvB = &dvB; row.dwB = &dwB;
        row.impulse = con.impulse;

        prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
        warm_start(row);

        cache.con_num_rows.push_back(1);
    });
}

}
