#include "edyn/constraints/soft_distance_constraint.hpp"
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

struct row_start_index_soft_distance_constraint {
    size_t value;
};

template<>
void prepare_constraints<soft_distance_constraint>(entt::registry &registry,
                                                   row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<soft_distance_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    size_t start_idx = cache.rows.size();
    registry.ctx_or_set<row_start_index_soft_distance_constraint>().value = start_idx;

    con_view.each([&] (soft_distance_constraint &con) {
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
        auto dist = std::sqrt(dist_sqr);
        vector3 dn;

        if (dist_sqr > EDYN_EPSILON) {
            dn = d / dist;
        } else {
            d = dn = vector3_x;
        }

        auto p = cross(rA, dn);
        auto q = cross(rB, dn);

        {
            // Spring row. By setting the error to +/- `large_scalar`, it will
            // always apply the impulse set in the limits.
            auto error = con.distance - dist;
            auto spring_force = con.stiffness * error;
            auto spring_impulse = spring_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {dn, p, -dn, -q};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            row.inv_mA = inv_mA; row.inv_mB = inv_mB;
            row.inv_IA = inv_IA; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dvB = &dvB;
            row.dwA = &dwA; row.dwB = &dwB;
            row.impulse = con.impulse[0];

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        {
            // Damping row. It functions like friction where the force is
            // proportional to the relative speed.
            auto &row = cache.rows.emplace_back();
            row.J = {dn, p, -dn, -q};

            auto relspd = dot(row.J[0], linvelA) +
                          dot(row.J[1], angvelA) +
                          dot(row.J[2], linvelB) +
                          dot(row.J[3], angvelB);
            con.relspd = relspd;
            auto damping_force = con.damping * relspd;
            auto damping_impulse = damping_force * dt;
            auto impulse = std::abs(damping_impulse);
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;

            row.inv_mA = inv_mA; row.inv_mB = inv_mB;
            row.inv_IA = inv_IA; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dvB = &dvB;
            row.dwA = &dwA; row.dwB = &dwB;
            row.impulse = con.impulse[1];

            prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);
        }

        size_t num_rows = 2;
        cache.con_num_rows.push_back(num_rows);
    });
}

template<>
void iterate_constraints<soft_distance_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto con_view = registry.view<soft_distance_constraint>(entt::exclude_t<disabled_tag>{});
    auto row_idx = registry.ctx<row_start_index_soft_distance_constraint>().value;

    con_view.each([&] (soft_distance_constraint &con) {
        // Adjust damping row limits to account for velocity changes during iterations.
        auto &damping_row = cache.rows[row_idx + 1];
        auto delta_relspd = dot(damping_row.J[0], *damping_row.dvA) +
                            dot(damping_row.J[1], *damping_row.dwA) +
                            dot(damping_row.J[2], *damping_row.dvB) +
                            dot(damping_row.J[3], *damping_row.dwB);

        auto relspd = con.relspd + delta_relspd;

        auto damping_force = con.damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        damping_row.lower_limit = -impulse;
        damping_row.upper_limit =  impulse;

        row_idx += 2;
    });
}

}
