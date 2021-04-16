#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/dynamics/solver.hpp"
#include <entt/entt.hpp>

namespace edyn {

struct row_start_index_soft_distance_constraint {
    size_t value;
};

void prepare_soft_distance_constraints(entt::registry &registry, 
                                       row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto con_view = registry.view<soft_distance_constraint>();
    size_t row_idx = cache.con_rows.size();
    registry.ctx_or_set<row_start_index_soft_distance_constraint>(row_idx);

    con_view.each([&] (soft_distance_constraint &con) {
        auto [posA, ornA, linvelA, angvelA] = body_view.get<position, orientation, linvel, angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB] = body_view.get<position, orientation, linvel, angvel>(con.body[1]);

        auto rA = rotate(ornA, con.pivot[0]);
        auto rB = rotate(ornB, con.pivot[1]);

        auto d = posA + rA - posB - rB;
        auto l2 = length_sqr(d);
        auto l = std::sqrt(l2);
        vector3 dn;
        
        if (l2 > EDYN_EPSILON) {
            dn = d / l;
        } else {
            d = dn = vector3_x;
        }

        auto p = cross(rA, dn);
        auto q = cross(rB, dn);

        {
            // Spring row. By setting the error to +/- `large_scalar`, it will
            // always apply the impulse set in the limits.
            auto error = con.distance - l;
            auto spring_force = con.stiffness * error;
            auto spring_impulse = spring_force * dt;

            auto [row, data] = cache.make_row();
            data.J = {dn, p, -dn, -q};
            data.lower_limit = std::min(spring_impulse, scalar(0));
            data.upper_limit = std::max(scalar(0), spring_impulse);
            row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        }

        {
            // Damping row. It functions like friction where the force is
            // proportional to the relative speed.
            auto [row, data] = cache.make_row();
            data.J = {dn, p, -dn, -q};
            row.error = 0;

            auto relspd = dot(data.J[0], linvelA) + 
                          dot(data.J[1], angvelA) +
                          dot(data.J[2], linvelB) +
                          dot(data.J[3], angvelB);
            auto damping_force = con.damping * relspd;
            auto damping_impulse = damping_force * dt;
            auto impulse = std::abs(damping_impulse);

            data.lower_limit = -impulse;
            data.upper_limit =  impulse;

            con.m_relspd = relspd;
        }

        auto [inv_mA, inv_IA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [inv_mB, inv_IB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto num_rows = 2;

        for (size_t i = 0; i < num_rows; ++i) {
            auto j = i + row_idx;
            const auto &row = cache.con_rows[j];
            auto &data = cache.con_datas[j];

            data.inv_mA = inv_mA;
            data.inv_mB = inv_mB;
            data.inv_IA = inv_IA;
            data.inv_IB = inv_IB;

            data.dvA = &dvA;
            data.dvB = &dvB;
            data.dwA = &dwA;
            data.dwB = &dwB;

            data.impulse = con.impulse[i];

            prepare_row(row, data, linvelA, linvelB, angvelA, angvelB);
            warm_start(data);
        }

        row_idx += num_rows;
        cache.con_num_rows.push_back(num_rows);
    });
}

void iterate_soft_distance_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto con_view = registry.view<soft_distance_constraint>();
    auto row_idx = registry.ctx<row_start_index_soft_distance_constraint>().value;

    con_view.each([&] (soft_distance_constraint &con) {
        // Adjust damping row limits to account for velocity changes during iterations.
        auto &damping_data = cache.con_datas[row_idx + 1];
        auto delta_relspd = dot(damping_data.J[0], *damping_data.dvA) + 
                            dot(damping_data.J[1], *damping_data.dwA) +
                            dot(damping_data.J[2], *damping_data.dvB) +
                            dot(damping_data.J[3], *damping_data.dwB);

        auto relspd = con.m_relspd + delta_relspd;

        auto damping_force = con.damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        damping_data.lower_limit = -impulse;
        damping_data.upper_limit =  impulse;

        row_idx += 2;
    });
}

}
